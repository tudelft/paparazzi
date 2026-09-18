#include "camera_speedtest.h"
#include "path_utils.h"

/**
 * @file camera_speedtest.c
 * @brief Backend-neutral still-image throughput benchmark and asynchronous progress reporter.
 * @details The benchmark performs one untimed warm-up followed by ten sequential synchronous
 * captures. It validates each returned JPEG before accepting it as evidence. Timing uses
 * @c CLOCK_MONOTONIC around only the backend capture call, excluding setup, UART, EXIF, SODA,
 * and terminal I/O. Progress is copied into a fixed-size queue and written by a separate
 * bounded-I/O thread so an unread pipe or slow terminal cannot throttle capture requests.
 */

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <pthread.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define SPEEDTEST_FILENAME_SIZE 512
#define SPEEDTEST_PROGRESS_CAPACITY ((CAMERA_SPEEDTEST_SAMPLE_COUNT + 1) * 2)

enum progress_kind {
  PROGRESS_STARTED,
  PROGRESS_COMPLETED
};

struct progress_record {
  enum progress_kind kind;
  unsigned sample;
  bool warmup;
  bool succeeded;
  uint64_t duration_ns;
  char filename[SPEEDTEST_FILENAME_SIZE];
};

struct progress_reporter {
  pthread_t thread;
  pthread_mutex_t mutex;
  pthread_cond_t available;
  struct progress_record records[SPEEDTEST_PROGRESS_CAPACITY];
  size_t read_index;
  size_t write_index;
  size_t count;
  bool stopping;
  bool output_failed;
  bool regular_output;
  int output_fd;
  int stdout_flags;
};

static int format_image_path(const struct camera_speedtest_config *config,
                             int image_number, char *path, size_t path_size)
{
  char resolved_directory[PATH_MAX];
  const char *directory = catia_resolve_path(config->photo_directory,
                                             resolved_directory,
                                             sizeof(resolved_directory));
  int length = snprintf(path, path_size, "%s/%c%06d.jpg", directory,
                        config->filename_prefix, image_number);
  return length >= 0 && (size_t)length < path_size ? 0 : -1;
}

/**
 * @brief Read a non-negative monotonic timestamp in nanoseconds.
 * @param time_ns Destination for the timestamp.
 * @return 0 on success; -1 if the clock is unavailable, invalid, or overflows conversion.
 * @details Nanoseconds retain sub-millisecond capture differences without requiring floating
 * point arithmetic in the measurement loop. The conversion rejects impossible values rather
 * than silently wrapping and publishing a plausible but false rate.
 */
static int monotonic_time_ns(uint64_t *time_ns)
{
  struct timespec now;
  if (time_ns == NULL || clock_gettime(CLOCK_MONOTONIC, &now) != 0
      || now.tv_sec < 0 || (uint64_t)now.tv_sec > UINT64_MAX / 1000000000ULL) {
    return -1;
  }
  *time_ns = (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_nsec;
  return 0;
}

int camera_speedtest_create_output_directory(const char *base_directory,
                                             char *output, size_t output_size)
{
  if (base_directory == NULL || output == NULL || output_size == 0) {
    errno = EINVAL;
    return -1;
  }
  char resolved_directory[PATH_MAX];
  const char *base = catia_resolve_path(base_directory, resolved_directory,
                                        sizeof(resolved_directory));
  if (catia_ensure_directory(base) != 0) return -1;

  uint64_t now_ns;
  if (monotonic_time_ns(&now_ns) != 0) return -1;
  for (unsigned attempt = 0; attempt < 100; ++attempt) {
    int length = snprintf(output, output_size, "%s/speedtest-%ld-%" PRIu64 "-%u",
                          base, (long)getpid(), now_ns, attempt);
    if (length < 0 || (size_t)length >= output_size) {
      errno = ENAMETOOLONG;
      return -1;
    }
    if (mkdir(output, 0700) == 0) return 0;
    if (errno != EEXIST) return -1;
  }
  errno = EEXIST;
  return -1;
}

/**
 * @brief Confirm that a backend produced exactly the requested usable image.
 * @param actual_filename Path returned by the backend capture operation.
 * @param expected_filename Deterministic path reserved before capture began.
 * @return 0 when the path matches and names a nonempty regular file; -1 otherwise.
 * @details Path equality prevents a backend fallback or stale output from being counted as a
 * benchmark sample. Regular-file and size checks catch missing, directory, FIFO, and empty
 * outputs while leaving image decoding to the normal backend contract.
 */
static int validate_capture(const char *actual_filename, const char *expected_filename)
{
  if (actual_filename == NULL || strcmp(actual_filename, expected_filename) != 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tbackend returned output path '%s', expected '%s'\n",
            actual_filename == NULL ? "(null)" : actual_filename, expected_filename);
    return -1;
  }
  struct stat image_status;
  if (stat(actual_filename, &image_status) != 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to validate image %s: %s\n",
            actual_filename, strerror(errno));
    return -1;
  }
  if (!S_ISREG(image_status.st_mode) || image_status.st_size <= 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tinvalid or empty image %s\n", actual_filename);
    return -1;
  }
  return 0;
}

/**
 * @brief Write one progress record without waiting indefinitely for stdout.
 * @param reporter Reporter owning the nonblocking output descriptor.
 * @param record Immutable event to render.
 * @return 0 if the complete line was written; -1 when output is unavailable or partial.
 * @details The 100 ms poll bound is a reporting policy, not a capture timeout. On failure the
 * reporter records the condition and the benchmark fails after draining, rather than blocking
 * the capture loop behind a terminal or pipe consumer.
 */
static int write_progress(struct progress_reporter *reporter,
                          const struct progress_record *record)
{
  char line[SPEEDTEST_FILENAME_SIZE + 128];
  int length;
  if (record->kind == PROGRESS_STARTED) {
    length = record->warmup
        ? snprintf(line, sizeof(line), "CATIA SPEEDTEST:\twarm-up capture started: %s\n",
                   record->filename)
        : snprintf(line, sizeof(line), "CATIA SPEEDTEST:\tcapture %u/%u started: %s\n",
                   record->sample, CAMERA_SPEEDTEST_SAMPLE_COUNT, record->filename);
  } else {
    length = record->warmup
        ? snprintf(line, sizeof(line), "CATIA SPEEDTEST:\twarm-up capture %s\n",
                   record->succeeded ? "completed" : "failed")
        : snprintf(line, sizeof(line), "CATIA SPEEDTEST:\tcapture %u/%u %s in %.3f ms\n",
                   record->sample, CAMERA_SPEEDTEST_SAMPLE_COUNT,
                   record->succeeded ? "completed" : "failed",
                   (double)record->duration_ns / 1000000.0);
  }
  if (length < 0 || (size_t)length >= sizeof(line)) return -1;

  struct pollfd output = {.fd = reporter->output_fd, .events = POLLOUT};
  int poll_result;
  do {
    poll_result = poll(&output, 1, 100);
  } while (poll_result < 0 && errno == EINTR);
  if (poll_result <= 0 || (output.revents & POLLOUT) == 0) return -1;
  ssize_t written;
  do {
    written = write(reporter->output_fd, line, (size_t)length);
  } while (written < 0 && errno == EINTR);
  return written == length ? 0 : -1;
}

/**
 * @brief Drain queued progress records on the dedicated reporter thread.
 * @param argument Pointer to the initialized @c progress_reporter.
 * @return Always @c NULL when the stop request is observed.
 * @details Records are removed under the mutex, then formatted and written without holding it.
 * This keeps producer critical sections constant-time and ensures slow output never prevents
 * the benchmark thread from issuing its next capture.
 */
static void *report_progress(void *argument)
{
  struct progress_reporter *reporter = argument;
  for (;;) {
    pthread_mutex_lock(&reporter->mutex);
    while (reporter->count == 0 && !reporter->stopping) {
      pthread_cond_wait(&reporter->available, &reporter->mutex);
    }
    if (reporter->count == 0 && reporter->stopping) {
      pthread_mutex_unlock(&reporter->mutex);
      return NULL;
    }
    struct progress_record record = reporter->records[reporter->read_index];
    reporter->read_index = (reporter->read_index + 1) % SPEEDTEST_PROGRESS_CAPACITY;
    --reporter->count;
    pthread_mutex_unlock(&reporter->mutex);

    if (write_progress(reporter, &record) != 0) {
      pthread_mutex_lock(&reporter->mutex);
      reporter->output_failed = true;
      pthread_mutex_unlock(&reporter->mutex);
    }
  }
}

/**
 * @brief Initialize and launch the bounded progress reporter.
 * @param reporter Storage owned by the caller for the reporter lifetime.
 * @return 0 on success; -1 after cleaning up any partially initialized resources.
 * @details Regular redirected stdout is temporarily opened with append semantics so the
 * reporter cannot overwrite buffered summary output. The duplicate nonblocking descriptor
 * isolates progress backpressure from the benchmark's normal stdout stream.
 */
static int reporter_start(struct progress_reporter *reporter)
{
  reporter->read_index = 0;
  reporter->write_index = 0;
  reporter->count = 0;
  reporter->stopping = false;
  reporter->output_failed = false;
  reporter->regular_output = false;
  if (fflush(stdout) != 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to flush progress output: %s\n",
            strerror(errno));
    return -1;
  }
  struct stat output_status;
  if (fstat(STDOUT_FILENO, &output_status) != 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to inspect progress output: %s\n",
            strerror(errno));
    return -1;
  }
  reporter->regular_output = S_ISREG(output_status.st_mode);
  reporter->stdout_flags = fcntl(STDOUT_FILENO, F_GETFL);
  if (reporter->stdout_flags < 0
      || (reporter->regular_output
          && fcntl(STDOUT_FILENO, F_SETFL, reporter->stdout_flags | O_APPEND) != 0)) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to configure progress output: %s\n",
            strerror(errno));
    return -1;
  }
  reporter->output_fd = open("/proc/self/fd/1",
                             O_WRONLY | O_NONBLOCK | O_APPEND | O_CLOEXEC);
  if (reporter->output_fd < 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to open bounded progress output: %s\n",
            strerror(errno));
    if (reporter->regular_output) {
      fcntl(STDOUT_FILENO, F_SETFL, reporter->stdout_flags);
    }
    return -1;
  }
  int result = pthread_mutex_init(&reporter->mutex, NULL);
  if (result != 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to initialize progress mutex: %s\n",
            strerror(result));
    close(reporter->output_fd);
    if (reporter->regular_output) fcntl(STDOUT_FILENO, F_SETFL, reporter->stdout_flags);
    return -1;
  }
  result = pthread_cond_init(&reporter->available, NULL);
  if (result != 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to initialize progress condition: %s\n",
            strerror(result));
    pthread_mutex_destroy(&reporter->mutex);
    close(reporter->output_fd);
        if (reporter->regular_output) fcntl(STDOUT_FILENO, F_SETFL, reporter->stdout_flags);
    return -1;
  }
  result = pthread_create(&reporter->thread, NULL, report_progress, reporter);
  if (result != 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to start progress reporter: %s\n",
            strerror(result));
    pthread_cond_destroy(&reporter->available);
    pthread_mutex_destroy(&reporter->mutex);
    close(reporter->output_fd);
        if (reporter->regular_output) fcntl(STDOUT_FILENO, F_SETFL, reporter->stdout_flags);
    return -1;
  }
  return 0;
}

/**
 * @brief Queue one capture lifecycle event without performing output I/O.
 * @param reporter Running reporter.
 * @param kind Start or completion event.
 * @param sample One-based measured sample number, or zero for warm-up.
 * @param warmup True for the excluded warm-up capture.
 * @param succeeded Completion success state.
 * @param duration_ns Measured duration for completion records.
 * @param filename Expected output path copied into the fixed-size event.
 * @return 0 if queued; -1 when the bounded queue or filename capacity is exhausted.
 * @details The queue has room for every planned start and completion event. Exhaustion is an
 * invariant breach, so failing explicitly is safer than silently dropping operator progress.
 */
static int reporter_enqueue(struct progress_reporter *reporter, enum progress_kind kind,
                            unsigned sample, bool warmup, bool succeeded,
                            uint64_t duration_ns, const char *filename)
{
  pthread_mutex_lock(&reporter->mutex);
  if (reporter->count >= SPEEDTEST_PROGRESS_CAPACITY) {
    pthread_mutex_unlock(&reporter->mutex);
    fprintf(stderr, "CATIA SPEEDTEST:\tprogress queue exhausted\n");
    return -1;
  }
  struct progress_record *record = &reporter->records[reporter->write_index];
  *record = (struct progress_record) {
    .kind = kind,
    .sample = sample,
    .warmup = warmup,
    .succeeded = succeeded,
    .duration_ns = duration_ns
  };
  int length = snprintf(record->filename, sizeof(record->filename), "%s",
                        filename == NULL ? "" : filename);
  if (length < 0 || (size_t)length >= sizeof(record->filename)) {
    pthread_mutex_unlock(&reporter->mutex);
    return -1;
  }
  reporter->write_index = (reporter->write_index + 1) % SPEEDTEST_PROGRESS_CAPACITY;
  ++reporter->count;
  pthread_cond_signal(&reporter->available);
  pthread_mutex_unlock(&reporter->mutex);
  return 0;
}

/**
 * @brief Request reporter shutdown, drain queued output, and release its resources.
 * @param reporter Running reporter.
 * @return 0 only when the thread joined and all queued progress was written.
 * @details Joining before closing the descriptor preserves event order and prevents a writer
 * from racing descriptor reuse. Restoring redirected stdout flags keeps benchmark setup from
 * leaking output-mode changes into callers that continue running after a failed test.
 */
static int reporter_stop(struct progress_reporter *reporter)
{
  pthread_mutex_lock(&reporter->mutex);
  reporter->stopping = true;
  pthread_cond_signal(&reporter->available);
  pthread_mutex_unlock(&reporter->mutex);
  int join_result = pthread_join(reporter->thread, NULL);
  bool output_failed = reporter->output_failed;
  close(reporter->output_fd);
  if (reporter->regular_output) {
    if (lseek(STDOUT_FILENO, 0, SEEK_END) < 0
        || fcntl(STDOUT_FILENO, F_SETFL, reporter->stdout_flags) != 0) {
      output_failed = true;
    }
  }
  pthread_cond_destroy(&reporter->available);
  pthread_mutex_destroy(&reporter->mutex);
  if (join_result != 0) {
    fprintf(stderr, "CATIA SPEEDTEST:\tunable to stop progress reporter: %s\n",
            strerror(join_result));
    return -1;
  }
  return output_failed ? -1 : 0;
}

int camera_speedtest_find_image_block(const struct camera_speedtest_config *config,
                                      int *first_image)
{
  if (config == NULL || config->photo_directory == NULL || first_image == NULL
      || config->filename_prefix == '\0') {
    errno = EINVAL;
    return -1;
  }
  const int block_size = CAMERA_SPEEDTEST_SAMPLE_COUNT + 1;
  int consecutive = 0;
  for (int image_number = CAMERA_SPEEDTEST_FIRST_IMAGE;
       image_number <= CAMERA_SPEEDTEST_LAST_IMAGE; ++image_number) {
    char path[SPEEDTEST_FILENAME_SIZE];
    if (format_image_path(config, image_number, path, sizeof(path)) != 0) {
      errno = ENAMETOOLONG;
      return -1;
    }
    struct stat status;
    if (lstat(path, &status) == 0) {
      consecutive = 0;
      continue;
    }
    if (errno != ENOENT) {
      return -1;
    }
    if (++consecutive == block_size) {
      *first_image = image_number - block_size + 1;
      return 0;
    }
  }
  errno = ENOSPC;
  return -1;
}

int camera_speedtest_run(const struct camera_speedtest_config *config, int first_image)
{
  if (config == NULL || config->backend_name == NULL || config->shoot == NULL
      || config->keep_running == NULL
      || first_image < CAMERA_SPEEDTEST_FIRST_IMAGE
      || first_image > CAMERA_SPEEDTEST_LAST_IMAGE - CAMERA_SPEEDTEST_SAMPLE_COUNT) {
    errno = EINVAL;
    return -1;
  }

  struct progress_reporter reporter;
  if (reporter_start(&reporter) != 0) return -1;

  uint64_t total_ns = 0;
  uint64_t minimum_ns = UINT64_MAX;
  uint64_t maximum_ns = 0;
  uint64_t sequence_start_ns = 0;
  uint64_t sequence_end_ns = 0;
  int result = 0;

  for (unsigned capture = 0; capture <= CAMERA_SPEEDTEST_SAMPLE_COUNT; ++capture) {
    const bool warmup = capture == 0;
    const unsigned sample = warmup ? 0 : capture;
    const int image_number = first_image + (int)capture;
    char expected_filename[SPEEDTEST_FILENAME_SIZE];
    char actual_filename[SPEEDTEST_FILENAME_SIZE] = "";
    if (!*config->keep_running
        || format_image_path(config, image_number, expected_filename,
                             sizeof(expected_filename)) != 0
        || reporter_enqueue(&reporter, PROGRESS_STARTED, sample, warmup, false, 0,
                            expected_filename) != 0) {
      result = -1;
      break;
    }

    uint64_t started_ns;
    uint64_t completed_ns;
    if (monotonic_time_ns(&started_ns) != 0) {
      fprintf(stderr, "CATIA SPEEDTEST:\tmonotonic clock unavailable: %s\n",
              strerror(errno));
      result = -1;
      break;
    }
    if (!warmup && sequence_start_ns == 0) sequence_start_ns = started_ns;
    int shoot_result = config->shoot(actual_filename, sizeof(actual_filename), image_number);
    if (monotonic_time_ns(&completed_ns) != 0 || completed_ns < started_ns) {
      fprintf(stderr, "CATIA SPEEDTEST:\tinvalid monotonic capture interval\n");
      result = -1;
      break;
    }
    uint64_t duration_ns = completed_ns - started_ns;
    bool succeeded = shoot_result == 0
                     && validate_capture(actual_filename, expected_filename) == 0;
    if (reporter_enqueue(&reporter, PROGRESS_COMPLETED, sample, warmup, succeeded,
                         duration_ns, expected_filename) != 0) {
      result = -1;
      break;
    }
    if (!succeeded) {
      fprintf(stderr, "CATIA SPEEDTEST:\t%s capture failed; aborting benchmark\n",
              warmup ? "warm-up" : "measured");
      result = -1;
      break;
    }
    if (!*config->keep_running) {
      fprintf(stderr, "CATIA SPEEDTEST:\tinterrupted after completed capture; aborting benchmark\n");
      result = -1;
      break;
    }
    if (warmup) continue;

    sequence_end_ns = completed_ns;
    if (UINT64_MAX - total_ns < duration_ns) {
      fprintf(stderr, "CATIA SPEEDTEST:\tcapture duration overflow\n");
      result = -1;
      break;
    }
    total_ns += duration_ns;
    if (duration_ns < minimum_ns) minimum_ns = duration_ns;
    if (duration_ns > maximum_ns) maximum_ns = duration_ns;
  }

  if (reporter_stop(&reporter) != 0) result = -1;
  if (result != 0) return -1;
  if (total_ns == 0 || minimum_ns == 0 || sequence_end_ns <= sequence_start_ns) {
    fprintf(stderr, "CATIA SPEEDTEST:\tinvalid zero-duration benchmark\n");
    return -1;
  }

  const double total_seconds = (double)total_ns / 1000000000.0;
  const double sequence_seconds = (double)(sequence_end_ns - sequence_start_ns) / 1000000000.0;
    const double sustained_seconds_per_photo = sequence_seconds / CAMERA_SPEEDTEST_SAMPLE_COUNT;
    const double capture_seconds_per_photo = total_seconds / CAMERA_SPEEDTEST_SAMPLE_COUNT;
  printf("CATIA SPEEDTEST:\tbackend: %s\n", config->backend_name);
  printf("CATIA SPEEDTEST:\tmeasured captures: %u (one warm-up excluded)\n",
         CAMERA_SPEEDTEST_SAMPLE_COUNT);
    printf("CATIA SPEEDTEST:\tpractical sustained speed: %.3f photos/s "
      "(1 photo every %.3f s, %.1f ms)\n",
      1.0 / sustained_seconds_per_photo, sustained_seconds_per_photo,
      sustained_seconds_per_photo * 1000.0);
    printf("CATIA SPEEDTEST:\taverage camera-call speed: %.3f photos/s "
      "(1 photo every %.3f s, %.1f ms)\n",
      1.0 / capture_seconds_per_photo, capture_seconds_per_photo,
      capture_seconds_per_photo * 1000.0);
    printf("CATIA SPEEDTEST:\taverage / fastest / slowest capture time: "
      "%.3f / %.3f / %.3f ms\n",
      capture_seconds_per_photo * 1000.0,
      (double)minimum_ns / 1000000.0, (double)maximum_ns / 1000000.0);
    printf("CATIA SPEEDTEST:\tfastest single capture: %.3f ms "
      "(equivalent to %.3f photos/s)\n",
      (double)minimum_ns / 1000000.0, 1000000000.0 / (double)minimum_ns);
    printf("CATIA SPEEDTEST:\ttotal camera-call time for %u photos: %.6f s\n",
      CAMERA_SPEEDTEST_SAMPLE_COUNT, total_seconds);
    printf("CATIA SPEEDTEST:\timages retained in %s from %c%06d.jpg through %c%06d.jpg\n",
      config->photo_directory, config->filename_prefix, first_image, config->filename_prefix,
         first_image + CAMERA_SPEEDTEST_SAMPLE_COUNT);
  return 0;
}