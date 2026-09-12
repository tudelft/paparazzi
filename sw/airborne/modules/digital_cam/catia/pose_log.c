#define _GNU_SOURCE
#include "pose_log.h"
#include "protocol.h"
#include "boot_id.h"
#include "path_utils.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "CATIA pose wire decoding requires a little-endian host"
#endif
_Static_assert(sizeof(union catia_pose_sample_union) == CATIA_POSE_SAMPLE_MSG_SIZE, "Pose wire size");

#ifndef POSE_LOG_CAPACITY
#define POSE_LOG_CAPACITY 256
#endif
#ifndef POSE_LOG_MAX_BYTES
#define POSE_LOG_MAX_BYTES (64U * 1024U * 1024U)
#endif
#define POSE_LOG_BATCH 32
_Static_assert(POSE_LOG_CAPACITY >= POSE_LOG_BATCH, "Pose queue must hold one batch");

struct pose_record {
  union catia_pose_sample_union sample;
  uint64_t received_us;
  uint64_t ordinal;
  uint64_t dropped;
  struct pose_clock_evidence clock;
};

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t available;
static pthread_once_t condition_once = PTHREAD_ONCE_INIT;
static int condition_error;
static pthread_t writer;
static FILE *output;
static struct pose_record queue[POSE_LOG_CAPACITY];
static size_t head, count, bytes_written;
static bool active, stopping;
static uint64_t accepted, synced, rejected;
static atomic_uint_fast64_t dropped;
static int log_error;
static char boot_id[37];

static const char header[] =
  "schema,mora_boot_id,receive_monotonic_us,accepted_index,queue_dropped,fc_sequence,fc_sample_begin_us,fc_sample_end_us,"
  "next_shot_nr,lat_e7deg,lon_e7deg,ellipsoid_alt_mm,roll_bfp,pitch_bfp,yaw_bfp,ground_speed_bfp,"
  "course_bfp,agl_bfp,velocity_north_bfp,velocity_east_bfp,velocity_down_bfp,gps_tow_ms,gps_week,"
  "gps_hacc_cm,gps_vacc_cm,gps_sacc_cm_s,gps_fix,gps_num_sv,gps_valid_fields,flags,"
  "clock_token_low,clock_token_high,clock_mapped,sample_earliest_monotonic_us,sample_latest_monotonic_us,"
  "probe_sent_monotonic_us,probe_received_monotonic_us,probe_receive_fc_us,probe_transmit_fc_us,clock_drift_bound_ppm\n";

static void initialize_condition(void)
{
  pthread_condattr_t attributes;
  condition_error = pthread_condattr_init(&attributes);
  if (condition_error != 0) return;
  condition_error = pthread_condattr_setclock(&attributes, CLOCK_MONOTONIC);
  if (condition_error == 0) condition_error = pthread_cond_init(&available, &attributes);
  pthread_condattr_destroy(&attributes);
}

static int write_record(const struct pose_record *record)
{
  const union catia_pose_sample_union *sample = &record->sample;
  const union dc_shot_union *shot = &sample->data.shot;
  char line[1024];
  int length = snprintf(line, sizeof(line),
      "2,%s,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
      ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
      ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
      ",%" PRId32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
      ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32,
      boot_id, record->received_us, record->ordinal, record->dropped,
      sample->data.sequence, sample->data.sample_begin_us, sample->data.sample_end_us,
      shot->data.nr, shot->data.lat, shot->data.lon, shot->data.alt,
      shot->data.phi, shot->data.theta, shot->data.psi, shot->data.vground,
      shot->data.course, shot->data.groundalt, sample->data.velocity_north_bfp,
      sample->data.velocity_east_bfp, sample->data.velocity_down_bfp,
      sample->data.gps_tow_ms, sample->data.gps_week, sample->data.gps_hacc_cm,
      sample->data.gps_vacc_cm, sample->data.gps_sacc_cm_s, sample->data.gps_fix,
      sample->data.gps_num_sv, sample->data.gps_valid_fields, sample->data.flags);
  if (length < 0 || (size_t)length >= sizeof(line)) return EOVERFLOW;
      const struct pose_clock_evidence *clock = &record->clock;
      int extra = snprintf(line + length, sizeof(line) - (size_t)length,
        ",%" PRIu32 ",%" PRIu32 ",%d,%" PRIu64 ",%" PRIu64 ",%" PRIu64
        ",%" PRIu64 ",%" PRIu32 ",%" PRIu32 ",%" PRIu64 "\n",
        clock->token.data.token_low, clock->token.data.token_high, clock->mapped,
        clock->sample_time.earliest_us, clock->sample_time.latest_us,
        clock->probe_sent_us, clock->probe_received_us,
        clock->probe_receive_fc_us, clock->probe_transmit_fc_us, CLOCK_ALIGNMENT_DRIFT_PPM);
      if (extra < 0 || (size_t)extra >= sizeof(line) - (size_t)length) return EOVERFLOW;
      length += extra;
  if (bytes_written > POSE_LOG_MAX_BYTES || (size_t)length > POSE_LOG_MAX_BYTES - bytes_written) return EFBIG;
  if (fwrite(line, 1, (size_t)length, output) != (size_t)length) return errno ? errno : EIO;
  bytes_written += (size_t)length;
  return 0;
}

static void *write_queue(void *unused)
{
  (void)unused;
  for (;;) {
    struct pose_record batch[POSE_LOG_BATCH];
    struct timespec deadline;
    if (clock_gettime(CLOCK_MONOTONIC, &deadline) != 0) {
      int error = errno;
      pthread_mutex_lock(&mutex);
      log_error = error;
      pthread_mutex_unlock(&mutex);
      return NULL;
    }
    ++deadline.tv_sec;
    pthread_mutex_lock(&mutex);
    while (!stopping && count < POSE_LOG_BATCH) {
      int result = pthread_cond_timedwait(&available, &mutex, &deadline);
      if (result == ETIMEDOUT) break;
      if (result != 0) {
        log_error = result;
        pthread_mutex_unlock(&mutex);
        return NULL;
      }
    }
    if (stopping && count == 0) {
      pthread_mutex_unlock(&mutex);
      return NULL;
    }
    size_t batch_count = 0;
    while (count > 0 && batch_count < POSE_LOG_BATCH) {
      batch[batch_count++] = queue[head];
      head = (head + 1) % POSE_LOG_CAPACITY;
      --count;
    }
    pthread_mutex_unlock(&mutex);
    if (batch_count == 0) continue;
    int error = 0;
    for (size_t index = 0; index < batch_count && error == 0; ++index) error = write_record(&batch[index]);
    if (error == 0 && (fflush(output) != 0 || fdatasync(fileno(output)) != 0)) error = errno ? errno : EIO;
    pthread_mutex_lock(&mutex);
    if (error == 0) synced += batch_count;
    else log_error = error;
    pthread_mutex_unlock(&mutex);
    if (error != 0) return NULL;
  }
}

int pose_log_start(const char *directory)
{
  pthread_mutex_lock(&mutex);
  if (active) {
    pthread_mutex_unlock(&mutex);
    errno = EBUSY;
    return -1;
  }
  head = count = bytes_written = 0;
  accepted = synced = rejected = 0;
  atomic_store(&dropped, 0);
  log_error = 0;
  stopping = false;
  if (directory == NULL || directory[0] == '\0') {
    pthread_mutex_unlock(&mutex);
    return 0;
  }
  int initialization = pthread_once(&condition_once, initialize_condition);
  if (initialization != 0 || condition_error != 0) {
    log_error = initialization != 0 ? initialization : condition_error;
    pthread_mutex_unlock(&mutex);
    errno = log_error;
    return -1;
  }
  char path[PATH_MAX];
  char resolved_dir[PATH_MAX];
  const char *target_dir = catia_resolve_path(directory, resolved_dir, sizeof(resolved_dir));
  catia_ensure_directory(target_dir);

  catia_boot_id(boot_id);
  char stamp[32];
  struct tm utc;
  time_t now = time(NULL);
  if (gmtime_r(&now, &utc) == NULL || strftime(stamp, sizeof(stamp), "%Y%m%dT%H%M%SZ", &utc) == 0) {
    pthread_mutex_unlock(&mutex);
    errno = EINVAL;
    return -1;
  }
  int length = snprintf(path, sizeof(path), "%s/pose-%s-XXXXXX.csv", target_dir, stamp);
  if (length < 0 || (size_t)length >= sizeof(path)) {
    pthread_mutex_unlock(&mutex);
    errno = ENAMETOOLONG;
    return -1;
  }
  int directory_fd = open(target_dir, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (directory_fd < 0) {
    log_error = errno;
    pthread_mutex_unlock(&mutex);
    return -1;
  }
  int descriptor = mkostemps(path, 4, O_CLOEXEC);
  int error = descriptor < 0 ? errno : 0;
  if (error == 0) {
    output = fdopen(descriptor, "w");
    if (output == NULL) {
      error = errno;
      close(descriptor);
    }
  }
  if (error == 0) {
    bytes_written = sizeof(header) - 1;
    if (bytes_written > POSE_LOG_MAX_BYTES) error = EFBIG;
    else if (fwrite(header, 1, bytes_written, output) != bytes_written || fflush(output) != 0
             || fsync(fileno(output)) != 0 || fsync(directory_fd) != 0) error = errno ? errno : EIO;
  }
  close(directory_fd);
  if (error == 0) error = pthread_create(&writer, NULL, write_queue, NULL);
  if (error != 0) {
    if (output != NULL) fclose(output);
    output = NULL;
    if (descriptor >= 0) unlink(path);
    log_error = error;
    pthread_mutex_unlock(&mutex);
    errno = error;
    return -1;
  }
  active = true;
  pthread_mutex_unlock(&mutex);
  fprintf(stderr, "CATIA POSE: logging to %s\n", path);
  return 0;
}

int pose_log_record(const uint8_t *payload, size_t length, uint64_t receive_monotonic_us)
{
  return pose_log_record_clocked(payload, length, receive_monotonic_us, NULL);
}

int pose_log_record_clocked(const uint8_t *payload, size_t length, uint64_t receive_monotonic_us,
                            const struct pose_clock_evidence *evidence)
{
  if (pthread_mutex_trylock(&mutex) != 0) {
    atomic_fetch_add(&dropped, 1);
    return 0;
  }
  if (!active || stopping) {
    pthread_mutex_unlock(&mutex);
    return 0;
  }
  union catia_pose_sample_union sample = {0};
  bool valid = payload != NULL && length == sizeof(sample.bin) && receive_monotonic_us != 0;
  if (valid) {
    for (size_t index = 0; index < sizeof(sample.bin); ++index) sample.bin[index] = payload[index];
    valid = (sample.data.flags & ~CATIA_POSE_SAMPLE_GPS_PRESENT) == 0
            && (uint32_t)(sample.data.sample_end_us - sample.data.sample_begin_us) <= 1000000
            && sample.data.shot.data.lat >= -900000000 && sample.data.shot.data.lat <= 900000000
            && sample.data.shot.data.lon >= -1800000000 && sample.data.shot.data.lon <= 1800000000;
  }
  if (valid && evidence != NULL && evidence->mapped) {
    valid = (evidence->token.data.token_low != 0 || evidence->token.data.token_high != 0)
            && evidence->sample_time.earliest_us > 0
            && evidence->sample_time.earliest_us <= evidence->sample_time.latest_us
            && evidence->sample_time.latest_us <= receive_monotonic_us
            && evidence->probe_sent_us > 0 && evidence->probe_sent_us <= evidence->probe_received_us
            && evidence->probe_received_us <= receive_monotonic_us;
  }
  if (!valid) {
    ++rejected;
    pthread_mutex_unlock(&mutex);
    return -1;
  }
  if (log_error != 0 || count == POSE_LOG_CAPACITY) {
    atomic_fetch_add(&dropped, 1);
    pthread_mutex_unlock(&mutex);
    return 0;
  }
  struct pose_record *record = &queue[(head + count) % POSE_LOG_CAPACITY];
  record->sample = sample;
  record->received_us = receive_monotonic_us;
  record->ordinal = ++accepted;
  record->dropped = atomic_load(&dropped);
  record->clock = evidence != NULL ? *evidence : (struct pose_clock_evidence){0};
  if (!record->clock.mapped) {
    const union catia_clock_request_union token = record->clock.token;
    record->clock = (struct pose_clock_evidence){.token = token};
  }
  ++count;
  pthread_cond_signal(&available);
  pthread_mutex_unlock(&mutex);
  return 1;
}

struct pose_log_stats pose_log_status(void)
{
  pthread_mutex_lock(&mutex);
  struct pose_log_stats status = {accepted, synced, atomic_load(&dropped), rejected, log_error};
  pthread_mutex_unlock(&mutex);
  return status;
}

int pose_log_stop(void)
{
  pthread_mutex_lock(&mutex);
  if (!active) {
    pthread_mutex_unlock(&mutex);
    return 0;
  }
  stopping = true;
  pthread_cond_signal(&available);
  pthread_mutex_unlock(&mutex);
  int error = pthread_join(writer, NULL);
  if (error != 0) {
    errno = error;
    return -1;
  }
  if (fclose(output) != 0) error = errno ? errno : EIO;
  pthread_mutex_lock(&mutex);
  output = NULL;
  active = false;
  if (log_error == 0) log_error = error;
  error = log_error;
  pthread_mutex_unlock(&mutex);
  if (error != 0) errno = error;
  return error == 0 ? 0 : -1;
}