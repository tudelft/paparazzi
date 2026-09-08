// C11 + POSIX.1-2008 only (posix_spawn, clock_gettime, nanosleep, pthread).
#define _POSIX_C_SOURCE 200809L

#include "ear_cam_pipe.h"
#include "ear_heatmap.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef CATIA_EAR_CAM_COMMAND
#define CATIA_EAR_CAM_COMMAND "earcam/earcam"
#endif

#ifndef CATIA_EAR_CAM_DEVICE
#define CATIA_EAR_CAM_DEVICE "auto"   // earcam picks the first USB capture card
#endif

#ifndef CATIA_EAR_CAM_LOG_DIR
#define CATIA_EAR_CAM_LOG_DIR "earlogs"
#endif

#ifndef CATIA_EAR_CAM_PHOTO_DIR
#define CATIA_EAR_CAM_PHOTO_DIR "photos"
#endif

#define EAR_MAX_SAMPLES 16384U
#define EAR_READY_TIMEOUT_MS 15000
#define EAR_RESTART_INTERVAL_MS 3000U
#define EAR_WARNING_INTERVAL_MS 1000U
#define EAR_SAMPLE_STALE_MS 500U
#define EAR_MIN_SAMPLES 8U
#define EAR_LINE_MAX 512U
#define ANGLE_BFP_SCALE 4096.0
#define POSITION_BFP_SCALE 256.0
#define EARTH_RADIUS_M 6378137.0
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232

extern char **environ;

struct latest_measurement {
  uint64_t timestamp_ms;
  double level_db;
  double trend_db;
  double contrast_db;
  double frequency_hz;
  bool alarm;
  bool clipped;
  bool valid;
};

static pid_t server_pid = -1;
static int server_input = -1;
static int server_output = -1;
static pthread_t reader_thread;
static bool reader_running;
static volatile sig_atomic_t reader_stop;
static pthread_mutex_t ear_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct latest_measurement latest;
static bool server_ready;
static uint64_t server_error_count;
static uint64_t server_started_ms;
static uint64_t last_warning_ms;

static struct ear_sample *session;
static size_t session_count;
static FILE *session_log;
static char session_log_path[512];

static bool simulated;
static double simulated_lat_deg;
static double simulated_lon_deg;
static double simulated_level_db_at_1m;
static uint32_t simulated_noise_state = 0x2545F491U;

void ear_cam_pipe_set_simulated_source(double lat_deg, double lon_deg, double level_db_at_1m)
{
  simulated = true;
  simulated_lat_deg = lat_deg;
  simulated_lon_deg = lon_deg;
  simulated_level_db_at_1m = level_db_at_1m;
}

static char band_low_arg[16];
static char band_high_arg[16];
static bool band_set;

void ear_cam_pipe_set_band(double low_cut_hz, double high_cut_hz)
{
  band_set = low_cut_hz > 0.0 && high_cut_hz > low_cut_hz;
  if (band_set) {
    snprintf(band_low_arg, sizeof(band_low_arg), "%.0f", low_cut_hz);
    snprintf(band_high_arg, sizeof(band_high_arg), "%.0f", high_cut_hz);
  }
}

// Deterministic +/-1.5 dB microphone noise (xorshift), independent of libc rand().
static double simulated_noise_db(void)
{
  uint32_t x = simulated_noise_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  simulated_noise_state = x;
  return ((double)(x % 30001U) / 10000.0) - 1.5;
}

static struct latest_measurement simulated_measurement(const union dc_shot_union *shot, uint64_t now)
{
  double lat = shot->data.lat / 1e7;
  double lon = shot->data.lon / 1e7;
  double agl = shot->data.groundalt / POSITION_BFP_SCALE;
  double north = (lat - simulated_lat_deg) * DEG_TO_RAD * EARTH_RADIUS_M;
  double east = (lon - simulated_lon_deg) * DEG_TO_RAD * EARTH_RADIUS_M * cos(simulated_lat_deg * DEG_TO_RAD);
  double distance = sqrt(north * north + east * east + agl * agl);
  if (distance < 1.0) {
    distance = 1.0;
  }
  struct latest_measurement measurement = {0};
  measurement.timestamp_ms = now;
  measurement.level_db = simulated_level_db_at_1m - 20.0 * log10(distance) + simulated_noise_db();
  measurement.contrast_db = 6.0;
  measurement.trend_db = 0.0;
  measurement.frequency_hz = 2700.0;
  measurement.alarm = true;
  measurement.clipped = false;
  measurement.valid = true;
  return measurement;
}

static uint64_t monotonic_ms(void)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U;
}

static void sleep_ms(unsigned int milliseconds)
{
  struct timespec interval = {
    .tv_sec = milliseconds / 1000U,
    .tv_nsec = (long)(milliseconds % 1000U) * 1000000L
  };
  while (nanosleep(&interval, &interval) != 0 && errno == EINTR) {
    continue;
  }
}

static int parse_field(const char *line, const char *key, double *value)
{
  const char *position = line;
  size_t key_length = strlen(key);
  while ((position = strstr(position, key)) != NULL) {
    if ((position == line || position[-1] == ' ') && position[key_length] == '=') {
      char *end = NULL;
      double parsed = strtod(position + key_length + 1, &end);
      if (end == position + key_length + 1 || !isfinite(parsed)) {
        return -1;
      }
      *value = parsed;
      return 0;
    }
    position += key_length;
  }
  return -1;
}

static void handle_server_line(const char *line)
{
  if (strncmp(line, "SAMPLE ", 7) == 0) {
    struct latest_measurement measurement = {0};
    double timestamp = 0.0;
    double alarm = 0.0;
    double clipped = 0.0;
    if (parse_field(line, "t", &timestamp) != 0
        || parse_field(line, "level", &measurement.level_db) != 0
        || parse_field(line, "freq", &measurement.frequency_hz) != 0
        || parse_field(line, "contrast", &measurement.contrast_db) != 0
        || parse_field(line, "trend", &measurement.trend_db) != 0
        || parse_field(line, "alarm", &alarm) != 0
        || parse_field(line, "clip", &clipped) != 0) {
      return;
    }
    measurement.timestamp_ms = (uint64_t)timestamp;
    measurement.alarm = alarm != 0.0;
    measurement.clipped = clipped != 0.0;
    measurement.valid = true;
    pthread_mutex_lock(&ear_mutex);
    latest = measurement;
    pthread_mutex_unlock(&ear_mutex);
    return;
  }
  if (strncmp(line, "EARCAM_READY", 12) == 0) {
    pthread_mutex_lock(&ear_mutex);
    server_ready = true;
    pthread_mutex_unlock(&ear_mutex);
    printf("EAR_CAM_PIPE:\t%s\n", line);
    return;
  }
  if (strncmp(line, "EARCAM_ERROR", 12) == 0) {
    pthread_mutex_lock(&ear_mutex);
    server_error_count++;
    pthread_mutex_unlock(&ear_mutex);
    fprintf(stderr, "EAR_CAM_PIPE:\t%s\n", line);
    return;
  }
  if (strncmp(line, "EARCAM_STOPPED", 14) == 0) {
    printf("EAR_CAM_PIPE:\t%s\n", line);
  }
}

static void *reader_main(void *unused)
{
  (void)unused;
  char line[EAR_LINE_MAX];
  size_t line_size = 0;

  while (!reader_stop && server_output >= 0) {
    struct pollfd descriptor = {.fd = server_output, .events = POLLIN, .revents = 0};
    int poll_result = poll(&descriptor, 1, 200);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    if (poll_result == 0) {
      continue;
    }
    char chunk[256];
    ssize_t count = read(server_output, chunk, sizeof(chunk));
    if (count == 0) {
      break;
    }
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      break;
    }
    for (ssize_t index = 0; index < count; index++) {
      if (chunk[index] == '\n') {
        line[line_size] = '\0';
        handle_server_line(line);
        line_size = 0;
      } else if (line_size + 1 < sizeof(line)) {
        line[line_size++] = chunk[index];
      } else {
        line_size = 0;
      }
    }
  }
  pthread_mutex_lock(&ear_mutex);
  server_ready = false;
  latest.valid = false;
  pthread_mutex_unlock(&ear_mutex);
  return NULL;
}

static void stop_server(void)
{
  reader_stop = 1;
  if (server_input >= 0) {
    static const char quit[] = "QUIT\n";
    ssize_t written = write(server_input, quit, sizeof(quit) - 1);
    (void)written;
    close(server_input);
    server_input = -1;
  }
  if (reader_running) {
    pthread_join(reader_thread, NULL);
    reader_running = false;
  }
  if (server_output >= 0) {
    close(server_output);
    server_output = -1;
  }
  if (server_pid > 0) {
    int status;
    pid_t wait_result;
    for (int attempt = 0; attempt < 100; attempt++) {
      wait_result = waitpid(server_pid, &status, WNOHANG);
      if (wait_result != 0) {
        break;
      }
      sleep_ms(20);
    }
    if (wait_result == 0) {
      kill(server_pid, SIGTERM);
      do {
        wait_result = waitpid(server_pid, &status, 0);
      } while (wait_result < 0 && errno == EINTR);
    }
    server_pid = -1;
  }
  server_ready = false;
}

static int open_session_log(void)
{
  if (mkdir(CATIA_EAR_CAM_LOG_DIR, 0755) != 0 && errno != EEXIST) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to create %s: %s\n",
            CATIA_EAR_CAM_LOG_DIR, strerror(errno));
    return -1;
  }
  time_t now = time(NULL);
  struct tm stamp;
  localtime_r(&now, &stamp);
  int length = snprintf(session_log_path, sizeof(session_log_path),
                        "%s/ear_%04d%02d%02d_%02d%02d%02d.csv", CATIA_EAR_CAM_LOG_DIR,
                        stamp.tm_year + 1900, stamp.tm_mon + 1, stamp.tm_mday,
                        stamp.tm_hour, stamp.tm_min, stamp.tm_sec);
  if (length < 0 || (size_t)length >= sizeof(session_log_path)) {
    return -1;
  }
  session_log = fopen(session_log_path, "w");
  if (session_log == NULL) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to open %s: %s\n", session_log_path, strerror(errno));
    return -1;
  }
  fputs("t_ms,shot,lat_deg,lon_deg,agl_m,alt_m,level_db,trend_db,contrast_db,freq_hz,alarm,clip\n",
        session_log);
  fflush(session_log);
  return 0;
}

static void close_session_log(void)
{
  if (session_log != NULL) {
    fclose(session_log);
    session_log = NULL;
  }
}

// Spawn `earcam --server` with a reader thread; returns without waiting for EARCAM_READY.
static int start_server(void)
{
  int command_pipe[2];
  int response_pipe[2];
  if (pipe(command_pipe) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to create pipes: %s\n", strerror(errno));
    return -1;
  }
  if (pipe(response_pipe) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to create pipes: %s\n", strerror(errno));
    close(command_pipe[0]);
    close(command_pipe[1]);
    return -1;
  }

  posix_spawn_file_actions_t actions;
  if (posix_spawn_file_actions_init(&actions) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to configure earcam server\n");
    close(command_pipe[0]);
    close(command_pipe[1]);
    close(response_pipe[0]);
    close(response_pipe[1]);
    return -1;
  }
  if (posix_spawn_file_actions_adddup2(&actions, command_pipe[0], STDIN_FILENO) != 0
      || posix_spawn_file_actions_adddup2(&actions, response_pipe[1], STDOUT_FILENO) != 0
      || posix_spawn_file_actions_addclose(&actions, command_pipe[1]) != 0
      || posix_spawn_file_actions_addclose(&actions, response_pipe[0]) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to configure earcam server\n");
    posix_spawn_file_actions_destroy(&actions);
    close(command_pipe[0]);
    close(command_pipe[1]);
    close(response_pipe[0]);
    close(response_pipe[1]);
    return -1;
  }

  char *const arguments[] = {
    (char *)CATIA_EAR_CAM_COMMAND,
    (char *)"--server",
    (char *)"--device", (char *)CATIA_EAR_CAM_DEVICE,
    band_set ? (char *)"--low-cut" : NULL, band_low_arg,
    (char *)"--high-cut", band_high_arg,
    NULL
  };
  int spawn_result = posix_spawn(&server_pid, CATIA_EAR_CAM_COMMAND, &actions, NULL,
                                 arguments, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(command_pipe[0]);
  close(response_pipe[1]);
  if (spawn_result != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to start %s: %s\n",
            CATIA_EAR_CAM_COMMAND, strerror(spawn_result));
    close(command_pipe[1]);
    close(response_pipe[0]);
    server_pid = -1;
    return -1;
  }
  server_input = command_pipe[1];
  server_output = response_pipe[0];
  reader_stop = 0;
  pthread_mutex_lock(&ear_mutex);
  server_ready = false;
  server_error_count = 0;
  latest.valid = false;
  pthread_mutex_unlock(&ear_mutex);
  server_started_ms = monotonic_ms();
  if (pthread_create(&reader_thread, NULL, reader_main, NULL) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to start reader thread\n");
    stop_server();
    return -1;
  }
  reader_running = true;
  return 0;
}

// The earcam process exits when the microphone disappears; bring it back when it does.
static void restart_server_if_dead(uint64_t now)
{
  if (simulated || ear_cam_pipe_ready()) {
    return;
  }
  if (now - server_started_ms < EAR_RESTART_INTERVAL_MS) {
    return;
  }
  fprintf(stderr, "EAR_CAM_PIPE:\tearcam server not running, restarting\n");
  stop_server();
  if (start_server() != 0) {
    server_started_ms = now;   // retry after the interval
  }
}

static bool warning_due(uint64_t now)
{
  if (now - last_warning_ms < EAR_WARNING_INTERVAL_MS) {
    return false;
  }
  last_warning_ms = now;
  return true;
}

int ear_cam_pipe_init(const char *unused)
{
  (void)unused;

  if (session == NULL) {
    session = calloc(EAR_MAX_SAMPLES, sizeof(*session));
    if (session == NULL) {
      fprintf(stderr, "EAR_CAM_PIPE:\tfailed to allocate sample buffer\n");
      return -1;
    }
  }
  session_count = 0;

  if (simulated) {
    pthread_mutex_lock(&ear_mutex);
    server_ready = true;
    latest.valid = false;
    pthread_mutex_unlock(&ear_mutex);
    printf("EAR_CAM_PIPE:\tSIMULATED loudspeaker at %.7f %.7f, %.1f dB at 1 m (no microphone)\n",
           simulated_lat_deg, simulated_lon_deg, simulated_level_db_at_1m);
    return 0;
  }

  if (access(CATIA_EAR_CAM_COMMAND, X_OK) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tcapture command is not executable: %s: %s\n",
            CATIA_EAR_CAM_COMMAND, strerror(errno));
    return -1;
  }
  if (start_server() != 0) {
    return -1;
  }

  uint64_t deadline = monotonic_ms() + EAR_READY_TIMEOUT_MS;
  while (monotonic_ms() < deadline) {
    pthread_mutex_lock(&ear_mutex);
    bool ready = server_ready;
    uint64_t errors = server_error_count;
    pthread_mutex_unlock(&ear_mutex);
    if (ready) {
      break;
    }
    if (errors > 0) {
      break;
    }
    sleep_ms(50);
  }
  if (!ear_cam_pipe_ready()) {
    // Not fatal: the microphone may be plugged in later; ear_cam_pipe_record()
    // restarts the server periodically and the optical cameras keep working.
    fprintf(stderr, "EAR_CAM_PIPE:\tearcam server did not become ready; will keep retrying\n");
    stop_server();
    server_started_ms = monotonic_ms();
    return 0;
  }
  printf("EAR_CAM_PIPE:\tcapture command: %s\n", CATIA_EAR_CAM_COMMAND);
  printf("EAR_CAM_PIPE:\tmicrophone: %s\n", CATIA_EAR_CAM_DEVICE);
  printf("EAR_CAM_PIPE:\tpersistent earcam server ready\n");
  return 0;
}

void ear_cam_pipe_deinit(void)
{
  if (!simulated) {
    stop_server();
  }
  server_ready = false;
  close_session_log();
  free(session);
  session = NULL;
  session_count = 0;
}

bool ear_cam_pipe_ready(void)
{
  pthread_mutex_lock(&ear_mutex);
  bool ready = server_ready;
  pthread_mutex_unlock(&ear_mutex);
  return ready;
}

int ear_cam_pipe_record(const union dc_shot_union *shot)
{
  if (shot == NULL || session == NULL) {
    return -1;
  }
  struct latest_measurement measurement;
  uint64_t now = monotonic_ms();
  restart_server_if_dead(now);
  pthread_mutex_lock(&ear_mutex);
  measurement = simulated ? simulated_measurement(shot, now) : latest;
  bool ready = server_ready;
  pthread_mutex_unlock(&ear_mutex);
  if (!ready || !measurement.valid) {
    if (warning_due(now)) {
      fprintf(stderr, "EAR_CAM_PIPE-%d:\tno microphone measurement available\n", shot->data.nr);
    }
    return -1;
  }
  if (now > measurement.timestamp_ms && now - measurement.timestamp_ms > EAR_SAMPLE_STALE_MS) {
    if (warning_due(now)) {
      fprintf(stderr, "EAR_CAM_PIPE-%d:\tmicrophone measurement is stale (%llu ms)\n",
              shot->data.nr, (unsigned long long)(now - measurement.timestamp_ms));
    }
    return -1;
  }
  if (session_count == 0 && session_log == NULL) {
    open_session_log();
  }
  if (session_count >= EAR_MAX_SAMPLES) {
    // Keep the newest data: drop the oldest half to stay bounded on a small board.
    size_t keep = EAR_MAX_SAMPLES / 2;
    for (size_t index = 0; index < keep; index++) {
      session[index] = session[index + keep];
    }
    session_count = keep;
  }
  struct ear_sample *sample = &session[session_count++];
  sample->timestamp_ms = measurement.timestamp_ms;
  sample->shot_nr = shot->data.nr;
  sample->lat_deg = shot->data.lat / 1e7;
  sample->lon_deg = shot->data.lon / 1e7;
  sample->agl_m = shot->data.groundalt / POSITION_BFP_SCALE;
  sample->alt_m = shot->data.alt / 1000.0;
  sample->level_db = measurement.level_db;
  sample->trend_db = measurement.trend_db;
  sample->contrast_db = measurement.contrast_db;
  sample->frequency_hz = measurement.frequency_hz;
  sample->alarm = measurement.alarm;
  sample->clipped = measurement.clipped;
  if (session_log != NULL) {
    fprintf(session_log, "%llu,%d,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%d,%d\n",
            (unsigned long long)sample->timestamp_ms, sample->shot_nr, sample->lat_deg,
            sample->lon_deg, sample->agl_m, sample->alt_m, sample->level_db,
            sample->trend_db, sample->contrast_db, sample->frequency_hz,
            sample->alarm ? 1 : 0, sample->clipped ? 1 : 0);
    fflush(session_log);
  }
  return 0;
}

int32_t ear_cam_pipe_last_shot_nr(void)
{
  return session_count > 0 ? session[session_count - 1].shot_nr : 0;
}

int ear_cam_pipe_render(const struct ear_loudest_spot *result, char *filename, size_t filename_size)
{
  if (filename == NULL || filename_size == 0) {
    return -1;
  }
  filename[0] = '\0';
  if (session == NULL || session_count == 0) {
    return -1;
  }
  if (mkdir(CATIA_EAR_CAM_PHOTO_DIR, 0755) != 0 && errno != EEXIST) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to create %s: %s\n", CATIA_EAR_CAM_PHOTO_DIR, strerror(errno));
    return -1;
  }
  int length = snprintf(filename, filename_size, "%s/e%06d.jpg", CATIA_EAR_CAM_PHOTO_DIR,
                        ear_cam_pipe_last_shot_nr());
  if (length < 0 || (size_t)length >= filename_size) {
    filename[0] = '\0';
    return -1;
  }
  if (ear_heatmap_write(filename, session, session_count, result) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to render %s\n", filename);
    unlink(filename);
    filename[0] = '\0';
    return -1;
  }
  // Undecorated field alongside, for ear_heatmap_overlay.py on the ground.
  char field[512];
  length = snprintf(field, sizeof(field), "%s/e%06d_field.jpg", CATIA_EAR_CAM_PHOTO_DIR,
                    ear_cam_pipe_last_shot_nr());
  if (length > 0 && (size_t)length < sizeof(field)) {
    ear_heatmap_write_field(field, session, session_count, result);
  }
  return 0;
}

int ear_cam_pipe_solve(struct ear_loudest_spot *result)
{
  if (result == NULL) {
    return -1;
  }
  memset(result, 0, sizeof(*result));
  int status = calculated_loudestspot(session, session_count, result);
  if (session_log != NULL) {
    fprintf(session_log, "# interim valid=%d lat=%.7f lon=%.7f agl=%.2f conf=%.3f used=%u/%u\n",
            result->valid ? 1 : 0, result->lat_deg, result->lon_deg, result->agl_m,
            result->confidence, result->used_count, result->sample_count);
    fflush(session_log);
  }
  return status;
}

int ear_cam_pipe_finish(struct ear_loudest_spot *result)
{
  if (result == NULL) {
    return -1;
  }
  memset(result, 0, sizeof(*result));
  int status = calculated_loudestspot(session, session_count, result);
  if (session_log != NULL) {
    fprintf(session_log, "# result valid=%d lat=%.7f lon=%.7f agl=%.2f alt=%.2f level=%.2f conf=%.3f used=%u/%u\n",
            result->valid ? 1 : 0, result->lat_deg, result->lon_deg, result->agl_m,
            result->alt_m, result->level_db, result->confidence, result->used_count,
            result->sample_count);
    close_session_log();
    printf("EAR_CAM_PIPE:\tsession saved to %s\n", session_log_path);
  }
  session_count = 0;
  return status;
}

/* ---------------------------------------------------------------------- */
/* Loudest-spot fusion                                                    */
/* ---------------------------------------------------------------------- */

static int compare_double(const void *a, const void *b)
{
  double left = *(const double *)a;
  double right = *(const double *)b;
  return (left > right) - (left < right);
}

static double median_of(double *values, size_t count)
{
  qsort(values, count, sizeof(*values), compare_double);
  if (count % 2 == 1) {
    return values[count / 2];
  }
  return 0.5 * (values[count / 2 - 1] + values[count / 2]);
}

struct enu_sample {
  double east;
  double north;
  double height2;   // AGL squared; per-sample so mixed-altitude sessions fit correctly
  double level;
  double weight;
  const struct ear_sample *source;
};

int calculated_loudestspot(const struct ear_sample *samples, size_t count,
                           struct ear_loudest_spot *result)
{
  memset(result, 0, sizeof(*result));
  result->sample_count = (uint32_t)count;
  if (samples == NULL || count < EAR_MIN_SAMPLES) {
    return -1;
  }

  // Gate 1: geometry and clipping. Prefer windows that passed the tone detector.
  size_t alarm_count = 0;
  for (size_t index = 0; index < count; index++) {
    const struct ear_sample *sample = &samples[index];
    if (sample->alarm && !sample->clipped && isfinite(sample->lat_deg)
        && isfinite(sample->lon_deg) && fabs(sample->lat_deg) > 1e-6) {
      alarm_count++;
    }
  }
  bool require_alarm = alarm_count >= EAR_MIN_SAMPLES;

  struct enu_sample *points = calloc(count, sizeof(*points));
  double *levels = calloc(count, sizeof(*levels));
  if (points == NULL || levels == NULL) {
    free(points);
    free(levels);
    return -1;
  }

  size_t used = 0;
  for (size_t index = 0; index < count; index++) {
    const struct ear_sample *sample = &samples[index];
    if (sample->clipped || !isfinite(sample->lat_deg) || !isfinite(sample->lon_deg)
        || !isfinite(sample->level_db) || fabs(sample->lat_deg) <= 1e-6
        || (require_alarm && !sample->alarm)) {
      continue;
    }
    points[used].source = sample;
    points[used].level = sample->level_db;
    levels[used] = sample->level_db;
    used++;
  }
  if (used < EAR_MIN_SAMPLES) {
    free(points);
    free(levels);
    return -1;
  }

  // Gate 2: Hampel-style outlier rejection on level (MAD-scaled).
  double median_level = median_of(levels, used);
  for (size_t index = 0; index < used; index++) {
    levels[index] = fabs(points[index].level - median_level);
  }
  double mad = median_of(levels, used) * 1.4826;
  if (mad < 0.5) {
    mad = 0.5;
  }
  size_t kept = 0;
  for (size_t index = 0; index < used; index++) {
    // Only suppress implausibly *low* spikes; a loud outlier may be the target.
    if (points[index].level < median_level - 4.0 * mad) {
      continue;
    }
    points[kept++] = points[index];
  }
  used = kept;
  if (used < EAR_MIN_SAMPLES) {
    free(points);
    free(levels);
    return -1;
  }

  // Local ENU frame about the sample of maximal level.
  size_t peak_index = 0;
  for (size_t index = 1; index < used; index++) {
    if (points[index].level > points[peak_index].level) {
      peak_index = index;
    }
  }
  double origin_lat = points[peak_index].source->lat_deg;
  double origin_lon = points[peak_index].source->lon_deg;
  double cos_lat = cos(origin_lat * DEG_TO_RAD);
  double max_level = points[peak_index].level;
  for (size_t index = 0; index < used; index++) {
    points[index].east = (points[index].source->lon_deg - origin_lon) * DEG_TO_RAD
                         * EARTH_RADIUS_M * cos_lat;
    points[index].north = (points[index].source->lat_deg - origin_lat) * DEG_TO_RAD
                          * EARTH_RADIUS_M;
  }

  // Step 1: power-weighted centroid of the loudest windows (top 25 %, min 4).
  for (size_t index = 0; index < used; index++) {
    levels[index] = points[index].level;
  }
  qsort(levels, used, sizeof(*levels), compare_double);
  size_t top_count = used / 4;
  if (top_count < 4) {
    top_count = used < 4 ? used : 4;
  }
  double threshold = levels[used - top_count];
  double sum_weight = 0.0;
  double east0 = 0.0;
  double north0 = 0.0;
  for (size_t index = 0; index < used; index++) {
    if (points[index].level < threshold) {
      points[index].weight = 0.0;
      continue;
    }
    // Acoustic power relative to the peak; trend rising adds a small bonus.
    double weight = pow(10.0, (points[index].level - max_level) / 10.0);
    if (points[index].source->trend_db > 0.0) {
      weight *= 1.0 + fmin(points[index].source->trend_db, 6.0) / 12.0;
    }
    points[index].weight = weight;
    sum_weight += weight;
    east0 += weight * points[index].east;
    north0 += weight * points[index].north;
  }
  if (sum_weight <= 0.0) {
    free(points);
    free(levels);
    return -1;
  }
  east0 /= sum_weight;
  north0 /= sum_weight;

  // Step 2: Gauss-Newton refinement of an inverse-distance source model
  //         L_i = L0 - 20 log10(sqrt(d_i^2 + h_i^2)), h_i = sample AGL (source on ground).
  // Fallback height for samples without AGL: median AGL of the loudest (weighted) windows.
  double agl_values[64];
  size_t agl_count = 0;
  for (size_t index = 0; index < used && agl_count < 64; index++) {
    if (points[index].weight > 0.0 && isfinite(points[index].source->agl_m)
        && points[index].source->agl_m > 0.5) {
      agl_values[agl_count++] = points[index].source->agl_m;
    }
  }
  double height = agl_count > 0 ? median_of(agl_values, agl_count) : 10.0;
  for (size_t index = 0; index < used; index++) {
    double h = points[index].source->agl_m;
    if (!isfinite(h) || h < 0.5) {
      h = height;
    }
    points[index].height2 = h * h;
  }
  double east = east0;
  double north = north0;
  double peak_height = sqrt(points[peak_index].height2);
  double level0 = max_level + 20.0 * log10(fmax(peak_height, 1.0));
  bool fit_ok = true;
  for (int iteration = 0; iteration < 8; iteration++) {
    double jtj[3][3] = {{0}};
    double jtr[3] = {0};
    for (size_t index = 0; index < used; index++) {
      double dx = points[index].east - east;
      double dy = points[index].north - north;
      double r2 = dx * dx + dy * dy + points[index].height2;
      double predicted = level0 - 10.0 * log10(r2);
      double residual = points[index].level - predicted;
      // Huber weighting keeps a stray loud/quiet window from steering the fit.
      double w = fabs(residual) > 6.0 ? 6.0 / fabs(residual) : 1.0;
      double k = 20.0 / (r2 * log(10.0));
      double j[3] = {k * dx, k * dy, 1.0};
      for (int a = 0; a < 3; a++) {
        jtr[a] += w * j[a] * residual;
        for (int b = 0; b < 3; b++) {
          jtj[a][b] += w * j[a] * j[b];
        }
      }
    }
    // Levenberg damping for stability with few, collinear samples.
    for (int a = 0; a < 3; a++) {
      jtj[a][a] += 1e-3 * (jtj[a][a] + 1.0);
    }
    // Solve 3x3 by Cramer's rule.
    double det = jtj[0][0] * (jtj[1][1] * jtj[2][2] - jtj[1][2] * jtj[2][1])
                 - jtj[0][1] * (jtj[1][0] * jtj[2][2] - jtj[1][2] * jtj[2][0])
                 + jtj[0][2] * (jtj[1][0] * jtj[2][1] - jtj[1][1] * jtj[2][0]);
    if (!isfinite(det) || fabs(det) < 1e-12) {
      fit_ok = false;
      break;
    }
    double delta[3];
    for (int column = 0; column < 3; column++) {
      double m[3][3];
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          m[row][col] = (col == column) ? jtr[row] : jtj[row][col];
        }
      }
      double d = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                 - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                 + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
      delta[column] = d / det;
    }
    // Bound each step so a degenerate geometry cannot fling the estimate away.
    double step = sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
    if (step > 25.0) {
      delta[0] *= 25.0 / step;
      delta[1] *= 25.0 / step;
    }
    east += delta[0];
    north += delta[1];
    level0 += fmax(-6.0, fmin(6.0, delta[2]));
    if (step < 0.05) {
      break;
    }
  }
  // Reject a fit that wandered far outside the surveyed track.
  double drift = hypot(east - east0, north - north0);
  if (!fit_ok || !isfinite(east) || !isfinite(north) || drift > 60.0) {
    east = east0;
    north = north0;
  }

  // Confidence: peak prominence, spatial concentration, and sample support.
  double prominence = fmin(1.0, fmax(0.0, (max_level - median_level) / 12.0));
  double spread = 0.0;
  for (size_t index = 0; index < used; index++) {
    if (points[index].weight > 0.0) {
      spread += points[index].weight * hypot(points[index].east - east, points[index].north - north);
    }
  }
  spread /= sum_weight;
  double concentration = fmin(1.0, fmax(0.0, 1.0 - spread / 40.0));
  double support = fmin(1.0, (double)used / 100.0);
  double confidence = 0.5 * prominence + 0.3 * concentration + 0.2 * support;

  result->valid = true;
  result->lat_deg = origin_lat + (north / EARTH_RADIUS_M) * RAD_TO_DEG;
  result->lon_deg = origin_lon + (east / (EARTH_RADIUS_M * cos_lat)) * RAD_TO_DEG;
  // The spot is on the ground: report the listening height above it and the
  // ground's ellipsoid altitude (aircraft altitude minus AGL at the loudest window).
  result->agl_m = height;
  result->alt_m = points[peak_index].source->alt_m - sqrt(points[peak_index].height2);
  result->level_db = max_level;
  result->confidence = confidence;
  result->used_count = (uint32_t)used;

  free(points);
  free(levels);
  return 0;
}
