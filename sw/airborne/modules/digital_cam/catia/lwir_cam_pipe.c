#include "lwir_cam_pipe.h"
#include "path_utils.h"

#include <errno.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#ifndef CATIA_LWIR_CAM_PHOTO_DIR
#define CATIA_LWIR_CAM_PHOTO_DIR "photos"
#endif

#ifndef CATIA_LWIR_CAM_COMMAND
#define CATIA_LWIR_CAM_COMMAND "lwircam/lwircam"
#endif

extern char **environ;

static pid_t capture_server_pid = -1;
static int capture_server_input = -1;
static int capture_server_output = -1;
static double capture_delay_s = -1;
static struct capture_timing capture_times;
static bool native_raw_enabled;
static const char *calibration_path;

void lwir_cam_pipe_set_native_raw(int enabled)
{
  native_raw_enabled = enabled != 0;
}

int lwir_cam_pipe_set_calibration(const char *path)
{
  if (path == NULL || path[0] == '\0') return -1;
  calibration_path = path;
  return 0;
}

struct capture_timing lwir_cam_pipe_capture_timing(void)
{
  return capture_times;
}

static int parse_capture_timing(const char *text, struct capture_timing *timing, bool callback)
{
  uint64_t values[4] = {0};
  const size_t count = callback ? 4 : 2;
  for (size_t index = 0; index < count; ++index) {
    char *end;
    if (*text < '0' || *text > '9') return 0;
    errno = 0;
    unsigned long long value = strtoull(text, &end, 10);
    if (errno != 0 || value > UINT64_MAX) return 0;
    values[index] = value;
    if (index + 1 == count) {
      if (*end != '\0') return 0;
    } else {
      if (*end != ' ') return 0;
      text = end + 1;
    }
  }
  struct capture_timing parsed = {
    .request_monotonic_us = values[0], .arrival_monotonic_us = values[1],
    .callback_sequence = values[2], .callback_drops = values[3], .callback_arrival = callback
  };
  if (!capture_timing_valid(&parsed)) return 0;
  *timing = parsed;
  return 1;
}

double lwir_cam_pipe_capture_delay(void)
{
  return capture_delay_s;
}

static int read_server_status(const char *expected, int timeout_ms)
{
  char line[256];
  size_t line_size = 0;

  while (capture_server_output >= 0) {
    struct pollfd descriptor = {capture_server_output, POLLIN, 0};
    int poll_result;
    do {
      poll_result = poll(&descriptor, 1, timeout_ms);
    } while (poll_result < 0 && errno == EINTR);
    if (poll_result <= 0) {
      return -1;
    }

    char character;
    ssize_t bytes_read;
    do {
      bytes_read = read(capture_server_output, &character, 1);
    } while (bytes_read < 0 && errno == EINTR);
    if (bytes_read != 1) {
      return -1;
    }
    if (character != '\n' && line_size + 1 < sizeof(line)) {
      line[line_size++] = character;
      continue;
    }

    line[line_size] = '\0';
    double delay;
    char trailing;
    if (strcmp(expected, "LWIR_SERVER_OK") == 0 && !capture_timing_valid(&capture_times)
        && sscanf(line, "LWIR_SERVER_DELAY %lf %c", &delay, &trailing) == 1
        && isfinite(delay) && delay >= 0 && delay <= 20) {
      capture_delay_s = delay;
    }
    const char timing_prefix[] = "LWIR_SERVER_TIMING ";
    if (strcmp(expected, "LWIR_SERVER_OK") == 0
        && !capture_times.callback_arrival
        && strncmp(line, timing_prefix, sizeof(timing_prefix) - 1) == 0
        && parse_capture_timing(line + sizeof(timing_prefix) - 1, &capture_times, false)) {
      capture_delay_s = (capture_times.arrival_monotonic_us - capture_times.request_monotonic_us) / 1e6;
    }
    const char callback_prefix[] = "LWIR_SERVER_CALLBACK_TIMING ";
    if (strcmp(expected, "LWIR_SERVER_OK") == 0
        && strncmp(line, callback_prefix, sizeof(callback_prefix) - 1) == 0
        && parse_capture_timing(line + sizeof(callback_prefix) - 1, &capture_times, true)) {
      capture_delay_s = (capture_times.arrival_monotonic_us - capture_times.request_monotonic_us) / 1e6;
    }
    if (strcmp(line, expected) == 0) {
      return 0;
    }
    if (strncmp(line, "LWIR_SERVER_ERROR", 17) == 0) {
      fprintf(stderr, "LWIR_CAM_PIPE:\t%s\n", line);
      return -1;
    }
    line_size = 0;
  }
  return -1;
}

static void stop_capture_server(void)
{
  if (capture_server_input >= 0) {
    close(capture_server_input);
    capture_server_input = -1;
  }
  if (capture_server_output >= 0) {
    close(capture_server_output);
    capture_server_output = -1;
  }
  if (capture_server_pid < 0) {
    return;
  }

  int status;
  pid_t wait_result;
  do {
    wait_result = waitpid(capture_server_pid, &status, WNOHANG);
  } while (wait_result < 0 && errno == EINTR);
  if (wait_result == 0) {
    kill(capture_server_pid, SIGTERM);
    do {
      wait_result = waitpid(capture_server_pid, &status, 0);
    } while (wait_result < 0 && errno == EINTR);
  }
  capture_server_pid = -1;
}

/** Send one line request to the running server and wait for its reply. On any
 * failure (broken pipe, dead process, or a timed-out/erroring reply) the server
 * state is fully reset so a caller's retry starts with a clean respawn instead of
 * repeatedly hitting the same stale, already-dead connection. */
static int send_request_and_wait(const char *request, size_t request_size, int timeout_ms)
{
  size_t sent = 0;
  while (sent < request_size) {
    ssize_t written = write(capture_server_input, request + sent, request_size - sent);
    if (written > 0) {
      sent += (size_t)written;
    } else if (written < 0 && errno != EINTR) {
      fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to send request: %s\n", strerror(errno));
      stop_capture_server();
      return -1;
    }
  }
  if (read_server_status("LWIR_SERVER_OK", timeout_ms) != 0) {
    stop_capture_server();
    return -1;
  }
  return 0;
}

int lwir_cam_pipe_init(const char *unused)
{
  (void)unused;
  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(CATIA_LWIR_CAM_PHOTO_DIR, resolved_dir, sizeof(resolved_dir));

  if (catia_ensure_directory(photo_dir) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to create photo directory %s: %s\n",
            photo_dir, strerror(errno));
    return -1;
  }

  struct stat directory_status;
  if (stat(photo_dir, &directory_status) != 0
      || !S_ISDIR(directory_status.st_mode)
      || access(photo_dir, W_OK) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tphoto directory is not writable: %s\n",
            photo_dir);
    return -1;
  }
  if (access(CATIA_LWIR_CAM_COMMAND, X_OK) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture command is not executable: %s: %s\n",
            CATIA_LWIR_CAM_COMMAND, strerror(errno));
    return -1;
  }

  int command_pipe[2];
  int response_pipe[2];
  if (pipe(command_pipe) != 0 || pipe(response_pipe) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to create capture-server pipes: %s\n",
            strerror(errno));
    return -1;
  }

  posix_spawn_file_actions_t actions;
  if (posix_spawn_file_actions_init(&actions) != 0
      || posix_spawn_file_actions_adddup2(&actions, command_pipe[0], STDIN_FILENO) != 0
      || posix_spawn_file_actions_adddup2(&actions, response_pipe[1], STDOUT_FILENO) != 0
      || posix_spawn_file_actions_addclose(&actions, command_pipe[1]) != 0
      || posix_spawn_file_actions_addclose(&actions, response_pipe[0]) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to configure capture server\n");
    close(command_pipe[0]);
    close(command_pipe[1]);
    close(response_pipe[0]);
    close(response_pipe[1]);
    return -1;
  }

  char *arguments[7];
  size_t count = 0;
  arguments[count++] = (char *)CATIA_LWIR_CAM_COMMAND;
  arguments[count++] = (char *)"--capture-server";
  arguments[count++] = (char *)"--bare";
  if (native_raw_enabled) arguments[count++] = (char *)"--native-raw";
  if (calibration_path != NULL) {
    arguments[count++] = (char *)"--calibration";
    arguments[count++] = (char *)calibration_path;
  }
  arguments[count] = NULL;
  int spawn_result = posix_spawn(&capture_server_pid, CATIA_LWIR_CAM_COMMAND,
                                 &actions, NULL, arguments, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(command_pipe[0]);
  close(response_pipe[1]);
  if (spawn_result != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to start %s: %s\n",
            CATIA_LWIR_CAM_COMMAND, strerror(spawn_result));
    close(command_pipe[1]);
    close(response_pipe[0]);
    capture_server_pid = -1;
    return -1;
  }
  capture_server_input = command_pipe[1];
  capture_server_output = response_pipe[0];
  if (read_server_status("LWIR_SERVER_READY", 30000) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server did not become ready\n");
    stop_capture_server();
    return -1;
  }

  printf("LWIR_CAM_PIPE:\tcapture command: %s\n", CATIA_LWIR_CAM_COMMAND);
  printf("LWIR_CAM_PIPE:\tphoto directory: %s\n", photo_dir);
  printf("LWIR_CAM_PIPE:\tpersistent capture server ready\n");
  return 0;
}

int lwir_cam_pipe_warmup(void)
{
  /** Open the sensor once, wait past its power-on "wiggly" image, then close it
   * again. Runs regardless of whether LWIR ends up selected for a shot, so a
   * later real open (persistent server or standalone capture) never has to
   * absorb that transient inline. Failure here is not fatal to CATIA. */
  if (access(CATIA_LWIR_CAM_COMMAND, X_OK) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\twarmup skipped, command is not executable: %s: %s\n",
            CATIA_LWIR_CAM_COMMAND, strerror(errno));
    return -1;
  }

  char *const arguments[] = {
    (char *)CATIA_LWIR_CAM_COMMAND,
    (char *)"--warmup",
    NULL
  };
  pid_t warmup_pid;
  int spawn_result = posix_spawn(&warmup_pid, CATIA_LWIR_CAM_COMMAND,
                                 NULL, NULL, arguments, environ);
  if (spawn_result != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to start warmup: %s\n", strerror(spawn_result));
    return -1;
  }

  int status;
  pid_t wait_result;
  do {
    wait_result = waitpid(warmup_pid, &status, 0);
  } while (wait_result < 0 && errno == EINTR);
  if (wait_result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tsensor not accessible or warmup failed\n");
    return -1;
  }
  return 0;
}

static int process_image(char *filename, int geolocate)
{
  if (filename == NULL || filename[0] == '\0') {
    return -1;
  }

  char *arguments[7];
  size_t count = 0;
  arguments[count++] = (char *)CATIA_LWIR_CAM_COMMAND;
  if (geolocate) {
    arguments[count++] = (char *)"--geolocate";
    arguments[count++] = filename;
    if (calibration_path != NULL) {
      arguments[count++] = (char *)"--calibration";
      arguments[count++] = (char *)calibration_path;
    }
  } else {
    arguments[count++] = (char *)"--mock-image";
    arguments[count++] = filename;
    arguments[count++] = (char *)"--output";
    arguments[count++] = filename;
  }
  arguments[count] = NULL;
  pid_t camera_pid;
  int spawn_result = posix_spawn(&camera_pid, CATIA_LWIR_CAM_COMMAND,
                                 NULL, NULL, arguments, environ);
  if (spawn_result != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to start image processing: %s\n",
                strerror(spawn_result));
    return -1;
  }

  int status;
  pid_t wait_result;
  do {
    wait_result = waitpid(camera_pid, &status, 0);
  } while (wait_result < 0 && errno == EINTR);
  if (wait_result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\timage processing failed\n");
    return -1;
  }
  return 0;
}

int lwir_cam_pipe_process_mock(char *filename)
{
  return process_image(filename, 0);
}

int lwir_cam_pipe_geolocate(char *filename)
{
  if (filename == NULL || filename[0] == '\0') {
    return -1;
  }
  if (capture_server_pid < 0 || capture_server_input < 0 || capture_server_output < 0) {
    /** No persistent server running (test/mock mode): fall back to a one-off process. */
    return process_image(filename, 1);
  }

  /** Reuse the already-running, already-warmed-up server for analysis instead of
   * spawning a fresh lwircam process for every shot. */
  char request[PATH_MAX + 8];
  int request_size = snprintf(request, sizeof(request), "GEO:%s\n", filename);
  if (request_size < 0 || (size_t)request_size >= sizeof(request)) {
    return -1;
  }
  if (send_request_and_wait(request, (size_t)request_size, 8000) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server failed to geolocate %s\n", filename);
    return -1;
  }
  return 0;
}

int lwir_cam_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  capture_delay_s = -1;
  capture_times = (struct capture_timing){0};
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(CATIA_LWIR_CAM_PHOTO_DIR, resolved_dir, sizeof(resolved_dir));

  int length = snprintf(filename, filename_size, "%s/l%06d.jpg",
                        photo_dir, image_number);
  if (length < 0 || (size_t)length >= filename_size) {
    filename[0] = '\0';
    return -1;
  }

  if (unlink(filename) != 0 && errno != ENOENT) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to replace %s: %s\n", filename, strerror(errno));
    filename[0] = '\0';
    return -1;
  }

  /** A mid-flight USB re-enumeration can silently kill the persistent server between
   * shots even though it was already marked initialized. Give this shot one bounded
   * chance to recover (respawn + retry) instead of losing it outright; if the sensor
   * is genuinely gone, camera_prepare()'s own init attempt on the next shot is what
   * stops further retries from then on. */
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (capture_server_pid < 0 || capture_server_input < 0 || capture_server_output < 0) {
      if (attempt > 0) {
        fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server died; restarting for one retry\n");
      }
      if (lwir_cam_pipe_init(NULL) != 0) {
        break;
      }
    }

    char request[PATH_MAX + 2];
    int request_size = snprintf(request, sizeof(request), "%s\n", filename);
    if (request_size < 0 || (size_t)request_size >= sizeof(request)) {
      filename[0] = '\0';
      return -1;
    }
    if (send_request_and_wait(request, (size_t)request_size, 8000) != 0) {
      fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server failed to save %s\n", filename);
      continue;
    }

    struct stat image_status;
    if (stat(filename, &image_status) != 0 || !S_ISREG(image_status.st_mode)
        || image_status.st_size == 0) {
      fprintf(stderr, "LWIR_CAM_PIPE:\t%s did not create a valid image at %s\n",
              CATIA_LWIR_CAM_COMMAND, filename);
      unlink(filename);
      filename[0] = '\0';
      return -1;
    }
    return 0;
  }

  unlink(filename);
  filename[0] = '\0';
  return -1;
}

void lwir_cam_pipe_deinit(void)
{
  stop_capture_server();
}