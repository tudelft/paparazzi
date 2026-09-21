#include "vehicle_detect_pipe.h"
#include "path_utils.h"

/**
 * @file vehicle_detect_pipe.c
 * @brief CATIA-side supervisor for the persistent on-sensor vehicle-detection server.
 * @details The server (vehicle_detect_server.py) holds the IMX500 network loaded and the
 * camera streaming, avoiding the 17-60s per-open firmware upload on every shot. This parent
 * owns its pipes and child lifecycle, validates the single-line protocol reply, and permits
 * one bounded respawn/retry when the server dies between requests. All failure paths clear
 * state before returning so a later shot never reuses a stale descriptor. Mirrors
 * lwir_cam_pipe.c's persistent-server pattern; the protocol here is simpler because every
 * field the caller needs rides on the one terminal reply line instead of LWIR's separate
 * opportunistic timing lines. vehicle_detect_pipe_save_detection_copy() additionally
 * duplicates a hit into a "detections" subdirectory of the photo directory once catia.c
 * has finished tagging it, so a review pass doesn't have to open every photo's EXIF to
 * find the ones worth looking at.
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

/** Defaults to the same directory as plain AICam (CATIA_AI_CAM_PHOTO_DIR, already
 * defined globally via the build's CFLAGS): --aicam-detect is still conceptually "the
 * AI camera", just with detection added, so it belongs in the same photo directory. */
#ifndef CATIA_VEHICLE_DETECT_PHOTO_DIR
#define CATIA_VEHICLE_DETECT_PHOTO_DIR CATIA_AI_CAM_PHOTO_DIR
#endif

#ifndef CATIA_VEHICLE_DETECT_COMMAND
#define CATIA_VEHICLE_DETECT_COMMAND "vehicle_detect_server.py"
#endif

#ifndef CATIA_VEHICLE_DETECT_MODEL
#define CATIA_VEHICLE_DETECT_MODEL "models/vehicle/network.rpk"
#endif

#ifndef CATIA_VEHICLE_DETECT_LABELS
#define CATIA_VEHICLE_DETECT_LABELS "models/vehicle/labels.txt"
#endif

extern char **environ;

static pid_t capture_server_pid = -1;
static int capture_server_input = -1;
static int capture_server_output = -1;
/** Buffered reassembly for read_one_line(): avoids one poll()+read() syscall pair per
 * byte, mirroring lwir_cam_pipe.c's status_buffer. */
static char status_buffer[512];
static size_t status_buffer_size, status_buffer_pos;
static const char *photo_directory = CATIA_VEHICLE_DETECT_PHOTO_DIR;

struct detection_result {
  bool valid;
  int count;
  double conf;
  int box_x, box_y, box_w, box_h;
};
static struct detection_result last_detection;

void vehicle_detect_pipe_set_photo_directory(const char *directory)
{
  photo_directory = directory == NULL ? CATIA_VEHICLE_DETECT_PHOTO_DIR : directory;
}

void vehicle_detect_pipe_last_detection_summary(char *buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) return;
  if (!last_detection.valid) {
    snprintf(buffer, buffer_size, "status=analysis_failed; coordinates_omitted=true");
  } else if (last_detection.count <= 0) {
    snprintf(buffer, buffer_size, "status=ok; count=0");
  } else {
    snprintf(buffer, buffer_size,
             "status=ok; count=%d; label=vehicle; confidence=%.4f; box=%d,%d,%d,%d",
             last_detection.count, last_detection.conf,
             last_detection.box_x, last_detection.box_y, last_detection.box_w, last_detection.box_h);
  }
}

/** @brief Copy every byte of @p source_path into a newly created/truncated @p dest_path.
 * @return 0 on a fully-flushed copy, -1 otherwise (partial output is removed). */
static int copy_file(const char *source_path, const char *dest_path)
{
  int source_fd = open(source_path, O_RDONLY);
  if (source_fd < 0) {
    return -1;
  }
  int dest_fd = open(dest_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (dest_fd < 0) {
    close(source_fd);
    return -1;
  }

  char buffer[65536];
  ssize_t bytes_read;
  int result = 0;
  while ((bytes_read = read(source_fd, buffer, sizeof(buffer))) > 0) {
    size_t written = 0;
    while (written < (size_t)bytes_read) {
      ssize_t bytes_written = write(dest_fd, buffer + written, (size_t)bytes_read - written);
      if (bytes_written < 0) {
        if (errno == EINTR) {
          continue;
        }
        result = -1;
        break;
      }
      written += (size_t)bytes_written;
    }
    if (result != 0) {
      break;
    }
  }
  if (bytes_read < 0) {
    result = -1;
  }

  close(source_fd);
  if (close(dest_fd) != 0) {
    result = -1;
  }
  if (result != 0) {
    unlink(dest_path);
  }
  return result;
}

void vehicle_detect_pipe_save_detection_copy(const char *filename)
{
  if (filename == NULL || filename[0] == '\0' || !last_detection.valid || last_detection.count <= 0) {
    return;
  }

  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(photo_directory, resolved_dir, sizeof(resolved_dir));

  char hits_dir[PATH_MAX];
  int dir_len = snprintf(hits_dir, sizeof(hits_dir), "%s/detections", photo_dir);
  if (dir_len < 0 || (size_t)dir_len >= sizeof(hits_dir) || catia_ensure_directory(hits_dir) != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tfailed to prepare detections directory under %s\n", photo_dir);
    return;
  }

  const char *base_name = strrchr(filename, '/');
  base_name = base_name != NULL ? base_name + 1 : filename;

  char dest_path[PATH_MAX];
  int dest_len = snprintf(dest_path, sizeof(dest_path), "%s/%s", hits_dir, base_name);
  if (dest_len < 0 || (size_t)dest_len >= sizeof(dest_path)) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tdetections path too long for %s\n", filename);
    return;
  }

  if (copy_file(filename, dest_path) != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tfailed to copy %s into detections directory: %s\n",
            filename, strerror(errno));
  } else {
    printf("VEHICLE_DETECT_PIPE:\tvehicle detected (count=%d, conf=%.2f); copied to %s\n",
           last_detection.count, last_detection.conf, dest_path);
  }
}

/** @brief Read one newline-terminated line from the server into a caller buffer.
 * @return 0 with @p line populated (newline stripped), or -1 on timeout, EOF, or error. */
static int read_one_line(char *line, size_t line_capacity, int timeout_ms)
{
  size_t line_size = 0;
  for (;;) {
    if (status_buffer_pos == status_buffer_size) {
      if (capture_server_output < 0) return -1;
      struct pollfd descriptor = {capture_server_output, POLLIN, 0};
      int poll_result;
      do {
        poll_result = poll(&descriptor, 1, timeout_ms);
      } while (poll_result < 0 && errno == EINTR);
      if (poll_result <= 0) {
        return -1;
      }
      ssize_t bytes_read;
      do {
        bytes_read = read(capture_server_output, status_buffer, sizeof(status_buffer));
      } while (bytes_read < 0 && errno == EINTR);
      if (bytes_read <= 0) {
        return -1;
      }
      status_buffer_size = (size_t)bytes_read;
      status_buffer_pos = 0;
    }
    char character = status_buffer[status_buffer_pos++];
    if (character != '\n') {
      if (line_size + 1 < line_capacity) {
        line[line_size++] = character;
      }
      continue;
    }
    line[line_size] = '\0';
    return 0;
  }
}

/** @brief Close pipes and reap or terminate the owned capture server. */
static void stop_capture_server(void)
{
  status_buffer_size = status_buffer_pos = 0;
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

int vehicle_detect_pipe_init(const char *unused)
{
  (void)unused;
  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(photo_directory, resolved_dir, sizeof(resolved_dir));

  if (catia_ensure_directory(photo_dir) != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tfailed to create photo directory %s: %s\n",
            photo_dir, strerror(errno));
    return -1;
  }

  struct stat directory_status;
  if (stat(photo_dir, &directory_status) != 0
      || !S_ISDIR(directory_status.st_mode)
      || access(photo_dir, W_OK) != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tphoto directory is not writable: %s\n", photo_dir);
    return -1;
  }

  if (access(CATIA_VEHICLE_DETECT_COMMAND, X_OK) != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tcapture command is not executable: %s: %s\n",
            CATIA_VEHICLE_DETECT_COMMAND, strerror(errno));
    return -1;
  }

  int command_pipe[2];
  int response_pipe[2];
  if (pipe(command_pipe) != 0 || pipe(response_pipe) != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tfailed to create capture-server pipes: %s\n",
            strerror(errno));
    return -1;
  }

  posix_spawn_file_actions_t actions;
  if (posix_spawn_file_actions_init(&actions) != 0
      || posix_spawn_file_actions_adddup2(&actions, command_pipe[0], STDIN_FILENO) != 0
      || posix_spawn_file_actions_adddup2(&actions, response_pipe[1], STDOUT_FILENO) != 0
      || posix_spawn_file_actions_addclose(&actions, command_pipe[1]) != 0
      || posix_spawn_file_actions_addclose(&actions, response_pipe[0]) != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tfailed to configure capture server\n");
    close(command_pipe[0]);
    close(command_pipe[1]);
    close(response_pipe[0]);
    close(response_pipe[1]);
    return -1;
  }

  char *const arguments[] = {
    (char *)CATIA_VEHICLE_DETECT_COMMAND,
    (char *)"--model", (char *)CATIA_VEHICLE_DETECT_MODEL,
    (char *)"--labels", (char *)CATIA_VEHICLE_DETECT_LABELS,
    NULL
  };
  int spawn_result = posix_spawn(&capture_server_pid, CATIA_VEHICLE_DETECT_COMMAND,
                                 &actions, NULL, arguments, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(command_pipe[0]);
  close(response_pipe[1]);
  if (spawn_result != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tfailed to start %s: %s\n",
            CATIA_VEHICLE_DETECT_COMMAND, strerror(spawn_result));
    close(command_pipe[1]);
    close(response_pipe[0]);
    capture_server_pid = -1;
    return -1;
  }
  capture_server_input = command_pipe[1];
  capture_server_output = response_pipe[0];

  /** Firmware upload onto the IMX500 chip is observed at 17s clean, up to 48-60s when
   * the sensor's onboard rp2040-gpio-bridge chatter is active (see Notes/Mission 1/IMAV
   * input tensor injection.md). 70s leaves headroom above the documented worst case,
   * matching LWIR's 30s readiness timeout for its own slow persistent-server startup. */
  char line[256];
  bool ready = false;
  while (read_one_line(line, sizeof(line), 70000) == 0) {
    if (strcmp(line, "AICAM_SERVER_READY") == 0) {
      ready = true;
      break;
    }
  }
  if (!ready) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tcapture server did not become ready\n");
    stop_capture_server();
    return -1;
  }

  printf("VEHICLE_DETECT_PIPE:\tcapture command: %s\n", CATIA_VEHICLE_DETECT_COMMAND);
  printf("VEHICLE_DETECT_PIPE:\tphoto directory: %s\n", photo_dir);
  printf("VEHICLE_DETECT_PIPE:\tpersistent capture server ready\n");
  return 0;
}

/** @brief Send one filename request and parse its single-line reply.
 * @return 0 with @p result populated on AICAM_SERVER_OK, otherwise -1 with the server
 * already reset (ERROR reply, malformed reply, timeout, or a broken pipe). */
static int send_request_and_wait(const char *filename, struct detection_result *result)
{
  char request[PATH_MAX + 2];
  int request_size = snprintf(request, sizeof(request), "%s\n", filename);
  if (request_size < 0 || (size_t)request_size >= sizeof(request)) {
    return -1;
  }

  size_t sent = 0;
  while (sent < (size_t)request_size) {
    ssize_t written = write(capture_server_input, request + sent, (size_t)request_size - sent);
    if (written > 0) {
      sent += (size_t)written;
    } else if (written < 0 && errno != EINTR) {
      fprintf(stderr, "VEHICLE_DETECT_PIPE:\tfailed to send request: %s\n", strerror(errno));
      stop_capture_server();
      return -1;
    }
  }

  char line[512];
  /** Per-shot work is a couple of camera frames (tens of ms); 10s leaves ample margin. */
  if (read_one_line(line, sizeof(line), 10000) != 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tcapture server did not reply in time\n");
    stop_capture_server();
    return -1;
  }
  if (strncmp(line, "AICAM_SERVER_ERROR", 19) == 0) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\t%s\n", line);
    return -1;
  }
  int count, box_x, box_y, box_w, box_h;
  double conf;
  if (sscanf(line, "AICAM_SERVER_OK count=%d conf=%lf box_x=%d box_y=%d box_w=%d box_h=%d",
             &count, &conf, &box_x, &box_y, &box_w, &box_h) != 6) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tunexpected reply: %s\n", line);
    stop_capture_server();
    return -1;
  }
  result->valid = true;
  result->count = count;
  result->conf = conf;
  result->box_x = box_x;
  result->box_y = box_y;
  result->box_w = box_w;
  result->box_h = box_h;
  return 0;
}

int vehicle_detect_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  last_detection = (struct detection_result){0};
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(photo_directory, resolved_dir, sizeof(resolved_dir));

  int length = snprintf(filename, filename_size, "%s/a%06d.jpg", photo_dir, image_number);
  if (length < 0 || (size_t)length >= filename_size) {
    filename[0] = '\0';
    return -1;
  }

  if (unlink(filename) != 0 && errno != ENOENT) {
    fprintf(stderr, "VEHICLE_DETECT_PIPE:\tfailed to replace %s: %s\n", filename, strerror(errno));
    filename[0] = '\0';
    return -1;
  }

  /** One bounded respawn/retry if the server died between shots, matching
   * lwir_cam_pipe_shoot()'s recovery so a single transient failure does not lose the shot. */
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (capture_server_pid < 0 || capture_server_input < 0 || capture_server_output < 0) {
      if (attempt > 0) {
        fprintf(stderr, "VEHICLE_DETECT_PIPE:\tcapture server died; restarting for one retry\n");
      }
      if (vehicle_detect_pipe_init(NULL) != 0) {
        break;
      }
    }

    struct detection_result result = {0};
    if (send_request_and_wait(filename, &result) != 0) {
      continue;
    }

    struct stat image_status;
    if (stat(filename, &image_status) != 0 || !S_ISREG(image_status.st_mode)
        || image_status.st_size == 0) {
      fprintf(stderr, "VEHICLE_DETECT_PIPE:\t%s did not create a valid image at %s\n",
              CATIA_VEHICLE_DETECT_COMMAND, filename);
      unlink(filename);
      filename[0] = '\0';
      return -1;
    }
    last_detection = result;
    return 0;
  }

  unlink(filename);
  filename[0] = '\0';
  return -1;
}

void vehicle_detect_pipe_deinit(void)
{
  stop_capture_server();
}
