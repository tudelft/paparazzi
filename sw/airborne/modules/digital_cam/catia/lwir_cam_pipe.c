#include "lwir_cam_pipe.h"

#include <errno.h>
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

#ifndef CATIA_LWIR_CAM_PHOTO_DIR
#define CATIA_LWIR_CAM_PHOTO_DIR "photos"
#endif

#ifndef CATIA_LWIR_CAM_COMMAND
#define CATIA_LWIR_CAM_COMMAND "lwircam/sample"
#endif

extern char **environ;

static pid_t capture_server_pid = -1;
static int capture_server_input = -1;
static int capture_server_output = -1;

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

int lwir_cam_pipe_init(const char *unused)
{
  (void)unused;

  if (mkdir(CATIA_LWIR_CAM_PHOTO_DIR, 0755) != 0 && errno != EEXIST) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to create photo directory %s: %s\n",
            CATIA_LWIR_CAM_PHOTO_DIR, strerror(errno));
    return -1;
  }

  struct stat directory_status;
  if (stat(CATIA_LWIR_CAM_PHOTO_DIR, &directory_status) != 0
      || !S_ISDIR(directory_status.st_mode)
      || access(CATIA_LWIR_CAM_PHOTO_DIR, W_OK) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tphoto directory is not writable: %s\n",
            CATIA_LWIR_CAM_PHOTO_DIR);
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

  char *const arguments[] = {
    (char *)CATIA_LWIR_CAM_COMMAND,
    (char *)"--capture-server",
    (char *)"--bare",
    NULL
  };
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
  printf("LWIR_CAM_PIPE:\tphoto directory: %s\n", CATIA_LWIR_CAM_PHOTO_DIR);
  printf("LWIR_CAM_PIPE:\tpersistent capture server ready\n");
  return 0;
}

int lwir_cam_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  int length = snprintf(filename, filename_size, "%s/l%06d.jpg",
                        CATIA_LWIR_CAM_PHOTO_DIR, image_number);
  if (length < 0 || (size_t)length >= filename_size) {
    filename[0] = '\0';
    return -1;
  }

  if (unlink(filename) != 0 && errno != ENOENT) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to replace %s: %s\n", filename, strerror(errno));
    filename[0] = '\0';
    return -1;
  }

  if (capture_server_pid < 0 || capture_server_input < 0 || capture_server_output < 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server is unavailable\n");
    filename[0] = '\0';
    return -1;
  }

  char request[PATH_MAX + 2];
  int request_size = snprintf(request, sizeof(request), "%s\n", filename);
  if (request_size < 0 || (size_t)request_size >= sizeof(request)) {
    filename[0] = '\0';
    return -1;
  }
  size_t sent = 0;
  while (sent < (size_t)request_size) {
    ssize_t written = write(capture_server_input, request + sent,
                            (size_t)request_size - sent);
    if (written > 0) {
      sent += (size_t)written;
    } else if (written < 0 && errno != EINTR) {
      fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to request capture: %s\n", strerror(errno));
      stop_capture_server();
      filename[0] = '\0';
      return -1;
    }
  }
  if (read_server_status("LWIR_SERVER_OK", 21000) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server failed to save %s\n", filename);
    unlink(filename);
    filename[0] = '\0';
    return -1;
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

void lwir_cam_pipe_deinit(void)
{
  stop_capture_server();
}