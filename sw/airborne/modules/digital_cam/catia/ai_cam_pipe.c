#include "ai_cam_pipe.h"
#include "path_utils.h"

#include <errno.h>
#include <signal.h>
#include <spawn.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef CATIA_AI_CAM_TIMEOUT_SECONDS
#define CATIA_AI_CAM_TIMEOUT_SECONDS 30
#endif
#include <unistd.h>

#ifndef CATIA_AI_CAM_PHOTO_DIR
#define CATIA_AI_CAM_PHOTO_DIR "photos"
#endif

#ifndef CATIA_AI_CAM_COMMAND
#define CATIA_AI_CAM_COMMAND "rpicam-still"
#endif

extern char **environ;

int ai_cam_pipe_init(const char *unused)
{
  (void)unused;
  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(CATIA_AI_CAM_PHOTO_DIR, resolved_dir, sizeof(resolved_dir));

  if (catia_ensure_directory(photo_dir) != 0) {
    fprintf(stderr, "AI_CAM_PIPE:\tfailed to create photo directory %s: %s\n",
            photo_dir, strerror(errno));
    return -1;
  }

  struct stat directory_status;
  if (stat(photo_dir, &directory_status) != 0 || !S_ISDIR(directory_status.st_mode)
      || access(photo_dir, W_OK) != 0) {
    fprintf(stderr, "AI_CAM_PIPE:\tphoto directory is not writable: %s\n", photo_dir);
    return -1;
  }

  printf("AI_CAM_PIPE:\tcapture command: %s\n", CATIA_AI_CAM_COMMAND);
  printf("AI_CAM_PIPE:\tphoto directory: %s\n", photo_dir);
  return 0;
}

int ai_cam_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(CATIA_AI_CAM_PHOTO_DIR, resolved_dir, sizeof(resolved_dir));

  int length = snprintf(filename, filename_size, "%s/a%06d.jpg",
                        photo_dir, image_number);
  if (length < 0 || (size_t)length >= filename_size) {
    filename[0] = '\0';
    return -1;
  }

  if (unlink(filename) != 0 && errno != ENOENT) {
    fprintf(stderr, "AI_CAM_PIPE:\tfailed to replace %s: %s\n", filename, strerror(errno));
    filename[0] = '\0';
    return -1;
  }

  char *const arguments[] = {
    (char *)CATIA_AI_CAM_COMMAND,
    (char *)"-o", filename,
    (char *)"--width", (char *)"4056",
    (char *)"--height", (char *)"3040",
    (char *)"--hflip",
    (char *)"--vflip",
    NULL
  };
  pid_t camera_pid;
  int spawn_result = posix_spawnp(&camera_pid, CATIA_AI_CAM_COMMAND, NULL, NULL, arguments, environ);
  if (spawn_result != 0) {
    fprintf(stderr, "AI_CAM_PIPE:\tfailed to start %s: %s\n",
            CATIA_AI_CAM_COMMAND, strerror(spawn_result));
    filename[0] = '\0';
    return -1;
  }

  int camera_status;
  /** A hung rpicam-still must not block this worker thread (and, once all worker slots
   * fill with the same hang, the entire capture pipeline) forever: wait with a bounded
   * deadline and kill it if exceeded, so the shot fails fast and the next one can retry. */
  struct timespec deadline;
  if (clock_gettime(CLOCK_MONOTONIC, &deadline) != 0) {
    fprintf(stderr, "AI_CAM_PIPE:\tfailed to read clock: %s\n", strerror(errno));
    filename[0] = '\0';
    return -1;
  }
  deadline.tv_sec += CATIA_AI_CAM_TIMEOUT_SECONDS;
  bool killed = false;
  for (;;) {
    pid_t wait_result = waitpid(camera_pid, &camera_status, WNOHANG);
    if (wait_result == camera_pid) {
      break;
    }
    if (wait_result < 0 && errno != EINTR) {
      fprintf(stderr, "AI_CAM_PIPE:\tfailed to wait for %s: %s\n",
              CATIA_AI_CAM_COMMAND, strerror(errno));
      filename[0] = '\0';
      return -1;
    }
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
      now = deadline;
    }
    if (!killed && (now.tv_sec > deadline.tv_sec
        || (now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec))) {
      fprintf(stderr, "AI_CAM_PIPE:\t%s did not exit within %d s; killing it\n",
              CATIA_AI_CAM_COMMAND, CATIA_AI_CAM_TIMEOUT_SECONDS);
      kill(camera_pid, SIGKILL);
      killed = true;
    }
    usleep(20000);
  }
  if (killed) {
    unlink(filename);
    filename[0] = '\0';
    return -1;
  }

  if (!WIFEXITED(camera_status) || WEXITSTATUS(camera_status) != 0) {
    if (WIFEXITED(camera_status)) {
      fprintf(stderr, "AI_CAM_PIPE:\t%s exited with status %d\n",
              CATIA_AI_CAM_COMMAND, WEXITSTATUS(camera_status));
    } else if (WIFSIGNALED(camera_status)) {
      fprintf(stderr, "AI_CAM_PIPE:\t%s terminated by signal %d\n",
              CATIA_AI_CAM_COMMAND, WTERMSIG(camera_status));
    } else {
      fprintf(stderr, "AI_CAM_PIPE:\t%s did not complete normally\n", CATIA_AI_CAM_COMMAND);
    }
    unlink(filename);
    filename[0] = '\0';
    return -1;
  }

  struct stat image_status;
  if (stat(filename, &image_status) != 0 || !S_ISREG(image_status.st_mode)
      || image_status.st_size == 0) {
    fprintf(stderr, "AI_CAM_PIPE:\t%s did not create a valid image at %s\n",
            CATIA_AI_CAM_COMMAND, filename);
    unlink(filename);
    filename[0] = '\0';
    return -1;
  }

  return 0;
}

void ai_cam_pipe_deinit(void)
{
}