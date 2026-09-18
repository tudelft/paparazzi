#include "ai_cam_pipe.h"
#include "path_utils.h"

#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

/**
 * @file ai_cam_pipe.c
 * @brief Raspberry Pi AI Camera adapter backed by rpicam-still.
 * @details By default one rpicam-still runs in signal mode for the whole session
 * (camera opened once) and each shot is a SIGUSR1; the finished JPEG is detected with
 * inotify and renamed to the shot filename. Starting rpicam-still per shot costs ~1 s
 * of libcamera/sensor setup, which on a Pi Zero 2W is slower than a 1 Hz survey trigger.
 * If the persistent process misses a shot or dies, that shot falls back to one
 * rpicam-still per shot and the persistent process is restarted for the next one.
 * Build with CATIA_AI_CAM_PERSISTENT=0, or set CATIA_AI_CAM_ONESHOT=1 in the
 * environment, to always use one process per shot.
 */

#ifndef CATIA_AI_CAM_TIMEOUT_SECONDS
#define CATIA_AI_CAM_TIMEOUT_SECONDS 30
#endif

#ifndef CATIA_AI_CAM_PHOTO_DIR
#define CATIA_AI_CAM_PHOTO_DIR "photos"
#endif

#ifndef CATIA_AI_CAM_COMMAND
#define CATIA_AI_CAM_COMMAND "rpicam-still"
#endif

#ifndef CATIA_AI_CAM_WIDTH
#define CATIA_AI_CAM_WIDTH "2048"
#endif

#ifndef CATIA_AI_CAM_HEIGHT
#define CATIA_AI_CAM_HEIGHT "1520"
#endif

#ifndef CATIA_AI_CAM_PERSISTENT
#define CATIA_AI_CAM_PERSISTENT 1
#endif

/** Time from starting the persistent process until it may be signalled (camera open,
 * SIGUSR1 handler installed); a signal before that would terminate it. */
#ifndef CATIA_AI_CAM_STARTUP_MS
#define CATIA_AI_CAM_STARTUP_MS 3000
#endif

/** Longest wait from SIGUSR1 to the finished JPEG before falling back. */
#ifndef CATIA_AI_CAM_SIGNAL_TIMEOUT_MS
#define CATIA_AI_CAM_SIGNAL_TIMEOUT_MS 4000
#endif

#define AI_CAM_SIGNAL_PREFIX ".aicam_"

extern char **environ;
static const char *photo_directory = CATIA_AI_CAM_PHOTO_DIR;

static pid_t signal_pid = -1;
static int watch_fd = -1;
static int64_t signal_ready_ms;
static char signal_dir[PATH_MAX];

void ai_cam_pipe_set_photo_directory(const char *directory)
{
  photo_directory = directory == NULL ? CATIA_AI_CAM_PHOTO_DIR : directory;
}

static int64_t monotonic_ms(void)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static bool persistent_enabled(void)
{
  if (!CATIA_AI_CAM_PERSISTENT) {
    return false;
  }
  const char *oneshot = getenv("CATIA_AI_CAM_ONESHOT");
  return oneshot == NULL || oneshot[0] == '\0' || strcmp(oneshot, "0") == 0;
}

/** Reap the persistent process if it has exited; true while it is still running. */
static bool persistent_alive(void)
{
  if (signal_pid <= 0) {
    return false;
  }
  int status;
  pid_t result = waitpid(signal_pid, &status, WNOHANG);
  if (result == 0) {
    return true;
  }
  if (result == signal_pid) {
    fprintf(stderr, "AI_CAM_PIPE:\tpersistent %s exited\n", CATIA_AI_CAM_COMMAND);
  }
  signal_pid = -1;
  return false;
}

static void persistent_stop(void)
{
  if (persistent_alive()) {
    kill(signal_pid, SIGUSR2);
    int64_t deadline = monotonic_ms() + 1000;
    while (monotonic_ms() < deadline && waitpid(signal_pid, NULL, WNOHANG) == 0) {
      usleep(20000);
    }
    if (waitpid(signal_pid, NULL, WNOHANG) == 0) {
      kill(signal_pid, SIGKILL);
      waitpid(signal_pid, NULL, 0);
    }
  }
  signal_pid = -1;
  if (watch_fd >= 0) {
    close(watch_fd);
    watch_fd = -1;
  }
}

static int persistent_start(const char *photo_dir)
{
  persistent_stop();
  if (snprintf(signal_dir, sizeof(signal_dir), "%s", photo_dir) >= (int)sizeof(signal_dir)) {
    return -1;
  }
  watch_fd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
  if (watch_fd < 0 || inotify_add_watch(watch_fd, photo_dir, IN_CLOSE_WRITE | IN_MOVED_TO) < 0) {
    fprintf(stderr, "AI_CAM_PIPE:\tcannot watch %s: %s\n", photo_dir, strerror(errno));
    persistent_stop();
    return -1;
  }

  char pattern[PATH_MAX];
  if (snprintf(pattern, sizeof(pattern), "%s/" AI_CAM_SIGNAL_PREFIX "%%06d.jpg", photo_dir)
      >= (int)sizeof(pattern)) {
    persistent_stop();
    return -1;
  }
  char *const arguments[] = {
    (char *)CATIA_AI_CAM_COMMAND,
    (char *)"-n",
    (char *)"-t", (char *)"0",
    (char *)"--signal",
    (char *)"-o", pattern,
    (char *)"--width", (char *)CATIA_AI_CAM_WIDTH,
    (char *)"--height", (char *)CATIA_AI_CAM_HEIGHT,
    NULL
  };
  int spawn_result = posix_spawnp(&signal_pid, CATIA_AI_CAM_COMMAND, NULL, NULL, arguments, environ);
  if (spawn_result != 0) {
    fprintf(stderr, "AI_CAM_PIPE:\tfailed to start persistent %s: %s\n",
            CATIA_AI_CAM_COMMAND, strerror(spawn_result));
    signal_pid = -1;
    persistent_stop();
    return -1;
  }
  signal_ready_ms = monotonic_ms() + CATIA_AI_CAM_STARTUP_MS;
  printf("AI_CAM_PIPE:\tpersistent %s started (signal mode)\n", CATIA_AI_CAM_COMMAND);
  return 0;
}

/** Discard inotify events queued before this shot's signal. */
static void persistent_drain_events(void)
{
  char buffer[4096];
  while (read(watch_fd, buffer, sizeof(buffer)) > 0) {
  }
}

/** One capture through the persistent process, published as `filename`. */
static int persistent_shoot(const char *filename)
{
  if (!persistent_alive() || watch_fd < 0) {
    return -1;
  }
  int64_t now = monotonic_ms();
  if (now < signal_ready_ms) {
    usleep((useconds_t)(signal_ready_ms - now) * 1000);
  }
  persistent_drain_events();
  if (kill(signal_pid, SIGUSR1) != 0) {
    return -1;
  }

  int64_t deadline = monotonic_ms() + CATIA_AI_CAM_SIGNAL_TIMEOUT_MS;
  char buffer[4096] __attribute__((aligned(__alignof__(struct inotify_event))));
  while (monotonic_ms() < deadline) {
    if (!persistent_alive()) {
      return -1;
    }
    struct pollfd watch = {.fd = watch_fd, .events = POLLIN};
    if (poll(&watch, 1, 50) <= 0) {
      continue;
    }
    ssize_t length = read(watch_fd, buffer, sizeof(buffer));
    for (char *cursor = buffer; length > 0 && cursor < buffer + length;) {
      const struct inotify_event *event = (const struct inotify_event *)cursor;
      cursor += sizeof(struct inotify_event) + event->len;
      if (event->len == 0 || strncmp(event->name, AI_CAM_SIGNAL_PREFIX, strlen(AI_CAM_SIGNAL_PREFIX)) != 0) {
        continue;
      }
      char captured[PATH_MAX];
      if (snprintf(captured, sizeof(captured), "%s/%s", signal_dir, event->name) >= (int)sizeof(captured)) {
        continue;
      }
      struct stat image_status;
      if (stat(captured, &image_status) != 0 || !S_ISREG(image_status.st_mode)
          || image_status.st_size == 0) {
        continue;
      }
      if (rename(captured, filename) != 0) {
        fprintf(stderr, "AI_CAM_PIPE:\tfailed to rename %s: %s\n", captured, strerror(errno));
        unlink(captured);
        return -1;
      }
      return 0;
    }
  }
  fprintf(stderr, "AI_CAM_PIPE:\tno image %d ms after signalling %s\n",
          CATIA_AI_CAM_SIGNAL_TIMEOUT_MS, CATIA_AI_CAM_COMMAND);
  return -1;
}

int ai_cam_pipe_init(const char *unused)
{
  (void)unused;
  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(photo_directory, resolved_dir, sizeof(resolved_dir));

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
  if (persistent_enabled()) {
    persistent_start(photo_dir);
  }
  return 0;
}

/** One rpicam-still process for this shot only. */
static int oneshot_shoot(char *filename)
{
  char *const arguments[] = {
    (char *)CATIA_AI_CAM_COMMAND,
    (char *)"-o", filename,
    (char *)"--width", (char *)CATIA_AI_CAM_WIDTH,
    (char *)"--height", (char *)CATIA_AI_CAM_HEIGHT,
    (char *)"-t", (char *)"200",
    NULL
  };
  pid_t camera_pid;
  int spawn_result = posix_spawnp(&camera_pid, CATIA_AI_CAM_COMMAND, NULL, NULL, arguments, environ);
  if (spawn_result != 0) {
    fprintf(stderr, "AI_CAM_PIPE:\tfailed to start %s: %s\n",
            CATIA_AI_CAM_COMMAND, strerror(spawn_result));
    return -1;
  }

  int camera_status;
  /** A hung rpicam-still must not block this worker thread (and, once all worker slots
   * fill with the same hang, the entire capture pipeline) forever: wait with a bounded
   * deadline and kill it if exceeded, so the shot fails fast and the next one can retry. */
  int64_t deadline = monotonic_ms() + (int64_t)CATIA_AI_CAM_TIMEOUT_SECONDS * 1000;
  bool killed = false;
  for (;;) {
    pid_t wait_result = waitpid(camera_pid, &camera_status, WNOHANG);
    if (wait_result == camera_pid) {
      break;
    }
    if (wait_result < 0 && errno != EINTR) {
      fprintf(stderr, "AI_CAM_PIPE:\tfailed to wait for %s: %s\n",
              CATIA_AI_CAM_COMMAND, strerror(errno));
      return -1;
    }
    if (!killed && monotonic_ms() >= deadline) {
      fprintf(stderr, "AI_CAM_PIPE:\t%s did not exit within %d s; killing it\n",
              CATIA_AI_CAM_COMMAND, CATIA_AI_CAM_TIMEOUT_SECONDS);
      kill(camera_pid, SIGKILL);
      killed = true;
    }
  }
  if (killed) {
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
    return -1;
  }
  return 0;
}

int ai_cam_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(photo_directory, resolved_dir, sizeof(resolved_dir));

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

  bool persistent = persistent_enabled();
  int result = -1;
  if (persistent) {
    if (signal_pid <= 0) {
      persistent_start(photo_dir);
    }
    result = persistent_shoot(filename);
    if (result != 0) {
      fprintf(stderr, "AI_CAM_PIPE:\tshot %d falls back to one %s per shot\n",
              image_number, CATIA_AI_CAM_COMMAND);
      persistent_stop();
    }
  }
  if (result != 0) {
    result = oneshot_shoot(filename);
    if (persistent) {
      /* warm up again now, so the next shot does not pay the startup */
      persistent_start(photo_dir);
    }
  }

  struct stat image_status;
  if (result != 0 || stat(filename, &image_status) != 0 || !S_ISREG(image_status.st_mode)
      || image_status.st_size == 0) {
    if (result == 0) {
      fprintf(stderr, "AI_CAM_PIPE:\t%s did not create a valid image at %s\n",
              CATIA_AI_CAM_COMMAND, filename);
    }
    unlink(filename);
    filename[0] = '\0';
    return -1;
  }
  return 0;
}

/** @brief Stop the persistent rpicam-still, if any. */
void ai_cam_pipe_deinit(void)
{
  persistent_stop();
}
