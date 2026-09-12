/*
 * A hung rpicam-still (e.g. stuck camera driver) must not block the calling
 * worker thread forever: ai_cam_pipe_shoot() must kill it after a bounded
 * timeout, reap it (no zombie left behind), and fail that one shot fast.
 */
#include "../ai_cam_pipe.h"
#include <assert.h>
#include <errno.h>
#include <spawn.h>
#include <stdio.h>
#include <sys/wait.h>
#include <unistd.h>

static pid_t hung_pid = -1;

int posix_spawnp(pid_t *pid, const char *file,
                 const posix_spawn_file_actions_t *actions,
                 const posix_spawnattr_t *attributes,
                 char *const arguments[], char *const environment[])
{
  (void)file;
  (void)actions;
  (void)attributes;
  (void)environment;
  (void)arguments;
  *pid = fork();
  assert(*pid >= 0);
  if (*pid == 0) {
    for (;;) pause();
  }
  hung_pid = *pid;
  return 0;
}

int main(void)
{
  assert(ai_cam_pipe_init(NULL) == 0);
  char filename[128];
  assert(ai_cam_pipe_shoot(filename, sizeof(filename), 1) == -1);
  assert(filename[0] == '\0');
  assert(hung_pid > 0);
  assert(waitpid(hung_pid, NULL, WNOHANG) == -1 && errno == ECHILD);
  ai_cam_pipe_deinit();
  puts("AIcam capture timeout: a hung rpicam-still is killed, reaped and the shot fails fast");
  return 0;
}
