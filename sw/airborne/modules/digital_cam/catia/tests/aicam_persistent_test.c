/*
 * Persistent AI camera mode: one rpicam-still in signal mode serves every shot,
 * each JPEG is published under the shot's own name, and a dead signal-mode
 * process falls back to one rpicam-still per shot.
 */
#include "../ai_cam_pipe.h"
#include <assert.h>
#include <signal.h>
#include <stdbool.h>
#include <spawn.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

static int persistent_spawns;
static int oneshot_spawns;
static pid_t persistent_pid = -1;
static char pattern[256];
static volatile sig_atomic_t captures_requested;
static volatile sig_atomic_t quit_requested;

static void on_capture(int signal_number) { (void)signal_number; ++captures_requested; }
static void on_quit(int signal_number) { (void)signal_number; quit_requested = 1; }

/* Fake rpicam-still --signal: one JPEG per SIGUSR1, exit on SIGUSR2. */
static void fake_signal_mode(void)
{
  signal(SIGUSR1, on_capture);
  signal(SIGUSR2, on_quit);
  int written = 0;
  while (!quit_requested) {
    while (written < captures_requested) {
      char name[300];
      snprintf(name, sizeof(name), pattern, written++);
      FILE *image = fopen(name, "wb");
      fputs("jpeg", image);
      fclose(image);
    }
    usleep(1000);
  }
  _exit(0);
}

int posix_spawnp(pid_t *pid, const char *file,
                 const posix_spawn_file_actions_t *actions,
                 const posix_spawnattr_t *attributes,
                 char *const arguments[], char *const environment[])
{
  (void)actions;
  (void)attributes;
  (void)environment;
  assert(strcmp(file, "rpicam-still") == 0);
  bool signal_mode = false;
  for (int index = 1; arguments[index] != NULL; ++index) {
    if (strcmp(arguments[index], "--signal") == 0) signal_mode = true;
    if (strcmp(arguments[index], "-o") == 0) snprintf(pattern, sizeof(pattern), "%s", arguments[index + 1]);
  }
  if (signal_mode) {
    assert(strstr(pattern, "/.aicam_%06d.jpg") != NULL);
    ++persistent_spawns;
    *pid = fork();
    assert(*pid >= 0);
    if (*pid == 0) fake_signal_mode();
    persistent_pid = *pid;
    return 0;
  }
  ++oneshot_spawns;
  FILE *image = fopen(pattern, "wb");
  assert(image != NULL);
  fputs("jpeg", image);
  fclose(image);
  *pid = fork();
  assert(*pid >= 0);
  if (*pid == 0) _exit(0);
  return 0;
}

static void assert_image(const char *filename, const char *expected)
{
  struct stat status;
  assert(strcmp(filename, expected) == 0);
  assert(stat(filename, &status) == 0 && status.st_size > 0);
}

int main(void)
{
  char filename[128];
  assert(ai_cam_pipe_init(NULL) == 0);
  assert(persistent_spawns == 1);

  assert(ai_cam_pipe_shoot(filename, sizeof(filename), 1001) == 0);
  assert_image(filename, "photos/a001001.jpg");
  assert(ai_cam_pipe_shoot(filename, sizeof(filename), 1002) == 0);
  assert_image(filename, "photos/a001002.jpg");
  assert(persistent_spawns == 1 && oneshot_spawns == 0);

  /* signal-mode process dies: this shot is taken one-shot, then it is restarted */
  kill(persistent_pid, SIGKILL);
  waitpid(persistent_pid, NULL, 0);
  assert(ai_cam_pipe_shoot(filename, sizeof(filename), 1003) == 0);
  assert_image(filename, "photos/a001003.jpg");
  assert(oneshot_spawns == 1);
  assert(persistent_spawns == 2);

  assert(ai_cam_pipe_shoot(filename, sizeof(filename), 1004) == 0);
  assert_image(filename, "photos/a001004.jpg");
  assert(oneshot_spawns == 1);

  pid_t last = persistent_pid;
  ai_cam_pipe_deinit();
  assert(waitpid(last, NULL, WNOHANG) == -1);
  puts("AIcam persistent signal mode: shots renamed, dead process falls back and restarts");
  return 0;
}
