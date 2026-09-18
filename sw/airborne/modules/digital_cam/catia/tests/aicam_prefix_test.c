#include "../ai_cam_pipe.h"
#include <assert.h>
#include <spawn.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int posix_spawnp(pid_t *pid, const char *file,
                 const posix_spawn_file_actions_t *actions,
                 const posix_spawnattr_t *attributes,
                 char *const arguments[], char *const environment[])
{
  (void)actions;
  (void)attributes;
  (void)environment;
  assert(strcmp(file, "rpicam-still") == 0);
  assert(strcmp(arguments[1], "-o") == 0);
  assert(strcmp(arguments[2], "photos/a000042.jpg") == 0);
  assert(strcmp(arguments[3], "--width") == 0);
  assert(strcmp(arguments[4], "2048") == 0);
  assert(strcmp(arguments[5], "--height") == 0);
  assert(strcmp(arguments[6], "1520") == 0);
  FILE *image = fopen(arguments[2], "wb");
  assert(image != NULL);
  assert(fputs("capture fixture", image) >= 0);
  assert(fclose(image) == 0);
  *pid = fork();
  assert(*pid >= 0);
  if (*pid == 0) _exit(0);
  return 0;
}

int main(void)
{
  assert(ai_cam_pipe_init(NULL) == 0);
  char filename[128];
  assert(ai_cam_pipe_shoot(filename, sizeof(filename), 42) == 0);
  assert(strcmp(filename, "photos/a000042.jpg") == 0);
  ai_cam_pipe_deinit();
  puts("AIcam real-backend filename prefix and capture arguments passed");
  return 0;
}