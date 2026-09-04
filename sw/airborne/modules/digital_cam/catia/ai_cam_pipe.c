#include "ai_cam_pipe.h"

#include <errno.h>
#include <spawn.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
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

  if (mkdir(CATIA_AI_CAM_PHOTO_DIR, 0755) != 0 && errno != EEXIST) {
    fprintf(stderr, "AI_CAM_PIPE:\tfailed to create photo directory %s: %s\n",
            CATIA_AI_CAM_PHOTO_DIR, strerror(errno));
    return -1;
  }

  struct stat directory_status;
  if (stat(CATIA_AI_CAM_PHOTO_DIR, &directory_status) != 0 || !S_ISDIR(directory_status.st_mode)
      || access(CATIA_AI_CAM_PHOTO_DIR, W_OK) != 0) {
    fprintf(stderr, "AI_CAM_PIPE:\tphoto directory is not writable: %s\n", CATIA_AI_CAM_PHOTO_DIR);
    return -1;
  }

  printf("AI_CAM_PIPE:\tcapture command: %s\n", CATIA_AI_CAM_COMMAND);
  printf("AI_CAM_PIPE:\tphoto directory: %s\n", CATIA_AI_CAM_PHOTO_DIR);
  return 0;
}

int ai_cam_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  int length = snprintf(filename, filename_size, "%s/a%06d.jpg",
                        CATIA_AI_CAM_PHOTO_DIR, image_number);
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
  while (waitpid(camera_pid, &camera_status, 0) < 0) {
    if (errno != EINTR) {
      fprintf(stderr, "AI_CAM_PIPE:\tfailed to wait for %s: %s\n",
              CATIA_AI_CAM_COMMAND, strerror(errno));
      filename[0] = '\0';
      return -1;
    }
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