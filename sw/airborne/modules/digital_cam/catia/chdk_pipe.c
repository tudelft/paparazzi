/* execill - How a parent and child might communicate. */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

#include "chdk_pipe.h"

#define READ 0
#define WRITE 1
#define MAX_FILENAME 255
#define SHELL "/root/develop/allthings_obc2014/src/popcorn/popcorn.sh"


const char *setup =
  "lua props=require(\"propcase\");print(\"SetupScript\");set_prop(props.ISO_MODE,3200);set_prop(props.FLASH_MODE,2);set_prop(props.RESOLUTION,0);set_prop(props.DATE_STAMP,0);set_prop(props.AF_ASSIST_BEAM,0);set_prop(props.QUALITY,0);print(\"Ready\");\n";

static int fo, fi;
static int write_command(const char *command, size_t length);
static int make_deadline(struct timespec *deadline, int timeout_seconds);
static int milliseconds_until(const struct timespec *deadline);
static int read_character(char *character, const struct timespec *deadline);
static int wait_for_cmd(int timeout_seconds);
static int wait_for_img(char *filename, int timeout_seconds);
static pid_t popen2(const char *command, int *infp, int *outfp);

/*void main(int argc, char ** argv, char ** envp)
{
  int i;
  char filename[MAX_FILENAME];

  // Initialize chdk pipe
  chdk_pipe_init();

  // Start taking photos
  for(i=0; i < 3; i++) {
    chdk_pipe_shoot(filename);
    printf("Shot image: %s\n", filename);
  }

  // Initialize chdk pipe
  chdk_pipe_deinit();
}*/

/**
 * Initialize the CHDK pipe
 */
void chdk_pipe_init(void)
{
  /* Check if SHELL is started */
  if (popen2(SHELL, &fi, &fo) <= 0) {
    perror("Can't start SHELL");
    exit(1);
  }
  wait_for_cmd(10);

  /* Connect to the camera */
  if (write_command("connect\n", sizeof("connect\n") - 1) != 0 || wait_for_cmd(10) != 0) {
    fprintf(stderr, "CHDK_PIPE:\tfailed to connect to camera\n");
    exit(1);
  }

  /* Kill all running scripts */
  //write(fi, "killscript\n", 11);
  //wait_for_cmd(10);

  /* Start recording mode */
  if (write_command("rec\n", sizeof("rec\n") - 1) != 0 || wait_for_cmd(10) != 0) {
    fprintf(stderr, "CHDK_PIPE:\tfailed to enter record mode\n");
    exit(1);
  }

  /* Start rsint mode */
  if (write_command(setup, strlen(setup)) != 0 || wait_for_cmd(10) != 0) {
    fprintf(stderr, "CHDK_PIPE:\tfailed to configure camera\n");
    exit(1);
  }
}

/**
 * Deinitialize CHDK pipe
 */
void chdk_pipe_deinit(void)
{
  /* Stop rsint mode */
  //write(fi, "q\n", 2);
  //wait_for_cmd(10);

  /* Quit SHELL */
  if (write_command("quit\n", sizeof("quit\n") - 1) != 0) {
    fprintf(stderr, "CHDK_PIPE:\tfailed to stop camera shell\n");
  }
}

/**
 * Shoot an image
 */
void chdk_pipe_shoot(char *filename)
{
  filename[0] = '\0';
  if (write_command("rs /root\n", sizeof("rs /root\n") - 1) != 0
      || wait_for_img(filename, 10) != 0) {
    fprintf(stderr, "CHDK_PIPE:\timage capture timed out or failed\n");
    filename[0] = '\0';
  }
}

static int write_command(const char *command, size_t length)
{
  size_t written = 0;
  while (written < length) {
    ssize_t result = write(fi, &command[written], length - written);
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result <= 0) {
      return -1;
    }
    written += (size_t)result;
  }
  return 0;
}

static int make_deadline(struct timespec *deadline, int timeout_seconds)
{
  if (clock_gettime(CLOCK_MONOTONIC, deadline) != 0) {
    return -1;
  }
  deadline->tv_sec += timeout_seconds;
  return 0;
}

static int milliseconds_until(const struct timespec *deadline)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    return -1;
  }
  int64_t seconds = (int64_t)deadline->tv_sec - (int64_t)now.tv_sec;
  int64_t nanoseconds = (int64_t)deadline->tv_nsec - (int64_t)now.tv_nsec;
  int64_t milliseconds = seconds * 1000 + nanoseconds / 1000000;
  if (milliseconds <= 0) {
    return 0;
  }
  return milliseconds > INT_MAX ? INT_MAX : (int)milliseconds;
}

static int read_character(char *character, const struct timespec *deadline)
{
  struct pollfd descriptor = {.fd = fo, .events = POLLIN, .revents = 0};
  int result;
  do {
    int timeout = milliseconds_until(deadline);
    if (timeout <= 0) {
      return -1;
    }
    result = poll(&descriptor, 1, timeout);
  } while (result < 0 && errno == EINTR);
  if (result <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    return -1;
  }
  do {
    result = (int)read(fo, character, 1);
  } while (result < 0 && errno == EINTR);
  return result == 1 ? 0 : -1;
}

static int wait_for_img(char *filename, int timeout_seconds)
{
  int hash_cnt = 0;
  char ch;
  int filename_idx = 0;
  struct timespec deadline;
  if (make_deadline(&deadline, timeout_seconds) != 0) {
    return -1;
  }

  while (hash_cnt < 4) {
    if (read_character(&ch, &deadline) != 0) {
      return -1;
    }
    if (ch == '#') {
      hash_cnt++;
    } else if (hash_cnt >= 2 && filename_idx < MAX_FILENAME - 1) {
      filename[filename_idx++] = ch;
    }
  }

  filename[filename_idx] = 0;
  do {
    if (read_character(&ch, &deadline) != 0) {
      return -1;
    }
  } while (ch != '>');
  return 0;
}

static int wait_for_cmd(int timeout_seconds)
{
  char ch;
  struct timespec deadline;
  if (make_deadline(&deadline, timeout_seconds) != 0) {
    return -1;
  }
  do {
    if (read_character(&ch, &deadline) != 0) {
      return -1;
    }
  } while (ch != '>');
  return 0;
}

/**
 * Open a process with stdin and stdout
 */
static pid_t popen2(const char *command, int *infp, int *outfp)
{
  int p_stdin[2], p_stdout[2];
  pid_t pid;

  if (pipe(p_stdin) != 0 || pipe(p_stdout) != 0) {
    return -1;
  }

  pid = fork();

  if (pid < 0) {
    return pid;
  } else if (pid == 0) {
    close(p_stdin[WRITE]);
    dup2(p_stdin[READ], READ);
    close(p_stdout[READ]);
    dup2(p_stdout[WRITE], WRITE);

    execl("/bin/sh", "sh", "-c", command, NULL);
    perror("execl");
    exit(1);
  }

  if (infp == NULL) {
    close(p_stdin[WRITE]);
  } else {
    *infp = p_stdin[WRITE];
  }

  if (outfp == NULL) {
    close(p_stdout[READ]);
  } else {
    *outfp = p_stdout[READ];
  }

  return pid;
}
