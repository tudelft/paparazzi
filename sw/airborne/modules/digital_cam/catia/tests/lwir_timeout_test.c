#include "../lwir_cam_pipe.c"

#include <assert.h>
#include <stdlib.h>

static uint64_t elapsed_ms(const struct timespec *start, const struct timespec *end)
{
  int64_t milliseconds = (int64_t)(end->tv_sec - start->tv_sec) * 1000
                         + (end->tv_nsec - start->tv_nsec) / 1000000;
  assert(milliseconds >= 0);
  return (uint64_t)milliseconds;
}

int main(void)
{
  signal(SIGPIPE, SIG_IGN);
  char directory[] = "/tmp/lwir-timeout-test.XXXXXX";
  assert(mkdtemp(directory) != NULL);
  lwir_cam_pipe_set_photo_directory(directory);
  assert(lwir_cam_pipe_init(NULL) == 0);

  struct timespec started, finished;
  assert(clock_gettime(CLOCK_MONOTONIC, &started) == 0);
  char filename[PATH_MAX];
  assert(lwir_cam_pipe_shoot(filename, sizeof(filename), 1) != 0);
  assert(clock_gettime(CLOCK_MONOTONIC, &finished) == 0);
  assert(filename[0] == '\0');
  uint64_t duration_ms = elapsed_ms(&started, &finished);
  fprintf(stderr, "LWIR timeout duration: %llu ms\n", (unsigned long long)duration_ms);
  assert(duration_ms < 1500);
  assert(capture_server_pid == -1);
  puts("LWIR timeout: noisy unresponsive server is killed within the absolute deadline");
  return 0;
}