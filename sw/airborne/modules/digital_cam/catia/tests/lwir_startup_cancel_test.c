#include "../lwir_cam_pipe.c"

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>

static volatile sig_atomic_t keep_test_running = 1;

static uint64_t elapsed_ms(const struct timespec *start, const struct timespec *end)
{
  int64_t milliseconds = (int64_t)(end->tv_sec - start->tv_sec) * 1000
                         + (end->tv_nsec - start->tv_nsec) / 1000000;
  assert(milliseconds >= 0);
  return (uint64_t)milliseconds;
}

static void stop_startup(int signal_number)
{
  (void)signal_number;
  keep_test_running = 0;
}

static void *cancel_startup(void *argument)
{
  usleep(100000);
  assert(pthread_kill(*(pthread_t *)argument, SIGUSR2) == 0);
  return NULL;
}

struct signal_storm {
  pthread_t target;
  atomic_bool active;
};

static void ignore_signal(int signal_number)
{
  (void)signal_number;
}

static void *interrupt_startup(void *argument)
{
  struct signal_storm *storm = argument;
  for (unsigned signal_count = 0;
       signal_count < 1500 && atomic_load(&storm->active);
       ++signal_count) {
    pthread_kill(storm->target, SIGUSR1);
    usleep(1000);
  }
  atomic_store(&storm->active, false);
  return NULL;
}

int main(void)
{
  char directory[] = "/tmp/lwir-startup-cancel.XXXXXX";
  assert(mkdtemp(directory) != NULL);
  lwir_cam_pipe_set_photo_directory(directory);

  signal(SIGUSR2, stop_startup);
  pthread_t startup_thread = pthread_self();
  pthread_t cancel_thread;
  assert(pthread_create(&cancel_thread, NULL, cancel_startup, &startup_thread) == 0);
  struct timespec started, finished;
  assert(clock_gettime(CLOCK_MONOTONIC, &started) == 0);
  assert(lwir_cam_pipe_init_cancelable(NULL, &keep_test_running) != 0);
  assert(clock_gettime(CLOCK_MONOTONIC, &finished) == 0);
  assert(pthread_join(cancel_thread, NULL) == 0);
  assert(elapsed_ms(&started, &finished) < 1000);
  assert(capture_server_pid == -1);

  signal(SIGUSR1, ignore_signal);
  struct signal_storm storm = {.target = pthread_self(), .active = ATOMIC_VAR_INIT(true)};
  pthread_t signal_thread;
  int descriptors[2];
  assert(pipe(descriptors) == 0);
  capture_server_output = descriptors[0];
  assert(pthread_create(&signal_thread, NULL, interrupt_startup, &storm) == 0);
  assert(clock_gettime(CLOCK_MONOTONIC, &started) == 0);
  assert(read_server_status("LWIR_SERVER_READY", 150, NULL) != 0);
  assert(clock_gettime(CLOCK_MONOTONIC, &finished) == 0);
  atomic_store(&storm.active, false);
  assert(pthread_join(signal_thread, NULL) == 0);
  assert(elapsed_ms(&started, &finished) < 2000);
  close(descriptors[0]);
  close(descriptors[1]);
  capture_server_output = -1;
  puts("LWIR startup: shutdown cancellation and EINTR-safe absolute deadline passed");
  return 0;
}