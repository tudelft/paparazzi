/*
 * A persistent LWIR capture server that dies right after answering one
 * request (see tests/fixtures/dying_lwir_server.sh) must not cost the next
 * shot: lwir_cam_pipe_shoot() should detect the broken connection and return
 * failure so CATIA's asynchronous supervisor can own the restart policy.
 */
#include "../lwir_cam_pipe.c"
#include <assert.h>
#include <stdlib.h>

int main(void)
{
  signal(SIGPIPE, SIG_IGN);

  char directory[] = "/tmp/lwir-shoot-recovery-test.XXXXXX";
  assert(mkdtemp(directory) != NULL);
  assert(chdir(directory) == 0);
  assert(mkdir("speedtest-output", 0700) == 0);
  lwir_cam_pipe_set_photo_directory("speedtest-output");

  char counter_path[PATH_MAX];
  int written = snprintf(counter_path, sizeof(counter_path), "%s/counter", directory);
  assert(written > 0 && (size_t)written < sizeof(counter_path));
  assert(setenv("FAKE_SERVER_COUNTER", counter_path, 1) == 0);

  char filename[PATH_MAX];

  assert(lwir_cam_pipe_init(NULL) == 0);
  assert(lwir_cam_pipe_shoot(filename, sizeof(filename), 1) == 0);
  assert(strcmp(filename, "speedtest-output/l000001.jpg") == 0);
  assert(access(filename, R_OK) == 0);

  /* The server that answered shot 1 has exited. The request fails and clears
   * stale state; an explicit supervisor restart restores the following shot. */
  assert(lwir_cam_pipe_shoot(filename, sizeof(filename), 2) != 0);
  assert(filename[0] == '\0');
  assert(lwir_cam_pipe_init(NULL) == 0);
  assert(lwir_cam_pipe_shoot(filename, sizeof(filename), 3) == 0);
  assert(strcmp(filename, "speedtest-output/l000003.jpg") == 0);
  assert(access(filename, R_OK) == 0);

  FILE *counter = fopen(counter_path, "r");
  assert(counter != NULL);
  int spawned = 0;
  assert(fscanf(counter, "%d", &spawned) == 1);
  assert(fclose(counter) == 0);
  assert(spawned == 2);

  lwir_cam_pipe_deinit();
  puts("LWIR shoot recovery: a dead server is cleared and explicit supervisor restart succeeds");
  return 0;
}
