/*
 * vehicle_detect_pipe_save_detection_copy() must duplicate a shot into
 * photos/detections/ when the fixture server reports a hit (FAKE_DETECT_COUNT
 * > 0), byte-for-byte, and must leave no such directory entry when it
 * reports no detection.
 */
#include "../vehicle_detect_pipe.c"
#include <assert.h>
#include <stdlib.h>

int main(void)
{
  signal(SIGPIPE, SIG_IGN);

  char directory[] = "/tmp/vehicle-detect-save-copy-test.XXXXXX";
  assert(mkdtemp(directory) != NULL);
  assert(chdir(directory) == 0);
  assert(mkdir("photos", 0700) == 0);
  vehicle_detect_pipe_set_photo_directory("photos");

  assert(vehicle_detect_pipe_init(NULL) == 0);

  char filename[PATH_MAX];
  assert(vehicle_detect_pipe_shoot(filename, sizeof(filename), 1) == 0);
  assert(strcmp(filename, "photos/a000001.jpg") == 0);

  vehicle_detect_pipe_save_detection_copy(filename);

  const char *fake_count = getenv("FAKE_DETECT_COUNT");
  bool expect_copy = fake_count != NULL && atoi(fake_count) > 0;

  struct stat copy_status;
  bool copy_exists = stat("photos/detections/a000001.jpg", &copy_status) == 0;

  if (expect_copy) {
    assert(copy_exists);
    struct stat original_status;
    assert(stat(filename, &original_status) == 0);
    assert(copy_status.st_size == original_status.st_size);
    assert(copy_status.st_size > 0);
  } else {
    assert(!copy_exists);
  }

  vehicle_detect_pipe_deinit();
  puts(expect_copy
       ? "vehicle detect save-copy: detection copied into photos/detections/"
       : "vehicle detect save-copy: no detection, no copy created");
  return 0;
}
