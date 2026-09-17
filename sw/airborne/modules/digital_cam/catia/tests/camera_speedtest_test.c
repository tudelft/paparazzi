#include "../camera_speedtest.h"

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t running = 1;
static int capture_count;
static int first_capture_number;
static int fail_capture = -1;
static int interrupt_capture = -1;
static bool create_empty_image;

static int fake_shoot(char *filename, size_t filename_size, int image_number)
{
  if (capture_count == 0) first_capture_number = image_number;
  assert(image_number == first_capture_number + capture_count);
  ++capture_count;

  struct timespec delay = {.tv_sec = 0, .tv_nsec = 1000000};
  assert(nanosleep(&delay, NULL) == 0);
  if (capture_count == interrupt_capture) running = 0;
  if (capture_count == fail_capture) return -1;

  int length = snprintf(filename, filename_size, "photos/a%06d.jpg", image_number);
  assert(length > 0 && (size_t)length < filename_size);
  FILE *image = fopen(filename, "wb");
  assert(image != NULL);
  if (!create_empty_image) assert(fputc(0x42, image) == 0x42);
  assert(fclose(image) == 0);
  return 0;
}

static struct camera_speedtest_config config(void)
{
  return (struct camera_speedtest_config) {
    .backend_name = "fake-aicam",
    .photo_directory = "photos",
    .filename_prefix = 'a',
    .shoot = fake_shoot,
    .keep_running = &running
  };
}

static void reset_capture(void)
{
  running = 1;
  capture_count = 0;
  first_capture_number = -1;
  fail_capture = -1;
  interrupt_capture = -1;
  create_empty_image = false;
}

static void remove_images(int first_image)
{
  for (int offset = 0; offset <= CAMERA_SPEEDTEST_SAMPLE_COUNT; ++offset) {
    char filename[64];
    int length = snprintf(filename, sizeof(filename), "photos/a%06d.jpg",
                          first_image + offset);
    assert(length > 0 && (size_t)length < sizeof(filename));
    if (unlink(filename) != 0) assert(errno == ENOENT);
  }
}

int main(void)
{
  assert(mkdir("photos", 0700) == 0);
  char isolated_directory[512];
  assert(camera_speedtest_create_output_directory("photos", isolated_directory,
                                                  sizeof(isolated_directory)) == 0);
  struct stat isolated_status;
  assert(stat(isolated_directory, &isolated_status) == 0);
  assert(S_ISDIR(isolated_status.st_mode));
  assert((isolated_status.st_mode & 0777) == 0700);
  assert(rmdir(isolated_directory) == 0);
  struct camera_speedtest_config speedtest = config();

  FILE *collision = fopen("photos/a900002.jpg", "wb");
  assert(collision != NULL);
  assert(fputs("mission evidence", collision) >= 0);
  assert(fclose(collision) == 0);
  int first_image = -1;
  assert(camera_speedtest_find_image_block(&speedtest, &first_image) == 0);
  assert(first_image == 900003);

  reset_capture();
  assert(camera_speedtest_run(&speedtest, first_image) == 0);
  assert(capture_count == CAMERA_SPEEDTEST_SAMPLE_COUNT + 1);
  collision = fopen("photos/a900002.jpg", "rb");
  assert(collision != NULL);
  char evidence[32];
  assert(fgets(evidence, sizeof(evidence), collision) != NULL);
  assert(strcmp(evidence, "mission evidence") == 0);
  assert(fclose(collision) == 0);
  remove_images(first_image);

  reset_capture();
  fail_capture = 4;
  assert(camera_speedtest_run(&speedtest, first_image) == -1);
  assert(capture_count == 4);
  remove_images(first_image);

  reset_capture();
  interrupt_capture = 2;
  assert(camera_speedtest_run(&speedtest, first_image) == -1);
  assert(capture_count == 2);
  remove_images(first_image);

  reset_capture();
  interrupt_capture = CAMERA_SPEEDTEST_SAMPLE_COUNT + 1;
  assert(camera_speedtest_run(&speedtest, first_image) == -1);
  assert(capture_count == CAMERA_SPEEDTEST_SAMPLE_COUNT + 1);
  remove_images(first_image);

  reset_capture();
  create_empty_image = true;
  assert(camera_speedtest_run(&speedtest, first_image) == -1);
  assert(capture_count == 1);
  remove_images(first_image);

  assert(unlink("photos/a900002.jpg") == 0);
  assert(rmdir("photos") == 0);
  puts("camera speedtest tests passed");
  return 0;
}