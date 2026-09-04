#include "local_pipe.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef CATIA_LOCAL_PHOTO_DIR
#define CATIA_LOCAL_PHOTO_DIR "photos"
#endif

#ifndef CATIA_MOCK_IMAGE
#define CATIA_MOCK_IMAGE "mock-camera.jpg"
#endif

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

static const char *source_image;
static char test_photo_directory[PATH_MAX];

static int prepare_photo_directory(void);
static int find_test_photo_directory(char *directory, size_t directory_size);
static int select_random_test_photo(char *path, size_t path_size);
static bool is_readable_jpg(DIR *directory, const struct dirent *entry);
static int random_index(size_t count, size_t *index);

int local_pipe_init(const char *mock_image)
{
  if (mock_image == NULL || access(mock_image, R_OK) != 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to access mock image %s: %s\n",
            mock_image != NULL ? mock_image : "(null)", strerror(errno));
    return -1;
  }

  if (prepare_photo_directory() != 0) {
    return -1;
  }

  source_image = mock_image;
  test_photo_directory[0] = '\0';
  printf("LOCAL_PIPE:\tphoto directory: %s\n", CATIA_LOCAL_PHOTO_DIR);
  return 0;
}

int local_pipe_test_init(const char *mock_image)
{
  if (mock_image != NULL) {
    return local_pipe_init(mock_image);
  }

  if (prepare_photo_directory() != 0) {
    return -1;
  }

  int photo_count = find_test_photo_directory(test_photo_directory, sizeof(test_photo_directory));
  if (photo_count < 0) {
    return -1;
  }
  if (photo_count == 0) {
    fprintf(stderr, "LOCAL_PIPE:\tno readable .jpg files found beside the executable; using %s\n",
                CATIA_MOCK_IMAGE);
              return local_pipe_init(CATIA_MOCK_IMAGE);
  }

  source_image = NULL;
  printf("LOCAL_PIPE:\ttest photo directory: %s (%d image%s)\n",
         test_photo_directory, photo_count, photo_count == 1 ? "" : "s");
  printf("LOCAL_PIPE:\tphoto directory: %s\n", CATIA_LOCAL_PHOTO_DIR);
  return 0;
}

static int prepare_photo_directory(void)
{
  if (mkdir(CATIA_LOCAL_PHOTO_DIR, 0755) != 0 && errno != EEXIST) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to create photo directory %s: %s\n",
            CATIA_LOCAL_PHOTO_DIR, strerror(errno));
    return -1;
  }

  struct stat directory_status;
  if (stat(CATIA_LOCAL_PHOTO_DIR, &directory_status) != 0 || !S_ISDIR(directory_status.st_mode)
      || access(CATIA_LOCAL_PHOTO_DIR, W_OK) != 0) {
    fprintf(stderr, "LOCAL_PIPE:\tphoto directory is not writable: %s\n", CATIA_LOCAL_PHOTO_DIR);
    return -1;
  }
  return 0;
}

int local_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  if ((source_image == NULL && test_photo_directory[0] == '\0')
      || filename == NULL || filename_size == 0) {
    return -1;
  }

  char selected_source[PATH_MAX];
  const char *current_source = source_image;
  if (test_photo_directory[0] != '\0') {
    if (select_random_test_photo(selected_source, sizeof(selected_source)) != 0) {
      return -1;
    }
    current_source = selected_source;
    printf("LOCAL_PIPE:\tselected test image: %s\n", current_source);
  }

  int length = snprintf(filename, filename_size, "%s/m%06d.jpg",
                        CATIA_LOCAL_PHOTO_DIR, image_number);
  if (length < 0 || (size_t)length >= filename_size) {
    return -1;
  }

  int source_fd = open(current_source, O_RDONLY);
  if (source_fd < 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to open mock image %s: %s\n", current_source, strerror(errno));
    return -1;
  }

  int destination_fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (destination_fd < 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to create %s: %s\n", filename, strerror(errno));
    close(source_fd);
    return -1;
  }

  char buffer[4096];
  ssize_t bytes_read;
  int result = 0;
  while ((bytes_read = read(source_fd, buffer, sizeof(buffer))) > 0) {
    ssize_t written = 0;
    while (written < bytes_read) {
      ssize_t count = write(destination_fd, &buffer[written], (size_t)(bytes_read - written));
      if (count <= 0) {
        result = -1;
        break;
      }
      written += count;
    }
    if (result != 0) {
      break;
    }
  }
  if (bytes_read < 0) {
    result = -1;
  }

  close(destination_fd);
  close(source_fd);
  return result;
}

void local_pipe_deinit(void)
{
  source_image = NULL;
  test_photo_directory[0] = '\0';
}

static int find_test_photo_directory(char *directory, size_t directory_size)
{
  char executable[PATH_MAX];
  ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
  if (length < 0 || (size_t)length >= sizeof(executable) - 1) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to locate CATIA executable: %s\n", strerror(errno));
    return -1;
  }
  executable[length] = '\0';

  char *separator = strrchr(executable, '/');
  if (separator == NULL) {
    fprintf(stderr, "LOCAL_PIPE:\tCATIA executable path has no directory: %s\n", executable);
    return -1;
  }
  if (separator == executable) {
    separator[1] = '\0';
  } else {
    *separator = '\0';
  }

  int path_length = snprintf(directory, directory_size, "%s%stestphotos",
                             executable, separator == executable ? "" : "/");
  if (path_length < 0 || (size_t)path_length >= directory_size) {
    fprintf(stderr, "LOCAL_PIPE:\ttest photo directory path is too long\n");
    return -1;
  }

  DIR *stream = opendir(directory);
  if (stream == NULL) {
    if (errno != ENOENT && errno != ENOTDIR) {
      fprintf(stderr, "LOCAL_PIPE:\tfailed to open test photo directory %s: %s\n",
              directory, strerror(errno));
      return -1;
    }
    directory[0] = '\0';
    return 0;
  }

  int count = 0;
  struct dirent *entry;
  while ((entry = readdir(stream)) != NULL) {
    if (is_readable_jpg(stream, entry)) {
      count++;
    }
  }
  closedir(stream);
  return count;
}

static int select_random_test_photo(char *path, size_t path_size)
{
  DIR *stream = opendir(test_photo_directory);
  if (stream == NULL) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to open test photo directory %s: %s\n",
            test_photo_directory, strerror(errno));
    return -1;
  }

  size_t count = 0;
  struct dirent *entry;
  while ((entry = readdir(stream)) != NULL) {
    if (is_readable_jpg(stream, entry)) {
      count++;
    }
  }
  if (count == 0) {
    fprintf(stderr, "LOCAL_PIPE:\ttest photo directory contains no readable .jpg files\n");
    closedir(stream);
    return -1;
  }

  size_t selected_index;
  if (random_index(count, &selected_index) != 0) {
    closedir(stream);
    return -1;
  }

  rewinddir(stream);
  size_t current_index = 0;
  while ((entry = readdir(stream)) != NULL) {
    if (!is_readable_jpg(stream, entry)) {
      continue;
    }
    if (current_index == selected_index) {
      int length = snprintf(path, path_size, "%s/%s", test_photo_directory, entry->d_name);
      closedir(stream);
      if (length < 0 || (size_t)length >= path_size) {
        fprintf(stderr, "LOCAL_PIPE:\tselected test image path is too long\n");
        return -1;
      }
      return 0;
    }
    current_index++;
  }

  closedir(stream);
  fprintf(stderr, "LOCAL_PIPE:\ttest photo directory changed during image selection\n");
  return -1;
}

static bool is_readable_jpg(DIR *directory, const struct dirent *entry)
{
  size_t name_length = strlen(entry->d_name);
  if (name_length < 4 || strcasecmp(&entry->d_name[name_length - 4], ".jpg") != 0) {
    return false;
  }

  struct stat status;
  return fstatat(dirfd(directory), entry->d_name, &status, 0) == 0
         && S_ISREG(status.st_mode)
         && faccessat(dirfd(directory), entry->d_name, R_OK, 0) == 0;
}

static int random_index(size_t count, size_t *index)
{
  int random_fd = open("/dev/urandom", O_RDONLY);
  if (random_fd < 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to open /dev/urandom: %s\n", strerror(errno));
    return -1;
  }

  uint64_t random_value;
  unsigned char *bytes = (unsigned char *)&random_value;
  size_t bytes_read = 0;
  while (bytes_read < sizeof(random_value)) {
    ssize_t count_read = read(random_fd, &bytes[bytes_read], sizeof(random_value) - bytes_read);
    if (count_read < 0 && errno == EINTR) {
      continue;
    }
    if (count_read <= 0) {
      fprintf(stderr, "LOCAL_PIPE:\tfailed to read /dev/urandom: %s\n", strerror(errno));
      close(random_fd);
      return -1;
    }
    bytes_read += (size_t)count_read;
  }
  close(random_fd);

  *index = (size_t)(random_value % (uint64_t)count);
  return 0;
}