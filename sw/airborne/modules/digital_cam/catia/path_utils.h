#ifndef CATIA_PATH_UTILS_H
#define CATIA_PATH_UTILS_H

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

static inline const char *catia_resolve_path(const char *path, char *buffer, size_t buffer_size)
{
  if (path == NULL) {
    return NULL;
  }
  if (path[0] == '~' && (path[1] == '/' || path[1] == '\0')) {
    const char *home = getenv("HOME");
    if (home != NULL && home[0] != '\0') {
      int length = snprintf(buffer, buffer_size, "%s%s", home, path + 1);
      if (length > 0 && (size_t)length < buffer_size) {
        return buffer;
      }
    }
  }
  return path;
}

static inline int catia_ensure_directory(const char *path)
{
  if (path == NULL || path[0] == '\0') {
    return -1;
  }
  char resolved[PATH_MAX];
  const char *target = catia_resolve_path(path, resolved, sizeof(resolved));
  if (target == NULL) {
    return -1;
  }

  char temp[PATH_MAX];
  size_t length = 0;
  while (target[length] != '\0' && length < sizeof(temp) - 1) {
    temp[length] = target[length];
    length++;
  }
  if (target[length] != '\0') {
    return -1;
  }
  temp[length] = '\0';

  for (size_t index = 1; index <= length; index++) {
    if (temp[index] == '/' || temp[index] == '\0') {
      char character = temp[index];
      temp[index] = '\0';
      if (temp[0] != '\0' && mkdir(temp, 0755) != 0 && errno != EEXIST) {
        // Parent directory attempt - ignore if it already exists
      }
      temp[index] = character;
    }
  }

  struct stat status;
  if (stat(target, &status) != 0 || !S_ISDIR(status.st_mode) || access(target, W_OK) != 0) {
    return -1;
  }
  return 0;
}

#endif // CATIA_PATH_UTILS_H
