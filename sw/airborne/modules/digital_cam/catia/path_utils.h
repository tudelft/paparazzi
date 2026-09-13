#ifndef CATIA_PATH_UTILS_H
#define CATIA_PATH_UTILS_H

/**
 * @file path_utils.h
 * @brief Small path and writable-directory helpers shared by CATIA backends.
 * @details Paths are intentionally resolved only for a leading `~`: CATIA configuration
 * is not a shell and must not expand arbitrary variables or execute shell syntax.
 */

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

/** @brief Expand a leading home-directory shorthand without allocating memory.
 * @param path Configured path, possibly beginning with `~` or `~/`.
 * @param buffer Caller-owned expansion storage.
 * @param buffer_size Capacity of @p buffer.
 * @return @p buffer for a successful expansion, @p path when no expansion is needed,
 * or NULL when @p path is NULL.
 * @details Returning the original pointer avoids copying the common literal-path case. */
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

/** @brief Create missing directory components and verify the final directory is writable.
 * @param path Directory path, with optional leading home shorthand.
 * @return 0 for a writable directory, otherwise -1.
 * @details Components are created one at a time to avoid invoking a shell. Existing
 * parents are harmless, while final stat/access checks distinguish a file or read-only
 * path from a usable capture destination. */
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
