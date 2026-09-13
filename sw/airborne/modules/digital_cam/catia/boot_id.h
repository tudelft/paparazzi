#ifndef CATIA_BOOT_ID_H
#define CATIA_BOOT_ID_H

/**
 * @file boot_id.h
 * @brief Read Linux's boot-unique identifier for pose-log session provenance.
 * @details A boot ID distinguishes monotonic timestamps from two daemon runs after a
 * board reboot. Failure is nonfatal and intentionally represented as "unknown" so
 * logging remains available on non-Linux development systems.
 */

#include <stdio.h>

/** @brief Fill a 36-character boot UUID or the stable fallback "unknown".
 * @param result Caller buffer of exactly 37 bytes, including the terminator.
 * @details Input is strictly validated instead of trusting procfs text, because this
 * value becomes durable log metadata and should never contain a partial read. */
static inline void catia_boot_id(char result[37])
{
  const char unknown[] = "unknown";
  for (size_t index = 0; index < sizeof(unknown); ++index) result[index] = unknown[index];
  FILE *file = fopen("/proc/sys/kernel/random/boot_id", "re");
  if (file == NULL) return;
  char value[38];
  size_t length = fread(value, 1, sizeof(value), file);
  int closed = fclose(file);
  if (closed != 0 || length != 37 || value[36] != '\n') return;
  for (size_t index = 0; index < 36; ++index) {
    if (index == 8 || index == 13 || index == 18 || index == 23) {
      if (value[index] != '-') return;
    } else if (!((value[index] >= '0' && value[index] <= '9')
                 || (value[index] >= 'a' && value[index] <= 'f'))) return;
  }
  for (size_t index = 0; index < 36; ++index) result[index] = value[index];
  result[36] = '\0';
}

#endif