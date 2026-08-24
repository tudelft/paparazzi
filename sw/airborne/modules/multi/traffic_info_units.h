#ifndef TRAFFIC_INFO_UNITS_H
#define TRAFFIC_INFO_UNITS_H

#include <stdbool.h>
#include <stdint.h>

/** Convert centimeters to millimeters without signed overflow.
 *
 * Datalink altitude fields can span the full signed 32-bit range. Scaling in
 * 64 bits and rejecting an unrepresentable result prevents malformed traffic
 * from wrapping across the vertical reference frame.
 *
 * @param[in] centimeters Input distance in centimeters.
 * @param[out] millimeters Converted distance; unchanged on failure.
 * @return @c true when the scaled value fits in an `int32_t`.
 */
static inline bool traffic_info_cm_to_mm(int32_t centimeters, int32_t *millimeters)
{
  const int64_t scaled = (int64_t)centimeters * INT64_C(10);
  if (scaled < INT32_MIN || scaled > INT32_MAX) {
    return false;
  }
  *millimeters = (int32_t)scaled;
  return true;
}

#endif /* TRAFFIC_INFO_UNITS_H */