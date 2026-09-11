#ifndef CATIA_MOTION_COMPENSATION_H
#define CATIA_MOTION_COMPENSATION_H

#include <math.h>
#include <stdint.h>

static inline int compensate_ground_position(int32_t *latitude, int32_t *longitude,
                                             double speed, double course, double delay)
{
  if (!latitude || !longitude || !isfinite(speed) || !isfinite(course) || !isfinite(delay)
      || speed < 0 || speed > 100 || delay < 0 || delay > 2
      || *latitude < -850000000 || *latitude > 850000000
      || *longitude < -1800000000 || *longitude > 1800000000) return 0;
  const double radians = 0.017453292519943295;
  const double phi = *latitude / 1e7 * radians;
  const double eccentricity = 0.0066943799901413165;
  const double denominator = 1 - eccentricity * sin(phi) * sin(phi);
  const double prime_vertical = 6378137.0 / sqrt(denominator);
  const double meridional = prime_vertical * (1 - eccentricity) / denominator;
  const double corrected_lat = *latitude / 1e7 + speed * delay * cos(course) / meridional / radians;
  double corrected_lon = *longitude / 1e7 + speed * delay * sin(course) / (prime_vertical * cos(phi)) / radians;
  if (corrected_lat < -85 || corrected_lat > 85) return 0;
  if (corrected_lon > 180) corrected_lon -= 360;
  if (corrected_lon < -180) corrected_lon += 360;
  *latitude = (int32_t)llround(corrected_lat * 1e7);
  *longitude = (int32_t)llround(corrected_lon * 1e7);
  return 1;
}

#endif