#ifndef CATIA_MOTION_COMPENSATION_H
#define CATIA_MOTION_COMPENSATION_H

/**
 * @file motion_compensation.h
 * @brief Project a capture location backward by the measured camera latency.
 * @details The helper uses WGS-84 local curvature radii rather than a fixed
 * meters-per-degree approximation, which keeps the correction meaningful over CATIA's
 * valid operating latitudes while retaining a small, allocation-free calculation.
 */

#include <math.h>
#include <stdint.h>

/** @brief Correct latitude/longitude for horizontal platform motion during capture.
 * @param latitude In/out latitude in $10^{-7}$ degrees.
 * @param longitude In/out longitude in $10^{-7}$ degrees.
 * @param speed Ground speed in m/s.
 * @param course Ground-track angle in radians, clockwise from north.
 * @param delay Capture latency in seconds.
 * @return 1 when corrected coordinates were stored, 0 when inputs were unsafe.
 * @details Conservative limits reject implausible telemetry and polar geometry, where
 * longitude correction becomes ill-conditioned. Longitude is normalized after the
 * local tangent-plane displacement to preserve the wire representation's range. */
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