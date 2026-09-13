/** @file test_loudestspot.c @brief Standalone synthetic-data regression tool for EARcam loudest-spot fusion. */
// C11 + POSIX.1-2008 (clock_gettime).
#define _POSIX_C_SOURCE 200809L

// Offline test for calculated_loudestspot(): synthetic 4-strip survey over a
// ground source, inverse-distance field, noise, and a few clipped/quiet outliers.
#include "ear_cam_pipe.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define DEG_TO_RAD 0.017453292519943295
#define EARTH_RADIUS_M 6378137.0
#define PI 3.14159265358979323846

static double uniform(void)
{
  return (double)rand() / (double)RAND_MAX;
}

static const double lat0 = 43.4630000;
static const double lon0 = 1.2730000;

static void fill_sample(struct ear_sample *s, size_t index, double east, double north,
                        double agl, double source_east, double source_north,
                        double noise_db, double *previous_level)
{
  const double cos_lat = cos(lat0 * DEG_TO_RAD);
  double d = hypot(hypot(east - source_east, north - source_north), agl);
  double level = 95.0 - 20.0 * log10(d / 3.0) - 118.4 + (uniform() - 0.5) * 2.0 * noise_db;
  s->timestamp_ms = 1000 + index * 50;
  s->shot_nr = (int32_t)index + 1;
  s->lat_deg = lat0 + (north / EARTH_RADIUS_M) / DEG_TO_RAD;
  s->lon_deg = lon0 + (east / (EARTH_RADIUS_M * cos_lat)) / DEG_TO_RAD;
  s->agl_m = agl + (uniform() - 0.5);
  s->alt_m = 260.0 + agl;
  s->level_db = level;
  s->trend_db = level - *previous_level;
  s->contrast_db = 4.0;
  s->frequency_hz = 2700.0;
  s->alarm = true;
  s->clipped = false;
  *previous_level = level;
}

static void estimate_xy(const struct ear_loudest_spot *spot, double *east, double *north)
{
  const double cos_lat = cos(lat0 * DEG_TO_RAD);
  *east = (spot->lon_deg - lon0) * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat;
  *north = (spot->lat_deg - lat0) * DEG_TO_RAD * EARTH_RADIUS_M;
}

// Coarse strip survey followed by iterated 4-leg stars through the estimate,
// mirroring the digital_cam_earcam refinement flow. Returns error after refinement.
static int run_refinement_case(double source_east, double source_north,
                               double survey_agl, double refine_agl, double noise_db)
{
  const double speed = 10.0, interval_s = 0.05, strip_length = 100.0, spacing = 25.0;
  const size_t strips = 4, per_strip = (size_t)(strip_length / (speed * interval_s));
  const size_t legs = 4, per_leg = (size_t)(100.0 / (speed * interval_s));
  const size_t capacity = strips * per_strip + 3 * legs * per_leg;
  struct ear_sample *samples = calloc(capacity, sizeof(*samples));
  if (samples == NULL) {
    return 1;
  }
  size_t count = 0;
  double previous_level = -60.0;
  for (size_t strip = 0; strip < strips; strip++) {
    double north = strip * spacing;
    for (size_t step = 0; step < per_strip; step++) {
      double east = (strip % 2 == 0) ? step * speed * interval_s : strip_length - step * speed * interval_s;
      fill_sample(&samples[count], count, east, north, survey_agl, source_east, source_north,
                  noise_db, &previous_level);
      count++;
    }
  }
  struct ear_loudest_spot spot;
  if (calculated_loudestspot(samples, count, &spot) != 0 || !spot.valid) {
    free(samples);
    return 1;
  }
  double cx, cy;
  estimate_xy(&spot, &cx, &cy);
  double coarse_error = hypot(cx - source_east, cy - source_north);

  double refined_error = coarse_error;
  for (int iteration = 0; iteration < 3; iteration++) {
    for (size_t leg = 0; leg < legs; leg++) {
      double heading = leg * PI / legs;
      double sign = (leg % 2 == 1) ? -1.0 : 1.0;
      for (size_t step = 0; step < per_leg; step++) {
        double t = -50.0 + step * speed * interval_s;
        double east = cx + sign * sin(heading) * t;
        double north = cy + sign * cos(heading) * t;
        fill_sample(&samples[count], count, east, north, refine_agl, source_east, source_north,
                    noise_db, &previous_level);
        count++;
      }
    }
    if (calculated_loudestspot(samples, count, &spot) != 0 || !spot.valid) {
      break;
    }
    double nx, ny;
    estimate_xy(&spot, &nx, &ny);
    double shift = hypot(nx - cx, ny - cy);
    cx = nx;
    cy = ny;
    refined_error = hypot(cx - source_east, cy - source_north);
    if (shift < 4.0) {
      break;
    }
  }
  printf("refine: survey AGL %.0f noise %.1f dB -> coarse err %.2f m, refined err %.2f m "
         "(conf %.2f, %zu samples)\n", survey_agl, noise_db, coarse_error, refined_error,
         spot.confidence, count);
  free(samples);
  return refined_error <= 3.0 && refined_error <= coarse_error + 0.5 ? 0 : 1;
}

// IMAV Mission 4: no survey, stars around a given centre up to 25 m from the alarm.
// Samples only in the quiet zone (motor off 40 m before to 30 m past the centre,
// first 1.5 s lost to prop spin-down) while gliding from 12 m down to about 6 m.
// Requires sub-metre agreement after at most three stars.
static int run_mission4_case(double source_east, double source_north, double noise_db)
{
  const double speed = 10.0, interval_s = 0.05, half_length = 72.0;
  const double quiet_from = -40.0 + speed * 1.5, quiet_to = 30.0, top_agl = 12.0, glide_ratio = 11.0;
  const size_t legs = 4, per_leg = (size_t)(2.0 * half_length / (speed * interval_s));
  const size_t capacity = 3 * legs * per_leg;
  struct ear_sample *samples = calloc(capacity, sizeof(*samples));
  if (samples == NULL) {
    return 1;
  }
  size_t count = 0;
  double previous_level = -60.0;
  double cx = 0.0, cy = 0.0, error = hypot(source_east, source_north), first_error = error;
  int stars = 0;
  for (int iteration = 0; iteration < 3; iteration++) {
    for (size_t leg = 0; leg < legs; leg++) {
      double heading = leg * PI / legs;
      double sign = (leg % 2 == 1) ? -1.0 : 1.0;
      for (size_t step = 0; step < per_leg; step++) {
        double t = -half_length + step * speed * interval_s;
        if (t < quiet_from || t > quiet_to) {
          continue;
        }
        double agl = top_agl - (t + 40.0) / glide_ratio;
        fill_sample(&samples[count], count, cx + sign * sin(heading) * t, cy + sign * cos(heading) * t,
                    agl, source_east, source_north, noise_db, &previous_level);
        count++;
      }
    }
    struct ear_loudest_spot spot;
    if (calculated_loudestspot(samples, count, &spot) != 0 || !spot.valid) {
      break;
    }
    stars++;
    double nx, ny;
    estimate_xy(&spot, &nx, &ny);
    double shift = hypot(nx - cx, ny - cy);
    cx = nx;
    cy = ny;
    error = hypot(cx - source_east, cy - source_north);
    if (shift < 1.5) {
      break;
    }
  }
  printf("mission4: centre off by %.1f m, noise %.1f dB -> err %.2f m after %d stars (%zu quiet samples)\n",
         first_error, noise_db, error, stars, count);
  free(samples);
  return stars > 0 && error <= 1.0 ? 0 : 1;
}

static int run_case(double source_east, double source_north, double agl,
                    size_t strips, double strip_length, double spacing,
                    double speed, double noise_db, size_t passes)
{
  const double lat0 = 43.4630000;
  const double lon0 = 1.2730000;
  const double cos_lat = cos(lat0 * DEG_TO_RAD);
  const double interval_s = 0.05;
  size_t per_strip = (size_t)(strip_length / (speed * interval_s));
  size_t count = strips * per_strip * passes;
  struct ear_sample *samples = calloc(count, sizeof(*samples));
  if (samples == NULL) {
    return 1;
  }

  size_t index = 0;
  double previous_level = -60.0;
  for (size_t pass = 0; pass < passes; pass++) {
    for (size_t strip = 0; strip < strips; strip++) {
      double north = strip * spacing;
      for (size_t step = 0; step < per_strip; step++) {
        double east = (strip % 2 == 0) ? step * speed * interval_s
                      : strip_length - step * speed * interval_s;
        double d = hypot(hypot(east - source_east, north - source_north), agl);
        double level = 95.0 - 20.0 * log10(d / 3.0) - 118.4 + (uniform() - 0.5) * 2.0 * noise_db;
        struct ear_sample *s = &samples[index];
        s->timestamp_ms = 1000 + index * 50;
        s->shot_nr = (int32_t)index + 1;
        s->lat_deg = lat0 + (north / EARTH_RADIUS_M) / DEG_TO_RAD;
        s->lon_deg = lon0 + (east / (EARTH_RADIUS_M * cos_lat)) / DEG_TO_RAD;
        s->agl_m = agl + (uniform() - 0.5);
        s->alt_m = 260.0 + agl;
        s->level_db = level;
        s->trend_db = level - previous_level;
        s->contrast_db = 4.0;
        s->frequency_hz = 2700.0;
        s->alarm = true;
        s->clipped = false;
        previous_level = level;
        // A handful of corrupt windows: clipped or silent dropouts.
        if (index % 37 == 0) {
          s->clipped = true;
          s->level_db = -10.0;
        } else if (index % 53 == 0) {
          s->level_db = -110.0;
          s->alarm = false;
        }
        index++;
      }
    }
  }

  struct ear_loudest_spot spot;
  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);
  int status = calculated_loudestspot(samples, count, &spot);
  clock_gettime(CLOCK_MONOTONIC, &end);
  double elapsed_us = (end.tv_sec - start.tv_sec) * 1e6 + (end.tv_nsec - start.tv_nsec) / 1e3;

  double est_east = (spot.lon_deg - lon0) * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat;
  double est_north = (spot.lat_deg - lat0) * DEG_TO_RAD * EARTH_RADIUS_M;
  double error = hypot(est_east - source_east, est_north - source_north);
  printf("samples=%zu used=%u valid=%d est=(%.1f,%.1f) truth=(%.1f,%.1f) err=%.2f m "
         "conf=%.2f agl=%.1f time=%.0f us\n",
         count, spot.used_count, spot.valid, est_east, est_north, source_east, source_north,
         error, spot.confidence, spot.agl_m, elapsed_us);
  free(samples);
  if (status != 0 || !spot.valid) {
    return 1;
  }
  return error <= fmax(6.0, spacing * 0.5) ? 0 : 1;
}

int main(void)
{
  srand(7);
  int failures = 0;
  // 4 strips x 25 m, 10 m/s, 50 ms -> ~200 samples, source between strips.
  failures += run_case(14.0, 11.0, 12.0, 4, 25.0, 8.0, 10.0, 1.5, 1);
  // Same area scanned twice (400 samples), source near an edge.
  failures += run_case(3.0, 22.0, 15.0, 4, 25.0, 8.0, 10.0, 2.5, 2);
  // Larger field, 6 strips of 100 m.
  failures += run_case(61.0, 33.0, 20.0, 6, 100.0, 12.0, 12.0, 2.0, 1);
  // Too few samples must be rejected, not guessed.
  {
    struct ear_sample few[3] = {{0}};
    struct ear_loudest_spot spot;
    if (calculated_loudestspot(few, 3, &spot) == 0 || spot.valid) {
      printf("small-set rejection failed\n");
      failures++;
    }
  }
  // Realistic: high noisy survey, source between strips, then star refinement lower.
  failures += run_refinement_case(37.0, 12.5, 40.0, 20.0, 3.0);
  failures += run_refinement_case(81.0, 66.0, 40.0, 20.0, 3.0);
  failures += run_refinement_case(12.0, 40.0, 60.0, 25.0, 4.0);
  // Mission 4: alarm 5, 15 and 24 m from the given centre, low quiet-zone samples only.
  failures += run_mission4_case(3.0, -4.0, 2.0);
  failures += run_mission4_case(-9.0, 12.0, 3.0);
  failures += run_mission4_case(17.0, -17.0, 3.0);
  printf(failures == 0 ? "loudest-spot self-test passed\n" : "loudest-spot self-test FAILED\n");
  return failures == 0 ? 0 : 1;
}
