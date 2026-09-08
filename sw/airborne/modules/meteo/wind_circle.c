/*
 * Copyright (C) 2026 OpenUAS
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/meteo/wind_circle.c
 * Onboard wind from GPS ground speed while circling. See wind_circle.h.
 */

#include "modules/meteo/wind_circle.h"
#include "state.h"
#include "modules/gps/gps.h"
#include "modules/datalink/telemetry.h"
#include "generated/modules.h"
#include <math.h>

/** Course bins over 360 deg; the ground server uses 36 as well. */
#ifndef WIND_CIRCLE_BINS
#define WIND_CIRCLE_BINS 36
#endif
/** Bins that must be filled before a fit is attempted (2/3 of a turn). */
#ifndef WIND_CIRCLE_MIN_BINS
#define WIND_CIRCLE_MIN_BINS 24
#endif
/** Ground speed below this is not a flying aircraft: ignore the sample. */
#ifndef WIND_CIRCLE_MIN_GSPEED
#define WIND_CIRCLE_MIN_GSPEED 3.f
#endif
/** Reject fits with a larger spread of the samples around the circle (gusts, speed changes). */
#ifndef WIND_CIRCLE_MAX_STDDEV
#define WIND_CIRCLE_MAX_STDDEV 1.5f
#endif
/** Reject nonsense (fit blew up). */
#ifndef WIND_CIRCLE_MAX_WIND
#define WIND_CIRCLE_MAX_WIND 20.f
#endif
/** Below this the direction is meaningless; the estimate is still published. */
#ifndef WIND_CIRCLE_MIN_WIND
#define WIND_CIRCLE_MIN_WIND 0.5f
#endif

struct wind_circle_t wind_circle;

static struct FloatVect2 bins[WIND_CIRCLE_BINS];   // ground speed per course bin, x north, y east
static bool bin_valid[WIND_CIRCLE_BINS];

static void clear_bins(void)
{
  for (int i = 0; i < WIND_CIRCLE_BINS; i++) {
    bin_valid[i] = false;
    bins[i].x = 0.f;
    bins[i].y = 0.f;
  }
  wind_circle.nb_bins = 0;
}

/**
 * Least squares circle through the ground speed vectors (Kasa fit):
 * |g - w|^2 = r^2  <=>  2 gx wx + 2 gy wy + c = gx^2 + gy^2, c = r^2 - |w|^2.
 * Returns false when the system is singular (samples on one line, too few).
 */
static bool fit_circle(struct FloatVect2 *centre, float *radius, float *stddev)
{
  float m[3][3] = {{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
  float v[3] = {0.f, 0.f, 0.f};
  int n = 0;
  for (int i = 0; i < WIND_CIRCLE_BINS; i++) {
    if (!bin_valid[i]) { continue; }
    float row[3] = {2.f * bins[i].x, 2.f * bins[i].y, 1.f};
    float rhs = bins[i].x * bins[i].x + bins[i].y * bins[i].y;
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        m[r][c] += row[r] * row[c];
      }
      v[r] += row[r] * rhs;
    }
    n++;
  }
  if (n < 3) { return false; }

  float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
              - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
              + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if (fabsf(det) < 1e-6f) { return false; }

  float p[3];
  for (int k = 0; k < 3; k++) {
    // Cramer: replace column k by v
    float a[3][3];
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        a[r][c] = (c == k) ? v[r] : m[r][c];
      }
    }
    p[k] = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])) / det;
  }
  float r2 = p[2] + p[0] * p[0] + p[1] * p[1];
  if (r2 <= 0.f) { return false; }
  centre->x = p[0];
  centre->y = p[1];
  *radius = sqrtf(r2);

  float sum = 0.f;
  for (int i = 0; i < WIND_CIRCLE_BINS; i++) {
    if (!bin_valid[i]) { continue; }
    float dx = bins[i].x - centre->x;
    float dy = bins[i].y - centre->y;
    float err = sqrtf(dx * dx + dy * dy) - *radius;
    sum += err * err;
  }
  *stddev = sqrtf(sum / (float)n);
  return true;
}

static void send_wind(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t flags = wind_circle.ready ? 1 : 0;
  float up = 0.f;
  pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID, &flags, &wind_circle.wind.y, &wind_circle.wind.x,
                              &up, &wind_circle.airspeed_mps);
}

void wind_circle_init(void)
{
  wind_circle.wind.x = 0.f;
  wind_circle.wind.y = 0.f;
  wind_circle.speed_mps = 0.f;
  wind_circle.dir_deg = 0.f;
  wind_circle.airspeed_mps = 0.f;
  wind_circle.stddev_mps = 0.f;
  wind_circle.sampling = false;
  wind_circle.ready = false;
  clear_bins();
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_INFO_RET, send_wind);
#endif
}

void wind_circle_periodic(void)
{
  if (!wind_circle.sampling || !GpsFixValid()) { return; }
  float gspeed = stateGetHorizontalSpeedNorm_f();
  if (gspeed < WIND_CIRCLE_MIN_GSPEED) { return; }
  float course = stateGetHorizontalSpeedDir_f();
  while (course < 0.f) { course += 2.f * M_PI; }
  while (course >= 2.f * M_PI) { course -= 2.f * M_PI; }
  int i = (int)(course * (float)WIND_CIRCLE_BINS / (2.f * M_PI));
  if (i < 0) { i = 0; }
  if (i >= WIND_CIRCLE_BINS) { i = WIND_CIRCLE_BINS - 1; }
  bins[i].x = gspeed * cosf(course);
  bins[i].y = gspeed * sinf(course);
  if (!bin_valid[i]) {
    bin_valid[i] = true;
    wind_circle.nb_bins++;
  }
  if (wind_circle.nb_bins < WIND_CIRCLE_MIN_BINS) { return; }

  struct FloatVect2 w;
  float airspeed, stddev;
  if (!fit_circle(&w, &airspeed, &stddev)) { return; }
  float speed = sqrtf(w.x * w.x + w.y * w.y);
  if (stddev > WIND_CIRCLE_MAX_STDDEV || speed > WIND_CIRCLE_MAX_WIND) { return; }

  wind_circle.wind = w;
  wind_circle.speed_mps = speed;
  wind_circle.airspeed_mps = airspeed;
  wind_circle.stddev_mps = stddev;
  if (speed >= WIND_CIRCLE_MIN_WIND) {
    float from = DegOfRad(atan2f(-w.y, -w.x));
    if (from < 0.f) { from += 360.f; }
    wind_circle.dir_deg = from;
  }
  stateSetHorizontalWindspeed_f(MODULE_WIND_CIRCLE_ID, &w);
  wind_circle.ready = true;
}

uint8_t wind_circle_start(void)
{
  clear_bins();
  wind_circle.ready = false;
  wind_circle.sampling = true;
  return 0;
}

uint8_t wind_circle_stop(void)
{
  wind_circle.sampling = false;
  return 0;
}

float wind_circle_upwind_course(void)
{
  if (!wind_circle.ready || wind_circle.speed_mps < WIND_CIRCLE_MIN_WIND) {
    return stateGetHorizontalSpeedDir_f();
  }
  return atan2f(-wind_circle.wind.y, -wind_circle.wind.x);
}
