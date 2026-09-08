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
 * @file modules/meteo/wind_circle.h
 * Onboard wind from GPS ground speed while circling (fixed-wing, no airspeed
 * sensor or magnetometer needed).
 *
 * Same idea as the ground server's Wind estimator (sw/ground_segment/tmtc/wind.ml):
 * at constant airspeed the ground speed vectors collected over a full turn lie
 * on a circle whose centre is the wind vector and whose radius is the airspeed.
 * The vectors are kept in course bins and the circle is fitted by least squares.
 * The result is published to the state interface, so nav_drop, upwind landing and
 * every other consumer of stateGetHorizontalWindspeed_f() use it without a datalink.
 */

#ifndef WIND_CIRCLE_H
#define WIND_CIRCLE_H

#include "std.h"
#include "math/pprz_algebra_float.h"

struct wind_circle_t {
  struct FloatVect2 wind;   ///< NED horizontal wind, x north, y east [m/s] (direction it blows to)
  float speed_mps;          ///< horizontal wind speed
  float dir_deg;            ///< direction the wind blows FROM, 0..360 deg (meteorological)
  float airspeed_mps;       ///< fitted mean airspeed (sanity check against the pitot)
  float stddev_mps;         ///< fit residual: spread of the ground speed vectors around the circle
  uint8_t nb_bins;          ///< course bins filled so far
  bool sampling;            ///< collecting ground speed samples
  bool ready;               ///< a valid estimate has been published since the last start
};

extern struct wind_circle_t wind_circle;

extern void wind_circle_init(void);
extern void wind_circle_periodic(void);

/** Flight plan API. start clears the samples and begins collecting; stop freezes the estimate. */
extern uint8_t wind_circle_start(void);
extern uint8_t wind_circle_stop(void);

/** Course (rad, from north clockwise) flying INTO the wind; the present course if no valid wind. */
extern float wind_circle_upwind_course(void);

#endif /* WIND_CIRCLE_H */
