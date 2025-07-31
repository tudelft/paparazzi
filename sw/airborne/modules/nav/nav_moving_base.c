/*
 * Copyright (C) 2025 Noah Wechtler <noah.wechtler@tudelft.nl>
 *
 * This file is part of paparazzi
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
 * @file "modules/nav/nav_moving_base.c"
 * Specific navigation functions for tracking and landing on a moving base
 */

#include "modules/nav/nav_moving_base.h"
#include "math/pprz_algebra.h"

#ifndef NAV_MOVING_BASE_MAX_H_ACCEL
#define NAV_MOVING_BASE_MAX_H_ACCEL (struct FloatVect2) { \
  .x = 2.0f, \
  .y = 2.0f \
}
#endif

#ifndef NAV_MOVING_BASE_MAX_H_SPEED
#define NAV_MOVING_BASE_MAX_H_SPEED (struct FloatVect2) { \
  .x = 10.0f, \
  .y = 10.0f \
}
#endif

#ifndef NAV_MOVING_BASE_MAX_V_ACCEL
// Use FloatVect2 for convenient math macros, x is upwards limit, y is downwards limit
#define NAV_MOVING_BASE_MAX_V_ACCEL (struct FloatVect2) { \
  .x = 2.0f, \
  .y = -2.0f \
}
#endif

#ifndef NAV_MOVING_BASE_MAX_V_SPEED
// Use FloatVect2 for convenient math macros, x is upwards limit, y is downwards limit
#define NAV_MOVING_BASE_MAX_V_SPEED (struct FloatVect2) { \
  .x = 2.0f, \
  .y = -2.0f \
}
#endif

static void nav_moving_base_track(void);
static void nav_moving_base_descend(void);
static void nav_moving_base_land(void);

static struct NavMovingBase nav_moving_base;
  
void nav_moving_base_init(void) {
  VECT2_COPY(nav_moving_base.max_accel_h, NAV_MOVING_BASE_MAX_H_ACCEL);
  VECT2_COPY(nav_moving_base.max_speed_h, NAV_MOVING_BASE_MAX_H_SPEED);
  VECT2_COPY(nav_moving_base.max_accel_v, NAV_MOVING_BASE_MAX_V_ACCEL);
  VECT2_COPY(nav_moving_base.max_speed_v, NAV_MOVING_BASE_MAX_V_SPEED);
  nav_moving_base.complete = false;
  nav_moving_base.stay_indefinitely = false;
}

void nav_moving_base_setup(int waypoint_id, enum NavMovingBaseMode mode) {
  nav_moving_base.complete = false;
  nav_moving_base.mode = mode;
  nav_moving_base.wp = waypoints[waypoint_id];
}

bool nav_moving_base_run(void) {
  switch (nav_moving_base.mode) {
    case NAV_MOVING_BASE_MODE_TRACKING_NO_FF:
      nav_moving_base_track();
      break;
    case NAV_MOVING_BASE_MODE_TRACKING:
      nav_moving_base_track();
      break;
    case NAV_MOVING_BASE_MODE_DESCEND_NO_FF:
      nav_moving_base_descend();
      break;
    case NAV_MOVING_BASE_MODE_DESCEND:
      nav_moving_base_descend();
      break;
    case NAV_MOVING_BASE_MODE_LANDING:
      nav_moving_base_land();
      break;
  }

  return !nav_moving_base.complete;
}

static void nav_moving_base_track(void) {
  nav_moving_base.complete = true;
}

static void nav_moving_base_descend(void) {
  nav_moving_base.complete = true;
}

static void nav_moving_base_land(void) {
  nav_moving_base.complete = true;
}