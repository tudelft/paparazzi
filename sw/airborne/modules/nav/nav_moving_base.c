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

static void nav_moving_base_track(void);
static void nav_moving_base_descend(void);
static void nav_moving_base_land(void);

void nav_moving_base_init() {
  return;
}

void nav_moving_base_setup(waypoint_t wp, NavMovingBaseMode mode) {
  nav_moving_base.complete = false;
  nav_moving_base.mode = mode;
  nav_moving_base.wp = wp;
}

void nav_moving_base_run(void) {
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