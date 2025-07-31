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
 * @file "modules/nav/nav_moving_base.h"
 * Specific navigation functions for tracking and landing on a moving base
 */

#ifndef NAV_MOVING_BASE_H
#define NAV_MOVING_BASE_H

#include "waypoints.h"

enum NavMovingBaseMode {
  NAV_MOVING_BASE_MODE_TRACKING_NO_FF,    // Track a moving base without feedforward speed
  NAV_MOVING_BASE_MODE_TRACKING,          // Track a moving base with feedforward speed
  NAV_MOVING_BASE_MODE_DESCEND_NO_FF,     // Descend towards a moving base without feedforward speed
  NAV_MOVING_BASE_MODE_DESCEND,           // Descend towards a moving base
  NAV_MOVING_BASE_MODE_LANDING
};

struct NavMovingBase {
  enum NavMovingBaseMode mode;
  struct Waypoint wp;
  struct FloatVect2 max_accel_h;
  struct FloatVect2 max_speed_h;
  struct FloatVect2 max_accel_v;
  struct FloatVect2 max_speed_v;
  bool complete;
  bool stay_indefinitely;
};

extern void nav_moving_base_init(void);
extern void nav_moving_base_setup(int waypoint_id, enum NavMovingBaseMode mode);
extern bool nav_moving_base_run(void);

#endif
