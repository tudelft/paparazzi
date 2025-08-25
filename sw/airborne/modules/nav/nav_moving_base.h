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

#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

enum NavMovingBaseMode {
  NAV_MOVING_BASE_MODE_TRACKING_NO_FF,    // Track a moving base without feedforward speed
  NAV_MOVING_BASE_MODE_TRACKING,          // Track a moving base with feedforward speed
  NAV_MOVING_BASE_MODE_DESCEND_NO_FF,     // Descend towards a moving base without feedforward speed
  NAV_MOVING_BASE_MODE_DESCEND,           // Descend towards a moving base
  NAV_MOVING_BASE_MODE_LANDING
};

struct NavMovingBase {
  enum NavMovingBaseMode mode;
  struct FloatVect2 max_accel_h[2]; ///< max horizontal acceleration in m/s^2, longitudinal (forwards, backwards), lateral (right, left)
  struct FloatVect2 max_speed_h[2]; ///< max horizontal speed in m/s, longitudinal (forwards, backwards), lateral (right, left)
  struct FloatVect2 max_accel_v; ///< max vertical acceleration in m/s^2, up, down
  struct FloatVect2 max_speed_v; ///< max vertical speed in m/s, up, down
  bool stay;
  struct EnuCoor_f pos;
  struct EnuCoor_f speed;
  struct EnuCoor_f accel;
  struct FloatVect3 pos_gain; ///< position gain for position control
  struct FloatVect3 speed_gain; ///< speed gain for speed control
  uint8_t active_wp;
};

extern struct NavMovingBase nav_moving_base;

extern void nav_moving_base_init(void);
extern void nav_moving_base_periodic(void);
extern void nav_moving_base_setup(enum NavMovingBaseMode mode, uint8_t wp_index);
extern bool nav_moving_base_run(void);
void nav_moving_base_set_pos(struct EnuCoor_f *pos);
void nav_moving_base_set_speed(struct EnuCoor_f *speed);
void nav_moving_base_set_accel(struct EnuCoor_f *accel);
void nav_moving_base_set_max_accel_h(struct FloatVect2 longitudinal, struct FloatVect2 lateral);
void nav_moving_base_set_max_speed_h(struct FloatVect2 longitudinal, struct FloatVect2 lateral);

#endif /* NAV_MOVING_BASE_H */
