/*
 * Copyright (C) 2021 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rot_wing_drone/wing_rotation_controller.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module that controls the wing rotation of the rotating wing drone
 */

#ifndef WING_ROTATION_CONTROLLER_H
#define WING_ROTATION_CONTROLLER_H

#include "std.h"

// Paramaters
struct wing_rotation_controller {
  int32_t servo_pprz_cmd;
  int32_t pprz_cmd_deadzone;
  uint16_t adc_wing_rotation;
  uint16_t adc_wing_servo;
  int16_t adc_wing_rotation_range;
  float wing_angle_rad;
  float wing_angle_deg;
  float wing_angle_rad_sp;
  float wing_angle_deg_sp;
  float servo_angle_rad;
  float servo_rate;
  bool initialized;
  uint8_t init_loop_count;

  // Control variables
  int32_t p_gain;
  int32_t d_gain;
  uint32_t max_cmd;

  // Endpoint protection
  int32_t cmd_threshold_0;
  int32_t cmd_threshold_90;
  float threshold_servo_rate_limit;

};

extern struct wing_rotation_controller wing_rotation;
extern bool automatic_rot;
extern float f_cutoff;
// Functions
extern void wing_rotation_init(void);
extern void wing_rotation_periodic(void);
extern void wing_rotation_event(void);
extern void skew_interpoler(void);
#endif  // WING_ROTATION_CONTROLLER_H
