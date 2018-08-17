/*
 * Copyright (C) MAVLab
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
 * @file "modules/nav/nav_hybrid_heading.c"
 * @author MAVLab
 * Move heading into the wind when hybrids are hovering.
 */

#include "modules/nav/nav_hybrid_heading.h"
#include "navigation.h"
#include "state.h"


#define NAV_HYBRID_MAX_YAW_RATE_RAD     RadOfDeg(90.0f/5.0f)   // rad/sec
#define NAV_HYBRID_PITCH_DEADBAND_RAD   RadOfDeg(8.0f)          // rad

// setpoint
float nav_hybrid_heading_setpoint = 0;

// settings
float nav_hybrid_heading_max_yaw_rate = NAV_HYBRID_MAX_YAW_RATE_RAD;
float nav_hybrid_heading_pitch_deadband = NAV_HYBRID_PITCH_DEADBAND_RAD;

void nav_hybrid_heading_init(void) {
  nav_hybrid_heading_setpoint = ANGLE_FLOAT_OF_BFP(nav_heading);
}

void nav_hybrid_heading_periodic(void) {
  // if pitch is down, yaw right
  if (stateGetNedToBodyEulers_f()->theta < -nav_hybrid_heading_pitch_deadband) {
    nav_hybrid_heading_setpoint += nav_hybrid_heading_max_yaw_rate / 512.0f;
  // if pitch is up, yaw left
  } else if (stateGetNedToBodyEulers_f()->theta > nav_hybrid_heading_pitch_deadband) {
    nav_hybrid_heading_setpoint -= nav_hybrid_heading_max_yaw_rate / 512.0f;
  }
}

void nav_hybrid_heading_set(void) {
  nav_set_heading_rad( nav_hybrid_heading_setpoint );
}
