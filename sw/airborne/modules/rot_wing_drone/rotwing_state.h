/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rotwing/rotwing_state.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module keeps track of the current state of a rotating wing drone and desired state set by the RC or flightplan. Paramters are being scheduled in each change of a current state and desired state. Functions are defined in this module to call the actual state and desired state and set a desired state.
 */

#ifndef ROTWING_STATE_H
#define ROTWING_STATE_H

#include "std.h"

enum rotwing_states_t {
  ROTWING_STATE_FORCE_HOVER,
  ROTWING_STATE_REQUEST_HOVER,
  ROTWING_STATE_REQUEST_FW,
  ROTWING_STATE_FREE,
};

struct rotwing_state_t {
  /* Control */
  enum rotwing_states_t state;  // Current desired state (requested only by NAV and can be overruled by RC)
  bool quad_motors_enabled;     // Quad motors enabled (> idle throttle)

  /* Skew */
  float sp_skew_angle_deg;    // Setpoint skew angle in degrees
  float meas_skew_angle_deg;  // Measured skew angle in degrees
  float meas_skew_angle_time; // Time of the last skew angle measurement
  int16_t skew_cmd;

  /* Airspeeds */
  float cruise_airspeed;      // Airspeed for cruising
  float min_airspeed;         // Minimum airspeed for bounding
  float max_airspeed;         // Maximum airspeed for bounding

  /* RPM measurements*/
  int32_t meas_rpm[5];        // Measured RPM of the hover and pusher motors
  float meas_rpm_time[5];     // Time of the last RPM measurement
};
extern struct rotwing_state_t rotwing_state;

void rotwing_state_init(void);
void rotwing_state_periodic(void);
bool rotwing_state_hover_motors_running(void);
bool rotwing_state_pusher_motor_running(void);

void rotwing_state_request_hover(bool force);
void rotwing_state_request_free(void);

#endif  // ROTWING_STATE_H
