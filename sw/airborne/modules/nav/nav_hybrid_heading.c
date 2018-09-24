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


#define NAV_HYBRID_MAX_YAW_RATE_RAD         RadOfDeg(30.0f/5.0f)    // rad/sec
#define NAV_HYBRID_MAX_YAW_RATE_TO_WP_RAD   RadOfDeg(160.0f/5.0f)    // rad/sec
#define NAV_HYBRID_PITCH_DEADBAND_RAD       RadOfDeg(8.0f)          // rad
#define NAV_HYBRID_TIP_IN_WIND              0;                     // Left (-1) - Right (1)

// setpoint
float nav_hybrid_heading_setpoint = 0;

// settings
float nav_hybrid_heading_max_yaw_rate = NAV_HYBRID_MAX_YAW_RATE_RAD;
float nav_hybrid_heading_max_yaw_rate_to_wp = NAV_HYBRID_MAX_YAW_RATE_TO_WP_RAD;
float nav_hybrid_heading_pitch_deadband = NAV_HYBRID_PITCH_DEADBAND_RAD;
float nav_hybrid_heading_tip_in_wind = NAV_HYBRID_TIP_IN_WIND;
float nav_hybrid_heading_highspeed = 1.0;

// setpoint for slow turn to WP
float nav_hybrid_heading_wp_ref = 0;
float nav_hybrid_heading_wp_setpoint = 0;


void nav_hybrid_heading_init(void) {
  nav_hybrid_heading_setpoint = ANGLE_FLOAT_OF_BFP(nav_heading);
}

void nav_hybrid_heading_init_to_waypoint(int wp) {

  nav_hybrid_heading_wp_ref = ANGLE_FLOAT_OF_BFP(nav_heading);

  // WP-X is East
  // WP-Y is North
  struct FloatVect2 target = {WaypointX(wp), WaypointY(wp)};
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
  // don't change heading if closer than 0.5m to target
  if (VECT2_NORM2(pos_diff) > 0.25) {
    nav_hybrid_heading_wp_setpoint = atan2f(pos_diff.x, pos_diff.y);
  } else {
    nav_hybrid_heading_wp_setpoint = nav_hybrid_heading_wp_ref;
  }

}

void nav_hybrid_heading_periodic(void) {

  // Nose in the wing
  if (nav_hybrid_heading_tip_in_wind == 2) {
    if (stateGetNedToBodyEulers_f()->phi < -nav_hybrid_heading_pitch_deadband) {
      nav_hybrid_heading_setpoint -= nav_hybrid_heading_max_yaw_rate / 512.0f;
    // if pitch is up, yaw left
    } else if (stateGetNedToBodyEulers_f()->phi > nav_hybrid_heading_pitch_deadband) {
      nav_hybrid_heading_setpoint += nav_hybrid_heading_max_yaw_rate / 512.0f;
    }
  } else {
    // Align with tip in wind

    float yawdirection = 1;
    if (nav_hybrid_heading_tip_in_wind == 1) {
      yawdirection = -1;
    }
    if (nav_hybrid_heading_tip_in_wind == 0) {
      if (stateGetNedToBodyEulers_f()->phi > 0) {
        yawdirection = 1;
      }
    }

    // if pitch is down, yaw right
    if (stateGetNedToBodyEulers_f()->theta < -nav_hybrid_heading_pitch_deadband) {
      nav_hybrid_heading_setpoint -= nav_hybrid_heading_max_yaw_rate / 512.0f * yawdirection * nav_hybrid_heading_highspeed;
    // if pitch is up, yaw left
    } else if (stateGetNedToBodyEulers_f()->theta > nav_hybrid_heading_pitch_deadband) {
      nav_hybrid_heading_setpoint += nav_hybrid_heading_max_yaw_rate / 512.0f * yawdirection * nav_hybrid_heading_highspeed;
    }

  }

  // Towards Waypoint
  float err = nav_hybrid_heading_wp_setpoint - nav_hybrid_heading_wp_ref;
  NormRadAngle(err);
  if (err < 0) {
    nav_hybrid_heading_wp_ref -= nav_hybrid_heading_max_yaw_rate_to_wp / 512.0f;
  } else {
    nav_hybrid_heading_wp_ref += nav_hybrid_heading_max_yaw_rate_to_wp / 512.0f;
  }
}

void nav_hybrid_heading_set(void) {
  nav_hybrid_heading_highspeed = 1.0;
  nav_set_heading_rad( nav_hybrid_heading_setpoint );
}

void nav_hybrid_heading_set_fast(void) {
  nav_hybrid_heading_highspeed = 2.0;
  nav_set_heading_rad( nav_hybrid_heading_setpoint );
}

void nav_hybrid_heading_set_left(void) {
  nav_hybrid_heading_tip_in_wind = 1;
  nav_set_heading_rad( nav_hybrid_heading_setpoint );
}

void nav_hybrid_heading_set_nose(void) {
  nav_hybrid_heading_tip_in_wind = 2;
  nav_set_heading_rad( nav_hybrid_heading_setpoint );
}

void nav_hybrid_heading_set_right(void) {
  nav_hybrid_heading_tip_in_wind = 3;
  nav_set_heading_rad( nav_hybrid_heading_setpoint );
}

void nav_hybrid_heading_set_to_waypoint(void) {
  nav_set_heading_rad( nav_hybrid_heading_wp_ref );
}
