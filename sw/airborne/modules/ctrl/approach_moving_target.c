/*
 * Copyright (C) 2021 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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
 * @file "modules/ctrl/approach_moving_target.c"
 * @author Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * Approach a moving target (e.g. ship)
 */

#include "approach_moving_target.h"

#include "generated/modules.h"
#include "subsystems/abi.h"

float approach_moving_target_angle_deg;

#define DEBUG_AMT TRUE
#include <stdio.h>

struct Amt amt = {
  .distance = 40,
  .speed = -1.0,
  .pos_gain = 1.0,
  .psi_ref = 0.0,
  .slope_ref = 30.0,
  .enabled_time = 0,
  .wp_id = 0,
};

struct SHIP {
  struct FloatVect3 pos;
  struct FloatVect3 vel;
};

struct SHIP ship = {
  .pos = {4.0, 2.0, 0.0},
  .vel = {-2.0, 0.0, 0.0},
};

bool approach_moving_target_enabled = false;

struct FloatVect3 nav_get_speed_sp_from_diagonal(struct EnuCoor_i target, float pos_gain, float rope_heading);
void update_waypoint(uint8_t wp_id, struct FloatVect3 * target_ned);

void approach_moving_target_init(void)
{
  // nothing to do here
}

// interface with ship position module?

// Update a waypoint such that you can see on the GCS where the drone wants to go
void update_waypoint(uint8_t wp_id, struct FloatVect3 * target_ned) {

  // Update the waypoint
  struct EnuCoor_f target_enu;
  ENU_OF_TO_NED(target_enu, *target_ned);
  waypoint_set_enu(wp_id, &target_enu);

  // Send waypoint update every half second
  RunOnceEvery(100/2, {
    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                                &waypoints[wp_id].enu_i.x,
                                &waypoints[wp_id].enu_i.y,
                                &waypoints[wp_id].enu_i.z);
  } );
}

// Function to enable from flight plan (call repeatedly!)
void approach_moving_target_enable(uint8_t wp_id) {
  amt.enabled_time = get_sys_time_msec();
  amt.wp_id = wp_id;
}

void follow_diagonal_approach(void) {

  // Check if the flight plan recently called the enable function
  if ( (get_sys_time_msec() - amt.enabled_time) > (2000 / NAV_FREQ)) {
    return;
  }

  // Reference model
  float gamma_ref = RadOfDeg(amt.slope_ref);
  float psi_ref = RadOfDeg(amt.psi_ref);

  float dt = FOLLOW_DIAGONAL_APPROACH_PERIOD;

  amt.distance += amt.speed*dt;

  amt.rel_unit_vec.x = cosf(gamma_ref) * cosf(psi_ref);
  amt.rel_unit_vec.y = cosf(gamma_ref) * sinf(psi_ref);
  amt.rel_unit_vec.z = -sinf(gamma_ref);
  
  // desired position = rel_pos + ship_pos
  struct FloatVect3 relpos;
  VECT3_SMUL(relpos, amt.rel_unit_vec, amt.distance);

  struct FloatVect3 des_pos;
  VECT3_SUM(des_pos, relpos, ship.pos);

  struct FloatVect3 relvel;
  VECT3_SMUL(relvel, amt.rel_unit_vec, amt.speed);

  // error controller
  struct FloatVect3 ec_vel;
  struct NedCoor_f *drone_pos = stateGetPositionNed_f();
  VECT3_DIFF(ec_vel, des_pos, *drone_pos);
  VECT3_SMUL(ec_vel, ec_vel, amt.pos_gain);

  // desired velocity = rel_vel + ship_vel + error_controller(using NED position)
  struct FloatVect3 des_vel = {
    relvel.x + ship.vel.x + ec_vel.x,
    relvel.y + ship.vel.y + ec_vel.y,
    relvel.z + ship.vel.z + ec_vel.z,
  };

  vect_bound_in_3d(&des_vel, 10.0);

  AbiSendMsgVEL_SP(VEL_SP_FCR_ID, &des_vel);

  // For display purposes
  update_waypoint(amt.wp_id, &des_pos);
}


