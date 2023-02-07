/*
 * Copyright (C) 2023	MAVLab
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/nav/nav_snap.c
 *
 * Feed-forward a hardcoded minium snap trajectory.
 */

#include "generated/airframe.h"
#include "modules/nav/nav_snap.h"
#include "modules/nav/nav_min_snap.h"
#include "mcu_periph/sys_time.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/core/abi.h"

#include <stdio.h>

double time_zero = 0;

float min_snap_alpha = 0.2;
float min_snap_alpha_active;

float min_snap_a_ff = 1.0;
float min_snap_v_ff = 1.0;
float min_snap_pos_gain = 1.5;
float min_snap_speed_gain = 2.5;
int min_snap_abi = 1;

double min_snap_dt = 0;


///< Call once, just before starting the run

void nav_snap_init(void)
{
  min_snap_dt = 0;
  time_zero = get_sys_time_float();
  min_snap_alpha_active = min_snap_alpha;
}


///< Move a WP to the beginning of the polynomial

bool nav_snap_x0(int _wp)
{
  // North East Down
  double dts = 0;
  int32_t x = POS_BFP_OF_REAL(get_x(min_snap_alpha_active, dts));
  int32_t y = POS_BFP_OF_REAL(get_y(min_snap_alpha_active, dts));
  int32_t z = POS_BFP_OF_REAL(get_z(min_snap_alpha_active, dts));
  double psi = get_psi(min_snap_alpha_active, dts);
  
  // East North Up
  struct EnuCoor_i temp;
  temp.x = y;
  temp.y = x;
  temp.z = -z;
  
  waypoint_set_enu_i(_wp, &temp);
  nav_set_heading_rad(psi);
  return true;
}

///< Run at NAVIGATION_FREQUENCY

bool nav_snap_run(void)
{
  // Import Snap:
  // Everything in North East Down
  double min_snap_dt = get_sys_time_float() - time_zero;
  //min_snap_dt += 1.0 / ((float)NAVIGATION_FREQUENCY);
  //printf("t1=%f, t2=%f \n",min_snap_dt, min_snap_dt2);
  float x_snap = get_x(min_snap_alpha_active, min_snap_dt);
  float y_snap = get_y(min_snap_alpha_active, min_snap_dt);
  float z_snap = get_z(min_snap_alpha_active, min_snap_dt);
  float psi_snap = get_psi(min_snap_alpha_active, min_snap_dt);
  
  float vx_snap = get_vx(min_snap_alpha_active, min_snap_dt);
  float vy_snap = get_vy(min_snap_alpha_active, min_snap_dt);
  float vz_snap = get_vz(min_snap_alpha_active, min_snap_dt);

  float ax_snap = get_ax(min_snap_alpha_active, min_snap_dt);
  float ay_snap = get_ay(min_snap_alpha_active, min_snap_dt);
  float az_snap = get_az(min_snap_alpha_active, min_snap_dt);
  
  /////////////////////////////
  // Export WP to Navigation
  // -> East North Up
  struct EnuCoor_i temp;
  temp.x = POS_BFP_OF_REAL(y_snap);
  temp.y = POS_BFP_OF_REAL(x_snap);
  temp.z = POS_BFP_OF_REAL(-z_snap);
  VECT3_COPY(navigation_target, temp);
  VECT3_COPY(navigation_carrot, temp);

  // Set heading
  nav_set_heading_rad(psi_snap);
  
  // Run linear correction controller on top of the Feed-forward polynomial
  //Linear controller to find the acceleration setpoint from position and velocity
  float pos_x_err = x_snap - stateGetPositionNed_f()->x;
  float pos_y_err = y_snap - stateGetPositionNed_f()->y;
  float pos_z_err = z_snap - stateGetPositionNed_f()->z;

  // Use feed forward velocity part from reference model
  float vx_sp = vx_snap * min_snap_v_ff + pos_x_err * min_snap_pos_gain;
  float vy_sp = vy_snap * min_snap_v_ff + pos_y_err * min_snap_pos_gain;
  float vz_sp = vz_snap * min_snap_v_ff + pos_z_err * min_snap_pos_gain;

  // Acceleration command
  float ax_sp = ax_snap * min_snap_a_ff + (vx_sp - stateGetSpeedNed_f()->x) * min_snap_speed_gain;
  float ay_sp = ay_snap * min_snap_a_ff + (vy_sp - stateGetSpeedNed_f()->y) * min_snap_speed_gain;
  float az_sp = az_snap * min_snap_a_ff + (vz_sp - stateGetSpeedNed_f()->z) * min_snap_speed_gain;

  // Export to ABI
  struct FloatVect3 u;
  // 0: 2D control, 1: 3D control
  uint8_t flag = 1;
  u.x = ax_sp;
  u.y = ay_sp;
  u.z = az_sp;

//  printf("zerr=%f v=%f p=%f \n", pos_z_err, vz_sp, az_sp);

  if (min_snap_abi > 0) {
    AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &u);
  }
  return true; /* This pattern never ends */
}

