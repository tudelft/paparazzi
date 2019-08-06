/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/ctrl_module_outerloop_demo.h
 * @brief example empty controller
 *
 */

#include "modules/ctrl/ctrl_module_outerloop_demo.h"
#include "state.h"
#include "autopilot.h"
#include <stdio.h>
#include "./dronerace/dronerace.h"
#include "./dronerace/filter.h"
// Own Variables

struct ctrl_module_demo_struct dr_ctrl = {0};

// Settings
float comode_time = 0;

float phi0 = 0;
float phi1 = 0;
float invert_time = 0;
float x0[2] = {-4, -2.5}; //GPS_take;
float v0[2] = {0.0, 0.0}; // needs non zero initial velocity, not sure why

float xd[2] = {0.0, 0.0};
float vd[2] = {3.0, 0};

float xt[2] = {0.0, 0.0};
float vt[2] = {0.0, 0.0};




////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  dronerace_init();

  float psi_init = stateGetNedToBodyEulers_f()->psi;
  
  // find_optimal(x0, v0, xd, vd, xt, vt, &phi0, &phi1, &invert_time, psi_init);
  printf("init check: phi0: %f, phi1: %f, dt: %f\n", phi0, phi1, invert_time);
  printf("init reaching set: x: %f, y: %f, vx: %f, vy: %f\n", xt[0], xt[1], vt[0], vt[1]);



} 




void guidance_h_module_enter(void)
{
  // Store current heading
  dr_ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;
  float temp_psi = 0;
  //find_optimal(x0, v0, xd, vd, xt, vt, &phi0, &phi1, &invert_time, temp_psi);

  // Convert RC to setpoint
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
  dronerace_enter();
}

void guidance_h_module_read_rc(void)
{
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

float var_time = 0.0;

float K_ff_theta1 = 14.0/57.0/5.0;   // rad to fly at (e.g. 10 deg = 5 m/s)
float K_p_theta1 = 6.0/57.0;       // m/s to radians


float alt = 0;
float roll = 0;
float pitch = 0;
float yaw = 0;


void guidance_h_module_run(bool in_flight)
{
  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);


  // dronerace_get_cmd(&alt, &roll, &pitch, &yaw);
  var_time = dr_state.time;

  // dr_ctrl.cmd.phi   = ANGLE_BFP_OF_REAL(roll);
  // dr_ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch); //-ANGLE_BFP_OF_REAL(5*3.142/180);
  // dr_ctrl.cmd.psi   = ANGLE_BFP_OF_REAL(yaw);   // stateGetNedToBodyEulers_f()->psi;//
  #if 1
  float lateral_vel_x = K_ff_theta1 * (vd[0] - dr_state.vx) + K_p_theta1 * vd[0];
  if (lateral_vel_x > 20.0/57.0) {
    lateral_vel_x = 20.0/57.0;
  }
  if (lateral_vel_x < -20.0/57.0) {
    lateral_vel_x = -20.0/57.0;
  }

  if (var_time < invert_time) {
    dr_ctrl.cmd.phi = ANGLE_BFP_OF_REAL(phi0);
    dr_ctrl.cmd.theta = ANGLE_BFP_OF_REAL(- lateral_vel_x);
  }
  if (var_time > invert_time) {
    dr_ctrl.cmd.phi = ANGLE_BFP_OF_REAL(phi1);
    dr_ctrl.cmd.theta = ANGLE_BFP_OF_REAL(- lateral_vel_x);
  }  
  if (var_time > invert_time / 0.45) {
    dr_ctrl.cmd.phi = ANGLE_BFP_OF_REAL(0);
    dr_ctrl.cmd.theta = ANGLE_BFP_OF_REAL(0);  
  }
  #endif

  stabilization_attitude_set_rpy_setpoint_i(&(dr_ctrl.cmd));
  stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}


#if 1

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
}

float sratio = 0.2;
float eratio = 0.01;

void guidance_v_module_run(bool in_flight)
{ 
  
  float nominal = radio_control.values[RADIO_THROTTLE];
  float flap = 0.85;
  stabilization_cmd[COMMAND_THRUST] = nominal / (cosf(dr_state.phi * flap) * cosf(dr_state.theta * flap));
  
  
  // Helicopter dynamics
  /*
  float aoa = (dr_state.phi);
  // dronerace_state_struct testdrone;

  if (var_time < 1.2) {

    stabilization_cmd[COMMAND_THRUST] = nominal / (cosf(dr_state.phi) * cosf(dr_state.theta)); // - sratio * aoa * vel * nominal ; //(cosf(dr_state.theta) * cosf(dr_state.phi));

  }
  // if (var_time >= 0.8 && var_time < 1.3) {
  //   stabilization_cmd[COMMAND_THRUST] = 1.35 * nominal / (cosf(roll) * cosf(pitch));  // because drag is helping to slow down (hopefully no downwash vortex)
  // }   

  if (var_time >= 1.2 && var_time < 2.4) {
    // float varafds = eratio * nominal * vel * aoa;
    stabilization_cmd[COMMAND_THRUST] = nominal / (cosf(dr_state.phi) * cosf(dr_state.theta)) + eratio * nominal * vel;  // 1.03 because drag is helping to slow down (hopefully no downwash vortex)
    // printf("comp %f\n", varafds);
    // printf("cmd: %d\n", stabilization_cmd[COMMAND_THRUST]);
  }   
    if (var_time >= 2.2 && var_time < 2.4) {
    // float varafds = eratio * nominal * vel * aoa;
    stabilization_cmd[COMMAND_THRUST] = nominal;  // 1.03 because drag is helping to slow down (hopefully no downwash vortex)
    // printf("comp %f\n", varafds);
    // printf("cmd: %d\n", stabilization_cmd[COMMAND_THRUST]);
  }   


  if (var_time >= 2.4 && var_time < 2.5) {
    stabilization_cmd[COMMAND_THRUST] = 1.3 * nominal; // just ..
  }
  // if (time_dr_state > 1.64 && time_dr_state < 2.5) {
  //   stabilization_cmd[COMMAND_THRUST] = nominal * 0.9;
  // }
  
  // your vertical controller goes here
  */
}

#endif