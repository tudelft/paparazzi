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
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "autopilot.h"

#include "./dronerace/dronerace.h"

// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct Int32Eulers rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;


// Settings
float comode_time = 0;


////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  dronerace_init();
}

void guidance_h_module_enter(void)
{
  // Store current heading
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
  dronerace_enter();
}

void guidance_h_module_read_rc(void)
{
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

float time_var = 0.0;
void guidance_h_module_run(bool in_flight)
{
  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);
  float alt = 0.0;
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;

  dronerace_get_cmd(&alt, &roll, &pitch, &yaw, &time_var);

  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll);
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch);//-ANGLE_BFP_OF_REAL(5*3.142/180);//ANGLE_BFP_OF_REAL(pitch);
  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(yaw); // stateGetNedToBodyEulers_f()->psi;//
  
  float abey = 45*3.142/180;
  if (time_var < 0.8) {
    ctrl.cmd.phi = ANGLE_BFP_OF_REAL(abey);
  }
  if (time_var > 0.8) {
    ctrl.cmd.phi = ANGLE_BFP_OF_REAL(-abey);
  }  
  if (time_var > 1.6) {
    ctrl.cmd.phi = 0;
  }

  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
  stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}


void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(bool in_flight)
{ 
  float nominal = radio_control.values[RADIO_THROTTLE];
  stabilization_cmd[COMMAND_THRUST] = nominal;
  if (time_var < 0.8) {
    stabilization_cmd[COMMAND_THRUST] = nominal * 1.2;
  }
  if (time_var > 0.8 && time_var < 1.6) {
    stabilization_cmd[COMMAND_THRUST] = nominal * 1.3;
  }  

  if (time_var >= 1.6 && time_var < 1.65) {
    stabilization_cmd[COMMAND_THRUST] = nominal * 1.1;
  }
  // if (time_var > 1.64 && time_var < 2.5) {
  //   stabilization_cmd[COMMAND_THRUST] = nominal * 0.9;
  // }
  
  // your vertical controller goes here
}
