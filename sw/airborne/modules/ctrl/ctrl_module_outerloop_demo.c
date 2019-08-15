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

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  dronerace_init();
  float psi_init = stateGetNedToBodyEulers_f()->psi;
} 




void guidance_h_module_enter(void)
{
  // Store current heading
  dr_ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
  dronerace_enter();
}

void guidance_h_module_read_rc(void)
{
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}


float alt = 0;
float roll = 0;
float pitch = 0;
float yaw = 0;
void guidance_h_module_run(bool in_flight)
{
  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);

  dronerace_get_cmd(&alt, &roll, &pitch, &yaw);

  dr_ctrl.cmd.phi   = ANGLE_BFP_OF_REAL(roll);
  dr_ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch); //-ANGLE_BFP_OF_REAL(5*3.142/180);
  dr_ctrl.cmd.psi   = ANGLE_BFP_OF_REAL(yaw);   // stateGetNedToBodyEulers_f()->psi;//

  stabilization_attitude_set_rpy_setpoint_i(&(dr_ctrl.cmd));
  stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}

#if 0
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
  float flap = 0.85;
  stabilization_cmd[COMMAND_THRUST] = nominal / (cosf(dr_state.phi * flap) * cosf(dr_state.theta * flap));
}
#endif