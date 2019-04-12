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


// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct Int32Eulers rc_sp;

  float phi;
  float theta;
  float psi;
  float alt;

// Output command
  struct Int32Eulers cmd;

} ctrl;


// Settings
float comode_time = 0;

#include "./dronerace/dronerace.h"


////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
}

void guidance_h_module_enter(void)
{
  // Store current heading
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  //stabilization_attitude_read_rc_setpoint_eulers(&rc_sp, in_flight, false, false);

  dronerace_enter();
}

void guidance_h_module_read_rc(void)
{
  //stabilization_attitude_read_rc_setpoint_eulers(&rc_sp, in_flight, false, false);
}


void guidance_h_module_run(bool in_flight)
{
  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);
  dronerace_get_cmd(&ctrl.alt, &ctrl.phi, &ctrl.theta, &ctrl.psi);

  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(ctrl.phi);
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(ctrl.theta);
  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(ctrl.psi);

  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
  stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}

