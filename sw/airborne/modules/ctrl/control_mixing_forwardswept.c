/*
 * Copyright (C) 2024 Noah Wechtler <noahwechtler@tudelft.nl>
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

/** @file "modules/ctrl/control_mixing_forwardswept.c"
 * @author Noah Wechtler <noahwechtler@tudelft.nl>
 * Control mixing specific to the Forward swept drone
 */

#include "modules/ctrl/control_mixing_forwardswept.h"
#include "modules/radio_control/radio_control.h"
#include "generated/modules.h"
#include "modules/core/commands.h"
#include "autopilot.h"
#include "state.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_plane.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"

void control_mixing_forwardswept_init(void) {
  force_forward = true;
}

void control_mixing_forwardswept_attitude_direct_enter(void) {
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_FORWARD);
}

void control_mixing_forwardswept_attitude_direct_run(void) {

  struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());

  stabilization_attitude_run(true, &stabilization.rc_sp, &th_sp, stabilization.cmd);
  autopilot.throttle = commands[COMMAND_THRUST];
}

void control_mixing_forwardswept_rate_enter(void) {
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_RATE, STABILIZATION_ATT_SUBMODE_FORWARD);
}

void control_mixing_forwardswept_rate_run(void) {
  
  struct StabilizationSetpoint stab_sp = stabilization_rate_read_rc(&radio_control);
  struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());

  stabilization_rate_run(autopilot_in_flight(), &stab_sp, &th_sp, stabilization.cmd);
  autopilot.throttle = commands[COMMAND_THRUST];
}

void control_mixing_forwardswept_nav_enter(void) {
  guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
  guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_FORWARD);
}

void control_mixing_forwardswept_nav_run(void) {
  struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());
  struct StabilizationSetpoint stab_sp = guidance_plane_attitude_from_nav(autopilot_in_flight());
  
  stabilization_run(autopilot_in_flight(), &stab_sp, &th_sp, stabilization.cmd);

  commands[COMMAND_ROLL] = stabilization.cmd[COMMAND_ROLL];
  commands[COMMAND_PITCH] = stabilization.cmd[COMMAND_PITCH];
  commands[COMMAND_YAW] = stabilization.cmd[COMMAND_YAW];

  autopilot.throttle = commands[COMMAND_THRUST];
}
