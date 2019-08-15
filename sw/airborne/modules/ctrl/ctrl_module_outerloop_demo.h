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
 * @file modules/ctrl/ctrl_module_outerloop_demo.c
 * @brief example empty controller
 *
 * Implements an example simple horizontal outerloop controller in a module.
 */

#ifndef CTRL_MODULE_OUTERLOOP_DEMO_H_
#define CTRL_MODULE_OUTERLOOP_DEMO_H_

#include <std.h>
#include "state.h"
#include "autopilot.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

// Settings
extern float comode_time;

struct ctrl_module_demo_struct {
  // RC Inputs
  struct Int32Eulers rc_sp;

  // Output command
  struct Int32Eulers cmd;
};

extern struct ctrl_module_demo_struct dr_ctrl;

// Demo with own guidance_h
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

// But re-using an existing altitude-hold controller
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER
//#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);



// Settings
extern float ctrl_module_demo_pr_ff_gain;  // Pitch/Roll
extern float ctrl_module_demo_pr_d_gain;
extern float ctrl_module_demo_y_ff_gain;   // Yaw
extern float ctrl_module_demo_y_d_gain;

#if 0
// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);
#endif

#endif /* CTRL_MODULE_OUTERLOOP_DEMO_H_ */
