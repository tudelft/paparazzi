/*
 * Copyright (C) 2018 Joost Meulenbeld
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file delftacopter_controller.h
 * @brief State feedback controller for Delftacopter
 * 
 * The feedback law is given by delta = g * cmd - K * x
 * g is the transformation matrix, transforming roll/pitch commands to system inputs
 * x is the estimated state from the state observer, K is the feedback matrix
 *
 */

#ifndef DELFTACOPTER_CONTROLLER_H
#define DELFTACOPTER_CONTROLLER_H

#include <std.h>
#include <stdint.h>

#define DELFTACOPTER_CONTROLLER_OLD_RATE 0
#define DELFTACOPTER_CONTROLLER_NEW_RATE 1
#define DELFTACOPTER_CONTROLLER_OLD_ATT 2
#define DELFTACOPTER_CONTROLLER_NEW_ATT 3

extern float controller_g_factor;
extern float k_phi_dc;
extern float k_theta_dc;

// Initialize all matrix pointer and the observer
void delftacopter_controller_init(void);

/**
 * @brief Get the controller values for [roll, pitch, elevator]
 * @param p_ref, qref: pprz reference values for roll and pitch rates
 */
int32_t** get_controller_output(float p_ref, float q_ref);

// Update attitude controller error
void set_attitude_controller_error(float err_x, float err_y, float err_z);

// Set the currently active controller (use the defines above as values)
void set_active_controller(uint8_t new_dc_controller_type);

#endif // DELFTACOPTER_CONTROLLER_H
