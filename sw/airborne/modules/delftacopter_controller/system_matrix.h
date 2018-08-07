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
 * @file system_matrix.h
 * @brief Here the Delftacopter observer and controller matrices are defined
 * This is mainly done to have a single place for choosing which matrix to use
 * i.e. for the DC3 different matrices are required.
 */

#ifndef SYSTEM_MATRIX_H
#define SYSTEM_MATRIX_H

#include <stdint.h>

/** 
 * System: Delftacopter inputs [roll, pitch, elevator] 
 *                      outputs [p, q]
 * Observer:            inputs [roll, pitch, elevator inputs, p_measured, q_measured]
 *                      states [p, q, a, b]
 *                      outputs [p, q, a, b] (all states)
 */
#ifdef DC_STATE_FEEDBACK
#define SYSTEM_N_INPUTS 3
#define SYSTEM_N_OUTPUTS 2
#define OBSERVER_N_STATES 4
#define CONTROLLER_N_STATES OBSERVER_N_STATES
#endif
#ifdef DC_OUTPUT_FEEDBACK
#define SYSTEM_N_INPUTS 3
#define SYSTEM_N_OUTPUTS 2
#define OBSERVER_N_STATES 4
#define CONTROLLER_N_STATES SYSTEM_N_OUTPUTS
#endif
#define OBSERVER_N_INPUTS SYSTEM_N_INPUTS + SYSTEM_N_OUTPUTS
#define OBSERVER_N_OUTPUTS OBSERVER_N_STATES

extern float _A_obsv_hover[][OBSERVER_N_STATES];
extern float _B_obsv_hover[][OBSERVER_N_INPUTS];
extern float _C_obsv_hover[][OBSERVER_N_STATES];
extern float _controller_K[][CONTROLLER_N_STATES];
extern float _controller_g[][SYSTEM_N_OUTPUTS];

extern uint16_t controller_matrix_id;
extern uint16_t observer_matrix_id;

#endif // SYSTEM_MATRIX_H
