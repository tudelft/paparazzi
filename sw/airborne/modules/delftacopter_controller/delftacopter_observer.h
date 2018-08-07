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
 * @file delftacopter_observer.h
 * @brief Luenberger observer for a and b angles of rotor blade
 *
 */

#ifndef DELFTACOPTER_OBSERVER_H
#define DELFTACOPTER_OBSERVER_H

#include <std.h>
#include <stdint.h>

#include "math/pprz_algebra_float.h"
#include "filters/low_pass_filter.h"
#include "state.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"
#include "subsystems/datalink/telemetry.h"

#include "state_space.h"
#include "system_matrix.h"

/**
 * Number of samples that the servo actuator lags behind the command signal
 */
#define DC_SWASH_INPUT_DELAY 10

/** 
 * Offsets of inputs
 */
#define DC_SWASH_OFFSET_X -0.015
#define DC_SWASH_OFFSET_Y 0.04

/** Make a pointer to a matrix of _rows lines
 * @param _ptr The pointer which will be pointing
 * @param _mat The matrix that is allocated on the stack
 * @param _rows Number of rows of the matrix
*/
#define INIT_MATRIX_PTR(_ptr, _mat, _rows) \
    for (int __i = 0; __i < _rows; __i++) { _ptr[__i] = &_mat[__i][0]; }

enum delftacopter_flight_phase {delftacopter_hover=1, delftacopter_forward=2};

struct delftacopter_observer_t {
  enum delftacopter_flight_phase phase; // Used for setting the correct matrices (each flight phase has its own matrix)
  struct state_space_t ss; // The discrete state-space system that performs the observer calculations

  float** measured_u; // [roll, pitch, elevator inputs]
  float** delayed_u; // [roll, pitch, elevator inputs] DC2_SWASH_INPUT_DELAY samples behind (10)
  float** measured_y; // [p_measured, q_measured]
  float** filtered_y; // [p_filtered, q_filtered]
  float** observer_u; // [roll, pitch, elevator inputs, p_measured, q_measured]
  float** estimate_x; // [p, q, a, b]

  float observer_dt; // delta-time between the last two observer propagate() calls (for debugging)
};

extern struct delftacopter_observer_t delftacopter_observer;

extern float controller_error_quat_x;
extern float controller_error_quat_y;
extern float controller_error_quat_z;

// Initialize the observer; set observer mode to hover
void delftacopter_observer_init(void);

// Set the observer matrices based on the flight phase
void set_observer_matrices(enum delftacopter_flight_phase new_phase);

/** Concatenate two vectors
 * o = [a; b] 
 * a: [m x 1]
 * b: [n x 1]
*/
void concat_v(float** o, float** a, float** b, int m, int n);

// Propagate the observer state estimate based on measurements and system input
void propagate_observer(int32_t roll_cmd, int32_t pitch_cmd, int32_t elevator_cmd);

// Update the current delfatcopter input values. Use pprz values as arguments
void update_input_vector(int32_t roll_cmd, int32_t pitch_cmd, int32_t elevator_cmd);

// Update the current roll and pitch measurements (observer gets it from state.h itself)
void update_measurement_vector(void);

#endif // DELFTACOPTER_OBSERVER_H
