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
 * @file delftacopter_observer.c
 * @brief Luenberger observer for a and b angles of rotor blade
 *
 */

#include "delftacopter_observer.h"

float* A_obsv_hover[OBSERVER_N_STATES];
float* B_obsv_hover[OBSERVER_N_STATES];
float* C_obsv_hover[OBSERVER_N_OUTPUTS];

// State-space temporary computation vectors
float _obsvx[OBSERVER_N_STATES][1];
float _obsvu[OBSERVER_N_STATES][1];
float _obsvy[OBSERVER_N_OUTPUTS][1];
float* obsvx[OBSERVER_N_STATES];
float* obsvu[OBSERVER_N_STATES];
float* obsvy[OBSERVER_N_OUTPUTS];

float _xdx[OBSERVER_N_STATES][1];
float _xdu[OBSERVER_N_STATES][1];
float _ydx[OBSERVER_N_OUTPUTS][1];
float* xdx[OBSERVER_N_STATES];
float* xdu[OBSERVER_N_STATES];
float* ydx[OBSERVER_N_OUTPUTS];

// Observer storage vectors
float _measured_u[SYSTEM_N_INPUTS][1];
float _delayed_u[SYSTEM_N_INPUTS][1];
float _measured_y[SYSTEM_N_OUTPUTS][1];
float _filtered_y[SYSTEM_N_OUTPUTS][1];
float _observer_u[OBSERVER_N_INPUTS][1];
float* measured_u[SYSTEM_N_INPUTS];
float* delayed_u[SYSTEM_N_INPUTS];
float* measured_y[SYSTEM_N_OUTPUTS];
float* filtered_y[SYSTEM_N_OUTPUTS];
float* observer_u[OBSERVER_N_INPUTS];

// Circular buffer for input
float delftacopter_input_buffer[SYSTEM_N_INPUTS][DC_SWASH_INPUT_DELAY];
uint8_t delftacopter_buffer_index = 0;

float observer_dt = 0;
static float observer_prev_time = 0;

struct delftacopter_observer_t delftacopter_observer;

const float controller_freq = 512; // PERIODIC_FREQUENCY;

float controller_error_quat_x = 0;
float controller_error_quat_y = 0;
float controller_error_quat_z = 0;

// #if PERIODIC_FREQUENCY != controller_freq
// #warning "PERIODIC_FREQUENCY of delftacopter_observer should be 512 but is " PERIODIC_FREQUENCY
// #endif

static void send_DC_OBSERVER(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_DC_OBSERVER(trans, dev, AC_ID,
    &observer_matrix_id, // observer matrix
    &delftacopter_observer.measured_u[0][0], // delt_x
    &delftacopter_observer.measured_u[1][0], // delt_y
    &delftacopter_observer.measured_u[2][0], // delt_e
    &delftacopter_observer.measured_y[0][0], // meas_p
    &delftacopter_observer.measured_y[1][0], // meas_q
    &delftacopter_observer.estimate_x[0][0], // obsv_p
    &delftacopter_observer.estimate_x[1][0], // obsv_q
    &delftacopter_observer.estimate_x[2][0], // obsv_a
    &delftacopter_observer.estimate_x[3][0], // obsv_b
    &delftacopter_observer.observer_dt);     // Time between two observer propagations
}

void delftacopter_observer_init(void) {
  delftacopter_observer.phase = delftacopter_hover;
  delftacopter_observer.ss = state_space_init(OBSERVER_N_STATES, OBSERVER_N_INPUTS, OBSERVER_N_OUTPUTS, A_obsv_hover, B_obsv_hover, C_obsv_hover, obsvx, obsvu, obsvy, xdx, xdu, ydx);

  INIT_MATRIX_PTR(A_obsv_hover, _A_obsv_hover, OBSERVER_N_STATES)
  INIT_MATRIX_PTR(B_obsv_hover, _B_obsv_hover, OBSERVER_N_STATES)
  INIT_MATRIX_PTR(C_obsv_hover, _C_obsv_hover, OBSERVER_N_OUTPUTS)

  INIT_MATRIX_PTR(obsvx, _obsvx, OBSERVER_N_STATES);
  INIT_MATRIX_PTR(obsvu, _obsvu, OBSERVER_N_STATES);
  INIT_MATRIX_PTR(obsvy, _obsvy, OBSERVER_N_OUTPUTS);
  INIT_MATRIX_PTR(xdx, _xdx, OBSERVER_N_STATES);
  INIT_MATRIX_PTR(xdu, _xdu, OBSERVER_N_STATES);
  INIT_MATRIX_PTR(ydx, _ydx, OBSERVER_N_OUTPUTS);

  INIT_MATRIX_PTR(measured_u, _measured_u, SYSTEM_N_INPUTS);
  INIT_MATRIX_PTR(delayed_u, _delayed_u, SYSTEM_N_INPUTS);
  INIT_MATRIX_PTR(delayed_u, _delayed_u, SYSTEM_N_INPUTS);
  INIT_MATRIX_PTR(measured_y, _measured_y, SYSTEM_N_OUTPUTS);
  INIT_MATRIX_PTR(filtered_y, _filtered_y, SYSTEM_N_OUTPUTS);
  INIT_MATRIX_PTR(observer_u, _observer_u, OBSERVER_N_INPUTS);

  delftacopter_observer.measured_u = measured_u;
  delftacopter_observer.delayed_u = delayed_u;
  delftacopter_observer.measured_y = measured_y;
  delftacopter_observer.filtered_y = filtered_y;
  delftacopter_observer.observer_u = observer_u;
  delftacopter_observer.estimate_x = delftacopter_observer.ss.x;

  for (uint8_t __i = 0; __i < SYSTEM_N_INPUTS; __i++) {
    for (uint8_t __j = 0; __j < DC_SWASH_INPUT_DELAY; __j++) {
      delftacopter_input_buffer[__i][__j] = 0;
    }
  }

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DC_OBSERVER, send_DC_OBSERVER);
}

void set_observer_matrices(enum delftacopter_flight_phase new_phase) {
  delftacopter_observer.phase = new_phase;

  switch (new_phase) {
    case delftacopter_hover:
      delftacopter_observer.ss.A = A_obsv_hover;
      delftacopter_observer.ss.B = B_obsv_hover;
      delftacopter_observer.ss.C = C_obsv_hover;
      break;
    default:
      // TODO error, but how?
      break;
  }
}

void concat_v(float** o, float** a, float** b, int m, int n)
{
  int i;
  for (i = 0; i < m; i++) {
    o[i][0] = a[i][0];
  }
  for (i = m; i < m+n; i++) {
    o[i][0] = b[i - m][0];
  }
}


void propagate_observer(int32_t roll_cmd, int32_t pitch_cmd, int32_t elevator_cmd) {
  float t = get_sys_time_float();
  delftacopter_observer.observer_dt = t - observer_prev_time;
  observer_prev_time = t;
  
  update_input_vector(roll_cmd, pitch_cmd, elevator_cmd);
  update_measurement_vector();

  concat_v(delftacopter_observer.observer_u, delftacopter_observer.delayed_u, delftacopter_observer.measured_y, SYSTEM_N_INPUTS, SYSTEM_N_OUTPUTS);
  state_space_propagate(&delftacopter_observer.ss, delftacopter_observer.observer_u);
}

void update_input_vector(int32_t roll_cmd, int32_t pitch_cmd, int32_t elevator_cmd) {
  delftacopter_observer.measured_u[0][0] = ((float) roll_cmd) / 9600.0;
  delftacopter_observer.measured_u[1][0] = ((float) pitch_cmd) / 9600.0;
  delftacopter_observer.measured_u[2][0] = ((float) elevator_cmd) / 9600.0;

  delftacopter_observer.measured_u[0][0] -= DC_SWASH_OFFSET_X;
  delftacopter_observer.measured_u[1][0] -= DC_SWASH_OFFSET_Y;

  uint8_t new_buffer_index = (delftacopter_buffer_index + 1) % DC_SWASH_INPUT_DELAY;
  for (uint8_t __i = 0; __i < SYSTEM_N_INPUTS; __i++) {
    delftacopter_input_buffer[__i][delftacopter_buffer_index] = delftacopter_observer.measured_u[__i][0];
    delftacopter_observer.delayed_u[__i][0] = delftacopter_input_buffer[__i][new_buffer_index];
  }
  delftacopter_buffer_index = new_buffer_index;
}

void update_measurement_vector(void) {
  struct FloatRates* rates = stateGetBodyRates_f();
  delftacopter_observer.measured_y[0][0] = rates->p;
  delftacopter_observer.measured_y[1][0] = rates->q;
}
