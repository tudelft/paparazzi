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
 * @file delftacopter_controller.c
 * @brief State feedback controller for Delftacopter
 *
 */

#include "delftacopter_controller.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"

#include "delftacopter_observer.h"
#include "system_matrix.h"


float _controller_ref[SYSTEM_N_OUTPUTS][1]; // Current reference body rates (roll, pitch)
float _controller_output[SYSTEM_N_INPUTS][1]; // Current controller output
int32_t _controller_output_pprz[SYSTEM_N_INPUTS][1]; // Current controller output, pprz value
float _codx[SYSTEM_N_INPUTS][1]; // controller output due to state; temporary matrix
float _codu[SYSTEM_N_INPUTS][1]; // controller output due to reference; temporary matrix

// Pointers to above matrices so they can be used in pprz_algebra_float functions
float* controller_K[SYSTEM_N_INPUTS];
float* controller_g[SYSTEM_N_INPUTS];
float* controller_ref[SYSTEM_N_OUTPUTS];
float* controller_output[SYSTEM_N_INPUTS];
int32_t* controller_output_pprz[SYSTEM_N_INPUTS];
float* codx[SYSTEM_N_INPUTS];
float* codu[SYSTEM_N_INPUTS];

uint8_t current_controller_active = DELFTACOPTER_CONTROLLER_OLD_RATE;
float controller_g_factor = 1.0;
float k_phi_dc = 2;
float k_theta_dc = 2;

static void send_DC_CONTROLLER(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_DC_CONTROLLER(trans, dev, AC_ID,
    &current_controller_active,               // controller_type 
    &controller_matrix_id,                    // matrix_cont
    &controller_ref[0][0],                    // ref_p
    &controller_ref[1][0],                    // ref_q
    &delftacopter_observer.measured_y[0][0],  // meas_p
    &delftacopter_observer.measured_y[1][0],  // meas_q
    &controller_error_quat_x,                 // err_x
    &controller_error_quat_y,                 // err_y
    &controller_error_quat_z);                // err_z
}

static void send_DC_CONT_SETTINGS(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_DC_CONT_SETTINGS(trans, dev, AC_ID,
    &controller_g_factor,                     // g_factor
    &k_phi_dc,                                // K_phi
    &k_theta_dc);                             // K_theta
}

void delftacopter_controller_init(void) {
  delftacopter_observer_init();

  INIT_MATRIX_PTR(controller_K, _controller_K, SYSTEM_N_INPUTS);
  INIT_MATRIX_PTR(controller_g, _controller_g, SYSTEM_N_INPUTS);
  INIT_MATRIX_PTR(controller_ref, _controller_ref, SYSTEM_N_OUTPUTS);
  INIT_MATRIX_PTR(controller_output, _controller_output, SYSTEM_N_INPUTS);
  INIT_MATRIX_PTR(controller_output_pprz, _controller_output_pprz, SYSTEM_N_INPUTS);
  INIT_MATRIX_PTR(codx, _codx, SYSTEM_N_INPUTS);
  INIT_MATRIX_PTR(codu, _codu, SYSTEM_N_INPUTS);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DC_CONTROLLER, send_DC_CONTROLLER);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DC_CONT_SETTINGS, send_DC_CONT_SETTINGS);
}

int32_t** get_controller_output(float p_ref, float q_ref)
{
  // Multiply the reference p and q by a sensitivity, to make it somewhat equal to the old rate controller
  _controller_ref[0][0] = p_ref * controller_g_factor;
  _controller_ref[1][0] = q_ref * controller_g_factor;

  // Controller logic
  #ifdef DC_STATE_FEEDBACK
    float_mat_mul(codx, controller_K, delftacopter_observer.estimate_x, SYSTEM_N_INPUTS, CONTROLLER_N_STATES, 1);
  #endif
  #ifdef DC_OUTPUT_FEEDBACK
    float_mat_mul(codx, controller_K, delftacopter_observer.measured_y, SYSTEM_N_INPUTS, CONTROLLER_N_STATES, 1);
  #endif
  float_mat_mul(codu, controller_g, controller_ref, SYSTEM_N_INPUTS, SYSTEM_N_OUTPUTS, 1);
  float_mat_sum(controller_output, codx, codu, SYSTEM_N_INPUTS, 1);

  // _controller_output[0][0] += DC_SWASH_OFFSET_X;
  // _controller_output[1][0] += DC_SWASH_OFFSET_Y;

  // change to pprz values
  for (int __i = 0; __i < 3; __i++)
    _controller_output_pprz[__i][0] = (int32_t) (_controller_output[__i][0] * 9600);

  return controller_output_pprz;
}

void set_attitude_controller_error(float err_x, float err_y, float err_z) {
  controller_error_quat_x = err_x;
  controller_error_quat_y = err_y;
  controller_error_quat_z = err_z;
}

void set_active_controller(uint8_t new_dc_controller_type) {
  current_controller_active = new_dc_controller_type;
}
