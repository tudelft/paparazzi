/*
 * Copyright (C) Huizerd
 * Copyright (C) Kirk Scheper
 * Copyright (C) Guido de Croon
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
/**
 * @file "modules/pid_landing/pid_landing.c"
 * @author Huizerd
 * Proportional controller for optical flow landing.
 */

// Header for this file
#include "modules/pid_landing/pid_landing.h"

// Paparazzi headers
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"

// Used for automated landing:
#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "subsystems/datalink/telemetry.h"

// For measuring time
#include "mcu_periph/sys_time.h"

// C standard library headers
#include <stdbool.h>
#include <stdio.h>

// Use optical flow estimates
#ifndef PL_OPTICAL_FLOW_ID
#define PL_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(PL_OPTICAL_FLOW_ID)

// Other default values
// Closed-loop thrust control, else linear transform
#define PL_ACTIVE_CONTROL true

// Gains and setpoint for optical flow control
#ifndef PL_P_GAIN
#define PL_P_GAIN 0.4f
#endif
#ifndef PL_I_GAIN
#define PL_I_GAIN 0.0f
#endif
#ifndef PL_D_GAIN
#define PL_D_GAIN 0.0f
#endif
#ifndef PL_DIV_SP
#define PL_DIV_SP 0.5f
#endif

// Gains for closed-loop control
#ifndef PL_THRUST_EFFECT
#define PL_THRUST_EFFECT 0.05f
#endif
#ifndef PL_THRUST_P_GAIN
#define PL_THRUST_P_GAIN 0.7f
#endif
#ifndef PL_THRUST_I_GAIN
#define PL_THRUST_I_GAIN 0.3f
#endif

// Optical flow settings
#ifndef PL_OF_FILTER_CUTOFF
#define PL_OF_FILTER_CUTOFF 1.5f
#endif

// Events
static abi_event optical_flow_ev;

// Low-pass filters for acceleration and thrust
static Butterworth2LowPass accel_ned_filt;
static Butterworth2LowPass thrust_filt;

// Variables retained between module calls
// For divergence + derivative, low-passed acceleration, thrust
float divergence, divergence_dot, acc_lp, thrust, thrust_lp;
float acceleration_sp;
float div_gt, divdot_gt;
float div_gt_tmp;
// Spike count --> not used
uint16_t spike_count;
// For recording
uint8_t record;
// For control
static float nominal_throttle;
static bool active_control;

// And struct to hold settings
struct PIDLandingSettings pl_settings;
// And one to hold errors
struct PIDErrors pl_errors;

// Sending stuff to ground station
// Divergence + derivative, height, velocity, acceleration, thrust, mode
static void send_pl(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_SPIKING_LANDING(
      trans, dev, AC_ID, &divergence, &divergence_dot,
      &(stateGetPositionNed_f()->z), &(stateGetPositionEnu_f()->z),
      &(state.ned_origin_f.hmsl), &(stateGetSpeedNed_f()->z),
      &(stateGetAccelNed_f()->z), &accel_ned_filt.o[0], &thrust,
      &autopilot.mode, &record);
}

// Function definitions
// Callback function of optical flow estimate (bound to optical flow ABI
// messages)
static void pl_optical_flow_cb(uint8_t sender_id, uint32_t stamp,
                               int16_t UNUSED flow_x, int16_t UNUSED flow_y,
                               int16_t UNUSED flow_der_x,
                               int16_t UNUSED flow_der_y, float UNUSED quality,
                               float size_divergence);

// Spiking landing module functions
static void pl_init(void);
static void pl_run(float divergence, float divergence_dot);

// Proportional divergence control
static float pl_divergence_control(float divergence, float P, float I, float D, float dt);

// Closed-loop, active thrust control
static void pl_control(void);

// Init global variables
static void init_globals(void);

// Module initialization function
static void pl_init() {
  // Fill settings
  pl_settings.p_gain = PL_P_GAIN;
  pl_settings.i_gain = PL_I_GAIN;
  pl_settings.d_gain = PL_D_GAIN;
  pl_settings.div_setpoint = PL_DIV_SP;
  pl_settings.thrust_effect = PL_THRUST_EFFECT;
  pl_settings.thrust_p_gain = PL_THRUST_P_GAIN;
  pl_settings.thrust_i_gain = PL_THRUST_I_GAIN;

  // Fill errors
  pl_errors.sum_err = 0.0f;
  pl_errors.d_err = 0.0f;
  pl_errors.prev_err  = 0.0f;

  // Init global variables
  init_globals();

  // Register telemetry message
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SPIKING_LANDING,
                              send_pl);

  // Subscribe to optical flow estimation
  AbiBindMsgOPTICAL_FLOW(PL_OPTICAL_FLOW_ID, &optical_flow_ev,
                         pl_optical_flow_cb);

  // Init low-pass filters for acceleration and thrust
  float tau = 1.0f / (2.0f * M_PI * PL_OF_FILTER_CUTOFF);
  float ts = 1.0f / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_ned_filt, tau, ts, 0.0f);
  init_butterworth_2_low_pass(&thrust_filt, tau, ts, 0.0f);
}

// Reset global variables (e.g., when starting/re-entering module)
static void init_globals() {
  divergence = 0.0f;
  divergence_dot = 0.0f;
  div_gt = 0.0f;
  divdot_gt = 0.0f;
  div_gt_tmp = 0.0f;
  thrust = 0.0f;
  spike_count = 0;
  acc_lp = 0.0f;
  thrust_lp = 0.0f;
  acceleration_sp = 0.0f;
  record = 0;
  nominal_throttle = guidance_v_nominal_throttle;
  active_control = false;
}

// Get optical flow estimate from sensors via callback
static void pl_optical_flow_cb(uint8_t sender_id, uint32_t stamp,
                               int16_t UNUSED flow_x, int16_t UNUSED flow_y,
                               int16_t UNUSED flow_der_x,
                               int16_t UNUSED flow_der_y, float UNUSED quality,
                               float size_divergence) {
  // Compute time step
  static uint32_t last_stamp = 0;
  float dt = (stamp - last_stamp) / 1e6f;
  last_stamp = stamp;

  // Compute derivative of divergence and divergence
  if (dt > 1e-5f) {
    divergence_dot = (2.0f * size_divergence - divergence) / dt;
  }
  divergence = 2.0f * size_divergence;

  // Compute GT of divergence + derivative
  if (fabsf(stateGetPositionNed_f()->z) > 1e-5f) {
    div_gt_tmp = -2.0f * stateGetSpeedNed_f()->z / stateGetPositionNed_f()->z;
  }
  if (dt > 1e-5f) {
    divdot_gt = (div_gt_tmp - div_gt) / dt;
  }
  div_gt = div_gt_tmp;

  // Run the proportional controller
  pl_run(divergence, dt);
}

// Run the proportional controller
static void pl_run(float divergence, float dt) {
  // These "static" types are great!
  static bool first_run = true;
  static float start_time = 0.0f;
  static float nominal_throttle_sum = 0.0f;
  static float nominal_throttle_samples = 0.0f;

  // TODO: is this for resetting altitude?
  if (autopilot_get_mode() != AP_MODE_GUIDED) {
    first_run = true;
    active_control = false;
    record = 0;
    return;
  }

  // TODO: here we reset the network in between runs!
  if (first_run) {
    start_time = get_sys_time_float();
    nominal_throttle = (float)stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
    pl_errors.sum_err = 0.0f;
    pl_errors.d_err = 0.0f;
    pl_errors.prev_err = 0.0f;
    first_run = false;
  }

  // Let the vehicle settle
  if (get_sys_time_float() - start_time < 5.0f) {
    return;
  }

  // After vehicle settling, compute and improve nominal throttle estimate
  if (get_sys_time_float() - start_time < 10.0f) {
    nominal_throttle_sum += (float)stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
    nominal_throttle_samples++;
    nominal_throttle = nominal_throttle_sum / nominal_throttle_samples;
    return;
  }

  // Set recording while in flight
  if (autopilot.in_flight) {
    record = 1;
  } else {
    record = 0;
  }

  // Proportional divergence control
  // Converting to G's and clamping happens here, in simulation this was done in
  // environment
  thrust = pl_divergence_control(divergence, pl_settings.p_gain, pl_settings.i_gain, pl_settings.d_gain, dt);

  // Bound thrust to limits (-0.8g, 0.5g)
  Bound(thrust, -7.848f, 4.905f);

  // Set control mode: active closed-loop control or linear transform
  if (PL_ACTIVE_CONTROL) {
    active_control = true;
  } else {
    guidance_v_set_guided_th(thrust * pl_settings.thrust_effect +
                             nominal_throttle);
  }
}

// Proportional divergence control
static float pl_divergence_control(float divergence, float P, float I, float D, float dt) {
  // Determine the error
  float err = divergence - pl_settings.div_setpoint;

  // Update errors
  // NOTE: this is only used when I and D gains are nonzero
  // Low-pass factor
  float lp_factor = dt / 0.02;
  Bound(lp_factor, 0.0f, 1.0f);

  // Maintain controller errors
  pl_errors.sum_err += err;
  pl_errors.d_err += (((err - pl_errors.prev_err) / dt) - pl_errors.d_err) * lp_factor;
  pl_errors.prev_err = err;

  // PID control relative to acceleration setpoint
  float thrust = 0.0f + P * err + I * pl_errors.sum_err + D * pl_errors.d_err;

  return thrust;
}

// Closed-loop PI control for going from acceleration to motor control
static void pl_control() {
  // "static" here implies that value is kept between function invocations
  static float error_integrator = 0.0f;

  // Low-pass filters for current acceleration and thrust setpoint
  struct NedCoor_f *acceleration = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&accel_ned_filt, acceleration->z);
  update_butterworth_2_low_pass(&thrust_filt, thrust);
  acc_lp = accel_ned_filt.o[0];
  thrust_lp = thrust_filt.o[0];

  // Proportional
  float error = thrust_filt.o[0] + accel_ned_filt.o[0];
  BoundAbs(error, 1.0f / (pl_settings.thrust_p_gain + 0.01f));

  // Integral
  error_integrator += error / PERIODIC_FREQUENCY;
  BoundAbs(error_integrator, 1.0f / (pl_settings.thrust_i_gain + 0.01f));

  // Acceleration setpoint
  acceleration_sp = (thrust + error * pl_settings.thrust_p_gain +
                           error_integrator * pl_settings.thrust_i_gain) *
                              pl_settings.thrust_effect +
                          nominal_throttle;

  // Perform active closed-loop control or do simple linear transform
  if (active_control) {
    guidance_v_set_guided_th(acceleration_sp);
  } else {
    error_integrator = 0.0f;
  }
}

// Module functions
// Init
void pid_landing_init() { pl_init(); }

// Run
void pid_landing_event() { pl_control(); }
