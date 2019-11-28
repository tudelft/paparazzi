/*
 * Copyright (C) Huizerd
 * Copyright (C) Kirk Scheper
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
 * @file "modules/spiking_landing/spiking_landing.c"
 * @author Huizerd
 * Spiking neural networks for optical flow landing.
 */

// Header for this file
#include "modules/spiking_landing/spiking_landing.h"

// tinysnn headers
#include "Network.h"

// Paparazzi headers
// TODO: do we need all this? And in what order?
//#include "navigation.h"
//#include "state.h"
//#include "subsystems/datalink/downlink.h"
//#include "subsystems/gps.h"
//#include "subsystems/gps/gps_datalink.h"

//#include "generated/flight_plan.h"
//#include "math/pprz_geodetic_double.h"
//#include "math/pprz_geodetic_int.h"

//#include "guidance/guidance_h.h"
//#include "guidance/guidance_indi.h"
//#include "guidance/guidance_v.h"

#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"

// Used for automated landing:
#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/navigation/common_flight_plan.h"

// For measuring time
#include "mcu_periph/sys_time.h"

// C standard library headers
#include <stdbool.h>
#include <stdio.h>

// Use optical flow estimates
#ifndef SL_OPTICAL_FLOW_ID
#define SL_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(SL_OPTICAL_FLOW_ID)

// Other default values
// Closed-loop thrust control, else linear transform
#define SL_ACTIVE_CONTROL true

// Gains for closed-loop control
#ifndef SL_THRUST_EFFECT
#define SL_THRUST_EFFECT 0.05f
#endif
#ifndef SL_THRUST_P_GAIN
#define SL_THRUST_P_GAIN 0.7f
#endif
#ifndef SL_THRUST_I_GAIN
#define SL_THRUST_I_GAIN 0.3f
#endif

// Optical flow settings
#ifndef SL_OF_FILTER_CUTOFF
#define SL_OF_FILTER_CUTOFF 1.5f
#endif

// Network configuration
// Layer sizes
#ifndef SL_NET_IN
#define SL_NET_IN 4
#endif
#ifndef SL_NET_HID
#define SL_NET_HID 20
#endif
#ifndef SL_NET_OUT
#define SL_NET_OUT 1
#endif
// File with network parameters
#ifndef SL_NET_FILE
#define SL_NET_FILE "sw/airborne/modules/spiking_landing/network.txt"
#endif

// Events
static abi_event optical_flow_ev;

// Low-pass filters for acceleration and thrust
static Butterworth2LowPass accel_ned_filt;
static Butterworth2LowPass thrust_filt;

// Variables retained between module calls
// For divergence + derivative, low-passed acceleration, thrust
float divergence, divergence_dot, acc_lp, thrust;
float div_gt, divdot_gt;
float div_gt_tmp;
// For recording
uint8_t record;
// For control
static float nominal_throttle;
static bool active_control;

// Declare network struct
Network net;
// And struct to hold settings
struct SpikingLandingSettings sl_settings;

// Sending stuff to ground station
// Divergence + derivative, height, velocity, acceleration, thrust, mode
static void send_sl(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_SPIKING_LANDING(trans, dev, AC_ID, &divergence, &divergence_dot,
                                &(stateGetPositionNed_f()->z),
                                &(stateGetSpeedNed_f()->z),
                                &(stateGetAccelNed_f()->z),
                                &accel_ned_filt.o[0], &thrust, &autopilot.mode, &record);
}

// Function definitions
// Callback function of optical flow estimate (bound to optical flow ABI
// messages)
static void sl_optical_flow_cb(uint8_t sender_id, uint32_t stamp,
                               int16_t UNUSED flow_x, int16_t UNUSED flow_y,
                               int16_t UNUSED flow_der_x,
                               int16_t UNUSED flow_der_y, float UNUSED quality,
                               float size_divergence);

// Spiking landing module functions
static void sl_init(void);
static void sl_run(float divergence, float divergence_dot);

// Closed-loop, active thrust control
static void sl_control(void);

// Init global variables
static void init_globals(void);

// Module initialization function
static void sl_init() {
  // Build network
  net = build_network(SL_NET_IN, SL_NET_HID, SL_NET_OUT);
  // Init network
  init_network(&net);
  // Load network parameters
  load_network(&net, SL_NET_FILE);
  // Reset network
  reset_network(&net);
  // Print network
  printf("\n============== Network configuration ===============\n\n");
  print_network(&net);
  printf("==================================================\n\n");

  // Fill settings
  sl_settings.thrust_effect = SL_THRUST_EFFECT;
  sl_settings.thrust_p_gain = SL_THRUST_P_GAIN;
  sl_settings.thrust_i_gain = SL_THRUST_I_GAIN;

  // Init global variables
  init_globals();

  // Register telemetry message
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SPIKING_LANDING,
                              send_sl);

  // Subscribe to optical flow estimation
  AbiBindMsgOPTICAL_FLOW(SL_OPTICAL_FLOW_ID, &optical_flow_ev,
                         sl_optical_flow_cb);

  // Init low-pass filters for acceleration and thrust
  float tau = 1.0f / (2.0f * M_PI * SL_OF_FILTER_CUTOFF);
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
  acc_lp = 0.0f;
  record = 0;
  nominal_throttle = guidance_v_nominal_throttle;
  active_control = false;
}

// Get optical flow estimate from sensors via callback
static void sl_optical_flow_cb(uint8_t sender_id, uint32_t stamp,
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

  // Run the spiking network
  sl_run(divergence, divergence_dot);
}

// Run the spiking network
static void sl_run(float divergence, float divergence_dot) {
  // These "static" types are great!
  static bool first_run = true;
  static float start_time = 0.0f;
  static float nominal_throttle_sum = 0.0f;
  static float nominal_throttle_samples = 0.0f;

  // TODO: is this for resetting altitude?
  if (autopilot_get_mode() != AP_MODE_GUIDED) {
    first_run = true;
    // TODO: what's the use of this guided set?
    // 4.9 instead of 5.0 to account for slight climb when setting guided
    guidance_v_set_guided_z(-4.9);
    active_control = false;
    record = 0;
    return;
  }

  // TODO: here we reset the network in between runs!
  if (first_run) {
    start_time = get_sys_time_float();
    nominal_throttle = (float)stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
    reset_network(&net);
    first_run = false;
  }

  // Let the vehicle settle
  if (get_sys_time_float() - start_time < 5.0f) {
    // 4.9 instead of 5.0 to account for slight climb when setting guided
    // guidance_v_set_guided_z(-4.9);
    return;
  }

  // After vehicle settling, compute and improve nominal throttle estimate
  if (get_sys_time_float() - start_time < 10.0f) {
    // 4.9 instead of 5.0 to account for slight climb when setting guided
    // guidance_v_set_guided_z(-4.9);
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

  // Forward spiking net to get action/thrust for control
  // TODO: mind that we still need to convert from G to m/s2!
  // TODO: and take into account the trace scaling!
  net.in[0] = divergence;
  net.in[1] = divergence_dot;
  thrust = forward_network(&net) * 9.81f;

  // Bound thrust to limits (-0.8g, 0.5g)
  Bound(thrust, -7.848f, 4.905f);

  // Set control mode: active closed-loop control or linear transform
  if (SL_ACTIVE_CONTROL) {
    active_control = true;
  } else {
    guidance_v_set_guided_th(thrust * sl_settings.thrust_effect +
                             nominal_throttle);
  }
}

// Closed-loop PI control for going from acceleration to motor control
static void sl_control() {
  // "static" here implies that value is kept between function invocations
  static float error_integrator = 0.0f;

  // Low-pass filters for current acceleration and thrust setpoint
  struct NedCoor_f *acceleration = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&accel_ned_filt, acceleration->z);
  update_butterworth_2_low_pass(&thrust_filt, thrust);
  acc_lp = accel_ned_filt.o[0];

  // Proportional
  float error = thrust_filt.o[0] + accel_ned_filt.o[0];
  BoundAbs(error, 1.0f / (sl_settings.thrust_p_gain + 0.01f));

  // Integral
  error_integrator += error / PERIODIC_FREQUENCY;
  BoundAbs(error_integrator, 1.0f / (sl_settings.thrust_i_gain + 0.01f));

  // Acceleration setpoint
  float acceleration_sp = (thrust + error * sl_settings.thrust_p_gain +
                           error_integrator * sl_settings.thrust_i_gain) *
                              sl_settings.thrust_effect +
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
void spiking_landing_init() { sl_init(); }

// Run
void spiking_landing_event() { sl_control(); }
