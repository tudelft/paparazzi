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
 * @file "modules/spiking_net/spiking_landing.c"
 * @author Huizerd
 * Spiking neural networks for optical flow landing.
 */

// Header for this file
#include "modules/spiking_landing/spiking_landing.h"

// tinysnn headers
#include "modules/spiking_landing/tinysnn/Network.h"

// Paparazzi headers
// TODO: do we need all this? And in what order?
#include "navigation.h"
#include "state.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/gps.h"
#include "subsystems/gps/gps_datalink.h"

#include "generated/flight_plan.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_int.h"

#include "generated/airframe.h"
#include "subsystems/abi.h"

#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_indi.h"
#include "guidance/guidance_v.h"

#include "subsystems/abi.h"

// C standard library headers
#include <stdbool.h>
#include <stdio.h>

// Constants
// TODO: is this the way to do this? Seems a bit overdreven, and inconsistent
//  between the different constants
// Thrust settings
// From G to thrust percentage
float thrust_effect = 0.05f;

// Closed-loop thrust control, else linear transform
#define ACTIVE_CONTROL true

// Gains for closed-loop control
#ifndef THRUST_P_GAIN
#define THRUST_P_GAIN 0.7
#endif
#ifndef THRUST_I_GAIN
#define THRUST_I_GAIN 0.3
#endif
float thrust_p_gain = THRUST_P_GAIN;
float thrust_i_gain = THRUST_I_GAIN;

// Optical flow settings
#ifndef OF_FILTER_CUTOFF
#define OF_FILTER_CUTOFF 1.5f
#endif
#ifndef OF_SNN_ID
#define OF_SNN_ID ABI_BROADCAST
#endif
static abi_event optical_flow_event;

// Low-pass filters for acceleration and thrust
static Butterworth2LowPass accel_ned_filt;
static Butterworth2LowPass thrust_filt;

// Variables for divergence + derivative, thrust
static float divergence, divergence_dot, thrust;

// Variables for control
static float nominal_throttle = 0.0f;
static bool active_control = false;

// Sending stuff to ground station
// TODO: do we want this? Or does this include OptiTrack data?
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

// Send divergence + derivative, height, velocity, acceleration, thrust, mode
static void send_spiking_landing(struct transport_tx *trans,
                                 struct link_device *dev) {
  pprz_msg_send_SPIKING_LANDING(trans, dev, AC_ID, &divergence, &divergence_dot,
                                &(stateGetPositionNed_f()->z),
                                &(stateGetSpeedNed_f()->z),
                                &accel_ned_filt.o[0], &thrust, &autopilot.mode);
}
#endif

// Create the spiking net in global scope
// TODO: correct way to do this? Or use defines?
// File containing parameters
char const param_path[] = "network.txt";
// Network layer sizes
int const in_size = 4;
int const hid_size = 20;
int const out_size = 1;

// Build network
Network net = build_network(in_size, hid_size, out_size);

// Callback function for control (bound to optical flow ABI messages)
static void snn_control_callback(uint8_t sender_id, uint32_t stamp,
                                 int16_t UNUSED flow_x, int16_t UNUSED flow_y,
                                 int16_t UNUSED flow_der_x,
                                 int16_t UNUSED flow_der_y,
                                 float UNUSED quality, float size_divergence) {
  // Compute time step
  static uint32_t last_stamp = 0;
  float dt = (stamp - last_stamp) / 1e6;
  last_stamp = stamp;

  // Compute derivative of divergence
  if (dt > 1e-5) {
    divergence_dot = (size_divergence - divergence) / dt;
  }
  divergence = size_divergence;

  // These "static" types are great!
  static bool first_run = true;
  static float start_time = 0.0f;
  static float nominal_throttle_sum = 0.0f;
  static float nominal_throttle_samples = 0.0f;

  // TODO: is this for resetting altitude?
  if (autopilot_get_mode() != AP_MODE_GUIDED) {
    first_run = true;
    guidance_v_set_guided_z(-3.9);
    active_control = false;
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
  if (get_sys_time_float() - start_time < 4.0f) {
    return;
  }

  // During vehicle settling and 1 sec after, compute and improve nominal
  // throttle estimate
  if (get_sys_time_float() - start_time < 5.0f) {
    nominal_throttle_sum += (float)stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
    nominal_throttle_samples++;
    nominal_throttle = nominal_throttle_sum / nominal_throttle_samples;

    // TODO: we don't need to init the network further, right?
  }

  // Forward spiking net to get action/thrust for control
  // TODO: mind that we still need to convert from G to m/s2!
  net.in[0] = divergence;
  net.in[1] = divergence_dot;
  thrust = forward_network(&net) * 9.81f;

  // Bound thrust to limits (-0.8g, 0.5g)
  Bound(thrust, -7.848f, 4.905f)

      // Set control mode: active closed-loop control or linear transform
      if (ACTIVE_CONTROL) {
    active_control = true;
  }
  else {
    guidance_v_set_guided_th(thrust * thrust_effect + nominal_throttle);
  }
}

// Initialize the spiking net
void snn_init() {
  // Init network
  init_network(&net);
  // Load network parameters
  load_network(&net, param_path);
  // Reset network
  reset_network(&net);

  // Reset network inputs/output
  divergence = 0.0f;
  divergence_dot = 0.0f;
  thrust = 0.0f;
  nominal_throttle = guidance_v_nominal_throttle;

  // Register telemetry message
  // TODO: or use #if here? And then move out of this function?
  if (PERIODIC_TELEMETRY) {
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SPIKING_LANDING,
                                send_spiking_landing);
  }

  // Bind network forward pass to optical flow message
  // TODO: to sync them? Or to have divergence?
  // TODO: is this the way to go, or more an approach similar to
  //  optical_flow_landing?
  AbiBindMsgOPTICAL_FLOW(OF_SNN_ID, &optical_flow_event, snn_control_callback);

  // Init low-pass filters for acceleration and thrust
  float tau = 1.0f / (2.0f * M_PI * OF_FILTER_CUTOFF);
  float ts = 1.0f / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_ned_filt, tau, ts, 0.0f);
  init_butterworth_2_low_pass(&thrust_filt, tau, ts, 0.0f);
}

// TODO: function for freeing memory at end?

// Periodic function for printing debugging info
void snn_print_debug() { printf("Output trace: %.2f\n", net.out->t[0]); }

// Event function for pushing control through filters
// TODO: or closed-loop PI control for going from acceleration to motor control?
void snn_filter_control() {
  // "static" here implies that value is kept between function invocations
  static float error_integrator = 0.0f;

  // Low-pass filters for current acceleration and thrust setpoint
  struct NedCoor_f *acceleration = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&accel_ned_filt, acceleration->z);
  update_butterworth_2_low_pass(&thrust_filt, thrust);

  // Proportional
  float error = thrust_filt.o[0] + accel_ned_filt.o[0];
  BoundAbs(error, 1.0f / (thrust_p_gain + 0.01f));

  // Integral
  error_integrator += error / PERIODIC_FREQUENCY;
  BoundAbs(error_integrator, 1.0f / (thrust_i_gain + 0.01f));

  // Acceleration setpoint
  float acceleration_sp =
      (thrust + error * thrust_p_gain + error_integrator * thrust_i_gain) *
          thrust_effect +
      nominal_throttle;

  // Perform active closed-loop control or do simple linear transform
  if (active_control) {
    guidance_v_set_guided_th(acceleration_sp);
  } else {
    error_integrator = 0.0f;
  }
}
