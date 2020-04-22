
/*
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
 * @file "modules/event_based_flow/nn.c"
 * @author Kirk Scheper
 * This module is generates a command to avoid other vehicles based on their relative gps location
 */

#include "modules/nn_divergence_landing/nn.h"
#include "modules/nn_divergence_landing/nn_weights.h"

#include "std.h"

#include "stdio.h"

#include "subsystems/gps.h"
#include "subsystems/gps/gps_datalink.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "navigation.h"
#include "filters/low_pass_filter.h"

#include "generated/flight_plan.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_double.h"

#include "generated/airframe.h"           // AC_ID
#include "subsystems/abi.h"               // rssi

#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "guidance/guidance_indi.h"
#include "autopilot.h"

#include "subsystems/abi.h"

float thrust_effectiveness = 0.05f; // transfer function from G to thrust percentage

#ifndef NN_FILTER_CUTOFF
#define NN_FILTER_CUTOFF 1.5f
#endif

#ifndef NN_THRUST_P_GAIN
#define NN_THRUST_P_GAIN 0.7
#endif
#ifndef NN_THRUST_I_GAIN
#define NN_THRUST_I_GAIN 0.3
#endif
float nn_thrust_p_gain = NN_THRUST_P_GAIN;
float nn_thrust_i_gain = NN_THRUST_I_GAIN;

#define ACTIVE_CTRL true

Butterworth2LowPass accel_ned_filt;
Butterworth2LowPass thrust_filt;

float input_layer_out[nr_input_neurons] = {0};
float hidden_layer_out[nr_hidden_neurons] = {0};
float layer2_out[nr_output_neurons] = {0};

#if NN_TYPE == NN || NN_TYPE == RNN
static float relu(float val)
{
  BoundLower(val, 0.f);
  return val;
}

#elif NN_TYPE == CTRNN

static float sigmoid(float val)
{
  return 1.f / (1.f + expf(-val));
}
#endif

static float divergence, divergence_dot, thrust;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "autopilot.h"
/**
 * Send optical flow telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void nn_landing_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_NN_LANDING(trans, dev, AC_ID,
                               &divergence, &divergence_dot,
                               &accel_ned_filt.o[0], &(stateGetSpeedNed_f()->z), &(stateGetPositionNed_f()->z),
                               &thrust, &autopilot.mode);
}
#endif

static void zero_neurons(void){
  for (int16_t i = 0; i < nr_input_neurons; i++){
    input_layer_out[i] = 0.f;
  }
  for (int16_t i = 0; i < nr_hidden_neurons; i++){
    hidden_layer_out[i] = 0.f;
  }
  for (int16_t i = 0; i < nr_output_neurons; i++){
    layer2_out[i] = 0.f;
  }
}

static float predict_nn(float in[], float dt)
{
  int i,j;

  for (i = 0; i < nr_input_neurons; i++){
#if NN_TYPE == NN
    input_layer_out[i] = in[i] + bias0[i];
#elif NN_TYPE == RNN
    input_layer_out[i] = in[i] + bias0[i] + input_layer_out[i]*recurrent_weights0[i];
#elif NN_TYPE == CTRNN
    input_layer_out[i] += (in[i] - input_layer_out[i]) * dt / (dt + time_const0[i]);
#endif
  }

  float potential;
#if NN_TYPE == NN || NN_TYPE == RNN
  for (i = 0; i < nr_hidden_neurons; i++){
    potential = 0.f;
    for (j = 0; j < nr_input_neurons; j++){
      potential += input_layer_out[j]*layer1_weights[j][i];
    }
#if NN_TYPE == RNN
    potential += hidden_layer_out[i]*recurrent_weights1[i];
#endif
    hidden_layer_out[i] = relu(potential + bias1[i]);
  }

  for (i = 0; i < nr_output_neurons; i++){
    potential = 0.f;
    for (j = 0; j < nr_hidden_neurons; j++){
      potential += hidden_layer_out[j]*layer2_weights[j][i];
    }
#if NN_TYPE == RNN
    potential += layer2_out[i]*recurrent_weights2[i];
#endif
    layer2_out[i] = potential + bias2[i];
  }

#elif NN_TYPE == CTRNN
  for (i = 0; i < nr_hidden_neurons; i++) {
    potential = 0.f;
    for (j = 0; j < nr_input_neurons; j++) {
      potential += tanhf(gain0[j]*(input_layer_out[j] + bias0[j]))*layer1_weights[j][i];
    }
    hidden_layer_out[i] += (potential - hidden_layer_out[i]) * dt / (time_const1[i] + dt);
  }

  for (i = 0; i < nr_output_neurons; i++) {
    potential = 0.f;
    for (j = 0; j < nr_hidden_neurons; j++) {
      potential += tanhf(gain1[j]*(hidden_layer_out[j] + bias1[j]))*layer2_weights[j][i];
    }
    layer2_out[i] += (potential - layer2_out[i]) * dt / (time_const2[i] + dt);
  }
#endif

  return layer2_out[0];
}


static float nominal_throttle = 0.f;
static bool active_control = false;
static int nn_run(float D, float Ddot, float dt)
{
  static bool first_run = true;
  static float start_time = 0.f;
  static float nominal_throttle_sum = 0.f;
  static float nominal_throttle_samples = 0.f;

  if(autopilot_get_mode() != AP_MODE_GUIDED){
    first_run = true;
    guidance_v_set_guided_z(-3.9);
    active_control = false;
    return 0;
  }

  if (first_run){
    start_time = get_sys_time_float();
    nominal_throttle = (float)stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
    zero_neurons();
    first_run = false;
  }

  // Stabilise the vehicle and improve the estimate of the nominal throttle
  if(get_sys_time_float() - start_time < 4.f){
    // wait a few seconds for the Guided controller to settle
    return 0;
  }

  if(get_sys_time_float() - start_time < 5.f){
    // get good estimate to nominal throttle
    nominal_throttle_sum += (float)stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
    nominal_throttle_samples++;

    nominal_throttle = nominal_throttle_sum / nominal_throttle_samples;

    // initialise network by running zeros through it
    static float zero_input[] = {0.f, 0.f};
    predict_nn(zero_input, dt);
    return 0;
  }

  float input[] = {D, Ddot};
  thrust = predict_nn(input, dt);

  // limit commands
  Bound(thrust, -7.848f, 4.905f); // [-0.8g, 0.5g]

  // add drag compensation
  //thrust -= 0.3 * stateGetSpeedNed_f()->z * stateGetSpeedNed_f()->z;

#if ACTIVE_CTRL
  // activate closed loop control
  active_control = true;
#else
  // set throttle using linear transform
  guidance_v_set_guided_th(thrust*thrust_effectiveness + nominal_throttle);
#endif

  return 1;

  struct FloatVect3 accel_sp;
  uint8_t accel_sp_flag = 0;
  SetBit(accel_sp_flag, GUIDANCE_INDI_VERT_SP_FLAG);   // vertical accel only

  accel_sp.z = -thrust;

  AbiSendMsgACCEL_SP(ACCEL_SP_DVS_ID, accel_sp_flag, &accel_sp);

  return 1;
}

/* Use optical flow estimates */
#ifndef OFL_NN_ID
#define OFL_NN_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFL_NN_ID)

static abi_event optical_flow_ev;
static void div_cb(uint8_t sender_id, uint32_t stamp, int16_t UNUSED flow_x,
    int16_t UNUSED flow_y, int16_t UNUSED flow_der_x, int16_t UNUSED flow_der_y,
    float UNUSED quality, float size_divergence)
{
  static uint32_t last_stamp = 0;

  float dt = (stamp - last_stamp) / 1e6;
  last_stamp = stamp;
  if (dt > 1e-5){
    divergence_dot = (size_divergence - divergence) / dt;
  }
  divergence = size_divergence;

  nn_run(divergence, divergence_dot, dt);
}

void nn_init(void)
{
  zero_neurons();

  divergence = 0.f;
  divergence_dot = 0.f;
  thrust = 0.f;
  nominal_throttle = guidance_v_nominal_throttle;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NN_LANDING, nn_landing_telem_send);
#endif

  // bind to optical flow messages to get divergence
  AbiBindMsgOPTICAL_FLOW(OFL_NN_ID, &optical_flow_ev, div_cb);

  float tau = 1.f / (2.f * M_PI * NN_FILTER_CUTOFF);
  float sample_time = 1.f / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_ned_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, 0.0);
}

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 */
void nn_cntrl(void)
{
  static float error_integrator = 0.f;
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&accel_ned_filt, accel->z);
  update_butterworth_2_low_pass(&thrust_filt, thrust);

  float error = thrust_filt.o[0] + accel_ned_filt.o[0]; // rotate accel to enu
  BoundAbs(error, 1.f / (nn_thrust_p_gain + 0.01f)); // limit effect of integrator to max 1m/s

  error_integrator += error / PERIODIC_FREQUENCY;
  BoundAbs(error_integrator, 1.f / (nn_thrust_i_gain + 0.01f));  // limit effect of integrator to max 1m/s

  // FF + P + I
  float accel_sp = (thrust + error*nn_thrust_p_gain + error_integrator*nn_thrust_i_gain)*thrust_effectiveness
      + nominal_throttle;

  if(active_control){
    guidance_v_set_guided_th(accel_sp);
  } else {
    error_integrator = 0.f;
  }
}

/*
 *  debug printout
 */
void nn_periodic(void)
{
  float input[] = {1.f, 1.f};
  printf("%f\n", predict_nn(input, 0.025));
}

