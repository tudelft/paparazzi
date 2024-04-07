/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
 * MAVLab Delft University of Technology
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_attitude_quat_indi.c
 * @brief MAVLab Delft University of Technology
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "modules/tailsitter_auto/tailsitter_auto_takeoff.h"

#include "state.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "math/wls/wls_alloc.h"
#include <stdio.h>
#include "firmwares/rotorcraft/autopilot_static.h"
#include "autopilot.h"
#include "modules/sonar/agl_dist.h"

// Factor that the estimated G matrix is allowed to deviate from initial one
#define INDI_ALLOWED_G_FACTOR 2.0
#define SIZE 2
#ifdef STABILIZATION_INDI_FILT_CUTOFF_P
#define STABILIZATION_INDI_FILTER_ROLL_RATE TRUE
#else
#define STABILIZATION_INDI_FILT_CUTOFF_P 20.0
#endif

#ifdef STABILIZATION_INDI_FILT_CUTOFF_Q
#define STABILIZATION_INDI_FILTER_PITCH_RATE TRUE
#else
#define STABILIZATION_INDI_FILT_CUTOFF_Q 20.0
#endif

#ifdef STABILIZATION_INDI_FILT_CUTOFF_R
#define STABILIZATION_INDI_FILTER_YAW_RATE TRUE
#else
#define STABILIZATION_INDI_FILT_CUTOFF_R 20.0
#endif

#ifndef RADIO_PIVOT_SWITCH
#define RADIO_PIVOT_SWITCH FALSE
#endif

#ifndef STABILIZATION_INDI_TAKEOFF_CUTOFF_P
#define STABILIZATION_INDI_TAKEOFF_CUTOFF_P 5
#endif

#ifndef STABILIZATION_INDI_TAKEOFF_CUTOFF_Q
#define STABILIZATION_INDI_TAKEOFF_CUTOFF_Q 5
#endif

#ifndef STABILIZATION_INDI_TAKEOFF_CUTOFF_R
#define STABILIZATION_INDI_TAKEOFF_CUTOFF_R 5
#endif


// Default is WLS
#ifndef STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#define STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE FALSE
#endif

#ifndef STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
#define STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER FALSE
#endif

// Airspeed [m/s] at which the forward flight throttle limit is used instead of
// the hover throttle limit.
#ifndef STABILIZATION_INDI_THROTTLE_LIMIT_AIRSPEED_FWD
#define STABILIZATION_INDI_THROTTLE_LIMIT_AIRSPEED_FWD 8.0
#endif

#if INDI_OUTPUTS > 4
#ifndef STABILIZATION_INDI_G1_THRUST_X
#error "You must define STABILIZATION_INDI_G1_THRUST_X for your number of INDI_OUTPUTS"
#endif
#endif

#ifdef SetCommandsFromRC
#warning SetCommandsFromRC not used: STAB_INDI writes actuators directly
#endif

#ifdef SetAutoCommandsFromRC
#warning SetAutoCommandsFromRC not used: STAB_INDI writes actuators directly
#endif


#if !STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#if INDI_NUM_ACT > WLS_N_U
#error Matrix-WLS_N_U too small or not defined: define WLS_N_U >= INDI_NUM_ACT in airframe file
#endif
#if INDI_OUTPUTS > WLS_N_V
#error Matrix-WLS_N_V too small or not defined: define WLS_N_U >= INDI_OUTPUTS in airframe file
#endif
#endif

#ifdef STABILIZATION_INDI_L1
float L1 = STABILIZATION_INDI_L1;
#else
float L1=1.0;
#endif

#ifdef STABILIZATION_INDI_L2
float L2 = STABILIZATION_INDI_L2;
#else
float L2=1.0;
#endif

#ifdef STABILIZATION_INDI_WEIGHT_T
float weight_T = STABILIZATION_INDI_WEIGHT_T;
#else
float weight_T=1.0;
#endif

#ifdef STABILIZATION_INDI_WEIGHT_S
float weight_S = STABILIZATION_INDI_WEIGHT_S;
#else
float weight_S=1.0;
#endif

#ifdef CTRL_EFF_CALC_K1
float K1 = CTRL_EFF_CALC_K1;
#else
float K1=1.0;
#endif

#ifdef CTRL_EFF_CALC_K2
float K2 = CTRL_EFF_CALC_K2;
#else
float K2=1.0;
#endif

#ifdef CTRL_EFF_CALC_K3
float K3 = CTRL_EFF_CALC_K3;
#else
float  K3=1.0;
#endif

#ifdef CTRL_EFF_CALC_MASS
float m = CTRL_EFF_CALC_MASS;
#else
float m=1.0;
#endif

#ifdef CTRL_EFF_CALC_TORQUE
float torque = CTRL_EFF_CALC_TORQUE;
#else
float torque=1.0;
#endif

#ifdef STABILIZATION_INDI_PIVOT_GAIN_Q
float pivot_gain_q = STABILIZATION_INDI_PIVOT_GAIN_Q;
#else
float pivot_gain_q=1.0;
#endif

#ifdef STABILIZATION_INDI_PIVOT_GAIN_THETA
float pivot_gain_theta = STABILIZATION_INDI_PIVOT_GAIN_THETA;
#else
float pivot_gain_theta=1.0;
#endif

#ifdef STABILIZATION_INDI_PIVOT_GAIN_I
float pivot_gain_i = STABILIZATION_INDI_PIVOT_GAIN_I;
#else
float pivot_gain_i=1.0;
#endif

float theta_d = RadOfDeg(-90.0); 
// float m = CTRL_EFF_CALC_MASS;
int16_t takeoff_stage = 0;
//define the size of B and W matrix used in takeoff
#define TYPE_ACT 2  // Type of actuator outputs
#define NUM_OUT 1  //B is 1x2
//float I_yy = CTRL_EFF_CALC_I_YY;
float du_min_stab_indi[INDI_NUM_ACT];
float du_max_stab_indi[INDI_NUM_ACT];
float du_pref_stab_indi[INDI_NUM_ACT];
float indi_v[INDI_OUTPUTS];
float *Bwls[INDI_OUTPUTS];
int num_iter = 0;
// Global variable for the counter
uint32_t indi_counter = 0;
void update_filters(void);
static void lms_estimation(void);
static void get_actuator_state(void);
static void calc_g1_element(float dx_error, int8_t i, int8_t j, float mu_extra);
static void calc_g2_element(float dx_error, int8_t j, float mu_extra);
void invertDiagonalMatrix(float W[TYPE_ACT][TYPE_ACT], float W_inv[TYPE_ACT][TYPE_ACT]);
void transposeMatrix(float matrix[NUM_OUT][TYPE_ACT], float transposed[TYPE_ACT][NUM_OUT]);
void multiplyMatrices(float matrix1[TYPE_ACT][TYPE_ACT], float matrix2[NUM_OUT][TYPE_ACT], float result[NUM_OUT][TYPE_ACT]);
void multiplyMatrix2x2_2x1(float matrix1[TYPE_ACT][TYPE_ACT], float matrix2[TYPE_ACT][NUM_OUT], float result[TYPE_ACT][NUM_OUT]);
void multiplyMatrixByScalar(float matrix[TYPE_ACT][NUM_OUT], float scalar, float result[TYPE_ACT][NUM_OUT]);
void pseudoinv_B(float B[NUM_OUT][TYPE_ACT], float W[TYPE_ACT][TYPE_ACT], float B_inv[TYPE_ACT][NUM_OUT]);
int16_t calculatePPRZCommand(float K1, float K2, float K3, float thrust);

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
static void calc_g1g2_pseudo_inv(void);
#endif
static void bound_g_mat(void);
int32_t heading_check;
int32_t stabilization_att_indi_cmd[COMMANDS_NB];
struct Indi_gains indi_gains = {
  .att = {
    STABILIZATION_INDI_REF_ERR_P,
    STABILIZATION_INDI_REF_ERR_Q,
    STABILIZATION_INDI_REF_ERR_R
  },
  .rate = {
    STABILIZATION_INDI_REF_RATE_P,
    STABILIZATION_INDI_REF_RATE_Q,
    STABILIZATION_INDI_REF_RATE_R
  },
};

#if STABILIZATION_INDI_USE_ADAPTIVE
bool indi_use_adaptive = true;
#else
bool indi_use_adaptive = false;
#endif

#ifdef STABILIZATION_INDI_ACT_RATE_LIMIT
float act_rate_limit[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_RATE_LIMIT;
#endif

#ifdef STABILIZATION_INDI_ACT_IS_SERVO
bool act_is_servo[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_IS_SERVO;
#else
bool act_is_servo[INDI_NUM_ACT] = {[0 ... INDI_NUM_ACT - 1] = 0};
#endif

#ifdef STABILIZATION_INDI_ACT_IS_THRUSTER_X
bool act_is_thruster_x[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_IS_THRUSTER_X;
#else
bool act_is_thruster_x[INDI_NUM_ACT] = {[0 ... INDI_NUM_ACT - 1] = 0};
#endif

bool act_is_thruster_z[INDI_NUM_ACT];

#ifdef STABILIZATION_INDI_ACT_PREF
// Preferred (neutral, least energy) actuator value
float act_pref[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_PREF;
#else
// Assume 0 is neutral
float act_pref[INDI_NUM_ACT] = {[0 ... INDI_NUM_ACT - 1] = 0.0f};
#endif

#ifdef STABILIZATION_INDI_ACT_DYN
#warning STABILIZATION_INDI_ACT_DYN is deprecated, use STABILIZATION_INDI_ACT_FREQ instead.
#warning You now have to define the continuous time corner frequency in rad/s of the actuators.
#warning "Use -ln(1 - old_number) * PERIODIC_FREQUENCY to compute it from the old values."
float act_dyn_discrete[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_DYN;
#else
float act_first_order_cutoff[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_FREQ;
float act_dyn_discrete[INDI_NUM_ACT]; // will be computed from freq at init
#endif

//-------------------------------------
float pivot_ratio;
// float pivot_servogain_q= STABILIZATION_INDI_PIVOT_SERVOGAIN_Q;
// float pivot_servogain_theta= STABILIZATION_INDI_PIVOT_SERVOGAIN_THETA;
//-------------------------------------

#ifdef STABILIZATION_INDI_WLS_PRIORITIES
static float Wv[INDI_OUTPUTS] = STABILIZATION_INDI_WLS_PRIORITIES;
#else
//State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
#if INDI_OUTPUTS == 5
static float Wv[INDI_OUTPUTS] = {1000, 1000, 1, 100, 100};
#else
static float Wv[INDI_OUTPUTS] = {1000, 1000, 1, 100};
#endif
#endif

/**
 * Weighting of different actuators in the cost function
 */
#ifdef STABILIZATION_INDI_WLS_WU
float indi_Wu[INDI_NUM_ACT] = STABILIZATION_INDI_WLS_WU;
#else
float indi_Wu[INDI_NUM_ACT] = {[0 ... INDI_NUM_ACT - 1] = 1.0};
#endif

// variables needed for control
float actuator_state_filt_vect[INDI_NUM_ACT];
struct FloatRates angular_accel_ref = {0., 0., 0.};
struct FloatRates angular_rate_ref = {0., 0., 0.};
float angular_acceleration[3] = {0., 0., 0.};
float actuator_state[INDI_NUM_ACT];
float indi_u[INDI_NUM_ACT];
float indi_du[INDI_NUM_ACT];
float g2_times_du;

float q_filt = 0.0;
float r_filt = 0.0;

// variables needed for estimation
float g1g2_trans_mult[INDI_OUTPUTS][INDI_OUTPUTS];
float g1g2inv[INDI_OUTPUTS][INDI_OUTPUTS];
float actuator_state_filt_vectd[INDI_NUM_ACT];
float actuator_state_filt_vectdd[INDI_NUM_ACT];
float estimation_rate_d[INDI_NUM_ACT];
float estimation_rate_dd[INDI_NUM_ACT];
float du_estimation[INDI_NUM_ACT];
float ddu_estimation[INDI_NUM_ACT];

// The learning rate per axis (roll, pitch, yaw, thrust)
float mu1[INDI_OUTPUTS] = {0.00001, 0.00001, 0.000003, 0.000002};
// The learning rate for the propeller inertia (scaled by 512 wrt mu1)
float mu2 = 0.002;

// other variables
float act_obs[INDI_NUM_ACT];

// Number of actuators used to provide thrust
int32_t num_thrusters;
int32_t num_thrusters_x;

struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
struct FloatRates  stab_att_ff_rates;

// Register actuator feedback if we rely on RPM information
#if STABILIZATION_INDI_RPM_FEEDBACK
#ifndef STABILIZATION_INDI_ACT_FEEDBACK_ID
#define STABILIZATION_INDI_ACT_FEEDBACK_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(STABILIZATION_INDI_ACT_FEEDBACK_ID)

abi_event act_feedback_ev;
static void act_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback, uint8_t num_act);
PRINT_CONFIG_MSG("STABILIZATION_INDI_RPM_FEEDBACK")
#endif

abi_event thrust_ev;
static void thrust_cb(uint8_t sender_id, struct FloatVect3 thrust_increment);
struct FloatVect3 indi_thrust_increment;
bool indi_thrust_increment_set = false;

float g1g2_pseudo_inv[INDI_NUM_ACT][INDI_OUTPUTS];
float g2[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
#ifdef STABILIZATION_INDI_G1
float g1[INDI_OUTPUTS][INDI_NUM_ACT] = STABILIZATION_INDI_G1;
#else // old defines TODO remove
#if INDI_OUTPUTS == 5
float g1[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                        STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW,
                                        STABILIZATION_INDI_G1_THRUST, STABILIZATION_INDI_G1_THRUST_X
                                       };
#else
float g1[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                        STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                       };
#endif
#endif

float g1g2[INDI_OUTPUTS][INDI_NUM_ACT];
float g1_est[INDI_OUTPUTS][INDI_NUM_ACT];
float g2_est[INDI_NUM_ACT];
float g1_init[INDI_OUTPUTS][INDI_NUM_ACT];
float g2_init[INDI_NUM_ACT];

//Variables for the auto take off
float roll_gain = 0.01;

Butterworth2LowPass actuator_lowpass_filters[INDI_NUM_ACT];
Butterworth2LowPass estimation_input_lowpass_filters[INDI_NUM_ACT];
Butterworth2LowPass measurement_lowpass_filters[3];
Butterworth2LowPass estimation_output_lowpass_filters[3];
Butterworth2LowPass acceleration_lowpass_filter;
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
Butterworth2LowPass rates_filt_so[3];
#else
static struct FirstOrderLowPass rates_filt_fo[3];
#endif

static struct FirstOrderLowPass rates_filt_takeoff_fo[3];

Butterworth2LowPass qfilt;

struct FloatVect3 body_accel_f;

void init_filters(void);
void sum_g1_g2(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_indi_g(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INDI_G(trans, dev, AC_ID, INDI_NUM_ACT, g1g2[0],
                       INDI_NUM_ACT, g1g2[1],
                       INDI_NUM_ACT, g1g2[2],
                       INDI_NUM_ACT, g1g2[3],
                       INDI_NUM_ACT, g2_est);
}

static void send_pivot(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_PIVOT(trans, dev, AC_ID,
                      &heading_check,
                      &stab_att_sp_euler.psi,
                      &takeoff_stage);
}

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &stab_att_sp_quat.qi,
                              &stab_att_sp_quat.qx,
                              &stab_att_sp_quat.qy,
                              &stab_att_sp_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}

static void send_att_full_indi(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatRates *body_rates = stateGetBodyRates_f();
  struct Int32Vect3 *body_accel_i = stateGetAccelBody_i();
  struct FloatVect3 body_accel_f_telem;
  ACCELS_FLOAT_OF_BFP(body_accel_f_telem, *body_accel_i);
  
  pprz_msg_send_STAB_ATTITUDE_INDI(trans, dev, AC_ID,
                                   &body_accel_f_telem.x,    // input lin.acc
                                   &body_accel_f_telem.y,
                                   &body_accel_f_telem.z,
                                   &body_rates->p,           // rate
                                   &body_rates->q,
                                   &body_rates->r,
                                   &angular_rate_ref.p,      // rate.sp
                                   &angular_rate_ref.q,
                                   &angular_rate_ref.r,
                                   &angular_acceleration[0], // ang.acc
                                   &angular_acceleration[1],
                                   &angular_acceleration[2],
                                   &angular_accel_ref.p,     // ang.acc.sp
                                   &angular_accel_ref.q,
                                   &angular_accel_ref.r,
                                   &actuator_state_filt_vect[0],
                                   &actuator_state_filt_vect[1],
                                   &actuator_state_filt_vect[2],
                                   &actuator_state_filt_vect[3],
                                   &actuator_state_filt_vect[4],
                                   &actuator_state_filt_vect[5],
                                   INDI_NUM_ACT, indi_u);    // out
}
#endif

/**
 * Function that initializes important values upon engaging INDI
 */
void stabilization_indi_init(void)
{
  // Initialize filters
  init_filters();
  init_filters_rc();

  int8_t i;
// If the deprecated STABILIZATION_INDI_ACT_DYN is used, convert it to the new FREQUENCY format
#ifdef STABILIZATION_INDI_ACT_FREQ
  // Initialize the array of pointers to the rows of g1g2
  for (i = 0; i < INDI_NUM_ACT; i++) {
    act_dyn_discrete[i] = 1-exp(-act_first_order_cutoff[i]/PERIODIC_FREQUENCY);
  }
#endif

#if STABILIZATION_INDI_RPM_FEEDBACK
  AbiBindMsgACT_FEEDBACK(STABILIZATION_INDI_ACT_FEEDBACK_ID, &act_feedback_ev, act_feedback_cb);
#endif
  AbiBindMsgTHRUST(THRUST_INCREMENT_ID, &thrust_ev, thrust_cb);

  float_vect_zero(actuator_state_filt_vectd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vectdd, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_d, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_dd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vect, INDI_NUM_ACT);

  //Calculate G1G2
  sum_g1_g2();

  // Do not compute if not needed
#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
  //Calculate G1G2_PSEUDO_INVERSE
  calc_g1g2_pseudo_inv();
#endif

  // Initialize the array of pointers to the rows of g1g2
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // Initialize the estimator matrices
  float_vect_copy(g1_est[0], g1[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2_est, g2, INDI_NUM_ACT);
  // Remember the initial matrices
  float_vect_copy(g1_init[0], g1[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2_init, g2, INDI_NUM_ACT);

  // Assume all non-servos are delivering thrust
  num_thrusters = INDI_NUM_ACT;
  num_thrusters_x = 0;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    num_thrusters -= act_is_servo[i];
    num_thrusters -= act_is_thruster_x[i];

    num_thrusters_x += act_is_thruster_x[i];

    act_is_thruster_z[i] = !act_is_servo[i] && !act_is_thruster_x[i];
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INDI_G, send_indi_g);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PIVOT, send_pivot);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_INDI, send_att_full_indi);
#endif
}

/**
 * Function that resets important values upon engaging INDI.
 *
 * Don't reset inputs and filters, because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 * FIXME: Ideally we should detect when coming from something that is not INDI
 */
void stabilization_indi_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  float_vect_zero(du_estimation, INDI_NUM_ACT);
  float_vect_zero(ddu_estimation, INDI_NUM_ACT);
}

/**
 * Function that resets the filters to zeros
 */
void init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF);
  float tau_est = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  // Filtering of the gyroscope
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&measurement_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], tau_est, sample_time, 0.0);
  }

  // Filtering of the actuators
  for (i = 0; i < INDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&actuator_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], tau_est, sample_time, 0.0);
  }

  // Filtering of the accel body z
  init_butterworth_2_low_pass(&acceleration_lowpass_filter, tau_est, sample_time, 0.0);

 // Init rate filter for feedback
  float time_constants[3] = {1.0 / (2 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_P), 1.0 / (2 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_Q), 1.0 / (2 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_R)};

  init_first_order_low_pass(&rates_filt_fo[0], time_constants[0], sample_time, stateGetBodyRates_f()->p);
  init_first_order_low_pass(&rates_filt_fo[1], time_constants[1], sample_time, stateGetBodyRates_f()->q);
  init_first_order_low_pass(&rates_filt_fo[2], time_constants[2], sample_time, stateGetBodyRates_f()->r);


 // Init rate filter for feedback in takeoff
  float takeoff_time_constants[3] = {1.0 / (2 * M_PI * STABILIZATION_INDI_TAKEOFF_CUTOFF_P), 1.0 / (2 * M_PI * STABILIZATION_INDI_TAKEOFF_CUTOFF_Q), 1.0 / (2 * M_PI * STABILIZATION_INDI_TAKEOFF_CUTOFF_R)};

  init_first_order_low_pass(&rates_filt_takeoff_fo[0], takeoff_time_constants[0], sample_time, stateGetBodyRates_f()->p);
  init_first_order_low_pass(&rates_filt_takeoff_fo[1], takeoff_time_constants[1], sample_time, stateGetBodyRates_f()->q);
  init_first_order_low_pass(&rates_filt_takeoff_fo[2], takeoff_time_constants[2], sample_time, stateGetBodyRates_f()->r);
 
  //tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_2ORDER_QFILT_CUTOFF);
  init_butterworth_2_low_pass(&qfilt, tau_est, sample_time, 0.0);
}

/**
 * Function that calculates the failsafe setpoint
 */
void stabilization_indi_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

/**
 * @param rpy rpy from which to calculate quaternion setpoint
 *
 * Function that calculates the setpoint quaternion from rpy
 */
void stabilization_indi_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler = *rpy;

  int32_quat_of_eulers(&stab_att_sp_quat, &stab_att_sp_euler);
  FLOAT_RATES_ZERO(stab_att_ff_rates);
}

/**
 * @param quat quaternion setpoint
 */
void stabilization_indi_set_quat_setpoint_i(struct Int32Quat *quat)
{
  stab_att_sp_quat = *quat;
  int32_eulers_of_quat(&stab_att_sp_euler, quat);
  FLOAT_RATES_ZERO(stab_att_ff_rates);
}

/**
 * @param cmd 2D command in North East axes
 * @param heading Heading of the setpoint
 *
 * Function that calculates the setpoint quaternion from a command in earth axes
 */
void stabilization_indi_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
  FLOAT_RATES_ZERO(stab_att_ff_rates);
}

/**
 * @brief Set attitude setpoint from stabilization setpoint struct
 *
 * @param sp Stabilization setpoint structure
 */
void stabilization_indi_set_stab_sp(struct StabilizationSetpoint *sp)
{
  stab_att_sp_euler = stab_sp_to_eulers_i(sp);
  stab_att_sp_quat = stab_sp_to_quat_i(sp);
  stab_att_ff_rates = stab_sp_to_rates_f(sp);
}

void update_filters(void) {
  /* Propagate the filter on the gyroscopes */
  struct FloatRates *body_rates = stateGetBodyRates_f();
  float rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};
  int8_t i;
  for (i = 0; i < 3; i++) {
    update_butterworth_2_low_pass(&measurement_lowpass_filters[i], rate_vect[i]);
    update_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], rate_vect[i]);

    //Calculate the angular acceleration via finite difference
    angular_acceleration[i] = (measurement_lowpass_filters[i].o[0]
                               - measurement_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;

    // Calculate derivatives for estimation
    float estimation_rate_d_prev = estimation_rate_d[i];
    estimation_rate_d[i] = (estimation_output_lowpass_filters[i].o[0] - estimation_output_lowpass_filters[i].o[1]) *
                           PERIODIC_FREQUENCY;
    estimation_rate_dd[i] = (estimation_rate_d[i] - estimation_rate_d_prev) * PERIODIC_FREQUENCY;

 // Propagate actuator filters
  get_actuator_state();
  for (i = 0; i < INDI_NUM_ACT; i++) {
    update_butterworth_2_low_pass(&actuator_lowpass_filters[i], actuator_state[i]);
    update_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], actuator_state[i]);
    actuator_state_filt_vect[i] = actuator_lowpass_filters[i].o[0];

    // calculate derivatives for estimation
    float actuator_state_filt_vectd_prev = actuator_state_filt_vectd[i];
    actuator_state_filt_vectd[i] = (estimation_input_lowpass_filters[i].o[0] - estimation_input_lowpass_filters[i].o[1]) *
                                   PERIODIC_FREQUENCY;
    actuator_state_filt_vectdd[i] = (actuator_state_filt_vectd[i] - actuator_state_filt_vectd_prev) * PERIODIC_FREQUENCY;
  }
  }
}

/**
 * @param att_err attitude error
 * @param rate_control boolean that states if we are in rate control or attitude control
 * @param in_flight boolean that states if the UAV is in flight or not
 *
 * Function that calculates the INDI commands
 */

void stabilization_indi_rate_run(struct FloatRates rate_sp, bool in_flight)
{
  float airspeed = stateGetAirspeed_f();
  struct FloatRates *body_rates = stateGetBodyRates_f();
  update_filters();

  //The rates used for feedback are by default the measured rates.
  //If there is a lot of noise on the gyroscope, it might be good to use the filtered value for feedback.
  //Note that due to the delay, the PD controller may need relaxed gains.
  struct FloatRates rates_filt;
#if STABILIZATION_INDI_FILTER_ROLL_RATE
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
  rates_filt.p = update_butterworth_2_low_pass(&rates_filt_so[0], body_rates->p);
#else
  rates_filt.p = update_first_order_low_pass(&rates_filt_fo[0], body_rates->p);
#endif
#else
  rates_filt.p = body_rates->p;
#endif
#if STABILIZATION_INDI_FILTER_PITCH_RATE
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
  rates_filt.q = update_butterworth_2_low_pass(&rates_filt_so[1], body_rates->q);
#else
  rates_filt.q = update_first_order_low_pass(&rates_filt_fo[1], body_rates->q);
#endif
#else
  rates_filt.q = body_rates->q;
#endif
#if STABILIZATION_INDI_FILTER_YAW_RATE
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
  rates_filt.r = update_butterworth_2_low_pass(&rates_filt_so[2], body_rates->r);
#else
  rates_filt.r = update_first_order_low_pass(&rates_filt_fo[2], body_rates->r);
#endif
#else
  rates_filt.r = body_rates->r;
#endif

  //calculate the virtual control (reference acceleration) based on a PD controller
  angular_accel_ref.p = (rate_sp.p - rates_filt.p) * indi_gains.rate.p;
  angular_accel_ref.q = (rate_sp.q - rates_filt.q) * indi_gains.rate.q;
  angular_accel_ref.r = (rate_sp.r - rates_filt.r) * indi_gains.rate.r;

/*
  float delta_time = 1/200; //calculate the time step
  // Calculate the derivative of the tilt angle
  float tilt_angle_rate_left = mapping * (indi_u[0] - actuator_state_filt_vect[0]) / delta_time;
  float tilt_angle_rate_right = mapping * (indi_u[1] - actuator_state_filt_vect[1]) / delta_time;
  
  // calculate compensation for servo reaction moment (torque)
  float servo_moment_pitch_left = torque * 9.8 * tilt_angle_rate_left ;
  float servo_moment_pitch_right = torque * 9.8 * tilt_angle_rate_right;

  // combine left and right servo reaction moments
  float servo_moment_pitch = servo_moment_pitch_left + servo_moment_pitch_right;
  //Compensate the counteract servo reaction moment
  angular_accel_ref.q += servo_moment_pitch / I_yy;
*/

  g2_times_du = 0.0;
  int8_t i;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    g2_times_du += g2[i] * indi_du[i];
  }
  //G2 is scaled by INDI_G_SCALING to make it readable
  g2_times_du = g2_times_du / INDI_G_SCALING;

  float use_increment = 0.0;
  if (in_flight) {
    use_increment = 1.0;
  }

  struct FloatVect3 v_thrust;
  v_thrust.x = 0.0;
  v_thrust.y = 0.0;
  v_thrust.z = 0.0;
  if (indi_thrust_increment_set) {
    v_thrust = indi_thrust_increment;

    //update thrust command such that the current is correctly estimated
    stabilization_cmd[COMMAND_THRUST] = 0;
    for (i = 0; i < INDI_NUM_ACT; i++) {
      stabilization_cmd[COMMAND_THRUST] += actuator_state[i] * (int32_t) act_is_thruster_z[i];
    }
    stabilization_cmd[COMMAND_THRUST] /= num_thrusters;

#if INDI_OUTPUTS == 5
    stabilization_cmd[COMMAND_THRUST_X] = 0;
    for (i = 0; i < INDI_NUM_ACT; i++) {
      stabilization_cmd[COMMAND_THRUST_X] += actuator_state[i] * (int32_t) act_is_thruster_x[i];
    }
    stabilization_cmd[COMMAND_THRUST_X] /= num_thrusters_x;
#endif

  } else {
    // incremental thrust
    for (i = 0; i < INDI_NUM_ACT; i++) {
      v_thrust.z +=
        (stabilization_cmd[COMMAND_THRUST] - use_increment * actuator_state_filt_vect[i]) * Bwls[3][i];
#if INDI_OUTPUTS == 5
      stabilization_cmd[COMMAND_THRUST_X] = radio_control.values[RADIO_CONTROL_THRUST_X];
      v_thrust.x +=
        (stabilization_cmd[COMMAND_THRUST_X] - use_increment * actuator_state_filt_vect[i]) * Bwls[4][i];
#endif
    }
  }

  // The control objective in array format
  indi_v[0] = (angular_accel_ref.p - use_increment * angular_acceleration[0]);
  indi_v[1] = (angular_accel_ref.q - use_increment * angular_acceleration[1]);
  indi_v[2] = (angular_accel_ref.r - use_increment * angular_acceleration[2] + g2_times_du);
  indi_v[3] = v_thrust.z;
#if INDI_OUTPUTS == 5
  indi_v[4] = v_thrust.x;
#endif


#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
  // Calculate the increment for each actuator
  for (i = 0; i < INDI_NUM_ACT; i++) {
    indi_du[i] = (g1g2_pseudo_inv[i][0] * indi_v[0])
                 + (g1g2_pseudo_inv[i][1] * indi_v[1])
                 + (g1g2_pseudo_inv[i][2] * indi_v[2])
                 + (g1g2_pseudo_inv[i][3] * indi_v[3]);
  }
#else
  stabilization_indi_set_wls_settings(use_increment);

  for (i = 0; i < INDI_NUM_ACT; i++) {

#ifdef STABILIZATION_INDI_MAX_SERVO_INCREMENT
    if (act_is_servo[i]) {
    	BoundAbs(du_min_stab_indi[i], STABILIZATION_INDI_MAX_SERVO_INCREMENT);
    	BoundAbs(du_max_stab_indi[i], STABILIZATION_INDI_MAX_SERVO_INCREMENT);
    }
#endif
  }

  //Control allocation Weights as a function of airspeed
  // Tilt-rotor tailsitter
  #if USE_PIVOT_SWITCH == TRUE
  float fun_tilt = 0.124875f * airspeed - 0.4985f;
  float fun_elevon = -0.124875f * airspeed + 1.4995f;
  indi_Wu[0] = (fun_tilt > 1.0f) ? 1.0f: ((fun_tilt < 0.001f) ? 0.001f : fun_tilt);
  indi_Wu[1] = indi_Wu[0];
  indi_Wu[4] = (fun_elevon > 1.0f) ? 1.0f: ((fun_elevon < 0.1f) ? 0.1f : fun_elevon);
  indi_Wu[5] = indi_Wu[4];

  // float pitch_preference = 0.5f; // Example value, adjust as needed
  // float roll_yaw_preference = 0.8f; // Example value, adjust as needed
  // if (airspeed > 10){
  //   indi_Wu[0] *= (fabs(indi_v[1]) > 0) ? pitch_preference : 1.0f;
  //   indi_Wu[1] *= (fabs(indi_v[1]) > 0) ? pitch_preference : 1.0f;
  //   indi_Wu[4] *= ((fabs(indi_v[0]) > 0) || (fabs(indi_v[2]) > 0)) ? roll_yaw_preference : 1.0f;
  //   indi_Wu[5] *= ((fabs(indi_v[0]) > 0) || (fabs(indi_v[2]) > 0)) ? roll_yaw_preference : 1.0f;

  // }
  #endif
  
 
 
  // Flap deflected tailsitter, for comparison of wind disturbance rejection in hovering
  // float fun_flap = 0.124875f * airspeed - 0.4985f;
  // float fun_tilt = -0.124875f * airspeed + 1.4995f;
  // indi_Wu[0] = (fun_tilt > 1.0f) ? 1.0f: ((fun_tilt < 0.001f) ? 0.001f : fun_tilt);
  // indi_Wu[1] = indi_Wu[0];
  // indi_Wu[4] = (fun_elevon > 1.0f) ? 1.0f: ((fun_elevon < 0.001f) ? 0.001f : fun_elevon);
  // indi_Wu[5] = indi_Wu[4];


  //RunOnceEvery(200, DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 1,indi_counter));
  //RunOnceEvery(200, DOWNLINK_SEND_DEBUG_VECT(DefaultChannel, DefaultDevice, 1, name, 6, du_max_stab_indi));

  // WLS Control Allocator
  num_iter =
    wls_alloc(indi_du, indi_v, du_min_stab_indi, du_max_stab_indi, Bwls, 0, 0, Wv, indi_Wu, du_pref_stab_indi, 10000, 10,
              INDI_NUM_ACT, INDI_OUTPUTS);
#endif

  if (in_flight) {
    // Add the increments to the actuators
    float_vect_sum(indi_u, actuator_state_filt_vect, indi_du, INDI_NUM_ACT);
  } else {
    // Not in flight, so don't increment
    float_vect_copy(indi_u, indi_du, INDI_NUM_ACT);
  }

  // Bound the inputs to the actuators
  for (i = 0; i < INDI_NUM_ACT; i++) {
    if (act_is_servo[i]) {
      BoundAbs(indi_u[i], MAX_PPRZ);
    } else {
      if (autopilot_get_motors_on()) {
        Bound(indi_u[i], 0, MAX_PPRZ);
      } else {
        indi_u[i] = -MAX_PPRZ;
      }
    }
  }

  // Use online effectiveness estimation only when flying
  if (in_flight && indi_use_adaptive) {
    lms_estimation();
  }

  /*Commit the actuator command*/
  for (i = 0; i < INDI_NUM_ACT; i++) {
    actuators_pprz[i] = (int16_t) indi_u[i];
  }
}

/**
 * @param use_increment
 *
 * Function that sets the du_min, du_max and du_pref if function not elsewhere defined
 */
void WEAK stabilization_indi_set_wls_settings(float use_increment)
{
  // Calculate the min and max increments
  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    du_min_stab_indi[i] = -MAX_PPRZ * act_is_servo[i] - use_increment * actuator_state_filt_vect[i];
    du_max_stab_indi[i] = MAX_PPRZ - use_increment * actuator_state_filt_vect[i];
    du_pref_stab_indi[i] = act_pref[i] - use_increment * actuator_state_filt_vect[i];

#ifdef GUIDANCE_INDI_MIN_THROTTLE
    float airspeed = stateGetAirspeed_f();
    //limit minimum thrust ap can give
    if (!act_is_servo[i]) {
      if ((guidance_h.mode == GUIDANCE_H_MODE_HOVER) || (guidance_h.mode == GUIDANCE_H_MODE_NAV)) {
        if (airspeed < STABILIZATION_INDI_THROTTLE_LIMIT_AIRSPEED_FWD) {
          du_min_stab_indi[i] = GUIDANCE_INDI_MIN_THROTTLE - use_increment * actuator_state_filt_vect[i];
        } else {
          du_min_stab_indi[i] = GUIDANCE_INDI_MIN_THROTTLE_FWD - use_increment * actuator_state_filt_vect[i];
        }
      }
    }
#endif
  }
}

/**
 * @param enable_integrator
 * @param rate_control boolean that determines if we are in rate control or attitude control
 *
 * Function that should be called to run the INDI controller
 */
void stabilization_indi_attitude_run(struct Int32Quat quat_sp, bool in_flight)
{
  /* attitude error                          */
  struct FloatQuat att_err;
  struct FloatQuat *att_quat = stateGetNedToBodyQuat_f();
  struct FloatQuat quat_sp_f;

  QUAT_FLOAT_OF_BFP(quat_sp_f, quat_sp);
  float_quat_inv_comp_norm_shortest(&att_err, att_quat, &quat_sp_f);

  struct FloatVect3 att_fb;

#if TILT_TWIST_CTRL
  struct FloatQuat tilt;
  struct FloatQuat twist;
  float_quat_tilt_twist(&tilt, &twist, &att_err);
  att_fb.x = tilt.qx;
  att_fb.y = tilt.qy;
  att_fb.z = twist.qz;
#else
  att_fb.x = att_err.qx;
  att_fb.y = att_err.qy;
  att_fb.z = att_err.qz;
#endif

  // local variable to compute rate setpoints based on attitude error
  struct FloatRates rate_sp;

  // calculate the virtual control (reference acceleration) based on a PD controller
  rate_sp.p = indi_gains.att.p * att_fb.x / indi_gains.rate.p;
  rate_sp.q = indi_gains.att.q * att_fb.y / indi_gains.rate.q;
  rate_sp.r = indi_gains.att.r * att_fb.z / indi_gains.rate.r;

  // Add feed-forward rates to the attitude feedback part
  RATES_ADD(rate_sp, stab_att_ff_rates);

  // Store for telemetry
  angular_rate_ref.p = rate_sp.p;
  angular_rate_ref.q = rate_sp.q;
  angular_rate_ref.r = rate_sp.r;

  // Possibly we can use some bounding here
  BoundAbs(rate_sp.p, 2.0);
  BoundAbs(rate_sp.q, 2.0);
  BoundAbs(rate_sp.r, 2.0);

#if RADIO_PIVOT_SWITCH == FALSE
  /* compute the INDI command */
  stabilization_indi_rate_run(rate_sp, in_flight);
#else
  int8_t i;
  struct FloatEulers eulers_zxy;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  struct FloatRates * body_rates = stateGetBodyRates_f();
  float_eulers_of_quat_zxy(&eulers_zxy, statequat);

  takeoff_stage = take_off_stage(eulers_zxy.theta, body_rates->q);
  // theta_d = take_off_theta();
  if (takeoff_stage == 0){
    // initialize pivoting by putting motors up
	  actuators_pprz[0] = MAX_PPRZ;
	  actuators_pprz[1] = MAX_PPRZ;
	  actuators_pprz[2] = -MAX_PPRZ;
	  actuators_pprz[3] = -MAX_PPRZ;
    // don't use the ailerons for the takeoff
    actuators_pprz[4] = 0;
    actuators_pprz[5] = 0;
  } else if (takeoff_stage == 1 || takeoff_stage == 3) {
    struct FloatRates rates_filt_takeoff;
    rates_filt_takeoff.q = update_first_order_low_pass(&rates_filt_takeoff_fo[1], body_rates->q);
    if(autopilot.mode == AP_MODE_NAV){
    if (takeoff_stage == 1) {
    float theta_d_max = 0.0 / 180.0 * M_PI;
    float increment = t_scale_to_theta / PERIODIC_FREQUENCY;
           theta_d += increment;
       if (theta_d > theta_d_max) {
        theta_d = theta_d_max;
    }
  }
  else if (takeoff_stage == 3){
          // theta_d gradually decrease for nav mode
    float theta_d_min = -90.0 / 180.0 * M_PI;
    float increment = t_scale_to_theta / PERIODIC_FREQUENCY;
           theta_d -= increment;
       if (theta_d < theta_d_min) {
        theta_d = theta_d_min;
    }
  }
  }
  else{
    struct FloatEulers euler_sp;
    float_eulers_of_quat_zxy(&euler_sp, &quat_sp_f);
    theta_d = euler_sp.theta;
  }

    // Define B as a 1x2 matrix and W as a 2x2 diagonal matrix
        // Calculate B using the provided equation
    float B[NUM_OUT][TYPE_ACT] = {{sin(-eulers_zxy.theta ) * L1, m * 9.81 * L2 / L1 * cos(-eulers_zxy.theta ) * L1}};
  
    float W[TYPE_ACT][TYPE_ACT] = {{1 / pow(weight_T, 2), 0}, {0, weight_S / pow(63 * M_PI / 180, 2)}};
    float B_inv[TYPE_ACT][NUM_OUT];

    pseudoinv_B(B, W, B_inv);
   
    float integral_theta_error = 0.0f;
    float theta_error = theta_d - eulers_zxy.theta;//error between desired and actual pitch angle

    // Update the integrator with the current error
    integral_theta_error += theta_error;

    float du = pivot_gain_theta * theta_error + pivot_gain_i * integral_theta_error - pivot_gain_q * rates_filt_takeoff.q;
 
    float du_out[TYPE_ACT][1];
    // Multiply B_inv by du
    multiplyMatrixByScalar(B_inv, du, du_out); 
    float thrust_eq = m*9.81*L2/L1;
    float tilt_eq = -eulers_zxy.theta;
    float takeoff_thrust = (thrust_eq + du_out[0][0])*0.5; //for one motor half 
    float takeoff_tilt = tilt_eq + du_out[1][0];

    int16_t servo_command = (int16_t)fmax(fmin(takeoff_tilt / (63.0 / 180.0 * M_PI) * 9600, 9600), -9600);
    actuators_pprz[0] = servo_command;
    actuators_pprz[1] = servo_command;
    if (autopilot_get_motors_on()) {
    int16_t motor_command = calculatePPRZCommand(K1, K2, K3, takeoff_thrust);
    actuators_pprz[2] = motor_command - angular_accel_ref.p * roll_gain;
    actuators_pprz[3] = motor_command + angular_accel_ref.p * roll_gain;
    } else {
      for (i = 2; i < INDI_NUM_ACT; i++) {
        actuators_pprz[i] = -9600;
      }
    }

   // don't use the ailerons for the takeoff
    actuators_pprz[4] = 0;
    actuators_pprz[5] = 0;

    int8_t i;
    for (i = 0; i < INDI_NUM_ACT; i++) {
      indi_u[i] = actuators_pprz[i];
    }

    /* reset psi setpoint to current psi angle */
    heading_check = stabilization_attitude_get_heading_i();
    stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
    update_filters();
  } else if (takeoff_stage == 2){ // not in a takeoff stage, flying
	  /* compute the INDI command */
	  stabilization_indi_rate_run(rate_sp, in_flight);
  }
#endif
  indi_thrust_increment_set = false;
}

// This function reads rc commands
void stabilization_indi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif

  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

/**
 * Function that tries to get actuator feedback.
 *
 * If this is not available it will use a first order filter to approximate the actuator state.
 * It is also possible to model rate limits (unit: PPRZ/loop cycle)
 */
void get_actuator_state(void)
{
#if INDI_RPM_FEEDBACK
  float_vect_copy(actuator_state, act_obs, INDI_NUM_ACT);
#else
  //actuator dynamics
  int8_t i;
  float UNUSED prev_actuator_state;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    prev_actuator_state = actuator_state[i];

    actuator_state[i] = actuator_state[i]
                        + act_dyn_discrete[i] * (indi_u[i] - actuator_state[i]);

#ifdef STABILIZATION_INDI_ACT_RATE_LIMIT
    if ((actuator_state[i] - prev_actuator_state) > act_rate_limit[i]) {
      actuator_state[i] = prev_actuator_state + act_rate_limit[i];
    } else if ((actuator_state[i] - prev_actuator_state) < -act_rate_limit[i]) {
      actuator_state[i] = prev_actuator_state - act_rate_limit[i];
    }
#endif
  }

#endif
}

/**
 * @param ddx_error error in output change
 * @param i row of the matrix element
 * @param j column of the matrix element
 * @param mu learning rate
 *
 * Function that calculates an element of the G1 matrix.
 * The elements are stored in a different matrix,
 * because the old matrix is necessary to caclulate more elements.
 */
void calc_g1_element(float ddx_error, int8_t i, int8_t j, float mu)
{
  g1_est[i][j] = g1_est[i][j] - du_estimation[j] * mu * ddx_error;
}

/**
 * @param ddx_error error in output change
 * @param j column of the matrix element
 * @param mu learning rate
 *
 * Function that calculates an element of the G2 matrix.
 * The elements are stored in a different matrix,
 * because the old matrix is necessary to caclulate more elements.
 */
void calc_g2_element(float ddx_error, int8_t j, float mu)
{
  g2_est[j] = g2_est[j] - ddu_estimation[j] * mu * ddx_error;
}

/**
 * Function that estimates the control effectiveness of each actuator online.
 * It is assumed that disturbances do not play a large role.
 * All elements of the G1 and G2 matrices are be estimated.
 */
void lms_estimation(void)
{

  // Get the acceleration in body axes
  struct Int32Vect3 *body_accel_i;
  body_accel_i = stateGetAccelBody_i();
  ACCELS_FLOAT_OF_BFP(body_accel_f, *body_accel_i);

  // Filter the acceleration in z axis
  update_butterworth_2_low_pass(&acceleration_lowpass_filter, body_accel_f.z);

  // Calculate the derivative of the acceleration via finite difference
  float indi_accel_d = (acceleration_lowpass_filter.o[0]
                        - acceleration_lowpass_filter.o[1]) * PERIODIC_FREQUENCY;

  // Use xml setting for adaptive mu for lms
  // Set default value if not defined
#ifndef STABILIZATION_INDI_ADAPTIVE_MU
  float adaptive_mu_lr = 0.001;
#else
  float adaptive_mu_lr = STABILIZATION_INDI_ADAPTIVE_MU;
#endif

  // scale the inputs to avoid numerical errors
  float_vect_smul(du_estimation, actuator_state_filt_vectd, adaptive_mu_lr, INDI_NUM_ACT);
  float_vect_smul(ddu_estimation, actuator_state_filt_vectdd, adaptive_mu_lr / PERIODIC_FREQUENCY, INDI_NUM_ACT);

  float ddx_estimation[INDI_OUTPUTS] = {estimation_rate_dd[0], estimation_rate_dd[1], estimation_rate_dd[2], indi_accel_d};

  //Estimation of G
  // TODO: only estimate when du_norm2 is large enough (enough input)
  /*float du_norm2 = du_estimation[0]*du_estimation[0] + du_estimation[1]*du_estimation[1] +du_estimation[2]*du_estimation[2] + du_estimation[3]*du_estimation[3];*/
  int8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    // Calculate the error between prediction and measurement
    float ddx_error = - ddx_estimation[i];
    int8_t j;
    for (j = 0; j < INDI_NUM_ACT; j++) {
      ddx_error += g1_est[i][j] * du_estimation[j];
      if (i == 2) {
        // Changing the momentum of the rotors gives a counter torque
        ddx_error += g2_est[j] * ddu_estimation[j];
      }
    }

    // when doing the yaw axis, also use G2
    if (i == 2) {
      for (j = 0; j < INDI_NUM_ACT; j++) {
        calc_g2_element(ddx_error, j, mu2);
      }
    } else if (i == 3) {
      // If the acceleration change is very large (rough landing), don't adapt
      if (fabs(indi_accel_d) > 60.0) {
        ddx_error = 0.0;
      }
    }

    // Calculate the row of the G1 matrix corresponding to this axis
    for (j = 0; j < INDI_NUM_ACT; j++) {
      calc_g1_element(ddx_error, i, j, mu1[i]);
    }
  }

  bound_g_mat();

  // Save the calculated matrix to G1 and G2
  // until thrust is included, first part of the array
  float_vect_copy(g1[0], g1_est[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2, g2_est, INDI_NUM_ACT);

  // Calculate sum of G1 and G2 for Bwls
  sum_g1_g2();

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
  // Calculate the inverse of (G1+G2)
  calc_g1g2_pseudo_inv();
#endif
}

/**
 * Function that sums g1 and g2 to obtain the g1g2 matrix
 * It also undoes the scaling that was done to make the values readable
 */
void sum_g1_g2(void)
{
  int8_t i;
  int8_t j;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    for (j = 0; j < INDI_NUM_ACT; j++) {
      if (i != 2) {
        g1g2[i][j] = g1[i][j] / INDI_G_SCALING;
      } else {
        g1g2[i][j] = (g1[i][j] + g2[j]) / INDI_G_SCALING;
      }
    }
  }
}

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
/**
 * Function that calculates the pseudo-inverse of (G1+G2).
 * Make sure to sum of G1 and G2 before running this!
 */
void calc_g1g2_pseudo_inv(void)
{
  //G1G2*transpose(G1G2)
  //calculate matrix multiplication of its transpose INDI_OUTPUTSxnum_act x num_actxINDI_OUTPUTS
  float element = 0;
  int8_t row;
  int8_t col;
  int8_t i;
  for (row = 0; row < INDI_OUTPUTS; row++) {
    for (col = 0; col < INDI_OUTPUTS; col++) {
      element = 0;
      for (i = 0; i < INDI_NUM_ACT; i++) {
        element = element + g1g2[row][i] * g1g2[col][i];
      }
      g1g2_trans_mult[row][col] = element;
    }
  }

  //there are numerical errors if the scaling is not right.
  float_vect_scale(g1g2_trans_mult[0], 1000.0, INDI_OUTPUTS * INDI_OUTPUTS);

  //inverse of 4x4 matrix
  float_mat_inv_4d(g1g2inv, g1g2_trans_mult);

  //scale back
  float_vect_scale(g1g2inv[0], 1000.0, INDI_OUTPUTS * INDI_OUTPUTS);

  //G1G2'*G1G2inv
  //calculate matrix multiplication INDI_NUM_ACTxINDI_OUTPUTS x INDI_OUTPUTSxINDI_OUTPUTS
  for (row = 0; row < INDI_NUM_ACT; row++) {
    for (col = 0; col < INDI_OUTPUTS; col++) {
      element = 0;
      for (i = 0; i < INDI_OUTPUTS; i++) {
        element = element + g1g2[i][row] * g1g2inv[col][i];
      }
      g1g2_pseudo_inv[row][col] = element;
    }
  }
}
#endif

#if STABILIZATION_INDI_RPM_FEEDBACK
static void act_feedback_cb(uint8_t sender_id UNUSED, struct act_feedback_t *feedback, uint8_t num_act)
{
  int8_t i;
  for (i = 0; i < num_act; i++) {
    // Sanity check that index is valid
    if (feedback[i].idx < INDI_NUM_ACT && feedback[i].set.rpm) {
      int8_t idx = feedback[i].idx;
      act_obs[idx] = (feedback[i].rpm - get_servo_min(idx));
      act_obs[idx] *= (MAX_PPRZ / (float)(get_servo_max(idx) - get_servo_min(idx)));
      Bound(act_obs[idx], 0, MAX_PPRZ);
    }
  }
}
#endif

/**
 * ABI callback that obtains the thrust increment from guidance INDI
 */
static void thrust_cb(uint8_t UNUSED sender_id, struct FloatVect3 thrust_increment)
{
  indi_thrust_increment = thrust_increment;
  indi_thrust_increment_set = true;
}

static void bound_g_mat(void)
{
  int8_t i;
  int8_t j;
  for (j = 0; j < INDI_NUM_ACT; j++) {
    float max_limit;
    float min_limit;

    // Limit the values of the estimated G1 matrix
    for (i = 0; i < INDI_OUTPUTS; i++) {
      if (g1_init[i][j] > 0.0) {
        max_limit = g1_init[i][j] * INDI_ALLOWED_G_FACTOR;
        min_limit = g1_init[i][j] / INDI_ALLOWED_G_FACTOR;
      } else {
        max_limit = g1_init[i][j] / INDI_ALLOWED_G_FACTOR;
        min_limit = g1_init[i][j] * INDI_ALLOWED_G_FACTOR;
      }

      if (g1_est[i][j] > max_limit) {
        g1_est[i][j] = max_limit;
      }
      if (g1_est[i][j] < min_limit) {
        g1_est[i][j] = min_limit;
      }
    }

    // Do the same for the G2 matrix
    if (g2_init[j] > 0.0) {
      max_limit = g2_init[j] * INDI_ALLOWED_G_FACTOR;
      min_limit = g2_init[j] / INDI_ALLOWED_G_FACTOR;
    } else {
      max_limit = g2_init[j] / INDI_ALLOWED_G_FACTOR;
      min_limit = g2_init[j] * INDI_ALLOWED_G_FACTOR;
    }

    if (g2_est[j] > max_limit) {
      g2_est[j] = max_limit;
    }
    if (g2_est[j] < min_limit) {
      g2_est[j] = min_limit;
    }
  }
}

void invertDiagonalMatrix(float W[TYPE_ACT][TYPE_ACT], float W_inv[TYPE_ACT][TYPE_ACT]) {
    // Initialize the whole W_inv array to zero first
    for (int i = 0; i < TYPE_ACT; i++) {
        for (int j = 0; j < TYPE_ACT; j++) {
            W_inv[i][j] = 0.0f; // Initialize all elements to zero
        }
    }
    // Now invert only the diagonal elements
    for (int i = 0; i < TYPE_ACT; i++) {
        W_inv[i][i] = 1.0f / W[i][i]; // Only diagonal elements are non-zero
    }
}


void transposeMatrix(float matrix[NUM_OUT][TYPE_ACT], float transposed[TYPE_ACT][NUM_OUT]) {
    for (int i = 0; i < NUM_OUT; i++) {
        for (int j = 0; j < TYPE_ACT; j++) {
            transposed[j][i] = matrix[i][j];
        }
    }
}

void multiplyMatrices(float matrix1[TYPE_ACT][TYPE_ACT], float matrix2[NUM_OUT][TYPE_ACT], float result[NUM_OUT][TYPE_ACT]) {
    for (int i = 0; i < NUM_OUT; i++) {
        for (int j = 0; j < TYPE_ACT; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < TYPE_ACT; k++) {
                result[i][j] += matrix1[k][j] * matrix2[i][k];
            }
        }
    }
}

void multiplyMatrix2x2_2x1(float matrix1[TYPE_ACT][TYPE_ACT], float matrix2[TYPE_ACT][NUM_OUT], float result[TYPE_ACT][NUM_OUT]) {
    for (int i = 0; i < TYPE_ACT; i++) {
        result[i][0] = 0.0f; 
        for (int k = 0; k < TYPE_ACT; k++) { 
            result[i][0] += matrix1[i][k] * matrix2[k][0]; // 
        }
    }
}

// Function to multiply a 2x1 matrix by a scalar
void multiplyMatrixByScalar(float matrix[TYPE_ACT][1], float scalar, float result[TYPE_ACT][1]) {
    for (int i = 0; i < TYPE_ACT; i++) {
        result[i][0] = matrix[i][0] * scalar;
    }
}

void pseudoinv_B(float B[NUM_OUT][TYPE_ACT], float W[TYPE_ACT][TYPE_ACT], float B_inv[TYPE_ACT][NUM_OUT]) {
    float W_inv[TYPE_ACT][TYPE_ACT], B_transposed[TYPE_ACT][NUM_OUT], B_W_inv[NUM_OUT][TYPE_ACT],W_inv_B_transposed[TYPE_ACT][NUM_OUT];
    float B_W_inv_B_transposed[NUM_OUT][NUM_OUT], B_W_inv_B_transposed_inv;

    // Invert W
    invertDiagonalMatrix(W, W_inv);
    
    // Transpose B
    transposeMatrix(B, B_transposed);

    // Multiply B by W_inv
    multiplyMatrices(W_inv, B, B_W_inv);

    // Multiply W_inv*B_transposed
    multiplyMatrix2x2_2x1(W_inv, B_transposed, W_inv_B_transposed);

    // Multiply B_W_inv by B_transposed to get a scalar since B is 1x2 and B_W_inv is 1x2
    B_W_inv_B_transposed[0][0] = 0.0f;
    for (int k = 0; k < TYPE_ACT; k++) {
        B_W_inv_B_transposed[0][0] += B_W_inv[0][k] * B_transposed[k][0];
    }

    // Invert B_W_inv_B_transposed (it's a scalar in this case)
    B_W_inv_B_transposed_inv = 1.0f / B_W_inv_B_transposed[0][0];

    // Finally, calculate B_inv by multiplying W_inv_B_transposed by B_W_inv_B_transposed_inv
    for (int i = 0; i < TYPE_ACT; i++) {
        B_inv[i][0] = W_inv_B_transposed[i][0] * B_W_inv_B_transposed_inv;
    }
}

int16_t calculatePPRZCommand(float K1, float K2, float K3, float thrust) {
    float a = K1;
    float b = K2;
    float c = K3 - thrust;
    float discriminant = b * b - 4 * a * c;

    // Check for no real solutions
    if (discriminant < 0) {
        return -1; // Indicate no solution
    } else {
        float x = (-b + sqrt(discriminant)) / (2 * a);

        // Check if solutions are within the valid range and return the valid one
        if (x >= 0 && x <= 9600) {
            // Both solutions are valid, return the smallest integer larger than or equal to the smaller one
            return (int)ceil(x);
        } else if (x < 0) {
            return 0;
        } else  {
            return 9600;
        } 
    }
}
