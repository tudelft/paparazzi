/*
 * Copyright (C) 2023 Tomaso De Ponti <tmldeponti@tudelft.nl>
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

/** @file "modules/wind_tunnel/oneloop_indi.c"
 * @author Tomaso De Ponti <tmldeponti@tudelft.nl>
 * One loop (Guidance + Stabilization) INDI controller for the rotating wing drone RW3C
 */

#include "firmwares/rotorcraft/oneloop/oneloop_indi.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#ifdef   STABILIZATION_INDI_ROTWING_V3A
#include "modules/rot_wing_drone/wing_rotation_controller_v3a.h"
#endif
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "wls/wls_alloc.h"
#include <stdio.h>

/*Physical Properties RW3C*/
float m   = 6.5;  // [kg] Mass
float g   = 9.81; // [m/s^2] Gravitational Acceleration
float Ixx = 0.251; // [kg m^2] Inertia around x axis
float Iyy = 1.071; // [kg m^2] Inertia around y axis
float Ixx = 1.334; // [kg m^2] Inertia around z axis
/*Actuator Dynamics*/
float w_F = 15.23; // [rad/s] First order bandwidth of actuator
float w_B = 15.23; // [rad/s] First order bandwidth of actuator
float w_L = 12.66; // [rad/s] First order bandwidth of actuator
float w_R = 12.66; // [rad/s] First order bandwidth of actuator
float w_P = 24.07; // [rad/s] First order bandwidth of actuator
float w_a = 1;     // [rad/s] First order bandwidth of actuator
/*WLS Settings*/
/* WLS_ALLOC - Control allocation using weighted least squares.

  [u,W,iter] = wls_alloc(B,v,umin,umax,[Wv,Wu,ud,gamma,u0,W0,imax])

 Solves the weighted, bounded least-squares problem

   min ||Wu(u-ud)||^2 + ||Wu_tran(u)||^2 + gamma ||Wv(Bu-v)||^2

   subj. to  umin <= u <= umax

 using an active set method.

  Inputs:
  -------
 B        control effectiveness matrix (k x m)
 v        commanded virtual control (k x 1)
 umin     lower position limits (m x 1)
 umax     upper position limits (m x 1)
 Wv       virtual control weighting matrix (k x k) [I]
 Wu       control weighting matrix (m x m) [I]
 Wu_tran  control weighting matrix for transition (m x m) [I]
 ud       desired control (m x 1) [0]
 gamma    weight (scalar) [1e6]
 u0       initial point (m x 1)
 W0       initial working set (m x 1) [empty]
 imax     max no. of iterations [100]
 
  Outputs:
  -------
 u     optimal control
 W     optimal active set
 iter  no. of iterations (= no. of changes in the working set + 1)

                            0 if u_i not saturated
 Working set syntax: W_i = -1 if u_i = umin_i
                           +1 if u_i = umax_i

 See also: WLSC_ALLOC, IP_ALLOC, FXP_ALLOC, QP_SIM. */
float gamma       = 100000;
float Wv[6]       = {2.0,2.0,5.0,10.0,10.0,1.0}; // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
float Wu[11]      = {2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,1.0,1.8,2.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
float Wu_tran[11] = {0.0,0.0,0.0,0.0,3.0,3.0,3.0,3.0,0.0,0.0,0.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}

/*Define Variables used in control*/
struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
float actuator_state_filt_vect[INDI_NUM_ACT];
struct FloatRates angular_accel_ref = {0., 0., 0.};
float actuator_state[INDI_NUM_ACT];
//struct FloatRates rate_sp_telem = {0., 0., 0.};
float angular_acceleration[3] = {0., 0., 0.};
float indi_u[INDI_NUM_ACT];
float indi_du[INDI_NUM_ACT];
float g2_times_du;
float q_filt = 0.0;
float r_filt = 0.0;

/*Error Controller Gain Design*/
static float k_e_1_3_f(float p1, float p2, float p3) {return (p1*p2*p3)};
static float k_e_2_3_f(float p1, float p2, float p3) {return (p1*p2+p1*p3+p2*p3)};
static float k_e_3_3_f(float p1, float p2, float p3) {return (p1+p2+p3)};
static float k_e_1_2_f(float p1, float p2) {return (p1*p2)};
static float k_e_2_2_f(float p1, float p2) {return (p1+p2)};

/*Reference Model Gain Design*/
static float k_rm_1_3_f(float omega_n, float zeta, float p1) {return (omega_n*omega_n*p1)/(omega_n*omega_n+omega_n*p1*zeta*2.0)};
static float k_rm_2_3_f(float omega_n, float zeta, float p1) {return (omega_n*omega_n+omega_n*p1*zeta*2.0)/(p1+omega_n*zeta*2.0)};
static float k_rm_3_3_f(float omega_n, float zeta, float p1) {return p1+omega_n*zeta*2.0};
static float k_rm_1_2_f(float omega_n, float zeta) {return omega_n/(2.0*zeta)};
static float k_rm_2_2_f(float omega_n, float zeta) {return 2.0*zeta*omega_n};

/*Error Controller Definition*/
static float ec_3rd(float x_ref, float x_d_ref, float x_2d_ref, float x_3d_ref, float x, float x_d, float x_2d, float k1_e, float k2_e, float k3_e){
  float y_4d = k1_e*(x_ref-x)+k2_e*(x_d_ref-x_d)+k3_e*(x_2d_ref-x_2d)+x_3d_ref;
  return y_4d;
}
static float ec_2rd(float x_ref, float x_d_ref, float x_2d_ref, float x, float x_d, float k1_e, float k2_e){
  float y_3d = k1_e*(x_ref-x)+k2_e*(x_d_ref-x_d)+x_2d_ref;
  return y_3d;
}

/*Reference Model Definition*/
void rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm){
  float e_x      = k1_rm * (x_des- *x_ref);
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  float e_x_2d   = k3_rm * (e_x_d- *x_2d_ref);
  float *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

void rm_2rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm){
  float e_x      = k1_rm * (x_des- *x_ref);
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  float *x_2d_ref = e_x_d;
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/*Third Order Dynamics Approximation*/
static float w_approx(float p1, float p2, float p3, float rm_k){
  float tao = (p1*p2+p1*p3+p2*p3)/(p1*p2*p3)/(rm_k);
  return 1/tao;
}

/*Attitude Loop*/
float rm_k_attitude = 0.8;
float p1_att = 0.8*w_L;
float p2_att = p1_att;
float p3_att = p1_att;

float k_theta_e = k_e_1_3_f(p1_att,p2_att,p3_att);
float k_q_e     = k_e_2_3_f(p1_att,p2_att,p3_att);
float k_qdot_e  = k_e_3_3_f(p1_att,p2_att,p3_att);
float k_phi_e   = k_theta_e;
float k_p_e     = k_q_e;
float k_pdot_e  = k_qdot_e;

float k_theta_rm = k_rm_1_3_f(rm_k_attitude*p1_att,1.0,rm_k_attitude*p3_att);
float k_q_rm     = k_rm_2_3_f(rm_k_attitude*p1_att,1.0,rm_k_attitude*p3_att);
float k_qdot_rm  = k_rm_3_3_f(rm_k_attitude*p1_att,1.0,rm_k_attitude*p3_att);
float k_phi_rm   = k_theta_rm;
float k_p_rm     = k_q_rm;
float k_pdot_rm  = k_qdot_rm;

/*Position Loop*/
float rm_k_pos = 0.9;
float p1_pos   = 1.0;
float p2_pos   = p1_pos;
float p3_pos   = p1_pos;
float route_k  = 2.0;

float k_N_e    = k_e_1_3_f(p1_pos,p2_pos,p3_pos);
float k_vN_e   = k_e_2_3_f(p1_pos,p2_pos,p3_pos);
float k_aN_e   = k_e_3_3_f(p1_pos,p2_pos,p3_pos);
float k_E_e    = k_N_e;
float k_vE_e   = k_vN_e;
float k_aE_e   = k_aN_e;

float k_N_rm    = k_rm_1_3_f(rm_k_pos*p1_pos,1.0,rm_k_pos*p3_pos);
float k_vN_rm   = k_rm_2_3_f(rm_k_pos*p1_pos,1.0,rm_k_pos*p3_pos);
float k_aN_rm   = k_rm_3_3_f(rm_k_pos*p1_pos,1.0,rm_k_pos*p3_pos);
float k_E_rm    = k_N_rm;
float k_vE_rm   = k_vN_rm;
float k_aE_rm   = k_aN_rm;

float k_N_route    = k_rm_1_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
float k_vN_route   = k_rm_2_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
float k_aN_route   = k_rm_3_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
float k_E_route    = k_N_route;
float k_vE_route   = k_vN_route;
float k_aE_route   = k_aN_route;

/*Altitude Loop*/
float rm_k_alt = 0.9;
float p1_alt   = 2.0;
float p2_alt   = p1_alt;
float p3_alt   = p1_alt;

float k_D_e    = k_e_1_3_f(p1_alt,p2_alt,p3_alt);
float k_vD_e   = k_e_2_3_f(p1_alt,p2_alt,p3_alt);
float k_aD_e   = k_e_3_3_f(p1_alt,p2_alt,p3_alt);

float k_D_rm    = k_rm_1_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);
float k_vD_rm   = k_rm_2_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);
float k_aD_rm   = k_rm_3_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);

/*Heading Loop*/
float rm_k_head = 0.9;
float p1_head   = 0.4 * w_L;
float p2_head   = p1_head;

float k_r_e     = k_e_1_2_f(p1_head,p2_head);
float k_r_d_e   = k_e_2_2_f(p1_head,p2_head);

float k_r_rm    = k_rm_1_2_f(rm_k_head*p1_head,rm_k_head*p2_head);
float k_r_d_rm  = k_rm_2_2_f(rm_k_head*p1_head,rm_k_head*p2_head);

/** state eulers in zxy order */
struct FloatEulers eulers_zxy;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass filt_accel_body[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass yaw_filt;
//Butterworth2LowPass thrust_filt;
//Butterworth2LowPass accely_filt;

/*Filters Initialization*/
float oneloop_indi_filt_cutoff = 2.0;
float oneloop_indi_estimation_filt_cutoff = 2.0;

Butterworth2LowPass actuator_lowpass_filters[INDI_NUM_ACT];
Butterworth2LowPass estimation_input_lowpass_filters[INDI_NUM_ACT];
Butterworth2LowPass att_dot_meas_lowpass_filters[3];
Butterworth2LowPass att_dot_est_output_lowpass_filters[3];

static struct FirstOrderLowPass rates_filt_fo[3];
struct FloatVect3 body_accel_f;

void init_filters(void);
void init_filters(void)
{
  float tau = 1.0 / (2.0 * M_PI *oneloop_indi_filt_cutoff);
  float tau_est = 1.0 / (2.0 * M_PI * oneloop_indi_estimation_filt_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  // Filtering of the Inputs with 3 dimensions (e.g. rates and accelerations)
  int8_t i;
  for (i = 0; i < 3; i++) {
    // Filtering of the gyroscope
    init_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&att_dot_est_output_lowpass_filters[i], tau_est, sample_time, 0.0);
    // Filtering of the linear accelerations
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_body[i], tau, sample_time, 0.0);
  }

  // Filtering of the actuators
  for (i = 0; i < INDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&actuator_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], tau_est, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&yaw_filt, tau, sample_time, 0.0);

  // Init rate filter for feedback
  float time_constants[3] = {1.0 / (2 * M_PI * ONELOOP_INDI_FILT_CUTOFF_P), 1.0 / (2 * M_PI * ONELOOP_INDI_FILT_CUTOFF_Q), 1.0 / (2 * M_PI * ONELOOP_INDI_FILT_CUTOFF_R)};
  init_first_order_low_pass(&rates_filt_fo[0], time_constants[0], sample_time, stateGetBodyRates_f()->p);
  init_first_order_low_pass(&rates_filt_fo[1], time_constants[1], sample_time, stateGetBodyRates_f()->q);
  init_first_order_low_pass(&rates_filt_fo[2], time_constants[2], sample_time, stateGetBodyRates_f()->r);
}

void guidance_indi_propagate_filters(void);
/*Low pass the accelerometer measurements to remove noise from vibrations.The roll and pitch also need to be filtered to synchronize them with the acceleration. Called as a periodic function with PERIODIC_FREQ*/
void guidance_indi_propagate_filters(void) {
  struct NedCoor_f *accel = stateGetAccelNed_f();
  float accel_b_x = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->x);
  float accel_b_y = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  float accel_b_z = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
  struct FloatRates *body_rates = stateGetBodyRates_f();
  float rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);
  update_butterworth_2_low_pass(&roll_filt, eulers_zxy.phi);
  update_butterworth_2_low_pass(&pitch_filt, eulers_zxy.theta);
  update_butterworth_2_low_pass(&yaw_filt, eulers_zxy.psi);
  update_butterworth_2_low_pass(&filt_accel_body[0], accel_b_x);
  update_butterworth_2_low_pass(&filt_accel_body[1], accel_b_y);
  update_butterworth_2_low_pass(&filt_accel_body[2], accel_b_z);
  
  int8_t i;
  for (i = 0; i < 3; i++) {
    update_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], rate_vect[i]);
    update_butterworth_2_low_pass(&att_dot_est_output_lowpass_filters[i], rate_vect[i]);
    //Calculate the angular acceleration via finite difference
    angular_acceleration[i] = (measurement_lowpass_filters[i].o[0]- measurement_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;
    // Calculate derivatives for estimation
    float estimation_rate_d_prev = estimation_rate_d[i];
    estimation_rate_d[i] = (estimation_output_lowpass_filters[i].o[0] - estimation_output_lowpass_filters[i].o[1]) *PERIODIC_FREQUENCY;
    estimation_rate_dd[i] = (estimation_rate_d[i] - estimation_rate_d_prev) * PERIODIC_FREQUENCY;
}

/*Init function of oneloop controller*/
void oneloop_indi_init(void)
{
  // Initialize filters
  init_filters();

  AbiBindMsgRPM(RPM_SENSOR_ID, &rpm_ev, rpm_cb);
  //AbiBindMsgTHRUST(THRUST_INCREMENT_ID, &thrust_ev, thrust_cb);
  //AbiBindMsgTHRUSTBX(THRUST_BX_INCREMENT_ID, &thrust_bx_ev, thrust_bx_cb);

 // #ifdef STABILIZATION_INDI_PUSHER_PROP_EFFECTIVENESS
 // actuator_thrust_bx_pprz = -MAX_PPRZ;
 // thrust_bx_state_filt = 0;
 // thrust_bx_state = 0;
 // #endif
  float_vect_zero(actuator_state_filt_vect, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vectd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vectdd, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_d, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_dd, INDI_NUM_ACT);


  //Calculate G1G2_PSEUDO_INVERSE
  //sum_g1_g2();
  //calc_g1g2_pseudo_inv();

  int8_t i;
  // Initialize the array of pointers to the rows of g1g2
  //for (i = 0; i < INDI_OUTPUTS; i++) {
  //  Bwls[i] = g1g2[i];
  //}

  // Initialize the estimator matrices
  //float_vect_copy(g1_est[0], g1[0], INDI_OUTPUTS * INDI_NUM_ACT);
  //float_vect_copy(g2_est, g2, INDI_NUM_ACT);
  // Remember the initial matrices
  //float_vect_copy(g1_init[0], g1[0], INDI_OUTPUTS * INDI_NUM_ACT);
  //float_vect_copy(g2_init, g2, INDI_NUM_ACT);

  // Assume all non-servos are delivering thrust
  //num_thrusters = INDI_NUM_ACT;
  //for (i = 0; i < INDI_NUM_ACT; i++) {
  //  num_thrusters -= act_is_servo[i];
  //}

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INDI_G, send_indi_g);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_FULL_INDI, send_att_full_indi);
#endif
}

/**
 * Function that resets important values upon engaging Oneloop INDI.
 *
 * FIXME: Ideally we should distinguish between the "stabilization" and "guidance" needs because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 */
void oneloop_indi_enter(void)
{
  /* Stabilization Reset */
  // To-Do // stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
  float_vect_zero(du_estimation, INDI_NUM_ACT);
  float_vect_zero(ddu_estimation, INDI_NUM_ACT);

  /*Guidance Reset*/
  // To-D- // guidance_indi_hybrid_heading_sp = stateGetNedToBodyEulers_f()->psi;

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_body[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&yaw_filt, tau, sample_time, stateGetNedToBodyEulers_f()->yaw);
}

/*Function that calculates the failsafe setpoint*/
void oneloop_indi_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void oneloop_indi_attitude_run(struct FloatVect3 att_ref, struct FloatVect3 att_d_ref, struct FloatVect3 att_2d_ref, struct FloatVect3 att_3d_ref, bool in_flight)
{
  struct FloatVect3 att;
  struct FloatVect3 att_d; 
  struct FloatVect3 att_2d;

  att[3]    = {roll_filt.o[0],pitch_filt.o[0],yaw_filt.o[0]};
  att_d[3]  = {measurement_lowpass_filters[0].o[0],measurement_lowpass_filters[1].o[0],measurement_lowpass_filters[2].o[0]};
  att_2d[3] = angular_acceleration;

  struct FloatVect3 nu;
  nu[0] = ec_3rd(float att_ref[0], float att_d_ref[0], float att_2d_ref[0], float att_3d_ref[0], float att[0], float att_d[0], float att_2d[0], float k_phi_e, float k_p_e, float k_pdot_e);
  nu[1] = ec_3rd(float att_ref[1], float att_d_ref[1], float att_2d_ref[1], float att_3d_ref[1], float att[1], float att_d[1], float att_2d[1], float k_theta_e, float k_q_e, float k_qdot_e);
  nu[2] = ec_2rd(float att_d_ref[2], float att_2d_ref[2], float att_3d_ref[2], float att_d[2], float att_2d[2], float k_r_e, float kr_d_e);
  
  // local variable to compute rate setpoints based on attitude error
  struct FloatRates rate_sp;

  // calculate the virtual control (reference acceleration) based on a PD controller
  rate_sp.p = indi_gains.att.p * att_fb.x / indi_gains.rate.p;
  rate_sp.q = indi_gains.att.q * att_fb.y / indi_gains.rate.q;
  rate_sp.r = indi_gains.att.r * att_fb.z / indi_gains.rate.r;

  rate_sp_telem.p = rate_sp.p;
  rate_sp_telem.q = rate_sp.q;
  rate_sp_telem.r = rate_sp.r;

  // Possibly we can use some bounding here
  /*BoundAbs(rate_sp.r, 5.0);*/

  /* compute the INDI command */
  stabilization_indi_rate_run(rate_sp, in_flight);

  // Reset thrust increment boolean
  indi_thrust_increment_set = false;
  indi_thrust_bx_increment_set = false;
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
