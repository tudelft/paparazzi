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

/** @file "firmwares/rotorcraft/oneloop/oneloop_andi.h"
 * @author Tomaso De Ponti <tmldeponti@tudelft.nl>
 * One loop (Guidance + Stabilization) ANDI controller for rotorcrafts
 */
///// EXPLANATION OF HALFLOOP ///////////////////////////////////////////////////////////
/**
 * @param oneloop_andi_half_loop - A Boolean indicating the state of the oneloop controller.
 * @param oneloop_andi_half_loop = true - Control allocation is performed to accommodate desired Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk (ANDI).
 * @param oneloop_andi_half_loop = false - Control allocation is performed to accommodate desired Jerk North, Jerk East, Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk (ANDI).
 */
///// Enter functions change the state of the oneloop controller //////////////////////
/**
 * @fn stabilization_attitude_enter in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 * @result oneloop_andi_half_loop = true
 */
/**
 * @fn guidance_h_run_enter in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * @result oneloop_andi_half_loop = false
 */
/**
 * @fn guidance_v_run_enter in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * @result nothing
 */
///// Example of execution of the oneloop controller for two different states in the state machine /////////////////////
/**
 * @file "sw/airborne/firmwares/rotorcraft/autopilot_static.c"
 * 
 * @if MODE_ATTITUDE_RC_DIRECT
 * 
 * - @fn stabilization_attitude_run() in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 * 
 * - - @if half_loop
 * 
 * - - - @fn oneloop_andi_run(true) in @file "firmwares/rotorcraft/oneloop/oneloop_andi.c"
 * 
 * - - - @result: Control allocation is performed to accommodate desired Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk from stick inputs
 * - - @endif
 * 
 * @elseif MODE_NAV
 * 
 * - @fn guidance_h_run() in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * 
 * - @fn oneloop_andi_run(false) in @file "firmwares/rotorcraft/oneloop/oneloop_andi.c"
 * 
 * - @result: Control allocation is performed to accommodate desired Jerk North, Jerk East, Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk from navigation outputs
 * 
 * - @fn guidance_v_run() in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * 
 * - @result: nothing
 * 
 * - @fn stabilization_attitude_run() in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 * 
 * - @result: nothing because of oneloop_andi_half_loop = false
 * @endif
 */


/* Include necessary header files */
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "filters/notch_filter_float.h"
#include "math/wls/wls_alloc.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/rotwing_drone/rotwing_state.h"
#include "modules/core/commands.h"
#include "modules/ctrl/eff_scheduling_rotwing_V2.h"
#include "modules/system_identification/sys_id_doublet.h"
#include <stdio.h>
#if INS_EXT_POSE
#include "modules/ins/ins_ext_pose.h"
#endif

#include "modules/gps/gps.h" // DELETE FIX
//#include "nps/nps_fdm.h"

/*Define general struct of the Oneloop ANDI controller*/
struct OneloopGeneral oneloop_andi;

// Number of real actuators (e.g. motors, servos)
#ifndef ONELOOP_ANDI_NUM_THRUSTERS
float num_thrusters_oneloop = 4.0; // Number of motors used for thrust
#else
float num_thrusters_oneloop = ONELOOP_ANDI_NUM_THRUSTERS;
#endif

#ifndef ONELOOP_ANDI_SCHEDULING
#define ONELOOP_ANDI_SCHEDULING FALSE
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF
float  oneloop_andi_filt_cutoff = ONELOOP_ANDI_FILT_CUTOFF;
#else
float  oneloop_andi_filt_cutoff = 2.0;
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF_ACC
float  oneloop_andi_filt_cutoff_a = ONELOOP_ANDI_FILT_CUTOFF_ACC;
#else
float  oneloop_andi_filt_cutoff_a = 2.0;
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF_VEL
float  oneloop_andi_filt_cutoff_v = ONELOOP_ANDI_FILT_CUTOFF_VEL;
#else
float  oneloop_andi_filt_cutoff_v = 2.0;
#endif
PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF_VEL)
#ifdef ONELOOP_ANDI_FILT_CUTOFF_POS
float  oneloop_andi_filt_cutoff_pos = ONELOOP_ANDI_FILT_CUTOFF_POS;
#else
float  oneloop_andi_filt_cutoff_pos = 2.0;
#endif

// Stabilization Structural Modes Filtering ----------------------------------------

//const float ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ = 17.90;
//#define ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ
//#define ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ 17.90
//#define USE_YAW_LP4

// Roll Structural Mode Filtering
#ifdef ONELOOP_ANDI_ROLL_STRUCTURAL_MODE_FREQ
#if !defined(USE_ROLL_NOTCH) && !defined(USE_ROLL_LP) && !defined(USE_ROLL_LP4)
#error "Either USE_ROLL_NOTCH, USE_ROLL_LP4 or USE_ROLL_LP must be defined."
#elif (defined(USE_ROLL_NOTCH) && defined(USE_ROLL_LP)) || \
      (defined(USE_ROLL_NOTCH) && defined(USE_ROLL_LP4)) || \
      (defined(USE_ROLL_LP) && defined(USE_ROLL_LP4))
#error "Only one of USE_ROLL_NOTCH, USE_ROLL_LP4 or USE_ROLL_LP can be defined."
#endif
struct Oneloop_StructuralModes_t roll_structural_mode = {
#ifdef USE_ROLL_NOTCH
  .freq = ONELOOP_ANDI_ROLL_STRUCTURAL_MODE_FREQ,
  .bandwidth = 4.0,
  .filter_type = NOTCH,
#elif defined(USE_ROLL_LP) 
  .freq = 11.0 //ONELOOP_ANDI_ROLL_STRUCTURAL_MODE_FREQ-3.0, 
  .bandwidth = 0.0,
  .filter_type = BUTTERWORTH_2,
#elif defined(USE_ROLL_LP4)
  .freq = 11.0 //ONELOOP_ANDI_ROLL_STRUCTURAL_MODE_FREQ-3.0,
  .bandwidth = 0.0,
  .filter_type = BUTTERWORTH_4,
#endif
};
#endif
// Pitch Structural Mode Filtering
#ifdef ONELOOP_ANDI_PITCH_STRUCTURAL_MODE_FREQ
#if !defined(USE_PITCH_NOTCH) && !defined(USE_PITCH_LP) && !defined(USE_PITCH_LP4)
#error "Either USE_PITCH_NOTCH, USE_PITCH_LP4 or USE_PITCH_LP must be defined."
#elif (defined(USE_PITCH_NOTCH) && defined(USE_PITCH_LP)) || \
      (defined(USE_PITCH_NOTCH) && defined(USE_PITCH_LP4)) || \
      (defined(USE_PITCH_LP) && defined(USE_PITCH_LP4))
#error "Only one of USE_PITCH_NOTCH, USE_PITCH_LP4 or USE_PITCH_LP can be defined."
#endif
struct Oneloop_StructuralModes_t pitch_structural_mode = {
#ifdef USE_PITCH_NOTCH
  .freq = ONELOOP_ANDI_PITCH_STRUCTURAL_MODE_FREQ,
  .bandwidth = 4.0,
  .filter_type = NOTCH,
#elif defined(USE_PITCH_LP)
  .freq = ONELOOP_ANDI_PITCH_STRUCTURAL_MODE_FREQ-3.0,
  .bandwidth = 0.0,
  .filter_type = BUTTERWORTH_2,
#elif defined(USE_PITCH_LP4)
  .freq = ONELOOP_ANDI_PITCH_STRUCTURAL_MODE_FREQ-3.0,
  .bandwidth = 0.0,
  .filter_type = BUTTERWORTH_4,
#endif
};
#endif
// Yaw Structural Mode Filtering
#ifdef ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ
#if !defined(USE_YAW_NOTCH) && !defined(USE_YAW_LP) && !defined(USE_YAW_LP4)
#error "Either USE_YAW_NOTCH, USE_YAW_LP4 or USE_YAW_LP must be defined."
#elif (defined(USE_YAW_NOTCH) && defined(USE_YAW_LP)) || \
      (defined(USE_YAW_NOTCH) && defined(USE_YAW_LP4)) || \
      (defined(USE_YAW_LP) && defined(USE_YAW_LP4))
#error "Only one of USE_YAW_NOTCH, USE_YAW_LP4 or USE_YAW_LP can be defined."
#endif
struct Oneloop_StructuralModes_t yaw_structural_mode = {
#ifdef USE_YAW_NOTCH
  .freq = ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ,
  .bandwidth = 4.0,
  .filter_type = NOTCH,
#elif defined(USE_YAW_LP)
  .freq = 11.0, //ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ-3.0,
  .bandwidth = 0.0,
  .filter_type = BUTTERWORTH_2,
#elif defined(USE_YAW_LP4)
  .freq = 11.0, //ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ-3.0,
  .bandwidth = 0.0,
  .filter_type = BUTTERWORTH_4,
#endif
};
#endif

PRINT_CONFIG_VAR(ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ)
PRINT_CONFIG_VAR(yaw_structural_mode.freq)
// ---------------------------------------------------------------------------------
#ifdef  ONELOOP_ANDI_FILT_CUTOFF_P
#define ONELOOP_ANDI_FILTER_ROLL_RATE TRUE
float oneloop_andi_filt_cutoff_p = ONELOOP_ANDI_FILT_CUTOFF_P;
#else
float oneloop_andi_filt_cutoff_p = 20.0;
#endif

#ifdef  ONELOOP_ANDI_FILT_CUTOFF_Q
#define ONELOOP_ANDI_FILTER_PITCH_RATE TRUE
float oneloop_andi_filt_cutoff_q = ONELOOP_ANDI_FILT_CUTOFF_Q;
#else
float oneloop_andi_filt_cutoff_q = 20.0;
#endif

#ifdef  ONELOOP_ANDI_FILT_CUTOFF_R
#define ONELOOP_ANDI_FILTER_YAW_RATE TRUE
float oneloop_andi_filt_cutoff_r = ONELOOP_ANDI_FILT_CUTOFF_R;
#else
float oneloop_andi_filt_cutoff_r = 20.0;
#endif

PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF);
PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF_Q);
PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF_P);
PRINT_CONFIG_VAR(ONELOOP_ANDI_FILT_CUTOFF_R);
#ifndef MAX_R 
float max_r = RadOfDeg(120.0);
#else
float max_r = RadOfDeg(MAX_R);
#endif

#ifdef ONELOOP_ANDI_YAW_DISTURBANCE_LIMIT
float oneloop_andi_yaw_dist_limit = ONELOOP_ANDI_YAW_DISTURBANCE_LIMIT;
#else // Put a high limit
float oneloop_andi_yaw_dist_limit = 99999.f;
#endif

#ifdef ONELOOP_ANDI_ACT_IS_SERVO
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_IS_SERVO;
#else
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = {0};
#endif

#ifdef ONELOOP_ANDI_ACT_DYN
float  act_dynamics[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_DYN;
float  act_dyn_ctrl[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_DYN;
#else
#error "You must specify the actuator dynamics"
float  act_dynamics[ANDI_NUM_ACT_TOT] = = {1};
float  act_dyn_ctrl[ANDI_NUM_ACT_TOT] = = {1};
#endif

#ifdef ONELOOP_ANDI_ACT_MAX
float  act_max[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX;
#else
float  act_max[ANDI_NUM_ACT_TOT] = = {MAX_PPRZ};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN
float  act_min[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN;
#else
float  act_min[ANDI_NUM_ACT_TOT] = = {0.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MAX_NORM
float  act_max_norm[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX_NORM;
#else
float  act_max_norm[ANDI_NUM_ACT_TOT] = = {1.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN_NORM
float  act_min_norm[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN_NORM;
#else
float  act_min_norm[ANDI_NUM_ACT_TOT] = = {0.0};
#endif

#ifdef ONELOOP_ANDI_NU_NORM_MAX
float  nu_norm_max = ONELOOP_ANDI_NU_NORM_MAX;
#else
float  nu_norm_max = 1.0;
#endif

#ifdef ONELOOP_ANDI_U_PREF
static float u_pref[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_U_PREF;
#else
static float u_pref[ANDI_NUM_ACT_TOT] = {0.0};
#endif

#ifndef ONELOOP_ANDI_DEBUG_MODE
#define ONELOOP_ANDI_DEBUG_MODE  FALSE
#endif

// Assume phi and theta are the first actuators after the real ones unless otherwise specified
#define ONELOOP_ANDI_MAX_BANK  act_max[COMMAND_ROLL] // assuming abs of max and min is the same
#define ONELOOP_ANDI_MAX_PHI   act_max[COMMAND_ROLL] // assuming abs of max and min is the same

#define ONELOOP_ANDI_MAX_THETA   act_max[COMMAND_PITCH] // assuming abs of max and min is the same

#ifndef ONELOOP_THETA_PREF_MAX
float theta_pref_max = RadOfDeg(20.0);
#else
float theta_pref_max = RadOfDeg(ONELOOP_THETA_PREF_MAX);
#endif

#if ANDI_NUM_ACT_TOT != WLS_N_U_MAX
#error Matrix-WLS_N_U_MAX is not equal to the number of actuators: define WLS_N_U_MAX == ANDI_NUM_ACT_TOT in airframe file
#define WLS_N_U_MAX == ANDI_NUM_ACT_TOT
#endif
#if ANDI_OUTPUTS != WLS_N_V_MAX
#error Matrix-WLS_N_V_MAX is not equal to the number of controlled axis: define WLS_N_V_MAX == ANDI_OUTPUTS in airframe file
#define WLS_N_V_MAX == ANDI_OUTPUTS
#endif

#ifndef ONELOOP_ANDI_AIRSPEED_SWITCH_THRESHOLD
#define ONELOOP_ANDI_AIRSPEED_SWITCH_THRESHOLD 10.0
#endif

/* Declaration of Navigation Variables*/
#ifdef NAV_HYBRID_MAX_DECELERATION
float max_a_nav = NAV_HYBRID_MAX_DECELERATION;
#else
float max_a_nav = 4.0;   // (35[N]/6.5[Kg]) = 5.38[m/s2]  [0.8 SF]
#endif

#ifdef ONELOOP_ANDI_MAX_LINEAR_JERK
float max_j_lin = ONELOOP_ANDI_MAX_LINEAR_JERK;
#else
float max_j_lin = 500.0; // Pusher Test shows erros above 2[Hz] ramp commands [0.6 SF]
#endif

struct OneloopStabilizationRef sta_bounds = {
  #ifdef ONELOOP_ANDI_MAX_ANGULAR_JERK
  .att_3d[0] = ONELOOP_ANDI_MAX_ANGULAR_JERK,
  .att_3d[1] = ONELOOP_ANDI_MAX_ANGULAR_JERK,
  #else
  .att_3d[0] = RadOfDeg(100000.0),
  .att_3d[1] = RadOfDeg(100000.0),
  #endif

  #ifdef ONELOOP_ANDI_MAX_ANGULAR_JERK_YAW
  .att_3d[2] = ONELOOP_ANDI_MAX_ANGULAR_JERK_YAW,
  #else
  .att_3d[2] = RadOfDeg(1520.0),
  #endif

  #ifdef ONELOOP_ANDI_MAX_ANGULAR_ACCEL
  .att_2d[0] = ONELOOP_ANDI_MAX_ANGULAR_ACCEL,
  .att_2d[1] = ONELOOP_ANDI_MAX_ANGULAR_ACCEL,
  #else
  .att_2d[0] = RadOfDeg(10000.0),
  .att_2d[1] = RadOfDeg(10000.0),
  #endif

  #ifdef ONELOOP_ANDI_MAX_ANGULAR_ACCEL_YAW
  .att_2d[2] = ONELOOP_ANDI_MAX_ANGULAR_ACCEL_YAW,
  #else
  .att_2d[2] = RadOfDeg(130.0),
  #endif

  #ifdef ONELOOP_ANDI_MAX_ANGULAR_VEL
  .att_d[0] = ONELOOP_ANDI_MAX_ANGULAR_VEL,
  .att_d[1] = ONELOOP_ANDI_MAX_ANGULAR_VEL,
  #else
  .att_d[0] = RadOfDeg(10000.0),
  .att_d[1] = RadOfDeg(10000.0),
  #endif

  #ifdef ONELOOP_ANDI_MAX_ANGULAR_VEL_YAW
  .att_d[2] = ONELOOP_ANDI_MAX_ANGULAR_VEL_YAW,
  #else
  .att_d[2] = RadOfDeg(30.0),
  #endif
};

#ifdef NAV_HYBRID_MAX_AIRSPEED
float max_v_nav = NAV_HYBRID_MAX_AIRSPEED; // Consider implications of difference Ground speed and airspeed
#else
float max_v_nav = 5.0;
#endif
float max_as = 19.0f;
float min_as = 0.0f;

#ifdef NAV_HYBRID_MAX_SPEED_V
float max_v_nav_v = NAV_HYBRID_MAX_SPEED_V;
#else
float max_v_nav_v = 1.5;
#endif

#ifndef FWD_SIDESLIP_GAIN
float fwd_sideslip_gain = 0.2;
#else
float fwd_sideslip_gain = FWD_SIDESLIP_GAIN;
#endif

#ifndef ONELOOP_ANDI_WU_QUAD_MOTORS_FWD
float Wu_quad_motors_fwd = 6.0;
#else
float Wu_quad_motors_fwd = ONELOOP_ANDI_WU_QUAD_MOTORS_FWD;
#endif


/*  Define Section of the functions used in this module*/
void  init_poles(void);
void  init_poles_att(void);
void  init_poles_pos(void);
void  calc_normalization(void);
void  normalize_nu(void);
void  G1G2_oneloop(int ctrl_type);
void  get_act_state_oneloop(void);
void  oneloop_andi_propagate_filters(void);
void  init_filter(void);
void  init_controller_gains(void);
void  reinit_controller(void);
void  float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3]);
void  float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3]);
void  err_nd(float err[], float a[], float b[], float k[], int n);
void  err_sum_nd(float err[], float a[], float b[], float k[], float c[], int n);
void  integrate_nd(float dt, float a[], float a_dot[], int n);
void  vect_bound_nd(float vect[], float bound, int n);
void  acc_body_bound(struct FloatVect2* vect, float bound);
float bound_v_from_a(float e_x[], float v_bound, float a_bound, int n);
void  rm_2nd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm);
void  rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3], struct OneloopStabilizationRef bounds);
void  rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n);
void  rm_2nd_pos(float dt, float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_d_des[], float k2_rm[], float k3_rm[], float x_2d_bound, float x_3d_bound, int n);
void  rm_1st_pos(float dt, float x_2d_ref[], float x_3d_ref[], float x_2d_des[], float k3_rm[], float x_3d_bound, int n);
void  ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3], struct OneloopStabilizationRef bounds, float fb[3]);
void  ec_3rd_pos(float y_4d[], float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x[], float x_d[], float x_2d[], float k1_e[], float k2_e[], float k3_e[], float x_d_bound, float x_2d_bound, float x_3d_bound, float fb[], int n);
void  calc_model(void);
float oneloop_andi_sideslip(void);
void  reshape_wind(void);
void  chirp_pos(float time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]);
void  chirp_call(bool* chirp_on, bool* chirp_first_call, float* t_0_chirp, float* time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]);
void  oneloop_axis_effectiveness_calc(void);
void  oneloop_andi_bound_disturbance(void);
void  oneloop_calc_model_disturbance(bool in_flight);
void  dynFilter_init(struct Oneloop_DynFilt_t *mu, float varepsilon, float sigma);
void  dynFilter_run(struct Oneloop_DynFilt_t *mu, float u_c, float sigma);

/* Oneloop Misc variables*/
static float use_increment = 0.0;
static float nav_target[3]; // Can be a position, speed or acceleration depending on the guidance H mode
static float nav_target_new[3];
static float dt_1l = 1./PERIODIC_FREQUENCY;
static float g   = 9.81; // [m/s^2] Gravitational Acceleration
float k_as = 2.0;
int16_t temp_pitch = 0;
float gi_unbounded_airspeed_sp = 0.0;

/* Oneloop Control Variables*/
float andi_u[ANDI_NUM_ACT_TOT];
float andi_du[ANDI_NUM_ACT_TOT];
static float andi_u_n[ANDI_NUM_ACT_TOT];
float nu[ANDI_OUTPUTS];
float nu_n[ANDI_OUTPUTS];
static float act_dynamics_d[ANDI_NUM_ACT_TOT];
float actuator_state_1l[ANDI_NUM_ACT_TOT];
static float a_thrust = 0.0;
static float g2_ff= 0.0;

/*Attitude related variables*/
struct Int32Eulers stab_att_sp_euler_1l;// here for now to correct warning, can be better eploited in the future 
struct Int32Quat   stab_att_sp_quat_1l; // here for now to correct warning, can be better eploited in the future
struct FloatEulers eulers_zxy_des;
struct FloatEulers eulers_zxy;
//static float  psi_des_rad = 0.0;
float  psi_des_rad = 0.0;
float  psi_des_deg = 0.0;
static float  psi_vec[4]  = {0.0, 0.0, 0.0, 0.0};
static float  phi_vec[4] = {0.0, 0.0, 0.0, 0.0};

#if ONELOOP_ANDI_HEADING_MANUAL
bool heading_manual = true;
#else
bool heading_manual = false;
#endif
#if ONELOOP_ANDI_YAW_STICK_IN_AUTO
bool yaw_stick_in_auto = true;
#else
bool yaw_stick_in_auto = false;
#endif
bool ctrl_off = false;
/*WLS Settings*/

static float pitch_pref = 0;
struct WLS_t WLS_one_p = {
  .nu        = ANDI_NUM_ACT_TOT,
  .nv        = ANDI_OUTPUTS,
  .gamma_sq  = 1000.0,
  .v         = {0.0},
#ifdef ONELOOP_ANDI_WV // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}  
  .Wv        = ONELOOP_ANDI_WV,
#else
  .Wv        = {1.0},
#endif
#ifdef ONELOOP_ANDI_WU // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}  
  .Wu        = ONELOOP_ANDI_WU,
#else
  .Wu        = {1.0},
#endif
  .u_pref    = {0.0},
  .u_min     = {0.0},
  .u_max     = {0.0},
  .PC        = 0.0,
  .SC        = 0.0,
  .iter      = 0
};

#ifdef ONELOOP_ANDI_WV // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
static float Wv_backup[ANDI_OUTPUTS] = ONELOOP_ANDI_WV;
#else
static float Wv_backup[ANDI_OUTPUTS] = {1.0};
#endif

#ifdef ONELOOP_ANDI_WU // {mF,mR,mB,mL,mP,de,dr,da,df,phi,theta}
static float Wu_backup[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_WU;
#else
static float Wu_backup[ANDI_NUM_ACT_TOT] = {1.0};
#endif

/*Filter Variables*/
#define USE_BW2
struct Oneloop_LP_t LP;
struct Oneloop_LP_t oneloop_andi_model_filt;
struct Oneloop_DynFilt_t mu_mL;
struct Oneloop_DynFilt_t mu_mR;  
bool  use_dyn_filter = true; 
float oneloop_andi_sigma = 29.0;
float oneloop_andi_sigma_max = 29.0;
float oneloop_andi_sigma_min = 8.0; 
/*Chirp test Variables*/
bool  chirp_on            = false;
bool  chirp_first_call    = true;
float time_elapsed_chirp  = 0.0;
float t_0_chirp           = 0.0;
float f0_chirp            = 0.2;//0.8 / (2.0 * M_PI);
float f1_chirp            = 0.2;//0.8 / (2.0 * M_PI);
float t_chirp             = 0.5;
float A_chirp             = 80.0;
int8_t chirp_axis         = 6;
float p_ref_0[3]          = {0.0, 0.0, 0.0};
     
/*Declaration of Reference Model and Error Controller Gains*/
struct PolePlacement p_att_e;
struct PolePlacement p_att_rm;
/*Position Loop*/
struct PolePlacement p_pos_e;  
struct PolePlacement p_pos_rm;
/*Altitude Loop*/
struct PolePlacement p_alt_e;   
struct PolePlacement p_alt_rm; 
/*Heading Loop*/
struct PolePlacement p_head_e;
struct PolePlacement p_head_rm;
/*Gains of EC and RM ANDI*/
struct Gains3rdOrder k_att_e;
struct Gains3rdOrder k_att_rm;  
struct Gains3rdOrder k_pos_e;
struct Gains3rdOrder k_pos_rm; 
/*Gains of EC and RM INDI*/
struct Gains3rdOrder k_att_e_indi;
struct Gains3rdOrder k_pos_e_indi;

/* Effectiveness Matrix definition */
float *bwls_1l[ANDI_OUTPUTS];
float EFF_MAT_G[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
float EFF_MAT_AXIS[ANDI_OUTPUTS];
float n_array[ANDI_OUTPUTS];
float m_array[ANDI_NUM_ACT_TOT];
float coupling_factor[ANDI_OUTPUTS];
float oneloop_andi_model[ANDI_OUTPUTS];
float oneloop_andi_dist_bound[ANDI_OUTPUTS];
float SF_BOUND_NU[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
float ratio_u_un[ANDI_NUM_ACT_TOT];
float ratio_vn_v[ANDI_OUTPUTS];

float temp_k = 3.0;
float temp_checks[2];
float temp_checks_2[3];
float temp_ref_att[3];
float temp_dist_r = 0.0;
float temp_ec_r = 0.0;
float temp_ec_r_2 = 0.0;
float  temp_e_x = 0.0;
float  temp_e_x_rates = 0.0;
float  temp_x_d_f = 0.0;
float  temp_x_2d_f = 0.0;
bool drop_yaw = true;
int16_t counter_andi = 0;
/*Filters Initialization*/
static Butterworth2LowPass filt_veloc_N;                 // Low pass filter for velocity NED - oneloop_andi_filt_cutoff_a (tau_a)       
static Butterworth2LowPass filt_veloc_E;
static Butterworth2LowPass filt_veloc_D;
static Butterworth2LowPass accely_filt;                       // Low pass filter for acceleration in y direction                - oneloop_andi_filt_cutoff (tau)
static Butterworth2LowPass airspeed_filt;                     // Low pass filter for airspeed                                - oneloop_andi_filt_cutoff (tau)
static Butterworth2LowPass u_filt[ANDI_NUM_ACT_TOT];          // Low pass filter for actuators                                
/* Define messages of the module*/
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_wls_v_oneloop(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_v("one", &WLS_one_p, trans, dev); 
}
static void send_wls_u_oneloop(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_u("one", &WLS_one_p, trans, dev); 
}
static void send_eff_mat_stab_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0;
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID, 
                ANDI_NUM_ACT, EFF_MAT_RW[3],
                ANDI_NUM_ACT, EFF_MAT_RW[4],
                ANDI_NUM_ACT, EFF_MAT_RW[5], 
                                   1, &zero,
                ANDI_NUM_ACT, G2_RW);
}

static void send_eff_mat_guid_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_EFF_MAT_GUID(trans, dev, AC_ID, 
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[0],
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[1],
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[2]);
}
static void send_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  float temp_eulers_zxy_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                                        3, temp_eulers_zxy_des,
                                        3, oneloop_andi.sta_state.att,
                                        3, oneloop_andi.sta_ref.att,
                                        3, oneloop_andi.sta_state.att_d,
                                        3, oneloop_andi.sta_ref.att_d,                                       
                                        3, oneloop_andi.sta_state.att_2d,
                                        3, oneloop_andi.sta_ref.att_2d,
                                        3, oneloop_andi.sta_ref.att_3d,                                       
                                        ANDI_NUM_ACT, actuator_state_1l);                                      
}
// static void send_oneloop_actuator_state(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_ACTUATOR_STATE(trans, dev, AC_ID, 
//                                         ANDI_NUM_ACT, actuator_state_1l,
//                                         ANDI_NUM_ACT_TOT, andi_u);
// }
static void send_guidance_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE(trans, dev, AC_ID,
                                        &oneloop_andi.gui_ref.pos[0],
                                        &oneloop_andi.gui_ref.pos[1],
                                        &oneloop_andi.gui_ref.pos[2],
                                        &oneloop_andi.gui_state.pos[0],
                                        &oneloop_andi.gui_state.pos[1],
                                        &oneloop_andi.gui_state.pos[2],
                                        &oneloop_andi.gui_ref.vel[0],
                                        &oneloop_andi.gui_ref.vel[1],
                                        &oneloop_andi.gui_ref.vel[2],
                                        &oneloop_andi.gui_state.vel[0],
                                        &oneloop_andi.gui_state.vel[1],
                                        &oneloop_andi.gui_state.vel[2],
                                        &oneloop_andi.gui_ref.acc[0],
                                        &oneloop_andi.gui_ref.acc[1],
                                        &oneloop_andi.gui_ref.acc[2],
                                        &oneloop_andi.gui_state.acc[0],
                                        &oneloop_andi.gui_state.acc[1],
                                        &oneloop_andi.gui_state.acc[2],
                                        &oneloop_andi.gui_ref.jer[0],
                                        &oneloop_andi.gui_ref.jer[1],
                                        &oneloop_andi.gui_ref.jer[2]);
}

static void debug_vect(struct transport_tx *trans, struct link_device *dev, char *name, float *data, int datasize)
{
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID,
                           strlen(name), name,
                           datasize, data);
}

static void send_oneloop_debug(struct transport_tx *trans, struct link_device *dev)
{
  float temp_debug_vect[14];
  temp_debug_vect[0] = oneloop_andi_model[0];//oneloop_andi_model_filt.ax.out;
  temp_debug_vect[1] = oneloop_andi_model[1];//oneloop_andi_model_filt.ay.out;
  temp_debug_vect[2] = oneloop_andi_model[2];//oneloop_andi_model_filt.az.out;
  temp_debug_vect[3] = oneloop_andi_model[3];//oneloop_andi_model_filt.p_dot.out;
  temp_debug_vect[4] = oneloop_andi_model[4];//oneloop_andi_model_filt.q_dot.out;
  temp_debug_vect[5] = oneloop_andi_model[5];//oneloop_andi_model_filt.r_dot.out;
  temp_debug_vect[6] = oneloop_andi_sigma;
  temp_debug_vect[7] = LP.ay.meas;
  temp_debug_vect[8] = LP.az.meas;
  temp_debug_vect[9] = temp_checks_2[0];
  temp_debug_vect[10] = temp_checks_2[1];
  temp_debug_vect[11] = temp_ref_att[0];
  temp_debug_vect[12] = temp_ref_att[1];
  temp_debug_vect[13] = temp_ref_att[2];
  debug_vect(trans, dev, "APF", temp_debug_vect, 14);
}

#endif

/** @brief Function to make sure that inputs are positive non zero vaues*/
static float positive_non_zero(float input)
{
  if (input < FLT_EPSILON) {
    input = 0.00001;
  }
  return input;
}

/** @brief  Error Controller Gain Design */

static float k_e_1_2_f_v2(float omega, float zeta) {
    omega = positive_non_zero(omega);
    zeta  = positive_non_zero(zeta);
    return (omega * omega);
}

static float k_e_2_2_f_v2(float omega, float zeta) {
    omega = positive_non_zero(omega);
    zeta  = positive_non_zero(zeta);
    return (2* zeta * omega);
}

static float k_1_2_f(float omega, float zeta) {
  omega = positive_non_zero(omega);
  zeta  = positive_non_zero(zeta);
  return (omega / (2*zeta));
}

static float k_2_2_f(float omega, float zeta) {
  omega = positive_non_zero(omega);
  zeta  = positive_non_zero(zeta);
  return (2* zeta * omega);
}

/** @brief Reference Model Gain Design */

static float k_rm_1_3_f(float omega_n, float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    zeta    = positive_non_zero(zeta);
    p1      = positive_non_zero(p1);
    return (omega_n * omega_n * p1) / (omega_n * omega_n + omega_n * p1 * zeta * 2.0);
}

static float k_rm_2_3_f(float omega_n, float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    zeta    = positive_non_zero(zeta);
    p1      = positive_non_zero(p1);
    return (omega_n * omega_n + omega_n * p1 * zeta * 2.0) / (p1 + omega_n * zeta * 2.0);
}

static float k_rm_3_3_f(float omega_n, float zeta, float p1) {
    omega_n = positive_non_zero(omega_n);
    zeta    = positive_non_zero(zeta);
    p1      = positive_non_zero(p1);
    return p1 + omega_n * zeta * 2.0;
}

/** @brief Attitude Rates to Euler Conversion Function */
void float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3])
{
  float sphi   = sinf(e[0]);
  float cphi   = cosf(e[0]);
  float stheta = sinf(e[1]);
  float ctheta = cosf(e[1]);
  r[0] =  edot[0] - stheta * edot[2];
  r[1] =  cphi * edot[1] + sphi * ctheta * edot[2];
  r[2] = -sphi * edot[1] + cphi * ctheta * edot[2];
}

/** @brief Attitude Euler to Rates Conversion Function */
void float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3])
{
  float sphi   = sinf(e[0]);
  float cphi   = cosf(e[0]);
  float stheta = sinf(e[1]);
  float ctheta = cosf(e[1]);
  if (fabs(ctheta) < FLT_EPSILON){
    ctheta = FLT_EPSILON;
  }
  edot[0] = r[0] + sphi*stheta/ctheta*r[1] + cphi*stheta/ctheta*r[2];
  edot[1] = cphi*r[1] - sphi*r[2];
  edot[2] = sphi/ctheta*r[1] + cphi/ctheta*r[2];
}

/** @brief Calculate Scaled Error between two 3D arrays*/
void err_nd(float err[], float a[], float b[], float k[], int n)
{
  int8_t i;
  for (i = 0; i < n; i++) {
    err[i] = k[i] * (a[i] - b[i]);
  }
}

/** @brief Calculate Scaled Error between two 3D arrays*/
void err_sum_nd(float err[], float a[], float b[], float k[], float c[], int n)
{
  int8_t i;
  for (i = 0; i < n; i++) {
    err[i] = k[i] * (a[i] - b[i]);
    err[i] += c[i];
  }
}

/** @brief Integrate in time 3D array*/
void integrate_nd(float dt, float a[], float a_dot[], int n)
{
  int8_t i;
  for (i = 0; i < n; i++) {
    a[i] = a[i] + dt * a_dot[i];
  }
}

/** @brief Scale a 3D array to within a 3D bound */
void vect_bound_nd(float vect[], float bound, int n) {
  float norm = float_vect_norm(vect,n);
  norm = positive_non_zero(norm);
  if((norm-bound) > FLT_EPSILON) {
    float scale = bound/norm;
    int8_t i;
    for(i = 0; i < n; i++) {
      vect[i] *= scale;
    }
  }
}

/** @brief Scale a 3D array to within a 3D bound */
void acc_body_bound(struct FloatVect2* vect, float bound) {
  int n = 2;
  float v[2] = {vect->x, vect->y};
  float sign_v0 = (v[0] > 0.f) ? 1.f : (v[0] < 0.f) ? -1.f : 0.f;
  float sign_v1 = (v[1] > 0.f) ? 1.f : (v[1] < 0.f) ? -1.f : 0.f;
  float norm = float_vect_norm(v,n);
  v[0] = fabsf(v[0]);
  v[1] = fabsf(v[1]);
  norm = positive_non_zero(norm);
  if((norm-bound) > FLT_EPSILON) {
    v[0] = Min(v[0], bound);
    float acc_b_y_2 = bound*bound - v[0]*v[0];
    acc_b_y_2 = positive_non_zero(acc_b_y_2);
    v[1] = sqrtf(acc_b_y_2);
  }
  vect->x = sign_v0*v[0];
  vect->y = sign_v1*v[1];
}

/** @brief Calculate velocity limit based on acceleration limit */
float bound_v_from_a(float e_x[], float v_bound, float a_bound, int n) {
  float norm = float_vect_norm(e_x,n);
  norm = fmaxf(norm, 1.0);
  float v_bound_a  = sqrtf(fabs(2.0 * a_bound * norm));
  return fminf(v_bound, v_bound_a);
}

/** 
 * @brief Reference Model Definition for 3rd order system with attitude conversion functions
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param ow_psi          Overwrite psi (for navigation functions) [bool]
 * @param psi_overwrite   Overwrite psi (for navigation functions) [values]
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 */
void rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3], struct OneloopStabilizationRef bounds){
  float e_x[3];
  float e_x_rates[3];
  float e_x_d[3];
  float e_x_2d[3];
  float x_d_eul_ref[3];
 
  float x_2d_ref_ubd[3]; // Unbounded 2nd degree reference
  float x_d_ref_ubd[3];  // Unbounded 1st degree reference
  // Attitude error -----------------------------------------------------
  err_nd(e_x, x_des, x_ref, k1_rm, 3);
  float temp_diff = x_des[2] - x_ref[2];
  NormRadAngle(temp_diff);
  e_x[2] = k1_rm[2] * temp_diff; // Correction for Heading error +-Pi
  float_rates_of_euler_dot_vec(e_x_rates, x_ref, e_x);
  BoundAbs(e_x_rates[0], bounds.att_d[0]);
  BoundAbs(e_x_rates[1], bounds.att_d[1]);
  BoundAbs(e_x_rates[2], bounds.att_d[2]);
  temp_ref_att[0] = e_x_rates[2];
  // Angular Rate error -------------------------------------------------
  err_nd(e_x_d, e_x_rates, x_d_ref, k2_rm, 3);
  BoundAbs(e_x_d[0], bounds.att_2d[0]);
  BoundAbs(e_x_d[1], bounds.att_2d[1]);
  BoundAbs(e_x_d[2], bounds.att_2d[2]);
  temp_ref_att[1] = e_x_d[2];
  //float temp_bound_r_dot = n_array[5]*coupling_factor[5]/(k_att_e.k3[2]*1.5);
  //BoundAbs(e_x_d[2], temp_bound_r_dot);
  // Angular Acceleration error -----------------------------------------
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, 3);
  BoundAbs(e_x_2d[0], bounds.att_3d[0]);
  BoundAbs(e_x_2d[1], bounds.att_3d[1]);
  BoundAbs(e_x_2d[2], bounds.att_3d[2]);
  temp_ref_att[2] = e_x_2d[2];
  // Angular Jerk Reference ---------------------------------------------
  float_vect_copy(x_3d_ref,e_x_2d,3);
  //BoundAbs(x_3d_ref[0], bounds.att_3d[0]);
  //BoundAbs(x_3d_ref[1], bounds.att_3d[1]);
  //BoundAbs(x_3d_ref[2], bounds.att_3d[2]);
  //vect_bound_nd(x_3d_ref, max_ang_jerk, 3);
  if(ow_psi){x_3d_ref[2] = psi_overwrite[3];}
  // Angular Acceleration Reference -------------------------------------
  integrate_nd(dt, x_2d_ref, x_3d_ref, 3);
  float_vect_copy(x_2d_ref_ubd,x_2d_ref,3);
  //BoundAbs(x_2d_ref[0], bounds.att_2d[0]);
  //BoundAbs(x_2d_ref[1], bounds.att_2d[1]);
  //BoundAbs(x_2d_ref[2], bounds.att_2d[2]);
  if(ow_psi){x_2d_ref[2] = psi_overwrite[2];}
  // Angular Rate Reference ---------------------------------------------
  integrate_nd(dt, x_d_ref, x_2d_ref, 3);
  float_vect_copy(x_d_ref_ubd,x_d_ref,3);
  //BoundAbs(x_d_ref[0], bounds.att_d[0]);
  //BoundAbs(x_d_ref[1], bounds.att_d[1]);
  //BoundAbs(x_d_ref[2], bounds.att_d[2]);
  if(ow_psi){x_d_ref[2] = psi_overwrite[1];}
  // Attitude Reference ------------------------------------------------
  float_euler_dot_of_rates_vec(x_d_ref, x_ref, x_d_eul_ref);
  integrate_nd(dt, x_ref, x_d_eul_ref, 3);
  if(ow_psi){x_ref[2] = psi_overwrite[0];}
  NormRadAngle(x_ref[2]);
  // Anti-windup Correction ---------------------------------------------
  // for (int i = 0; i < 3; i++) {
  //   x_3d_ref[i] = x_3d_ref[i] + (x_2d_ref[i]-x_2d_ref_ubd[i])/(dt) + (x_d_ref[i]-x_d_ref_ubd[i])/(dt*dt);
  //   x_2d_ref[i] = x_2d_ref[i] + (x_d_ref[i]-x_d_ref_ubd[i])/(dt);
  // }
}

/** 
 * @brief Reference Model Definition for 3rd order system 
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 */
void rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm){
  float e_x      = k1_rm * (x_des- *x_ref);
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  float e_x_2d   = k3_rm * (e_x_d- *x_2d_ref);
  *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to the heading angle
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 */
void rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm){
  float temp_diff = x_des - *x_ref;
  NormRadAngle(temp_diff);
  float e_x      = k1_rm * temp_diff;
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  float e_x_2d   = k3_rm * (e_x_d- *x_2d_ref);
  *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to positioning with bounds
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 * @param x_d_bound       Bound for the 2nd order reference signal
 * @param x_2d_bound      Bound for the 3rd order reference signal
 * @param x_3d_bound      Bound for the 4th order reference signal
 * @param n               Number of dimensions
 */
void rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n){
  float e_x[n];
  float e_x_d[n];
  float e_x_2d[n];
  err_nd(e_x, x_des, x_ref, k1_rm, n);
  float max_x_d = bound_v_from_a(e_x, x_d_bound, x_2d_bound, n);
  vect_bound_nd(e_x, max_x_d, n);
  err_nd(e_x_d, e_x, x_d_ref, k2_rm, n);
  vect_bound_nd(e_x_d,x_2d_bound, n);
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref,e_x_2d,n);
  vect_bound_nd(x_3d_ref, x_3d_bound, n);
  integrate_nd(dt, x_2d_ref, x_3d_ref, n);
  integrate_nd(dt, x_d_ref, x_2d_ref, n);
  integrate_nd(dt, x_ref, x_d_ref, n);
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to positioning with bounds
 * @param dt              Delta time [s]
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_d_des         Desired 2nd order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 * @param x_2d_bound      Bound for the 3rd order reference signal
 * @param x_3d_bound      Bound for the 4th order reference signal
 * @param n               Number of dimensions
 */
void rm_2nd_pos(float dt, float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_d_des[], float k2_rm[], float k3_rm[], float x_2d_bound, float x_3d_bound, int n){
  float e_x_d[n];
  float e_x_2d[n];
  err_nd(e_x_d, x_d_des, x_d_ref, k2_rm, n);
  vect_bound_nd(e_x_d,x_2d_bound, n);
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref,e_x_2d,n);
  vect_bound_nd(x_3d_ref, x_3d_bound, n);
  integrate_nd(dt, x_2d_ref, x_3d_ref, n);
  integrate_nd(dt, x_d_ref, x_2d_ref, n);
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to positioning with bounds
 * @param dt              Delta time [s]
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_2d_des        Desired 3rd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 * @param x_3d_bound      Bound for the 4th order reference signal
 * @param n               Number of dimensions
 */
void rm_1st_pos(float dt, float x_2d_ref[], float x_3d_ref[], float x_2d_des[], float k3_rm[], float x_3d_bound, int n){
  float e_x_2d[n];
  err_nd(e_x_2d, x_2d_des, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref,e_x_2d,n);
  vect_bound_nd(x_3d_ref, x_3d_bound, n);
  integrate_nd(dt, x_2d_ref, x_3d_ref, n);
}


/** 
 * @brief Reference Model Definition for 2nd order system
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 */
void rm_2nd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm){
  float e_x      = k1_rm * (x_des- *x_ref);
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  *x_2d_ref = e_x_d;
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/** 
 * @brief Error Controller Definition for 3rd order system 
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param x               Current 1st order signal
 * @param x_d             Current 2nd order signal
 * @param x_2d            Current 3rd order signal
 * @param k1_e            Error Controller Gain 1st order signal
 * @param k2_e            Error Controller Gain 2nd order signal
 * @param k3_e            Error Controller Gain 3rd order signal
 */
static float ec_3rd(float x_ref, float x_d_ref, float x_2d_ref, float x_3d_ref, float x, float x_d, float x_2d, float k1_e, float k2_e, float k3_e){
  float y_4d = k1_e*(x_ref-x)+k2_e*(x_d_ref-x_d)+k3_e*(x_2d_ref-x_2d)+x_3d_ref;
  return y_4d;
}
void ec_3rd_pos( float y_4d[], float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x[], float x_d[], float x_2d[], float k1_e[], float k2_e[], float k3_e[], float x_d_bound, float x_2d_bound, float x_3d_bound, float fb[], int n){
  float e_x_d[n];
  float e_x_2d[n];

  err_sum_nd(e_x_d, x_ref, x, k1_e, x_d_ref, n);
  vect_bound_nd(e_x_d, x_d_bound*1.5, n);
  float temp_delete = x_d_ref[2]+k1_e[2]*(x_ref[2]-x[2]);
  //printf("A.P_e_ub: %f\n", temp_delete);
  //printf("A.P_e_bn: %f\n", e_x_d[2]);
  err_sum_nd(e_x_2d, e_x_d, x_d, k2_e, x_2d_ref, n);
  vect_bound_nd(e_x_2d,x_2d_bound*1.5, n);
  float temp_delete2 = x_2d_ref[2]+k2_e[2]*(e_x_d[2]-x_d[2]);
  //printf("A.V_e_ub: %f\n", temp_delete2);
  //printf("A.V_e_bn: %f\n", e_x_2d[2]);
  //printf("A.V boun: %f\n", x_2d_bound*1.5);
  //printf("A.V norm: %f\n", float_vect_norm(e_x_2d,n));
  // Calculate and bound distrubance --------------------
  float dist[3];
  float_vect_diff(dist, x_2d, fb, 3);
  //BoundAbs(dist[2], oneloop_andi_yaw_dist_limit); Bound to be decided
  err_sum_nd(y_4d, e_x_2d, dist, k3_e, x_3d_ref, n);
  //printf("CheeeeeeeekkkkkXXXXXXXXX \n");
  //printf("k3_e: %f,%f,%f\n", k3_e[0], k3_e[1], k3_e[2]);
  //vect_bound_nd(y_4d, x_3d_bound, n); This bound does not work anymore in this format
  //temp_checks_2[0] = e_x_2d[2];
  //temp_checks_2[1] = e_x_d[2];
}

/** 
 * @brief Error Controller Definition for 3rd order system specific to attitude
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_3d_ref        Reference signal 4th order
 * @param x_des           Desired 1st order signal
 * @param x               Current 1st order signal
 * @param x_d             Current 2nd order signal
 * @param x_2d            Current 3rd order signal
 * @param k1_e            Error Controller Gain 1st order signal
 * @param k2_e            Error Controller Gain 2nd order signal
 * @param k3_e            Error Controller Gain 3rd order signal
 */
void ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3], struct OneloopStabilizationRef bounds, float fb[3]){
  float e_x[3];    // (x-x_ref)*k1_e
  float e_x_rates[3]={0.}; // (x_ref-x)*k1_e
  float x_d_f[3];  // x_d_ref + e_x
  float x_2d_f[3]; // x_2d_ref + e_x_d

  // Attitude Error and Heading conversion --------------------------------
  err_nd(e_x, x_ref, x, k1_e, 3);
  float temp_diff = x_ref[2] - x[2];
  NormRadAngle(temp_diff);
  e_x[2] = k1_e[2] * temp_diff; // Correction for Heading error +-Pi
  //float_rates_of_euler_dot_vec(e_x_rates, x, e_x);
  //float_vect_sum(x_d_f, x_d_ref, e_x_rates, 3);
  BoundAbs(e_x[0], bounds.att_d[0]*1.5);
  BoundAbs(e_x[1], bounds.att_d[1]*1.5);
  BoundAbs(e_x[2], bounds.att_d[2]*1.5);
  float_vect_sum(x_d_f, x_d_ref, e_x, 3);
  //BoundAbs(x_d_f[0], bounds.att_d[0]*1.5);
  //BoundAbs(x_d_f[1], bounds.att_d[1]*1.5);
  //BoundAbs(x_d_f[2], bounds.att_d[2]*1.5);
  // Angular Rate Error ---------------------------------------------------
  x_2d_f[0] = (x_d_f[0]-x_d[0])*k2_e[0];
  x_2d_f[1] = (x_d_f[1]-x_d[1])*k2_e[1];
  x_2d_f[2] = (x_d_f[2]-x_d[2])*k2_e[2];
  BoundAbs(x_2d_f[0], bounds.att_2d[0]*1.5);
  BoundAbs(x_2d_f[1], bounds.att_2d[1]*1.5);
  BoundAbs(x_2d_f[2], bounds.att_2d[2]*1.5);
  x_2d_f[0] += x_2d_ref[0];
  x_2d_f[1] += x_2d_ref[1];
  x_2d_f[2] += x_2d_ref[2]; 
  //err_sum_nd(x_2d_f, x_d_f,  x_d,  k2_e, x_2d_ref, 3);
  //BoundAbs(x_2d_f[0], bounds.att_2d[0]*1.5);
  //BoundAbs(x_2d_f[1], bounds.att_2d[1]*1.5);
  //BoundAbs(x_2d_f[2], bounds.att_2d[2]*1.5);
  // Calculate and bound distrubance --------------------------------------
  float dist[3];
  float_vect_diff(dist, x_2d, fb, 3);
  temp_checks[0] = x_2d[2];
  temp_checks[1] = fb[2];
  //BoundAbs(dist[2], oneloop_andi_yaw_dist_limit); //FIXME UNCOMMENT ME
  // Angular Acceleration Error -------------------------------------------
  err_sum_nd(y_4d, x_2d_f, dist, k3_e, x_3d_ref, 3);
  temp_e_x = e_x[2];
  temp_e_x_rates = e_x_rates[2];
  temp_x_d_f = x_d_f[2];
  temp_x_2d_f = x_2d_f[2];
  temp_dist_r = dist[2];
  temp_ec_r = (x_2d_f[2]-x_2d[2])*k3_e[2]+x_3d_ref[2];
  temp_ec_r_2 = temp_diff*k1_e[2]*k2_e[2]*k3_e[2]+(x_d_ref[2]-x_d[2])*k2_e[2]*k3_e[2]+(x_2d_ref[2]-x_2d[2])*k3_e[2]+x_3d_ref[2];
  temp_checks_2[0] = x_d_f[2];
  temp_checks_2[1] = x_2d_f[2];
}

/**
 * @brief  Third Order to First Order Dynamics Approximation
 * @param p1              Pole 1
 * @param p2              Pole 2
 * @param p3              Pole 3
 * @param rm_k            Reference Model Gain
 */
static float w_approx(float p1, float p2, float p3, float rm_k){
  p1   = positive_non_zero(p1);
  p2   = positive_non_zero(p2);
  p3   = positive_non_zero(p3);
  rm_k = positive_non_zero(rm_k);
  float tao = (p1*p2+p1*p3+p2*p3)/(p1*p2*p3)/(rm_k);
  tao  = positive_non_zero(tao);
  return 1.0/tao;
}

/**
 * @brief Calculate EC poles given RM poles
 * @param p_rm      Reference Model Pole (3 coincident poles)
 * @param slow_pole Pole of the slowest dynamics
 * @param k         EC / RM ratio
 * @param omega_n   Natural Frequency
 */
static float ec_poles(float p_rm, float slow_pole, float k){
  p_rm      = positive_non_zero(p_rm);
  slow_pole = positive_non_zero(slow_pole);
  k         = positive_non_zero(k);
  //float omega_n = (2*p_rm*slow_pole*k)/(3*slow_pole-p_rm);
  float omega_n = (2*k*p_rm*slow_pole)/(3*slow_pole-k*p_rm);
  return omega_n;
}

/**
 * @brief Initialize Position of Poles
 * 
 */
void init_poles_att(void){
  float slow_pole  = 22.0; // Pole of the slowest dynamics used in the attitude controller
  p_att_e.omega_n  = ec_poles(p_att_rm.omega_n,  slow_pole, 1.28); // k = 1.28;
  p_head_e.omega_n = ec_poles(p_head_rm.omega_n, slow_pole, 1.28); // k = 1.28;
}
void init_poles_pos(void){
  act_dynamics[COMMAND_ROLL]  = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  act_dynamics[COMMAND_PITCH] = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  float slow_pole = act_dynamics[COMMAND_ROLL]; // Pole of the slowest dynamics used in the position controller
  p_pos_e.omega_n = ec_poles(p_pos_rm.omega_n,slow_pole,1.28);// k = 1.28; 1.0;
  p_alt_e.omega_n = ec_poles(p_alt_rm.omega_n,slow_pole,1.28);// k = 1.28; 1.0;// 3.0
}

/**
 * @brief Initialize Position of Poles
 * 
 */
void init_poles(void){

  // Attitude Controller Poles----------------------------------------------------------
  float slow_pole = 22.0; // Pole of the slowest dynamics used in the attitude controller

  p_att_e.omega_n = slow_pole/3.0;
  p_att_e.zeta    = 1.0;
  p_att_e.p3      = p_att_e.omega_n;

  p_att_rm.omega_n = p_att_e.omega_n*0.8; 
  p_att_rm.zeta    = 1.0;
  p_att_rm.p3      = p_att_rm.omega_n;

  p_head_e.omega_n = slow_pole/3.0;
  p_head_e.zeta    = 1.0;
  p_head_e.p3      = p_head_e.omega_n;

  p_head_rm.omega_n = p_head_e.omega_n*0.8; 
  p_head_rm.zeta    = 1.0;
  p_head_rm.p3      = p_head_rm.omega_n;

  act_dynamics[COMMAND_ROLL]  = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  act_dynamics[COMMAND_PITCH] = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);

  // Position Controller Poles----------------------------------------------------------
  slow_pole = act_dynamics[COMMAND_ROLL]; // Pole of the slowest dynamics used in the position controller

  p_pos_e.omega_n = 1.19;//slow_pole/3.0;
  p_pos_e.zeta    = 0.5; 
  p_pos_e.p3      = p_pos_e.omega_n; 

  p_pos_rm.omega_n = p_pos_e.omega_n*0.8; 
  p_pos_rm.zeta    = 0.5;  
  p_pos_rm.p3      = p_pos_rm.omega_n;

  p_alt_e.omega_n = 1.19;//slow_pole/3.0*2.0;
  p_alt_e.zeta    = 0.5; 
  p_alt_e.p3      = p_alt_e.omega_n;

  p_alt_rm.omega_n = p_alt_e.omega_n*0.8;
  p_alt_rm.zeta    = 0.5;
  p_alt_rm.p3      = p_alt_rm.omega_n; 
}

/** 
 * @brief Initialize Controller Gains
 * FIXME: Calculate the gains dynamically for transition
 */
void init_controller_gains(void){
  /*Register a variable from nav_hybrid. Should be improved when nav hybrid is final.*/
  float max_wind  = 20.0;
  max_v_nav = nav_max_speed + max_wind;
  max_a_nav = nav_max_acceleration_sp;
  /*Some calculations in case new poles have been specified*/
  //init_poles_att();
  //init_poles_pos();
  p_att_rm.p3  = p_att_rm.omega_n  * p_att_rm.zeta;
  p_pos_rm.p3  = p_pos_rm.omega_n  * p_pos_rm.zeta;
  p_alt_rm.p3  = p_alt_rm.omega_n  * p_alt_rm.zeta;
  p_head_rm.p3 = p_head_rm.omega_n * p_head_rm.zeta;

  //--ANDI Controller gains --------------------------------------------------------------------------------
  /*Attitude Loop*/
  k_att_e.k1[0]  = k_rm_1_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  k_att_e.k2[0]  = k_rm_2_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  k_att_e.k3[0]  = k_rm_3_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);

  k_att_e.k1[1]  = k_att_e.k1[0]; 
  k_att_e.k2[1]  = k_att_e.k2[0]; 
  k_att_e.k3[1]  = k_att_e.k3[0]; 

  k_att_rm.k1[0] = k_rm_1_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k2[0] = k_rm_2_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k3[0] = k_rm_3_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k1[1] = k_att_rm.k1[0];
  k_att_rm.k2[1] = k_att_rm.k2[0];
  k_att_rm.k3[1] = k_att_rm.k3[0];
  
  /*Heading Loop NAV*/
  k_att_e.k1[2]  = k_rm_1_3_f(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  k_att_e.k2[2]  = k_rm_2_3_f(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  k_att_e.k3[2]  = k_rm_3_3_f(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);

  k_att_rm.k1[2] = k_rm_1_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  k_att_rm.k2[2] = k_rm_2_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  k_att_rm.k3[2] = k_rm_3_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);

  // Print INNERLOOP ANDI controller gains
  //printf("Attitude RM poles, omega_n: %f, zeta: %f, p3: %f\n", p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  //printf("Attitude EC poles, omega_n: %f, zeta: %f, p3: %f\n", p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  //printf("Heading  RM poles, omega_n: %f, zeta: %f, p3: %f\n", p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  //printf("Heading  EC poles, omega_n: %f, zeta: %f, p3: %f\n", p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  //printf("ANDI Attitude RM Gains: %f %f %f\n", k_att_rm.k1[0], k_att_rm.k2[0], k_att_rm.k3[0]);
  //printf("ANDI Attitude EC Gains: %f %f %f\n", k_att_e.k1[0], k_att_e.k2[0], k_att_e.k3[0]);
  //printf("ANDI Heading  RM Gains: %f %f %f\n", k_att_rm.k1[2], k_att_rm.k2[2], k_att_rm.k3[2]);
  //printf("ANDI Heading  EC Gains: %f %f %f\n", k_att_e.k1[2], k_att_e.k2[2], k_att_e.k3[2]);

  /*Position Loop*/
  k_pos_e.k1[0]  = k_rm_1_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3); //0.595;//
  k_pos_e.k2[0]  = k_rm_2_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3); //1.190;//
  k_pos_e.k3[0]  = k_rm_3_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3); //2.380;//
  k_pos_e.k1[1]  = k_pos_e.k1[0];  
  k_pos_e.k2[1]  = k_pos_e.k2[0];  
  k_pos_e.k3[1]  = k_pos_e.k3[0]; 

  k_pos_rm.k1[0] = k_rm_1_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3);//0.595;
  k_pos_rm.k2[0] = k_rm_2_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3);//1.190;
  k_pos_rm.k3[0] = k_rm_3_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3);//2.380;
  k_pos_rm.k1[1] = k_pos_rm.k1[0];
  k_pos_rm.k2[1] = k_pos_rm.k2[0];  
  k_pos_rm.k3[1] = k_pos_rm.k3[0];
  nav_hybrid_pos_gain   = k_pos_rm.k1[0];
  nav_hybrid_max_bank   = ONELOOP_ANDI_MAX_BANK;

  /*Altitude Loop*/
  k_pos_e.k1[2]  = k_rm_1_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3); //0.595;
  k_pos_e.k2[2]  = k_rm_2_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3); //1.190;
  k_pos_e.k3[2]  = 22;//temp_k;//k_rm_3_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3); //2.380;

  k_pos_rm.k1[2] = k_rm_1_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3); //0.595;
  k_pos_rm.k2[2] = k_rm_2_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3); //1.190;
  k_pos_rm.k3[2] = k_rm_3_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3); //2.380;

  // Print OUTERLOOP ANDI controller gains
  //printf("Position NE RM poles, omega_n: %f, zeta: %f, p3: %f\n", p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3);
  //printf("Position NE EC poles, omega_n: %f, zeta: %f, p3: %f\n", p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3);
  //printf("Position D  RM poles, omega_n: %f, zeta: %f, p3: %f\n", p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3);
  //printf("Position D  EC poles, omega_n: %f, zeta: %f, p3: %f\n", p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3);
  //printf("Position N  RM Gains: %f %f %f\n", k_pos_rm.k1[0], k_pos_rm.k2[0], k_pos_rm.k3[0]);
  //printf("Position N  EC Gains: %f %f %f\n", k_pos_e.k1[0], k_pos_e.k2[0], k_pos_e.k3[0]);
  //printf("Position D  RM Gains: %f %f %f\n", k_pos_rm.k1[2], k_pos_rm.k2[2], k_pos_rm.k3[2]);
  //printf("Position D  EC Gains: %f %f %f\n", k_pos_e.k1[2], k_pos_e.k2[2], k_pos_e.k3[2]);
  //--INDI Controller gains --------------------------------------------------------------------------------
    /*Attitude Loop*/
  k_att_e_indi.k1[0]  = k_att_e.k1[0];//k_1_2_f(p_att_e.omega_n, p_att_e.zeta);
  k_att_e_indi.k2[0]  = k_att_e.k2[0];//k_2_2_f(p_att_e.omega_n, p_att_e.zeta);
  k_att_e_indi.k3[0]  = 1.0;
  k_att_e_indi.k1[1]  = k_att_e_indi.k1[0]; 
  k_att_e_indi.k2[1]  = k_att_e_indi.k2[0]; 
  k_att_e_indi.k3[1]  = k_att_e_indi.k3[0]; 

  /*Heading Loop NAV*/
  k_att_e_indi.k1[2]  = k_att_e.k1[2];//k_1_2_f(p_head_e.omega_n, p_head_e.zeta);
  k_att_e_indi.k2[2]  = k_att_e.k2[2];//k_2_2_f(p_head_e.omega_n, p_head_e.zeta);
  k_att_e_indi.k3[2]  = 1.0;
  
  // Print INDI INNERLOOP controller gains
  //printf("INDI Attitude EC gains: %f %f %f\n", k_att_e_indi.k1[0], k_att_e_indi.k2[0], k_att_e_indi.k3[0]);
  //printf("INDI Heading  EC gains: %f %f %f\n", k_att_e_indi.k1[2], k_att_e_indi.k2[2], k_att_e_indi.k3[2]);

  /*Position Loop*/
  k_pos_e_indi.k1[0]  = k_pos_e.k1[0];//k_1_2_f(p_pos_e.omega_n, p_pos_e.zeta);
  k_pos_e_indi.k2[0]  = k_pos_e.k2[0];//k_2_2_f(p_pos_e.omega_n, p_pos_e.zeta);
  k_pos_e_indi.k3[0]  = 1.0;
  k_pos_e_indi.k1[1]  = k_pos_e_indi.k1[0];  
  k_pos_e_indi.k2[1]  = k_pos_e_indi.k2[0];  
  k_pos_e_indi.k3[1]  = k_pos_e_indi.k3[0]; 

  /*Altitude Loop*/
  k_pos_e_indi.k1[2]  = k_pos_e.k1[2];//k_1_2_f(p_alt_e.omega_n, p_alt_e.zeta);
  k_pos_e_indi.k2[2]  = k_pos_e.k2[2];//k_2_2_f(p_alt_e.omega_n, p_alt_e.zeta);
  k_pos_e_indi.k3[2]  = 1.0;
  
  // Print INDI OUTERLOOP controller gains
  //printf("INDI Position NE EC gains: %f %f %f\n", k_pos_e_indi.k1[0], k_pos_e_indi.k2[0], k_pos_e_indi.k3[0]);
  //printf("INDI Position D  EC gains: %f %f %f\n", k_pos_e_indi.k1[2], k_pos_e_indi.k2[2], k_pos_e_indi.k3[2]);

  //------------------------------------------------------------------------------------------
  /*Approximated Dynamics*/
  act_dynamics[COMMAND_ROLL]   = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  act_dynamics[COMMAND_PITCH]  = w_approx(p_att_rm.p3, p_att_rm.p3, p_att_rm.p3, 1.0);
  //printf("Act Dynamics: %f %f\n", act_dynamics[COMMAND_ROLL], act_dynamics[COMMAND_PITCH]);
}
// -----------------------------------------------------------------------------------------
// Filter Functions ------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

/** @brief Initialize a filter based on its type */
static inline void init_filter_on_type(struct LP_t *filter, float x0) {
  switch(filter->filter_type) {
    case LOWPASS_1:
      init_first_order_low_pass(&filter->meas_filt.lp1, filter->tau, 1.0 / PERIODIC_FREQUENCY, x0);
      break;
    case BUTTERWORTH_2:
      init_butterworth_2_low_pass(&filter->meas_filt.bw2, filter->tau, 1.0 / PERIODIC_FREQUENCY, x0);
      break;
    case BUTTERWORTH_4:
      init_butterworth_4_low_pass(&filter->meas_filt.bw4, filter->tau, 1.0 / PERIODIC_FREQUENCY, x0);
      break;
    case NOTCH:
      notch_filter_init(&filter->meas_filt.notch, filter->freq, filter->bandwidth, PERIODIC_FREQUENCY);
      break;
    default:
      // Handle unexpected filter type
      break;
  }
}
/** @brief Update a filter based on its type */
static inline void update_filter_on_type(struct LP_t *filter, float input) {
  //filter->meas_prev = filter->meas;
  //filter->meas = input;
  switch(filter->filter_type) {
    case LOWPASS_1:
      update_first_order_low_pass(&filter->meas_filt.lp1, input);
      filter->out = filter->meas_filt.lp1.last_out;
      break;
    case BUTTERWORTH_2:
      update_butterworth_2_low_pass(&filter->meas_filt.bw2, input);
      filter->out = filter->meas_filt.bw2.o[0];
      break;
    case BUTTERWORTH_4:
      update_butterworth_4_low_pass(&filter->meas_filt.bw4, input);
      filter->out = filter->meas_filt.bw4.lp2.o[0];
      break;
    case NOTCH: {
      notch_filter_update(&filter->meas_filt.notch, &input, &filter->out);
      break;
    }
    default:
      filter->out = 0.0;
      break;
      // Handle unexpected filter type
  }
}
/** @brief Initialize the Low Pass Filter Struct */
static inline void init_LP(struct LP_t *LP, float fc){
  LP->freq     = fc;
  LP->freq_set = fc;
  LP->tau      = 1/(2*M_PI*LP->freq);
#ifdef USE_LP1
  LP->filter_type = LOWPASS_1;
#elif defined(USE_BW2)
  LP->filter_type = BUTTERWORTH_2;
#elif defined(USE_YAW_LP4)
  LP->filter_type = BUTTERWORTH_4;
#endif
  init_filter_on_type(LP, 0.0);
  //init_first_order_low_pass(&LP->meas_filt, LP->tau, 1.0 / PERIODIC_FREQUENCY, 0.0);
  LP->meas      = 0.0;
  LP->meas_prev = 0.0;
  LP->out       = 0.0;
}
/** @brief Reinitialize Low Pass filter if new frequency setting or if forced */
static inline void reinit_LP_synchronous(struct LP_t *LP, struct LP_t *f, bool reinit){
  if(LP->freq != LP->freq_set || reinit){
    LP->freq = LP->freq_set;
    LP->tau = 1/(2*M_PI*LP->freq);
    init_filter_on_type(LP, LP->out);
    init_filter_on_type(f,  f->out);
  }
}
static inline void reinit_LP(struct LP_t *LP, bool reinit){
  if(LP->freq != LP->freq_set || reinit){
    LP->freq = LP->freq_set;
    LP->tau = 1/(2*M_PI*LP->freq);
    init_filter_on_type(LP, LP->out);
  }
}

/** @brief  Initialize all Low Pass Filters */
static inline void init_all_LP(void){
  init_LP(&LP.ax,    2.0); //oneloop_andi_filt_cutoff_a 
  init_LP(&LP.ay,    2.0); //oneloop_andi_filt_cutoff_a 
  init_LP(&LP.az,    2.0); //oneloop_andi_filt_cutoff_a 
  init_LP(&LP.p_dot, 2.0);
  init_LP(&LP.q_dot, 2.0);
  init_LP(&LP.r_dot, 2.0);
  init_LP(&LP.p,    15.0); //oneloop_andi_filt_cutoff_p
  init_LP(&LP.q,    15.0); //oneloop_andi_filt_cutoff_q
  init_LP(&LP.r,    15.0); //oneloop_andi_filt_cutoff_r
  
  init_LP(&oneloop_andi_model_filt.ax,    2.0); //oneloop_andi_filt_cutoff_a
  init_LP(&oneloop_andi_model_filt.ay,    2.0); //oneloop_andi_filt_cutoff_a
  init_LP(&oneloop_andi_model_filt.az,    2.0); //oneloop_andi_filt_cutoff_a
  init_LP(&oneloop_andi_model_filt.p_dot, 2.0);
  init_LP(&oneloop_andi_model_filt.q_dot, 2.0);
  init_LP(&oneloop_andi_model_filt.r_dot, 2.0);
}

/** @brief Reinitialize all the Low Pass Filters */
static inline void reinit_all_LP(bool reinit){
  reinit_LP_synchronous(&LP.ax,    &oneloop_andi_model_filt.ax   , reinit);
  reinit_LP_synchronous(&LP.ay,    &oneloop_andi_model_filt.ay   , reinit);
  reinit_LP_synchronous(&LP.az,    &oneloop_andi_model_filt.az   , reinit);
  reinit_LP_synchronous(&LP.p_dot, &oneloop_andi_model_filt.p_dot, reinit);
  reinit_LP_synchronous(&LP.q_dot, &oneloop_andi_model_filt.q_dot, reinit);
  reinit_LP_synchronous(&LP.r_dot, &oneloop_andi_model_filt.r_dot, reinit);
  reinit_LP(&LP.p, reinit);
  reinit_LP(&LP.q, reinit);
  reinit_LP(&LP.r, reinit);
}
//------------------------------------------------------------------------------------------


/** @brief  Initialize the filters */
void init_filter(void)
{
#ifdef ONELOOP_ANDI_ROLL_STRUCTURAL_MODE_FREQ
  init_filter_on_type(&roll_structural_mode);
#endif
#ifdef ONELOOP_ANDI_PITCH_STRUCTURAL_MODE_FREQ
  init_filter_on_type(&pitch_structural_mode);
#endif
#ifdef ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ
  init_filter_on_type(&yaw_structural_mode);
#endif
  // Filtering of the velocities 
  float tau   = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff);
  float tau_v = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_v);
  float tau_2 = 1.0 / (2.0 * M_PI * 2.0);
  //printf("tau: %f tau_v: %f\n", tau, tau_v);
  //printf("initializing filters\n");
  init_butterworth_2_low_pass(&filt_veloc_N,      tau_v, 1.0 / PERIODIC_FREQUENCY, filt_veloc_N.o[0]);
  init_butterworth_2_low_pass(&filt_veloc_E,      tau_v, 1.0 / PERIODIC_FREQUENCY, filt_veloc_E.o[0]);
  init_butterworth_2_low_pass(&filt_veloc_D,      tau_v, 1.0 / PERIODIC_FREQUENCY, filt_veloc_D.o[0]);
  init_butterworth_2_low_pass(&accely_filt,       tau,   1.0 / PERIODIC_FREQUENCY, accely_filt.o[0]);
  init_butterworth_2_low_pass(&airspeed_filt,     tau,   1.0 / PERIODIC_FREQUENCY, airspeed_filt.o[0]);
  for (int i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    init_butterworth_2_low_pass(&u_filt[i], tau_2, 1.0 / PERIODIC_FREQUENCY, 0.0);
  }
}


/** @brief  Propagate the filters */
void oneloop_andi_propagate_filters(void) {
  reinit_all_LP(false);
  struct  NedCoor_f *accel = stateGetAccelNed_f();
  struct  NedCoor_f *veloc = stateGetSpeedNed_f();
  //printf("veloc: %f %f %f\n", veloc->x, veloc->y, veloc->z);
  struct  FloatRates *body_rates = stateGetBodyRates_f();
  // Store Feedbacks in the Complementary Filters
  LP.ax.meas        = accel->x;
  LP.ay.meas        = accel->y;
  LP.az.meas        = accel->z;
  LP.p.meas_prev    = LP.p.meas;
  LP.q.meas_prev    = LP.q.meas;
  LP.r.meas_prev    = LP.r.meas;
  LP.p.meas         = body_rates->p;
  LP.q.meas         = body_rates->q;
  LP.r.meas         = body_rates->r;
  float temp_p_dot  = (LP.p.meas-LP.p.meas_prev)*PERIODIC_FREQUENCY;
  float temp_q_dot  = (LP.q.meas-LP.q.meas_prev)*PERIODIC_FREQUENCY;
  float temp_r_dot  = (LP.r.meas-LP.r.meas_prev)*PERIODIC_FREQUENCY;
#ifdef ONELOOP_ANDI_ROLL_STRUCTURAL_MODE_FREQ
    LP.p_dot.meas = update_filter_on_type_feedback(&roll_structural_mode, temp_p_dot);
#else
  LP.p_dot.meas = temp_p_dot;
#endif
#ifdef ONELOOP_ANDI_PITCH_STRUCTURAL_MODE_FREQ
    LP.q_dot.meas = update_filter_on_type_feedback(&pitch_structural_mode, temp_q_dot);
#else
  LP.q_dot.meas = temp_q_dot;
#endif
#ifdef ONELOOP_ANDI_YAW_STRUCTURAL_MODE_FREQ
    LP.r_dot.meas = update_filter_on_type_feedback(&yaw_structural_mode, temp_r_dot);
#else
  LP.r_dot.meas = temp_r_dot;
#endif

  // Update Filters of Feedbacks
  update_filter_on_type(&LP.ax,    LP.ax.meas);
  update_filter_on_type(&LP.ay,    LP.ay.meas);
  update_filter_on_type(&LP.az,    LP.az.meas);
  update_filter_on_type(&LP.p_dot, LP.p_dot.meas);
  update_filter_on_type(&LP.q_dot, LP.q_dot.meas);
  update_filter_on_type(&LP.r_dot, LP.r_dot.meas);
  update_filter_on_type(&LP.p,     LP.p.meas);
  update_filter_on_type(&LP.q,     LP.q.meas);
  update_filter_on_type(&LP.r,     LP.r.meas); 

  update_butterworth_2_low_pass(&filt_veloc_N,      veloc->x);
  update_butterworth_2_low_pass(&filt_veloc_E,      veloc->y);
  update_butterworth_2_low_pass(&filt_veloc_D,      veloc->z); 
 
  // Calculate Model Predictions for Linear and Angular Accelerations Using the Effectiveness Matrix
  // calc_model();

  // Propagate filter for sideslip correction
  float accely = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  update_butterworth_2_low_pass(&accely_filt, accely);
  float airspeed_meas = stateGetAirspeed_f();
  Bound(airspeed_meas, 0.0, 30.0);
  update_butterworth_2_low_pass(&airspeed_filt, airspeed_meas);
}
//------------------------------------------------------------------------------------------
/** @brief Re-Init function of controller variables */
void reinit_controller(void)
{ 
  // store float version of commands
  // float commands_float[ANDI_NUM_ACT_TOT];
  // for (int i = 0; i < ANDI_NUM_ACT; i++) {
  //   commands_float[i] = (float)commands[i];
  // }
  // Actuators
  //float_vect_copy(andi_u, commands_float, ANDI_NUM_ACT);
  andi_u[COMMAND_ROLL]  = oneloop_andi.sta_state.att[0];
  andi_u[COMMAND_PITCH] = oneloop_andi.sta_state.att[1];
  
  //float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);  // Not used
  //float_vect_zero(andi_u_n, ANDI_NUM_ACT_TOT); // Not used
  //float_vect_copy(actuator_state_1l, commands_float, ANDI_NUM_ACT);
  // Stabilization
  float_vect_copy(oneloop_andi.sta_ref.att,    oneloop_andi.sta_state.att, 3);
  float_vect_copy(oneloop_andi.sta_ref.att_d,  oneloop_andi.sta_state.att_d, 3);
  float_vect_copy(oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_state.att_2d, 3);
  float_vect_zero(oneloop_andi.sta_ref.att_3d, 3);
  eulers_zxy_des.phi   =  oneloop_andi.sta_state.att[0];
  eulers_zxy_des.theta =  oneloop_andi.sta_state.att[1];
  eulers_zxy_des.psi   =  oneloop_andi.sta_state.att[2];
  // Guidance
  float_vect_copy(oneloop_andi.gui_ref.pos, oneloop_andi.gui_state.pos, 3);
  float_vect_copy(oneloop_andi.gui_ref.vel, oneloop_andi.gui_state.vel, 3);
  float_vect_copy(oneloop_andi.gui_ref.acc, oneloop_andi.gui_state.acc, 3);
  float_vect_zero(oneloop_andi.gui_ref.jer, 3);
  // Controller Inputs
  //float_vect_zero(nu, ANDI_OUTPUTS); // Not used
  //float_vect_zero(nu_n, ANDI_OUTPUTS); // Not used
  // float_vect_zero(nav_target,3); // Not used
  // float_vect_zero(nav_target_new,3); // Not used
}
/** @brief Init function of Oneloop ANDI controller  */
void oneloop_andi_init(void)
{ 
  //printf("INIT \n");
  oneloop_andi.half_loop = true;
  oneloop_andi.ctrl_type = CTRL_ANDI;
  init_poles();
  // Make sure that the dynamics are positive and non-zero
  int8_t i;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    act_dynamics[i] = positive_non_zero(act_dynamics[i]);
  }
  dynFilter_init(&mu_mR, act_dynamics[COMMAND_MOTOR_RIGHT], act_dynamics[COMMAND_MOTOR_RIGHT]);
  dynFilter_init(&mu_mL, act_dynamics[COMMAND_MOTOR_LEFT], act_dynamics[COMMAND_MOTOR_LEFT]);
  // Initialize Effectiveness matrix
  calc_normalization();
  G1G2_oneloop(oneloop_andi.ctrl_type);
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = EFF_MAT_G[i];
  }
  // Initialize filters and other variables
  init_all_LP();
  init_filter();
  init_controller_gains();
  float_vect_zero(andi_u, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_u_n, ANDI_NUM_ACT_TOT);
  float_vect_zero(actuator_state_1l,ANDI_NUM_ACT_TOT);
  float_vect_zero(oneloop_andi.sta_ref.att,3);
  float_vect_zero(oneloop_andi.sta_ref.att_d,3);
  float_vect_zero(oneloop_andi.sta_ref.att_2d,3);
  float_vect_zero(oneloop_andi.sta_ref.att_3d,3);
  float_vect_zero(nu, ANDI_OUTPUTS);
  float_vect_zero(nu_n, ANDI_OUTPUTS);
  float_vect_zero(nav_target,3);
  float_vect_zero(nav_target_new,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;
  // Start telemetry
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_STAB, send_eff_mat_stab_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_GUID, send_eff_mat_guid_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE, send_guidance_oneloop_andi);
    // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATOR_STATE, send_oneloop_actuator_state);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_oneloop_debug);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_oneloop);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_oneloop);
  #endif
}

/**
 * @brief Function that resets important values upon engaging Oneloop ANDI.
 * FIXME: Ideally we should distinguish between the "stabilization" and "guidance" needs because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 */
void oneloop_andi_enter(bool half_loop_sp, int ctrl_type)
{
  //printf("ENTER %d \n", counter_andi);
  //counter_andi++;
  ele_min = 0.0;
  oneloop_andi.half_loop      = half_loop_sp;
  oneloop_andi.ctrl_type      = ctrl_type;
  psi_des_rad                 = eulers_zxy.psi; 
  psi_des_deg                 = DegOfRad(eulers_zxy.psi);
  // if (oneloop_andi.half_loop){
  //   printf("HALF LOOP\n");
  // } else {
  //   printf("FULL LOOP\n");
  // }
  // if (oneloop_andi.ctrl_type == CTRL_ANDI){
  //   printf("ANDI\n");
  // } else if (oneloop_andi.ctrl_type == CTRL_INDI){
  //   printf("INDI\n");
  // } else {
  //   printf("CTRL_TYPE NOT SET\n");
  // }
  calc_normalization();
  G1G2_oneloop(oneloop_andi.ctrl_type);
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = EFF_MAT_G[i];
  }
  //reinit_all_LP(true); // Not used
  //init_filter();
  init_controller_gains();
  /* Stabilization Reset */
  /*Guidance Reset*/
  reinit_controller();
}

/**
 * @brief  Function to generate the reference signals for the oneloop controller
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * @param PSA_des    Desired position/speed/acceleration
 * @param rm_order_h Order of the reference model for horizontal guidance
 * @param rm_order_v Order of the reference model for vertical guidance
 */
void oneloop_andi_RM(bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v, bool in_flight_oneloop)
{
  // Initialize some variables
  a_thrust = 0.0;
  nav_target[0] = PSA_des.x;
  nav_target[1] = PSA_des.y;
  nav_target[2] = PSA_des.z;
  float thrust_cmd_1l = 0.0;
  float des_r = 0.0;
  // Generate reference signals with reference model 
  if(half_loop){
    // Disregard X and Y jerk objectives
    WLS_one_p.Wv[0] = 0.0;
    WLS_one_p.Wv[1] = 0.0;
    if(drop_yaw){
      WLS_one_p.Wv[5] = 0.0;
    } else {
      WLS_one_p.Wv[5] = Wv_backup[5];
    }
    // Overwrite references with actual signals (for consistent plotting)
    float_vect_copy(oneloop_andi.gui_ref.pos,oneloop_andi.gui_state.pos,3);
    float_vect_copy(oneloop_andi.gui_ref.vel,oneloop_andi.gui_state.vel,3);
    float_vect_copy(oneloop_andi.gui_ref.acc,oneloop_andi.gui_state.acc,3);
    float_vect_zero(oneloop_andi.gui_ref.jer,3);
    // Set desired attitude with stick input
    eulers_zxy_des.phi   = (float) (radio_control_get(RADIO_ROLL)) /MAX_PPRZ * ONELOOP_ANDI_MAX_PHI  ;
    eulers_zxy_des.theta = (float) (radio_control_get(RADIO_PITCH))/MAX_PPRZ * ONELOOP_ANDI_MAX_THETA;
    // Set desired Yaw rate with stick input
    des_r = (float) (radio_control_get(RADIO_YAW))/MAX_PPRZ*max_r;            // Get yaw rate from stick
    BoundAbs(des_r,max_r);                                                    // Bound yaw rate
    float delta_psi_des_rad = des_r * dt_1l;                                  // Integrate desired Yaw rate to get desired change in yaw
    float delta_psi_rad = eulers_zxy_des.psi-eulers_zxy.psi;                  // Calculate current yaw difference between des and actual
    NormRadAngle(delta_psi_rad);                                              // Normalize the difference
    if (fabs(delta_psi_rad) > RadOfDeg(30.0)){                                // If difference is bigger than 10 deg do not further increment desired
      delta_psi_des_rad = 0.0;
    }
    psi_des_rad += delta_psi_des_rad;                                         // Incrementdesired yaw
    NormRadAngle(psi_des_rad);
    // Register Attitude Setpoints from previous loop
    if (!in_flight_oneloop){
      psi_des_rad   = eulers_zxy.psi;
    }
    eulers_zxy_des.psi = psi_des_rad;
    float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
    // Create commands adhoc to get actuators to the wanted level
    thrust_cmd_1l = (float) radio_control_get(RADIO_THROTTLE);
    Bound(thrust_cmd_1l,0.0,MAX_PPRZ); 
    int8_t i;
    // To calculate the nu corrsponding to the Thrust command, plug it in the control law.
    for (i = 0; i < ANDI_NUM_ACT; i++) {
      if(oneloop_andi.ctrl_type == CTRL_ANDI){
        a_thrust +=(thrust_cmd_1l) * EFF_MAT_RW[RW_aD][i] * act_dyn_ctrl[i];
      }else{
        a_thrust +=(thrust_cmd_1l) * EFF_MAT_RW[RW_aD][i];
      }
    }
    a_thrust = a_thrust - oneloop_andi_model[RW_aD];  //oneloop_andi_model_filt.az.out; 
    rm_3rd_attitude(dt_1l, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, att_des, false, psi_vec, k_att_rm.k1, k_att_rm.k2, k_att_rm.k3, sta_bounds);
  }else{
    // Make sure X and Y jerk objectives are active
    WLS_one_p.Wv[0] = Wv_backup[0];
    WLS_one_p.Wv[1] = Wv_backup[1];
    // Generate Reference signals for positioning using RM
    if (rm_order_h == 3){
      rm_3rd_pos(dt_1l, oneloop_andi.gui_ref.pos, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target, k_pos_rm.k1, k_pos_rm.k2, k_pos_rm.k3, max_v_nav, max_a_nav, max_j_lin, 2);    
    } else if (rm_order_h == 2){
      float_vect_copy(oneloop_andi.gui_ref.pos, oneloop_andi.gui_state.pos,2);
      float_vect_copy(oneloop_andi.gui_ref.vel, oneloop_andi.gui_state.vel,2);
      reshape_wind();//returns accel sp as nav target new
      rm_1st_pos(dt_1l, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target_new, k_pos_rm.k3, max_j_lin, 2); 
      // rm_2nd_pos(dt_1l, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target_new, k_pos_rm.k2, k_pos_rm.k3, max_a_nav, max_j_lin, 2);   
    } else if (rm_order_h == 1){
      float_vect_copy(oneloop_andi.gui_ref.pos, oneloop_andi.gui_state.pos,2);
      float_vect_copy(oneloop_andi.gui_ref.vel, oneloop_andi.gui_state.vel,2);
      rm_1st_pos(dt_1l, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, nav_target, k_pos_rm.k3, max_j_lin, 2);   
    }
    // Update desired Heading (psi_des_rad) based on previous loop or changed setting
    if (heading_manual){
      psi_des_rad = RadOfDeg(psi_des_deg);
      if (yaw_stick_in_auto){
        psi_des_rad += (float) (radio_control_get(RADIO_YAW))/MAX_PPRZ*max_r * dt_1l;
      }
    } else {
      psi_des_rad += oneloop_andi_sideslip() * dt_1l;
      NormRadAngle(psi_des_rad);
    }
    // Register Attitude Setpoints from previous loop
    if (!in_flight_oneloop){
      psi_des_rad   = eulers_zxy.psi;
    }
    eulers_zxy_des.psi = psi_des_rad;
    float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
    // The RM functions want an array as input. Create a single entry array and write the vertical guidance entries. 
    float single_value_ref[1]        = {oneloop_andi.gui_ref.pos[2]};
    float single_value_d_ref[1]      = {oneloop_andi.gui_ref.vel[2]};
    float single_value_2d_ref[1]     = {oneloop_andi.gui_ref.acc[2]};
    float single_value_3d_ref[1]     = {oneloop_andi.gui_ref.jer[2]};
    float single_value_nav_target[1] = {nav_target[2]};
    float single_value_k1_rm[1]      = {k_pos_rm.k1[2]};
    float single_value_k2_rm[1]      = {k_pos_rm.k2[2]};
    float single_value_k3_rm[1]      = {k_pos_rm.k3[2]};
    if (rm_order_v == 3){
      rm_3rd_pos(dt_1l, single_value_ref, single_value_d_ref, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k1_rm, single_value_k2_rm, single_value_k3_rm, max_v_nav_v, max_a_nav, max_j_lin, 1);    
      oneloop_andi.gui_ref.pos[2] = single_value_ref[0];
      oneloop_andi.gui_ref.vel[2] = single_value_d_ref[0];
      oneloop_andi.gui_ref.acc[2] = single_value_2d_ref[0];
      oneloop_andi.gui_ref.jer[2] = single_value_3d_ref[0];
    } else if (rm_order_v == 2){
      rm_2nd_pos(dt_1l, single_value_d_ref, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k2_rm, single_value_k3_rm, max_a_nav, max_j_lin, 1);   
      oneloop_andi.gui_ref.pos[2] = oneloop_andi.gui_state.pos[2];
      oneloop_andi.gui_ref.vel[2] = single_value_d_ref[0];
      oneloop_andi.gui_ref.acc[2] = single_value_2d_ref[0];
      oneloop_andi.gui_ref.jer[2] = single_value_3d_ref[0];
    } else if (rm_order_v == 1){
      rm_1st_pos(dt_1l, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k3_rm, max_j_lin, 1); 
      oneloop_andi.gui_ref.pos[2] = oneloop_andi.gui_state.pos[2];
      oneloop_andi.gui_ref.vel[2] = oneloop_andi.gui_state.vel[2];
      oneloop_andi.gui_ref.acc[2] = single_value_2d_ref[0];
      oneloop_andi.gui_ref.jer[2] = single_value_3d_ref[0];  
    }    
    // Run chirp test if turnerd on (overwrite the guidance references)
    chirp_call(&chirp_on, &chirp_first_call, &t_0_chirp, &time_elapsed_chirp, f0_chirp, f1_chirp, t_chirp, A_chirp, chirp_axis, att_des[2], oneloop_andi.gui_ref.pos, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer,p_ref_0);
    // Generate Reference signals for attitude using RM
    // FIX ME ow not yet defined, will be useful in the future to have accurate psi tracking in NAV functions
    bool ow_psi = false;
    if (chirp_on && (chirp_axis==4|| chirp_axis==5)) {
      ow_psi = true;
      psi_vec[0] = oneloop_andi.sta_ref.att[2];
      psi_vec[1] = oneloop_andi.sta_ref.att_d[2];
      psi_vec[2] = oneloop_andi.sta_ref.att_2d[2];
      psi_vec[3] = oneloop_andi.sta_ref.att_3d[2];
    }
    bool ow_phi = false;
    if (chirp_on && chirp_axis==6){
      ow_phi = true;
      phi_vec[0] = oneloop_andi.sta_ref.att[0];
      phi_vec[1] = oneloop_andi.sta_ref.att_d[0];
      phi_vec[2] = oneloop_andi.sta_ref.att_2d[0];
      phi_vec[3] = oneloop_andi.sta_ref.att_3d[0];
    }
    rm_3rd_attitude(dt_1l, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, att_des, ow_psi, psi_vec, k_att_rm.k1, k_att_rm.k2, k_att_rm.k3, sta_bounds);
    if (ow_phi){
      oneloop_andi.sta_ref.att[0]    = phi_vec[0];
      oneloop_andi.sta_ref.att_d[0]  = phi_vec[1];
      oneloop_andi.sta_ref.att_2d[0] = phi_vec[2];
      oneloop_andi.sta_ref.att_3d[0] = phi_vec[3];
    }
 }
}

/**
 * @brief  Main function that runs the controller and performs control allocation
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * @param in_flight  The drone is in flight
 * @param PSA_des    Desired position/speed/acceleration
 * @param rm_order_h Order of the reference model for horizontal guidance
 * @param rm_order_v Order of the reference model for vertical guidance
 */
void oneloop_andi_run(bool in_flight, bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v)
{
  // At beginnig of the loop: (1) Register Attitude, (2) Initialize gains of RM and EC, (3) Calculate Normalization of Actuators Signals, (4) Propagate Actuator Model, (5) Update effectiveness matrix
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  init_controller_gains();
  calc_normalization();
  get_act_state_oneloop();
  G1G2_oneloop(oneloop_andi.ctrl_type);
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = EFF_MAT_G[i];
  }
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    act_dyn_ctrl[i] = act_dynamics[i];
  }
  act_dyn_ctrl[COMMAND_MOTOR_RIGHT] = oneloop_andi_sigma;
  act_dyn_ctrl[COMMAND_MOTOR_LEFT]  = oneloop_andi_sigma;

  // If drone is not on the ground use incremental law
  use_increment = 0.0;
  bool  in_flight_oneloop = false;
  if(in_flight) {
    use_increment = 1.0;
    in_flight_oneloop = true;
  } 
  if (ONELOOP_ANDI_DEBUG_MODE) {
    in_flight_oneloop = false;
  }
  oneloop_calc_model_disturbance(in_flight);
  // Register the state of the drone in the variables used in RM and EC
  // (1) Attitude related
  oneloop_andi.sta_state.att[0]    = eulers_zxy.phi  ;
  oneloop_andi.sta_state.att[1]    = eulers_zxy.theta;
  oneloop_andi.sta_state.att[2]    = eulers_zxy.psi  ;
  oneloop_andi_propagate_filters();   //needs to be after update of attitude vector
  oneloop_andi.sta_state.att_d[0]  = LP.p.out;
  oneloop_andi.sta_state.att_d[1]  = LP.q.out;
  oneloop_andi.sta_state.att_d[2]  = LP.r.out;
  oneloop_andi.sta_state.att_2d[0] = LP.p_dot.out;
  oneloop_andi.sta_state.att_2d[1] = LP.q_dot.out;
  oneloop_andi.sta_state.att_2d[2] = LP.r_dot.out;
  // (2) Position related
  oneloop_andi.gui_state.pos[0] = stateGetPositionNed_f()->x;   
  oneloop_andi.gui_state.pos[1] = stateGetPositionNed_f()->y;   
  oneloop_andi.gui_state.pos[2] = stateGetPositionNed_f()->z;   
  oneloop_andi.gui_state.vel[0] = filt_veloc_N.o[0];      
  oneloop_andi.gui_state.vel[1] = filt_veloc_E.o[0];      
  oneloop_andi.gui_state.vel[2] = filt_veloc_D.o[0];      
  oneloop_andi.gui_state.acc[0] = LP.ax.out;
  oneloop_andi.gui_state.acc[1] = LP.ay.out;
  oneloop_andi.gui_state.acc[2] = LP.az.out;
  // Calculated feedforward signal for yaw control
  g2_ff = 0.0;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    if (oneloop_andi.ctrl_type == CTRL_ANDI){
      g2_ff += G2_RW[i] * act_dyn_ctrl[i] * (andi_u[i]-u_filt[i].o[0]);
      //printf("i: %d\n", i);
      //printf("andi_u: %f\n", andi_u[i]);
      //printf("actuator_state_1l: %f\n", actuator_state_1l[i]);
      //printf("act_dynamics: %f\n", act_dynamics[i]);
      //printf("G2_RW: %f\n", G2_RW[i]);
      //printf("g2_ff: %f\n", g2_ff);
    } else if (oneloop_andi.ctrl_type == CTRL_INDI){
      g2_ff += G2_RW[i] * (andi_u[i]-u_filt[i].o[0]);
    }
  }
  // Run the Reference Model (RM)
  oneloop_andi_RM(half_loop, PSA_des, rm_order_h, rm_order_v, in_flight_oneloop);
  // Run Distrubance Bounder
  oneloop_andi_bound_disturbance(); // Fixme, can be removed
  // Guidance Pseudo Control Vector (nu) based on error controller
  if(half_loop){
    nu[0] = 0.0;
    nu[1] = 0.0;
    nu[2] = a_thrust;
    ctrl_off = false;
  }else{
    if(oneloop_andi.ctrl_type == CTRL_ANDI){
      //float temp_dist_bound_gui[3] = {oneloop_andi_model[0], oneloop_andi_model[1], oneloop_andi_model[2]};
      float temp_dist_bound_gui[3] = {0.0, 0.0, 0.0};
      ec_3rd_pos(nu, oneloop_andi.gui_ref.pos, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, oneloop_andi.gui_ref.jer, oneloop_andi.gui_state.pos, oneloop_andi.gui_state.vel, oneloop_andi.gui_state.acc, k_pos_e.k1, k_pos_e.k2, k_pos_e.k3, max_v_nav, max_a_nav, max_j_lin,temp_dist_bound_gui,3);
    } else if (oneloop_andi.ctrl_type == CTRL_INDI){
      float dummy1[3] = {0.0, 0.0, 0.0};
      ec_3rd_pos(nu, oneloop_andi.gui_ref.pos, oneloop_andi.gui_ref.vel, oneloop_andi.gui_ref.acc, dummy1, oneloop_andi.gui_state.pos, oneloop_andi.gui_state.vel, oneloop_andi.gui_state.acc, k_pos_e_indi.k1, k_pos_e_indi.k2, k_pos_e_indi.k3, max_v_nav, max_a_nav, max_j_lin,dummy1,3);
      //nu[0] = ec_3rd(oneloop_andi.gui_ref.pos[0], oneloop_andi.gui_ref.vel[0], oneloop_andi.gui_ref.acc[0], 0.0, oneloop_andi.gui_state.pos[0], oneloop_andi.gui_state.vel[0], oneloop_andi.gui_state.acc[0], k_pos_e_indi.k1[0], k_pos_e_indi.k2[0], k_pos_e_indi.k3[0]);
      //nu[1] = ec_3rd(oneloop_andi.gui_ref.pos[1], oneloop_andi.gui_ref.vel[1], oneloop_andi.gui_ref.acc[1], 0.0, oneloop_andi.gui_state.pos[1], oneloop_andi.gui_state.vel[1], oneloop_andi.gui_state.acc[1], k_pos_e_indi.k1[1], k_pos_e_indi.k2[1], k_pos_e_indi.k3[1]);
      //nu[2] = ec_3rd(oneloop_andi.gui_ref.pos[2], oneloop_andi.gui_ref.vel[2], oneloop_andi.gui_ref.acc[2], 0.0, oneloop_andi.gui_state.pos[2], oneloop_andi.gui_state.vel[2], oneloop_andi.gui_state.acc[2], k_pos_e_indi.k1[2], k_pos_e_indi.k2[2], k_pos_e_indi.k3[2]);  
    }
  }
  // Attitude Pseudo Control Vector (nu) based on error controller
  float y_4d_att[3];  
  if(oneloop_andi.ctrl_type == CTRL_ANDI){
    //float temp_dist_bound_sta[3] = {oneloop_andi_model_filt.p_dot.out, oneloop_andi_model_filt.q_dot.out, oneloop_andi_model_filt.r_dot.out};
    float temp_dist_bound_sta[3] = {oneloop_andi_model[RW_ap], oneloop_andi_model[RW_aq], oneloop_andi_model[RW_ar]};
    ec_3rd_att(y_4d_att, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, oneloop_andi.sta_ref.att_3d, oneloop_andi.sta_state.att, oneloop_andi.sta_state.att_d, oneloop_andi.sta_state.att_2d, k_att_e.k1, k_att_e.k2, k_att_e.k3, sta_bounds,temp_dist_bound_sta);
} else if (oneloop_andi.ctrl_type == CTRL_INDI){
    float dummy0[3] = {0.0, 0.0, 0.0};
    float temp_dist_bound_sta_INDI[3] = {oneloop_andi_model[RW_ap]*k_att_e.k3[0], oneloop_andi_model[RW_aq]*k_att_e.k3[1], oneloop_andi_model[RW_ar]*k_att_e.k3[2]};
    ec_3rd_att(y_4d_att, oneloop_andi.sta_ref.att, oneloop_andi.sta_ref.att_d, oneloop_andi.sta_ref.att_2d, dummy0, oneloop_andi.sta_state.att, oneloop_andi.sta_state.att_d, oneloop_andi.sta_state.att_2d, k_att_e_indi.k1, k_att_e_indi.k2, k_att_e_indi.k3, sta_bounds, temp_dist_bound_sta_INDI);
  }
  if(half_loop && radio_control_get(RADIO_THROTTLE)<200){
    nu[3] = 0.0;
    nu[4] = 0.0;
    nu[5] = 0.0;
  } else {
    nu[3] = y_4d_att[0];  
    nu[4] = y_4d_att[1]; 
    nu[5] = y_4d_att[2] + g2_ff;
  }

  // temp restructuring------------------
  nu[0] = nu[0] + oneloop_andi_model[RW_aN];//oneloop_andi_model_filt.ax.out;
  nu[1] = nu[1] + oneloop_andi_model[RW_aE];//oneloop_andi_model_filt.ay.out;
  nu[2] = nu[2] + oneloop_andi_model[RW_aD];//oneloop_andi_model_filt.az.out;
  //nu[3] = nu[3] + oneloop_andi_model[3];
  //nu[4] = nu[4] + oneloop_andi_model[4];
  //nu[5] = nu[5] + oneloop_andi_model[5];
  //BoundAbs(nu[5], n_array[5]*coupling_factor[5]); //FIXME UNCOMMENT ME
  // weather vaning ---------------------
  //nu[5] = oneloop_andi_model[5] - temp_k * oneloop_andi.sta_ref.att_d[2]; //Interesting idea to weather vane the drone
  if (drop_yaw){
    // nu[0] = oneloop_andi_model_filt.ax.out;
    // nu[1] = oneloop_andi_model_filt.ay.out;
    // nu[2] = oneloop_andi_model_filt.az.out;
    // nu[3] = oneloop_andi_model_filt.p_dot.out*k_att_e.k3[0];
    // nu[4] = oneloop_andi_model_filt.q_dot.out*k_att_e.k3[1];
    nu[5] = oneloop_andi_model_filt.r_dot.out*k_att_e.k3[2];
  }
  //------------------------------------

  if (!chirp_on){
    pitch_pref = radio_control.values[RADIO_AUX5]; 
    pitch_pref = pitch_pref / MAX_PPRZ * theta_pref_max;
    Bound(pitch_pref,0.0,theta_pref_max);
  }
  u_pref[COMMAND_PITCH] = 0.0;//pitch_pref; // FIXME
  // Calculate the min and max increments
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    //printf("Act_dyn[%d]: %f\n",i,act_dynamics[i]);
    switch (i) {
      case COMMAND_MOTOR_FRONT:
      case COMMAND_MOTOR_RIGHT:
      case COMMAND_MOTOR_BACK:
      case COMMAND_MOTOR_LEFT: {
        float skew_bound = RW.skew.deg;
        Bound(skew_bound,70.0,90.0);
        float Wu_sched   = (Wu_quad_motors_fwd-Wu_backup[i])/(90.0-70.0)*(skew_bound-70)+Wu_backup[i];
        WLS_one_p.Wu[i]     = Wu_sched;
        WLS_one_p.u_min[i]  = (act_min[i])/ratio_u_un[i];
        WLS_one_p.u_pref[i] = (u_pref[i] )/ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i])/ratio_u_un[i];
        if (rotwing_state.hover_motors_enabled){
          WLS_one_p.u_max[i]  = (act_max[i])/ratio_u_un[i];
        } else
        {
          WLS_one_p.u_max[i]  = (0.0)/ratio_u_un[i];
        }
        break;
      }
      case COMMAND_MOTOR_PUSHER:
      case COMMAND_RUDDER:
        WLS_one_p.u_min[i]  = (act_min[i])/ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i])/ratio_u_un[i];
        WLS_one_p.u_pref[i] = (u_pref[i] )/ratio_u_un[i]; 
        break;      
      case COMMAND_AILERONS:
        if(RW.skew.deg > 25.0){
            WLS_one_p.u_min[i]  = (act_min[i])/ratio_u_un[i];
            WLS_one_p.u_max[i]  = (act_max[i])/ratio_u_un[i];
          } else {
            WLS_one_p.u_min[i]  = (0.0)/ratio_u_un[i];
            WLS_one_p.u_max[i]  = (0.0)/ratio_u_un[i];
          }
        WLS_one_p.u_pref[i] = (u_pref[i])/ratio_u_un[i];   
        break;      
      case COMMAND_FLAPS:
        if(RW.skew.deg > 50.0){
            WLS_one_p.u_min[i]  = (act_min[i])/ratio_u_un[i];
            WLS_one_p.u_max[i]  = (act_max[i])/ratio_u_un[i];
          } else {
            WLS_one_p.u_min[i]  = (0.0)/ratio_u_un[i];
            WLS_one_p.u_max[i]  = (0.0)/ratio_u_un[i];
          }
        WLS_one_p.u_pref[i] = (u_pref[i])/ratio_u_un[i];   
        break;
      case COMMAND_ELEVATOR:  
        WLS_one_p.u_min[i]  = (act_min[i])/ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i])/ratio_u_un[i];
        u_pref[i]           = RW.ele_pref;
        WLS_one_p.u_pref[i] = (u_pref[i])/ratio_u_un[i];  
        break;     
      case COMMAND_ROLL:
        WLS_one_p.u_min[i]  = (act_min[i])/ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i])/ratio_u_un[i];
        WLS_one_p.u_pref[i] = (u_pref[i] )/ratio_u_un[i];
        break;         
      case COMMAND_PITCH:
        if (RW.skew.deg > 50.0){
          WLS_one_p.u_min[i]  = (RadOfDeg(-17.0))/ratio_u_un[i];
          WLS_one_p.u_max[i]  = (RadOfDeg(17.0) )/ratio_u_un[i];
        } else {
          WLS_one_p.u_min[i]  = (act_min[i])/ratio_u_un[i];
          WLS_one_p.u_max[i]  = (act_max[i])/ratio_u_un[i];
        }
        WLS_one_p.u_pref[i] = (u_pref[i])/ratio_u_un[i];
        break;           
    }
  }
  // WLS Control Allocator
  normalize_nu();
  wls_alloc(&WLS_one_p, bwls_1l, 0, 0, 10);

  if (!sys_id_doublet_running()){   
    for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
      andi_u_n[i] = WLS_one_p.u[i];
      andi_u[i]   = (float)(andi_u_n[i] * ratio_u_un[i]);
    }
  } else {
    reinit_controller();
  }
#ifdef COMMAND_MOTOR_PUSHER
  if ((half_loop)){
    andi_u[COMMAND_MOTOR_PUSHER] = radio_control.values[RADIO_AUX4];
  }
#endif  
  // Run the dynamics substitution filter 
  dynFilter_run(&mu_mR, andi_u[COMMAND_MOTOR_RIGHT], oneloop_andi_sigma);
  dynFilter_run(&mu_mL, andi_u[COMMAND_MOTOR_LEFT],  oneloop_andi_sigma);
  if (use_dyn_filter){
    andi_u[COMMAND_MOTOR_RIGHT] = mu_mR.mu_c;
    andi_u[COMMAND_MOTOR_LEFT]  = mu_mL.mu_c;
  }
  // TODO : USE THE PROVIDED MAX AND MIN and change limits for phi and theta
  // Bound the inputs to the actuators
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    Bound(andi_u[i], act_min[i], act_max[i]);
  }
  /*Commit the actuator command*/
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    commands[i] = (int16_t) andi_u[i];
  }
  if (rotwing_state.fail_pusher_motor){
    commands[COMMAND_MOTOR_PUSHER] = -9600;//Min(1000,andi_u[COMMAND_MOTOR_PUSHER]);
  }
  // if (drop_yaw && radio_control_get(RADIO_THROTTLE)>200){
  //   commands[COMMAND_MOTOR_RIGHT] = radio_control_get(RADIO_THROTTLE);
  //   commands[COMMAND_MOTOR_LEFT]  = radio_control_get(RADIO_THROTTLE);
  //   commands[COMMAND_MOTOR_FRONT] = radio_control_get(RADIO_THROTTLE);
  //   commands[COMMAND_MOTOR_BACK]  = radio_control_get(RADIO_THROTTLE);

  // }
  commands[COMMAND_THRUST] = (commands[COMMAND_MOTOR_FRONT] + commands[COMMAND_MOTOR_RIGHT] + commands[COMMAND_MOTOR_BACK] + commands[COMMAND_MOTOR_LEFT])/num_thrusters_oneloop;
  autopilot.throttle = commands[COMMAND_THRUST];
  stabilization.cmd[COMMAND_THRUST] = commands[COMMAND_THRUST];
  if(!half_loop){
    eulers_zxy_des.phi   =  andi_u[COMMAND_ROLL];
    eulers_zxy_des.theta =  andi_u[COMMAND_PITCH];
    //eulers_zxy_des.phi   = (float) (radio_control_get(RADIO_ROLL)) /MAX_PPRZ * ONELOOP_ANDI_MAX_PHI  ;
    //eulers_zxy_des.theta = (float) (radio_control_get(RADIO_PITCH))/MAX_PPRZ * ONELOOP_ANDI_MAX_THETA;
  }
  if (heading_manual){
    psi_des_deg = DegOfRad(psi_des_rad);
  } 

  stabilization.cmd[COMMAND_ROLL]  = (int16_t) (DegOfRad(eulers_zxy_des.phi  ) * MAX_PPRZ / DegOfRad(ONELOOP_ANDI_MAX_PHI  ));
  stabilization.cmd[COMMAND_PITCH] = (int16_t) (DegOfRad(eulers_zxy_des.theta) * MAX_PPRZ / DegOfRad(ONELOOP_ANDI_MAX_THETA));
  stabilization.cmd[COMMAND_YAW]   = (int16_t) (psi_des_deg * MAX_PPRZ / 180.0);
}

/** @brief  Function to reconstruct actuator state using first order dynamics */
void get_act_state_oneloop(void)
{
  int8_t i;
  float prev_actuator_state_1l;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    if(i < ANDI_NUM_ACT){
      prev_actuator_state_1l = actuator_state_1l[i];
      actuator_state_1l[i] = prev_actuator_state_1l + act_dynamics_d[i] * (andi_u[i] - prev_actuator_state_1l);
      if(!autopilot_get_motors_on()){
        actuator_state_1l[i] = 0.0;
      }
      Bound(actuator_state_1l[i],act_min[i], act_max[i]);
    } else {
      actuator_state_1l[i] = oneloop_andi.sta_state.att[i-ANDI_NUM_ACT];
    }
  }
}

/**
 * @brief Function that samples and scales the effectiveness matrix
 * FIXME: make this function into a for loop to make it more adaptable to different configurations
 */
void G1G2_oneloop(int ctrl_type) {
  int i = 0;
  float scaler = 1.0;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    //printf("act_dyn_ctrl[%d] = %f\n", i, act_dyn_ctrl[i]);
    switch (i) {
      case (COMMAND_MOTOR_FRONT):
      case (COMMAND_MOTOR_RIGHT):
      case (COMMAND_MOTOR_BACK):
      case (COMMAND_MOTOR_LEFT):
      case (COMMAND_MOTOR_PUSHER):
      case (COMMAND_ELEVATOR):
      case (COMMAND_RUDDER):  
      case (COMMAND_AILERONS):
      case (COMMAND_FLAPS):   
        if(ctrl_type == CTRL_ANDI){
          scaler = act_dyn_ctrl[i] * ratio_u_un[i];
        } else if (ctrl_type == CTRL_INDI){
          scaler = ratio_u_un[i];
        }
        break;
      case (COMMAND_ROLL):
      case (COMMAND_PITCH):
        if(ctrl_type == CTRL_ANDI){
          scaler = act_dyn_ctrl[i] * ratio_u_un[i];
        } else if (ctrl_type == CTRL_INDI){
          scaler = ratio_u_un[i];
        }
        break;
      default:
        break;        
    }
    int j = 0;
    bool turn_quad_off = ((!rotwing_state.hover_motors_enabled || !rotwing_state_hover_motors_running()) && rotwing_state.state != ROTWING_STATE_FORCE_HOVER);
    for (j = 0; j < ANDI_OUTPUTS; j++) {
      EFF_MAT_G[j][i] = EFF_MAT_RW[j][i] * scaler * ratio_vn_v[j];
      if (drop_yaw){
        EFF_MAT_G[5][i] = 0.0;
      }
      if (airspeed_filt.o[0] < ELE_MIN_AS && i == COMMAND_ELEVATOR){
        EFF_MAT_G[j][i] = 0.0;
      }
      if (turn_quad_off && i < 4){
        EFF_MAT_G[j][i] = 0.0;
      }
      if (ctrl_off && i < 4  && j == 5){ //hack test
        EFF_MAT_G[j][i] = 0.0;
      } 
      if (ctrl_off && i < 4  && j == 4){ //hack test
        EFF_MAT_G[j][i] = 0.0;
      } 
      if (ctrl_off && i < 4  && j == 3){ //hack test
        EFF_MAT_G[j][i] = 0.0;
      } 
      // if (j < 2 && i < 4){
      //   EFF_MAT_G[j][i] = 0.0;
      // }

    }
  }
  oneloop_axis_effectiveness_calc();
}

/** @brief  Calculate Normalization of actuators and discrete actuator dynamics  */
void calc_normalization(void){
  int8_t i;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
    act_dynamics_d[i] = 1.0-exp(-act_dynamics[i]*dt_1l);
    Bound(act_dynamics_d[i],0.00001,1.0);
    Bound(act_max[i],0,MAX_PPRZ);
    Bound(act_min[i],-MAX_PPRZ,0);
    float ratio_numerator = act_max[i]-act_min[i];
    ratio_numerator = positive_non_zero(ratio_numerator);// make sure numerator is non-zero
    float ratio_denominator = act_max_norm[i]-act_min_norm[i];
    ratio_denominator = positive_non_zero(ratio_denominator); // make sure denominator is non-zero
    ratio_u_un[i] = ratio_numerator/ratio_denominator;
    ratio_u_un[i] = positive_non_zero(ratio_u_un[i]);// make sure ratio is not zero
    //printf("ratio_u_un[%d]= %f \n",i,ratio_u_un[i]);
  }
  for (i = 0; i < ANDI_OUTPUTS; i++){
    float ratio_numerator = positive_non_zero(nu_norm_max);
    float ratio_denominator = 1.0;
    switch (i) {
      case (RW_aN):
      case (RW_aE):
      case (RW_aD):
        ratio_denominator = positive_non_zero(max_j_lin);
        ratio_vn_v[i] = ratio_numerator/ratio_denominator;
        break;
      case (RW_ap):
        ratio_denominator = positive_non_zero(sta_bounds.att_3d[0]);
        ratio_vn_v[i] = ratio_numerator/ratio_denominator;
        break;            
      case (RW_aq):
        ratio_denominator = positive_non_zero(sta_bounds.att_3d[1]);
        ratio_vn_v[i] = ratio_numerator/ratio_denominator;
        break;      
      case (RW_ar):
        ratio_denominator = positive_non_zero(sta_bounds.att_3d[2]);
        ratio_vn_v[i] = ratio_numerator/ratio_denominator;
        break;      
    }
  }
}

/** @brief  Function to normalize the pseudo control vector */
void normalize_nu(void){
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++){
    //printf("ratio_vn_v[%d] = %f \n",i,ratio_vn_v[i]);
    nu_n[i] = nu[i] * ratio_vn_v[i];
    WLS_one_p.v[i] = nu_n[i];
  }
}

/** @brief  Function that calculates the model prediction for the complementary filter. */
// void calc_model(void){
//   int8_t i;
//   int8_t j;
//   // // Absolute Model Prediction : 
//   float sphi   = sinf(eulers_zxy.phi);
//   float cphi   = cosf(eulers_zxy.phi);
//   float stheta = sinf(eulers_zxy.theta);
//   float ctheta = cosf(eulers_zxy.theta);
//   float spsi   = sinf(eulers_zxy.psi);
//   float cpsi   = cosf(eulers_zxy.psi);
//   // Thrust and Pusher force estimation
//   float L      = RW.wing.L / RW.m;          // Lift specific force
//   float T      = RW.T / RW.m;             //  Thrust specific force. Minus gravity is a guesstimate.
//   float P      = RW.P / RW.m;               // Pusher specific force

//   cf.ax.model = -(cpsi * stheta + ctheta * sphi * spsi) * T + (cpsi * ctheta - sphi * spsi * stheta) * P - sphi * spsi * L;
//   cf.ay.model = -(spsi * stheta - cpsi * ctheta * sphi) * T + (ctheta * spsi + cpsi * sphi * stheta) * P + cpsi * sphi * L;
//   cf.az.model = g - cphi * ctheta * T - cphi * stheta * P - cphi * L;
//   float model_pqr_dot[3] = {0.0, 0.0, 0.0};
//   for (i = 0; i < 3; i++){ // For loop for prediction of angular acceleration 
//     for (j = 0; j < ANDI_NUM_ACT; j++){
//       if(j == COMMAND_ELEVATOR){
//         model_pqr_dot[i] = model_pqr_dot[i] +  (actuator_state_1l[j] - RW.ele_pref) * EFF_MAT_RW[i+3][j]; // Ele pref is incidence angle
//       } else {
//         model_pqr_dot[i] = model_pqr_dot[i] +  actuator_state_1l[j] * EFF_MAT_RW[i+3][j];
//       }
//     }
//   }
//   cf.p_dot.model = model_pqr_dot[0];
//   cf.q_dot.model = model_pqr_dot[1];
//   cf.r_dot.model = model_pqr_dot[2];
// }

/** @brief  Function that maps navigation inputs to the oneloop controller for the generated autopilot. */
void oneloop_from_nav(bool in_flight)
{
  if (!in_flight) {
    oneloop_andi_enter(false, oneloop_andi.ctrl_type);
  }
  struct FloatVect3 PSA_des;
  PSA_des.x = stateGetPositionNed_f()->x;
  PSA_des.y = stateGetPositionNed_f()->y;
  PSA_des.z = stateGetPositionNed_f()->z;
  int    rm_order_h = 3;
  int    rm_order_v = 3;
  // Oneloop controller wants desired targets and handles reference generation internally
  switch (nav.setpoint_mode) {
    case NAV_SETPOINT_MODE_POS:
      PSA_des.x   = POS_FLOAT_OF_BFP(POS_BFP_OF_REAL(nav.target.y));
      PSA_des.y   = POS_FLOAT_OF_BFP(POS_BFP_OF_REAL(nav.target.x));
      rm_order_h  = 3;
      break;
    case NAV_SETPOINT_MODE_SPEED:
      PSA_des.x   = SPEED_FLOAT_OF_BFP(SPEED_BFP_OF_REAL(nav.speed.y));
      PSA_des.y   = SPEED_FLOAT_OF_BFP(SPEED_BFP_OF_REAL(nav.speed.x));
      rm_order_h  = 2;
      break;
  }
  switch (nav.vertical_mode) {
    case NAV_VERTICAL_MODE_ALT:
      PSA_des.z   = POS_FLOAT_OF_BFP(-POS_BFP_OF_REAL(nav.nav_altitude));
      rm_order_v  = 3;
      break;
    case NAV_VERTICAL_MODE_CLIMB:
      PSA_des.z   = SPEED_FLOAT_OF_BFP(-SPEED_BFP_OF_REAL(nav.climb));
      rm_order_v  = 2;
      break;
  }
  oneloop_andi_run(in_flight, false, PSA_des, rm_order_h, rm_order_v);
}

/** @brief Function to calculate corrections for sideslip*/
float oneloop_andi_sideslip(void)
{
  //printf("Calculating the sideslip correction\n");
  // Coordinated turn
  // feedforward estimate angular rotation omega = g*tan(phi)/v
  float omega = 0.0;
  const float max_phi = ONELOOP_ANDI_MAX_BANK;//RadOfDeg(ONELOOP_ANDI_MAX_BANK);
  float airspeed_turn = airspeed_filt.o[0];
  Bound(airspeed_turn, 1.0f, 30.0f);
  // Use the current roll angle to determine the corresponding heading rate of change.
  float coordinated_turn_roll = eulers_zxy.phi;
  // Prevent flipping
  if( (eulers_zxy.theta > 0.0f) && ( fabs(eulers_zxy.phi) < eulers_zxy.theta)) {
    //printf("Preventing flipping\n");
    coordinated_turn_roll = ((eulers_zxy.phi > 0.0f) - (eulers_zxy.phi < 0.0f)) * eulers_zxy.theta;
  }
  BoundAbs(coordinated_turn_roll, max_phi);
  omega = g / airspeed_turn * tanf(coordinated_turn_roll);
  //printf("Omega: %f\n", omega);
  //printf("Coordinated turn roll: %f\n", DegOfRad(coordinated_turn_roll));
  //printf("phi: %f\n", DegOfRad(eulers_zxy.phi));
  //printf("Airspeed: %f\n", airspeed_turn);
  #ifdef FWD_SIDESLIP_GAIN
  // Add sideslip correction
  omega -= accely_filt.o[0]*fwd_sideslip_gain;
  //printf("Omega ay: %f\n", omega);
  #endif
  return omega;
}

/** @brief Function to calculate the position reference during the chirp*/
static float chirp_pos_p_ref(float delta_t, float f0, float k, float A){
  float p_ref_fun = sinf(delta_t * M_PI * (f0 + k * delta_t) * 2.0);
  return (A * p_ref_fun);
}
/** @brief Function to calculate the velocity reference during the chirp*/
static float chirp_pos_v_ref(float delta_t, float f0, float k, float A){
  float v_ref_fun = cosf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * (M_PI * (f0 + k * delta_t) * 2.0 + k * delta_t * M_PI * 2.0);
  return (A * v_ref_fun);
}
/** @brief Function to calculate the acceleration reference during the chirp*/
static float chirp_pos_a_ref(float delta_t, float f0, float k, float A){
  float a_ref_fun = -sinf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * pow((M_PI * (f0 + k * delta_t) * 2.0 + k * delta_t * M_PI * 2.0), 2) + k * M_PI * cosf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * 4.0;
  return (A * a_ref_fun);
}
/** @brief Function to calculate the jerk reference during the chirp*/
static float chirp_pos_j_ref(float delta_t, float f0, float k, float A){
  float j_ref_fun = -cosf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * pow((M_PI * (f0 + k * delta_t) * 2.0 + k * delta_t * M_PI * 2.0), 3) - k * M_PI * sinf(delta_t * M_PI * (f0 + k * delta_t) * 2.0) * (M_PI * (f0 + k * delta_t) * 2.0 + k * delta_t * M_PI * 2.0) * 1.2e+1;
  return (A * j_ref_fun);
}

/** 
 * @brief Reference Model Definition for 3rd order system specific to positioning with bounds
 * @param dt      [s]    time passed since start of the chirp
 * @param f0      [Hz]   initial frequency of the chirp
 * @param f1      [Hz]   final frequency of the chirp
 * @param t_chirp [s]    duration of the chirp
 * @param p_ref   [m]    position reference
 * @param v_ref   [m/s]  velocity reference
 * @param a_ref   [m/s2] acceleration reference
 * @param j_ref   [m/s3] jerk reference
 */
void chirp_pos(float time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]) {
  float A_backup = A;
  f0      = positive_non_zero(f0);
  f1      = positive_non_zero(f1);
  t_chirp = positive_non_zero(t_chirp);
  A       = positive_non_zero(A);
  if ((f1-f0) < -FLT_EPSILON){
    f1 = f0;
  }
  // 0 body x, 1 body y, 2 body z, 3 pitch pref, 4 Yaw
  if (n > 6){
    n = 0;
  }
  if (n < 0){
    n = 0;
  }
  // I think there should not be a problem with f1 being equal to f0
  float k = (f1 - f0) / t_chirp;
  float p_ref_chirp = chirp_pos_p_ref(time_elapsed, f0, k, A);
  float v_ref_chirp = chirp_pos_v_ref(time_elapsed, f0, k, A);
  float a_ref_chirp = chirp_pos_a_ref(time_elapsed, f0, k, A);
  float j_ref_chirp = chirp_pos_j_ref(time_elapsed, f0, k, A);

  float spsi   = sinf(psi);
  float cpsi   = cosf(psi);
  float mult_0 = 0.0;
  float mult_1 = 0.0;
  float mult_2 = 0.0;
  switch (n) {
      case 0:
          mult_0 = cpsi;
          mult_1 = spsi;
          mult_2 = 0.0;
          
          p_ref[0] = p_ref_0[0] + p_ref_chirp * mult_0;
          p_ref[1] = p_ref_0[1] + p_ref_chirp * mult_1; 
          v_ref[0] = v_ref_chirp * mult_0;
          v_ref[1] = v_ref_chirp * mult_1;
          a_ref[0] = a_ref_chirp * mult_0;
          a_ref[1] = a_ref_chirp * mult_1; 
          j_ref[0] = j_ref_chirp * mult_0;
          j_ref[1] = j_ref_chirp * mult_1;
          break;
      case 1:
          mult_0 = -spsi;
          mult_1 = cpsi;
          mult_2 = 0.0;
          
          p_ref[0] = p_ref_0[0] + p_ref_chirp * mult_0;
          p_ref[1] = p_ref_0[1] + p_ref_chirp * mult_1; 
          v_ref[0] = v_ref_chirp * mult_0;
          v_ref[1] = v_ref_chirp * mult_1;
          a_ref[0] = a_ref_chirp * mult_0;
          a_ref[1] = a_ref_chirp * mult_1; 
          j_ref[0] = j_ref_chirp * mult_0;
          j_ref[1] = j_ref_chirp * mult_1;
          break;
      case 2:
          mult_0 = 0.0;
          mult_1 = 0.0;
          mult_2 = 1.0;
          p_ref[2] = p_ref_0[2] + p_ref_chirp * mult_2;
          v_ref[2] = v_ref_chirp * mult_2;
          a_ref[2] = a_ref_chirp * mult_2;
          j_ref[2] = j_ref_chirp * mult_2;
          break;
      case 3:
          // Pitch preferred chirp
          pitch_pref = p_ref_chirp;
          pitch_pref = (pitch_pref / A + 1.0) * (theta_pref_max / 2.0);
          float pitch_offset = RadOfDeg(5.0);
          pitch_pref = pitch_pref + pitch_offset;
          Bound(pitch_pref,0.0,25.0);
          break;
      case 4:
          // Do a yaw acceleration chirp
          oneloop_andi.sta_ref.att[2]    = psi_des_rad+p_ref_chirp*M_PI/180.0;//oneloop_andi.sta_state.att[2];
          NormRadAngle(oneloop_andi.sta_ref.att[2]);
          oneloop_andi.sta_ref.att_d[2]  = v_ref_chirp*M_PI/180.0;//oneloop_andi.sta_state.att_d[2];
          oneloop_andi.sta_ref.att_2d[2] = a_ref_chirp*M_PI/180.0;//p_ref_chirp;
          oneloop_andi.sta_ref.att_3d[2] = j_ref_chirp*M_PI/180.0;//v_ref_chirp;   
          // Include change in dynamics
          oneloop_andi_sigma = (oneloop_andi_sigma_min-oneloop_andi_sigma_max)/t_chirp * time_elapsed + oneloop_andi_sigma_max;
          Bound(oneloop_andi_sigma, oneloop_andi_sigma_min, oneloop_andi_sigma_max);       
          break;
      case 5:
          oneloop_andi.sta_ref.att[2]    = oneloop_andi.sta_state.att[2];
          oneloop_andi.sta_ref.att_d[2]  = oneloop_andi.sta_state.att_d[2];
          oneloop_andi.sta_ref.att_2d[2] = A_backup*M_PI/180.0;
          oneloop_andi.sta_ref.att_3d[2] = 0.0;
          float S_psi = 0.5*A_backup*M_PI/180.0*t_chirp*t_chirp;
          BoundAbs(S_psi, 0.9*M_PI_2)
          psi_des_rad = oneloop_andi.sta_state.att[2] + S_psi;
          NormRadAngle(psi_des_rad);
          break;
      case 6:
          // Do a doublet on the roll acceleration
          oneloop_andi.sta_ref.att[0]    = oneloop_andi.sta_state.att[0];
          oneloop_andi.sta_ref.att_d[0]  = oneloop_andi.sta_state.att_d[0];
          //oneloop_andi.sta_ref.att_2d[0] = (time_elapsed < (t_chirp/2.0)) ? A*M_PI/180.0 : -A*M_PI/180.0;
          oneloop_andi.sta_ref.att_2d[0] = -A*M_PI/180.0;
          oneloop_andi.sta_ref.att_3d[0] = 0.0;
          break;
  }

}

void chirp_call(bool *chirp_on, bool *chirp_first_call, float* t_0, float* time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]){
  if (*chirp_on){
    if (*chirp_first_call){
      *time_elapsed = 0.0;
      *chirp_first_call = false;
      *t_0 = get_sys_time_float();
      p_ref_0[0] = p_ref[0];
      p_ref_0[1] = p_ref[1];
      p_ref_0[2] = p_ref[2];
    }
    if (*time_elapsed < t_chirp){
      *time_elapsed = get_sys_time_float() - *t_0;
      chirp_pos(*time_elapsed, f0, f1, t_chirp, A, n, psi, p_ref, v_ref, a_ref, j_ref, p_ref_0);
    } else {
      *chirp_on   = false;
      *chirp_first_call = true;
      *time_elapsed = 0.0;
      *t_0 = 0.0;
      p_ref_0[0] = p_ref[0];
      p_ref_0[1] = p_ref[1];
      p_ref_0[2] = p_ref[2];
      float_vect_zero(v_ref, 3);
      float_vect_zero(a_ref, 3);
      float_vect_zero(j_ref, 3);
      //oneloop_andi_sigma = oneloop_andi_sigma_max;
      //oneloop_andi_enter(false, oneloop_andi.ctrl_type);
    }
  }
}

/** Quadplanes can still be in-flight with COMMAND_THRUST==0 and can even soar not descending in updrafts with all thrust off */
bool autopilot_in_flight_end_detection(bool motors_on UNUSED) {
  return ! motors_on;
}


void reshape_wind(void)
{
  float psi = eulers_zxy.psi;
  float cpsi = cosf(psi);
  float spsi = sinf(psi);
  float airspeed = airspeed_filt.o[0];
  struct FloatVect2 NT_v_NE     = {nav_target[0], nav_target[1]}; // Nav target in North and East frame
  struct FloatVect2 airspeed_v  = { cpsi * airspeed, spsi * airspeed };
  struct FloatVect2 windspeed;
  struct FloatVect2 groundspeed = { oneloop_andi.gui_state.vel[0], oneloop_andi.gui_state.vel[1] };
  struct FloatVect2 des_as_NE;
  struct FloatVect2 des_as_B;
  struct FloatVect2 des_acc_B;
  VECT2_DIFF(windspeed, groundspeed, airspeed_v); // Wind speed in North and East frame
  VECT2_DIFF(des_as_NE, NT_v_NE, windspeed); // Desired airspeed in North and East frame
  float norm_des_as = FLOAT_VECT2_NORM(des_as_NE);
  gi_unbounded_airspeed_sp = norm_des_as;
  //Check if some minimum airspeed is desired (e.g. to prevent stall)
  if (norm_des_as < min_as) {
     norm_des_as = min_as;
  }
  nav_target_new[0] = NT_v_NE.x;
  nav_target_new[1] = NT_v_NE.y;
  // if the desired airspeed is larger than the max airspeed or we are in force forward reshape gs des to cancel wind and fly at max airspeed
  if ((norm_des_as > max_as)||(force_forward)){
    float groundspeed_factor = 0.0f;
    if (FLOAT_VECT2_NORM(windspeed) < max_as) {
      float av = NT_v_NE.x * NT_v_NE.x + NT_v_NE.y * NT_v_NE.y; // norm squared of nav target 
      float bv = -2.f * (windspeed.x * NT_v_NE.x + windspeed.y * NT_v_NE.y);
      float cv = windspeed.x * windspeed.x + windspeed.y * windspeed.y - max_as * max_as;
      float dv = bv * bv - 4.0f * av * cv;
      // dv can only be positive, but just in case
      if (dv < 0.0f) {
        dv = fabsf(dv);
      }
      float d_sqrt = sqrtf(dv);
      groundspeed_factor = (-bv + d_sqrt)  / (2.0f * av); 
    }
    des_as_NE.x = groundspeed_factor * NT_v_NE.x - windspeed.x;
    des_as_NE.y = groundspeed_factor * NT_v_NE.y - windspeed.y;
    NT_v_NE.x   = groundspeed_factor * NT_v_NE.x;
    NT_v_NE.y   = groundspeed_factor * NT_v_NE.y; 
    norm_des_as = max_as;
  }
  des_as_B.x  = norm_des_as; // Desired airspeed in body x frame
  des_as_B.y  = 0.0; // Desired airspeed in body y frame
  if (((airspeed > ONELOOP_ANDI_AIRSPEED_SWITCH_THRESHOLD) && (norm_des_as > (ONELOOP_ANDI_AIRSPEED_SWITCH_THRESHOLD+2.0f)))|| (force_forward)){
    float delta_psi = atan2f(des_as_NE.y, des_as_NE.x) - psi; 
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    des_acc_B.y = delta_psi * 5.0;//gih_params.heading_bank_gain;
    des_acc_B.x = (des_as_B.x - airspeed) * k_pos_rm.k2[0];//gih_params.speed_gain;
    acc_body_bound(&des_acc_B, max_a_nav); // Scale down side acceleration if norm is too large
    nav_target_new[0] = cpsi * des_acc_B.x - spsi * des_acc_B.y;
    nav_target_new[1] = spsi * des_acc_B.x + cpsi * des_acc_B.y; 
  } else {
    nav_target_new[0] = (NT_v_NE.x - groundspeed.x) * k_pos_rm.k2[0];
    nav_target_new[1] = (NT_v_NE.y - groundspeed.y) * k_pos_rm.k2[1];
  }
  vect_bound_nd(nav_target_new, max_a_nav, 2);
}

void guidance_set_min_max_airspeed(float min_airspeed, float max_airspeed) {
  min_as = min_airspeed;
  max_as = max_airspeed;
}

void oneloop_axis_effectiveness_calc(void){
  // Reset the effectiveness matrix
  memset(n_array, 0, sizeof(n_array));
  memset(coupling_factor, 0, sizeof(coupling_factor));
  memset(m_array, 0, sizeof(m_array));
  float u_trim[ANDI_NUM_ACT_TOT];
  u_trim[COMMAND_MOTOR_FRONT]  = 4800.0;
  u_trim[COMMAND_MOTOR_RIGHT]  = 4800.0;
  u_trim[COMMAND_MOTOR_BACK]   = 4800.0;
  u_trim[COMMAND_MOTOR_LEFT]   = 4800.0;
  u_trim[COMMAND_MOTOR_PUSHER] = 0.0;
  u_trim[COMMAND_ELEVATOR]     = RW.ele_pref;
  u_trim[COMMAND_RUDDER]       = 0.0;
  u_trim[COMMAND_AILERONS]     = 0.0;
  u_trim[COMMAND_FLAPS]        = 0.0;
  u_trim[COMMAND_ROLL]         = 0.0;
  u_trim[COMMAND_PITCH]        = 0.0;


  for (int i = 0; i < ANDI_OUTPUTS; i++){
    for (int j = 0; j < ANDI_NUM_ACT_TOT; j++){
      float eff = positive_non_zero(fabsf(EFF_MAT_G[i][j]));
      float den = positive_non_zero(ratio_u_un[j]*ratio_vn_v[i]);
      float u_range = MAX_PPRZ - u_trim[j];
      eff = (eff / den) * u_range;
      n_array[i] += eff;
      m_array[j] += eff; 
    }
  }
  for (int i = 0; i < ANDI_OUTPUTS; i++){
    for (int j = 0; j < ANDI_NUM_ACT_TOT; j++){
      float eff = positive_non_zero(fabsf(EFF_MAT_G[i][j]));
      float den = positive_non_zero(ratio_u_un[j]*ratio_vn_v[i]);
      float u_range = MAX_PPRZ - u_trim[j];
      eff = (eff / den) * u_range;
      float eff2 = eff * eff;
      coupling_factor[i] += eff2 / (n_array[i] * m_array[j]);
    }
  }
  Bound(coupling_factor[5],0.2,0.8);
  //printf("Coupling factor: %f %f %f %f %f %f \n", coupling_factor[0], coupling_factor[1], coupling_factor[2], coupling_factor[3], coupling_factor[4], coupling_factor[5]);
  //printf("Axis effectiven: %f %f %f %f %f %f \n", n_array[0], n_array[1], n_array[2], n_array[3], n_array[4], n_array[5]);
}

void oneloop_calc_model_disturbance(bool in_flight){
  int8_t i;
  int8_t j;
  for(i=0; i<ANDI_NUM_ACT_TOT; i++){
    if (i < ANDI_NUM_ACT){
      update_butterworth_2_low_pass(&u_filt[i],actuator_state_1l[i]);
    } else {
      update_butterworth_2_low_pass(&u_filt[i],oneloop_andi.sta_state.att[i-ANDI_NUM_ACT]);
    }
  }
  // Store the distrubance 
  float k3;
  if(in_flight){
    for (i = 0; i < ANDI_OUTPUTS; i++){ // For loop for prediction of acceleration 
      oneloop_andi_model[i] = 0.0;
      switch (i){
        case (RW_aN):
          k3 = 1.0;//k_pos_e.k3[0];
          break;
        case (RW_aE):
          k3 = 1.0;//k_pos_e.k3[1];
          break;
        case (RW_aD):
          k3 = 1.0;//k_pos_e.k3[2];
          break;
        case (RW_ap):
          k3 = k_att_e.k3[0];
          break;            
        case (RW_aq):
          k3 = k_att_e.k3[1];
          break;      
        case (RW_ar):
          k3 = k_att_e.k3[2];
          break;      
      }
      k3 = positive_non_zero(k3);
      if (i == RW_aN){
        //printf("start oneloop_andi_model             : %f \n",oneloop_andi_model[i]);
      }
      for (j = 0; j < ANDI_NUM_ACT_TOT; j++){
        float den = positive_non_zero(ratio_u_un[j]*ratio_vn_v[i]);
        float num = u_filt[j].o[0] * EFF_MAT_G[i][j];
        oneloop_andi_model[i] += num / den;
      }
      oneloop_andi_model[i] = oneloop_andi_model[i] / k3;
    } 
    update_filter_on_type(&oneloop_andi_model_filt.ax, oneloop_andi_model[RW_aN]);
    update_filter_on_type(&oneloop_andi_model_filt.ay, oneloop_andi_model[RW_aE]);
    update_filter_on_type(&oneloop_andi_model_filt.az, oneloop_andi_model[RW_aD]);
    update_filter_on_type(&oneloop_andi_model_filt.p_dot, oneloop_andi_model[RW_ap]);
    update_filter_on_type(&oneloop_andi_model_filt.q_dot, oneloop_andi_model[RW_aq]);
    update_filter_on_type(&oneloop_andi_model_filt.r_dot, oneloop_andi_model[RW_ar]);
  }else {
    float_vect_zero(oneloop_andi_model, ANDI_OUTPUTS);
  }
} 
void oneloop_andi_bound_disturbance(void){
    oneloop_andi_dist_bound[RW_aN] = oneloop_andi.gui_state.acc[0] - oneloop_andi_model[RW_aN];
    BoundAbs(oneloop_andi_dist_bound[RW_aN], 99999.f); // Large value to not bound
    oneloop_andi_dist_bound[RW_aE] = oneloop_andi.gui_state.acc[1] - oneloop_andi_model[RW_aE];
    BoundAbs(oneloop_andi_dist_bound[RW_aE], 99999.f); // Large value to not bound
    oneloop_andi_dist_bound[RW_aD] = oneloop_andi.gui_state.acc[2] - oneloop_andi_model[RW_aD];
    BoundAbs(oneloop_andi_dist_bound[RW_aD], 99999.f); // Large value to not bound
    oneloop_andi_dist_bound[RW_ap] = oneloop_andi.sta_state.att_2d[0] - oneloop_andi_model[RW_ap];
    BoundAbs(oneloop_andi_dist_bound[RW_ap], 99999.f); // Large value to not bound
    oneloop_andi_dist_bound[RW_aq] = oneloop_andi.sta_state.att_2d[1] - oneloop_andi_model[RW_aq];
    BoundAbs(oneloop_andi_dist_bound[RW_aq], 99999.f); // Large value to not bound
    oneloop_andi_dist_bound[RW_ar] = oneloop_andi.sta_state.att_2d[2] - oneloop_andi_model[RW_ar];
    BoundAbs(oneloop_andi_dist_bound[RW_ar], 99999.f); // Large value to not bound
    //BoundAbs(oneloop_andi_dist_bound[RW_ar], oneloop_andi_yaw_dist_limit); // Bound from INDI controller
}

// void dynFilter_run(struct Oneloop_DynFilt_t *mu, float u_c, float sigma){
//   mu->mu_c_0      = mu->mu_c;
//   mu->u_c_0       = mu->u_c;
//   mu->u_c         = u_c;
//   mu->sigma       = sigma;
//   float c_den     = (mu->fs+mu->sigma)*mu->varepsilon;
//   float c_u_c     = (mu->fs*mu->sigma+mu->varepsilon*mu->sigma)/c_den;
//   float c_u_c_0   = (-mu->fs*mu->sigma)/c_den;
//   float c_mu_c_0  = (mu->fs*mu->varepsilon)/c_den;
//   mu->mu_c        = c_u_c * mu->u_c + c_u_c_0 * mu->u_c_0 + c_mu_c_0 * mu->mu_c_0;
// }
void dynFilter_run(struct Oneloop_DynFilt_t *mu, float u_c, float sigma) {
    mu->mu_c_0 = mu->mu_c;
    mu->u_c_0  = mu->u_c;
    mu->u_c    = u_c;
    mu->sigma  = sigma;

    // Calculate ZOH-based coefficients
    float exp_term = expf(-mu->sigma / mu->fs); // Use expf for float precision
    float c_u_c    = mu->sigma / mu->varepsilon;
    float c_u_c_0  = (-mu->sigma + mu->varepsilon - mu->varepsilon * exp_term) / mu->varepsilon;
    float c_mu_c_0 = exp_term;

    mu->mu_c = c_u_c * mu->u_c + c_u_c_0 * mu->u_c_0 + c_mu_c_0 * mu->mu_c_0;
}


void dynFilter_init(struct Oneloop_DynFilt_t *mu, float varepsilon, float sigma){
  mu->fs         = (float) PERIODIC_FREQUENCY;
  mu->varepsilon = varepsilon;
  mu->sigma      = sigma;
  mu->mu_c       = 0.0;
  mu->u_c        = 0.0;
  mu->mu_c_0     = 0.0;
  mu->u_c_0      = 0.0;
  //printf("DynFilter initialized with varepsilon: %f, sigma: %f, fs: %f\n", mu->varepsilon, mu->sigma, mu->fs);
}
