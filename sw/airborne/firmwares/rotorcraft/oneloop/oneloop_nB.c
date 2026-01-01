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

/** @file "firmwares/rotorcraft/oneloop/oneloop_nB.h"
 * @author Tomaso De Ponti <tmldeponti@tudelft.nl>
 * One loop (Guidance + Stabilization) ANDI controller for rotorcrafts
 */
//====================================================================================================================================
/**
 * @brief  EXPLANATION OF HALFLOOP
 * @param oneloop_nB_half_loop - A Boolean indicating the state of the oneloop controller.
 * @param oneloop_nB_half_loop = true - Control allocation is performed to accommodate desired Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk (ANDI).
 * @param oneloop_nB_half_loop = false - Control allocation is performed to accommodate desired Jerk North, Jerk East, Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk (ANDI).
 */
//====================================================================================================================================
// Enter functions change the state of the oneloop controller
/**
 * @fn stabilization_attitude_enter in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 * @result oneloop_nB_half_loop = true
 */
/**
 * @fn guidance_h_run_enter in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * @result oneloop_nB_half_loop = false
 */
/**
 * @fn guidance_v_run_enter in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 * @result nothing
 */
// Example of execution of the oneloop controller for two different states in the state machine
/**
 * @file "sw/airborne/firmwares/rotorcraft/autopilot_static.c"
 *
 * @if MODE_ATTITUDE_RC_DIRECT
 *
 * - @fn stabilization_attitude_run() in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 *
 * - - @if half_loop
 *
 * - - - @fn oneloop_nB_run(true) in @file "firmwares/rotorcraft/oneloop/oneloop_nB.c"
 *
 * - - - @result: Control allocation is performed to accommodate desired Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk from stick inputs
 * - - @endif
 *
 * @elseif MODE_NAV
 *
 * - @fn guidance_h_run() in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 *
 * - @fn oneloop_nB_run(false) in @file "firmwares/rotorcraft/oneloop/oneloop_nB.c"
 *
 * - @result: Control allocation is performed to accommodate desired Jerk North, Jerk East, Jerk Down, Roll Jerk, Pitch Jerk, and Yaw Jerk from navigation outputs
 *
 * - @fn guidance_v_run() in @file "firmwares/rotorcraft/guidance/guidance_oneloop.c"
 *
 * - @result: nothing
 *
 * - @fn stabilization_attitude_run() in @file "firmwares/rotorcraft/stabilization/stabilization_oneloop.c"
 *
 * - @result: nothing because of oneloop_nB_half_loop = false
 * @endif
 */
//====================================================================================================================================
/* Include necessary header files */
#include "firmwares/rotorcraft/oneloop/oneloop_nB.h"
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
#ifndef ONELOOP_NB_SIMPLE_QUAD
#include "modules/rotwing_drone/rotwing_state.h"
#endif
#include "modules/core/commands.h"
// #include "modules/ctrl/eff_scheduling_rotwing_V2.h"
#include "modules/system_identification/sys_id_doublet.h"
#include <stdio.h>
#if INS_EXT_POSE
#include "modules/ins/ins_ext_pose.h"
#endif
//====================================================================================================================================
//====================================================================================================================================
// Define configuration variables with default values if not defined in the airframe file
//====================================================================================================================================
//====================================================================================================================================
// GENERAL VARIABLES 
//====================================================================================================================================
#ifndef ONELOOP_NB_DEBUG_MODE                                             // Debug mode, sets in_flight to FALSE     
#define ONELOOP_NB_DEBUG_MODE  FALSE
#endif

struct OneloopGeneral oneloop_nB;                                         // Define general struct of the Oneloop ANDI controller   
static float          dt_1l = 1./PERIODIC_FREQUENCY;                        // Time step of the oneloop controller [s]   
static float          g = 9.81;                                             // [m/s^2] Gravitational Acceleration

#define IDX_aD RW_aD-2 // Down axis (linear acceleration)
#define IDX_ap RW_ap-2 // // X body axis (angular acceleration)
#define IDX_aq RW_aq-2 // // Y body axis (angular acceleration)
#define IDX_ar RW_ar-2 // // Z body axis (angular acceleration)
//====================================================================================================================================
// FILTERING VARIABLES 
//====================================================================================================================================
#define USE_BW2

#ifdef ONELOOP_NB_FILT_CUTOFF
float oneloop_nB_filt_cutoff = ONELOOP_NB_FILT_CUTOFF;
#else
float oneloop_nB_filt_cutoff = 2.0;
#endif

#ifdef ONELOOP_NB_FILT_CUTOFF_ACC
float oneloop_nB_filt_cutoff_a = ONELOOP_NB_FILT_CUTOFF_ACC;
#else
float oneloop_nB_filt_cutoff_a = 2.0;
#endif

#ifdef ONELOOP_NB_FILT_CUTOFF_VEL
float oneloop_nB_filt_cutoff_v = ONELOOP_NB_FILT_CUTOFF_VEL;
#else
float  oneloop_nB_filt_cutoff_v = 30.0;
#endif

#ifdef ONELOOP_NB_FILT_CUTOFF_P
#define ONELOOP_NB_FILTER_ROLL_RATE TRUE
float oneloop_nB_filt_cutoff_p = ONELOOP_NB_FILT_CUTOFF_P;
#else
float oneloop_nB_filt_cutoff_p = 20.0;
#endif

#ifdef ONELOOP_NB_FILT_CUTOFF_Q
#define ONELOOP_NB_FILTER_PITCH_RATE TRUE
float oneloop_nB_filt_cutoff_q = ONELOOP_NB_FILT_CUTOFF_Q;
#else
float oneloop_nB_filt_cutoff_q = 20.0;
#endif

#ifdef ONELOOP_NB_FILT_CUTOFF_R
#define ONELOOP_NB_FILTER_YAW_RATE TRUE
float oneloop_nB_filt_cutoff_r = ONELOOP_NB_FILT_CUTOFF_R;
#else
float oneloop_nB_filt_cutoff_r = 20.0;
#endif

PRINT_CONFIG_MSG("============================================================")
PRINT_CONFIG_MSG("%%% ONELOOP FILTERING VARIABLES %%%")
PRINT_CONFIG_VAR(ONELOOP_NB_FILT_CUTOFF_ACC)
PRINT_CONFIG_VAR(ONELOOP_NB_FILT_CUTOFF_VEL)
PRINT_CONFIG_VAR(ONELOOP_NB_FILT_CUTOFF);
PRINT_CONFIG_VAR(ONELOOP_NB_FILT_CUTOFF_Q);
PRINT_CONFIG_VAR(ONELOOP_NB_FILT_CUTOFF_P);
PRINT_CONFIG_VAR(ONELOOP_NB_FILT_CUTOFF_R);
PRINT_CONFIG_MSG("============================================================")
PRINT_CONFIG_MSG("============================================================")
PRINT_CONFIG_MSG("%%% ONELOOP GENERAL VARIABLES %%%")
PRINT_CONFIG_VAR(ANDI_NUM_ACT)
PRINT_CONFIG_VAR(ANDI_NUM_VIRTUAL_ACT)
PRINT_CONFIG_VAR(ANDI_NUM_ACT_TOT)
PRINT_CONFIG_VAR(ANDI_OUTPUTS)
PRINT_CONFIG_MSG("============================================================")
struct Oneloop_LP_t         LP;

#define USE_PID_NUMDIFF_DERIVATIVE
#ifdef USE_PID_NUMDIFF_DERIVATIVE
static Butterworth2LowPass  filt_veloc[3];
#endif
static Butterworth2LowPass  accely_filt;                  // FIXME (check if condenseable) Low pass filter for acceleration in y direction   
static Butterworth2LowPass  airspeed_filt;                // FIXME (check if condenseable) Low pass filter for airspeed                            
static Butterworth2LowPass  u_filt[ANDI_NUM_ACT_TOT];     // FIXME (check if condenseable) Low pass filter for actuators for synchronous filtering      
static Butterworth2LowPass  nB_filt[3];
static Butterworth2LowPass  nB_2d_filt[3];
//====================================================================================================================================
// ACTUATOR VARIABLES 
//====================================================================================================================================
#ifndef ONELOOP_NB_NUM_THRUSTERS                                              // Number of motors used for thrust
float num_thrusters_oneloop = 4.0; 
#else
float num_thrusters_oneloop = ONELOOP_NB_NUM_THRUSTERS;
#endif

#ifdef ONELOOP_NB_ACT_IS_SERVO                                                // Unused but kept for future developments
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = ONELOOP_NB_ACT_IS_SERVO;
#else
bool actuator_is_servo[ANDI_NUM_ACT_TOT] = {0};
#endif

#ifdef ONELOOP_NB_ACT_DYN                                                     // Actuator dynamics (first order lag) corner frequency [rad/s]
float  act_dynamics[ANDI_NUM_ACT_TOT] = ONELOOP_NB_ACT_DYN;
float  act_dyn_ctrl[ANDI_NUM_ACT_TOT] = ONELOOP_NB_ACT_DYN;
#else
#error "You must specify the actuator dynamics"
float act_dynamics[ANDI_NUM_ACT_TOT] = = {1};
float act_dyn_ctrl[ANDI_NUM_ACT_TOT] = = {1};
#endif

#ifdef ONELOOP_NB_ACT_MAX                                                     // Maximum Paparazzi actuator command 
float  act_max[ANDI_NUM_ACT_TOT] = ONELOOP_NB_ACT_MAX;
#else
float act_max[ANDI_NUM_ACT_TOT] = = {MAX_PPRZ};
#endif

#ifdef ONELOOP_NB_ACT_MIN                                                     // Minimum Paparazzi actuator command
float  act_min[ANDI_NUM_ACT_TOT] = ONELOOP_NB_ACT_MIN;
#else
float act_min[ANDI_NUM_ACT_TOT] = = {0.0};
#endif

#ifdef ONELOOP_NB_ACT_MAX_NORM                                                // Maximum normalized actuator command 
float  act_max_norm[ANDI_NUM_ACT_TOT] = ONELOOP_NB_ACT_MAX_NORM;
#else
float act_max_norm[ANDI_NUM_ACT_TOT] = = {1.0};
#endif

#ifdef ONELOOP_NB_ACT_MIN_NORM                                               // Minimum normalized actuator command
float  act_min_norm[ANDI_NUM_ACT_TOT] = ONELOOP_NB_ACT_MIN_NORM;
#else
float act_min_norm[ANDI_NUM_ACT_TOT] = = {0.0};
#endif

#ifdef ONELOOP_NB_NU_NORM_MAX                                                // Maximum norm of the normalized pseudo control vector
float  nu_norm_max = ONELOOP_NB_NU_NORM_MAX;
#else
float nu_norm_max = 1.0;
#endif

#ifdef ONELOOP_NB_U_PREF                                                     // Preferred Paparazzi actuator command 
static float u_pref[ANDI_NUM_ACT_TOT] = ONELOOP_NB_U_PREF;
#else
static float u_pref[ANDI_NUM_ACT_TOT] = {0.0};
#endif

#define ONELOOP_NB_MAX_BANK  M_PI_6                           // Max Bank (can be different from Max Roll) assuming abs of max and min is the same
#define ONELOOP_NB_MAX_PHI   M_PI_6                           // Max Roll assuming abs of max and min is the same
#define ONELOOP_NB_MAX_THETA M_PI_6                           // Max Pitch assuming abs of max and min is the same

#ifndef ONELOOP_THETA_PREF_MAX                                                  // Max preferred pitch angles
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

float         andi_u[ANDI_NUM_ACT_TOT];                                         // Control command vector   
float         andi_du[ANDI_NUM_ACT_TOT];                                        // Control command increment vector
static float  andi_u_n[ANDI_NUM_ACT_TOT];                                       // Control command vector (normalized)
float         nu[ANDI_OUTPUTS];                                                 // Pseudo control vector
float         nu_n[ANDI_OUTPUTS];                                               // Pseudo control vector (normalized)
static float  act_dynamics_d[ANDI_NUM_ACT_TOT];                                 // Actuator dynamics (first order lag) corner frequency [rad/s]
float         actuator_state_1l[ANDI_NUM_ACT_TOT];                              // Actuator state vector (including virtual actuators)
float         nB_jerk_des[3];
float SQ_r = 0.0; 
float max_pitch_mot = 3000.0;
//====================================================================================================================================
// STABILIZATION VARIABLES
//====================================================================================================================================
#ifndef MAX_R                                                                     // Max Yaw rate with the sticks  
float max_r = RadOfDeg(120.0);
#else
float max_r = RadOfDeg(MAX_R);
#endif

#ifdef ONELOOP_NB_YAW_DISTURBANCE_LIMIT                                         // Yaw disturbance rejection limit     
float oneloop_nB_yaw_dist_limit = ONELOOP_NB_YAW_DISTURBANCE_LIMIT;
#else 
float oneloop_nB_yaw_dist_limit = 99999.f;                                      // High value to disable the feature
#endif

#if ONELOOP_NB_HEADING_MANUAL                                                   // Specify heading in NAV with a slider
bool heading_manual = true;
#else
bool heading_manual = false;
#endif

#if ONELOOP_NB_YAW_STICK_IN_AUTO                                                // Specify heading in NAV with the yaw stick
bool yaw_stick_in_auto = true;
#else
bool yaw_stick_in_auto = false;
#endif

struct OneloopStabilizationRef sta_bounds = {                                     // Stabilization bounds specified per axis
  #ifdef ONELOOP_NB_MAX_ANGULAR_JERK
  .att_3d[0] = ONELOOP_NB_MAX_ANGULAR_JERK,
  .att_3d[1] = ONELOOP_NB_MAX_ANGULAR_JERK,
  #else
  .att_3d[0] = RadOfDeg(100000.0),
  .att_3d[1] = RadOfDeg(100000.0),
  #endif

#ifdef ONELOOP_NB_MAX_ANGULAR_JERK_YAW
    .att_3d[2] = ONELOOP_NB_MAX_ANGULAR_JERK_YAW,
#else
    .att_3d[2] = RadOfDeg(1520.0),
#endif

  #ifdef ONELOOP_NB_MAX_ANGULAR_ACCEL
  .att_2d[0] = ONELOOP_NB_MAX_ANGULAR_ACCEL,
  .att_2d[1] = ONELOOP_NB_MAX_ANGULAR_ACCEL,
  #else
  .att_2d[0] = RadOfDeg(700.0),
  .att_2d[1] = RadOfDeg(480.0),
  #endif

#ifdef ONELOOP_NB_MAX_ANGULAR_ACCEL_YAW
    .att_2d[2] = ONELOOP_NB_MAX_ANGULAR_ACCEL_YAW,
#else
    .att_2d[2] = RadOfDeg(130.0),
#endif

#ifdef ONELOOP_NB_MAX_ANGULAR_VEL
    .att_d[0] = ONELOOP_NB_MAX_ANGULAR_VEL,
    .att_d[1] = ONELOOP_NB_MAX_ANGULAR_VEL,
#else
    .att_d[0] = RadOfDeg(10000.0),
    .att_d[1] = RadOfDeg(10000.0),
#endif

#ifdef ONELOOP_NB_MAX_ANGULAR_VEL_YAW
    .att_d[2] = ONELOOP_NB_MAX_ANGULAR_VEL_YAW,
#else
    .att_d[2] = RadOfDeg(30.0),
#endif
};

struct FloatEulers  eulers_zxy_des;                                               // Desired Euler angles in ZXY sequence
struct FloatEulers  eulers_zxy;                                                   // Actual Euler angles in ZXY sequence
float               psi_des_rad = 0.0;                                            // Desired Yaw angle [rad]
float               psi_des_deg = 0.0;                                            // Desired Yaw angle [deg]
static float        psi_vec[4]  = {0.0, 0.0, 0.0, 0.0};                           // Vector to overwrite the Yaw reference in the attitude controller
//====================================================================================================================================
// GUIDANCE VARIABLES 
//====================================================================================================================================
#ifndef ONELOOP_NB_AIRSPEED_SWITCH_THRESHOLD                                    // Airspeed threshold to switch to coordinated turn behavior    
#define ONELOOP_NB_AIRSPEED_SWITCH_THRESHOLD 10.0
#endif

#ifndef FWD_SIDESLIP_GAIN                                                         // Gain to reduce sideslip when flying forward 
float fwd_sideslip_gain = 0.2;
#else
float fwd_sideslip_gain = FWD_SIDESLIP_GAIN;
#endif

#ifdef NAV_HYBRID_MAX_DECELERATION                                                // Max (ac)/(de)celeration. Can be overwritte nby NAV HYBRID   
float max_a_nav = NAV_HYBRID_MAX_DECELERATION;
#else
float max_a_nav = 4.0;  
#endif

#ifdef ONELOOP_NB_MAX_LINEAR_JERK                                               // Max linear jerk [m/s^3]    
float max_j_lin = ONELOOP_NB_MAX_LINEAR_JERK;
#else
float max_j_lin = 500.0; 
#endif

#ifdef NAV_HYBRID_MAX_AIRSPEED
float max_v_nav = NAV_HYBRID_MAX_AIRSPEED;                                        // Max horizontal speed. Can be overwritten by NAV HYBRID
#else
float max_v_nav = 5.0;
#endif

#ifdef NAV_HYBRID_MAX_SPEED_V                                                     // Max vertical speed
float max_v_nav_v = NAV_HYBRID_MAX_SPEED_V;
#else
float max_v_nav_v = 1.5;
#endif

#define USE_ND_VELOCITIES                                                         // Use NumDiff of velocities instead of Accelerometer.

float         max_as = 19.0f;                                                     // Max airspeed [m/s]    
float         min_as = 0.0f;                                                      // Min airspeed [m/s] 
float         xdot_lim_sf = 0.8;                                                  // Safety Factor on SUVAT velocity limit
float         ec_headroom = 1.5;                                                  // Extra headroom for the EC wrt the RM  
static float  nav_target[3];                                                      // Can be a position, speed or acceleration depending on the guidance H mode
static float  nav_target_new[3];                                                  // Wind triangle reshaped NAV target
float         gi_unbounded_airspeed_sp = 0.0;                                     // Unbounded airspeed setpoint [m/s] (mimics guidance_indi_hybrid)                                          
//====================================================================================================================================
// CONTROL ALLOCATION VARIABLES
//====================================================================================================================================
#ifndef ONELOOP_NB_WU_QUAD_MOTORS_FWD
float Wu_quad_motors_fwd = 6.0;
#else
float Wu_quad_motors_fwd = ONELOOP_NB_WU_QUAD_MOTORS_FWD;
#endif

struct WLS_t WLS_one_p = {
  .nu        = ANDI_NUM_ACT_TOT,
  .nv        = ANDI_OUTPUTS,
  .gamma_sq  = 1000.0,
  .v         = {0.0},
#ifdef ONELOOP_NB_WV // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}  
  .Wv        = ONELOOP_NB_WV,
#else
  .Wv        = {1.0},
#endif
#ifdef ONELOOP_NB_WU // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}  
  .Wu        = ONELOOP_NB_WU,
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

#ifdef ONELOOP_NB_WU // {mF,mR,mB,mL,mP,de,dr,da,df,phi,theta}
static float Wu_backup[ANDI_NUM_ACT_TOT] = ONELOOP_NB_WU;
#else
static float Wu_backup[ANDI_NUM_ACT_TOT] = {1.0};
#endif

#ifdef ONELOOP_NB_WV // {mF,mR,mB,mL,mP,de,dr,da,df,phi,theta}
static float Wv_backup[ANDI_OUTPUTS] = ONELOOP_NB_WV;
#else
static float Wv_backup[ANDI_OUTPUTS] = {1.0};
#endif

static float a_thrust = 0.0;                                                      // Virtual z-Acceleration to command a Thrust level[m/s^2]
static float g2_ff = 0.0;                                                         // Feedforward G2 term for Yaw control
bool         ctrl_off = false;                                                      // Preferred pitch angle for the control allocation

float         *bwls_1l[ANDI_OUTPUTS];                                             // Pointer to the WLS Effectiveness matrix  
float         EFF_MAT_G[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];                          // Intermidiate variable to save Effectiveness matrix 
float         n_array[ANDI_OUTPUTS];
float         m_array[ANDI_NUM_ACT_TOT];
float         ctrl_effort_model[ANDI_OUTPUTS];
float         ratio_u_un[ANDI_NUM_ACT_TOT];
float         ratio_vn_v[ANDI_OUTPUTS];
//====================================================================================================================================
// CONTROL LAW VARIABLES
//====================================================================================================================================
float         SpinQuadRate  =0.0;
bool          SpinQuad              = false;                                      // Quadrotor spinning configuration
bool          fault_pitch           = false;
bool          drop_yaw              = false;                                      // Drop the control of the Yaw axis
bool          drop_roll             = false;                                      // Drop the control of the Roll axis
bool          drop_pitch            = false;                                     // Drop the control of the aE axis
bool          drop_aD               = false;                                      // Drop the control of the aD axis
bool          state_compensation_on = false;                                      // State compensation for rotating bodies
//====================================================================================================================================
// Error Controller and Reference Model VARIABLES
//====================================================================================================================================
/*Declaration of Reference Model and Error Controller Gains*/
struct PolePlacement p_att_e;
struct PolePlacement p_roll_e;
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
float k1_NE_tune = 0.6;
float k2_NE_tune = 1.85;
/* PID */
float k_P = 0.6;
float k_D = 1.2;
float k_I = 9.0e-4;
float temp_P_error[3];
float temp_D_error[3];
float temp_I_error[3];
//====================================================================================================================================
// Function Declaration Section
//====================================================================================================================================
void  init_poles(void);
void  init_poles_att(void); // FIXME check if still needed 
void  init_poles_pos(void); // FIXME check if still needed 
void  calc_normalization(void);
void  normalize_nu(void);
void  G1G2_oneloop(int ctrl_type);
void  get_act_state_oneloop(void);
void  oneloop_nB_propagate_filters(void);
void  init_filter(void);
void  init_controller_gains(void);
void  reinit_controller(void);
void  float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3]);
void  float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3]);
void  err_nd(float err[], float a[], float b[], float k[], int n);
void  err_sum_nd(float err[], float a[], float b[], float k[], float c[], int n);
void  float_vect_diff_euler(float err[3], float a[3], float b[3]);
void  integrate_nd(float dt, float a[], float a_dot[], int n);
void  vect_bound_nd(float vect[], float bound, int n);
void  acc_body_bound(struct FloatVect2* vect, float bound);
float bound_v_from_a_vect(float e_x[], float v_bound, float a_bound, int n);
float bound_v_from_a(float e_x, float v_bound, float a_bound);
void  rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3], struct OneloopStabilizationRef bounds);
void  rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n);
void  rm_2nd_pos(float dt, float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_d_des[], float k2_rm[], float k3_rm[], float x_2d_bound, float x_3d_bound, int n);
void  rm_1st_pos(float dt, float x_2d_ref[], float x_3d_ref[], float x_2d_des[], float k3_rm[], float x_3d_bound, int n);
void  rm_3rd_nI(float dt, struct FloatVect3 *x_ref, struct FloatVect3 *x_d_ref, struct FloatVect3 *x_2d_ref, struct FloatVect3 *x_3d_ref, const struct FloatVect3 *x_des, const float k1_rm[3], const float k2_rm[3], const float k3_rm[3]);
void  ec_3rd_att(float y_4d[3], float x_des[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3], struct OneloopStabilizationRef bounds, float fb[3]);
void  ec_3rd_pos(float y_4d[], float x_des[], float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x[], float x_d[], float x_2d[], float k1_e[], float k2_e[], float k3_e[], float x_d_bound, float x_2d_bound, float fb[], int n);
float oneloop_nB_sideslip(void);
void  reshape_wind(void);
void  chirp_pos(float time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]);
void  chirp_call(bool* chirp_on, bool* chirp_first_call, float* t_0_chirp, float* time_elapsed, float f0, float f1, float t_chirp, float A, int8_t n, float psi, float p_ref[], float v_ref[], float a_ref[], float j_ref[], float p_ref_0[]);
void  oneloop_calc_model_disturbance(bool in_flight);
void  oneloop_nB_state_compensation(bool state_compensation_on);
void  set_WLS_settings(void);
void  drop_axis(void);
void  skew_symmetric(struct FloatRMat *out, const struct FloatVect3 *in);
void  oneloop_nB_calc_nB_states(void);
void  nB_EC(struct FloatVect3 nB, struct FloatVect3 nB_d, struct FloatVect3 nB_2d, struct FloatVect3 mu_B, float k1_e[3], float k2_e[3], float k3_e[3], float dist[3], float nB_nu[3]);
void  calc_HB_matrix(float HB[3][3], struct FloatVect3 nB);
void  SpinQuad_overwrite(float gain, float ce_model, float *nu_stab_2);
//====================================================================================================================================
// Telemetry Section
//====================================================================================================================================
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
static void send_eff_mat_stab_oneloop_nB(struct transport_tx *trans, struct link_device *dev)
{
#define STREAM_BWLS  
#ifdef STREAM_BWLS
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID, 
                ANDI_NUM_ACT, bwls_1l[1],
                ANDI_NUM_ACT, bwls_1l[2],
                ANDI_NUM_ACT, bwls_1l[3], 
                ANDI_NUM_ACT, bwls_1l[0],
                ANDI_NUM_ACT, G2_RW);
#else
  float zero = 0.0;
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID, 
                ANDI_NUM_ACT, EFF_MAT_RW[3],
                ANDI_NUM_ACT, EFF_MAT_RW[4],
                ANDI_NUM_ACT, EFF_MAT_RW[5], 
                           1, &zero,
                ANDI_NUM_ACT, G2_RW);
#endif                
}
static void send_eff_mat_guid_oneloop_nB(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_EFF_MAT_GUID(trans, dev, AC_ID,
                             ANDI_NUM_ACT_TOT, EFF_MAT_RW[0],
                             ANDI_NUM_ACT_TOT, EFF_MAT_RW[1],
                             ANDI_NUM_ACT_TOT, EFF_MAT_RW[2]);
}
static void send_oneloop_nB(struct transport_tx *trans, struct link_device *dev)
{
  float temp_eulers_zxy_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                              3, temp_eulers_zxy_des,
                              3, oneloop_nB.sta_state.att,
                              3, oneloop_nB.sta_ref.att,
                              3, oneloop_nB.sta_state.att_d,
                              3, oneloop_nB.sta_ref.att_d,
                              3, oneloop_nB.sta_state.att_2d,
                              3, oneloop_nB.sta_ref.att_2d,
                              3, oneloop_nB.sta_ref.att_3d,
                              ANDI_NUM_ACT, actuator_state_1l);
}
static void send_oneloop_nB_ctrl(struct transport_tx *trans, struct link_device *dev)
{
  float nI_des[3] = {oneloop_nB.sta_nB_state.nI_des.x, oneloop_nB.sta_nB_state.nI_des.y, oneloop_nB.sta_nB_state.nI_des.z};
  float nI[3]     = {oneloop_nB.sta_nB_state.nI.x,     oneloop_nB.sta_nB_state.nI.y,     oneloop_nB.sta_nB_state.nI.z};
  float nI_d[3]   = {oneloop_nB.sta_nB_state.nI_d.x,   oneloop_nB.sta_nB_state.nI_d.y,   oneloop_nB.sta_nB_state.nI_d.z};
  float nI_2d[3]  = {oneloop_nB.sta_nB_state.nI_2d.x,  oneloop_nB.sta_nB_state.nI_2d.y,  oneloop_nB.sta_nB_state.nI_2d.z};
  float nI_3d[3]  = {oneloop_nB.sta_nB_state.nI_3d.x,  oneloop_nB.sta_nB_state.nI_3d.y,  oneloop_nB.sta_nB_state.nI_3d.z};
  float nB[3]     = {oneloop_nB.sta_nB_state.nB.x,     oneloop_nB.sta_nB_state.nB.y,     oneloop_nB.sta_nB_state.nB.z};
  float nB_d[3]   = {oneloop_nB.sta_nB_state.nB_d.x,   oneloop_nB.sta_nB_state.nB_d.y,   oneloop_nB.sta_nB_state.nB_d.z};
  float nB_2d[3]  = {oneloop_nB.sta_nB_state.nB_2d.x,  oneloop_nB.sta_nB_state.nB_2d.y,  oneloop_nB.sta_nB_state.nB_2d.z};
  float mu_B[3]    = {oneloop_nB.sta_nB_state.mu_B.x,    oneloop_nB.sta_nB_state.mu_B.y,    oneloop_nB.sta_nB_state.mu_B.z};
  pprz_msg_send_NB_CTRL(trans, dev, AC_ID,
                              3, nI_des,
                              3, nI,
                              3, nI_d,
                              3, nI_2d,
                              3, nI_3d,
                              3, nB,
                              3, nB_d,
                              3, nB_2d,
                              3, mu_B);
}
static void send_guidance_oneloop_nB(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE(trans, dev, AC_ID,
                         &oneloop_nB.gui_ref.pos[0],
                         &oneloop_nB.gui_ref.pos[1],
                         &oneloop_nB.gui_ref.pos[2],
                         &oneloop_nB.gui_state.pos[0],
                         &oneloop_nB.gui_state.pos[1],
                         &oneloop_nB.gui_state.pos[2],
                         &oneloop_nB.gui_ref.vel[0],
                         &oneloop_nB.gui_ref.vel[1],
                         &oneloop_nB.gui_ref.vel[2],
                         &oneloop_nB.gui_state.vel[0],
                         &oneloop_nB.gui_state.vel[1],
                         &oneloop_nB.gui_state.vel[2],
                         &oneloop_nB.gui_ref.acc[0],
                         &oneloop_nB.gui_ref.acc[1],
                         &oneloop_nB.gui_ref.acc[2],
                         &oneloop_nB.gui_state.acc[0],
                         &oneloop_nB.gui_state.acc[1],
                         &oneloop_nB.gui_state.acc[2],
                         &oneloop_nB.gui_ref.jer[0],
                         &oneloop_nB.gui_ref.jer[1],
                         &oneloop_nB.gui_ref.jer[2]);
}
static void debug_vect(struct transport_tx *trans, struct link_device *dev, char *name, float *data, int datasize)
{
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID,
                           strlen(name), name,
                           datasize, data);
}
static void send_oneloop_debug(struct transport_tx *trans, struct link_device *dev)
{
  float temp_debug_vect[23];
  temp_debug_vect[0]  = temp_P_error[0];
  temp_debug_vect[1]  = temp_P_error[1];
  temp_debug_vect[2]  = temp_P_error[2];
  temp_debug_vect[3]  = temp_D_error[0];
  temp_debug_vect[4]  = temp_D_error[1];
  temp_debug_vect[5]  = temp_D_error[2];
  temp_debug_vect[6]  = temp_I_error[0];
  temp_debug_vect[7]  = temp_I_error[1];
  temp_debug_vect[8]  = temp_I_error[2]; 
  temp_debug_vect[9]  = nB_jerk_des[0];
  temp_debug_vect[10] = nB_jerk_des[1];
  temp_debug_vect[11] = nB_jerk_des[2]; 
  temp_debug_vect[12] = ctrl_effort_model[IDX_aD];
  temp_debug_vect[13] = ctrl_effort_model[IDX_ap];
  temp_debug_vect[14] = ctrl_effort_model[IDX_aq];
  temp_debug_vect[15] = ctrl_effort_model[IDX_ar];
  temp_debug_vect[16] = SQ_r;
  temp_debug_vect[17] = LP.p.meas;    
  temp_debug_vect[18] = LP.q.meas;    
  temp_debug_vect[19] = LP.r.meas;    
  temp_debug_vect[20] = LP.p_dot.meas;
  temp_debug_vect[21] = LP.q_dot.meas;
  temp_debug_vect[22] = LP.r_dot.meas;
  debug_vect(trans, dev, "APF", temp_debug_vect, 23);
}
#endif
//====================================================================================================================================
// Functions Definition Section
//====================================================================================================================================
// General Mathematical Functions
/** @brief Function to make sure that inputs are positive non zero vaues*/
static float positive_non_zero(float input)
{
  if (input < FLT_EPSILON)
  {
    input = 0.00001;
  }
  return input;
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
/** @brief Calculate Error between two 3D euler arrays with the psi normalized*/
void float_vect_diff_euler(float err[3], float a[3], float b[3])
{
  err[0] = (a[0] - b[0]);
  err[1] = (a[1] - b[1]);
  err[2] = (a[2] - b[2]);
  NormRadAngle(err[2]);
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
//====================================================================================================================================
// Reference Model Gain Design Functions
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

// 3-axis position PID controller

static void Pos_PID(float x_des[3], float x[3],float x_dot[3], float k_P, float k_I, float k_D,float a[3])
{
    static float prev_pos[3]  = {0.0f, 0.0f, 0.0f};
    static float integral[3]  = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++)
    {
        float err = x_des[i] - x[i];
        integral[i] += err * dt_1l;
        // Optional anti-windup
        const float I_MAX = 0.4f;
        if (integral[i] >  I_MAX) integral[i] =  I_MAX;
        if (integral[i] < -I_MAX) integral[i] = -I_MAX;
#ifdef USE_PID_NUMDIFF_DERIVATIVE
        // Numerical derivative  
        //static float prev_err[3]  = {0.0f, 0.0f, 0.0f};
        //float derivative = (err - prev_err[i]) / dt_1l; 
        //prev_err[i] = err;
        float derivative_nd = (x[i] - prev_pos[i]) / dt_1l;  
        prev_pos[i] = x[i];     
        update_butterworth_2_low_pass(&filt_veloc[i], derivative_nd);   
        float derivative = -filt_veloc[i].o[0];  
#else
        float derivative = -x_dot[i];
#endif

        // Temp debug
        temp_P_error[i] = k_P * err;
        temp_D_error[i] = k_D * derivative;
        temp_I_error[i] = k_I * integral[i];
        a[i] = k_P * err+ k_I * integral[i]+ k_D * derivative;
    }
    vect_bound_nd(a,0.5*9.81,3);
}
static void shape_vector(float a[3]){
  a[2] += -9.81;
  vect_bound_nd(a,1.0,3);
}
static void eul_of_acc(float a[3], float psi){
  float phi_des;
  float theta_des;

  float spsi = sinf(psi);
  float cpsi = cosf(psi);
  float aX = cpsi*a[0]+spsi*a[1];
  float aY =-spsi*a[0]+cpsi*a[1];
  theta_des = asinf(-aX);
  float ctheta_des = cosf(theta_des);
  if (fabs(ctheta_des) < FLT_EPSILON){ctheta_des = FLT_EPSILON;}
  phi_des   = asinf(aY/ctheta_des);
  BoundAbs(phi_des, M_PI_6); // Limit to 30 deg
  BoundAbs(theta_des, M_PI_6); // Limit to 30 deg
  eulers_zxy_des.phi   = phi_des;
  eulers_zxy_des.theta = theta_des;
}
//====================================================================================================================================
// Attitude Conversion Functions
/** @brief Attitude Rates to Euler Conversion Function ZYX */
// void float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3])
// {
//   float sphi = sinf(e[0]);
//   float cphi = cosf(e[0]);
//   float stheta = sinf(e[1]);
//   float ctheta = cosf(e[1]);
//   r[0] = edot[0] - stheta * edot[2];
//   r[1] = cphi * edot[1] + sphi * ctheta * edot[2];
//   r[2] = -sphi * edot[1] + cphi * ctheta * edot[2];
// }

/** @brief Attitude Rates to Euler Conversion Function ZXY */
void float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3])
{
    float sphi   = sinf(e[0]);
    float cphi   = cosf(e[0]);
    float stheta = sinf(e[1]);
    float ctheta = cosf(e[1]);
    r[0] =  ctheta * edot[0] - stheta * cphi * edot[2]; 
    r[1] =  edot[1] + sphi * edot[2];
    r[2] =  stheta * edot[0] + ctheta *cphi * edot[2]; 
}

/** @brief Attitude Euler to Rates Conversion Function ZYX */
// void float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3])
// {
//   float sphi = sinf(e[0]);
//   float cphi = cosf(e[0]);
//   float stheta = sinf(e[1]);
//   float ctheta = cosf(e[1]);
//   if (fabs(ctheta) < FLT_EPSILON)
//   {
//     ctheta = FLT_EPSILON;
//   }
//   edot[0] = r[0] + sphi * stheta / ctheta * r[1] + cphi * stheta / ctheta * r[2];
//   edot[1] = cphi * r[1] - sphi * r[2];
//   edot[2] = sphi / ctheta * r[1] + cphi / ctheta * r[2];
// }

/** @brief Attitude Euler to Rates Conversion Function ZXY */
void float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3])
{
  float sphi = sinf(e[0]);
  float cphi = cosf(e[0]);
  float stheta = sinf(e[1]);
  float ctheta = cosf(e[1]);
  float spsi = sinf(e[2]);
   float cpsi = cosf(e[2]);

  if (fabs(ctheta) < FLT_EPSILON)
  {
    ctheta = FLT_EPSILON;
  }
  edot[0] = r[0] + spsi * stheta / ctheta * r[1] + cpsi * stheta / ctheta * r[2];
  edot[1] = cphi * r[1] - sphi * r[2];
  edot[2] = sphi * r[1] + cphi * r[2];
}
//====================================================================================================================================
// Bounding Functions
/** @brief Scale a 3D array to within a 3D bound */
void acc_body_bound(struct FloatVect2 *vect, float bound)
{
  int n = 2;
  float v[2] = {vect->x, vect->y};
  float sign_v0 = (v[0] > 0.f) ? 1.f : (v[0] < 0.f) ? -1.f                                               : 0.f;
  float sign_v1 = (v[1] > 0.f) ? 1.f : (v[1] < 0.f) ? -1.f                                               : 0.f;
  float norm = float_vect_norm(v, n);
  v[0] = fabsf(v[0]);
  v[1] = fabsf(v[1]);
  norm = positive_non_zero(norm);
  if ((norm - bound) > FLT_EPSILON)
  {
    v[0] = Min(v[0], bound);
    float acc_b_y_2 = bound * bound - v[0] * v[0];
    acc_b_y_2 = positive_non_zero(acc_b_y_2);
    v[1] = sqrtf(acc_b_y_2);
  }
  vect->x = sign_v0 * v[0];
  vect->y = sign_v1 * v[1];
}
/** 
 * @brief Calculate velocity limit based on acceleration limit for a vector
 * From SUVAT we have that v^2 = u^2 + 2*a*s. Assuming a is constant we can calculate the velocity
 * bound given the position error and the acceleration limit. This bounding also assumes that we want to arrive
 * at the waypoint with zero velocity which might be untrue.
 * @param e_x             Error in Position (always wrt the Desired)
 * @param v_bound         Safety bound on velocity
 * @param a_bound         Acceleration limit
 * @param n               Size of vector */
float bound_v_from_a_vect(float e_x[], float v_bound, float a_bound, int n)
{
  float norm = float_vect_norm(e_x, n);
  norm = fmaxf(norm, 1.0);
  float v_bound_a = sqrtf(fabs(2.0 * a_bound * norm * xdot_lim_sf));
  return fminf(v_bound, v_bound_a);
}
/** 
 * @brief Calculate velocity limit based on acceleration limit for a float
 * From SUVAT we have that v^2 = u^2 + 2*a*s. Assuming a is constant we can calculate the velocity
 * bound given the position error and the acceleration limit. This bounding also assumes that we want to arrive
 * at the waypoint with zero velocity which might be untrue.
 * @param e_x             Error in Position (always wrt the Desired)
 * @param v_bound         Safety bound on velocity
 * @param a_bound         Acceleration limit
 * @param n               Size of vector */
float bound_v_from_a(float e_x, float v_bound, float a_bound)
{
  float norm;
  norm = fabsf(e_x);
  //norm = fmaxf(norm, 1.0);
  float v_bound_a  = sqrtf(fabs(2.0 * a_bound * norm * xdot_lim_sf));
  return fminf(v_bound, v_bound_a);
}
//====================================================================================================================================
// Reference Model Functions
/** 
 * @brief Reference Model Definition for 3rd order system with attitude conversion functions
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order (Attitude)
 * @param x_d_ref         Reference signal 2nd order (Angular Rate)
 * @param x_2d_ref        Reference signal 3rd order (Angular Acceleration)
 * @param x_3d_ref        Reference signal 4th order (Angular Jerk)
 * @param x_des           Desired 1st order signal   (Desired Attitude)
 * @param ow_psi          Overwrite psi (for navigation functions) [bool]
 * @param psi_overwrite   Overwrite psi (for navigation functions) [values]
 * @param k1_rm           Reference Model Gain 1st order signal
 * @param k2_rm           Reference Model Gain 2nd order signal
 * @param k3_rm           Reference Model Gain 3rd order signal
 * @param bounds          Bounds for the Reference Model
 */
void rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3], struct OneloopStabilizationRef bounds){
  
  float e_x[3];                // Attitude Error
  float x_d_fw[3];             // Forward Signal Euler Dot
  float x_d_fw_rates[3];       // Forward Signal Angular Rates
  float x_2d_fw[3];            // Forward Signal Angular Acceleration
  float x_3d_fw[3];            // Forward Signal Angular Jerk
  float bounds_att_d[3];       // Bounds on Angular Rate (SUVAT based)
  float x_d_eul_ref[3];        // Euler angle Reference
  // Attitude error --------------------------------------------------------------------------------------------------
  float_vect_diff_euler(e_x, x_des, x_ref);                    // Calculate Attitude Error
  x_d_fw[0] = e_x[0] * k1_rm[0];                               // Calculate Forward Signal Euler Dot
  x_d_fw[1] = e_x[1] * k1_rm[1];                               // Calculate Forward Signal Euler Dot
  x_d_fw[2] = e_x[2] * k1_rm[2];                               // Calculate Forward Signal Euler Dot
  float_rates_of_euler_dot_vec(x_d_fw_rates, x_ref, x_d_fw);   // Convert Euler Dot to Angular Rates
  bounds_att_d[0] =  bound_v_from_a(e_x[0], bounds.att_d[0], bounds.att_2d[0]);
  bounds_att_d[1] =  bound_v_from_a(e_x[1], bounds.att_d[1], bounds.att_2d[1]);
  bounds_att_d[2] =  bound_v_from_a(e_x[2], bounds.att_d[2], bounds.att_2d[2]);
  BoundAbs(x_d_fw_rates[0], bounds_att_d[0]);
  BoundAbs(x_d_fw_rates[1], bounds_att_d[1]);
  BoundAbs(x_d_fw_rates[2], bounds_att_d[2]);
  // Angular Rate error ----------------------------------------------------------------------------------------------
  err_nd(x_2d_fw, x_d_fw_rates, x_d_ref, k2_rm, 3);
  BoundAbs(x_2d_fw[0], bounds.att_2d[0]);
  BoundAbs(x_2d_fw[1], bounds.att_2d[1]);
  BoundAbs(x_2d_fw[2], bounds.att_2d[2]);
  // Angular Acceleration error -----------------------------------------
  err_nd(x_3d_fw, x_2d_fw, x_2d_ref, k3_rm, 3);
  BoundAbs(x_3d_fw[0], bounds.att_3d[0]);
  BoundAbs(x_3d_fw[1], bounds.att_3d[1]);
  BoundAbs(x_3d_fw[2], bounds.att_3d[2]);
  // Angular Jerk Reference ---------------------------------------------
  float_vect_copy(x_3d_ref, x_3d_fw, 3);
  if (ow_psi){x_3d_ref[2] = psi_overwrite[3];}
  // Angular Acceleration Reference -------------------------------------
  integrate_nd(dt, x_2d_ref, x_3d_ref, 3);
  if (ow_psi){x_2d_ref[2] = psi_overwrite[2];}
  // Angular Rate Reference ---------------------------------------------
  integrate_nd(dt, x_d_ref, x_2d_ref, 3);
  if (ow_psi){x_d_ref[2] = psi_overwrite[1];}
  // Attitude Reference ------------------------------------------------
  float_euler_dot_of_rates_vec(x_d_ref, x_ref, x_d_eul_ref);
  integrate_nd(dt, x_ref, x_d_eul_ref, 3);
  if (ow_psi){x_ref[2] = psi_overwrite[0];}
  NormRadAngle(x_ref[2]);
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
void rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n)
{
  float e_x[n];
  float e_x_d[n];
  float e_x_2d[n];
  err_nd(e_x, x_des, x_ref, k1_rm, n);
  float max_x_d = bound_v_from_a_vect(e_x, x_d_bound, x_2d_bound, n);
  vect_bound_nd(e_x, max_x_d, n);
  err_nd(e_x_d, e_x, x_d_ref, k2_rm, n);
  vect_bound_nd(e_x_d, x_2d_bound, n);
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref, e_x_2d, n);
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
void rm_2nd_pos(float dt, float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_d_des[], float k2_rm[], float k3_rm[], float x_2d_bound, float x_3d_bound, int n)
{
  float e_x_d[n];
  float e_x_2d[n];
  err_nd(e_x_d, x_d_des, x_d_ref, k2_rm, n);
  vect_bound_nd(e_x_d, x_2d_bound, n);
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref, e_x_2d, n);
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
void rm_1st_pos(float dt, float x_2d_ref[], float x_3d_ref[], float x_2d_des[], float k3_rm[], float x_3d_bound, int n)
{
  float e_x_2d[n];
  err_nd(e_x_2d, x_2d_des, x_2d_ref, k3_rm, n);
  float_vect_copy(x_3d_ref, e_x_2d, n);
  vect_bound_nd(x_3d_ref, x_3d_bound, n);
  integrate_nd(dt, x_2d_ref, x_3d_ref, n);
}
//====================================================================================================================================
// Error Controller Functions
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
void ec_3rd_pos(float y_4d[], float x_des[], float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x[], float x_d[], float x_2d[], float k1_e[], float k2_e[], float k3_e[], float x_d_bound, float x_2d_bound, float fb[], int n)
{
  float e_x_d[n];
  float e_x_2d[n];
  float e_x_des[n];
  float_vect_diff(e_x_des, x_des, x, n);
  float max_x_d = bound_v_from_a_vect(e_x_des, x_d_bound * ec_headroom, x_2d_bound * ec_headroom, n);
  err_sum_nd(e_x_d, x_ref, x, k1_e, x_d_ref, n);
  vect_bound_nd(e_x_d, max_x_d, n);
  err_sum_nd(e_x_2d, e_x_d, x_d, k2_e, x_2d_ref, n);
  vect_bound_nd(e_x_2d, x_2d_bound * ec_headroom, n);
  // Calculate and bound distrubance --------------------
  float dist[3];
  float_vect_diff(dist, x_2d, fb, 3);
  err_sum_nd(y_4d, e_x_2d, dist, k3_e, x_3d_ref, n);
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
void ec_3rd_att(float y_4d[3], float x_des[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3], struct OneloopStabilizationRef bounds, float fb[3])
{
  float e_x[3];              // Attitude Error on Reference
  float e_x_des[3];          // Attitude Error on Desired
  float x_d_fw[3];           // Forward Signal Euler Dot
  float x_d_fw_rates[3];     // Forward Signal Angular Rates
  float x_d_fw_rates_rm[3];  // Forward Signal Angular Rates plus RM
  float x_2d_fw[3];          // Forward Signal Angular Acceleration
  float bounds_att_d[3];
  // Attitude Error and Heading conversion --------------------------------
  float_vect_diff_euler(e_x,x_ref,x);                    // Calculate the Attitude Error
  x_d_fw[0] = e_x[0]*k1_e[0];                            // Calculate Forward Signal Euler Dot
  x_d_fw[1] = e_x[1]*k1_e[1];                            // Calculate Forward Signal Euler Dot
  x_d_fw[2] = e_x[2]*k1_e[2];                            // Calculate Forward Signal Euler Dot
  float_rates_of_euler_dot_vec(x_d_fw_rates, x, x_d_fw); // Euler dot to rates conversion
  float_vect_sum(x_d_fw_rates_rm, x_d_ref, x_d_fw_rates, 3);// Add body rates reference signal
  float_vect_diff_euler(e_x_des, x_des, x);
  bounds_att_d[0] = bound_v_from_a(e_x_des[0], bounds.att_d[0] * ec_headroom, bounds.att_2d[0] * ec_headroom);
  bounds_att_d[1] = bound_v_from_a(e_x_des[1], bounds.att_d[1] * ec_headroom, bounds.att_2d[1] * ec_headroom);
  bounds_att_d[2] = bound_v_from_a(e_x_des[2], bounds.att_d[2] * ec_headroom, bounds.att_2d[2] * ec_headroom);
  BoundAbs(x_d_fw_rates_rm[0], bounds_att_d[0]);
  BoundAbs(x_d_fw_rates_rm[1], bounds_att_d[1]);
  BoundAbs(x_d_fw_rates_rm[2], bounds_att_d[2]);
  // Angular Rate Error ---------------------------------------------------
  x_2d_fw[0] = (x_d_fw_rates_rm[0] - x_d[0]) * k2_e[0] + x_2d_ref[0];
  x_2d_fw[1] = (x_d_fw_rates_rm[1] - x_d[1]) * k2_e[1] + x_2d_ref[1];
  x_2d_fw[2] = (x_d_fw_rates_rm[2] - x_d[2]) * k2_e[2] + x_2d_ref[2];
  BoundAbs(x_2d_fw[0], bounds.att_2d[0] * ec_headroom);
  BoundAbs(x_2d_fw[1], bounds.att_2d[1] * ec_headroom);
  BoundAbs(x_2d_fw[2], bounds.att_2d[2] * ec_headroom);
  //  Calculate and bound distrubance --------------------------------------
  float dist[3];
  fb[0] = fb[0] / k3_e[0];
  fb[1] = fb[1] / k3_e[1];
  fb[2] = fb[2] / k3_e[2];
  float_vect_diff(dist, x_2d, fb, 3); // The Disturbance is THe difference between the measurment and the Model
  // Here we can bound the disturbance to avoid too large control efforts
  // Example MAX YAW CONTROL EFFORT: BoundAbs(dist[2], oneloop_nB_yaw_dist_limit); 
  // Angular Acceleration Error -------------------------------------------
  err_sum_nd(y_4d, x_2d_fw, dist, k3_e, x_3d_ref, 3);
}

/**
 * @brief Calculate EC poles given RM poles
 * @param p_rm      Reference Model Pole (3 coincident poles)
 * @param slow_pole Pole of the slowest dynamics
 * @param k         EC / RM ratio
 * @param omega_n   Natural Frequency
 */
static float ec_poles(float p_rm, float slow_pole, float k)
{
  p_rm = positive_non_zero(p_rm);
  slow_pole = positive_non_zero(slow_pole);
  k = positive_non_zero(k);
  // float omega_n = (2*p_rm*slow_pole*k)/(3*slow_pole-p_rm);
  float omega_n = (2 * k * p_rm * slow_pole) / (3 * slow_pole - k * p_rm);
  return omega_n;
}

/**
 * @brief Initialize Position of Poles
 *
 */
void init_poles_att(void)
{
  float slow_pole = 22.0;                                          // Pole of the slowest dynamics used in the attitude controller
  p_att_e.omega_n = ec_poles(p_att_rm.omega_n, slow_pole, 1.28);   // k = 1.28;
  p_head_e.omega_n = ec_poles(p_head_rm.omega_n, slow_pole, 1.28); // k = 1.28;
}
void init_poles_pos(void)
{
  float slow_pole = 22.0/3.0;// Pole of the slowest dynamics used in the position controller
  p_pos_e.omega_n = ec_poles(p_pos_rm.omega_n, slow_pole, 1.28); // k = 1.28; 1.0;
  p_alt_e.omega_n = ec_poles(p_alt_rm.omega_n, slow_pole, 1.28); // k = 1.28; 1.0;// 3.0
}

/**
 * @brief Initialize Position of Poles
 *
 */
void init_poles(void)
{

  // Attitude Controller Poles----------------------------------------------------------
  float slow_pole = 22.0; // Pole of the slowest dynamics used in the attitude controller

  p_att_e.omega_n = slow_pole / 3.0;
  p_att_e.zeta = 1.0;
  p_att_e.p3 = p_att_e.omega_n;

  p_roll_e.omega_n = slow_pole/3.0;
  p_roll_e.zeta    = 1.0;
  p_roll_e.p3      = p_roll_e.omega_n;

  p_att_rm.omega_n = p_att_e.omega_n * 0.8;
  p_att_rm.zeta = 1.0;
  p_att_rm.p3 = p_att_rm.omega_n;

  p_head_e.omega_n = slow_pole / 3.0;
  p_head_e.zeta = 1.0;
  p_head_e.p3 = p_head_e.omega_n;

  p_head_rm.omega_n = p_head_e.omega_n * 0.8;
  p_head_rm.zeta = 1.0;
  p_head_rm.p3 = p_head_rm.omega_n;

  // Position Controller Poles----------------------------------------------------------
  slow_pole = 22.0/3.0;// Pole of the slowest dynamics used in the position controller

  p_pos_e.omega_n = 1.19; // slow_pole/3.0;
  p_pos_e.zeta = 0.5;
  p_pos_e.p3 = p_pos_e.omega_n;

  p_pos_rm.omega_n = p_pos_e.omega_n * 0.8;
  p_pos_rm.zeta = 0.5;
  p_pos_rm.p3 = p_pos_rm.omega_n;

  p_alt_e.omega_n = 1.19; // slow_pole/3.0*2.0;
  p_alt_e.zeta = 0.5;
  p_alt_e.p3 = p_alt_e.omega_n;

  p_alt_rm.omega_n = p_alt_e.omega_n * 0.8;
  p_alt_rm.zeta = 0.5;
  p_alt_rm.p3 = p_alt_rm.omega_n;
}

/**
 * @brief Initialize Controller Gains
 * FIXME: Calculate the gains dynamically for transition
 */
void init_controller_gains(void)
{
  /*Register a variable from nav_hybrid. Should be improved when nav hybrid is final.*/
  float max_wind = 20.0;
  max_v_nav = nav_max_speed + max_wind;
  max_a_nav = nav_max_acceleration_sp;
  /*Some calculations in case new poles have been specified*/
  // init_poles_att();
  // init_poles_pos();
  p_att_rm.p3 = p_att_rm.omega_n * p_att_rm.zeta;
  p_pos_rm.p3 = p_pos_rm.omega_n * p_pos_rm.zeta;
  p_alt_rm.p3 = p_alt_rm.omega_n * p_alt_rm.zeta;
  p_head_rm.p3 = p_head_rm.omega_n * p_head_rm.zeta;

  //--ANDI Controller gains --------------------------------------------------------------------------------
  /*Attitude Loop*/
  k_att_e.k1[0] = k_rm_1_3_f(p_roll_e.omega_n, p_roll_e.zeta, p_roll_e.p3); // k_rm_1_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  k_att_e.k2[0] = k_rm_2_3_f(p_roll_e.omega_n, p_roll_e.zeta, p_roll_e.p3); // k_rm_2_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  k_att_e.k3[0] = k_rm_3_3_f(p_roll_e.omega_n, p_roll_e.zeta, p_roll_e.p3); // k_rm_3_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);

  k_att_e.k1[1] = k_rm_1_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3); // k_att_e.k1[0];
  k_att_e.k2[1] = k_rm_2_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3); // k_att_e.k2[0];
  k_att_e.k3[1] = k_rm_3_3_f(p_att_e.omega_n, p_att_e.zeta, p_att_e.p3); // k_att_e.k3[0];

  k_att_rm.k1[0] = k_rm_1_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k2[0] = k_rm_2_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k3[0] = k_rm_3_3_f(p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  k_att_rm.k1[1] = k_att_rm.k1[0];
  k_att_rm.k2[1] = k_att_rm.k2[0];
  k_att_rm.k3[1] = k_att_rm.k3[0];

  /*Heading Loop NAV*/
  k_att_e.k1[2] = k_rm_1_3_f(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  k_att_e.k2[2] = k_rm_2_3_f(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  k_att_e.k3[2] = k_rm_3_3_f(p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);

  k_att_rm.k1[2] = k_rm_1_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  k_att_rm.k2[2] = k_rm_2_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  k_att_rm.k3[2] = k_rm_3_3_f(p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);

  // Temporary overrirde of gains
  k_att_e.k1[0]   = 4.19;
  k_att_e.k2[0]   = 10.01;
  k_att_e.k3[0]   = 22.0;
  k_att_e.k1[1]   = k_att_e.k1[0];
  k_att_e.k2[1]   = k_att_e.k2[0];
  k_att_e.k3[1]   = k_att_e.k3[0];

  //k_att_rm.k1[0]  = 4.19;
  //k_att_rm.k2[0]  = 10.01;
  //k_att_rm.k3[0]  = 22.0;
//
  //k_pos_e.k1[0]  = 0.6539;
  //k_pos_e.k2[0]  = 1.795;
  //k_pos_e.k3[0]  = XXX;

  // Print INNERLOOP ANDI controller gains
  // printf("Attitude RM poles, omega_n: %f, zeta: %f, p3: %f\n", p_att_rm.omega_n, p_att_rm.zeta, p_att_rm.p3);
  // printf("Attitude EC poles, omega_n: %f, zeta: %f, p3: %f\n", p_att_e.omega_n, p_att_e.zeta, p_att_e.p3);
  // printf("Heading  RM poles, omega_n: %f, zeta: %f, p3: %f\n", p_head_rm.omega_n, p_head_rm.zeta, p_head_rm.p3);
  // printf("Heading  EC poles, omega_n: %f, zeta: %f, p3: %f\n", p_head_e.omega_n, p_head_e.zeta, p_head_e.p3);
  // printf("ANDI Attitude RM Gains: %f %f %f\n", k_att_rm.k1[0], k_att_rm.k2[0], k_att_rm.k3[0]);
  // printf("ANDI Attitude EC Gains: %f %f %f\n", k_att_e.k1[0], k_att_e.k2[0], k_att_e.k3[0]);
  // printf("ANDI Heading  RM Gains: %f %f %f\n", k_att_rm.k1[2], k_att_rm.k2[2], k_att_rm.k3[2]);
  // printf("ANDI Heading  EC Gains: %f %f %f\n", k_att_e.k1[2], k_att_e.k2[2], k_att_e.k3[2]);

  /*Position Loop*/
  k_pos_e.k1[0] = k1_NE_tune;//k_rm_1_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3); // 0.595;//
  k_pos_e.k2[0] = k2_NE_tune;//k_rm_2_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3); // 1.190;//
  k_pos_e.k3[0] = k_rm_3_3_f(p_pos_e.omega_n, p_pos_e.zeta, p_pos_e.p3); // 2.380;//
  k_pos_e.k1[1] = k_pos_e.k1[0];
  k_pos_e.k2[1] = k_pos_e.k2[0];
  k_pos_e.k3[1] = k_pos_e.k3[0];

  k_pos_rm.k1[0] = k_rm_1_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3); // 0.595;
  k_pos_rm.k2[0] = k_rm_2_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3); // 1.190;
  k_pos_rm.k3[0] = k_rm_3_3_f(p_pos_rm.omega_n, p_pos_rm.zeta, p_pos_rm.p3); // 2.380;
  k_pos_rm.k1[1] = k_pos_rm.k1[0];
  k_pos_rm.k2[1] = k_pos_rm.k2[0];
  k_pos_rm.k3[1] = k_pos_rm.k3[0];
  nav_hybrid_pos_gain = k_pos_rm.k1[0];
  nav_hybrid_max_bank = ONELOOP_NB_MAX_BANK;

  /*Altitude Loop*/
  k_pos_e.k1[2] = k_rm_1_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3); // 0.595;
  k_pos_e.k2[2] = k_rm_2_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3); // 1.190;
  k_pos_e.k3[2] = 22;                                                    // k_rm_3_3_f(p_alt_e.omega_n, p_alt_e.zeta, p_alt_e.p3); //2.380;

  k_pos_rm.k1[2] = k_rm_1_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3); // 0.595;
  k_pos_rm.k2[2] = k_rm_2_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3); // 1.190;
  k_pos_rm.k3[2] = k_rm_3_3_f(p_alt_rm.omega_n, p_alt_rm.zeta, p_alt_rm.p3); // 2.380;

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
  k_att_e_indi.k1[0]  = k_att_e.k1[0];
  k_att_e_indi.k2[0]  = k_att_e.k2[0];
  k_att_e_indi.k3[0]  = 1.0;
  k_att_e_indi.k1[1]  = k_att_e_indi.k1[0]; 
  k_att_e_indi.k2[1]  = k_att_e_indi.k2[0]; 
  k_att_e_indi.k3[1]  = k_att_e_indi.k3[0]; 

  /*Heading Loop NAV*/
  k_att_e_indi.k1[2]  = k_att_e.k1[2];
  k_att_e_indi.k2[2]  = k_att_e.k2[2];
  k_att_e_indi.k3[2]  = 1.0;
  
  // Print INDI INNERLOOP controller gains
  // printf("INDI Attitude EC gains: %f %f %f\n", k_att_e_indi.k1[0], k_att_e_indi.k2[0], k_att_e_indi.k3[0]);
  // printf("INDI Heading  EC gains: %f %f %f\n", k_att_e_indi.k1[2], k_att_e_indi.k2[2], k_att_e_indi.k3[2]);

  /*Position Loop*/
  k_pos_e_indi.k1[0]  = k_pos_e.k1[0];
  k_pos_e_indi.k2[0]  = k_pos_e.k2[0];
  k_pos_e_indi.k3[0]  = 1.0;
  k_pos_e_indi.k1[1]  = k_pos_e_indi.k1[0];  
  k_pos_e_indi.k2[1]  = k_pos_e_indi.k2[0];  
  k_pos_e_indi.k3[1]  = k_pos_e_indi.k3[0]; 

  /*Altitude Loop*/
  k_pos_e_indi.k1[2]  = k_pos_e.k1[2];
  k_pos_e_indi.k2[2]  = k_pos_e.k2[2];
  k_pos_e_indi.k3[2]  = 1.0;
  
  // Print INDI OUTERLOOP controller gains
  // printf("INDI Position NE EC gains: %f %f %f\n", k_pos_e_indi.k1[0], k_pos_e_indi.k2[0], k_pos_e_indi.k3[0]);
  // printf("INDI Position D  EC gains: %f %f %f\n", k_pos_e_indi.k1[2], k_pos_e_indi.k2[2], k_pos_e_indi.k3[2]);

  //------------------------------------------------------------------------------------------
 }
// -----------------------------------------------------------------------------------------
// Filter Functions ------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

/** @brief Initialize a filter based on its type */
static inline void init_filter_on_type(struct LP_t *filter, float x0)
{
  switch (filter->filter_type)
  {
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
static inline void update_filter_on_type(struct LP_t *filter, float input)
{
  // filter->meas_prev = filter->meas;
  // filter->meas = input;
  switch (filter->filter_type)
  {
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
  case NOTCH:
  {
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
static inline void init_LP(struct LP_t *LP, float fc)
{
  LP->freq = fc;
  LP->freq_set = fc;
  LP->tau = 1 / (2 * M_PI * LP->freq);
#ifdef USE_LP1
  LP->filter_type = LOWPASS_1;
#elif defined(USE_BW2)
  LP->filter_type = BUTTERWORTH_2;
#elif defined(USE_YAW_LP4)
  LP->filter_type = BUTTERWORTH_4;
#endif
  init_filter_on_type(LP, 0.0);
  // init_first_order_low_pass(&LP->meas_filt, LP->tau, 1.0 / PERIODIC_FREQUENCY, 0.0);
  LP->meas = 0.0;
  LP->meas_prev = 0.0;
  LP->out = 0.0;
}
/** @brief Reinitialize Low Pass filter if new frequency setting or if forced */
static inline void reinit_LP_synchronous(struct LP_t *LP, struct LP_t *f, bool reinit)
{
  if (LP->freq != LP->freq_set || reinit)
  {
    LP->freq = LP->freq_set;
    LP->tau = 1 / (2 * M_PI * LP->freq);
    init_filter_on_type(LP, LP->out);
    init_filter_on_type(f, f->out);
  }
}
static inline void reinit_LP(struct LP_t *LP, bool reinit)
{
  if (LP->freq != LP->freq_set || reinit)
  {
    LP->freq = LP->freq_set;
    LP->tau = 1 / (2 * M_PI * LP->freq);
    init_filter_on_type(LP, LP->out);
  }
}

/** @brief  Initialize all Low Pass Filters */
static inline void init_all_LP(void)
{
  init_LP(&LP.ax, 2.0); // oneloop_nB_filt_cutoff_a
  init_LP(&LP.ay, 2.0); // oneloop_nB_filt_cutoff_a
  init_LP(&LP.az, 2.0); // oneloop_nB_filt_cutoff_a
  init_LP(&LP.p_dot, 2.0);
  init_LP(&LP.q_dot, 2.0);
  init_LP(&LP.r_dot, 2.0);
  init_LP(&LP.p, 15.0); // oneloop_nB_filt_cutoff_p
  init_LP(&LP.q, 15.0); // oneloop_nB_filt_cutoff_q
  init_LP(&LP.r, 15.0); // oneloop_nB_filt_cutoff_r
}

/** @brief Reinitialize all the Low Pass Filters */
static inline void reinit_all_LP(bool reinit)
{
  //reinit_LP_synchronous(&LP.ax, &ctrl_effort_model_filt.ax, reinit);
  //reinit_LP_synchronous(&LP.ay, &ctrl_effort_model_filt.ay, reinit);
  //reinit_LP_synchronous(&LP.az, &ctrl_effort_model_filt.az, reinit);
  //reinit_LP_synchronous(&LP.p_dot, &ctrl_effort_model_filt.p_dot, reinit);
  //reinit_LP_synchronous(&LP.q_dot, &ctrl_effort_model_filt.q_dot, reinit);
  //reinit_LP_synchronous(&LP.r_dot, &ctrl_effort_model_filt.r_dot, reinit);
  reinit_LP(&LP.p, reinit);
  reinit_LP(&LP.q, reinit);
  reinit_LP(&LP.r, reinit);
}
//------------------------------------------------------------------------------------------

/** @brief  Initialize the filters */
void init_filter(void)
{
  // Filtering of the velocities
  float tau = 1.0 / (2.0 * M_PI * oneloop_nB_filt_cutoff);
  float tau_v = 1.0 / (2.0 * M_PI * oneloop_nB_filt_cutoff_v);
  float tau_2 = 1.0 / (2.0 * M_PI * 2.0);
  // printf("tau: %f tau_v: %f\n", tau, tau_v);
  // printf("initializing filters\n");
  init_butterworth_2_low_pass(&accely_filt, tau, 1.0 / PERIODIC_FREQUENCY, accely_filt.o[0]);
  init_butterworth_2_low_pass(&airspeed_filt, tau, 1.0 / PERIODIC_FREQUENCY, airspeed_filt.o[0]);
#ifdef USE_PID_NUMDIFF_DERIVATIVE  
  for (int i = 0; i < 3; i++){
    init_butterworth_2_low_pass(&filt_veloc[i], tau_v, 1.0 / PERIODIC_FREQUENCY, filt_veloc[i].o[0]);
  }
#endif  
  for (int i = 0; i < ANDI_NUM_ACT_TOT; i++)
  {
    init_butterworth_2_low_pass(&u_filt[i], tau_2, 1.0 / PERIODIC_FREQUENCY, 0.0);
  }
  for (int i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&nB_filt[i], tau_2, 1.0 / PERIODIC_FREQUENCY, 0.0);
    init_butterworth_2_low_pass(&nB_2d_filt[i], tau_2, 1.0 / PERIODIC_FREQUENCY, 0.0);
  }
}

/** @brief  Propagate the filters */
void oneloop_nB_propagate_filters(void)
{
  reinit_all_LP(false); 
  struct NedCoor_f *veloc = stateGetSpeedNed_f();
  struct FloatRates *body_rates = stateGetBodyRates_f();
#ifdef USE_ND_VELOCITIES  
  static bool vel_inited = false;
  static struct NedCoor_f veloc_prev;
  if (!vel_inited) { veloc_prev = *veloc; vel_inited = true; }
  LP.ax.meas      = (veloc->x - veloc_prev.x) * PERIODIC_FREQUENCY;
  LP.ay.meas      = (veloc->y - veloc_prev.y) * PERIODIC_FREQUENCY;
  LP.az.meas      = (veloc->z - veloc_prev.z) * PERIODIC_FREQUENCY;
  veloc_prev      = *veloc;
#else
  struct NedCoor_f *accel = stateGetAccelNed_f();
  LP.ax.meas      = accel->x;
  LP.ay.meas      = accel->y;
  LP.az.meas      = accel->z;
#endif  
  LP.p.meas_prev  = LP.p.meas;
  LP.q.meas_prev  = LP.q.meas;
  LP.r.meas_prev  = LP.r.meas;
  LP.p.meas       = body_rates->p;
  LP.q.meas       = body_rates->q;
  LP.r.meas       = body_rates->r;
  LP.p_dot.meas   = (LP.p.meas - LP.p.meas_prev) * PERIODIC_FREQUENCY;
  LP.q_dot.meas   = (LP.q.meas - LP.q.meas_prev) * PERIODIC_FREQUENCY;
  LP.r_dot.meas   = (LP.r.meas - LP.r.meas_prev) * PERIODIC_FREQUENCY;
  // Update Filters of Feedbacks
  update_filter_on_type(&LP.ax, LP.ax.meas);
  update_filter_on_type(&LP.ay, LP.ay.meas);
  update_filter_on_type(&LP.az, LP.az.meas);
  update_filter_on_type(&LP.p_dot, LP.p_dot.meas);
  update_filter_on_type(&LP.q_dot, LP.q_dot.meas);
  update_filter_on_type(&LP.r_dot, LP.r_dot.meas);
  update_filter_on_type(&LP.p, LP.p.meas);
  update_filter_on_type(&LP.q, LP.q.meas);
  update_filter_on_type(&LP.r, LP.r.meas);
  // Propagate filter for sideslip correction
  float accely = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  update_butterworth_2_low_pass(&accely_filt, accely);
  float airspeed_meas = stateGetAirspeed_f();
  Bound(airspeed_meas, 0.0, 30.0);
  update_butterworth_2_low_pass(&airspeed_filt, airspeed_meas);
  update_butterworth_2_low_pass(&nB_filt[0], oneloop_nB.sta_nB_state.nB.x);
  update_butterworth_2_low_pass(&nB_filt[1], oneloop_nB.sta_nB_state.nB.y);
  update_butterworth_2_low_pass(&nB_filt[2], oneloop_nB.sta_nB_state.nB.z);
  update_butterworth_2_low_pass(&nB_2d_filt[0], oneloop_nB.sta_nB_state.nB_2d.x);
  update_butterworth_2_low_pass(&nB_2d_filt[1], oneloop_nB.sta_nB_state.nB_2d.y);
  update_butterworth_2_low_pass(&nB_2d_filt[2], oneloop_nB.sta_nB_state.nB_2d.z);

}
//------------------------------------------------------------------------------------------
/** @brief Re-Init function of controller variables */
void reinit_controller(void)
{
  //  Stabilization
  float_vect_copy(oneloop_nB.sta_ref.att, oneloop_nB.sta_state.att, 3);
  float_vect_copy(oneloop_nB.sta_ref.att_d, oneloop_nB.sta_state.att_d, 3);
  float_vect_copy(oneloop_nB.sta_ref.att_2d, oneloop_nB.sta_state.att_2d, 3);
  float_vect_zero(oneloop_nB.sta_ref.att_3d, 3);
  eulers_zxy_des.phi = oneloop_nB.sta_state.att[0];
  eulers_zxy_des.theta = oneloop_nB.sta_state.att[1];
  eulers_zxy_des.psi = oneloop_nB.sta_state.att[2];
  // Guidance
  float_vect_copy(oneloop_nB.gui_ref.pos, oneloop_nB.gui_state.pos, 3);
  float_vect_copy(oneloop_nB.gui_ref.vel, oneloop_nB.gui_state.vel, 3);
  float_vect_copy(oneloop_nB.gui_ref.acc, oneloop_nB.gui_state.acc, 3);
  float_vect_zero(oneloop_nB.gui_ref.jer, 3);
  // nB controller
  oneloop_nB.sta_nB_state.nI.x = 0.0;
  oneloop_nB.sta_nB_state.nI.y = 0.0;
  oneloop_nB.sta_nB_state.nI.z =-1.0;
}
/** @brief Init function of Oneloop ANDI controller  */
void oneloop_nB_init(void)
{
  oneloop_nB.half_loop = true;
  oneloop_nB.ctrl_type = CTRL_ANDI;
  init_poles();
  // Make sure that the dynamics are positive and non-zero
  for (int8_t i = 0; i < ANDI_NUM_ACT_TOT; i++){act_dynamics[i] = positive_non_zero(act_dynamics[i]);}
  // Initialize Effectiveness matrix
  calc_normalization();
  G1G2_oneloop(oneloop_nB.ctrl_type);
  // Initialize filters and other variables
  init_all_LP();
  init_filter();
  init_controller_gains();
  float_vect_zero(andi_u, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_u_n, ANDI_NUM_ACT_TOT);
  float_vect_zero(actuator_state_1l, ANDI_NUM_ACT_TOT);
  float_vect_zero(oneloop_nB.sta_ref.att, 3);
  float_vect_zero(oneloop_nB.sta_ref.att_d, 3);
  float_vect_zero(oneloop_nB.sta_ref.att_2d, 3);
  float_vect_zero(oneloop_nB.sta_ref.att_3d, 3);
  float_vect_zero(nu, ANDI_OUTPUTS);
  float_vect_zero(nu_n, ANDI_OUTPUTS);
  float_vect_zero(nav_target, 3);
  float_vect_zero(nav_target_new, 3);
  eulers_zxy_des.phi = 0.0;
  eulers_zxy_des.theta = 0.0;
  eulers_zxy_des.psi = 0.0;
  // nB controller
  oneloop_nB.sta_nB_state.nI.x = 0.0;
  oneloop_nB.sta_nB_state.nI.y = 0.0;
  oneloop_nB.sta_nB_state.nI.z =-1.0;
// Start telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_oneloop_nB);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_oneloop_nB_ctrl);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_STAB, send_eff_mat_stab_oneloop_nB);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_GUID, send_eff_mat_guid_oneloop_nB);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE, send_guidance_oneloop_nB);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_oneloop_debug);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_oneloop);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_oneloop);
#endif
    // Log pointer to effectiveness matrix (can do only once after init)
}

/**
 * @brief Function that resets important values upon engaging Oneloop ANDI.
 * FIXME: Ideally we should distinguish between the "stabilization" and "guidance" needs because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 */
void oneloop_nB_enter(bool half_loop_sp, int ctrl_type)
{
  oneloop_nB.half_loop = half_loop_sp;
  oneloop_nB.ctrl_type = ctrl_type;
  psi_des_rad = eulers_zxy.psi;
  psi_des_deg = DegOfRad(eulers_zxy.psi);
  calc_normalization();
  G1G2_oneloop(oneloop_nB.ctrl_type);
  init_controller_gains();
  reinit_controller();
}

/**
 * @brief  Function to generate the reference signals for the oneloop controller
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * @param PSA_des    Desired position/speed/acceleration
 */
void oneloop_nB_RM(bool half_loop, struct FloatVect3 PSA_des, bool in_flight_oneloop)
{
  //printf("Starting Oneloop ANDI RM\n");
  // Initialize some variables
  a_thrust               = 0.0;
  nav_target[0]          = PSA_des.x;
  nav_target[1]          = PSA_des.y;
  nav_target[2]          = PSA_des.z;
  float radio_thrust_cmd = 0.0;
  float radio_roll_cmd   = 0.0;
  float radio_pitch_cmd  = 0.0;
  float des_r            = 0.0;
  // Generate reference signals with reference model
  if (half_loop)
  {
    //printf("Half loop mode RM enabled\n");
    // ======================================================================================================================================================
    // PHI & THETA Set desired attitude with stick input
    radio_roll_cmd  = (float)(radio_control_get(RADIO_ROLL));
    radio_pitch_cmd = (float)(radio_control_get(RADIO_PITCH));
    Bound(radio_roll_cmd, -MAX_PPRZ, MAX_PPRZ);
    Bound(radio_pitch_cmd, -MAX_PPRZ, MAX_PPRZ);
    eulers_zxy_des.phi   = radio_roll_cmd / MAX_PPRZ * ONELOOP_NB_MAX_PHI;
    eulers_zxy_des.theta = radio_pitch_cmd / MAX_PPRZ * ONELOOP_NB_MAX_THETA;
    float sphi_des   = sinf(eulers_zxy_des.phi);
    float cphi_des   = cosf(eulers_zxy_des.phi);
    float stheta_des = sinf(eulers_zxy_des.theta);
    float ctheta_des = cosf(eulers_zxy_des.theta);
    oneloop_nB.sta_nB_state.nI_des.x = -stheta_des;
    oneloop_nB.sta_nB_state.nI_des.y = sphi_des*ctheta_des;
    oneloop_nB.sta_nB_state.nI_des.z = -cphi_des*ctheta_des;
//#define nB_RADIO_BODY_CTRL
#ifdef nB_RADIO_BODY_CTRL
    float sin_psi = sinf(eulers_zxy.psi);
    float cos_psi = cosf(eulers_zxy.psi);
    struct FloatVect3 nI_des_NED;
    nI_des_NED.x = oneloop_nB.sta_nB_state.nI_des.x * cos_psi - oneloop_nB.sta_nB_state.nI_des.y * sin_psi;
    nI_des_NED.y = oneloop_nB.sta_nB_state.nI_des.x * sin_psi + oneloop_nB.sta_nB_state.nI_des.y * cos_psi;
    oneloop_nB.sta_nB_state.nI_des.x = nI_des_NED.x;
    oneloop_nB.sta_nB_state.nI_des.y = nI_des_NED.y;
#else
#endif
    // ======================================================================================================================================================
    // PSI Set desired Yaw rate with stick input
    if (!SpinQuad){
    des_r = (float)(radio_control_get(RADIO_YAW)) / MAX_PPRZ * max_r; // Get yaw rate from stick
    BoundAbs(des_r, max_r);  
    }                                       // Bound yaw rate
    float delta_psi_des_rad = des_r * dt_1l;                          // Integrate desired Yaw rate to get desired change in yaw
    float delta_psi_rad = eulers_zxy_des.psi - eulers_zxy.psi;        // Calculate current yaw difference between des and actual
    NormRadAngle(delta_psi_rad);                                      // Normalize the difference
    if (fabs(delta_psi_rad) > RadOfDeg(30.0))                         // If difference is bigger than 10 deg do not further increment desired
    { 
      delta_psi_des_rad = 0.0;
    }
    psi_des_rad += delta_psi_des_rad;                                 // Incrementdesired yaw
    NormRadAngle(psi_des_rad);
    // ======================================================================================================================================================
    // THRUST Create commands adhoc to get actuators to the wanted level
    radio_thrust_cmd = (float)radio_control_get(RADIO_THROTTLE);
    Bound(radio_thrust_cmd, 0.0, MAX_PPRZ);
    int8_t i;
    // To calculate the nu corrsponding to the Thrust command, plug it in the control law.
    for (i = 0; i < ANDI_NUM_ACT; i++)
    {
      float den = positive_non_zero(ratio_u_un[i] * ratio_vn_v[IDX_aD]);
      a_thrust += (radio_thrust_cmd) * EFF_MAT_G[IDX_aD][i] / den;
    }
    a_thrust = a_thrust - ctrl_effort_model[IDX_aD]; // Subtract model disturbance
    ctrl_off = false; // Make sure all control on for manual takeover
    nu[IDX_aD] = a_thrust;
  }
  else
  {
    // ======================================================================================================================================================
    // Need to save and convert desired position for bounding. Not sure NAV always updates this correctly.
    float pos_des[3];
    pos_des[0] = POS_FLOAT_OF_BFP(POS_BFP_OF_REAL(nav.target.y));
    pos_des[1] = POS_FLOAT_OF_BFP(POS_BFP_OF_REAL(nav.target.x));
    pos_des[2] = POS_FLOAT_OF_BFP(-POS_BFP_OF_REAL(nav.nav_altitude));
    // ======================================================================================================================================================
    // PID Guidance 
    float acc_des[3];
    Pos_PID(pos_des, oneloop_nB.gui_state.pos, oneloop_nB.gui_state.vel, k_P, k_I, k_D,acc_des);
    switch (oneloop_nB.ctrl_type)
    {
      case CTRL_ANDI:
      case CTRL_NB_ANDI:
        nu[IDX_aD] = (acc_des[2]-oneloop_nB.gui_state.acc[2])*k_pos_e.k3[2];
        break;
      case CTRL_INDI:
      case CTRL_NB_INDI:
        nu[IDX_aD] = (acc_des[2]-oneloop_nB.gui_state.acc[2]);
        break;
    }
    shape_vector(acc_des);
    eul_of_acc(acc_des, eulers_zxy.psi);
    oneloop_nB.sta_nB_state.nI_des.x = acc_des[0];
    oneloop_nB.sta_nB_state.nI_des.y = acc_des[1];
    oneloop_nB.sta_nB_state.nI_des.z = acc_des[2];
    // Update desired Heading (psi_des_rad) based on previous loop or changed setting
    if (heading_manual)
    {
      psi_des_rad = RadOfDeg(psi_des_deg);
      if (yaw_stick_in_auto)
      {
        psi_des_rad += (float)(radio_control_get(RADIO_YAW)) / MAX_PPRZ * max_r * dt_1l;
      }
    }
    else
    {
      psi_des_rad += oneloop_nB_sideslip() * dt_1l;
    }
    NormRadAngle(psi_des_rad);
  }
  // ======================================================================================================================================================
  if (!in_flight_oneloop){psi_des_rad = eulers_zxy.psi;}            // Reset if not flying
  eulers_zxy_des.psi = psi_des_rad;
  // ======================================================================================================================================================
  // Set and Save the desired attitude and run the attitude RM
  float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
#define OVERRIDE_ATT_RM
#ifdef OVERRIDE_ATT_RM
  float_vect_zero(oneloop_nB.sta_ref.att, 3);
  float_vect_zero(oneloop_nB.sta_ref.att_d, 3);
  float_vect_zero(oneloop_nB.sta_ref.att_2d, 3);
  float_vect_zero(oneloop_nB.sta_ref.att_3d, 3); 
  oneloop_nB.sta_ref.att[0] = att_des[0];
  oneloop_nB.sta_ref.att[1] = att_des[1];
  oneloop_nB.sta_ref.att[2] = att_des[2];
#else 
  rm_3rd_attitude(dt_1l, oneloop_nB.sta_ref.att, oneloop_nB.sta_ref.att_d, oneloop_nB.sta_ref.att_2d, oneloop_nB.sta_ref.att_3d, att_des, false, psi_vec, k_att_rm.k1, k_att_rm.k2, k_att_rm.k3, sta_bounds);
#endif  
  //printf("Running nB RM\n");
  rm_3rd_nI(dt_1l, &oneloop_nB.sta_nB_state.nI, &oneloop_nB.sta_nB_state.nI_d, &oneloop_nB.sta_nB_state.nI_2d, &oneloop_nB.sta_nB_state.nI_3d, &oneloop_nB.sta_nB_state.nI_des, k_att_rm.k1, k_att_rm.k2, k_att_rm.k3);

  //printf("Completed RM step\n");
  //printf("nI: x: %f y: %f z: %f\n", oneloop_nB.sta_nB_state.nI.x, oneloop_nB.sta_nB_state.nI.y, oneloop_nB.sta_nB_state.nI.z);
  //printf("nI_d: x: %f y: %f z: %f\n", oneloop_nB.sta_nB_state.nI_d.x, oneloop_nB.sta_nB_state.nI_d.y, oneloop_nB.sta_nB_state.nI_d.z);
  //printf("nI_2d: x: %f y: %f z: %f\n", oneloop_nB.sta_nB_state.nI_2d.x, oneloop_nB.sta_nB_state.nI_2d.y, oneloop_nB.sta_nB_state.nI_2d.z);
  
  // ======================================================================================================================================================
}

/**
 * @brief  Main function that runs the controller and performs control allocation
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * @param in_flight  The drone is in flight
 * @param PSA_des    Desired position/speed/acceleration
 */
void oneloop_nB_run(bool in_flight, bool half_loop, struct FloatVect3 PSA_des)
{
  // ======================================================================================================================================================
  // At beginnig of the loop: (1) Register Attitude, (2) Initialize gains of RM and EC, (3) Calculate Normalization of Actuators Signals, (4) Propagate Actuator Model, (5) Update effectiveness matrix
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  init_controller_gains();
  calc_normalization();
  get_act_state_oneloop();
  // ======================================================================================================================================================
  // If drone is not on the ground use incremental law
  bool  in_flight_oneloop = false;
  if(in_flight) {in_flight_oneloop = true;}
  if (ONELOOP_NB_DEBUG_MODE){in_flight_oneloop = true;}
  // ======================================================================================================================================================
  // Calculate disturbance (model control effort) based on the model
  oneloop_calc_model_disturbance(in_flight);
  // ======================================================================================================================================================
  // Register the state of the drone in the variables used in RM and EC
  // (1) Attitude related
  oneloop_nB.sta_state.att[0]    = eulers_zxy.phi;
  oneloop_nB.sta_state.att[1]    = eulers_zxy.theta;
  oneloop_nB.sta_state.att[2]    = eulers_zxy.psi;
  oneloop_nB_propagate_filters(); // needs to be after update of attitude vector
  oneloop_nB.sta_state.att_d[0]  = LP.p.out;
  oneloop_nB.sta_state.att_d[1]  = LP.q.out;
  oneloop_nB.sta_state.att_d[2]  = LP.r.out;
  oneloop_nB.sta_state.att_2d[0] = LP.p_dot.out;
  oneloop_nB.sta_state.att_2d[1] = LP.q_dot.out;
  oneloop_nB.sta_state.att_2d[2] = LP.r_dot.out;
  // (2) Position related
  oneloop_nB.gui_state.pos[0]    = stateGetPositionNed_f()->x;
  oneloop_nB.gui_state.pos[1]    = stateGetPositionNed_f()->y;
  oneloop_nB.gui_state.pos[2]    = stateGetPositionNed_f()->z;
  oneloop_nB.gui_state.vel[0]    = stateGetSpeedNed_f()->x;
  oneloop_nB.gui_state.vel[1]    = stateGetSpeedNed_f()->y;
  oneloop_nB.gui_state.vel[2]    = stateGetSpeedNed_f()->z;
  oneloop_nB.gui_state.acc[0]    = LP.ax.out;
  oneloop_nB.gui_state.acc[1]    = LP.ay.out;
  oneloop_nB.gui_state.acc[2]    = LP.az.out;
  // ======================================================================================================================================================
  // Calculated feedforward signal for yaw control
  g2_ff = 0.0;
  for (int i = 0; i < ANDI_NUM_ACT; i++)
  {
    switch (oneloop_nB.ctrl_type)
    {
    case CTRL_ANDI:
      g2_ff += G2_RW[i] * act_dyn_ctrl[i] * (andi_u[i] - u_filt[i].o[0]);
      break;
    case CTRL_INDI:
      g2_ff += G2_RW[i] * (andi_u[i] - u_filt[i].o[0]);
      break;
    default:
      break;
    }
  }
  // ======================================================================================================================================================
  // Run the Reference Model (RM)
  oneloop_nB_RM(half_loop, PSA_des, in_flight_oneloop);
  // ======================================================================================================================================================
  // Calculate new nB states based on updated RM for nI
  oneloop_nB_calc_nB_states();
  // ======================================================================================================================================================
  // Update Effectiveness matrix based on the control type
  G1G2_oneloop(oneloop_nB.ctrl_type);
  // ======================================================================================================================================================
  // Attitude EC
  float att_jerk_des[3];
  float nu_stab[3]= {0.0, 0.0, 0.0};
  float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
  oneloop_nB.sta_nB_state.mu_B.x = 0.0;
  oneloop_nB.sta_nB_state.mu_B.y = 0.0;
  oneloop_nB.sta_nB_state.mu_B.z =-1.0;
  
  float ctrl_effort_model_att[3] = {ctrl_effort_model[IDX_ap], ctrl_effort_model[IDX_aq], ctrl_effort_model[IDX_ar]};
  float dummy1[3] = {1.0, 1.0, 1.0};
  float dummy0[3] = {0.0, 0.0, 0.0};
  struct FloatVect3 nB_2d_FV3;
  nB_2d_FV3.x = nB_2d_filt[0].o[0];
  nB_2d_FV3.y = nB_2d_filt[1].o[0];
  nB_2d_FV3.z = nB_2d_filt[2].o[0];
  switch (oneloop_nB.ctrl_type)
  {
    case CTRL_ANDI:
      ec_3rd_att(att_jerk_des, att_des, oneloop_nB.sta_ref.att, oneloop_nB.sta_ref.att_d, oneloop_nB.sta_ref.att_2d, oneloop_nB.sta_ref.att_3d, oneloop_nB.sta_state.att, oneloop_nB.sta_state.att_d, oneloop_nB.sta_state.att_2d, k_att_e.k1, k_att_e.k2, k_att_e.k3, sta_bounds, ctrl_effort_model_att);
      nB_EC(oneloop_nB.sta_nB_state.nB, oneloop_nB.sta_nB_state.nB_d, oneloop_nB.sta_nB_state.nB_2d, oneloop_nB.sta_nB_state.mu_B, k_att_e.k1, k_att_e.k2, dummy1, ctrl_effort_model_att, nB_jerk_des);
      nu_stab[0] = att_jerk_des[0];
      nu_stab[1] = att_jerk_des[1];
      nu_stab[2] = att_jerk_des[2];
      SpinQuad_overwrite(k_att_e.k3[2], ctrl_effort_model[IDX_ar], &nu_stab[2]);
      break;
    case CTRL_INDI:
      ec_3rd_att(att_jerk_des, att_des, oneloop_nB.sta_ref.att, oneloop_nB.sta_ref.att_d, oneloop_nB.sta_ref.att_2d, dummy0, oneloop_nB.sta_state.att, oneloop_nB.sta_state.att_d, oneloop_nB.sta_state.att_2d, k_att_e.k1, k_att_e.k2, dummy1, sta_bounds, ctrl_effort_model_att);
      nu_stab[0] = att_jerk_des[0];
      nu_stab[1] = att_jerk_des[1];
      nu_stab[2] = att_jerk_des[2];
      SpinQuad_overwrite(1.0, ctrl_effort_model[IDX_ar], &nu_stab[2]);
      break;
    case CTRL_NB_ANDI:
      ec_3rd_att(att_jerk_des, att_des, oneloop_nB.sta_ref.att, oneloop_nB.sta_ref.att_d, oneloop_nB.sta_ref.att_2d, oneloop_nB.sta_ref.att_3d, oneloop_nB.sta_state.att, oneloop_nB.sta_state.att_d, oneloop_nB.sta_state.att_2d, k_att_e.k1, k_att_e.k2, k_att_e.k3, sta_bounds, ctrl_effort_model_att);
      //nB_EC(oneloop_nB.sta_nB_state.nB, oneloop_nB.sta_nB_state.nB_d, oneloop_nB.sta_nB_state.nB_2d, oneloop_nB.sta_nB_state.mu_B, k_att_e.k1, k_att_e.k2, k_att_e.k3, ctrl_effort_model_att, nB_jerk_des);
      nB_EC(oneloop_nB.sta_nB_state.nB, oneloop_nB.sta_nB_state.nB_d, nB_2d_FV3, oneloop_nB.sta_nB_state.mu_B, k_att_e.k1, k_att_e.k2, k_att_e.k3, ctrl_effort_model_att, nB_jerk_des);
      nu_stab[0] = nB_jerk_des[0];
      nu_stab[1] = nB_jerk_des[1];
      nu_stab[2] = att_jerk_des[2];
      SpinQuad_overwrite(k_att_e.k3[2], ctrl_effort_model[IDX_ar], &nu_stab[2]);
      break;
    case CTRL_NB_INDI:
      ec_3rd_att(att_jerk_des, att_des, oneloop_nB.sta_ref.att, oneloop_nB.sta_ref.att_d, oneloop_nB.sta_ref.att_2d, dummy0, oneloop_nB.sta_state.att, oneloop_nB.sta_state.att_d, oneloop_nB.sta_state.att_2d, k_att_e.k1, k_att_e.k2, dummy1, sta_bounds, ctrl_effort_model_att);
      //nB_EC(oneloop_nB.sta_nB_state.nB, oneloop_nB.sta_nB_state.nB_d, oneloop_nB.sta_nB_state.nB_2d, oneloop_nB.sta_nB_state.mu_B, k_att_e.k1, k_att_e.k2, dummy1, ctrl_effort_model_att, nB_jerk_des);
      nB_EC(oneloop_nB.sta_nB_state.nB, oneloop_nB.sta_nB_state.nB_d, nB_2d_FV3, oneloop_nB.sta_nB_state.mu_B, k_att_e.k1, k_att_e.k2, dummy1, ctrl_effort_model_att, nB_jerk_des);
      nu_stab[0] = nB_jerk_des[0];
      nu_stab[1] = nB_jerk_des[1];
      nu_stab[2] = att_jerk_des[2];
      SpinQuad_overwrite(1.0, ctrl_effort_model[IDX_ar], &nu_stab[2]);
      break;
  }
  // ======================================================================================================================================================
  // Set Pseudo-control inputs nu based on EC outputs
  if (half_loop && radio_control_get(RADIO_THROTTLE) < 200)
  {
    nu[IDX_ap] = 0.0;
    nu[IDX_aq] = 0.0;
    nu[IDX_ar] = 0.0;
  }
  else
  {
    nu[IDX_ap] = nu_stab[0];
    nu[IDX_aq] = nu_stab[1];
    nu[IDX_ar] = nu_stab[2] + g2_ff;
  }
  nu[IDX_aD] = nu[IDX_aD] + ctrl_effort_model[IDX_aD]; 
  // ======================================================================================================================================================
  // Manipulate Pseudo control vecotors for state compensation and axis dropping
  // BoundAbs(nu[5], n_array[5]*coupling_factor[5]); //FIXME UNCOMMENT ME
  drop_axis();
  oneloop_nB_state_compensation(state_compensation_on);
  // ======================================================================================================================================================
  // Set the WLS settings
  set_WLS_settings();
  // ======================================================================================================================================================
  // WLS Control Allocator
  normalize_nu();
  wls_alloc(&WLS_one_p, bwls_1l, 0, 0, 10);
  // ======================================================================================================================================================
  // Save the commands to the actuators
  for (int i = 0; i < ANDI_NUM_ACT_TOT; i++)
  {
    andi_u_n[i] = WLS_one_p.u[i];
    andi_u[i] = (float)(andi_u_n[i] * ratio_u_un[i]);
    Bound(andi_u[i], act_min[i], act_max[i]);
  }
  if (fault_pitch && oneloop_nB.ctrl_type == CTRL_NB_INDI){
    float temp_thrust = (float)radio_control_get(RADIO_THROTTLE);
    Bound(temp_thrust, 0.0, max_pitch_mot);
    andi_u[COMMAND_MOTOR_FRONT] = temp_thrust;
    andi_u[COMMAND_MOTOR_BACK]  = temp_thrust;
  }
  /*Commit the actuator command*/
  for (int i = 0; i < ANDI_NUM_ACT; i++)
  {
#if ONELOOP_NB_DEBUG_MODE
    commands[i] = (int16_t)(0.0);   
#else 
    commands[i] = (int16_t)andi_u[i];
#endif
  }

  commands[COMMAND_THRUST] = (commands[COMMAND_MOTOR_FRONT] + commands[COMMAND_MOTOR_RIGHT] + commands[COMMAND_MOTOR_BACK] + commands[COMMAND_MOTOR_LEFT]) / num_thrusters_oneloop;
  autopilot.throttle = commands[COMMAND_THRUST];
  stabilization.cmd[COMMAND_THRUST] = commands[COMMAND_THRUST];
  if (heading_manual)
  {
    psi_des_deg = DegOfRad(psi_des_rad);
  }
  stabilization.cmd[COMMAND_ROLL]  = (int16_t)(DegOfRad(eulers_zxy_des.phi) * MAX_PPRZ / DegOfRad(ONELOOP_NB_MAX_PHI));
  stabilization.cmd[COMMAND_PITCH] = (int16_t)(DegOfRad(eulers_zxy_des.theta) * MAX_PPRZ / DegOfRad(ONELOOP_NB_MAX_THETA));
  stabilization.cmd[COMMAND_YAW]   = (int16_t)(psi_des_deg * MAX_PPRZ / 180.0);
}

//=========================================================================================================================================================
/** @brief  Function to reconstruct actuator state using first order dynamics */
//==========================================================================================================================================================
void get_act_state_oneloop(void)
{
  int8_t i;
  float prev_actuator_state_1l;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++)
  {
    if (i < ANDI_NUM_ACT)
    {
      prev_actuator_state_1l = actuator_state_1l[i];
      actuator_state_1l[i] = prev_actuator_state_1l + act_dynamics_d[i] * (andi_u[i] - prev_actuator_state_1l);
      if (!autopilot_get_motors_on())
      {
        actuator_state_1l[i] = 0.0;
      }
      Bound(actuator_state_1l[i], act_min[i], act_max[i]);
    }
    else
    {
      actuator_state_1l[i] = oneloop_nB.sta_state.att[i - ANDI_NUM_ACT];
    }
  }
}
//=========================================================================================================================================================
/**
 * @brief Function that samples and scales the effectiveness matrix
 * FIXME: make this function into a for loop to make it more adaptable to different configurations
 */
//=========================================================================================================================================================
void G1G2_oneloop(int ctrl_type)
{
  for (int i = 0; i < ANDI_OUTPUTS; i++)
    {
      bwls_1l[i] = EFF_MAT_G[i];
    }
  float scaler = 1.0;
  for (int i = 0; i < ANDI_NUM_ACT_TOT; i++)
  {
    switch (ctrl_type)
    {
    case (CTRL_ANDI):
    case (CTRL_NB_ANDI):
      scaler = act_dyn_ctrl[i] * ratio_u_un[i];
      break;
    case (CTRL_INDI):
    case (CTRL_NB_INDI):
      scaler = ratio_u_un[i];
      break;
    }
    for (int j = 0; j < ANDI_OUTPUTS; j++)
    {
      EFF_MAT_G[j][i] = EFF_MAT_RW[2+j][i] * scaler * ratio_vn_v[j]; // EFF_MAT_RW has extra entries for North and EAST
    }
  }

  switch (ctrl_type)
  {
  case (CTRL_NB_ANDI):
  case (CTRL_NB_INDI):
   {
    //printf("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
    //printf("EFF_MAT_G MOTOR RIGHT [%i][%i]=%f\n",IDX_ap,COMMAND_MOTOR_RIGHT,EFF_MAT_G[IDX_ap][COMMAND_MOTOR_RIGHT]);
    //printf("EFF_MAT_G MOTOR LEFT  [%i][%i]=%f\n",IDX_ap,COMMAND_MOTOR_LEFT,EFF_MAT_G[IDX_ap][COMMAND_MOTOR_LEFT]);
    //printf("nB_filt[0].o[0]=%f\n",nB_filt[0].o[0]);
    //printf("nB_filt[1].o[0]=%f\n",nB_filt[1].o[0]);
    //printf("nB_filt[2].o[0]=%f\n",nB_filt[2].o[0]);
    for (int j = 0; j < ANDI_NUM_ACT_TOT; j++){ 
      
      if(j == COMMAND_MOTOR_RIGHT){
        //printf("PC EFF_MAT_G[IDX_aq][COMMAND_MOTOR_RIGHT]=%f\n",EFF_MAT_G[IDX_ap][j]);
        //printf("PC EFF_MAT_G[IDX_ar][COMMAND_MOTOR_RIGHT]=%f\n",EFF_MAT_G[IDX_ar][j]);
        //printf("nB_filt[0].o[0]=%f\n",nB_filt[0].o[0]);
        //printf("nB_filt[2].o[0]=%f\n",nB_filt[2].o[0]);
      }
      if (j == COMMAND_MOTOR_LEFT){
        //printf("PC EFF_MAT_G[IDX_aq][COMMAND_MOTOR_LEFT]=%f\n",EFF_MAT_G[IDX_ap][j]);
        //printf("PC EFF_MAT_G[IDX_ar][COMMAND_MOTOR_LEFT]=%f\n",EFF_MAT_G[IDX_ar][j]);
        //printf("nB_filt[0].o[0]=%f\n",nB_filt[0].o[0]);
        //printf("nB_filt[2].o[0]=%f\n",nB_filt[2].o[0]);
      }      
      float temp_ap_j = - EFF_MAT_G[IDX_aq][j]*nB_filt[2].o[0] ;//+ EFF_MAT_G[IDX_ar][j]*nB_filt[1].o[0] ;
      float temp_aq_j = EFF_MAT_G[IDX_ap][j]*nB_filt[2].o[0]   ;//- EFF_MAT_G[IDX_ar][j]*nB_filt[0].o[0];
      EFF_MAT_G[IDX_ap][j] = temp_ap_j;
      EFF_MAT_G[IDX_aq][j] = temp_aq_j;
    
      if(j == COMMAND_MOTOR_RIGHT){
        //printf("bwls_1l[IDX_aq][COMMAND_MOTOR_RIGHT]=%f\n",bwls_1l[IDX_aq][j]);
      }
      if (j == COMMAND_MOTOR_LEFT){
        //printf("bwls_1l[IDX_aq][COMMAND_MOTOR_LEFT]=%f\n",bwls_1l[IDX_aq][j]);
      }
    }
    if (fault_pitch){
      for (int j = 0; j < ANDI_NUM_ACT_TOT; j++){ 
        EFF_MAT_G[IDX_ap][j] = 0.0;
        EFF_MAT_G[IDX_ar][j] = 0.0;
      }
      for (int i = 0; i < ANDI_OUTPUTS; i++){
        EFF_MAT_G[i][COMMAND_MOTOR_FRONT] = 0.0;
        EFF_MAT_G[i][COMMAND_MOTOR_BACK]  = 0.0;
      }      
    }
  }
    break;
  }

  for (int i = 0; i < ANDI_OUTPUTS; i++){
    //printf("MOTOR FRONT [%i][%i]=%f\n",i,COMMAND_MOTOR_FRONT,bwls_1l[i][COMMAND_MOTOR_FRONT]);
    //printf("MOTOR BACK  [%i][%i]=%f\n",i,COMMAND_MOTOR_BACK,bwls_1l[i][COMMAND_MOTOR_BACK]);
    //printf("MOTOR RIGHT [%i][%i]=%f\n",i,COMMAND_MOTOR_RIGHT,bwls_1l[i][COMMAND_MOTOR_RIGHT]);
    //printf("MOTOR LEFT  [%i][%i]=%f\n",i,COMMAND_MOTOR_LEFT,bwls_1l[i][COMMAND_MOTOR_LEFT]);
  }

  for (int i = 0; i < ANDI_NUM_ACT_TOT; i++)
  {
    act_dyn_ctrl[i] = act_dynamics[i];
  }
}
//=========================================================================================================================================================
/** @brief  Calculate Normalization of actuators and discrete actuator dynamics  */
//=========================================================================================================================================================
void calc_normalization(void)
{
  int8_t i;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++)
  {
    act_dynamics_d[i] = 1.0 - exp(-act_dynamics[i] * dt_1l);
    Bound(act_dynamics_d[i], 0.00001, 1.0);
    Bound(act_max[i], 0, MAX_PPRZ);
    Bound(act_min[i], -MAX_PPRZ, 0);
    float ratio_numerator = act_max[i] - act_min[i];
    ratio_numerator = positive_non_zero(ratio_numerator); // make sure numerator is non-zero
    float ratio_denominator = act_max_norm[i] - act_min_norm[i];
    ratio_denominator = positive_non_zero(ratio_denominator); // make sure denominator is non-zero
    ratio_u_un[i] = ratio_numerator / ratio_denominator;
    ratio_u_un[i] = positive_non_zero(ratio_u_un[i]); // make sure ratio is not zero
  }
  for (i = 0; i < ANDI_OUTPUTS; i++)
  {
    float ratio_numerator = positive_non_zero(nu_norm_max);
    float ratio_denominator = 1.0;
    switch (i)
    {
    case (IDX_aD):
      ratio_denominator = positive_non_zero(max_j_lin);
      ratio_vn_v[i] = ratio_numerator / ratio_denominator;
      break;
    case (IDX_ap):
      ratio_denominator = positive_non_zero(sta_bounds.att_3d[0]);
      ratio_vn_v[i] = ratio_numerator / ratio_denominator;
      break;
    case (IDX_aq):
      ratio_denominator = positive_non_zero(sta_bounds.att_3d[1]);
      ratio_vn_v[i] = ratio_numerator / ratio_denominator;
      break;
    case (IDX_ar):
      ratio_denominator = positive_non_zero(sta_bounds.att_3d[2]);
      ratio_vn_v[i] = ratio_numerator / ratio_denominator;
      break;
    }
  }
}
//=========================================================================================================================================================
/** @brief  Function to normalize the pseudo control vector */
//=========================================================================================================================================================
void normalize_nu(void)
{
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++)
  {
    // printf("ratio_vn_v[%d] = %f \n",i,ratio_vn_v[i]);
    nu_n[i] = nu[i] * ratio_vn_v[i];
    WLS_one_p.v[i] = nu_n[i];
  }
}
//=========================================================================================================================================================
/** @brief  Function that maps navigation inputs to the oneloop controller for the generated autopilot. */
//=========================================================================================================================================================
void oneloop_from_nav(bool in_flight)
{
  if (!in_flight)
  {
    oneloop_nB_enter(false, oneloop_nB.ctrl_type);
  }
  struct FloatVect3 PSA_des;
  PSA_des.x = stateGetPositionNed_f()->x;
  PSA_des.y = stateGetPositionNed_f()->y;
  PSA_des.z = stateGetPositionNed_f()->z;
  //int rm_order_h = 3;
  //int rm_order_v = 3;
  // Oneloop controller wants desired targets and handles reference generation internally
  switch (nav.setpoint_mode)
  {
  case NAV_SETPOINT_MODE_POS:
    PSA_des.x = POS_FLOAT_OF_BFP(POS_BFP_OF_REAL(nav.target.y));
    PSA_des.y = POS_FLOAT_OF_BFP(POS_BFP_OF_REAL(nav.target.x));
    //rm_order_h = 3;
    break;
  case NAV_SETPOINT_MODE_SPEED:
    PSA_des.x = SPEED_FLOAT_OF_BFP(SPEED_BFP_OF_REAL(nav.speed.y));
    PSA_des.y = SPEED_FLOAT_OF_BFP(SPEED_BFP_OF_REAL(nav.speed.x));
    //rm_order_h = 2;
    break;
  }
  switch (nav.vertical_mode)
  {
  case NAV_VERTICAL_MODE_ALT:
    PSA_des.z = POS_FLOAT_OF_BFP(-POS_BFP_OF_REAL(nav.nav_altitude));
    //rm_order_v = 3;
    break;
  case NAV_VERTICAL_MODE_CLIMB:
    PSA_des.z = SPEED_FLOAT_OF_BFP(-SPEED_BFP_OF_REAL(nav.climb));
    //rm_order_v = 2;
    break;
  }
  oneloop_nB_run(in_flight, false, PSA_des);
}
//=========================================================================================================================================================
/** @brief Function to calculate corrections for sideslip DEPRECIATED */ 
//=========================================================================================================================================================
float oneloop_nB_sideslip(void)
{
  //  Coordinated turn
  //  feedforward estimate angular rotation omega = g*tan(phi)/v
  float omega = 0.0;
  const float max_phi = ONELOOP_NB_MAX_BANK;
  float airspeed_turn = airspeed_filt.o[0];
  Bound(airspeed_turn, 1.0f, 30.0f);
  // Use the current roll angle to determine the corresponding heading rate of change.
  float coordinated_turn_roll = eulers_zxy.phi;
  // Prevent flipping
  if ((eulers_zxy.theta > 0.0f) && (fabs(eulers_zxy.phi) < eulers_zxy.theta))
  {
    // printf("Preventing flipping\n");
    coordinated_turn_roll = ((eulers_zxy.phi > 0.0f) - (eulers_zxy.phi < 0.0f)) * eulers_zxy.theta;
  }
  BoundAbs(coordinated_turn_roll, max_phi);
  omega = g / airspeed_turn * tanf(coordinated_turn_roll);
#ifdef FWD_SIDESLIP_GAIN
  // Add sideslip correction
  omega -= accely_filt.o[0] * fwd_sideslip_gain;
#endif
  return omega;
}
//=========================================================================================================================================================
/** Quadplanes can still be in-flight with COMMAND_THRUST==0 and can even soar not descending in updrafts with all thrust off */
//=========================================================================================================================================================
bool autopilot_in_flight_end_detection(bool motors_on UNUSED)
{
  return !motors_on;
}
//=========================================================================================================================================================
// Some functions for Airspeed Control DEPRECIATED
//=========================================================================================================================================================
void reshape_wind(void)
{
  float psi = eulers_zxy.psi;
  float cpsi = cosf(psi);
  float spsi = sinf(psi);
  float airspeed = airspeed_filt.o[0];
  struct FloatVect2 NT_v_NE = {nav_target[0], nav_target[1]}; // Nav target in North and East frame
  struct FloatVect2 airspeed_v = {cpsi * airspeed, spsi * airspeed};
  struct FloatVect2 windspeed;
  struct FloatVect2 groundspeed = {oneloop_nB.gui_state.vel[0], oneloop_nB.gui_state.vel[1]};
  struct FloatVect2 des_as_NE;
  struct FloatVect2 des_as_B;
  struct FloatVect2 des_acc_B;
  VECT2_DIFF(windspeed, groundspeed, airspeed_v); // Wind speed in North and East frame
  VECT2_DIFF(des_as_NE, NT_v_NE, windspeed);      // Desired airspeed in North and East frame
  float norm_des_as = FLOAT_VECT2_NORM(des_as_NE);
  gi_unbounded_airspeed_sp = norm_des_as;
  // Check if some minimum airspeed is desired (e.g. to prevent stall)
  if (norm_des_as < min_as)
  {
    norm_des_as = min_as;
  }
  nav_target_new[0] = NT_v_NE.x;
  nav_target_new[1] = NT_v_NE.y;
  // if the desired airspeed is larger than the max airspeed or we are in force forward reshape gs des to cancel wind and fly at max airspeed
  if ((norm_des_as > max_as) || (force_forward))
  {
    float groundspeed_factor = 0.0f;
    if (FLOAT_VECT2_NORM(windspeed) < max_as)
    {
      float av = NT_v_NE.x * NT_v_NE.x + NT_v_NE.y * NT_v_NE.y; // norm squared of nav target
      float bv = -2.f * (windspeed.x * NT_v_NE.x + windspeed.y * NT_v_NE.y);
      float cv = windspeed.x * windspeed.x + windspeed.y * windspeed.y - max_as * max_as;
      float dv = bv * bv - 4.0f * av * cv;
      // dv can only be positive, but just in case
      if (dv < 0.0f)
      {
        dv = fabsf(dv);
      }
      float d_sqrt = sqrtf(dv);
      groundspeed_factor = (-bv + d_sqrt) / (2.0f * av);
    }
    des_as_NE.x = groundspeed_factor * NT_v_NE.x - windspeed.x;
    des_as_NE.y = groundspeed_factor * NT_v_NE.y - windspeed.y;
    NT_v_NE.x = groundspeed_factor * NT_v_NE.x;
    NT_v_NE.y = groundspeed_factor * NT_v_NE.y;
    norm_des_as = max_as;
  }
  des_as_B.x = norm_des_as; // Desired airspeed in body x frame
  des_as_B.y = 0.0;         // Desired airspeed in body y frame
  if (((airspeed > ONELOOP_NB_AIRSPEED_SWITCH_THRESHOLD) && (norm_des_as > (ONELOOP_NB_AIRSPEED_SWITCH_THRESHOLD + 2.0f))) || (force_forward))
  {
    float delta_psi = atan2f(des_as_NE.y, des_as_NE.x) - psi;
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    des_acc_B.y = delta_psi * 5.0;                          // gih_params.heading_bank_gain;
    des_acc_B.x = (des_as_B.x - airspeed) * k_pos_rm.k2[0]; // gih_params.speed_gain;
    acc_body_bound(&des_acc_B, max_a_nav);                  // Scale down side acceleration if norm is too large
    nav_target_new[0] = cpsi * des_acc_B.x - spsi * des_acc_B.y;
    nav_target_new[1] = spsi * des_acc_B.x + cpsi * des_acc_B.y;
  }
  else
  {
    nav_target_new[0] = (NT_v_NE.x - groundspeed.x) * k_pos_rm.k2[0];
    nav_target_new[1] = (NT_v_NE.y - groundspeed.y) * k_pos_rm.k2[1];
  }
  vect_bound_nd(nav_target_new, max_a_nav, 2);
}

void guidance_set_min_max_airspeed(float min_airspeed, float max_airspeed)
{
  min_as = min_airspeed;
  max_as = max_airspeed;
}
//=========================================================================================================================================================
/** @brief  Function that calculates the model disturbance (Control Effort) 
 * The control effort is equalt to :
 * EFF_MAT*u_filt
 * where EFF_MAT is what is inverted and u_filt is the filtered actuator state
*/
//=========================================================================================================================================================
void oneloop_calc_model_disturbance(bool in_flight)
{
  for (int8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    update_butterworth_2_low_pass(&u_filt[i], actuator_state_1l[i]);
  }
  // Store the distrubance
  // float k3;
  if (in_flight)
  {
    for (int8_t i = 0; i < ANDI_OUTPUTS; i++)
    { // For loop for prediction of acceleration
      ctrl_effort_model[i] = 0.0;
      // switch (i)
      // {
      // case (IDX_aD):
      //   k3 = 1.0; // k_pos_e.k3[2];
      //   break;
      // case (IDX_ap):
      //   k3 = k_att_e.k3[0];
      //   break;
      // case (IDX_aq):
      //   k3 = k_att_e.k3[1];
      //   break;
      // case (IDX_ar):
      //   k3 = k_att_e.k3[2];
      //   break;
      // }
      // k3 = positive_non_zero(k3);
      for (int8_t j = 0; j < ANDI_NUM_ACT_TOT; j++)
      {
        float den = positive_non_zero(ratio_u_un[j] * ratio_vn_v[i]);
        float num = u_filt[j].o[0] * EFF_MAT_G[i][j];
        ctrl_effort_model[i] += num / den;
      }
      // ctrl_effort_model[i] = ctrl_effort_model[i] / k3;
    }
  }
  else
  {
    float_vect_zero(ctrl_effort_model, ANDI_OUTPUTS);
  }
}
//=========================================================================================================================================================
/** @brief  Function that performs state compensation for gyroscopic effects */
//=========================================================================================================================================================
void oneloop_nB_state_compensation(bool state_compensation_on)
{
  float p     = oneloop_nB.sta_state.att_d[0];
  float q     = oneloop_nB.sta_state.att_d[1];
  float r     = oneloop_nB.sta_state.att_d[2];
  float p_dot = oneloop_nB.sta_state.att_2d[0];
  float q_dot = oneloop_nB.sta_state.att_2d[1];
  float r_dot = oneloop_nB.sta_state.att_2d[2];
  if (state_compensation_on)
  {
    nu[IDX_ap] -= (r * q_dot + q * r_dot) * (RW.I.yy - RW.I.zz) / RW.I.xx; // Roll
    nu[IDX_aq] -= (r * p_dot + p * r_dot) * (RW.I.zz - RW.I.xx) / RW.I.yy; // Pitch
  }
}
//=========================================================================================================================================================
/** @brief  Function that drops control along selected axes */
//=========================================================================================================================================================
void drop_axis(void){
  if (drop_yaw)
  {
    nu[IDX_ar] = ctrl_effort_model[IDX_ar];
  }
  if (drop_roll)
  {
    nu[IDX_ap] = ctrl_effort_model[IDX_ap];
  }
  if (drop_pitch)
  {
    nu[IDX_aq] = ctrl_effort_model[IDX_aq];
  }
  if (drop_aD)
  {
    nu[IDX_aD] = ctrl_effort_model[IDX_aD];
  }  
}
//=========================================================================================================================================================
/** @brief  Function that sets the WLS settings for oneloop controller */
//=========================================================================================================================================================
void set_WLS_settings(void){
  for (int i = 0; i < ANDI_NUM_ACT_TOT; i++)
  {
    switch (i)
    {
      case COMMAND_MOTOR_FRONT:
      case COMMAND_MOTOR_RIGHT:
      case COMMAND_MOTOR_BACK:
      case COMMAND_MOTOR_LEFT:
        WLS_one_p.Wu[i]     = Wu_backup[i];
        WLS_one_p.u_min[i]  = (act_min[i]) / ratio_u_un[i];
        WLS_one_p.u_pref[i] = (u_pref[i]) / ratio_u_un[i];
        WLS_one_p.u_max[i]  = (act_max[i]) / ratio_u_un[i];
        break;
    }
  }
  if (fault_pitch && oneloop_nB.ctrl_type == CTRL_NB_INDI){
    //printf("I AM FAULTED \n");
    WLS_one_p.Wv[IDX_ap] = 0.0; // Roll axis dropped because it is body axis
    WLS_one_p.Wv[IDX_ar] = 0.0; // Drop Yaw axis
    drop_yaw = true;
    drop_roll = true;
  } else {
    WLS_one_p.Wv[IDX_ap] = Wv_backup[IDX_ap];
    WLS_one_p.Wv[IDX_ar] = Wv_backup[IDX_ar];
    drop_yaw = false;
    drop_roll = false;
  }
  //printf("drop_roll = %i\n", drop_roll);
}
//=========================================================================================================================================================
/** @brief Function which estimates the nB states */
void oneloop_nB_calc_nB_states(void)
{
  struct FloatRMat *LBI = stateGetNedToBodyRMat_f();
  struct FloatRMat Omega_B;
  struct FloatRMat Omega_B_dot;
  struct FloatVect3 pqr;
  struct FloatVect3 pqr_dot;
  struct FloatVect3 A;
  struct FloatVect3 B;
  struct FloatVect3 C;
  struct FloatVect3 D;
  struct FloatVect3 temp;
  FLOAT_VECT3_ZERO(oneloop_nB.sta_nB_state.nB);
  FLOAT_VECT3_ZERO(oneloop_nB.sta_nB_state.nB_d);
  FLOAT_VECT3_ZERO(oneloop_nB.sta_nB_state.nB_2d);
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  pqr.x     = LP.p.meas;      //oneloop_nB.sta_state.att_d[0];
  pqr.y     = LP.q.meas;      //oneloop_nB.sta_state.att_d[1];
  pqr.z     = LP.r.meas;      //oneloop_nB.sta_state.att_d[2];
  pqr_dot.x = LP.p_dot.meas;  //oneloop_nB.sta_state.att_2d[0];
  pqr_dot.y = LP.q_dot.meas;  //oneloop_nB.sta_state.att_2d[1];
  pqr_dot.z = LP.r_dot.meas;  //oneloop_nB.sta_state.att_2d[2];
  skew_symmetric(&Omega_B, &pqr);
  skew_symmetric(&Omega_B_dot, &pqr_dot);
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  // nB = LBI * nI 
  float_rmat_vmult(&oneloop_nB.sta_nB_state.nB, LBI, &oneloop_nB.sta_nB_state.nI);
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  // nB_d= -Omega_B * nB + LBI * nI_dot 
  float_rmat_vmult(&A, &Omega_B, &oneloop_nB.sta_nB_state.nB);
  float_rmat_vmult(&B, LBI, &oneloop_nB.sta_nB_state.nI_d);
  VECT3_SUB(oneloop_nB.sta_nB_state.nB_d, A);
  VECT3_ADD(oneloop_nB.sta_nB_state.nB_d, B);
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  // nB_2d= -Omega_B_dot * nB - Omega_B * nB_dot - Omega_B * LBI * nI_dot + LBI * nI_2dot
#ifdef USE_ND_NB_2D  
  static bool nB_2d_inited = false;
  static struct FloatVect3 nB_d_prev;
  if (!nB_2d_inited) { 
    nB_d_prev.x = oneloop_nB.sta_nB_state.nB_d.x;
    nB_d_prev.y = oneloop_nB.sta_nB_state.nB_d.y;
    nB_d_prev.z = oneloop_nB.sta_nB_state.nB_d.z;
    nB_2d_inited = true; 
  }
  oneloop_nB.sta_nB_state.nB_2d.x = (oneloop_nB.sta_nB_state.nB_d.x - nB_d_prev.x) * PERIODIC_FREQUENCY;
  oneloop_nB.sta_nB_state.nB_2d.y = (oneloop_nB.sta_nB_state.nB_d.y - nB_d_prev.y) * PERIODIC_FREQUENCY;
  oneloop_nB.sta_nB_state.nB_2d.z = (oneloop_nB.sta_nB_state.nB_d.z - nB_d_prev.z) * PERIODIC_FREQUENCY;
  nB_d_prev.x = oneloop_nB.sta_nB_state.nB_d.x;
  nB_d_prev.y = oneloop_nB.sta_nB_state.nB_d.y;
  nB_d_prev.z = oneloop_nB.sta_nB_state.nB_d.z;
#else   
  float_rmat_vmult(&A, &Omega_B_dot, &oneloop_nB.sta_nB_state.nB);
  float_rmat_vmult(&B, &Omega_B, &oneloop_nB.sta_nB_state.nB_d);
  float_rmat_vmult(&temp, LBI, &oneloop_nB.sta_nB_state.nI_d);
  float_rmat_vmult(&C, &Omega_B, &temp);
  float_rmat_vmult(&D, LBI, &oneloop_nB.sta_nB_state.nI_2d);
  VECT3_SUB(oneloop_nB.sta_nB_state.nB_2d, A);
  VECT3_SUB(oneloop_nB.sta_nB_state.nB_2d, B);
  VECT3_SUB(oneloop_nB.sta_nB_state.nB_2d, C);
  VECT3_ADD(oneloop_nB.sta_nB_state.nB_2d, D); 
#endif
}
void nB_EC(struct FloatVect3 nB, struct FloatVect3 nB_d, struct FloatVect3 nB_2d, struct FloatVect3 mu_B, float k1_e[3], float k2_e[3], float k3_e[3], float dist[3], float nB_nu[3])
{
  float nB_d_des[3];
  nB_d_des[0] = k1_e[0]*(mu_B.x-nB.x);
  nB_d_des[1] = k1_e[1]*(mu_B.y-nB.y);
  nB_d_des[2] = k1_e[2]*(mu_B.z-nB.z);
  float nB_2d_des[3];
  nB_2d_des[0] = k2_e[0]*(nB_d_des[0]-nB_d.x);
  nB_2d_des[1] = k2_e[1]*(nB_d_des[1]-nB_d.y);
  nB_2d_des[2] = k2_e[2]*(nB_d_des[2]-nB_d.z);
  
  nB_nu[0] = k3_e[0]*(nB_2d_des[0]-nB_2d.x) + dist[0];
  nB_nu[1] = k3_e[1]*(nB_2d_des[1]-nB_2d.y) + dist[1];
  nB_nu[2] = 0.0;//k3_e[2]*(nB_2d_des[2]-nB_2d.z) + dist[2];
  //printf("nB_nu function: %f, %f, %f\n", nB_nu[0], nB_nu[1], nB_nu[2]);
}

void skew_symmetric(struct FloatRMat *out, const struct FloatVect3 *in)
{
  // Row 0
  out->m[0] = 0.0f;      // (0,0)
  out->m[1] = -in->z;    // (0,1)
  out->m[2] =  in->y;    // (0,2)
  // Row 1
  out->m[3] =  in->z;    // (1,0)
  out->m[4] =  0.0f;     // (1,1)
  out->m[5] = -in->x;    // (1,2)
  // Row 2
  out->m[6] = -in->y;    // (2,0)
  out->m[7] =  in->x;    // (2,1)
  out->m[8] =  0.0f;     // (2,2)
}

void rm_3rd_nI(float dt,struct FloatVect3 *x_ref,struct FloatVect3 *x_d_ref,struct FloatVect3 *x_2d_ref,struct FloatVect3 *x_3d_ref,const struct FloatVect3 *x_des,const float k1_rm[3],const float k2_rm[3],const float k3_rm[3])
{
  //printf("RM 3rd nI\n");
  //printf("x_ref: %f, %f, %f\n", x_ref->x, x_ref->y, x_ref->z);
  //printf("x_d_ref: %f, %f, %f\n", x_d_ref->x, x_d_ref->y, x_d_ref->z);
  //printf("x_2d_ref: %f, %f, %f\n", x_2d_ref->x, x_2d_ref->y, x_2d_ref->z);   
  //printf("x_3d_ref: %f, %f, %f\n", x_3d_ref->x, x_3d_ref->y, x_3d_ref->z);
  //printf("x_des: %f, %f, %f\n", x_des->x, x_des->y, x_des->z);

  float e_x[3];
  float e_x_d[3];
  float e_x_2d[3];

  e_x[0]    = k1_rm[0] * (x_des->x - x_ref->x);
  e_x[1]    = k1_rm[1] * (x_des->y - x_ref->y);
  e_x[2]    = k1_rm[2] * (x_des->z - x_ref->z);

  e_x_d[0]  = k2_rm[0] * (e_x[0] - x_d_ref->x);
  e_x_d[1]  = k2_rm[1] * (e_x[1] - x_d_ref->y);
  e_x_d[2]  = k2_rm[2] * (e_x[2] - x_d_ref->z);

  e_x_2d[0] = k3_rm[0] * (e_x_d[0] - x_2d_ref->x);
  e_x_2d[1] = k3_rm[1] * (e_x_d[1] - x_2d_ref->y);
  e_x_2d[2] = k3_rm[2] * (e_x_d[2] - x_2d_ref->z);

  //printf("e_x   : %f, %f, %f\n", e_x[0], e_x[1], e_x[2]);
  //printf("e_x_d : %f, %f, %f\n", e_x_d[0], e_x_d[1], e_x_d[2]);
  //printf("e_x_2d: %f, %f, %f\n", e_x_2d[0], e_x_2d[1], e_x_2d[2]);

  // 3rd derivative
  x_3d_ref->x = e_x_2d[0];
  x_3d_ref->y = e_x_2d[1];
  x_3d_ref->z = e_x_2d[2];

  // integrate jerk -> acceleration
  x_2d_ref->x += dt * x_3d_ref->x;
  x_2d_ref->y += dt * x_3d_ref->y;
  x_2d_ref->z += dt * x_3d_ref->z;

  // integrate acceleration -> velocity
  x_d_ref->x  += dt * x_2d_ref->x;
  x_d_ref->y  += dt * x_2d_ref->y;
  x_d_ref->z  += dt * x_2d_ref->z;

  // integrate velocity -> position
  x_ref->x    += dt * x_d_ref->x;
  x_ref->y    += dt * x_d_ref->y;
  x_ref->z    += dt * x_d_ref->z;

  //printf("Updated x_ref   : %f, %f, %f\n", x_ref->x, x_ref->y, x_ref->z);
  //printf("Updated x_d_ref : %f, %f, %f\n", x_d_ref->x, x_d_ref->y, x_d_ref->z);
  //printf("Updated x_2d_ref: %f, %f, %f\n", x_2d_ref->x, x_2d_ref->y, x_2d_ref->z);   
  //printf("Updated x_3d_ref: %f, %f, %f\n", x_3d_ref->x, x_3d_ref->y, x_3d_ref->z);
}


void calc_HB_matrix(float HB[3][3], struct FloatVect3 nB){
  HB[0][0] =  0.0;
  HB[0][1] = -nB.z;
  HB[0][2] =  nB.y;
  HB[1][0] =  nB.z;
  HB[1][1] =  0.0;
  HB[1][2] = -nB.x;
  HB[2][0] = -nB.y;
  HB[2][1] =  nB.x;
  HB[2][2] =  0.0;
}

void SpinQuad_overwrite(float gain, float ce_model, float *nu_stab_2)
{
  SQ_r = 0.0;

  if (SpinQuad){
    SQ_r = (float) radio_control.values[RADIO_AUX4] / MAX_PPRZ * 40.0;
    Bound(SQ_r, 0.0, 40.0);

    *nu_stab_2 = (SQ_r - oneloop_nB.sta_state.att_d[2]) * k_att_e.k2[2];
    BoundAbs(*nu_stab_2, sta_bounds.att_2d[2]);

    *nu_stab_2 = (*nu_stab_2 - oneloop_nB.sta_state.att_2d[2]) * gain;
    *nu_stab_2 += ce_model;
  }
}

