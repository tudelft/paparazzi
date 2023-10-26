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
 * One loop (Guidance + Stabilization) ANDI controller for the rotating wing drone RW3C
 */

/* Include necessary header files */
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"
#include "firmwares/rotorcraft/oneloop/navigation_oneloop.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#ifdef   STABILIZATION_ANDI_ROTWING_V3A
#include "modules/rot_wing_drone/wing_rotation_controller_v3a.h"
#endif
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "wls/wls_alloc.h"
#include <stdio.h>


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

#ifdef ONELOOP_ANDI_FILT_CUTOFF_POS
float  oneloop_andi_filt_cutoff_p = ONELOOP_ANDI_FILT_CUTOFF_POS;
#else
float  oneloop_andi_filt_cutoff_p = 2.0;
#endif

#ifdef  ONELOOP_ANDI_FILT_CUTOFF_P
#define ONELOOP_ANDI_FILTER_ROLL_RATE TRUE
#else
#define ONELOOP_ANDI_FILT_CUTOFF_P 20.0
#endif

#ifdef  ONELOOP_ANDI_FILT_CUTOFF_Q
#define ONELOOP_ANDI_FILTER_PITCH_RATE TRUE
#else
#define ONELOOP_ANDI_FILT_CUTOFF_Q 20.0
#endif

#ifdef  ONELOOP_ANDI_FILT_CUTOFF_R
#define ONELOOP_ANDI_FILTER_YAW_RATE TRUE
#else
#define ONELOOP_ANDI_FILT_CUTOFF_R 20.0
#endif

#ifdef ONELOOP_ANDI_ACT_IS_SERVO
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_IS_SERVO;
#else
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = {0};
#endif

#ifdef ONELOOP_ANDI_ACT_DYN
float  act_dynamics[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_DYN;
#else
float  act_dynamics[ANDI_NUM_ACT_TOT] = = {0};
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

#ifdef ONELOOP_ANDI_WV // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
static float Wv[ANDI_OUTPUTS]     = ONELOOP_ANDI_WV;
static float Wv_wls[ANDI_OUTPUTS] = ONELOOP_ANDI_WV;
#else
static float Wv[ANDI_OUTPUTS]     = {1.0};
static float Wv_wls[ANDI_OUTPUTS] = {1.0};
#endif

#ifdef ONELOOP_ANDI_WU // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
static float Wu[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_WU;
#else
static float Wu[ANDI_NUM_ACT_TOT] = {1.0};
#endif

#ifdef ONELOOP_ANDI_U_PREF
static float u_pref[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_U_PREF;
#else
static float u_pref[ANDI_NUM_ACT_TOT] = {0.0};
#endif

#ifndef ONELOOP_ANDI_DEBUG_MODE
printf("Debug Mode not defined\n");
#define ONELOOP_ANDI_DEBUG_MODE  FALSE;
#endif

#ifndef ONELOOP_ANDI_AC_HAS_PUSHER
printf("Did not specify if ac has a pusher\n");
#define ONELOOP_ANDI_AC_HAS_PUSHER  FALSE;
#endif

#ifndef ONELOOP_ANDI_PUSHER_IDX
printf("Did not specify pusher index\n");
#define ONELOOP_ANDI_PUSHER_IDX  4;
#endif
/* Declaration of Navigation Variables*/
float v_nav_des = 1.2;
float max_j_nav = 500.0; // Pusher Test shows erros above 2[Hz] ramp commands [0.6 SF]
float max_a_nav = 4.0;   // (35[N]/6.5[Kg]) = 5.38[m/s2]  [0.8 SF]
//float max_v_nav = 5.0;

#ifdef ONELOOP_MAX_AIRSPEED
float max_v_nav = ONELOOP_MAX_AIRSPEED; // Consider implications of difference Ground speed and airspeed
#else
float max_v_nav = 5.0;
#endif
// #define GUIDANCE_INDI_MAX_AIRSPEED max_v_nav; // hack until nav hybrid is modified to be used without guidance indi hybrid

// Delete once nav hybrid is fixed
#ifndef GUIDANCE_INDI_SPEED_GAIN
#define GUIDANCE_INDI_SPEED_GAIN 1.8
#define GUIDANCE_INDI_SPEED_GAINZ 1.8
#endif

#ifndef GUIDANCE_INDI_POS_GAIN
#define GUIDANCE_INDI_POS_GAIN 0.5
#define GUIDANCE_INDI_POS_GAINZ 0.5
#endif

#ifndef GUIDANCE_INDI_LIFTD_ASQ
#define GUIDANCE_INDI_LIFTD_ASQ 0.20
#endif

/* If lift effectiveness at low airspeed not defined,
 * just make one interpolation segment that connects to
 * the quadratic part from 12 m/s onward
 */
#ifndef GUIDANCE_INDI_LIFTD_P50
#define GUIDANCE_INDI_LIFTD_P80 (GUIDANCE_INDI_LIFTD_ASQ*12*12)
#define GUIDANCE_INDI_LIFTD_P50 (GUIDANCE_INDI_LIFTD_P80/2)
#endif

struct guidance_indi_hybrid_params gih_params = {
  .pos_gain = GUIDANCE_INDI_POS_GAIN,
  .pos_gainz = GUIDANCE_INDI_POS_GAINZ,

  .speed_gain = GUIDANCE_INDI_SPEED_GAIN,
  .speed_gainz = GUIDANCE_INDI_SPEED_GAINZ,

  .heading_bank_gain = 5.0,
  .liftd_asq = GUIDANCE_INDI_LIFTD_ASQ, // coefficient of airspeed squared
  .liftd_p80 = GUIDANCE_INDI_LIFTD_P80,
  .liftd_p50 = GUIDANCE_INDI_LIFTD_P50,
};
bool force_forward = false;


/*  Define Section of the functions used in this module*/
void  calc_normalization(void);
void  sum_g1g2_1l(void);
void  get_act_state_oneloop(void);
void  oneloop_andi_propagate_filters(void);
void  init_filter(void);
void  init_controller(void);
void  float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3]);
void  float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3]);
void  err_nd(float err[], float a[], float b[], float k[], int n);
void  integrate_nd(float dt, float a[], float a_dot[], int n);
void  vect_bound_nd(float vect[], float bound, int n);
void  rm_2nd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm);
void  rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3]);
void  rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n);
void  rm_2nd_pos(float dt, float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_d_des[], float k2_rm[], float k3_rm[], float x_2d_bound, float x_3d_bound, int n);
void  rm_1st_pos(float dt, float x_2d_ref[], float x_3d_ref[], float x_2d_des[], float k3_rm[], float x_3d_bound, int n);
void  ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3]);
void  calc_model(void);

/* Define messages of the module*/
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_oneloop_g(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ONELOOP_G(trans, dev, AC_ID,
                                        ANDI_NUM_ACT_TOT, g1g2_1l[0],
                                        ANDI_NUM_ACT_TOT, g1g2_1l[1],
                                        ANDI_NUM_ACT_TOT, g1g2_1l[2],
                                        ANDI_NUM_ACT_TOT, g1g2_1l[3],
                                        ANDI_NUM_ACT_TOT, g1g2_1l[4],
                                        ANDI_NUM_ACT_TOT, g1g2_1l[5]);
}
static void send_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ONELOOP_ANDI(trans, dev, AC_ID,
                                        &att_ref[0],
                                        &att_ref[1],
                                        &att_ref[2],
                                        &att_1l[0],
                                        &att_1l[1],
                                        &att_1l[2],
                                        &att_d_ref[0],
                                        &att_d_ref[1],
                                        &att_d_ref[2],
                                        &att_d[0],
                                        &att_d[1],
                                        &att_d[2],
                                        &att_2d_ref[0],
                                        &att_2d_ref[1],
                                        &att_2d_ref[2],
                                        &att_2d[0],
                                        &att_2d[1],
                                        &att_2d[2],
                                        ANDI_OUTPUTS, nu,
                                        ANDI_NUM_ACT, actuator_state_1l);
}
static void send_oneloop_guidance(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ONELOOP_GUIDANCE(trans, dev, AC_ID,
                                        &pos_ref[0],
                                        &pos_ref[1],
                                        &pos_ref[2],
                                        &pos_1l[0],
                                        &pos_1l[1],
                                        &pos_1l[2],
                                        &pos_d_ref[0],
                                        &pos_d_ref[1],
                                        &pos_d_ref[2],
                                        &pos_d[0],
                                        &pos_d[1],
                                        &pos_d[2],
                                        &pos_2d_ref[0],
                                        &pos_2d_ref[1],
                                        &pos_2d_ref[2],
                                        &pos_2d[0],
                                        &pos_2d[1],
                                        &pos_2d[2],
                                        &pos_3d_ref[0],
                                        &pos_3d_ref[1],
                                        &pos_3d_ref[2],
                                        ANDI_NUM_ACT_TOT,andi_u);
}
#endif
/* Oneloop Basic Variables*/
bool half_loop = true;
/*Physical Properties RW3C*/
float m   = 6.5;  // [kg] Mass Bebop 0.4
float g   = 9.81; // [m/s^2] Gravitational Acceleration
float Ixx = 0.251; // [kg m^2] Inertia around x axis
float Iyy = 1.071; // [kg m^2] Inertia around y axis
float Izz = 1.334; // [kg m^2] Inertia around z axis
float num_thrusters_1l = 4.0; // Number of motors used for thrust
/*Actuator Dynamics*/
float w_F = 15.23; // [rad/s] First order bandwidth of actuator
float w_B = 15.23; // [rad/s] First order bandwidth of actuator
float w_L = 12.66; // [rad/s] First order bandwidth of actuator
float w_R = 12.66; // [rad/s] First order bandwidth of actuator
float w_P = 24.07; // [rad/s] First order bandwidth of actuator
float w_phi = 1.0;     // [rad/s] First order bandwidth of actuator
float w_theta = 1.0;     // [rad/s] First order bandwidth of actuator

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

float gamma_wls                   = 1000.0;
float du_min_1l[ANDI_NUM_ACT_TOT]; 
float du_max_1l[ANDI_NUM_ACT_TOT];
float du_pref_1l[ANDI_NUM_ACT_TOT];
float model_pred[ANDI_OUTPUTS];
float old_state[ANDI_NUM_ACT_TOT];
int   number_iter = 0;

/*Declaration of Reference Model and Error Controller Gains*/
float rm_k_attitude = 0.8;
float p1_att = 7.68;//7.68; %12 works well for bebop1       
float p2_att;      
float p3_att;

float k_theta_e;    
float k_q_e;        
float k_qdot_e;     
float k_phi_e;      
float k_p_e;        
float k_pdot_e;      

float k_theta_rm;   
float k_q_rm;       
float k_qdot_rm;    
float k_phi_rm;     
float k_p_rm ;      
float k_pdot_rm ;    

/*Position Loop*/
float rm_k_pos;    
float damp_pos = 0.85;
float p1_pos   = 1.2;
float p2_pos   = 6.0;    
float p3_pos  ;     
float route_k ;   
float p1_pos_rm = 0.6;
float p2_pos_rm = 6.0; 
float damp_pos_rm = 1.0; //0.3979;//0.7;

float k_N_e    ;    
float k_vN_e   ;    
float k_aN_e   ;    
float k_E_e    ;    
float k_vE_e   ;   
float k_aE_e   ;     

float k_N_rm   ;    
float k_vN_rm  ;     
float k_aN_rm  ;    
float k_E_rm   ;    
float k_vE_rm  ;    
float k_aE_rm  ;    

float k_N_route ;   
float k_vN_route ; 
float k_aN_route ;  
float k_E_route  ;  
float k_vE_route ;  
float k_aE_route ;  

/*Altitude Loop*/
float rm_k_alt ;   
float damp_alt = 0.7;//0.7; 
float p1_alt   = 3.0;  
float p1_alt_2 = 3.0;//0.5;  
float p2_alt   ;    
float p3_alt   ;     

float k_D_e  ;      
float k_vD_e  ;     
float k_aD_e  ;     

float k_D_rm  ;     
float k_vD_rm ;     
float k_aD_rm ;     

/*Heading Loop*/
float rm_k_head  ;  
float p1_head = 1.5;//5.0;     //3.0 works well for bebop1
float p2_head ;
float p3_head ;      

float k_r_e  ;      
float k_r_d_e ;     
float k_psi_e     ;  
float k_psi_d_e   ;  
float k_psi_2d_e  ;  
    
float k_r_rm  ;     
float k_r_d_rm ;   
float k_psi_rm    ;  
float k_psi_d_rm  ;  
float k_psi_2d_rm ;

float psi_des_rad = 0.0;
float psi_des_deg = 0.0;
float psi_vec[4] = {0.0, 0.0, 0.0, 0.0};

float use_increment = 0.0;
float a_thrust = 0.0;


/*Define Variables used in control*/
float old_time = 0.0;
float new_time = 0.0;
float dt_actual = 1./PERIODIC_FREQUENCY;
struct Int32Eulers stab_att_sp_euler_1l;
struct Int32Quat   stab_att_sp_quat_1l;
float act_dynamics_d[ANDI_NUM_ACT_TOT];
float actuator_state_1l[ANDI_NUM_ACT];

float ang_acc[3];
float lin_acc[3];
float andi_u[ANDI_NUM_ACT_TOT];
float andi_du[ANDI_NUM_ACT_TOT];
float andi_du_n[ANDI_NUM_ACT_TOT];
float g2_times_du_1l;
float dt_1l = 1./PERIODIC_FREQUENCY;
bool  rc_on = true;
struct FloatEulers eulers_zxy_des;
float nav_target[3]; // Can be a position, speed or acceleration depending on the guidance H mode
float pos_init[3];
float att_ref[3];
float att_d_ref[3];
float att_2d_ref[3];
float att_3d_ref[3];
float pos_ref[3];
float pos_d_ref[3];
float pos_2d_ref[3];
float pos_3d_ref[3];

float att_1l[3];
float att_d[3]; 
float att_2d[3];
float pos_1l[3];
float pos_d[3];
float pos_2d[3];

float nu[ANDI_OUTPUTS];

float g2_1l[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_G2; //scaled by INDI_G_SCALING
float g1_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT] = {ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_THRUST, ONELOOP_ANDI_G1_ROLL, ONELOOP_ANDI_G1_PITCH, ONELOOP_ANDI_G1_YAW};  
float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
float thrust_eff[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_G1_THRUST;

float ratio_u_un[ANDI_NUM_ACT_TOT];
float ratio_vn_v[ANDI_NUM_ACT_TOT];
float w_scale = 1.0;

float *bwls_1l[ANDI_OUTPUTS];

bool check_1st_nav = true; 
bool verbose_oneloop = true;
bool heading_on = false;

/** state eulers in zxy order */
struct FloatEulers eulers_zxy;

/*Filters Initialization*/
float oneloop_andi_estimation_filt_cutoff = 2.0;
static struct FirstOrderLowPass filt_accel_ned[3];
Butterworth2LowPass filt_accel_body[3];
Butterworth2LowPass roll_filt_1l;
Butterworth2LowPass pitch_filt_1l;
Butterworth2LowPass yaw_filt;
Butterworth2LowPass att_dot_meas_lowpass_filters[3];
Butterworth2LowPass att_ref_lowpass_filters[3];
Butterworth2LowPass rate_ref_lowpass_filters[3];
Butterworth2LowPass measurement_lowpass_filters_1l[3];
Butterworth2LowPass estimation_output_lowpass_filters_1l[3];
Butterworth2LowPass model_pred_filt[ANDI_OUTPUTS];
static struct FirstOrderLowPass rates_filt_fo[3];
static struct FirstOrderLowPass pos_filt_fo[3];
static struct FirstOrderLowPass vel_filt_fo[3];
static struct FirstOrderLowPass model_pred_a_filt[3];
struct FloatVect3 body_accel_f_1l;

/** @brief  Error Controller Gain Design */
static float k_e_1_3_f(float p1, float p2, float p3) {return (p1*p2*p3);}
static float k_e_2_3_f(float p1, float p2, float p3) {return (p1*p2+p1*p3+p2*p3);}
static float k_e_3_3_f(float p1, float p2, float p3) {return (p1+p2+p3);}
static float k_e_1_2_f(float p1, float p2) {return (p1*p2);}
static float k_e_2_2_f(float p1, float p2) {return (p1+p2);}

static float k_e_1_3_f_v2(float omega_n, UNUSED float zeta, float p1) {return (omega_n*omega_n*p1);}
static float k_e_2_3_f_v2(float omega_n,        float zeta, float p1) {return (omega_n*omega_n+2*zeta*omega_n*p1);}
static float k_e_3_3_f_v2(float omega_n,        float zeta, float p1) {return (2*zeta*omega_n+p1);}

/** @brief Reference Model Gain Design */
static float k_rm_1_3_f(float omega_n, float zeta, float p1) {return (omega_n*omega_n*p1)/(omega_n*omega_n+omega_n*p1*zeta*2.0);}
static float k_rm_2_3_f(float omega_n, float zeta, float p1) {return (omega_n*omega_n+omega_n*p1*zeta*2.0)/(p1+omega_n*zeta*2.0);}
static float k_rm_3_3_f(float omega_n, float zeta, float p1) {return p1+omega_n*zeta*2.0;}
static float k_rm_1_2_f(float omega_n, float zeta) {return omega_n/(2.0*zeta);}
static float k_rm_2_2_f(float omega_n, float zeta) {return 2.0*zeta*omega_n;}

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
  if((norm-bound) > FLT_EPSILON) {
    float scale = bound/norm;
    int8_t i;
    for(i = 0; i < n; i++) {
      vect[i] *= scale;
    }
  }
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
 * FIXME: 
 */
void rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3]){
  float e_x[3];
  float e_x_rates[3];
  float e_x_d[3];
  float e_x_2d[3];
  float x_d_eul_ref[3];
  err_nd(e_x, x_des, x_ref, k1_rm, 3);
  e_x[2] = k1_rm[2] * convert_angle((x_des[2] - x_ref[2])); // Correction for Heading error +-Pi
  float_rates_of_euler_dot_vec(e_x_rates, x_ref, e_x);
  err_nd(e_x_d, e_x_rates, x_d_ref, k2_rm, 3);
  err_nd(e_x_2d, e_x_d, x_2d_ref, k3_rm, 3);
  float_vect_copy(x_3d_ref,e_x_2d,3);
  if(ow_psi){x_3d_ref[2] = psi_overwrite[3];}
  integrate_nd(dt, x_2d_ref, x_3d_ref, 3);
  if(ow_psi){x_2d_ref[2] = psi_overwrite[2];}
  integrate_nd(dt, x_d_ref, x_2d_ref, 3);
  if(ow_psi){x_d_ref[2] = psi_overwrite[1];}
  float_euler_dot_of_rates_vec(x_d_ref, x_ref, x_d_eul_ref);
  integrate_nd(dt, x_ref, x_d_eul_ref, 3);
  if(ow_psi){x_ref[2] = psi_overwrite[0];}
  x_ref[2] = convert_angle(x_ref[2]);
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
 * FIXME: 
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
 * FIXME: 
 */
void rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm){
  float e_x      = k1_rm * convert_angle((x_des- *x_ref));
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
 * FIXME: 
 */
void rm_3rd_pos(float dt, float x_ref[], float x_d_ref[], float x_2d_ref[], float x_3d_ref[], float x_des[], float k1_rm[], float k2_rm[], float k3_rm[], float x_d_bound, float x_2d_bound, float x_3d_bound, int n){
  float e_x[n];
  float e_x_d[n];
  float e_x_2d[n];
  err_nd(e_x, x_des, x_ref, k1_rm, n);
  vect_bound_nd(e_x,x_d_bound, n);
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
 * FIXME: 
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
 * FIXME: 
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
 * FIXME: 
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
 * FIXME: 
 */
static float ec_3rd(float x_ref, float x_d_ref, float x_2d_ref, float x_3d_ref, float x, float x_d, float x_2d, float k1_e, float k2_e, float k3_e){
  float y_4d = k1_e*(x_ref-x)+k2_e*(x_d_ref-x_d)+k3_e*(x_2d_ref-x_2d)+x_3d_ref;
  return y_4d;
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
 * FIXME: 
 */
void ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3]){
  float y_4d_1[3];
  float y_4d_2[3];
  float y_4d_3[3];
  err_nd(y_4d_1, x_ref, x, k1_e, 3);
  y_4d_1[2] = k1_e[2] * convert_angle((x_ref[2] - x[2])); // Correction for Heading error +-Pi
  err_nd(y_4d_2, x_d_ref, x_d, k2_e, 3);
  err_nd(y_4d_3, x_2d_ref, x_2d, k3_e, 3);
  float_vect_copy(y_4d,x_3d_ref,3);
  float_vect_sum(y_4d, y_4d, y_4d_1, 3);
  float_vect_sum(y_4d, y_4d, y_4d_2, 3);
  float_vect_sum(y_4d, y_4d, y_4d_3, 3);
}

/** 
 * @brief Error Controller Definition for 2rd order system 
 * @param dt              Delta time [s]
 * @param x_ref           Reference signal 1st order
 * @param x_d_ref         Reference signal 2nd order
 * @param x_2d_ref        Reference signal 3rd order
 * @param x_des           Desired 1st order signal
 * @param x               Current 1st order signal
 * @param x_d             Current 2nd order signal
 * @param k1_e            Error Controller Gain 1st order signal
 * @param k2_e            Error Controller Gain 2nd order signal
 * FIXME: 
 */
static float ec_2rd(float x_ref, float x_d_ref, float x_2d_ref, float x, float x_d, float k1_e, float k2_e){
  float y_3d = k1_e*(x_ref-x)+k2_e*(x_d_ref-x_d)+x_2d_ref;
  return y_3d;
}


/**
 * @brief  Third Order to First Order Dynamics Approximation
 * @param p1              Pole 1
 * @param p2              Pole 2
 * @param p3              Pole 3
 * @param rm_k            Reference Model Gain
 * FIXME: 
 */
static float w_approx(float p1, float p2, float p3, float rm_k){
  float tao = (p1*p2+p1*p3+p2*p3)/(p1*p2*p3)/(rm_k);
  return 1.0*w_scale/tao;
}

/** 
 * @brief Initialize Controller Gains
 * FIXME: Calculate the gains dynamically for transition
 */
void init_controller(void){
/*Attitude Loop*/
p2_att        = p1_att;
p3_att        = p1_att;
k_theta_e     = k_e_1_3_f(p1_att,p2_att,p3_att);
k_q_e         = k_e_2_3_f(p1_att,p2_att,p3_att);
k_qdot_e      = k_e_3_3_f(p1_att,p2_att,p3_att);
k_phi_e       = k_theta_e;
k_p_e         = k_q_e;
k_pdot_e      = k_qdot_e;
k_theta_rm    = k_rm_1_3_f(rm_k_attitude*p1_att,1.0,rm_k_attitude*p3_att);
k_q_rm        = k_rm_2_3_f(rm_k_attitude*p1_att,1.0,rm_k_attitude*p3_att);
k_qdot_rm     = k_rm_3_3_f(rm_k_attitude*p1_att,1.0,rm_k_attitude*p3_att);
k_phi_rm      = k_theta_rm;
k_p_rm        = k_q_rm;
k_pdot_rm     = k_qdot_rm;

/*Position Loop*/
rm_k_pos      = 0.9;
p3_pos        = p1_pos;
route_k       = 0.8;
k_N_e         = k_e_1_3_f_v2(p1_pos/damp_pos,damp_pos,p2_pos);
k_vN_e        = k_e_2_3_f_v2(p1_pos/damp_pos,damp_pos,p2_pos);
k_aN_e        = k_e_3_3_f_v2(p1_pos/damp_pos,damp_pos,p2_pos);
k_E_e         = k_N_e;
k_vE_e        = k_vN_e;
k_aE_e        = k_aN_e;
k_N_rm        = k_rm_1_3_f(p1_pos_rm/damp_pos_rm, damp_pos_rm, p2_pos_rm);
k_vN_rm       = k_rm_2_3_f(p1_pos_rm/damp_pos_rm, damp_pos_rm, p2_pos_rm);
k_aN_rm       = k_rm_3_3_f(p1_pos_rm/damp_pos_rm, damp_pos_rm, p2_pos_rm);
k_E_rm        = k_N_rm;
k_vE_rm       = k_vN_rm;
k_aE_rm       = k_aN_rm;

k_N_route    = k_rm_1_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
k_vN_route   = k_rm_2_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
k_aN_route   = k_rm_3_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
k_E_route    = k_N_route;
k_vE_route   = k_vN_route;
k_aE_route   = k_aN_route;
gih_params.pos_gain   = k_N_rm;  //delete once nav hybrid is fixed
gih_params.speed_gain = k_vN_rm; //delete once nav hybrid is fixed
/*Altitude Loop*/
rm_k_alt      = 0.9;
p2_alt        = 6.0;
p3_alt        = p1_alt;
k_D_e         = k_e_1_3_f_v2(p1_alt/damp_alt,damp_alt,p2_alt);
k_vD_e        = k_e_2_3_f_v2(p1_alt/damp_alt,damp_alt,p2_alt);
k_aD_e        = k_e_3_3_f_v2(p1_alt/damp_alt,damp_alt,p2_alt);
k_D_rm        = k_rm_1_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);
k_vD_rm       = k_rm_2_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);
k_aD_rm       = k_rm_3_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);
gih_params.pos_gainz   = k_D_rm;
gih_params.speed_gainz = k_vD_rm;

/*Heading Loop Manual*/
rm_k_head     = 0.9;
p2_head       = p1_head;
k_r_e         = k_e_1_2_f(p1_head,p2_head);
k_r_d_e       = k_e_2_2_f(p1_head,p2_head);
k_r_rm        = k_rm_1_2_f(rm_k_head*p1_head,rm_k_head*p2_head);
k_r_d_rm      = k_rm_2_2_f(rm_k_head*p1_head,rm_k_head*p2_head);

/*Heading Loop NAV*/
rm_k_head     = 0.9;
p2_head       = p1_head;
p3_head       = p1_head;
k_psi_e       = k_e_1_3_f(p1_head,p2_head,p3_head);
k_psi_d_e     = k_e_2_3_f(p1_head,p2_head,p3_head);
k_psi_2d_e    = k_e_3_3_f(p1_head,p2_head,p3_head);
k_psi_rm      = k_rm_1_3_f(rm_k_head*p1_head,1.0,rm_k_head*p3_head);
k_psi_d_rm    = k_rm_2_3_f(rm_k_head*p1_head,1.0,rm_k_head*p3_head);
k_psi_2d_rm   = k_rm_3_3_f(rm_k_head*p1_head,1.0,rm_k_head*p3_head);

/*Approximated Dynamics*/
act_dynamics[ANDI_NUM_ACT]   = w_approx( p1_att,  p2_att,  p3_att,  1.0);//rm_k_attitude);
act_dynamics[ANDI_NUM_ACT+1]   = w_approx( p1_att,  p2_att,  p3_att,  1.0);//rm_k_attitude);
}


/** @brief  Initialize the filters */
void init_filter(void)
{
  float tau = 1.0 / (2.0 * M_PI *oneloop_andi_filt_cutoff);
  float tau_a = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_a);
  float tau_v = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_v);
  float tau_p = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_p);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  // Filtering of the Inputs with 3 dimensions (e.g. rates and accelerations)
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], tau, sample_time, 0.0);
    init_first_order_low_pass(&filt_accel_ned[i], tau_a, sample_time, 0.0 );
    init_butterworth_2_low_pass(&filt_accel_body[i], tau_a, sample_time, 0.0);
    init_butterworth_2_low_pass(&att_ref_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&rate_ref_lowpass_filters[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt_1l, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt_1l, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&yaw_filt, tau, sample_time, 0.0);

  // Init rate filter for feedback
  float time_constants[3] = {1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_P), 1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_Q), 1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_R)};
  init_first_order_low_pass(&rates_filt_fo[0], time_constants[0], sample_time, stateGetBodyRates_f()->p);
  init_first_order_low_pass(&rates_filt_fo[1], time_constants[1], sample_time, stateGetBodyRates_f()->q);
  init_first_order_low_pass(&rates_filt_fo[2], time_constants[2], sample_time, stateGetBodyRates_f()->r);
  
  // Init pos and vel filters
  init_first_order_low_pass(&pos_filt_fo[0], tau_p, sample_time, stateGetPositionNed_f()->x );
  init_first_order_low_pass(&pos_filt_fo[1], tau_p, sample_time, stateGetPositionNed_f()->y );
  init_first_order_low_pass(&pos_filt_fo[2], tau_p, sample_time, stateGetPositionNed_f()->z );
  init_first_order_low_pass(&vel_filt_fo[0], tau_v, sample_time, stateGetSpeedNed_f()->x );
  init_first_order_low_pass(&vel_filt_fo[1], tau_v, sample_time, stateGetSpeedNed_f()->y );
  init_first_order_low_pass(&vel_filt_fo[2], tau_v, sample_time, stateGetSpeedNed_f()->z );
  
  // Remember to change the time constant if you provide different P Q R filters
  for (i = 0; i < ANDI_OUTPUTS; i++){
    if (i < 3){
       init_butterworth_2_low_pass(&model_pred_filt[i], tau_a, sample_time, 0.0);
       init_first_order_low_pass(&model_pred_a_filt[i], tau_a, sample_time, 0.0);      
    } else {
    init_butterworth_2_low_pass(&model_pred_filt[i], tau, sample_time, 0.0);
    }
  }
}


/** @brief  Propagate the filters */
void oneloop_andi_propagate_filters(void) {
  struct  NedCoor_f *accel = stateGetAccelNed_f();
  float   accel_b_x = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->x);
  float   accel_b_y = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  float   accel_b_z = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
  struct  FloatRates *body_rates = stateGetBodyRates_f();
  float   rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};
  float   pos_vect[3]  = {stateGetPositionNed_f()->x,stateGetPositionNed_f()->y,stateGetPositionNed_f()->z};
  float   vel_vect[3]  = {stateGetSpeedNed_f()->x,stateGetSpeedNed_f()->y,stateGetSpeedNed_f()->z};
  update_first_order_low_pass(&filt_accel_ned[0], accel->x);
  update_first_order_low_pass(&filt_accel_ned[1], accel->y);
  update_first_order_low_pass(&filt_accel_ned[2], accel->z);
  update_butterworth_2_low_pass(&roll_filt_1l, eulers_zxy.phi);
  update_butterworth_2_low_pass(&pitch_filt_1l, eulers_zxy.theta);
  update_butterworth_2_low_pass(&yaw_filt, eulers_zxy.psi);
  update_butterworth_2_low_pass(&filt_accel_body[0], accel_b_x);
  update_butterworth_2_low_pass(&filt_accel_body[1], accel_b_y);
  update_butterworth_2_low_pass(&filt_accel_body[2], accel_b_z);
  
  calc_model();
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++){
    update_butterworth_2_low_pass(&model_pred_filt[i], model_pred[i]);
  }
  
  for (i = 0; i < 3; i++) {
    update_first_order_low_pass(&model_pred_a_filt[i], model_pred[i]);
    update_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], rate_vect[i]);
    update_first_order_low_pass(&rates_filt_fo[i], rate_vect[i]);
    update_first_order_low_pass(&pos_filt_fo[i], pos_vect[i]);
    update_first_order_low_pass(&vel_filt_fo[i], vel_vect[i]);
 
    ang_acc[i] = (att_dot_meas_lowpass_filters[i].o[0]- att_dot_meas_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY + model_pred[3+i] - model_pred_filt[3+i].o[0];
    lin_acc[i] = filt_accel_ned[i].last_out + model_pred[i] - model_pred_a_filt[i].last_out;     
}}

/** @brief Init function of Oneloop ANDI controller  */
void oneloop_andi_init(void)
{ 
  half_loop = true;
  // Initialize Effectiveness matrix
  calc_normalization();
  sum_g1g2_1l();
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++) {
   bwls_1l[i] = g1g2_1l[i];
  }

  // Initialize filters and other variables
  init_filter();
  init_controller();
  float_vect_zero(andi_u, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du_n, ANDI_NUM_ACT_TOT);
  float_vect_zero(actuator_state_1l,ANDI_NUM_ACT);
  float_vect_zero(att_ref,3);
  float_vect_zero(att_d_ref,3);
  float_vect_zero(att_2d_ref,3);
  float_vect_zero(att_3d_ref,3);
  float_vect_zero(nu, ANDI_OUTPUTS);
  float_vect_zero(ang_acc,3);
  float_vect_zero(lin_acc,3);
  float_vect_zero(nav_target,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;
  float_vect_zero(model_pred,ANDI_OUTPUTS);

  // Start telemetry
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ONELOOP_ANDI, send_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ONELOOP_G, send_oneloop_g);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ONELOOP_GUIDANCE, send_oneloop_guidance);
  #endif
}

/**
 * @brief Function that resets important values upon engaging Oneloop ANDI.
 * FIXME: Ideally we should distinguish between the "stabilization" and "guidance" needs because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 */
void oneloop_andi_enter(bool half_loop_sp)
{
  half_loop = half_loop_sp;
  calc_normalization();
  sum_g1g2_1l();
  init_filter();
  init_controller();
  /* Stabilization Reset */
  float_vect_zero(att_ref,2);
  float_vect_zero(att_d_ref,3);
  float_vect_zero(att_2d_ref,3);
  float_vect_zero(att_3d_ref,3);
  float_vect_zero(ang_acc,3);
  float_vect_zero(lin_acc,3);
  float_vect_zero(nav_target,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;
  float_vect_zero(model_pred,ANDI_OUTPUTS);
  /*Guidance Reset*/
}

/**
 * @brief  Function to generate the reference signals for the oneloop controller
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * FIXME: 
 */
void oneloop_andi_RM(bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v)
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
    Wv_wls[0] = 0.0;
    Wv_wls[1] = 0.0;
    // Overwrite references with actual signals (for consistent plotting)
    float_vect_copy(pos_ref,pos_1l,3);
    float_vect_copy(pos_d_ref,pos_d,3);
    float_vect_copy(pos_2d_ref,pos_2d,3);
    float_vect_zero(pos_3d_ref,3);
    float_vect_copy(pos_init,pos_1l,3);
    // Set desired attitude with stick input
    eulers_zxy_des.phi   = (float) (radio_control_get(RADIO_ROLL))/MAX_PPRZ*45.0*M_PI/180.0;
    eulers_zxy_des.theta = (float) (radio_control_get(RADIO_PITCH))/MAX_PPRZ*45.0*M_PI/180.0;
    eulers_zxy_des.psi   = eulers_zxy.psi;
    psi_des_rad          = eulers_zxy.psi;
    // Initialize some variables
    check_1st_nav  = true;
    // Create commands adhoc to get actuators to the wanted level
    thrust_cmd_1l = (float) radio_control_get(RADIO_THROTTLE);
    Bound(thrust_cmd_1l,0.0,MAX_PPRZ); 
    int8_t i;
    for (i = 0; i < ANDI_NUM_ACT; i++) {
     a_thrust +=(thrust_cmd_1l - use_increment*actuator_state_1l[i]) * bwls_1l[2][i] / (ratio_u_un[i] * ratio_vn_v[i]);
    }
    // Overwrite Yaw Ref with desired Yaw setting (for consistent plotting)
    att_ref[2] = psi_des_rad;
    // Set desired Yaw rate with stick input
    des_r = (float) (radio_control_get(RADIO_YAW))/MAX_PPRZ*3.0; 
    BoundAbs(des_r,3.0);
    // Generate reference signals
    rm_2nd(dt_1l, &att_d_ref[2], &att_2d_ref[2], &att_3d_ref[2], des_r, k_r_rm, k_r_d_rm);
    rm_3rd(dt_1l, &att_ref[0],   &att_d_ref[0],  &att_2d_ref[0], &att_3d_ref[0], eulers_zxy_des.phi, k_phi_rm, k_p_rm, k_pdot_rm);
    rm_3rd(dt_1l, &att_ref[1],   &att_d_ref[1],  &att_2d_ref[1], &att_3d_ref[1], eulers_zxy_des.theta, k_theta_rm, k_q_rm, k_qdot_rm);
  }else{
    // Make sure X and Y jerk objectives are active
    Wv_wls[0] = Wv[0];
    Wv_wls[1] = Wv[1];
    // Group RM gains in arrays
    float k1_pos_rm[3] = {k_N_rm,  k_E_rm,  k_D_rm};
    float k2_pos_rm[3] = {k_vN_rm, k_vE_rm, k_vD_rm};
    float k3_pos_rm[3] = {k_aN_rm, k_aE_rm, k_aD_rm};
    float k1_att_rm[3] = {k_phi_rm, k_theta_rm, k_psi_rm};
    float k2_att_rm[3] = {k_p_rm, k_q_rm, k_psi_d_rm};
    float k3_att_rm[3] = {k_pdot_rm, k_qdot_rm, k_psi_2d_rm};
    // First time engaging NAV Mode or Navigation Functions (e.g. Oval) requires some initialization
    if(check_1st_nav){ 
      // Mark that NAV Mode has been engaged
      check_1st_nav  = false;
      // Reset desired heading to current heading
      psi_des_rad = eulers_zxy.psi; 
    }
    // Register Arttitude Setpoints from previous loop
    float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, psi_des_rad};
    // Generate Reference signals for positioning using RM
    if (rm_order_h == 3){
      rm_3rd_pos(dt_1l, pos_ref, pos_d_ref, pos_2d_ref, pos_3d_ref, nav_target, k1_pos_rm, k2_pos_rm, k3_pos_rm, max_v_nav, max_a_nav, max_j_nav, 2);    
    } else if (rm_order_h == 2){
      float_vect_copy(pos_ref, pos_1l,2);
      rm_2nd_pos(dt_1l, pos_d_ref, pos_2d_ref, pos_3d_ref, nav_target, k2_pos_rm, k3_pos_rm, max_a_nav, max_j_nav, 2);   
    } else if (rm_order_h == 1){
      float_vect_copy(pos_ref, pos_1l,2);
      float_vect_copy(pos_d_ref, pos_d,2);
      rm_1st_pos(dt_1l, pos_2d_ref, pos_3d_ref, nav_target, k3_pos_rm, max_j_nav, 2);   
    }

    // The RM functions want an array as input. Create a single entry array and write the vertical guidance entries. 
    float single_value_ref[1]        = {pos_ref[2]};
    float single_value_d_ref[1]      = {pos_d_ref[2]};
    float single_value_2d_ref[1]     = {pos_2d_ref[2]};
    float single_value_3d_ref[1]     = {pos_3d_ref[2]};
    float single_value_nav_target[1] = {nav_target[2]};
    float single_value_k1_rm[1]      = {k1_pos_rm[2]};
    float single_value_k2_rm[1]      = {k2_pos_rm[2]};
    float single_value_k3_rm[1]      = {k3_pos_rm[2]};

    if (rm_order_v == 3){
      rm_3rd_pos(dt_1l, single_value_ref, single_value_d_ref, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k1_rm, single_value_k2_rm, single_value_k3_rm, max_v_nav, max_a_nav, max_j_nav, 1);    
      pos_ref[2]    = single_value_ref[0];
      pos_d_ref[2]  = single_value_d_ref[0];
      pos_2d_ref[2] = single_value_2d_ref[0];
      pos_3d_ref[2] = single_value_3d_ref[0];
    } else if (rm_order_v == 2){
      rm_2nd_pos(dt_1l, single_value_d_ref, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k2_rm, single_value_k3_rm, max_a_nav, max_j_nav, 1);   
      pos_ref[2]    = pos_1l[2];
      pos_d_ref[2]  = single_value_d_ref[0];
      pos_2d_ref[2] = single_value_2d_ref[0];
      pos_3d_ref[2] = single_value_3d_ref[0];
    } else if (rm_order_v == 1){
      rm_1st_pos(dt_1l, single_value_2d_ref, single_value_3d_ref, single_value_nav_target, single_value_k3_rm, max_j_nav, 1); 
      pos_ref[2]    = pos_1l[2];
      pos_d_ref[2]  = pos_d[2];
      pos_2d_ref[2] = single_value_2d_ref[0];
      pos_3d_ref[2] = single_value_3d_ref[0];  
    }    
    // Generate Reference signals for attitude using RM
    // FIX ME ow not defined anymore without oval
    bool ow_psi = false;
    rm_3rd_attitude(dt_1l, att_ref, att_d_ref, att_2d_ref, att_3d_ref, att_des, ow_psi, psi_vec, k1_att_rm, k2_att_rm, k3_att_rm);
}
}

/**
 * @brief  Main function that runs the controller and performs control allocation
 * @param half_loop  In half-loop mode the controller is used for stabilization only
 * @param in_flight  The drone is in flight
 * FIXME: 
 */
void oneloop_andi_run(bool in_flight, bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v)
{
  // For NPS print time and loop delimeters
  printf("############### NEW LOOP ##########################\n");
  new_time = get_sys_time_float();
  dt_actual = new_time - old_time;
  old_time = new_time;
  printf("This function is running at a dt=%f \n",new_time-old_time);
  printf("Half loop = %d \n",half_loop);
  // At beginnig of the loop: (1) Register Attitude, (2) Initialize gains of RM and EC, (3) Calculate Normalization of Actuators Signals, (4) Propagate Actuator Model, (5) Update effectiveness matrix
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  init_controller();
  calc_normalization();
  get_act_state_oneloop();
  sum_g1g2_1l();
  
  // If drone is not on the ground use incremental law
  use_increment = 0.0;
  bool  volando       = false;
  if(in_flight) {
    use_increment = 1.0;
    volando       = true;
    printf("I am in flight\n");
  }
  if (ONELOOP_ANDI_DEBUG_MODE) {
    volando = false;
  }
  
  // Update desired Heading (psi_des_rad) based on previous loop or changed setting
  psi_des_rad = psi_des_deg * M_PI / 180.0;

  // Register the state of the drone in the variables used in RM and EC
  // (1) Attitude related
  att_1l[0] = eulers_zxy.phi                        * use_increment;
  att_1l[1] = eulers_zxy.theta                      * use_increment;
  att_1l[2] = eulers_zxy.psi                        * use_increment;
  oneloop_andi_propagate_filters();   //needs to be after update of attitude vector
  att_d[0]  = rates_filt_fo[0].last_out             * use_increment;
  att_d[1]  = rates_filt_fo[1].last_out             * use_increment;
  att_d[2]  = rates_filt_fo[2].last_out             * use_increment;
  att_2d[0] = ang_acc[0]                            * use_increment;
  att_2d[1] = ang_acc[1]                            * use_increment;
  att_2d[2] = ang_acc[2]                            * use_increment;
  // (2) Position related
  pos_1l[0] = stateGetPositionNed_f()->x;   
  pos_1l[1] = stateGetPositionNed_f()->y;   
  pos_1l[2] = stateGetPositionNed_f()->z;   
  pos_d[0]  = stateGetSpeedNed_f()->x;      
  pos_d[1]  = stateGetSpeedNed_f()->y;      
  pos_d[2]  = stateGetSpeedNed_f()->z;      
  pos_2d[0] = lin_acc[0];
  pos_2d[1] = lin_acc[1];
  pos_2d[2] = lin_acc[2];
  
  // Update the effectiveness matrix used in WLS
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = g1g2_1l[i];
  }

  // Run the Reference Model (RM)
  oneloop_andi_RM(half_loop, PSA_des, rm_order_h, rm_order_v);
  // Generate pseudo control for stabilization vector (nu) based on error controller
  if(half_loop){
    nu[0] = 0.0;
    nu[1] = 0.0;
    nu[2] = a_thrust;
    nu[3] = ec_3rd(att_ref[0], att_d_ref[0], att_2d_ref[0], att_3d_ref[0], att_1l[0], att_d[0], att_2d[0], k_phi_e, k_p_e, k_pdot_e);
    nu[4] = ec_3rd(att_ref[1], att_d_ref[1], att_2d_ref[1], att_3d_ref[1], att_1l[1], att_d[1], att_2d[1], k_theta_e, k_q_e, k_qdot_e);
    nu[5] = ec_2rd(att_d_ref[2], att_2d_ref[2], att_3d_ref[2], att_d[2], att_2d[2], k_r_e, k_r_d_e);
  }else{
    float k1_att_e[3] = {k_phi_e,  k_theta_e, k_psi_e};
    float k2_att_e[3] = {k_p_e,    k_q_e,     k_psi_d_e};
    float k3_att_e[3] = {k_pdot_e, k_qdot_e,  k_psi_2d_e};
    float y_4d_att[3];
    ec_3rd_att(y_4d_att, att_ref, att_d_ref, att_2d_ref, att_3d_ref, att_1l, att_d, att_2d, k1_att_e, k2_att_e, k3_att_e);
    nu[0] = ec_3rd(pos_ref[0], pos_d_ref[0], pos_2d_ref[0], pos_3d_ref[0], pos_1l[0], pos_d[0], pos_2d[0], k_N_e, k_vN_e, k_aN_e);
    nu[1] = ec_3rd(pos_ref[1], pos_d_ref[1], pos_2d_ref[1], pos_3d_ref[1], pos_1l[1], pos_d[1], pos_2d[1], k_E_e, k_vE_e, k_aE_e);
    nu[2] = ec_3rd(pos_ref[2], pos_d_ref[2], pos_2d_ref[2], pos_3d_ref[2], pos_1l[2], pos_d[2], pos_2d[2], k_D_e, k_vD_e, k_aD_e); 
    nu[3] = y_4d_att[0];  
    nu[4] = y_4d_att[1]; 
    nu[5] = y_4d_att[2]; 
  }

  // Calculate the min and max increments
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    if(i<ANDI_NUM_ACT){
      du_min_1l[i]  = (act_min[i] - use_increment * actuator_state_1l[i])/ratio_u_un[i];//
      du_max_1l[i]  = (act_max[i] - use_increment * actuator_state_1l[i])/ratio_u_un[i];//
      du_pref_1l[i] = (u_pref[i]  - use_increment * actuator_state_1l[i])/ratio_u_un[i];//
    }else{
      du_min_1l[i]  = (act_min[i] - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];//
      du_max_1l[i]  = (act_max[i] - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];//
      // if(i== ANDI_NUM_ACT_TOT-1){
      //   u_pref[i] = radio_control_get(RADIO_AUX4) * (13.0) / MAX_PPRZ - 3.0;
      // }
      du_pref_1l[i] = (u_pref[i]  - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];//
    }
    }
    // WLS Control Allocator
    number_iter = wls_alloc_oneloop(andi_du_n, nu, du_min_1l, du_max_1l, bwls_1l, 0, 0, Wv_wls, Wu, du_pref_1l, gamma_wls, 10);

  for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
    andi_du[i] = (float)(andi_du_n[i] * ratio_u_un[i]);
  }

  if (volando) {
    // Add the increments to the actuators
    float_vect_sum(andi_u, actuator_state_1l, andi_du, ANDI_NUM_ACT);
    andi_u[ANDI_NUM_ACT]   = andi_du[ANDI_NUM_ACT]   + att_1l[0];
    andi_u[ANDI_NUM_ACT+1] = andi_du[ANDI_NUM_ACT+1] + att_1l[1];
  } else {
    // Not in flight, so don't increment
    float_vect_copy(andi_u, andi_du, ANDI_NUM_ACT);
    andi_u[ANDI_NUM_ACT] = andi_du[ANDI_NUM_ACT];
    andi_u[ANDI_NUM_ACT+1] = andi_du[ANDI_NUM_ACT+1];
  }

  // TODO : USE THE PROVIDED MAX AND MIN and change limits for phi and theta
  // Bound the inputs to the actuators
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    Bound(andi_u[i], act_min[i], act_max[i]);
  }

  /*Commit the actuator command*/
  stabilization_cmd[COMMAND_THRUST] = 0;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    actuators_pprz[i] = (int16_t) andi_u[i];
    stabilization_cmd[COMMAND_THRUST] += actuator_state_1l[i];
  }
  stabilization_cmd[COMMAND_THRUST] = stabilization_cmd[COMMAND_THRUST]/4.0;
  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
    eulers_zxy_des.phi   =  andi_u[ANDI_NUM_ACT];
    eulers_zxy_des.theta =  andi_u[ANDI_NUM_ACT+1];
  } else {
    eulers_zxy_des.phi   =  andi_u[ANDI_NUM_ACT];
    eulers_zxy_des.theta =  andi_u[ANDI_NUM_ACT+1];
  }
  psi_des_deg = psi_des_rad * 180.0 / M_PI;  
  printf("###################################################\n");
}

/** @brief  Function to reconstruct actuator state using first order dynamics */
void get_act_state_oneloop(void)
{
  int8_t i;
  float prev_actuator_state_1l;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    prev_actuator_state_1l = actuator_state_1l[i];
    actuator_state_1l[i] = prev_actuator_state_1l + act_dynamics_d[i] * (andi_u[i] - prev_actuator_state_1l);
    if(!autopilot_get_motors_on()){
      actuator_state_1l[i] = 0.0;
    }
    Bound(actuator_state_1l[i],0,MAX_PPRZ);
  }
}

/**
 * @brief Function that sums g1 and g2 to obtain the g1_g2 matrix. It also undoes the scaling that was done to make the values readable
 * FIXME: make this function into a for loop to make it more adaptable to different configurations
 */
void sum_g1g2_1l(void) {
  //struct FloatEulers *euler = stateGetNedToBodyEulers_f();
  float sphi   = sinf(eulers_zxy.phi);
  float cphi   = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi   = sinf(eulers_zxy.psi);
  float cpsi   = cosf(eulers_zxy.psi);
  float T      = -9.81; //minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float P      = actuator_state_1l[ONELOOP_ANDI_PUSHER_IDX] * g1_1l[2][ONELOOP_ANDI_PUSHER_IDX] / ANDI_G_SCALING;
  float scaler;
  int i = 0;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    // Effectiveness matrix for real actuators
    if (i < ANDI_NUM_ACT){
      scaler = act_dynamics[i] * ratio_u_un[i] * ratio_vn_v[i] / ANDI_G_SCALING;
      g1g2_1l[0][i] = (cpsi * stheta + ctheta * sphi * spsi) * g1_1l[2][i] * scaler;
      g1g2_1l[1][i] = (spsi * stheta - cpsi * ctheta * sphi) * g1_1l[2][i] * scaler;
      g1g2_1l[2][i] = (cphi * ctheta                       ) * g1_1l[2][i] * scaler;
      g1g2_1l[3][i] = (g1_1l[3][i])                                        * scaler;
      g1g2_1l[4][i] = (g1_1l[4][i])                                        * scaler;
      g1g2_1l[5][i] = (g1_1l[5][i] + g2_1l[i])                             * scaler;
      if ((ONELOOP_ANDI_AC_HAS_PUSHER)&&(i==ONELOOP_ANDI_PUSHER_IDX)){
        g1g2_1l[0][i] = (cpsi * ctheta - sphi * spsi * stheta) * g1_1l[2][i] * scaler;
        g1g2_1l[1][i] = (ctheta * spsi + cpsi * sphi * stheta) * g1_1l[2][i] * scaler;
        g1g2_1l[2][i] = (- cphi * stheta                     ) * g1_1l[2][i] * scaler;
        g1g2_1l[3][i] = 0.0;
        g1g2_1l[4][i] = 0.0;
        g1g2_1l[5][i] = 0.0;
      }
    }else{
      scaler = act_dynamics[i] * ratio_u_un[i] * ratio_vn_v[i];
      // Effectiveness matrix for Phi
      if (i == ANDI_NUM_ACT){
        g1g2_1l[0][i] = ( cphi * ctheta * spsi * T - cphi * spsi * stheta * P) * scaler;
        g1g2_1l[1][i] = (-cphi * ctheta * cpsi * T + cphi * cpsi * stheta * P) * scaler;
        g1g2_1l[2][i] = (-sphi * ctheta * T + sphi * stheta * P) * scaler;
        g1g2_1l[3][i] = 0.0;
        g1g2_1l[4][i] = 0.0;
        g1g2_1l[5][i] = 0.0;
      }
      // Effectiveness matrix for Theta
      if (i == ANDI_NUM_ACT+1){
        g1g2_1l[0][i] = ((ctheta*cpsi - sphi*stheta*spsi) * T - (cpsi * stheta + ctheta * sphi * spsi) * P) * scaler;
        g1g2_1l[1][i] = ((ctheta*spsi + sphi*stheta*cpsi) * T - (spsi * stheta - cpsi * ctheta * sphi) * P) * scaler;
        g1g2_1l[2][i] = (-stheta * cphi * T - cphi * ctheta * P) * scaler;
        g1g2_1l[3][i] = 0.0;
        g1g2_1l[4][i] = 0.0;
        g1g2_1l[5][i] = 0.0;
      }
    }
  }
}

/** @brief  Calculate Normalization of actuators and discrete actuator dynamics  */
void calc_normalization(void){
  int8_t i;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
    act_dynamics_d[i] = 1.0-exp(-act_dynamics[i]*dt_1l);
    Bound(act_dynamics_d[i],0.0,1.0);
    ratio_vn_v[i] = 1.0;
    Bound(act_max[i],0,MAX_PPRZ);
    Bound(act_min[i],-MAX_PPRZ,0);
    ratio_u_un[i] = (act_max[i]-act_min[i])/(act_max_norm[i]-act_min_norm[i]);
  }
}

/** @brief  Function that calculates the model prediction for the complementary filter. */
void calc_model(void){
  int8_t i;
  int8_t j;
  // Absolute Model Prediction : 
  float sphi   = sinf(eulers_zxy.phi);
  float cphi   = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi   = sinf(eulers_zxy.psi);
  float cpsi   = cosf(eulers_zxy.psi);
  float T      = -9.81; 
  float P      = actuator_state_1l[ONELOOP_ANDI_PUSHER_IDX] * g1_1l[2][ONELOOP_ANDI_PUSHER_IDX] / ANDI_G_SCALING;
  
  model_pred[0] = (cpsi * stheta + ctheta * sphi * spsi) * T + (cpsi * ctheta + sphi * spsi * stheta) * P;
  model_pred[1] = (spsi * stheta - cpsi * ctheta * sphi) * T + (ctheta * spsi + cpsi * sphi * stheta) * P;
  model_pred[2] = g + cphi * ctheta * T - cphi * stheta * P;

  for (i = 3; i < ANDI_OUTPUTS; i++){ // For loop for prediction of angular acceleration
    model_pred[i] = 0.0;              // 
    for (j = 0; j < ANDI_NUM_ACT; j++){
      model_pred[i] = model_pred[i] +  actuator_state_1l[j] * g1g2_1l[i][j] / (act_dynamics[j] * ratio_u_un[j] * ratio_vn_v[j]);
    }
  }
}

/**
 * @brief Converts an angle in radians to an angle between -pi and pi.
 *
 * @param psi The input angle in radians.
 * @return The converted angle between -pi and pi.
 */
float convert_angle(float psi) {
    // Compute the equivalent angle between -2pi and 2pi
    psi = fmodf(psi, 2 * M_PI);

    if (psi < -M_PI){
        psi += 2 * M_PI;
    }else if (psi > M_PI){
        psi -= 2 * M_PI;
    }
    // Convert to angle between -pi and pi
    if (psi <= M_PI){
        return psi;
    }else{
        return psi - 2 * M_PI;
    }
}
