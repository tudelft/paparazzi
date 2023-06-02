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

#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"
//#include "firmwares/rotorcraft/stabilization/stabilization.h"
//#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

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

//#define ANDI_NUM_ACT 4 
//#define ANDI_OUTPUTS 6

#ifdef ONELOOP_ANDI_FILT_CUTOFF
float oneloop_andi_filt_cutoff = ONELOOP_ANDI_FILT_CUTOFF;
#else
float oneloop_andi_filt_cutoff = 2.0;
#endif

#ifdef ONELOOP_ANDI_ACT_IS_SERVO
bool actuator_is_servo[ANDI_NUM_ACT] = ONELOOP_ANDI_ACT_IS_SERVO;
#else
bool actuator_is_servo[ANDI_NUM_ACT] = {0};
#endif

#ifdef ONELOOP_ANDI_ACT_DYN
float act_dynamics[ANDI_NUM_ACT] = ONELOOP_ANDI_ACT_DYN;
//float act_dynamics_d[ANDI_NUM_ACT] = = {0};
#else
float act_dynamics[ANDI_NUM_ACT] = = {0};
f//loat act_dynamics_d[ANDI_NUM_ACT] = = {0};
#endif
//float act_dynamics_d[ANDI_NUM_ACT] = = {0};

#ifdef ONELOOP_ANDI_ACT_MAX
float act_max[ANDI_NUM_ACT] = ONELOOP_ANDI_ACT_MAX;
#else
float act_max[ANDI_NUM_ACT] = = {9600.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN
float act_min[ANDI_NUM_ACT] = ONELOOP_ANDI_ACT_MIN;
#else
float act_min[ANDI_NUM_ACT] = = {0.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MAX_NORM
float act_max_norm[ANDI_NUM_ACT] = ONELOOP_ANDI_ACT_MAX_NORM;
#else
float act_max_norm[ANDI_NUM_ACT] = = {1.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN_NORM
float act_min_norm[ANDI_NUM_ACT] = ONELOOP_ANDI_ACT_MIN_NORM;
#else
float act_min_norm[ANDI_NUM_ACT] = = {0.0};
#endif

/*  Define Section of the functions used in this module*/
void calc_normalization(void);
void sum_g1g2_1l(void);
void get_act_state_oneloop(void);
void oneloop_andi_propagate_filters(void);
void init_filter(void);
void init_controller(void);
void rm_2rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm);
void rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

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
                                        ANDI_NUM_ACT, g1g2_1l[0],
                                        ANDI_NUM_ACT, g1g2_1l[1],
                                        ANDI_NUM_ACT, g1g2_1l[2],
                                        ANDI_NUM_ACT, g1g2_1l[3],
                                        ANDI_NUM_ACT, g1g2_1l[4],
                                        ANDI_NUM_ACT, g1g2_1l[5],
                                        ANDI_OUTPUTS, nu,
                                        ANDI_NUM_ACT, act_state_filt_vect_1l); //andi_u
}
#endif
/*Physical Properties RW3C*/
float m   = 0.4;//6.5;  // [kg] Mass
float g   = 9.81; // [m/s^2] Gravitational Acceleration
float Ixx = 0.251; // [kg m^2] Inertia around x axis
float Iyy = 1.071; // [kg m^2] Inertia around y axis
float Izz = 1.334; // [kg m^2] Inertia around z axis
float num_thrusters = 4; // Number of motors used for thrust
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
float gamma_wls             = 0.01;//100;//100000.0;
static float Wv[ANDI_OUTPUTS]      = {0.0,0.0,0.0,10.0*100.0,10.0*100.0,100.0};//{0.0,0.0,5.0,10.0*100.0,10.0*100.0,0.0}; // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
static float Wu[ANDI_NUM_ACT]      = {2.0,2.0,2.0,2.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
float u_pref[ANDI_NUM_ACT]  = {0.0,0.0,0.0,0.0};
// float Wu[11]      = {2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,1.0,1.8,2.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
// float Wu_tran[11] = {0.0,0.0,0.0,0.0,3.0,3.0,3.0,3.0,0.0,0.0,0.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
// float u_pref[11] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float du_min[ANDI_NUM_ACT] = {0.0,0.0,0.0,0.0};
float du_max[ANDI_NUM_ACT];
float du_pref[ANDI_NUM_ACT];
int   number_iter = 0;
/*Declaration of Reference Model and Error Controller Gains*/
float rm_k_attitude;
float p1_att = 7.68;       
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
float p1_pos = 1.0;     
float p2_pos  ;     
float p3_pos  ;     
float route_k  ;    

float k_N_e    ;    
float k_vN_e   ;    
float k_aN_e   ;    
float k_E_e    ;    
float k_vE_e    ;   
float k_aE_e   ;     

float k_N_rm   ;    
float k_vN_rm ;     
float k_aN_rm  ;    
float k_E_rm   ;    
float k_vE_rm  ;    
float k_aE_rm  ;    

float k_N_route ;   
float k_vN_route  ; 
float k_aN_route ;  
float k_E_route  ;  
float k_vE_route ;  
float k_aE_route ;  

/*Altitude Loop*/
float rm_k_alt ;    
float p1_alt = 2.0;    
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
float p1_head = 4.8;     
float p2_head ;      

float k_r_e  ;      
float k_r_d_e ;     

    
float k_r_rm  ;     
float k_r_d_rm ;    


/*Define Variables used in control*/
float old_time = 0.0;
float new_time = 0.0;
struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
float act_dynamics_d[ANDI_NUM_ACT];
//struct FloatRates ang_accel_ref = {0., 0., 0.};
float actuator_state_1l[ANDI_NUM_ACT] = {0.,0.,0.,0.};
//struct FloatRates rate_sp_telem = {0., 0., 0.};
float ang_acc[3] = {0., 0., 0.};
float andi_u[ANDI_NUM_ACT];
float andi_du[ANDI_NUM_ACT];
float andi_du_n[ANDI_NUM_ACT];
float g2_times_du;
//float q_filt = 0.0;
//float r_filt = 0.0;
float dt_1l = 1./PERIODIC_FREQUENCY;
//float G1[6][ANDI_NUM_ACT];
bool  rc_on = true;
float act_state_filt_vect_1l[ANDI_NUM_ACT];
float du_estimation[ANDI_NUM_ACT];
float ddu_estimation[ANDI_NUM_ACT];
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

float g2_1l[ANDI_NUM_ACT] = ONELOOP_ANDI_G2; //scaled by INDI_G_SCALING
//float g1_1l[ANDI_OUTPUTS][ANDI_NUM_ACT] = {ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_THRUST, ONELOOP_ANDI_G1_ROLL, ONELOOP_ANDI_G1_PITCH, ONELOOP_ANDI_G1_YAW};  
float g1_1l[ANDI_OUTPUTS][ANDI_NUM_ACT] = {ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ROLL, ONELOOP_ANDI_G1_PITCH, ONELOOP_ANDI_G1_YAW};
float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT];
float thrust_eff[ANDI_NUM_ACT] = ONELOOP_ANDI_G1_THRUST;

float ratio_u_un[ANDI_NUM_ACT] = {1.0,1.0,1.0,1.0};
float ratio_vn_v[ANDI_NUM_ACT];

float *bwls_1l[ANDI_OUTPUTS];

bool verbose_oneloop = true;
/*Error Controller Gain Design*/
static float k_e_1_3_f(float p1, float p2, float p3) {return (p1*p2*p3);}
static float k_e_2_3_f(float p1, float p2, float p3) {return (p1*p2+p1*p3+p2*p3);}
static float k_e_3_3_f(float p1, float p2, float p3) {return (p1+p2+p3);}
static float k_e_1_2_f(float p1, float p2) {return (p1*p2);}
static float k_e_2_2_f(float p1, float p2) {return (p1+p2);}

/*Reference Model Gain Design*/
static float k_rm_1_3_f(float omega_n, float zeta, float p1) {return (omega_n*omega_n*p1)/(omega_n*omega_n+omega_n*p1*zeta*2.0);}
static float k_rm_2_3_f(float omega_n, float zeta, float p1) {return (omega_n*omega_n+omega_n*p1*zeta*2.0)/(p1+omega_n*zeta*2.0);}
static float k_rm_3_3_f(float omega_n, float zeta, float p1) {return p1+omega_n*zeta*2.0;}
static float k_rm_1_2_f(float omega_n, float zeta) {return omega_n/(2.0*zeta);}
static float k_rm_2_2_f(float omega_n, float zeta) {return 2.0*zeta*omega_n;}

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
  *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}


void rm_2rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm){
  float e_x      = k1_rm * (x_des- *x_ref);
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  *x_2d_ref = e_x_d;
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/*Third Order Dynamics Approximation*/
// static float w_approx(float p1, float p2, float p3, float rm_k){
//   float tao = (p1*p2+p1*p3+p2*p3)/(p1*p2*p3)/(rm_k);
//   return 1/tao;
// }

/** Gain Design
 * @brief This section is used to define the gains of the reference model and error controller
 * FIXME: Calculate the gains dynamically for transition
 */

void init_controller(void){
/*Attitude Loop*/
rm_k_attitude = 0.8;
// p1_att        = 0.8*0.8*w_L;
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
// p1_pos        = 1.0;
p2_pos        = p1_pos;
p3_pos        = p1_pos;
route_k       = 0.8;
k_N_e         = k_e_1_3_f(p1_pos,p2_pos,p3_pos);
k_vN_e        = k_e_2_3_f(p1_pos,p2_pos,p3_pos);
k_aN_e        = k_e_3_3_f(p1_pos,p2_pos,p3_pos);
k_E_e         = k_N_e;
k_vE_e        = k_vN_e;
k_aE_e        = k_aN_e;
k_N_rm        = k_rm_1_3_f(rm_k_pos*p1_pos,1.0,rm_k_pos*p3_pos);
k_vN_rm       = k_rm_2_3_f(rm_k_pos*p1_pos,1.0,rm_k_pos*p3_pos);
k_aN_rm       = k_rm_3_3_f(rm_k_pos*p1_pos,1.0,rm_k_pos*p3_pos);
k_E_rm        = k_N_rm;
k_vE_rm       = k_vN_rm;
k_aE_rm       = k_aN_rm;
k_N_route    = k_rm_1_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
k_vN_route   = k_rm_2_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
k_aN_route   = k_rm_3_3_f(route_k*rm_k_pos*p1_pos,1.0,route_k*rm_k_pos*p3_pos);
k_E_route    = k_N_route;
k_vE_route   = k_vN_route;
k_aE_route   = k_aN_route;

/*Altitude Loop*/
rm_k_alt      = 0.9;
// p1_alt        = 2.0;
p2_alt        = p1_alt;
p3_alt        = p1_alt;
k_D_e         = k_e_1_3_f(p1_alt,p2_alt,p3_alt);
k_vD_e        = k_e_2_3_f(p1_alt,p2_alt,p3_alt);
k_aD_e        = k_e_3_3_f(p1_alt,p2_alt,p3_alt);
k_D_rm        = k_rm_1_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);
k_vD_rm       = k_rm_2_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);
k_aD_rm       = k_rm_3_3_f(rm_k_alt*p1_alt,1.0,rm_k_alt*p3_alt);

/*Heading Loop*/
rm_k_head     = 0.9;
// p1_head       = 0.4 * w_L;
p2_head       = p1_head;
k_r_e         = k_e_1_2_f(p1_head,p2_head);
k_r_d_e       = k_e_2_2_f(p1_head,p2_head);
k_r_rm        = k_rm_1_2_f(rm_k_head*p1_head,rm_k_head*p2_head);
k_r_d_rm      = k_rm_2_2_f(rm_k_head*p1_head,rm_k_head*p2_head);

/*Approximated Dynamics*/
// UNCOMMENT ME
//act_dynamics[ANDI_NUM_ACT-2] = w_approx( p1_att,  p2_att,  p3_att,  rm_k_attitude);
//act_dynamics[ANDI_NUM_ACT-1]   = w_approx( p1_att,  p2_att,  p3_att,  rm_k_attitude);
}

/** state eulers in zxy order */
struct FloatEulers eulers_zxy;


/*Filters Initialization*/
float oneloop_andi_estimation_filt_cutoff = 2.0;

Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass filt_accel_body[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass yaw_filt;
Butterworth2LowPass act_lowpass_filt[ANDI_NUM_ACT];
Butterworth2LowPass att_dot_meas_lowpass_filters[3];
Butterworth2LowPass att_ref_lowpass_filters[3];
Butterworth2LowPass rate_ref_lowpass_filters[3];


Butterworth2LowPass measurement_lowpass_filters[3];
Butterworth2LowPass estimation_output_lowpass_filters[3];

//static struct FirstOrderLowPass rates_filt_fo[3];
struct FloatVect3 body_accel_f;


void init_filter(void)
{
  float tau = 1.0 / (2.0 * M_PI *oneloop_andi_filt_cutoff);
  float tau_est = 1.0 / (2.0 * M_PI * oneloop_andi_estimation_filt_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  // Filtering of the Inputs with 3 dimensions (e.g. rates and accelerations)
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_body[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&att_ref_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&rate_ref_lowpass_filters[i], tau, sample_time, 0.0);
  }

  // Filtering of the actuators
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&act_lowpass_filt[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&yaw_filt, tau, sample_time, 0.0);

  // Init rate filter for feedback
  //float time_constants[3] = {1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_P), 1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_Q), 1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_R)};
  //init_first_order_low_pass(&rates_filt_fo[0], time_constants[0], sample_time, stateGetBodyRates_f()->p);
  //init_first_order_low_pass(&rates_filt_fo[1], time_constants[1], sample_time, stateGetBodyRates_f()->q);
  //init_first_order_low_pass(&rates_filt_fo[2], time_constants[2], sample_time, stateGetBodyRates_f()->r);
}


/*Low pass the accelerometer measurements to remove noise from vibrations.The roll and pitch also need to be filtered to synchronize them with the acceleration. Called as a periodic function with PERIODIC_FREQ*/
void oneloop_andi_propagate_filters(void) {
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
    ang_acc[i] = (att_dot_meas_lowpass_filters[i].o[0]- att_dot_meas_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;
}}

/*Init function of oneloop controller*/
void oneloop_andi_init(void)
{ 
  //printf("I am initiating ONELOOP ANDI\n");
  //Calculate g1_g2. 
  calc_normalization();
  sum_g1g2_1l();

  // Initialize filters
  int8_t i;
  // Initialize the array of pointers to the rows of g1_g2
  for (i = 0; i < ANDI_OUTPUTS; i++) {
   bwls_1l[i] = g1g2_1l[i];
  }
  init_filter();
  init_controller();
  float_vect_zero(andi_u, ANDI_NUM_ACT);
  float_vect_zero(andi_du, ANDI_NUM_ACT);
  float_vect_zero(andi_du_n, ANDI_NUM_ACT);
  float_vect_zero(act_state_filt_vect_1l, ANDI_NUM_ACT);
  float_vect_zero(actuator_state_1l,ANDI_NUM_ACT);
  float_vect_zero(att_ref,3);
  float_vect_zero(att_d_ref,3);
  float_vect_zero(att_2d_ref,3);
  float_vect_zero(att_3d_ref,3);
  float_vect_zero(pos_ref,3);
  float_vect_zero(pos_d_ref,3);
  float_vect_zero(pos_2d_ref,3);
  float_vect_zero(pos_3d_ref,3);  
  float_vect_zero(nu, ANDI_OUTPUTS);

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ONELOOP_ANDI, send_oneloop_andi);
  #endif
}

/**
 * Function that resets important values upon engaging Oneloop ANDI.
 *
 * FIXME: Ideally we should distinguish between the "stabilization" and "guidance" needs because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 */
void oneloop_andi_enter(void)
{
  calc_normalization();
  sum_g1g2_1l();
  init_filter();
  init_controller();
  /* Stabilization Reset */
  // To-Do // stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
  float_vect_zero(andi_u, ANDI_NUM_ACT);
  float_vect_zero(andi_du, ANDI_NUM_ACT);
  float_vect_zero(andi_du_n, ANDI_NUM_ACT);
  float_vect_zero(act_state_filt_vect_1l, ANDI_NUM_ACT);
  float_vect_zero(actuator_state_1l,ANDI_NUM_ACT);
  float_vect_zero(du_estimation, ANDI_NUM_ACT);
  float_vect_zero(ddu_estimation, ANDI_NUM_ACT);
  float_vect_zero(att_ref,3);
  float_vect_zero(att_d_ref,3);
  float_vect_zero(att_2d_ref,3);
  float_vect_zero(att_3d_ref,3);
  float_vect_zero(pos_ref,3);
  float_vect_zero(pos_d_ref,3);
  float_vect_zero(pos_2d_ref,3);
  float_vect_zero(pos_3d_ref,3);  
  float_vect_zero(nu, ANDI_OUTPUTS);
  /*Guidance Reset*/
  // To-Do- // guidance_andi_hybrid_heading_sp = stateGetNedToBodyEulers_f()->psi;

  //float tau = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff);
  //float sample_time = 1.0 / PERIODIC_FREQUENCY;
  //for (int8_t i = 0; i < 3; i++) {
    //init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
    //init_butterworth_2_low_pass(&filt_accel_body[i], tau, sample_time, 0.0);
  //}
  //init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  //init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  //init_butterworth_2_low_pass(&yaw_filt, tau, sample_time, stateGetNedToBodyEulers_f()->psi);
}


/**
 * @brief Function that calculates the actuator command based on the desired control vector
 * 
 * @param att_des 
 * @param in_flight 
 * FIXME: 
 */
void oneloop_andi_attitude_run(struct Int32Quat quat_sp, struct FloatVect3 pos_des, bool in_flight)
{
  printf("############### NEW LOOP ##########################\n");
  new_time = get_sys_time_float();
  printf("This function is running at a dt=%f \n",new_time-old_time);
  old_time = new_time;
  printf("k_theta_e = %f\n",k_theta_e);
  printf("k_q_e = %f\n",k_q_e);
  printf("k_qdot_e = %f\n",k_qdot_e);
  printf("k_theta_rm = %f\n",k_theta_rm);
  printf("k_q_rm = %f\n",k_q_rm);
  printf("k_qdot_rm = %f\n",k_qdot_rm);
  init_controller();
  calc_normalization();
  sum_g1g2_1l();
  //printf("I am running ONELOOP ANDI\n");
  // If drone is not on the ground use incremental law
  float use_increment = 0.0;
  bool volando = false;
  if(in_flight) {
    use_increment = 1.0;
    volando = true;
    printf("I am in flight\n");
    }

  // Convert Quaternion setpoint to Euler setpoint
  struct FloatEulers eulers_zxy_des;
  struct FloatQuat quat_sp_f;
  QUAT_FLOAT_OF_BFP(quat_sp_f, quat_sp);
  float_eulers_of_quat_zxy(&eulers_zxy_des, &quat_sp_f);
  // Sample and filter inputs

  // Override
  eulers_zxy_des.phi   = (float) (radio_control.values[RADIO_ROLL] )/9600.0*45.0*3.14/180.0;//0.0;
  eulers_zxy_des.theta = (float) (radio_control.values[RADIO_PITCH])/9600.0*45.0*3.14/180.0;//0.0;
  eulers_zxy_des.psi   = 0.0;//
  eulers_zxy.phi   = stateGetNedToBodyEulers_f()->phi;
  eulers_zxy.theta = stateGetNedToBodyEulers_f()->theta;
  eulers_zxy.psi   = stateGetNedToBodyEulers_f()->psi;
  oneloop_andi_propagate_filters();
  att_1l[0] = roll_filt.o[0]                        * use_increment;
  att_1l[1] = pitch_filt.o[0]                       * use_increment;
  att_1l[2] = yaw_filt.o[0]                         * use_increment;
  att_d[0]  = att_dot_meas_lowpass_filters[0].o[0]  * use_increment;
  att_d[1]  = att_dot_meas_lowpass_filters[1].o[0]  * use_increment;
  att_d[2]  = att_dot_meas_lowpass_filters[2].o[0]  * use_increment;
  att_2d[0] = ang_acc[0]                            * use_increment;
  att_2d[1] = ang_acc[1]                            * use_increment;
  att_2d[2] = ang_acc[2]                            * use_increment;

  
  int8_t i;
    // Update bwls_1l
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = g1g2_1l[i];
  }

  float thrust_cmd_1l = (float) radio_control.values[RADIO_THROTTLE];
  Bound(thrust_cmd_1l,0,MAX_PPRZ);

  printf("thrust_cmd_1l: %f\n",thrust_cmd_1l);
  stabilization_cmd[COMMAND_THRUST] = thrust_cmd_1l;
  //printf("Use increment: %f\n",use_increment);
  float a_thrust = 0.0;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    //printf("bwls_thrust column %i is %f\n",i,bwls_1l[2][i]);
    //printf("filtered state of actuator %i is %f\n",i,act_state_filt_vect_1l[i]);
    a_thrust +=(thrust_cmd_1l - use_increment*act_state_filt_vect_1l[i]) * bwls_1l[2][i]; //stabilization_cmd[COMMAND_THRUST] radio_control.values[RADIO_THROTTLE] 2000.0
    //a_thrust +=(thrust_cmd_1l - use_increment*actuator_state_1l[i]) * bwls_1l[2][i];
    //a_thrust += (thrust_cmd_1l - use_increment*andi_u[i]) * bwls_1l[2][i];
    }
  printf("a_thrust: %f\n",a_thrust);
  att_ref[2] = eulers_zxy_des.psi;
  float des_r = (float) (radio_control.values[RADIO_YAW])/9600.0*2.0; //(eulers_zxy_des.psi-eulers_zxy.psi);
  BoundAbs(des_r,2.0);
  // Generate reference signals with reference model
  // for (i = 0; i < 3; i++) {
  //   printf("att_ref[%i]=%f ",i,att_ref[i]);
  //   update_butterworth_2_low_pass(&att_ref_lowpass_filters[i],  att_ref[i]);
  //   update_butterworth_2_low_pass(&rate_ref_lowpass_filters[i], att_d_ref[i]);
  //   att_ref[i]   = att_ref_lowpass_filters[i].o[0];
  //   printf("att_ref_filt[%i]=%f ",i,att_ref[i]);
  //   printf("\n");
  //   att_d_ref[i] = rate_ref_lowpass_filters[i].o[0];
  // }

  rm_3rd(dt_1l, &att_ref[0],   &att_d_ref[0],  &att_2d_ref[0], &att_3d_ref[0],     eulers_zxy_des.phi, k_phi_rm, k_p_rm, k_pdot_rm);
  rm_3rd(dt_1l, &att_ref[1],   &att_d_ref[1],  &att_2d_ref[1], &att_3d_ref[1],     eulers_zxy_des.theta, k_theta_rm, k_q_rm, k_qdot_rm);
  rm_2rd(dt_1l, &att_d_ref[2], &att_2d_ref[2], &att_3d_ref[2], des_r, k_r_rm, k_r_d_rm);
  for (i = 0; i < 3; i++) {
    printf("att_ref_new[%i]=%f",i,att_ref[i]);
  }
  printf("\n");
  // Generate pseudo control vector (nu) based on error controller
  nu[0] = 0.0;
  nu[1] = 0.0;
  //nu[2] = a_thrust;
  nu[2] = 0.0;

  nu[3] = ec_3rd(att_ref[0], att_d_ref[0], att_2d_ref[0], att_3d_ref[0], att_1l[0], att_d[0], att_2d[0], k_phi_e, k_p_e, k_pdot_e);
  nu[4] = ec_3rd(att_ref[1], att_d_ref[1], att_2d_ref[1], att_3d_ref[1], att_1l[1], att_d[1], att_2d[1], k_theta_e, k_q_e, k_qdot_e);
  nu[5] = ec_2rd(att_d_ref[2], att_2d_ref[2], att_3d_ref[2], att_d[2], att_2d[2], k_r_e, k_r_d_e);

 
  // Calculate the min and max increments
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    du_min[i]  = (0.0       - use_increment * act_state_filt_vect_1l[i])/ratio_u_un[i];//-MAX_PPRZ * actuator_is_servo[i] - use_increment*act_state_filt_vect_1l[i];
    du_max[i]  = (MAX_PPRZ  - use_increment * act_state_filt_vect_1l[i])/ratio_u_un[i];//MAX_PPRZ - use_increment*act_state_filt_vect_1l[i];
    //du_pref[i] = (u_pref[i] - use_increment * act_state_filt_vect_1l[i])/ratio_u_un[i];//(u_pref[i] - use_increment*act_state_filt_vect_1l[i])/ratio_u_un[i];
    du_pref[i] = (thrust_cmd_1l - use_increment * act_state_filt_vect_1l[i])/ratio_u_un[i];
    }

    // if(autopilot.arming_status==3){
    //   printf("I AM ARMED\n");
    // }

  
    // WLS Control Allocator
    number_iter = wls_alloc_oneloop(andi_du_n, nu, du_min, du_max, bwls_1l, 0, 0, Wv, Wu, du_pref, gamma_wls, 10);
    //int8_t i;
    printf("@@@@@@@@@ END WLS @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
  // int8_t j;
  // for (i = 0; i < ANDI_OUTPUTS; i++) {
  //   for (j = 0; j < ANDI_NUM_ACT; j++) {
  //     printf("B row %i, column %i is % f\n",i,j,g1g2_1l[i][j]);
  //   }
  // }

  for (i = 0; i < ANDI_NUM_ACT; i++){
    printf("andi_du_pre_denormalization[%i]: %f ",i,andi_du_n[i]);
    andi_du[i] = (float)(andi_du_n[i] * ratio_u_un[i]);
  }
  printf("\n");
  if (volando) {
    // Add the increments to the actuators
    //printf("I am integrating\n");
    float_vect_sum(andi_u, act_state_filt_vect_1l, andi_du, ANDI_NUM_ACT);
    //float_vect_copy(andi_u, andi_du, ANDI_NUM_ACT);
  } else {
    // Not in flight, so don't increment
    //printf("I am NOT integrating\n");
    float_vect_copy(andi_u, andi_du, ANDI_NUM_ACT);
  }

  // TODO : USE THE PROVIDED MAX AND MIN
  // Bound the inputs to the actuators
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    if (actuator_is_servo[i]) {
        BoundAbs(andi_u[i], MAX_PPRZ);
      } else {
        Bound(andi_u[i], 0, MAX_PPRZ);
        }
  }
  // Propagate actuator filters
  get_act_state_oneloop();
  for (i = 0; i < ANDI_NUM_ACT; i++) {

    update_butterworth_2_low_pass(&act_lowpass_filt[i], actuator_state_1l[i]);
    printf("actuator_state_pre_filter[%i]: %f ",i,actuator_state_1l[i]);
    printf("actuator_state_post_filter[%i]: %f ",i,act_lowpass_filt[i].o[0]);
    printf("\n");

    if (actuator_is_servo[i]) {
        BoundAbs(act_lowpass_filt[i].o[0], MAX_PPRZ);
      } else {
        Bound(act_lowpass_filt[i].o[0], 0.0, MAX_PPRZ);
      }
    act_state_filt_vect_1l[i] = act_lowpass_filt[i].o[0];
    if (actuator_is_servo[i]) {
        BoundAbs(act_state_filt_vect_1l[i], MAX_PPRZ);
      } else {
        Bound(act_state_filt_vect_1l[i], 0.0, MAX_PPRZ);
      }
  }

  // if (verbose_oneloop){
  //   verbose_oneloop = true;
  //   printf("I am running oneloop ANDI\n");
  //   printf("dt_1l: %f\n",dt_1l);
  //   printf("Mean of ratios: %f\n",(ratio_u_un[0]+ratio_u_un[1]+ratio_u_un[2]+ratio_u_un[3])/4.0);
  // }
  /*Commit the actuator command*/

  for (i = 0; i < ANDI_NUM_ACT; i++) {
    actuators_pprz[i] = (int16_t) andi_u[i];
    //printf("Actuator %i set to %i\n",i,actuators_pprz[i]);
  }

  // Set the stab_cmd to 42 to indicate that it is not used
  //stabilization_cmd[COMMAND_ROLL] = 42;
  //tabilization_cmd[COMMAND_PITCH] = 42;
  //stabilization_cmd[COMMAND_YAW] = 42;
}

void get_act_state_oneloop(void)
{
  //actuator dynamics
  int8_t i;
  float prev_actuator_state_1l;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    prev_actuator_state_1l = actuator_state_1l[i];
    actuator_state_1l[i] = prev_actuator_state_1l + act_dynamics_d[i] * (andi_u[i] - prev_actuator_state_1l);
    if(!autopilot_get_motors_on()){
      actuator_state_1l[i] = 0.0;
    }
    //actuator_state_1l[i] = prev_actuator_state_1l + act_dynamics_d[i] * (actuators_pprz[i] - prev_actuator_state_1l);

    printf("andi_du_pre_dynamics[%i]: %f ",i,andi_du[i]);
    printf("andi_u_pre_dynamics[%i]: %f ",i,andi_u[i]);
    printf("actuator_state_pre_dynamics[%i]: %f ",i,prev_actuator_state_1l);
    printf("actuator_state_post_dynamics[%i]: %f ",i,actuator_state_1l[i]);
    printf("\n");

    Bound(actuator_state_1l[i],0,MAX_PPRZ);
    // TO DO ACTUATORS BOUND ACTUATOR_STATE_1L TO NOT BE ZERO!!!!!
    //printf("Actuator %i is currently estimated to be at %f\n",i,actuator_state_1l[i]);
  }
}

/**
 * Function that sums g1 and g2 to obtain the g1_g2 matrix
 * It also undoes the scaling that was done to make the values readable
 */

void sum_g1g2_1l(void) {
  int8_t i;
  int8_t j;
  //printf("I am calculating what g1g2\n");
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    for (j = 0; j < ANDI_NUM_ACT; j++) {
      if (i != 5) {
        g1g2_1l[i][j] = (g1_1l[i][j] / ANDI_G_SCALING) * act_dynamics[j] * ratio_u_un[j] * ratio_vn_v[j];
      } else {
        g1g2_1l[i][j] = ((g1_1l[i][j] + g2_1l[j]) / ANDI_G_SCALING) * act_dynamics[j] *ratio_u_un[j] * ratio_vn_v[j];
      }
    }
  }
}

void calc_normalization(void){
  //printf("I am calculating the normalization factors\n");
  int8_t i;
  for (i = 0; i < ANDI_NUM_ACT; i++){
    act_dynamics_d[i] = 1.0-exp(-act_dynamics[i]*dt_1l);
    Bound(act_dynamics_d[i],0.0,1.0);
    //printf("discrete dynamics actuator %i is %f\n",i,act_dynamics_d[i]);
    ratio_vn_v[i] = 1.0;
    //ratio_u_un[i] = 1.0;
    ratio_u_un[i] = (act_max[i]-act_min[i])/(act_max_norm[i]-act_min_norm[i]);
  }
}

