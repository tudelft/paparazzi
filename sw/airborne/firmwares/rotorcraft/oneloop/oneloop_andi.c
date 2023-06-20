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

//#define ANDI_NUM_ACT_TOT 4 
//#define ANDI_OUTPUTS 6

#ifdef ONELOOP_ANDI_FILT_CUTOFF
float oneloop_andi_filt_cutoff = ONELOOP_ANDI_FILT_CUTOFF;
#else
float oneloop_andi_filt_cutoff = 2.0;
#endif

#ifdef ONELOOP_ANDI_ACT_IS_SERVO
bool actuator_is_servo[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_IS_SERVO;
#else
bool actuator_is_servo[ANDI_NUM_ACT_TOT] = {0};
#endif

#ifdef ONELOOP_ANDI_ACT_DYN
float act_dynamics[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_DYN;
#else
float act_dynamics[ANDI_NUM_ACT_TOT] = = {0};
#endif


#ifdef ONELOOP_ANDI_ACT_MAX
float act_max[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX;
#else
float act_max[ANDI_NUM_ACT_TOT] = = {9600.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN
float act_min[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN;
#else
float act_min[ANDI_NUM_ACT_TOT] = = {0.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MAX_NORM
float act_max_norm[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX_NORM;
#else
float act_max_norm[ANDI_NUM_ACT_TOT] = = {1.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN_NORM
float act_min_norm[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN_NORM;
#else
float act_min_norm[ANDI_NUM_ACT_TOT] = = {0.0};
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF_P
#define ONELOOP_ANDI_FILTER_ROLL_RATE TRUE
#else
#define ONELOOP_ANDI_FILT_CUTOFF_P 20.0
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF_Q
#define ONELOOP_ANDI_FILTER_PITCH_RATE TRUE
#else
#define ONELOOP_ANDI_FILT_CUTOFF_Q 20.0
#endif

#ifdef ONELOOP_ANDI_FILT_CUTOFF_R
#define ONELOOP_ANDI_FILTER_YAW_RATE TRUE
#else
#define ONELOOP_ANDI_FILT_CUTOFF_R 20.0
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
void calc_model(void);
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
                                        ANDI_NUM_ACT, actuator_state_1l); //andi_u
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
                                        &pos_2d[2]); //andi_u
}
#endif
/*Physical Properties RW3C*/
float m   = 0.4;//6.5;  // [kg] Mass
float g   = 9.81; // [m/s^2] Gravitational Acceleration
float Ixx = 0.251; // [kg m^2] Inertia around x axis
float Iyy = 1.071; // [kg m^2] Inertia around y axis
float Izz = 1.334; // [kg m^2] Inertia around z axis
float num_thrusters = 4.0; // Number of motors used for thrust
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
float gamma_wls             = 1000.0;//0.01;//100;//100000.0;
static float Wv[ANDI_OUTPUTS]          = {1.0,1.0,1.0,10.0*100.0,10.0*100.0,100.0};//{0.0,0.0,5.0,10.0*100.0,10.0*100.0,0.0}; // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
static float Wu[ANDI_NUM_ACT_TOT]      = {2.0, 2.0, 2.0,2.0,2.0,2.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
float u_pref[ANDI_NUM_ACT_TOT]         = {0.0,0.0,0.0,0.0,0.0,0.0};
// float Wu[11]      = {2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,1.0,1.8,2.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
// float Wu_tran[11] = {0.0,0.0,0.0,0.0,3.0,3.0,3.0,3.0,0.0,0.0,0.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
// float u_pref[11] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float du_min[ANDI_NUM_ACT_TOT]; 
float du_max[ANDI_NUM_ACT_TOT];
float du_pref[ANDI_NUM_ACT_TOT];
float model_pred[ANDI_OUTPUTS];
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
float p1_alt = 3.0;    
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
float act_dynamics_d[ANDI_NUM_ACT_TOT];
float actuator_state_1l[ANDI_NUM_ACT];

float ang_acc[3];
float lin_acc[3];
float andi_u[ANDI_NUM_ACT_TOT];
float andi_du[ANDI_NUM_ACT_TOT];
float andi_du_n[ANDI_NUM_ACT_TOT];
float g2_times_du;
float dt_1l = 1./PERIODIC_FREQUENCY;

bool  rc_on = true;

struct FloatEulers eulers_zxy_des;
float pos_des[3];
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
//float g1_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT] = {ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ZERO, ONELOOP_ANDI_G1_ROLL, ONELOOP_ANDI_G1_PITCH, ONELOOP_ANDI_G1_YAW};
float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
float thrust_eff[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_G1_THRUST;

float ratio_u_un[ANDI_NUM_ACT_TOT];
float ratio_vn_v[ANDI_NUM_ACT_TOT];

float *bwls_1l[ANDI_OUTPUTS];

bool check_1st_nav = true; 
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
static float w_approx(float p1, float p2, float p3, float rm_k){
  float tao = (p1*p2+p1*p3+p2*p3)/(p1*p2*p3)/(rm_k);
  return 1/tao;
}

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
// k_theta_e     = 2.7706*6.3155*9.1441;
// k_q_e         = 6.3155*9.1441;
// k_qdot_e      = 9.1441;
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
p2_head       = p1_head;
k_r_e         = k_e_1_2_f(p1_head,p2_head);
k_r_d_e       = k_e_2_2_f(p1_head,p2_head);
k_r_rm        = k_rm_1_2_f(rm_k_head*p1_head,rm_k_head*p2_head);
k_r_d_rm      = k_rm_2_2_f(rm_k_head*p1_head,rm_k_head*p2_head);

/*Approximated Dynamics*/

act_dynamics[ANDI_NUM_ACT_TOT-2]   = w_approx( p1_att,  p2_att,  p3_att,  rm_k_attitude);
act_dynamics[ANDI_NUM_ACT_TOT-1]   = w_approx( p1_att,  p2_att,  p3_att,  rm_k_attitude);
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
Butterworth2LowPass att_dot_meas_lowpass_filters[3];
Butterworth2LowPass att_ref_lowpass_filters[3];
Butterworth2LowPass rate_ref_lowpass_filters[3];


Butterworth2LowPass measurement_lowpass_filters[3];
Butterworth2LowPass estimation_output_lowpass_filters[3];

static struct FirstOrderLowPass rates_filt_fo[3];
Butterworth2LowPass model_pred_filt[ANDI_OUTPUTS];

struct FloatVect3 body_accel_f;


void init_filter(void)
{
  float tau = 1.0 / (2.0 * M_PI *oneloop_andi_filt_cutoff);
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
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&yaw_filt, tau, sample_time, 0.0);

  // Init rate filter for feedback
  float time_constants[3] = {1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_P), 1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_Q), 1.0 / (2 * M_PI * ONELOOP_ANDI_FILT_CUTOFF_R)};
  init_first_order_low_pass(&rates_filt_fo[0], time_constants[0], sample_time, stateGetBodyRates_f()->p);
  init_first_order_low_pass(&rates_filt_fo[1], time_constants[1], sample_time, stateGetBodyRates_f()->q);
  init_first_order_low_pass(&rates_filt_fo[2], time_constants[2], sample_time, stateGetBodyRates_f()->r);
  
  // Remember to change the time constant if you provide different P Q R filters
  for (i = 0; i < ANDI_OUTPUTS; i++){
    init_butterworth_2_low_pass(&model_pred_filt[i], tau, sample_time, 0.0);
  }
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
  
  calc_model();
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++){
    update_butterworth_2_low_pass(&model_pred_filt[i], model_pred[i]);
  }

  for (i = 0; i < 3; i++) {
    update_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], rate_vect[i]);
    float old_rate = rates_filt_fo[i].last_out;
    update_first_order_low_pass(&rates_filt_fo[i], rate_vect[i]);
    ang_acc[i] = (att_dot_meas_lowpass_filters[i].o[0]- att_dot_meas_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY + model_pred[3+i] - model_pred_filt[3+i].o[0];
    lin_acc[i] = filt_accel_ned[i].o[0] + model_pred[i] - model_pred_filt[i].o[0];  
    //ang_acc[i] = (rates_filt_fo[i].last_out- old_rate) * PERIODIC_FREQUENCY;
    
}}

/*Init function of oneloop controller*/
void oneloop_andi_init(void)
{ 
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
  float_vect_zero(andi_u, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du_n, ANDI_NUM_ACT_TOT);
  float_vect_zero(actuator_state_1l,ANDI_NUM_ACT);
  float_vect_zero(att_ref,3);
  float_vect_zero(att_d_ref,3);
  float_vect_zero(att_2d_ref,3);
  float_vect_zero(att_3d_ref,3);
  // float_vect_zero(pos_ref,3);
  // float_vect_zero(pos_d_ref,3);
  // float_vect_zero(pos_2d_ref,3);
  // float_vect_zero(pos_3d_ref,3);  
  float_vect_zero(nu, ANDI_OUTPUTS);
  float_vect_zero(ang_acc,3);
  float_vect_zero(lin_acc,3);
  float_vect_zero(pos_des,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ONELOOP_ANDI, send_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ONELOOP_G, send_oneloop_g);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ONELOOP_GUIDANCE, send_oneloop_guidance);
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
  //float_vect_zero(andi_u, ANDI_NUM_ACT_TOT);
  //float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);
  //float_vect_zero(andi_du_n, ANDI_NUM_ACT_TOT);
  //float_vect_zero(actuator_state_1l,ANDI_NUM_ACT);
  float_vect_zero(att_ref,3);
  float_vect_zero(att_d_ref,3);
  float_vect_zero(att_2d_ref,3);
  float_vect_zero(att_3d_ref,3);
  // float_vect_zero(pos_ref,3);
  // float_vect_zero(pos_d_ref,3);
  // float_vect_zero(pos_2d_ref,3);
  // float_vect_zero(pos_3d_ref,3);  
  //float_vect_zero(nu, ANDI_OUTPUTS);
  float_vect_zero(ang_acc,3);
  float_vect_zero(lin_acc,3);
  float_vect_zero(pos_des,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;
  /*Guidance Reset*/

}


/**
 * @brief Function that calculates the actuator command based on the desired control vector
 * 
 * @param att_des 
 * @param in_flight 
 * FIXME: 
 */
void oneloop_andi_attitude_run(bool in_flight)
{
  printf("############### NEW LOOP ##########################\n");
  new_time = get_sys_time_float();
  printf("This function is running at a dt=%f \n",new_time-old_time);
  old_time = new_time;

  printf("k_D_e = %f\n",k_D_e);
  printf("k_vD_e = %f\n",k_vD_e);
  printf("k_aD_e = %f\n",k_aD_e);
  printf("k_D_rm = %f\n",k_D_rm);
  printf("k_vD_rm = %f\n",k_vD_rm);
  printf("k_aD_rm = %f\n",k_aD_rm);  
  //printf("k_theta_e = %f\n",k_theta_e);
  //printf("k_q_e = %f\n",k_q_e);
  //printf("k_qdot_e = %f\n",k_qdot_e);
  //printf("k_theta_rm = %f\n",k_theta_rm);
  //printf("k_q_rm = %f\n",k_q_rm);
  //printf("k_qdot_rm = %f\n",k_qdot_rm);
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
  //struct FloatEulers eulers_zxy_des;
  // struct FloatQuat quat_sp_f;
  // QUAT_FLOAT_OF_BFP(quat_sp_f, quat_sp);
  // float_eulers_of_quat_zxy(&eulers_zxy_des, &quat_sp_f);
  // Sample and filter inputs

  // if(autopilot.mode==AP_MODE_NAV){
  //   printf("I AM IN NAV\n");
  //   // eulers_zxy_des.phi   =  0.0;
  //   // eulers_zxy_des.theta =  0.0;
  //   // eulers_zxy_des.psi   =  0.0;
  // }
  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
    printf("I AM IN ATT\n");
    eulers_zxy_des.phi   = (float) (radio_control.values[RADIO_ROLL] )/9600.0*45.0*3.14/180.0;//0.0;
    eulers_zxy_des.theta = (float) (radio_control.values[RADIO_PITCH])/9600.0*45.0*3.14/180.0;//0.0;
    eulers_zxy_des.psi   = 0.0;//
    check_1st_nav = true;
  }
  eulers_zxy.phi   = stateGetNedToBodyEulers_f()->phi;
  eulers_zxy.theta = stateGetNedToBodyEulers_f()->theta;
  eulers_zxy.psi   = stateGetNedToBodyEulers_f()->psi;
  
  att_1l[0] = eulers_zxy.phi                        * use_increment;
  att_1l[1] = eulers_zxy.theta                      * use_increment;
  att_1l[2] = eulers_zxy.psi                        * use_increment;
  oneloop_andi_propagate_filters(); //needs to be after update of attitude vector
  att_d[0]  = rates_filt_fo[0].last_out             * use_increment;
  att_d[1]  = rates_filt_fo[1].last_out             * use_increment;
  att_d[2]  = rates_filt_fo[2].last_out             * use_increment;
  att_2d[0] = ang_acc[0]                            * use_increment;
  att_2d[1] = ang_acc[1]                            * use_increment;
  att_2d[2] = ang_acc[2]                            * use_increment;

  pos_1l[0] = stateGetPositionNed_f()->x;
  pos_1l[1] = stateGetPositionNed_f()->y;
  pos_1l[2] = stateGetPositionNed_f()->z;
  pos_d[0]  = stateGetSpeedNed_f()->x;
  pos_d[1]  = stateGetSpeedNed_f()->y;
  pos_d[2]  = stateGetSpeedNed_f()->z;
  pos_2d[0] = lin_acc[0];
  pos_2d[1] = lin_acc[1];
  pos_2d[2] = lin_acc[2];
  
  int8_t i;
    // Update bwls_1l
  for (i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = g1g2_1l[i];
  }

  // Generate reference signals with reference model
  float a_thrust = 0.0;
  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
    Wv[0] = 0.0;
    Wv[1] = 0.0;
    float_vect_copy(pos_des,pos_1l,3);
    float_vect_copy(pos_ref,pos_1l,3);
    float_vect_copy(pos_d_ref,pos_d,3);
    float_vect_copy(pos_2d_ref,pos_2d,3);
    float_vect_zero(pos_3d_ref,3);
    float_vect_copy(pos_init,pos_1l,3);

    float thrust_cmd_1l = (float) radio_control.values[RADIO_THROTTLE];
    Bound(thrust_cmd_1l,0,MAX_PPRZ);
    printf("thrust_cmd_1l: %f\n",thrust_cmd_1l);
    // stabilization_cmd[COMMAND_THRUST] = thrust_cmd_1l;
    for (i = 0; i < ANDI_NUM_ACT; i++) {
     a_thrust +=(thrust_cmd_1l - use_increment*actuator_state_1l[i]) * bwls_1l[2][i] / (ratio_u_un[i] * ratio_vn_v[i]);
    }
    printf("a_thrust: %f\n",a_thrust);
  }else{
    printf("I AM IN NAV\n");
    Wv[0] = 1.0;
    Wv[1] = 1.0;
      if(check_1st_nav){
        check_1st_nav = false;
          for (i = 0; i < 3; i++) {
            pos_init[i] = pos_1l[i];
          }
      }
      float rc_x = (float) (-radio_control.values[RADIO_PITCH])/9600.0*3.0;
      float rc_y = (float) (radio_control.values[RADIO_ROLL] )/9600.0*3.0;
      pos_des[0] = pos_init[0] + cosf(eulers_zxy.psi) * rc_x - sinf(eulers_zxy.psi) * rc_y;;
      pos_des[1] = pos_init[1] + sinf(eulers_zxy.psi) * rc_x + cosf(eulers_zxy.psi) * rc_y;;
      pos_des[2] = pos_init[2];
      printf("Desired position is :[%f, %f, %f]\n",pos_des[0],pos_des[1],pos_des[2]);
      
      rm_3rd(dt_1l, &pos_ref[0],   &pos_d_ref[0],  &pos_2d_ref[0], &pos_3d_ref[0], pos_des[0], k_N_rm, k_vN_rm, k_aN_rm);
      rm_3rd(dt_1l, &pos_ref[1],   &pos_d_ref[1],  &pos_2d_ref[1], &pos_3d_ref[1], pos_des[1], k_E_rm, k_vE_rm, k_aE_rm); 
      rm_3rd(dt_1l, &pos_ref[2],   &pos_d_ref[2],  &pos_2d_ref[2], &pos_3d_ref[2], pos_des[2], k_D_rm, k_vD_rm, k_aD_rm);   
  }

  att_ref[2] = eulers_zxy_des.psi;
  float des_r = (float) (radio_control.values[RADIO_YAW])/9600.0*2.0; //(eulers_zxy_des.psi-eulers_zxy.psi);
  BoundAbs(des_r,2.0);

  rm_3rd(dt_1l, &att_ref[0],   &att_d_ref[0],  &att_2d_ref[0], &att_3d_ref[0],     eulers_zxy_des.phi, k_phi_rm, k_p_rm, k_pdot_rm);
  rm_3rd(dt_1l, &att_ref[1],   &att_d_ref[1],  &att_2d_ref[1], &att_3d_ref[1],     eulers_zxy_des.theta, k_theta_rm, k_q_rm, k_qdot_rm);
  rm_2rd(dt_1l, &att_d_ref[2], &att_2d_ref[2], &att_3d_ref[2], des_r, k_r_rm, k_r_d_rm);
  
  // Generate pseudo control for stabilization vector (nu) based on error controller
  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
    nu[0] = 0.0;
    nu[1] = 0.0;
    nu[2] = a_thrust;
  }else{
    nu[0] = ec_3rd(pos_ref[0], pos_d_ref[0], pos_2d_ref[0], pos_3d_ref[0], pos_1l[0], pos_d[0], pos_2d[0], k_N_e, k_vN_e, k_aN_e);
    nu[1] = ec_3rd(pos_ref[1], pos_d_ref[1], pos_2d_ref[1], pos_3d_ref[1], pos_1l[1], pos_d[1], pos_2d[1], k_E_e, k_vE_e, k_aE_e);
    nu[2] = ec_3rd(pos_ref[2], pos_d_ref[2], pos_2d_ref[2], pos_3d_ref[2], pos_1l[2], pos_d[2], pos_2d[2], k_D_e, k_vD_e, k_aD_e); 
    printf("PVAJ D Ref    [%f, %f, %f, %f]\n",pos_ref[2],pos_d_ref[2],pos_2d_ref[2],pos_3d_ref[2]);
    printf("PVA  D Actual [%f, %f, %f]\n",pos_1l[2],pos_d[2],pos_2d[2]);
  }
  
  nu[3] = ec_3rd(att_ref[0], att_d_ref[0], att_2d_ref[0], att_3d_ref[0], att_1l[0], att_d[0], att_2d[0], k_phi_e, k_p_e, k_pdot_e);
  nu[4] = ec_3rd(att_ref[1], att_d_ref[1], att_2d_ref[1], att_3d_ref[1], att_1l[1], att_d[1], att_2d[1], k_theta_e, k_q_e, k_qdot_e);
  nu[5] = ec_2rd(att_d_ref[2], att_2d_ref[2], att_3d_ref[2], att_d[2], att_2d[2], k_r_e, k_r_d_e);

 
  // Calculate the min and max increments
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    if(i<ANDI_NUM_ACT){
      du_min[i]  = (act_min[i]    - use_increment * actuator_state_1l[i])/ratio_u_un[i];//
      du_max[i]  = (act_max[i]    - use_increment * actuator_state_1l[i])/ratio_u_un[i];//
      //du_pref[i] = (thrust_cmd_1l - use_increment * actuator_state_1l[i])/ratio_u_un[i];//
      du_pref[i] = (u_pref[i] - use_increment * actuator_state_1l[i])/ratio_u_un[i];
    }else{
      du_min[i]  = (act_min[i]    - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];
      du_max[i]  = (act_max[i]    - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];//
      du_pref[i] = (0.0           - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];//
    }
    }
    
    
    
    // if(autopilot.arming_status==3){
    //   printf("I AM ARMED\n");
    // }

  
    // WLS Control Allocator
    number_iter = wls_alloc_oneloop(andi_du_n, nu, du_min, du_max, bwls_1l, 0, 0, Wv, Wu, du_pref, gamma_wls, 10);

    printf("@@@@@@@@@ END WLS @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");

  for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
    printf("andi_du_pre_denormalization[%i]: %f ",i,andi_du_n[i]);
    andi_du[i] = (float)(andi_du_n[i] * ratio_u_un[i]);
  }
  printf("\n");
  if (volando) {
    // Add the increments to the actuators
    float_vect_sum(andi_u, actuator_state_1l, andi_du, ANDI_NUM_ACT);
    andi_u[ANDI_NUM_ACT_TOT-2] = andi_du[ANDI_NUM_ACT_TOT-2] + att_1l[0];
    andi_u[ANDI_NUM_ACT_TOT-1] = andi_du[ANDI_NUM_ACT_TOT-1] + att_1l[1];
  } else {
    // Not in flight, so don't increment
    float_vect_copy(andi_u, andi_du, ANDI_NUM_ACT);
    andi_u[ANDI_NUM_ACT_TOT-2] = andi_du[ANDI_NUM_ACT_TOT-2];
    andi_u[ANDI_NUM_ACT_TOT-1] = andi_du[ANDI_NUM_ACT_TOT-1];
  }

  // TODO : USE THE PROVIDED MAX AND MIN and change limits for phi and theta
  // Bound the inputs to the actuators
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    Bound(andi_u[i], act_min[i], act_max[i]);
  }
  // Propagate actuator filters
  get_act_state_oneloop();

  /*Commit the actuator command*/
  stabilization_cmd[COMMAND_THRUST] = 0;
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    actuators_pprz[i] = (int16_t) andi_u[i];
    stabilization_cmd[COMMAND_THRUST] += actuator_state_1l[i];
    //printf("Actuator %i set to %i\n",i,actuators_pprz[i]);
  }
  stabilization_cmd[COMMAND_THRUST] = stabilization_cmd[COMMAND_THRUST]/4.0;
  eulers_zxy_des.phi   =  andi_u[ANDI_NUM_ACT];
  eulers_zxy_des.theta =  andi_u[ANDI_NUM_ACT+1];


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

  struct FloatEulers *euler = stateGetNedToBodyEulers_f();
  float sphi   = sinf(euler->phi);
  float cphi   = cosf(euler->phi);
  float stheta = sinf(euler->theta);
  float ctheta = cosf(euler->theta);
  float spsi   = sinf(euler->psi);
  float cpsi   = cosf(euler->psi);
    //minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float T = -9.81;
  float scaler;
  // M0
  scaler = act_dynamics[0] * ratio_u_un[0] * ratio_vn_v[0] / ANDI_G_SCALING;

  g1g2_1l[0][0] = (sphi * spsi + cphi * cpsi * stheta) * g1_1l[2][0] * scaler;
  g1g2_1l[1][0] = (cphi * spsi * stheta - cpsi * sphi) * g1_1l[2][0] * scaler;
  g1g2_1l[2][0] = (cphi * ctheta                     ) * g1_1l[2][0] * scaler;
  g1g2_1l[3][0] = (g1_1l[3][0])                                      * scaler;
  g1g2_1l[4][0] = (g1_1l[4][0])                                      * scaler;
  g1g2_1l[5][0] = (g1_1l[5][0] + g2_1l[0])                           * scaler;
  // M1
  scaler = act_dynamics[1] * ratio_u_un[1] * ratio_vn_v[1] / ANDI_G_SCALING;
  g1g2_1l[0][1] = (sphi * spsi + cphi * cpsi * stheta) * g1_1l[2][1] * scaler;
  g1g2_1l[1][1] = (cphi * spsi * stheta - cpsi * sphi) * g1_1l[2][1] * scaler;
  g1g2_1l[2][1] = (cphi * ctheta                     ) * g1_1l[2][1] * scaler;
  g1g2_1l[3][1] = (g1_1l[3][1])                     * scaler;
  g1g2_1l[4][1] = (g1_1l[4][1])                     * scaler;
  g1g2_1l[5][1] = (g1_1l[5][1] + g2_1l[1])       * scaler;  
  // M2
  scaler = act_dynamics[2] * ratio_u_un[2] * ratio_vn_v[2] / ANDI_G_SCALING;
  g1g2_1l[0][2] = (sphi * spsi + cphi * cpsi * stheta) * g1_1l[2][2] * scaler;
  g1g2_1l[1][2] = (cphi * spsi * stheta - cpsi * sphi) * g1_1l[2][2] * scaler;
  g1g2_1l[2][2] = (cphi * ctheta                     ) * g1_1l[2][2] * scaler;
  g1g2_1l[3][2] = (g1_1l[3][2])                                      * scaler;
  g1g2_1l[4][2] = (g1_1l[4][2])                                      * scaler;
  g1g2_1l[5][2] = (g1_1l[5][2] + g2_1l[2])                           * scaler;  
  // M3
  scaler = act_dynamics[3] * ratio_u_un[3] * ratio_vn_v[3] / ANDI_G_SCALING;
  g1g2_1l[0][3] = (sphi * spsi + cphi * cpsi * stheta) * g1_1l[2][3] * scaler;
  g1g2_1l[1][3] = (cphi * spsi * stheta - cpsi * sphi) * g1_1l[2][3] * scaler;
  g1g2_1l[2][3] = (cphi * ctheta                     ) * g1_1l[2][3] * scaler;
  g1g2_1l[3][3] = (g1_1l[3][3])                                      * scaler;
  g1g2_1l[4][3] = (g1_1l[4][3])                                      * scaler;
  g1g2_1l[5][3] = (g1_1l[5][3] + g2_1l[3])                           * scaler; 
  // Phi
  scaler = act_dynamics[4] * ratio_u_un[4] * ratio_vn_v[4];
  g1g2_1l[0][4]= (cphi * spsi - sphi * cpsi * stheta)  * T * scaler;
  g1g2_1l[1][4]= (-sphi * spsi * stheta - cpsi * cphi) * T * scaler;
  g1g2_1l[2][4]= -ctheta * sphi                        * T * scaler;
  g1g2_1l[3][4]= 0.0;
  g1g2_1l[4][4]= 0.0;
  g1g2_1l[5][4]= 0.0;
  // Theta
  scaler = act_dynamics[5] * ratio_u_un[5] * ratio_vn_v[5];
  g1g2_1l[0][5] = (cphi * cpsi * ctheta) * T * scaler;
  g1g2_1l[1][5] = (cphi * spsi * ctheta) * T * scaler;
  g1g2_1l[2][5] = -stheta * cphi         * T * scaler;
  g1g2_1l[3][5] = 0.0;
  g1g2_1l[4][5] = 0.0;
  g1g2_1l[5][5] = 0.0;

 
  //printf("I am calculating what g1g2\n");
  // for (i = 0; i < ANDI_OUTPUTS; i++) {
  //   for (j = 0; j < ANDI_NUM_ACT_TOT; j++) {
  //     if (i != 5) {
        
  //     } else if (i !=5) {
  //       g1g2_1l[i][j] = (g1_1l[i][j] / ANDI_G_SCALING) * act_dynamics[j] * ratio_u_un[j] * ratio_vn_v[j];
  //     }
  //     } else {
  //       g1g2_1l[i][j] = ((g1_1l[i][j] + g2_1l[j]) / ANDI_G_SCALING) * act_dynamics[j] *ratio_u_un[j] * ratio_vn_v[j];
  //     }
  //   }
  }


void calc_normalization(void){
  //printf("I am calculating the normalization factors\n");
  int8_t i;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++){
    act_dynamics_d[i] = 1.0-exp(-act_dynamics[i]*dt_1l);
    Bound(act_dynamics_d[i],0.0,1.0);
    printf("discrete dynamics actuator %i is %f\n",i,act_dynamics_d[i]);
    ratio_vn_v[i] = 1.0;
    Bound(act_max[i],0,MAX_PPRZ);
    Bound(act_min[i],-MAX_PPRZ,0);
    ratio_u_un[i] = (act_max[i]-act_min[i])/(act_max_norm[i]-act_min_norm[i]);
  }
}

void calc_model(void){
  // TO DO switch to complementary filter on delta_u instead of u
  int8_t i;
  int8_t j;
 
  for (i = 0; i < ANDI_OUTPUTS; i++){
    model_pred[i] = 0.0;
    for (j = 0; j < ANDI_NUM_ACT_TOT; j++){
      if (j < ANDI_NUM_ACT){
        model_pred[i] = model_pred[i] + g1g2_1l[i][j] * actuator_state_1l[j]    / (act_dynamics[j] * ratio_u_un[j] * ratio_vn_v[j]);
      } else {
        //model_pred[i] = model_pred[i] + g1g2_1l[i][j] *  att_1l[j-ANDI_NUM_ACT] / (act_dynamics[j] *ratio_u_un[j] * ratio_vn_v[j]);
      }
    }
    printf("Model Prediction axis %i = %f\n",i,model_pred[i]);
  }
}

