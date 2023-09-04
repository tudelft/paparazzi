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
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
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

// Define default filter cutoff frequencies

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

// Define default settings for actuator properties

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
float  act_max[ANDI_NUM_ACT_TOT] = = {9600.0};
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

/*  Define Section of the functions used in this module*/
void  calc_normalization(void);
void  sum_g1g2_1l(void);
void  get_act_state_oneloop(void);
void  oneloop_andi_propagate_filters(void);
void  init_filter(void);
void  init_controller(void);
void  float_rates_of_euler_dot_vec(float r[3], float e[3], float edot[3]);
void  float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3]);
void  err_3d(float err[3], float a[3], float b[3], float k[3]);
void  integrate_3d(float dt, float a[3], float a_dot[3]);
void  vect_bound_3d(float vect[3], float bound);
float ho_bound(float dt, float x_d[3],float x_bound);
void  rm_2rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm);
void  rm_3rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm);
void  rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3]);
void  rm_3rd_pos(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], float k1_rm[3], float k2_rm[3], float k3_rm[3], float x_d_bound, float x_2d_bound);
void  ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3]);
void  calc_model(void);
void  straight_oval(float s, float r, float l, float psi_i, float v_route, float a_route, float j_route, float p[3], float p0[3],  float v[3], float a[3], float j[3], float psi_vec[4], float* lap);
void  nav_speed_controller(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_d_des, float x_d_actual, float x_2d_bound, float k1_rm, float k2_rm);
void  nav_speed_controller_enter(void);
float convert_angle(float psi);

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
                                        &pos_2d[2],
                                        &pos_3d_ref[0],
                                        &pos_3d_ref[1],
                                        &pos_3d_ref[2]); //andi_u
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
float gamma_wls                   = 1000.0;//0.01;//100;//100000.0;
static float Wv[ANDI_OUTPUTS]     = {1.0,1.0,1.0,10.0*100.0,10.0*100.0,100.0};//{0.0,0.0,5.0,10.0*100.0,10.0*100.0,0.0}; // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
static float Wu[ANDI_NUM_ACT_TOT] = {2.0, 2.0, 2.0,2.0,2.0,2.0}; // {de,dr,daL,daR,mF,mB,mL,mR,mP,phi,theta}
float u_pref[ANDI_NUM_ACT_TOT]    = {0.0,0.0,0.0,0.0,0.0,0.0};
float du_min[ANDI_NUM_ACT_TOT]; 
float du_max[ANDI_NUM_ACT_TOT];
float du_pref[ANDI_NUM_ACT_TOT];
float model_pred[ANDI_OUTPUTS];
float old_state[ANDI_NUM_ACT_TOT];
int   number_iter = 0;

/*Declaration of Reference Model and Error Controller Gains*/
float rm_k_attitude;
float p1_att = 7.0;//7.68;       
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
float p1_pos = 2.0;//2.0;     
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
float p1_head = 5.0;//4.8;     
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

/*NAV PVAJ Loop*/

float p1_nav = 1.0;
float p2_nav;
float k_v_nav;
float k_v_d_nav;

/* Declaration of Navigation Variables*/

float p_nav    ;
float v_nav    ;
float a_nav    ;
float j_nav    ;
float v_nav_des = 1.2;//2.5;
float max_a_nav = 8.0;
float max_v_nav = 5.0;

float r_oval    = 2.0;//5.0;
float l_oval    = 0.1;//10.0;
float psi_oval  = 0.0;
float p_oval[3] = {0.0,0.0,0.0};
float v_oval[3] = {0.0,0.0,0.0};
float a_oval[3] = {0.0,0.0,0.0};
float j_oval[3] = {0.0,0.0,0.0};
float lap_oval  = 0.0;
bool  oval_on   = false;
bool  ow_psi    = false;
float psi_des_rad = 0.0;
float psi_des_deg = 0.0;
float psi_vec[4] = {0.0, 0.0, 0.0, 0.0};

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
float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
float thrust_eff[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_G1_THRUST;

float ratio_u_un[ANDI_NUM_ACT_TOT];
float ratio_vn_v[ANDI_NUM_ACT_TOT];
float w_scale = 1.0;

float *bwls_1l[ANDI_OUTPUTS];

bool check_1st_nav = true; 
bool check_1st_oval= true;
bool verbose_oneloop = true;
bool heading_on = false;


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

/*Attitude Conversion Functions*/
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

void float_euler_dot_of_rates_vec(float r[3], float e[3], float edot[3])
{
  // To-Do bound ctheta to never be zero 
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

void err_3d(float err[3], float a[3], float b[3], float k[3])
{
  int8_t i;
  for (i = 0; i < 3; i++) {
    err[i] = k[i] * (a[i] - b[i]);
  }
}

void integrate_3d(float dt, float a[3], float a_dot[3])
{
  int8_t i;
  for (i = 0; i < 3; i++) {
    a[i] = a[i] + dt * a_dot[i];
  }
}

/* Scale a 3D vector to within a 3D bound */
void vect_bound_3d(float vect[3], float bound) {
  float norm = float_vect_norm(vect,3);
  if((norm-bound) > FLT_EPSILON) {
    float scale = bound/norm;
    vect[0] *= scale;
    vect[1] *= scale;
    vect[2] *= scale;
  }
}

/* Generate higher order bound based on lower order bound*/
float ho_bound(float dt, float x_d[3],float x_bound){
  float x_d_norm = float_vect_norm(x_d,3);
  printf("x_d_norm: %f\n", x_d_norm);
  printf("x_bound: %f\n", x_bound);
  float x_d_bound = (x_bound-x_d_norm)/dt;
  printf("x_d_bound: %f\n", x_d_bound);
  return fabs(x_d_bound);
}
/*Reference Model Definition*/
void rm_3rd_attitude(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], bool ow_psi, float psi_overwrite[4], float k1_rm[3], float k2_rm[3], float k3_rm[3]){
  float e_x[3];
  float e_x_rates[3];
  float e_x_d[3];
  float e_x_2d[3];
  float x_d_eul_ref[3];
  err_3d(e_x, x_des, x_ref, k1_rm);
  e_x[2] = k1_rm[2] * convert_angle((x_des[2] - x_ref[2])); // Correction for Heading error +-Pi
  float_rates_of_euler_dot_vec(e_x_rates, x_ref, e_x);
  err_3d(e_x_d, e_x_rates, x_d_ref, k2_rm);
  err_3d(e_x_2d, e_x_d, x_2d_ref, k3_rm);
  float_vect_copy(x_3d_ref,e_x_2d,3);
  if(ow_psi){x_3d_ref[2] = psi_overwrite[3];}
  integrate_3d(dt, x_2d_ref, x_3d_ref);
  if(ow_psi){x_2d_ref[2] = psi_overwrite[2];}
  integrate_3d(dt, x_d_ref, x_2d_ref);
  if(ow_psi){x_d_ref[2] = psi_overwrite[1];}
  float_euler_dot_of_rates_vec(x_d_ref, x_ref, x_d_eul_ref);
  integrate_3d(dt, x_ref, x_d_eul_ref);
  if(ow_psi){x_ref[2] = psi_overwrite[0];}
  x_ref[2] = convert_angle(x_ref[2]);
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

void rm_3rd_head(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_des, float k1_rm, float k2_rm, float k3_rm){
  float e_x      = k1_rm * convert_angle((x_des- *x_ref));
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  float e_x_2d   = k3_rm * (e_x_d- *x_2d_ref);
  *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

void rm_3rd_pos(float dt, float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x_des[3], float k1_rm[3], float k2_rm[3], float k3_rm[3], float x_d_bound, float x_2d_bound){
  float e_x[3];
  float e_x_d[3];
  float e_x_2d[3];
  printf("x_d_bound: %f\n", x_d_bound);
  printf("norm(x_d_ref): %f\n", float_vect_norm(x_d_ref,3));
  float x_2d_bound_vel = 1000.0;//(x_d_bound - float_vect_norm(x_d_ref,3)) / dt;
  printf("x_2d_bound_vel: %f\n", x_2d_bound_vel);
  float x_2d_bound_gen = fminf(x_2d_bound_vel, x_2d_bound);
  printf("x_2d_bound_gen: %f\n", x_2d_bound_gen);
  float x_3d_bound = ho_bound(dt, x_2d_ref, x_2d_bound_gen);
  printf("x_3d_bound: %f\n", x_3d_bound);
  err_3d(e_x, x_des, x_ref, k1_rm);
  vect_bound_3d(e_x,x_d_bound);
  printf("e_x: %f\n", e_x[0]);
  err_3d(e_x_d, e_x, x_d_ref, k2_rm);
  vect_bound_3d(e_x_d,x_2d_bound);
  printf("e_x_d: %f\n", e_x_d[0]);
  err_3d(e_x_2d, e_x_d, x_2d_ref, k3_rm);
  printf("e_x_2d: %f\n", e_x_2d[0]);
  printf("e_x_2d: %f\n", e_x_2d[0]);
  float_vect_copy(x_3d_ref,e_x_2d,3);
  integrate_3d(dt, x_2d_ref, x_3d_ref);
  vect_bound_3d(x_2d_ref, x_2d_bound);
  integrate_3d(dt, x_d_ref, x_2d_ref);
  vect_bound_3d(x_d_ref, x_d_bound);
  integrate_3d(dt, x_ref, x_d_ref);
}



void rm_2rd(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float x_des, float k1_rm, float k2_rm){
  float e_x      = k1_rm * (x_des- *x_ref);
  float e_x_d    = k2_rm * (e_x- *x_d_ref);
  *x_2d_ref = e_x_d;
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));
}

/*Error Controller Definition*/
static float ec_3rd(float x_ref, float x_d_ref, float x_2d_ref, float x_3d_ref, float x, float x_d, float x_2d, float k1_e, float k2_e, float k3_e){
  float y_4d = k1_e*(x_ref-x)+k2_e*(x_d_ref-x_d)+k3_e*(x_2d_ref-x_2d)+x_3d_ref;
  return y_4d;
}
static float ec_3rd_head(float x_ref, float x_d_ref, float x_2d_ref, float x_3d_ref, float x, float x_d, float x_2d, float k1_e, float k2_e, float k3_e){
  float y_4d = k1_e*(convert_angle(x_ref-x))+k2_e*(x_d_ref-x_d)+k3_e*(x_2d_ref-x_2d)+x_3d_ref;
  return y_4d;
}
void ec_3rd_att(float y_4d[3], float x_ref[3], float x_d_ref[3], float x_2d_ref[3], float x_3d_ref[3], float x[3], float x_d[3], float x_2d[3], float k1_e[3], float k2_e[3], float k3_e[3]){
  float y_4d_1[3];
  float y_4d_2[3];
  float y_4d_3[3];
  err_3d(y_4d_1, x_ref, x, k1_e);
  y_4d_1[2] = k1_e[2] * convert_angle((x_ref[2] - x[2])); // Correction for Heading error +-Pi
  err_3d(y_4d_2, x_d_ref, x_d, k2_e);
  err_3d(y_4d_3, x_2d_ref, x_2d, k3_e);
  float_vect_copy(y_4d,x_3d_ref,3);
  float_vect_sum(y_4d, y_4d, y_4d_1, 3);
  float_vect_sum(y_4d, y_4d, y_4d_2, 3);
  float_vect_sum(y_4d, y_4d, y_4d_3, 3);
  //return y_4d;
}
static float ec_2rd(float x_ref, float x_d_ref, float x_2d_ref, float x, float x_d, float k1_e, float k2_e){
  float y_3d = k1_e*(x_ref-x)+k2_e*(x_d_ref-x_d)+x_2d_ref;
  return y_3d;
}
/*Third Order Dynamics Approximation*/
static float w_approx(float p1, float p2, float p3, float rm_k){
  float tao = (p1*p2+p1*p3+p2*p3)/(p1*p2*p3)/(rm_k);
  return 1.0*w_scale/tao;
}

/** Gain Design
 * @brief This section is used to define the gains of the reference model and error controller
 * FIXME: Calculate the gains dynamically for transition
 */

void init_controller(void){
/*Attitude Loop*/
rm_k_attitude = 0.8;
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

/*Nav PVAJ Loop*/
p2_nav        = p1_nav;
k_v_nav       = k_rm_1_2_f(p1_nav,p2_nav);
k_v_d_nav     = k_rm_2_2_f(p1_nav,p2_nav);


/*Approximated Dynamics*/

act_dynamics[ANDI_NUM_ACT_TOT-2]   = w_approx( p1_att,  p2_att,  p3_att,  1.0);//rm_k_attitude);
act_dynamics[ANDI_NUM_ACT_TOT-1]   = w_approx( p1_att,  p2_att,  p3_att,  1.0);//rm_k_attitude);
}

/** state eulers in zxy order */
struct FloatEulers eulers_zxy;


/*Filters Initialization*/
float oneloop_andi_estimation_filt_cutoff = 2.0;

static struct FirstOrderLowPass filt_accel_ned[3];
//Butterworth2LowPass filt_accel_ned[3];
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
static struct FirstOrderLowPass pos_filt_fo[3];
static struct FirstOrderLowPass vel_filt_fo[3];
Butterworth2LowPass model_pred_filt[ANDI_OUTPUTS];
static struct FirstOrderLowPass model_pred_a_filt[3];
struct FloatVect3 body_accel_f;


void init_filter(void)
{
  printf("init of the filters");
  float tau = 1.0 / (2.0 * M_PI *oneloop_andi_filt_cutoff);
  printf("tau = %f\n",tau);
  float tau_a = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_a);
  printf("tau_a = %f\n",tau_a);
  float tau_v = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_v);
  float tau_p = 1.0 / (2.0 * M_PI * oneloop_andi_filt_cutoff_p);
  printf("tau_p = %f\n",tau_p);
  printf("oneloop_andi_filt_cutoff: %f\n", oneloop_andi_filt_cutoff);
  printf("oneloop_andi_filt_cutoff_a: %f\n", oneloop_andi_filt_cutoff_a);
  printf("oneloop_andi_filt_cutoff_v: %f\n", oneloop_andi_filt_cutoff_v);
  printf("oneloop_andi_filt_cutoff_p: %f\n", oneloop_andi_filt_cutoff_p);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  // Filtering of the Inputs with 3 dimensions (e.g. rates and accelerations)
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], tau, sample_time, 0.0);
    //init_butterworth_2_low_pass(&filt_accel_ned[i], tau_a, sample_time, 0.0);
    init_first_order_low_pass(&filt_accel_ned[i], tau_a, sample_time, 0.0 );
    init_butterworth_2_low_pass(&filt_accel_body[i], tau_a, sample_time, 0.0);
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
       init_first_order_low_pass(&model_pred_a_filt[i], tau_a, sample_time, 0.0 );
       
    } else {
    init_butterworth_2_low_pass(&model_pred_filt[i], tau, sample_time, 0.0);
    }
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
  float pos_vect[3]  = {stateGetPositionNed_f()->x,stateGetPositionNed_f()->y,stateGetPositionNed_f()->z};
  float vel_vect[3]  = {stateGetSpeedNed_f()->x,stateGetSpeedNed_f()->y,stateGetSpeedNed_f()->z};
  printf("oneloop_andi_filt_cutoff_a = %f\n",oneloop_andi_filt_cutoff_a);
  //printf("tau_a = %f\n",tau_a);
  // update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  // update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  // update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);
  update_first_order_low_pass(&filt_accel_ned[0], accel->x);
  update_first_order_low_pass(&filt_accel_ned[1], accel->y);
  update_first_order_low_pass(&filt_accel_ned[2], accel->z);
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
    update_first_order_low_pass(&model_pred_a_filt[i], model_pred[i]);
    update_butterworth_2_low_pass(&att_dot_meas_lowpass_filters[i], rate_vect[i]);
    //float old_rate = rates_filt_fo[i].last_out;
    update_first_order_low_pass(&rates_filt_fo[i], rate_vect[i]);
    update_first_order_low_pass(&pos_filt_fo[i], pos_vect[i]);
    update_first_order_low_pass(&vel_filt_fo[i], vel_vect[i]);
 
    ang_acc[i] = (att_dot_meas_lowpass_filters[i].o[0]- att_dot_meas_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY + model_pred[3+i] - model_pred_filt[3+i].o[0];
    //lin_acc[i] = filt_accel_ned[i].o[0] + model_pred[i] - model_pred_filt[i].o[0];  
    lin_acc[i] = filt_accel_ned[i].last_out + model_pred[i] - model_pred_a_filt[i].last_out; 
    //ang_acc[i] = (rates_filt_fo[i].last_out- old_rate) * PERIODIC_FREQUENCY;
    
}}

/*Init function of oneloop controller*/
void oneloop_andi_init(void)
{ 
  //Calculate g1_g2. 
  calc_normalization();
  sum_g1g2_1l();

  // Initialize the array of pointers to the rows of g1_g2
  int8_t i;
  for (i = 0; i < ANDI_OUTPUTS; i++) {
   bwls_1l[i] = g1g2_1l[i];
  }

  // Initialize filters
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
  float_vect_zero(pos_des,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;
  float_vect_zero(model_pred,ANDI_OUTPUTS);
  nav_speed_controller_enter();

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
  float_vect_zero(att_ref,3);
  float_vect_zero(att_d_ref,3);
  float_vect_zero(att_2d_ref,3);
  float_vect_zero(att_3d_ref,3);
  float_vect_zero(ang_acc,3);
  float_vect_zero(lin_acc,3);
  float_vect_zero(pos_des,3);
  eulers_zxy_des.phi   =  0.0;
  eulers_zxy_des.theta =  0.0;
  eulers_zxy_des.psi   =  0.0;
  float_vect_zero(model_pred,ANDI_OUTPUTS);
  nav_speed_controller_enter();
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

  printf("k_N_e = %f\n",k_N_e);
  printf("k_vN_e = %f\n",k_vN_e);
  printf("k_aN_e = %f\n",k_aN_e);
  printf("k_E_e = %f\n",k_E_e);
  printf("k_vE_e = %f\n",k_vE_e);
  printf("k_aE_e = %f\n",k_aE_e);
  printf("k_D_e = %f\n",k_D_e);
  printf("k_vD_e = %f\n",k_vD_e);
  printf("k_aD_e = %f\n",k_aD_e);

  init_controller();
  calc_normalization();
  sum_g1g2_1l();

  // If drone is not on the ground use incremental law
  float use_increment = 0.0;
  bool volando = false;
  if(in_flight) {
    use_increment = 1.0;
    volando = true;
    printf("I am in flight\n");
    }

  psi_des_rad = psi_des_deg * M_PI / 180.0;

  eulers_zxy.phi   = stateGetNedToBodyEulers_f()->phi;
  eulers_zxy.theta = stateGetNedToBodyEulers_f()->theta;
  eulers_zxy.psi   = stateGetNedToBodyEulers_f()->psi;
  
  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
    printf("I AM IN ATT\n");
    eulers_zxy_des.phi   = (float) (radio_control.values[RADIO_ROLL] )/9600.0*45.0*3.14/180.0;//0.0;
    eulers_zxy_des.theta = (float) (radio_control.values[RADIO_PITCH])/9600.0*45.0*3.14/180.0;//0.0;
    eulers_zxy_des.psi   = eulers_zxy.psi;//
    psi_des_rad          = eulers_zxy.psi;
    check_1st_nav  = true;
    check_1st_oval = true;
  }
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

  pos_1l[0] = stateGetPositionNed_f()->x;   // pos_filt_fo[0].last_out;s
  pos_1l[1] = stateGetPositionNed_f()->y;   // pos_filt_fo[1].last_out;s
  pos_1l[2] = stateGetPositionNed_f()->z;   // pos_filt_fo[2].last_out;s
  pos_d[0]  = stateGetSpeedNed_f()->x;      // vel_filt_fo[0].last_out;s
  pos_d[1]  = stateGetSpeedNed_f()->y;      // vel_filt_fo[1].last_out;s
  pos_d[2]  = stateGetSpeedNed_f()->z;      // vel_filt_fo[2].last_out;s
  //struct NedCoor_f *accel = stateGetAccelNed_f();
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
    for (i = 0; i < ANDI_NUM_ACT; i++) {
     a_thrust +=(thrust_cmd_1l - use_increment*actuator_state_1l[i]) * bwls_1l[2][i] / (ratio_u_un[i] * ratio_vn_v[i]);
    }
    printf("a_thrust: %f\n",a_thrust);
  }else{
    printf("I AM IN NAV\n");
    Wv[0] = 1.0;
    Wv[1] = 1.0;
      if(check_1st_nav || (check_1st_oval && oval_on)){ // First time engaging NAV
        check_1st_nav  = false;
        check_1st_oval = false;
          for (i = 0; i < 3; i++) {
            pos_init[i] = pos_1l[i];
          }
        psi_des_rad = eulers_zxy.psi; // At first NAV reset desired heading to current heading
        psi_oval = psi_des_rad; 

      } else { // Use the horizontal position input of the wp
        pos_init[0] = (float) (guidance_h.sp.pos.x * 0.0039063);
        pos_init[1] = (float) (guidance_h.sp.pos.y * 0.0039063);
        pos_init[2] = (float) (guidance_v.z_sp     * 0.0039063);
      }
      float rc_x = (float) (-radio_control.values[RADIO_PITCH])/9600.0*3.0;
      float rc_y = (float) (radio_control.values[RADIO_ROLL] )/9600.0*3.0;
      if(oval_on){
        float v_actual = sqrtf(pos_d[0] * pos_d[0]+pos_d[1] * pos_d[1]);
        
        nav_speed_controller(dt_1l, &p_nav, &v_nav, &a_nav, &j_nav, v_nav_des, v_actual, max_a_nav, k_v_nav, k_v_d_nav);
        printf("p_nav = %f\n",p_nav);
        
        straight_oval(p_nav, r_oval, l_oval, psi_oval, v_nav, a_nav, j_nav, pos_ref, pos_init,  pos_d_ref, pos_2d_ref, pos_3d_ref, psi_vec, &lap_oval);
        if (v_actual > 0.5){
          //if (heading_on){
            //psi_des_rad = atan2f(pos_d_ref[1], pos_d_ref[0]); // Investigate whether to use the ref or the actual signal
            //psi_des_rad = convert_angle(psi_des_rad);
          //}
        }
        
      }else{
        check_1st_oval = true;
        
        nav_speed_controller_enter();
        pos_des[0] = pos_init[0] + cosf(eulers_zxy.psi) * rc_x - sinf(eulers_zxy.psi) * rc_y;;
        pos_des[1] = pos_init[1] + sinf(eulers_zxy.psi) * rc_x + cosf(eulers_zxy.psi) * rc_y;;
        pos_des[2] = pos_init[2];
        float k1_pos_rm[3] = {k_N_rm,  k_E_rm,  k_D_rm};
        float k2_pos_rm[3] = {k_vN_rm, k_vE_rm, k_vD_rm};
        float k3_pos_rm[3] = {k_aN_rm, k_aE_rm, k_aD_rm};
        rm_3rd_pos(dt_1l, pos_ref, pos_d_ref, pos_2d_ref, pos_3d_ref, pos_des, k1_pos_rm, k2_pos_rm, k3_pos_rm, max_v_nav, max_a_nav);
      }
      printf("Desired position is :[%f, %f, %f]\n",pos_des[0],pos_des[1],pos_des[2]);
       
  }

  
  float des_r = 0.0;
  
  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
    att_ref[2] = psi_des_rad;
    des_r = (float) (radio_control.values[RADIO_YAW])/9600.0*3.0; //(eulers_zxy_des.psi-eulers_zxy.psi);
    BoundAbs(des_r,5.0);
    rm_2rd(dt_1l, &att_d_ref[2], &att_2d_ref[2], &att_3d_ref[2], des_r, k_r_rm, k_r_d_rm);
    rm_3rd(dt_1l, &att_ref[0],   &att_d_ref[0],  &att_2d_ref[0], &att_3d_ref[0], eulers_zxy_des.phi, k_phi_rm, k_p_rm, k_pdot_rm);
    rm_3rd(dt_1l, &att_ref[1],   &att_d_ref[1],  &att_2d_ref[1], &att_3d_ref[1], eulers_zxy_des.theta, k_theta_rm, k_q_rm, k_qdot_rm);
  } else {
    float k1_att_rm[3] = {k_phi_rm, k_theta_rm, k_psi_rm};
    float k2_att_rm[3] = {k_p_rm, k_q_rm, k_psi_d_rm};
    float k3_att_rm[3] = {k_pdot_rm, k_qdot_rm, k_psi_2d_rm};
    float att_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, psi_des_rad};
    
    if(oval_on && heading_on) {
      ow_psi = true;
    }else{
      ow_psi = false;
    }

    rm_3rd_attitude(dt_1l, att_ref, att_d_ref, att_2d_ref, att_3d_ref, att_des, ow_psi, psi_vec, k1_att_rm, k2_att_rm, k3_att_rm);

    if(ow_psi) {
      att_ref[2]    = psi_vec[0];
      att_d_ref[2]  = psi_vec[1];
      att_2d_ref[2] = psi_vec[2];
      att_3d_ref[2] = psi_vec[3];
    }
  }

  // Generate pseudo control for stabilization vector (nu) based on error controller
  if(autopilot.mode==AP_MODE_ATTITUDE_DIRECT){
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
    printf("pos_3d_ref[0]: %f\n",pos_3d_ref[0]);
    printf("pos_3d_ref[1]: %f\n",pos_3d_ref[1]);
    printf("pos_3d_ref[2]: %f\n",pos_3d_ref[2]);
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
      du_min[i]  = (act_min[i]    - use_increment * actuator_state_1l[i])/ratio_u_un[i];//
      du_max[i]  = (act_max[i]    - use_increment * actuator_state_1l[i])/ratio_u_un[i];//
      du_pref[i] = (u_pref[i] - use_increment * actuator_state_1l[i])/ratio_u_un[i];
    }else{
      du_min[i]  = (act_min[i]    - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];
      du_max[i]  = (act_max[i]    - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];//
      du_pref[i] = (0.0           - use_increment * att_1l[i-ANDI_NUM_ACT])/ratio_u_un[i];//
    }
    }
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
  }
  stabilization_cmd[COMMAND_THRUST] = stabilization_cmd[COMMAND_THRUST]/4.0;
  eulers_zxy_des.phi   =  andi_u[ANDI_NUM_ACT];
  eulers_zxy_des.theta =  andi_u[ANDI_NUM_ACT+1];

  psi_des_deg = psi_des_rad * 180.0 / M_PI;

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
    Bound(actuator_state_1l[i],0,MAX_PPRZ);
    // TO DO ACTUATORS BOUND ACTUATOR_STATE_1L TO NOT BE ZERO!!!!!
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

  float T = -9.81; //minus gravity is a guesstimate of the thrust force, thrust measurement would be better
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
  }


void calc_normalization(void){
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
    //model_pred[i] = 0.0;
    for (j = 0; j < ANDI_NUM_ACT_TOT; j++){
      //THIS MIGHT BE WRONG
      if (j < ANDI_NUM_ACT){
        model_pred[i] = model_pred[i] +  (actuator_state_1l[j] - old_state[j]) * g1g2_1l[i][j]    / (act_dynamics[j] * ratio_u_un[j] * ratio_vn_v[j]);
      } else {
        model_pred[i] = model_pred[i] +  (att_1l[j-ANDI_NUM_ACT] - old_state[j]) * g1g2_1l[i][j]  / (act_dynamics[j] * ratio_u_un[j] * ratio_vn_v[j]);
      }
    }
     printf("Model Prediction axis %i = %f\n",i,model_pred[i]);
  }
  // Store the old state of the actuators for the next iteration
  for ( j = 0; j < ANDI_NUM_ACT_TOT; j++){
    if(j < ANDI_NUM_ACT){
      old_state[j] = actuator_state_1l[j];
    } else {
      old_state[j] = att_1l[j-ANDI_NUM_ACT];
    }
  }
}

/**
 * Navigation speed controller function.
 *
 * This function calculates the control signal for the navigation speed controller based on the desired
 * speed (x_d_des) and the actual speed (x_d_actual) of a system. It updates the reference values
 * for velocity (x_d_ref), acceleration (x_2d_ref), jerk (x_3d_ref) and position (x_ref) using a proportional controller.
 *
 * @param dt          Time step between control updates.
 * @param x_ref       Pointer to the reference position.
 * @param x_d_ref     Pointer to the reference velocity.
 * @param x_2d_ref    Pointer to the reference acceleration.
 * @param x_3d_ref    Pointer to the reference jerk.
 * @param x_d_des     Desired velocity.
 * @param x_d_actual  Actual velocity.
 * @param x_2d_bound  Upper bound for the control acceleration.
 * @param k1_rm       Proportional control gain velocity.
 * @param k2_rm       Proportional control gain acceleration.
 */

void nav_speed_controller_enter(){
  p_nav     = 0.0;
  v_nav     = 0.0;
  a_nav     = 0.0;
  j_nav     = 0.0;
  max_a_nav = 8.0;
  max_v_nav = 5.0;
}

void nav_speed_controller(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_d_des, float x_d_actual, float x_2d_bound, float k1_rm, float k2_rm){
  float e_x_d      = k1_rm * (x_d_des- x_d_actual);
  float e_x_2d     = k2_rm * (e_x_d- *x_2d_ref);
  float x_3d_bound = (x_2d_bound - fabs(*x_2d_ref)) / dt;
  BoundAbs(e_x_2d, x_3d_bound);
  *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));  
}
/**
 * @brief the position, velocity, acceleration, jerk, and lap count for a point moving along a straight oval path.
 *
 * @param s         The distance along the path.
 * @param r         The radius of the corners of the oval path.
 * @param l         The length of the straight sections of the oval path.
 * @param psi_i     The heading angle of the oval path.
 * @param v_route   The velocity magnitude along the path.
 * @param a_route   The acceleration magnitude along the path.
 * @param j_route   The jerk magnitude along the path.
 * @param p         Array of length 3 representing the position vector [x, y, z] of the point on the path.
 * @param v         Array of length 3 representing the velocity vector [Vx, Vy, Vz] of the point on the path.
 * @param a         Array of length 3 representing the acceleration vector [Ax, Ay, Az] of the point on the path.
 * @param j         Array of length 3 representing the jerk vector [Jx, Jy, Jz] of the point on the path.
 * @param lap       Pointer to a variable that stores the number of laps completed around the oval path.
 */

//void straight_oval(float s, float r, float l, float psi_i, float v_route, float a_route, float j_route, float* p, float* v, float* a, float* j, float* lap)
void straight_oval(float s, float r, float l, float psi_i, float v_route, float a_route, float j_route, float p[3], float pi[3], float v[3], float a[3], float j[3], float psi_vec[4], float* lap) {
    printf("psi_oval = %f\n",psi_i);
    float cd = cosf(psi_i);
    float sd = sinf(psi_i);
    float head_vec[4] = {0.0, 0.0, 0.0, 0.0};
    float p0[3] = {0.0, 0.0, 0.0};
    //float pi[3] = {0.0,0.0,0.0};
    float p1[3] = {p0[0] + r, p0[1]-r, p0[2]};
    float p2[3] = {p1[0] + l, p0[1]-r, p0[2]};

    // Calculate inner vertices
    float p1i[3] = {p1[0], p1[1] + r, p1[2]};
    float p2i[3] = {p2[0], p2[1] + r, p2[2]};
    float p3i[3] = {p2[0], p2[1] - r, p2[2]};
    float p4i[3] = {p1[0], p1[1] - r, p1[2]};
    float l01 = 1.0; 
    float l12 = 1.0;
    float l34 = 1.0;
    
    float tmp_1[3] = {0.,0.,0.};
    float tmp_2[3] = {0.,0.,0.};
    float tmp_3[3] = {0.,0.,0.};
    float_vect_diff(tmp_1,p1i,p0,3);
    float_vect_diff(tmp_2,p2i,p1i,3);
    float_vect_diff(tmp_3,p4i,p3i,3);
    l01 = float_vect_norm(tmp_1,3);
    l12 = float_vect_norm(tmp_2,3);
    l34 = float_vect_norm(tmp_3,3);
    
    float u01[3] = {0.,0.,0.};  
    float u12[3] = {0.,0.,0.};
    float u34[3] = {0.,0.,0.};

    float_vect_sdiv(u01,tmp_1,l01,3);
    float_vect_sdiv(u12,tmp_2,l12,3);
    float_vect_sdiv(u34,tmp_3,l34,3);
    float runway = r;
    int8_t i;
    if (s < runway) {
      for (i = 0; i < 3; i++){
        p[i] = p0[i] + s * u01[i];
        v[i] = v_route * u01[i];
        a[i] = a_route * u01[i];
        j[i] = j_route * u01[i];
      }
    } else {
        s = s - runway;

        // Calculate length of route
        float L = 2 * M_PI * r + l12 + l34;
        float s1 = l12;
        float s2 = s1 + M_PI * r;
        float s3 = s2 + l34;
        float s4 = s3 + M_PI * r;
        float dist = fmodf(s, L);
        *lap = floorf(s / (1.05f * L));
        if (dist < s1) {
            float sector = dist;
              for (i = 0; i < 3; i++){
                p[i] = p1i[i] + sector * u12[i];
                v[i] = v_route * u12[i];
                a[i] = a_route * u12[i];
                j[i] = j_route * u12[i];
              }
        } else if (dist < s2) {
            float sector = dist - s1;
            float beta = sector / r;
            float sb = sinf(beta);
            float cb = cosf(beta);
            float v2 = v_route * v_route;
            float v3 = v2 * v_route;
            float r2 = r * r;
            head_vec[0] = beta;
            head_vec[1] = v_route / r;
            head_vec[2] = a_route / r;
            head_vec[3] = j_route / r; 

            p[0] = r * sb + p2[0];
            p[1] = r * cb + p2[1];
            p[2] = p2[2];
            v[0] =  cb * v_route;
            v[1] = -sb * v_route;
            v[2] = 0; 
            a[0] =  cb * a_route - sb * v2 /  r;
            a[1] = -sb * a_route - cb * v2 / r;
            a[2] = 0 ;
            j[0] = cb * j_route - cb * v3 / r2 - 3 * sb * a_route * v_route / r;
            j[1] = sb * v3 / r2 - sb * j_route - 3 * cb * a_route * v_route / r;
            j[2] = 0;
            //v[0] = v_route * cosf(beta);
            //v[1] = -v_route * sinf(beta);
            //v[2] = 0;
            //a[0] = v_route * v_route / r * (-sinf(beta));
            //a[1] = v_route * v_route / r * (-cosf(beta));
            //a[2] = 0;
            //j[0] = 0;
            //j[1] = 0;
            //j[2] = 0;
        } else if (dist < s3) {
            float sector = dist - s2;
               for (i = 0; i < 3; i++){
                p[i] = p3i[i] + sector * u34[i];
                v[i] = v_route * u34[i];
                a[i] = a_route * u34[i];
                j[i] = j_route * u34[i];
                head_vec[0] = M_PI;
              }           
        } else if (dist < s4) {
            float sector = dist - s3;
            float beta = M_PI + sector / r;
            float sb = sinf(beta);
            float cb = cosf(beta);
            float v2 = v_route * v_route;
            float v3 = v2 * v_route;
            float r2 = r * r;
            head_vec[0] = beta;
            head_vec[1] = v_route / r;
            head_vec[2] = a_route / r;
            head_vec[3] = j_route / r;

            p[0] = r * sb + p1[0];
            p[1] = r * cb + p1[1];
            p[2] = p1[2];
            v[0] =  cb * v_route;
            v[1] = -sb * v_route;
            v[2] = 0; 
            a[0] =  cb * a_route - sb * v2 / r;
            a[1] = -sb * a_route - cb * v2 / r;
            a[2] = 0 ;
            j[0] = cb * j_route - cb * v3 / r2 - 3 * sb * a_route * v_route / r;
            j[1] = sb * v3 / r2 - sb * j_route - 3 * cb * a_route * v_route / r;
            j[2] = 0;            
        }
    }

    float temp_p[3] = {
        cd * p[0] - sd * p[1],
        sd * p[0] + cd * p[1],
        p[2]
    };
    float temp_v[3] = {
        cd * v[0] - sd * v[1],
        sd * v[0] + cd * v[1],
        v[2]
    };
    float temp_a[3] = {
        cd * a[0] - sd * a[1],
        sd * a[0] + cd * a[1],
        a[2]
    };
    float temp_j[3] = {
        cd * j[0] - sd * j[1],
        sd * j[0] + cd * j[1],
        j[2]
    };
    for (i = 0; i < 3; i++){
      p[i] = temp_p[i] + pi[i];
      v[i] = temp_v[i];
      a[i] = temp_a[i];
      j[i] = temp_j[i];
    }
    psi_vec[0] = convert_angle(-head_vec[0] + psi_i);
    psi_vec[1] = -head_vec[1];
    psi_vec[2] = -head_vec[2];
    psi_vec[3] = -head_vec[3];
}

/**
 * Converts an angle in radians to an angle between -pi and pi.
 *
 * @param psi The input angle in radians.
 * @return The converted angle between -pi and pi.
 */
float convert_angle(float psi) {
    // Compute the equivalent angle between -2pi and 2pi
    psi = fmodf(psi, 2 * M_PI);

    if (psi < -M_PI)
        psi += 2 * M_PI;
    else if (psi > M_PI)
        psi -= 2 * M_PI;

    // Convert to angle between -pi and pi
    if (psi <= M_PI)
        return psi;
    else
        return psi - 2 * M_PI;
}