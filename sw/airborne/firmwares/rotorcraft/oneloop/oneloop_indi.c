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

