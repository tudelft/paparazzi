/*
 * Copyright (C) 2021 A. Mancinelli
 *
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
 * @file "modules/Overactuated_vehicle/Overactuated_vehicle.c"
 * @author Alessandro Mancinelli
 * Control laws for Overactuated Vehicle
 */

#include "Overactuated_mixing.h"
#include <math.h>
#include "subsystems/radio_control.h"
#include "state.h"
#include "paparazzi.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/navigation/waypoints.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "subsystems/actuators/motor_mixing.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.c"

struct overactuated_mixing_t overactuated_mixing;

float phi, theta, psi, p,q,r,ax,ay,az, x, y, z, u, v, w;

// PID and general settings from slider
float P_az_gain = 12.7 ;
float I_az_gain = 1 ;
float D_az_gain = 0.002 ;
float P_el_gain = 12.9 ;
float I_el_gain = 1 ;
float D_el_gain = 0.002 ;
int Deadband_stick = 500;
float Stick_gain_position = 0.1; // Stick to position gain
bool activate_longitudinal_over = 1;
bool activate_lateral_over = 1;
bool activate_yaw_over = 1;
bool manual_yaw_overactuated = 0;
float wind_speed = 0;

float desired_x_e = 0;
float desired_y_e = 0;
float desired_z_e = 0;
float lateral_cmd_old = 0;
float longitudinal_cmd_old = 0;
float x_stb, y_stb, z_stb;
float elevation_cmd = 0;
float azimuth_cmd = 0;
float yaw_cmd = 0;

Butterworth2LowPass measurement_lowpass_filters[3];
struct FloatVect3 body_accel_f;

void Compute_B_matrix(float **B_matrix, float I_xx,float I_yy,float I_zz,float J_r,float l_1,float l_2,float l_3,float l_4,float l_z,float m,float K_p_T,float K_p_M,float *Omega_sens,float *b_sens,float *g_sens,float *Omega_dot_sens,float *b_dot_sens,float *g_dot_sens, float *Euler_rad, float *pqr_rad_s){
    //Transpose pointer to local variables:
    float Omega_1, Omega_2, Omega_3, Omega_4;
    Omega_1 = Omega_sens[0];
    Omega_2 = Omega_sens[1];
    Omega_3 = Omega_sens[2];
    Omega_4 = Omega_sens[3];
    float b_1, b_2, b_3, b_4, g_1, g_2, g_3, g_4;
    b_1 = b_sens[0];
    b_2 = b_sens[1];
    b_3 = b_sens[2];
    b_4 = b_sens[3];
    g_1 = g_sens[0];
    g_2 = g_sens[1];
    g_3 = g_sens[2];
    g_4 = g_sens[3];

    float Omega_1_dot, Omega_2_dot, Omega_3_dot, Omega_4_dot;
    Omega_1_dot = Omega_dot_sens[0];
    Omega_2_dot = Omega_dot_sens[1];
    Omega_3_dot = Omega_dot_sens[2];
    Omega_4_dot = Omega_dot_sens[3];
    float b_1_dot, b_2_dot, b_3_dot, b_4_dot, g_1_dot, g_2_dot, g_3_dot, g_4_dot;
    b_1_dot = b_dot_sens[0];
    b_2_dot = b_dot_sens[1];
    b_3_dot = b_dot_sens[2];
    b_4_dot = b_dot_sens[3];
    g_1_dot = g_dot_sens[0];
    g_2_dot = g_dot_sens[1];
    g_3_dot = g_dot_sens[2];
    g_4_dot = g_dot_sens[3];


    float Phi, Theta, Psi, p, q, r;
    Phi = Euler_rad[0];
    Theta = Euler_rad[1];
    Psi = Euler_rad[2];
    p = pqr_rad_s[0];
    q = pqr_rad_s[1];
    r = pqr_rad_s[2];

    //Motor disposition
    float l_1_x = -l_4;
    float l_2_x = -l_4;
    float l_3_x = l_3;
    float l_4_x = l_3;
    float l_1_y = l_1;
    float l_2_y = -l_1;
    float l_3_y = -l_2;
    float l_4_y = l_2;
    float l_1_z = l_z;
    float l_2_z = l_z;
    float l_3_z = l_z;
    float l_4_z = l_z;

    //Assumptions:
    float I_xx_tilt, I_yy_tilt;
    I_xx_tilt = 0;
    I_yy_tilt = 0;
    float b_1_ddot, b_2_ddot, b_3_ddot, b_4_ddot;
    float g_1_ddot, g_2_ddot, g_3_ddot, g_4_ddot;
    b_1_ddot = 0;
    b_2_ddot = 0;
    b_3_ddot = 0;
    b_4_ddot = 0;
    g_1_ddot = 0;
    g_2_ddot = 0;
    g_3_ddot = 0;
    g_4_ddot = 0;

    // First row
    B_matrix[0][0] = 0;
    B_matrix[0][1] = 0;
    B_matrix[0][2] = 0;
    B_matrix[0][3] = 0;

    B_matrix[0][4] = (K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix[0][8] = 0;
    B_matrix[0][9] = 0;
    B_matrix[0][10] = 0;
    B_matrix[0][11] = 0;

    // Second row
    B_matrix[1][0] = 0;
    B_matrix[1][1] = 0;
    B_matrix[1][2] = 0;
    B_matrix[1][3] = 0;

    B_matrix[1][4] = 0;
    B_matrix[1][5] = 0;
    B_matrix[1][6] = 0;
    B_matrix[1][7] = 0;

    B_matrix[1][8] = (K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;

    // Third row
    B_matrix[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
    B_matrix[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
    B_matrix[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
    B_matrix[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;

    B_matrix[2][4] = 0;
    B_matrix[2][5] = 0;
    B_matrix[2][6] = 0;
    B_matrix[2][7] = 0;

    B_matrix[2][8] = 0;
    B_matrix[2][9] = 0;
    B_matrix[2][10] = 0;
    B_matrix[2][11] = 0;

    // Fourth row
    B_matrix[3][0] = (2*K_p_M*Omega_1*sin(b_1) + J_r*b_1_dot*cos(b_1) + J_r*q*cos(b_1)*cos(g_1) + J_r*r*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
    B_matrix[3][1] = -(2*K_p_M*Omega_2*sin(b_2) + J_r*b_2_dot*cos(b_2) + J_r*q*cos(b_2)*cos(g_2) + J_r*r*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
    B_matrix[3][2] = (2*K_p_M*Omega_3*sin(b_3) + J_r*b_3_dot*cos(b_3) + J_r*q*cos(b_3)*cos(g_3) + J_r*r*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
    B_matrix[3][3] = -(2*K_p_M*Omega_4*sin(b_4) + J_r*b_4_dot*cos(b_4) + J_r*q*cos(b_4)*cos(g_4) + J_r*r*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;

    B_matrix[3][4] = 0;
    B_matrix[3][5] = 0;
    B_matrix[3][6] = 0;
    B_matrix[3][7] = 0;

    B_matrix[3][8] = 0;
    B_matrix[3][9] = 0;
    B_matrix[3][10] = 0;
    B_matrix[3][11] = 0;

    // Fifth row
    B_matrix[4][0] = -(J_r*g_1_dot*cos(g_1) - J_r*r*sin(b_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_M*Omega_1*cos(b_1)*sin(g_1) + J_r*p*cos(b_1)*cos(g_1) - J_r*b_1_dot*sin(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
    B_matrix[4][1] = (J_r*g_2_dot*cos(g_2) - J_r*r*sin(b_2) + 2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) + J_r*p*cos(b_2)*cos(g_2) - J_r*b_2_dot*sin(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
    B_matrix[4][2] = -(J_r*g_3_dot*cos(g_3) - J_r*r*sin(b_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_M*Omega_3*cos(b_3)*sin(g_3) + J_r*p*cos(b_3)*cos(g_3) - J_r*b_3_dot*sin(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
    B_matrix[4][3] = (J_r*g_4_dot*cos(g_4) - J_r*r*sin(b_4) + 2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) + J_r*p*cos(b_4)*cos(g_4) - J_r*b_4_dot*sin(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;

    B_matrix[4][4] = 0;
    B_matrix[4][5] = 0;
    B_matrix[4][6] = 0;
    B_matrix[4][7] = 0;

    B_matrix[4][8] = 0;
    B_matrix[4][9] = 0;
    B_matrix[4][10] = 0;
    B_matrix[4][11] = 0;

    // Sixth row
    B_matrix[5][0] = -(J_r*g_1_dot*sin(g_1) + J_r*q*sin(b_1) + 2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + J_r*b_1_dot*cos(g_1)*sin(b_1) + J_r*p*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
    B_matrix[5][1] = (J_r*g_2_dot*sin(g_2) + J_r*q*sin(b_2) - 2*K_p_T*Omega_2*l_2_y*sin(b_2) - 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + J_r*b_2_dot*cos(g_2)*sin(b_2) + J_r*p*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
    B_matrix[5][2] = -(J_r*g_3_dot*sin(g_3) + J_r*q*sin(b_3) + 2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + J_r*b_3_dot*cos(g_3)*sin(b_3) + J_r*p*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
    B_matrix[5][3] = (J_r*g_4_dot*sin(g_4) + J_r*q*sin(b_4) - 2*K_p_T*Omega_4*l_4_y*sin(b_4) - 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + J_r*b_4_dot*cos(g_4)*sin(b_4) + J_r*p*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;

//    B_matrix[5][4] = -(J_r*Omega_1*q*cos(b_1) + K_p_T*Omega_1^2*l_1_y*cos(b_1) + J_r*Omega_1_dot*cos(g_1)*sin(b_1) + I_xx_tilt*g_1_ddot*cos(b_1)*cos(g_1) + K_p_M*Omega_1^2*cos(g_1)*sin(b_1) - J_r*Omega_1*p*sin(b_1)*sin(g_1) - K_p_T*Omega_1^2*l_1_x*sin(b_1)*sin(g_1) + J_r*Omega_1*b_1_dot*cos(b_1)*cos(g_1))/I_zz;
//    B_matrix[5][5] = (J_r*Omega_2*q*cos(b_2) - K_p_T*Omega_2^2*l_2_y*cos(b_2) - J_r*Omega_2_dot*cos(g_2)*sin(b_2) - I_xx_tilt*g_2_ddot*cos(b_2)*cos(g_2) + K_p_M*Omega_2^2*cos(g_2)*sin(b_2) - J_r*Omega_2*p*sin(b_2)*sin(g_2) + K_p_T*Omega_2^2*l_2_x*sin(b_2)*sin(g_2) + J_r*Omega_2*b_2_dot*cos(b_2)*cos(g_2))/I_zz;
//    B_matrix[5][6] = -(J_r*Omega_3*q*cos(b_3) + K_p_T*Omega_3^2*l_3_y*cos(b_3) + J_r*Omega_3_dot*cos(g_3)*sin(b_3) + I_xx_tilt*g_3_ddot*cos(b_3)*cos(g_3) + K_p_M*Omega_3^2*cos(g_3)*sin(b_3) - J_r*Omega_3*p*sin(b_3)*sin(g_3) - K_p_T*Omega_3^2*l_3_x*sin(b_3)*sin(g_3) + J_r*Omega_3*b_3_dot*cos(b_3)*cos(g_3))/I_zz;
//    B_matrix[5][7] = (J_r*Omega_4*q*cos(b_4) - K_p_T*Omega_4^2*l_4_y*cos(b_4) - J_r*Omega_4_dot*cos(g_4)*sin(b_4) - I_xx_tilt*g_4_ddot*cos(b_4)*cos(g_4) + K_p_M*Omega_4^2*cos(g_4)*sin(b_4) - J_r*Omega_4*p*sin(b_4)*sin(g_4) + K_p_T*Omega_4^2*l_4_x*sin(b_4)*sin(g_4) + J_r*Omega_4*b_4_dot*cos(b_4)*cos(g_4))/I_zz;

    B_matrix[5][4] = 0;
    B_matrix[5][5] = 0;
    B_matrix[5][6] = 0;
    B_matrix[5][7] = 0;

    B_matrix[5][8] = -(J_r*Omega_1*g_1_dot*cos(g_1) - I_yy_tilt*b_1_ddot*cos(g_1) + J_r*Omega_1_dot*cos(b_1)*sin(g_1) - I_xx_tilt*g_1_ddot*sin(b_1)*sin(g_1) + K_p_M*Omega_1*Omega_1*cos(b_1)*sin(g_1) - J_r*Omega_1*b_1_dot*sin(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*cos(g_1) + J_r*Omega_1*p*cos(b_1)*cos(g_1))/I_zz;
    B_matrix[5][9] = (I_yy_tilt*b_2_ddot*cos(g_2) + J_r*Omega_2*g_2_dot*cos(g_2) - J_r*Omega_2_dot*cos(b_2)*sin(g_2) + I_xx_tilt*g_2_ddot*sin(b_2)*sin(g_2) + K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - J_r*Omega_2*b_2_dot*sin(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2) + J_r*Omega_2*p*cos(b_2)*cos(g_2))/I_zz;
    B_matrix[5][10] = -(J_r*Omega_3*g_3_dot*cos(g_3) - I_yy_tilt*b_3_ddot*cos(g_3) + J_r*Omega_3_dot*cos(b_3)*sin(g_3) - I_xx_tilt*g_3_ddot*sin(b_3)*sin(g_3) + K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) - J_r*Omega_3*b_3_dot*sin(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3) + J_r*Omega_3*p*cos(b_3)*cos(g_3))/I_zz;
    B_matrix[5][11] = (I_yy_tilt*b_4_ddot*cos(g_4) + J_r*Omega_4*g_4_dot*cos(g_4) - J_r*Omega_4_dot*cos(b_4)*sin(g_4) + I_xx_tilt*g_4_ddot*sin(b_4)*sin(g_4) + K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - J_r*Omega_4*b_4_dot*sin(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4) + J_r*Omega_4*p*cos(b_4)*cos(g_4))/I_zz;

//    //Print the B matrix
//    printf("B_matrix = \n");
//    int i = 0;
//    int j = 0;
//    for (i = 0; i < INDI_OUTPUTS ; i++) {
//        for (j = 0; j < INDI_NUM_ACT; j++) {
//            printf("%f, ", B_matrix[i][j]);
//        }
//        printf("\n");
//    }

}



static void send_overactuated_variables( struct transport_tx *trans , struct link_device * dev ) {

// Send telemetry message
    int16_t actuators[12];
    for (uint8_t i = 0; i < 12; i++)
    {
        if (i < 8){
            actuators[i] = (int16_t) overactuated_mixing.commands[i];
        }
        else {
            actuators[i] = (int16_t) motor_mixing.commands[i-8];
        }

    }
    float psi_deg = psi*180/3.14;

    pprz_msg_send_OVERACTUATED_VARIABLES(trans , dev , AC_ID , & wind_speed, & x, & y, & z, & u, & v, & w, & psi_deg, & desired_x_e, & desired_y_e, & desired_z_e, & yaw_cmd, & elevation_cmd, & azimuth_cmd, 12, actuators );
}


/**
 * Initialize the motor mixing and calculate the trim values
 */
void overactuated_mixing_init() {
    uint8_t i;
    phi = stateGetNedToBodyEulers_f()->phi;
    theta = stateGetNedToBodyEulers_f()->theta;
    psi = stateGetNedToBodyEulers_f()->psi;
    x = stateGetPositionNed_i()->x;
    y = stateGetPositionNed_i()->y;
    z = stateGetPositionNed_i()->z;
    u = stateGetSpeedNed_i()->x;
    v = stateGetSpeedNed_i()->y;
    w = stateGetSpeedNed_i()->z;
    x_stb = waypoint_get_y(WP_STDBY);
    y_stb = waypoint_get_x(WP_STDBY);
    z_stb = waypoint_get_alt(WP_STDBY);

    // Case of Manual direct mode
    if(radio_control.values[RADIO_MODE] < 500 && radio_control.values[RADIO_MODE] > -500)
    {
        // Go trough all the motors and calculate the trim value and set the initial command
        for (i = 0; i < N_ACT; i++) {
            overactuated_mixing.commands[i] = 0;
            if (i % 2 != 0)   //Odd value --> (azimuth angle)
            {
                overactuated_mixing.commands[i] = radio_control.values[RADIO_ROLL];
            } else           //Even value --> (elevation angle)
            {
                overactuated_mixing.commands[i] = radio_control.values[RADIO_PITCH];
            }
            BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
        }

        // Reset variables for the position hold to avoid bump.
        desired_x_e = x;
        desired_y_e = y;
    }

    // Case of Position hold mode
    if(radio_control.values[RADIO_MODE] > 500)
    {
        //Calculate the desired position considering the RC input
        if( abs(radio_control.values[RADIO_PITCH]) > Deadband_stick ){
            desired_x_e += radio_control.values[RADIO_PITCH]*Stick_gain_position*-0.002;
        }
        if( abs(radio_control.values[RADIO_ROLL]) > Deadband_stick ){
            desired_y_e += radio_control.values[RADIO_ROLL]*Stick_gain_position*0.002;
        }

//        //Calculate the desired position considering the STANDBY point
//        desired_x_e = x_stb*100; //Get the wp goal x-position in cm
//        desired_y_e = y_stb*100; //Get the wp goal y-position in cm
//        desired_z_e = z_stb*100; //Get the wp goal z-position in cm

        float longitudinal_cmd = cos(psi) * (desired_x_e - x) + sin(psi)*(desired_y_e - y);
        float longitudinal_speed = cos(psi) * u + sin(psi)*v;

        float lateral_cmd = cos(psi) * (desired_y_e - y) -sin(psi) * (desired_x_e - x);
        float lateral_speed = cos(psi) * v - sin(psi) * u;

//        // Compute derivative of this term considering 500 Hz as module frequency update
//        float lateral_cmd_der = (lateral_cmd - lateral_cmd_old)*500;
//        float longitudinal_cmd_der = (longitudinal_cmd - longitudinal_cmd_old)*500;
//        lateral_cmd_old = lateral_cmd;
//        longitudinal_cmd_old = longitudinal_cmd;

        //Applying PID to command to get the servo deflection values:
        float azimuth_cmd = 5*(P_az_gain*lateral_cmd - D_az_gain*lateral_speed);
        float elevation_cmd = 5*(P_el_gain*longitudinal_cmd - D_el_gain*longitudinal_speed);

        // Write values to servos
        for (i = 0; i < N_ACT; i++) {
            overactuated_mixing.commands[i] = 0;
            if (i % 2 != 0)   //Odd value --> (azimuth angle)
            {
                overactuated_mixing.commands[i] = azimuth_cmd;
            } else           //Even value --> (elevation angle)
            {
                overactuated_mixing.commands[i] = -elevation_cmd;
            }
            BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
        }
    }

    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_OVERACTUATED_VARIABLES , send_overactuated_variables );
}

/*
 * Run the overactuated mixing
 * This depends on the ROLL and PITCH command
 * It also depends on the throttle_curve.collective
 */
void overactuated_mixing_run(pprz_t in_cmd[])
{
    uint8_t i;
    phi = stateGetNedToBodyEulers_f()->phi;
    theta = stateGetNedToBodyEulers_f()->theta;
    psi = stateGetNedToBodyEulers_f()->psi;

    struct FloatRates *body_rates = stateGetBodyRates_f();
    /* Propagate the filter on the gyroscopes */
    float rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};
    for (i = 0; i < 3; i++) {
        update_butterworth_2_low_pass(&measurement_lowpass_filters[i], rate_vect[i]);
        update_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], rate_vect[i]);

        //Calculate the angular acceleration via finite difference
        angular_acceleration[i] = (measurement_lowpass_filters[i].o[0]
                                   - measurement_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;
    i = 0;

    ax = stateGetAccelNed_f()->x;
    ay = stateGetAccelNed_f()->y;
    az = stateGetAccelNed_f()->z;

    u = stateGetSpeedNed_i()->x;
    v = stateGetSpeedNed_i()->y;
    w = stateGetSpeedNed_i()->z;

    x = stateGetPositionNed_i()->x;
    y = stateGetPositionNed_i()->y;
    z = stateGetPositionNed_i()->z;

    x_stb = waypoint_get_y(WP_STDBY);
    y_stb = waypoint_get_x(WP_STDBY);
    z_stb = waypoint_get_alt(WP_STDBY);

    yaw_cmd = 0;
    elevation_cmd = 0;
    azimuth_cmd = 0;

//    #ifdef INDI_CONTROLLER_OVERACTUATED
    float command[INDI_NUM_ACT];
    //Append yaw command in every cases if the bool is active:
    if(manual_yaw_overactuated){
        yaw_cmd = radio_control.values[RADIO_YAW]*1.f;
    }
    else{
        yaw_cmd = in_cmd[COMMAND_YAW]*1.f;
    }

    // Case of Manual direct mode [FAILSAFE]
    if(radio_control.values[RADIO_MODE] < 500 && radio_control.values[RADIO_MODE] > -500)
    {
        //Compute cmd in manual mode:
        elevation_cmd = - radio_control.values[RADIO_PITCH];
        azimuth_cmd = radio_control.values[RADIO_ROLL];

        // Reset variables for the next mode to avoid bump.
        desired_x_e = x;
        desired_y_e = y;
    }

    // Case of INDI control mode
    if(radio_control.values[RADIO_MODE] > 500)
    {
        float Desired_acceleration[INDI_OUTPUTS] = { 1, 0, 0, 0, 0, 0};

        float B_matrix_in[INDI_OUTPUTS][INDI_NUM_ACT];

        int i, j;
        float * B_matrix_in_[INDI_OUTPUTS];
        for (i = 0; i < INDI_OUTPUTS; i++) {
            B_matrix_in_[i] = &B_matrix_in[i][0];
        }
        i = 0;
        float Omega_sens[4] = {787,787,787,787};
        float Omega_dot_sens[4] = {0,0,0,0};
        float b_sens[4] = {0,0,0,0};
        float b_dot_sens[4] = {0,0,0,0};
        float g_sens[4] = {0,0,0,0};
        float g_dot_sens[4] = {0,0,0,0};

        float Euler_rad[3] = {phi,theta,psi};
        float pqr_rad_s[3] = {p,q,r};



        Compute_B_matrix(B_matrix_in_,0.1,.15,.2,1.984e-5,0.185,0.185,0.36,0.29,0,2.3,0.91e-5,1.3e-7,&Omega_sens[0],&b_sens[0],&g_sens[0],&Omega_dot_sens[0],&b_dot_sens[0],&g_dot_sens[0],&Euler_rad[0],&pqr_rad_s[0]);



        float B_matrix[INDI_OUTPUTS][INDI_NUM_ACT];
        memcpy(B_matrix, B_matrix_in, INDI_OUTPUTS * INDI_NUM_ACT * sizeof(float));

        float B_matrix_transposed[INDI_NUM_ACT][INDI_OUTPUTS];
        float * B_matrix_[INDI_NUM_ACT];

        //Transpose matrix B_in

        for (i = 0; i < INDI_OUTPUTS; i++) {
            for (j = 0; j < INDI_NUM_ACT; j++) {
                B_matrix_transposed[j][i] = B_matrix[i][j];
                B_matrix_[j] = &B_matrix_transposed[j][0];
            }
        }
        i = 0;
        j = 0;

        float w_in[INDI_OUTPUTS];

        float v_in[INDI_OUTPUTS][INDI_OUTPUTS];
        float * v_in_[INDI_OUTPUTS];
        for (i = 0; i < INDI_OUTPUTS; i++) {
            v_in_[i] = &v_in[i][0];
        }
        i = 0;

        pprz_svd_float(B_matrix_, &w_in[0], v_in_, INDI_NUM_ACT, INDI_OUTPUTS);

        //Transpose matrix U
        float U_transposed[INDI_OUTPUTS][INDI_NUM_ACT];
        for (i = 0; i < INDI_NUM_ACT; i++) {
            for (j = 0; j < INDI_OUTPUTS; j++) {
                U_transposed[j][i] = B_matrix_[i][j];
            }
        }
        i = 0;
        j = 0;

        // Invert the diag values
        float w_inverted[INDI_OUTPUTS][INDI_OUTPUTS];
        for (i = 0; i < INDI_OUTPUTS; i++) {
            w_inverted[i][i] = 1/w_in[i];
        }
        i = 0;

        //Multiply the diagonal matrix with U_transposed
        float out_1[INDI_OUTPUTS][INDI_NUM_ACT];
        int k;
        for (i = 0; i < INDI_OUTPUTS; i++) {
            for (j = 0; j < INDI_NUM_ACT; j++) {
                out_1[i][j] = 0.;
                for (k = 0; k < INDI_OUTPUTS; k++) {
                    out_1[i][j] += w_inverted[i][k] * U_transposed[k][j];
                }
            }
        }
        i = 0;
        j = 0;
        k = 0;

        //Multiply V with out_1
        float Pseudoinverse[INDI_OUTPUTS][INDI_NUM_ACT];
        for (i = 0; i < INDI_OUTPUTS; i++) {
            for (j = 0; j < INDI_NUM_ACT; j++) {
                Pseudoinverse[i][j] = 0.;
                for (k = 0; k < INDI_OUTPUTS; k++) {
                    Pseudoinverse[i][j] += v_in_[i][k] * out_1[k][j];
                }
            }
        }
        i = 0;
        j = 0;
        k = 0;

        //Get the final actuation command by multiplying desired acceleration with the out matrix

        for (j = 0; j < INDI_NUM_ACT; j++) {
            command[j] = 0.;
            for (k = 0; k < INDI_OUTPUTS; k++) {
                command[j] += Desired_acceleration[k] * Pseudoinverse[k][j];
            }
        }

        i = 0;
        j = 0;
        k = 0;
    }
    // Write values to servos
    for (i = 0; i < N_ACT; i++) {
        overactuated_mixing.commands[i] = command[i+4]*100;
        BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
    }
//    #endif

//
//    #ifndef INDI_CONTROLLER_OVERACTUATED
//    //Append yaw command in every cases if the bool is active:
//    if(manual_yaw_overactuated){
//        yaw_cmd = radio_control.values[RADIO_YAW]*1.f;
//    }
//    else{
//        yaw_cmd = in_cmd[COMMAND_YAW]*1.f;
//    }
//
//    // Case of Manual direct mode
//    if(radio_control.values[RADIO_MODE] < 500 && radio_control.values[RADIO_MODE] > -500)
//    {
//        //Compute cmd in manual mode:
//        elevation_cmd = - radio_control.values[RADIO_PITCH];
//        azimuth_cmd = radio_control.values[RADIO_ROLL];
//
//        // Reset variables for the next mode to avoid bump.
//        desired_x_e = x;
//        desired_y_e = y;
//    }
//
//    // Case of Position hold mode
//    if(radio_control.values[RADIO_MODE] > 500)
//    {
//        //Calculate the desired position considering the RC input
//        if( abs(radio_control.values[RADIO_PITCH]) > Deadband_stick ){
//            desired_x_e += radio_control.values[RADIO_PITCH]*Stick_gain_position*-0.002;
//        }
//        if( abs(radio_control.values[RADIO_ROLL]) > Deadband_stick ){
//            desired_y_e += radio_control.values[RADIO_ROLL]*Stick_gain_position*0.002;
//        }
//
////        //Calculate the desired position considering the STANDBY point
////        desired_x_e = x_stb*100; //Get the wp goal x-position in cm
////        desired_y_e = y_stb*100; //Get the wp goal y-position in cm
////        desired_z_e = z_stb*100; //Get the wp goal z-position in cm
//
//        float longitudinal_cmd = cos(psi) * (desired_x_e - x) + sin(psi)*(desired_y_e - y);
//        float longitudinal_speed = cos(psi) * u + sin(psi)*v;
//
//        float lateral_cmd = cos(psi) * (desired_y_e - y) -sin(psi) * (desired_x_e - x);
//        float lateral_speed = cos(psi) * v - sin(psi) * u;
//
//        // Applying PID to command to get the servo deflection values:
//        azimuth_cmd = 5*(P_az_gain*lateral_cmd - D_az_gain*lateral_speed);
//        elevation_cmd = 5*(P_el_gain*longitudinal_cmd - D_el_gain*longitudinal_speed);
//        if(manual_yaw_overactuated){
//            yaw_cmd = radio_control.values[RADIO_YAW]*1.f;
//        }
//        else{
//            yaw_cmd = in_cmd[COMMAND_YAW]*1.f;
//        }
//
//    }
//
//    // Write values to servos
//    for (i = 0; i < N_ACT; i++) {
//        overactuated_mixing.commands[i] = 0;
//        if (i % 2 != 0)   //Odd value --> (azimuth angle)
//        {
//            if(activate_lateral_over) {
//
//                overactuated_mixing.commands[i] = azimuth_cmd;
//            }
//            else{
//                overactuated_mixing.commands[i] = 0;
//            }
//        }
//        else           //Even value --> (elevation angle)
//        {
//            if(activate_longitudinal_over) {
//                overactuated_mixing.commands[i] = -elevation_cmd;
//            }
//            else{
//                overactuated_mixing.commands[i] = 0;
//            }
//        }
//
//        //Add eventual yaw order:
//        if (activate_yaw_over) {
//            if (i == 5 || i == 7) {
//                overactuated_mixing.commands[i] -= yaw_cmd;
//            }
//            if (i == 1 || i == 3) {
//                overactuated_mixing.commands[i] += yaw_cmd;
//            }
//        }
//
//        BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
//    }
//    #endif

}

