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



//General state variables:
float rate_vect[3];
float euler_vect[3];
float acc_vect[3];
float speed_vect[3];
float pos_vect[3];

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

// INDI VARIABLES
bool INDI_engaged = 0;
float indi_du[]
//Variables needed for the filters:
Butterworth2LowPass measurement_rates_filters[3]; //Filter of pqr
Butterworth2LowPass measurement_acc_filters[3]; //Filter of acceleration
// Variables needed for the actuators:
float act_dyn[INDI_NUM_ACT] = { OVERACTUATED_MIXING_INDI_ACT_DYN_MOTOR, OVERACTUATED_MIXING_INDI_ACT_DYN_MOTOR,
                                OVERACTUATED_MIXING_INDI_ACT_DYN_MOTOR, OVERACTUATED_MIXING_INDI_ACT_DYN_MOTOR,
                                OVERACTUATED_MIXING_INDI_ACT_DYN_EL, OVERACTUATED_MIXING_INDI_ACT_DYN_EL,
                                OVERACTUATED_MIXING_INDI_ACT_DYN_EL, OVERACTUATED_MIXING_INDI_ACT_DYN_EL,
                                OVERACTUATED_MIXING_INDI_ACT_DYN_AZ, OVERACTUATED_MIXING_INDI_ACT_DYN_AZ,
                                OVERACTUATED_MIXING_INDI_ACT_DYN_AZ, OVERACTUATED_MIXING_INDI_ACT_DYN_AZ };

// variables needed for control
struct FloatVect3 pos_setpoint;
struct FloatEulers euler_setpoint;

static struct FirstOrderLowPass sensors_filt_out[12];


struct FloatVect3 body_accel_f;

void Compute_B_matrix(float **B_matrix, float I_xx,float I_yy,float I_zz,float J_r,float l_1,float l_2,float l_3,
                      float l_4,float l_z,float m,float K_p_T,float K_p_M,float *Omega_sens,float *b_sens,
                      float *g_sens,float *Omega_dot_sens,float *b_dot_sens,float *g_dot_sens, float *Euler_rad,
                      float *pqr_rad_s){
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
//    for (i = 0; i < INDI_INPUTS ; i++) {
//        for (j = 0; j < INDI_NUM_ACT; j++) {
//            printf("%f, ", B_matrix[i][j]);
//        }
//        printf("\n");
//    }

}

static void send_overactuated_variables( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message
    int16_t actuators[12];
    int16_t phi, theta, psi, p_dot, q_dot, r_dot, p, q, r, ax, ay, az, vx, vy, vz, x, y, z;
    //Euler angles [deg]
    phi = (int16_t) (euler_vect[0]*180/3.14);
    theta = (int16_t) (euler_vect[1]*180/3.14);
    psi = (int16_t) (euler_vect[2]*180/3.14);
    //Angular acceleration in body reference frame [deg/s^2]
    p_dot = (int16_t) (rate_dot_vect[0]*180/3.14);
    q_dot = (int16_t) (rate_dot_vect[1]*180/3.14);
    r_dot = (int16_t) (rate_dot_vect[2]*180/3.14);
    //Angular rates in body reference frame [deg/s]
    p = (int16_t) (rate_vect[0]*180/3.14);
    q = (int16_t) (rate_vect[1]*180/3.14);
    r = (int16_t) (rate_vect[2]*180/3.14);
    //Acceleration in earth reference frame [cm/s^2]
    ax = (int16_t) (acc_vect[0]*100);
    ay = (int16_t) (acc_vect[1]*100);
    az = (int16_t) (acc_vect[2]*100);
    //Speed in earth reference frame [cm/s]
    vx = (int16_t) (speed_vect[0]*100);
    vy = (int16_t) (speed_vect[1]*100);
    vz = (int16_t) (speed_vect[2]*100);
    //Position in earth reference frame [cm]
    x = (int16_t) (pos_vect[0]*100);
    y = (int16_t) (pos_vect[1]*100);
    z = (int16_t) (pos_vect[2]*100);

    for (uint8_t i = 0; i < 12; i++)
    {
        if (i < 8){
            actuators[i] = (int16_t) overactuated_mixing.commands[i];
        }
        else {
            actuators[i] = (int16_t) motor_mixing.commands[i-8];
        }
    }
    pprz_msg_send_OVERACTUATED_VARIABLES(trans , dev , AC_ID , & wind_speed, & x, & y, & z, & vx, & vy, & vz, & psi, & desired_x_e, & desired_y_e, & desired_z_e, & yaw_cmd, & elevation_cmd, & azimuth_cmd, 12, actuators );
}

/**
 * Initialize the filters
 */
void init_filters(){
    float sample_time = 1.0 / PERIODIC_FREQUENCY;
    //Sensors cutoff frequency
    float tau_p = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_P);
    float tau_q = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_Q);
    float tau_r = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_R);
    float tau_ax = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_ACC_X);
    float tau_ay = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_ACC_Y);
    float tau_az = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_ACC_Z);
    float tau_el = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_EL);
    float tau_az = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_AZ);
    float tau_motor = 1.0 / (2.0 * M_PI * OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_MOTOR);

    // Initialize filters for the actuators
    for (i = 0; i < INDI_NUM_ACT; i++) {
        if(i<4){ //Case motor filter
            init_butterworth_2_low_pass(&actuator_state_filters[i], tau_motor, sample_time, actuator_state_filters[i]);
        }
        else if(i>3 && i<8){ //Case elevation servos filter
            init_butterworth_2_low_pass(&actuator_state_filters[i], tau_el, sample_time, 0.0);
        }
        else{   //Case azimuth servos filter
            init_butterworth_2_low_pass(&actuator_state_filters[i], tau_az, sample_time, 0.0);
        }
    }

    // Initialize filters for the rates and accelerations
    init_butterworth_2_low_pass(&actuator_state_filters[i], tau, sample_time, 0.0);

}

void get_actuator_state(void)
{
    //actuator dynamics
    int8_t i;
    for (i = 0; i < INDI_NUM_ACT; i++) {
        actuator_state[i] = actuator_state[i] + act_dyn[i] * (indi_u[i] - actuator_state[i]);

        //Propagate the actuator values into the filters and calculate the derivative
        update_butterworth_2_low_pass(&actuator_state_filters[i], actuator_state[i]);
    }




}

void overactuated_indi_enter(void)
{
    /* reset setpoints in order to hover leveled at the actual psi value*/
    euler_setpoint.phi = 0;
    euler_setpoint.theta = 0;
    euler_setpoint.psi = stateGetNedToBodyEulers_f()->psi;

    pos_setpoint.x = stateGetPositionNed_f()->x;
    pos_setpoint.y = stateGetPositionNed_f()->y;
    pos_setpoint.z = stateGetPositionNed_f()->z;

    /* Init the filters: */
    init_filters();
}

/**
 * Initialize the overactuated mixing module
 */
void overactuated_mixing_init() {

    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_OVERACTUATED_VARIABLES , send_overactuated_variables );

    //Startup the init variables of the INDI
    overactuated_indi_enter();

}

/*
 * Run the overactuated mixing
 */
void overactuated_mixing_run(pprz_t in_cmd[], bool in_flight)
{
    uint8_t i, j, k;
    struct FloatEulers *euler_angles = stateGetNedToBodyEulers_f();
    struct FloatRates *body_rates = stateGetBodyRates_f();
    struct FloatVect3 *earth_accelerations = stateGetAccelNed_f();
    struct FloatVect3 *earth_speed = stateGetSpeedNed_f();
    struct FloatVect3 *earth_positions = stateGetPositionNed_f();
    rate_vect[0] = body_rates->p;
    rate_vect[1] = body_rates->q;
    rate_vect[2] = body_rates->r;
    euler_vect[0] = euler_angles->phi;
    euler_vect[1] = euler_angles->theta;
    euler_vect[2] = euler_angles->psi;
    acc_vect[0] = earth_accelerations->x;
    acc_vect[1] = earth_accelerations->y;
    acc_vect[2] = earth_accelerations->z;
    speed_vect[0] = earth_speed->x;
    speed_vect[1] = earth_speed->y;
    speed_vect[2] = earth_speed->z;
    pos_vect[0] = earth_positions->x;
    pos_vect[1] = earth_positions->y;
    pos_vect[2] = earth_positions->z;
    /* Propagate the filter on the gyroscopes and accelerometers */
    for (i = 0; i < 3; i++) {

        update_butterworth_2_low_pass(&measurement_rates_filters[i], rate_vect[i]);
        update_butterworth_2_low_pass(&measurement_acc_filters[i], acc_vect[i]);

        //Calculate the angular acceleration via finite difference
        rate_dot_vect[i] = (measurement_rates_filters[i].o[0]
                                   - measurement_rates_filters[i].o[1]) * PERIODIC_FREQUENCY;
    }

    x_stb = waypoint_get_y(WP_STDBY);
    y_stb = waypoint_get_x(WP_STDBY);
    z_stb = waypoint_get_alt(WP_STDBY);

    yaw_cmd = 0;
    elevation_cmd = 0;
    azimuth_cmd = 0;

    // Case of Manual direct mode [FAILSAFE]
    if(radio_control.values[RADIO_MODE] < 500 && radio_control.values[RADIO_MODE] > -500)
    {
        //Compute cmd in manual mode:
        elevation_cmd = - radio_control.values[RADIO_PITCH];
        azimuth_cmd = radio_control.values[RADIO_ROLL];
        yaw_cmd = radio_control.values[RADIO_YAW];


        // Also reset the status boolean, in order for the INDI to reset its states later
        INDI_engaged = 0;

        // Write values to servos
        for (i = 3; i < N_ACT; i++) {
            if(i<4){

            }
            overactuated_mixing.commands[i] = indi_du[i+4]*100;
            BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
        }
    }

    // Case of INDI control mode
    if(radio_control.values[RADIO_MODE] > 500)
    {
        if(INDI_engaged == 0){
            /*
            INIT CODE FOR THE INDI GOES HERE
             */
            overactuated_indi_enter();
            INDI_engaged = 1;
        }

        float Desired_acceleration[INDI_INPUTS] = { 1, 0, 0, 0, 0, 0};
        float B_matrix_in[INDI_INPUTS][INDI_NUM_ACT];

        float * B_matrix_in_[INDI_INPUTS];
        for (i = 0; i < INDI_INPUTS; i++) {
            B_matrix_in_[i] = &B_matrix_in[i][0];
        }
        float Omega_sens[4] = {787,787,787,787};
        float Omega_dot_sens[4] = {0,0,0,0};
        float b_sens[4] = {0,0,0,0};
        float b_dot_sens[4] = {0,0,0,0};
        float g_sens[4] = {0,0,0,0};
        float g_dot_sens[4] = {0,0,0,0};

        float pqr_rad_s[3] = {p,q,r};
        
        Compute_B_matrix(B_matrix_in_,0.1,.15,.2,1.984e-5,0.185,0.185,0.36,0.29,
                         0,2.3,0.91e-5,1.3e-7,&Omega_sens[0],&b_sens[0],
                         &g_sens[0],&Omega_dot_sens[0],&b_dot_sens[0],
                         &g_dot_sens[0],&euler_vect[0],&pqr_rad_s[0]);
        
        float B_matrix[INDI_INPUTS][INDI_NUM_ACT];
        memcpy(B_matrix, B_matrix_in, INDI_INPUTS * INDI_NUM_ACT * sizeof(float));

        float B_matrix_transposed[INDI_NUM_ACT][INDI_INPUTS];
        float * B_matrix_[INDI_NUM_ACT];

        //Transpose matrix B_in

        for (i = 0; i < INDI_INPUTS; i++) {
            for (j = 0; j < INDI_NUM_ACT; j++) {
                B_matrix_transposed[j][i] = B_matrix[i][j];
                B_matrix_[j] = &B_matrix_transposed[j][0];
            }
        }

        float w_in[INDI_INPUTS];

        float v_in[INDI_INPUTS][INDI_INPUTS];
        float * v_in_[INDI_INPUTS];
        for (i = 0; i < INDI_INPUTS; i++) {
            v_in_[i] = &v_in[i][0];
        }

        pprz_svd_float(B_matrix_, &w_in[0], v_in_, INDI_NUM_ACT, INDI_INPUTS);

        //Transpose matrix U
        float U_transposed[INDI_INPUTS][INDI_NUM_ACT];
        for (i = 0; i < INDI_NUM_ACT; i++) {
            for (j = 0; j < INDI_INPUTS; j++) {
                U_transposed[j][i] = B_matrix_[i][j];
            }
        }

        // Invert the diag values
        float w_inverted[INDI_INPUTS][INDI_INPUTS];
        for (i = 0; i < INDI_INPUTS; i++) {
            w_inverted[i][i] = 1/w_in[i];
        }

        //Multiply the diagonal matrix with U_transposed
        float out_1[INDI_INPUTS][INDI_NUM_ACT];

        for (i = 0; i < INDI_INPUTS; i++) {
            for (j = 0; j < INDI_NUM_ACT; j++) {
                out_1[i][j] = 0.;
                for (k = 0; k < INDI_INPUTS; k++) {
                    out_1[i][j] += w_inverted[i][k] * U_transposed[k][j];
                }
            }
        }

        //Multiply V with out_1
        float Pseudoinverse[INDI_INPUTS][INDI_NUM_ACT];
        for (i = 0; i < INDI_INPUTS; i++) {
            for (j = 0; j < INDI_NUM_ACT; j++) {
                Pseudoinverse[i][j] = 0.;
                for (k = 0; k < INDI_INPUTS; k++) {
                    Pseudoinverse[i][j] += v_in_[i][k] * out_1[k][j];
                }
            }
        }

        //Get the final actuation command by multiplying desired acceleration with the out matrix
        for (j = 0; j < INDI_NUM_ACT; j++) {
            //Cleanup previous value
            indi_du[j] = 0.;
            for (k = 0; k < INDI_INPUTS; k++) {
                indi_du[j] += Desired_acceleration[k] * Pseudoinverse[k][j];
            }
        }
        
        //Do the incremental part of the INDI (only if in flight) 
        if(in_flight) {
            for (i = 0; i < INDI_NUM_ACT; i++) {
                commands[i] = indi_du[i] + 
            }
        }
        else{
            for (i = 0; i < INDI_NUM_ACT; i++) {
                commands[i] = 0;
            }            
        }
        
        // Write values to servos and motor
        for (i = 0; i < N_ACT; i++) {
            overactuated_mixing.commands[i] = indi_du[i+4]*100;
            BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
        }        
    }
    
}

