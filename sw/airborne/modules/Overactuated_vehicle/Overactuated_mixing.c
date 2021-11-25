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
#include "generated/airframe.h"
#include "Overactuated_mixing.h"
#include <math.h>
#include "subsystems/radio_control.h"
#include "state.h"
#include "paparazzi.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/navigation/waypoints.h"
#include "generated/flight_plan.h"
#include "subsystems/actuators/motor_mixing.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.c"


/**
 * Variables declaration
 */

//Array which contains all the actuator values (sent to motor and servos)
struct overactuated_mixing_t overactuated_mixing;

//General state variables:
float rate_vect[3];
float rate_vect_filt[3];
float rate_vect_filt_dot[3];
float euler_vect[3];
float acc_vect[3];
float acc_vect_filt[3];
float speed_vect[3];
float pos_vect[3];
float actuator_state[N_ACT];
float actuator_state_filt[N_ACT];
float actuator_state_filt_dot[N_ACT];
float euler_error[3];
float euler_error_dot[3];
float euler_error_integrated[3];
float euler_error_old[3];
float angular_body_error[3];
float pos_error[3];
float pos_error_dot[3];
float pos_error_integrated[3];
float pos_error_old[3];
float pos_order_body[3];
float pos_order_earth[3];
float euler_order[3];
float psi_order_motor;
float rate_vect_filt_deg[3];
float rate_vect_filt_dot_deg[3];

float euler_error_dot_filt[3];
float pos_error_dot_filt[3];

//Flight states variables:
bool INDI_engaged = 0;
bool FAILSAFE_engaged = 0;
bool PID_engaged = 0;

//Variable to store the INDI cmd.
int32_t INDI_commands[12];

// PID and general settings from slider
int deadband_stick_yaw = 500;
int deadband_stick_throttle = 1500;
float stick_gain_yaw = 0.01; // Stick to yaw gain
float stick_gain_throttle = 0.03; //  Stick to throttle gain
bool activate_tilting_az = 0;
bool activate_tilting_el = 0;
bool yaw_with_tilting = 1;
bool yaw_with_motors = 0;
bool position_with_attitude = 0;
bool soft_PID = 1;
bool manual_motor_stick = 0;

bool static_tilt_motor_for_yaw = 0;
int static_tilt_angle = 0;

bool manual_heading = 0;
int manual_heading_value_rad = 0;

float wind_speed = 0;

float x_stb, y_stb, z_stb;

float alt_cmd = 0;
float pitch_cmd = 0;
float roll_cmd = 0;
float yaw_motor_cmd = 0;
float yaw_tilt_cmd = 0;
float elevation_cmd = 0;
float azimuth_cmd = 0;


// INDI VARIABLES
float indi_du[INDI_NUM_ACT];
float indi_u[INDI_NUM_ACT];
float R_matrix[3][3];

float pos_setpoint[3];
float speed_setpoint[3];
float euler_setpoint[3];
float rate_setpoint[3];
float acc_setpoint[6];
float INDI_acceleration_inputs[INDI_INPUTS];

// Try to make all the matrices global
float w_in[INDI_INPUTS];
float v_in[INDI_INPUTS][INDI_INPUTS];

float B_matrix_in[INDI_INPUTS][INDI_NUM_ACT];
float * B_matrix_in_[INDI_INPUTS];
float B_matrix_transposed[INDI_NUM_ACT][INDI_INPUTS];
float * B_matrix_[INDI_NUM_ACT];
float * v_in_[INDI_INPUTS];
float out_1_multiply[INDI_INPUTS][INDI_NUM_ACT];
float w_inverted[INDI_INPUTS][INDI_INPUTS];
float Pseudoinverse_over[INDI_INPUTS][INDI_NUM_ACT];
float U_transposed[INDI_INPUTS][INDI_NUM_ACT];

//Variables for failsafe mode
float P_az_gain = 12371;
float D_az_gain = 6075;
float P_el_gain = 12426;
float D_el_gain = 6144;

//Variables needed for the filters:
Butterworth2LowPass measurement_rates_filters[3]; //Filter of pqr
Butterworth2LowPass measurement_acc_filters[3];   //Filter of acceleration
Butterworth2LowPass actuator_state_filters[12];   //Filter of actuators

Butterworth2LowPass angular_error_dot_filters[3]; //Filter of angular error
Butterworth2LowPass position_error_dot_filters[3];//Filter of position error

struct PID_over pid_gains_over = {
        .p = { OVERACTUATED_MIXING_PID_P_GAIN_PHI,
               OVERACTUATED_MIXING_PID_P_GAIN_THETA,
               OVERACTUATED_MIXING_PID_P_GAIN_PSI_AZ, //It natively uses the azimuth, specific pid for motor are defined below
               OVERACTUATED_MIXING_PID_P_GAIN_POS_X_TILT,
               OVERACTUATED_MIXING_PID_P_GAIN_POS_Y_TILT,
               OVERACTUATED_MIXING_PID_P_GAIN_POS_Z
        },
        .i = { OVERACTUATED_MIXING_PID_I_GAIN_PHI,
               OVERACTUATED_MIXING_PID_I_GAIN_THETA,
               OVERACTUATED_MIXING_PID_I_GAIN_PSI_AZ,
               OVERACTUATED_MIXING_PID_I_GAIN_POS_X_TILT,
               OVERACTUATED_MIXING_PID_I_GAIN_POS_Y_TILT,
               OVERACTUATED_MIXING_PID_I_GAIN_POS_Z
        },
        .d = { OVERACTUATED_MIXING_PID_D_GAIN_PHI,
               OVERACTUATED_MIXING_PID_D_GAIN_THETA,
               OVERACTUATED_MIXING_PID_D_GAIN_PSI_AZ,
               OVERACTUATED_MIXING_PID_D_GAIN_POS_X_TILT,
               OVERACTUATED_MIXING_PID_D_GAIN_POS_Y_TILT,
               OVERACTUATED_MIXING_PID_D_GAIN_POS_Z
        }
};
struct PID_over_simple pid_gain_psi_motor = {
        .p = OVERACTUATED_MIXING_PID_P_GAIN_PSI_MOTOR,
        .i = OVERACTUATED_MIXING_PID_I_GAIN_PSI_MOTOR,
        .d = OVERACTUATED_MIXING_PID_D_GAIN_PSI_MOTOR
};

struct PID_over_simple pid_pos_x_att = {
        .p = OVERACTUATED_MIXING_PID_P_GAIN_POS_X_ATT,
        .i = OVERACTUATED_MIXING_PID_I_GAIN_POS_X_ATT,
        .d = OVERACTUATED_MIXING_PID_D_GAIN_POS_X_ATT
};

struct PID_over_simple pid_pos_y_att = {
        .p = OVERACTUATED_MIXING_PID_P_GAIN_POS_Y_ATT,
        .i = OVERACTUATED_MIXING_PID_I_GAIN_POS_Y_ATT,
        .d = OVERACTUATED_MIXING_PID_D_GAIN_POS_Y_ATT
};

struct PD_indi_over indi_gains_over = {
        .p = { OVERACTUATED_MIXING_INDI_REF_ERR_P,
               OVERACTUATED_MIXING_INDI_REF_ERR_Q,
               OVERACTUATED_MIXING_INDI_REF_ERR_R,
               OVERACTUATED_MIXING_INDI_REF_ERR_X,
               OVERACTUATED_MIXING_INDI_REF_ERR_Y,
               OVERACTUATED_MIXING_INDI_REF_ERR_Z
        },
        .d = { OVERACTUATED_MIXING_INDI_REF_RATE_P,
               OVERACTUATED_MIXING_INDI_REF_RATE_Q,
               OVERACTUATED_MIXING_INDI_REF_RATE_R,
               OVERACTUATED_MIXING_INDI_REF_RATE_X,
               OVERACTUATED_MIXING_INDI_REF_RATE_Y,
               OVERACTUATED_MIXING_INDI_REF_RATE_Z
        }
};

struct FloatEulersPosition max_value_error = {  OVERACTUATED_MIXING_MAX_PHI,
                                                OVERACTUATED_MIXING_MAX_THETA,
                                                OVERACTUATED_MIXING_MAX_PSI_ERR,
                                                OVERACTUATED_MIXING_MAX_X_ERR,
                                                OVERACTUATED_MIXING_MAX_Y_ERR,
                                                OVERACTUATED_MIXING_MAX_Z_ERR
};
struct ActuatorsStruct act_dyn_struct = {
        OVERACTUATED_MIXING_INDI_ACT_DYN_MOTOR,
        OVERACTUATED_MIXING_INDI_ACT_DYN_EL,
        OVERACTUATED_MIXING_INDI_ACT_DYN_AZ
};
// Variables needed for the actuators:
float act_dyn[INDI_NUM_ACT];

/**
 * Function which computes the actuator effectiveness of my system
 */
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

    B_matrix[0][8] = -(K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix[0][9] = -(K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix[0][10] = -(K_p_T*Omega_3*Omega_3*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix[0][11] = -(K_p_T*Omega_4*Omega_4*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;

    // Second row
    B_matrix[1][0] = 0;
    B_matrix[1][1] = 0;
    B_matrix[1][2] = 0;
    B_matrix[1][3] = 0;

    B_matrix[1][4] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][5] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_2*Omega_2*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][6] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_3*Omega_3*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][7] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_4*Omega_4*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

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

//void Compute_B_matrix(float **B_matrix, float I_xx,float I_yy,float I_zz,float J_r,float l_1,float l_2,float l_3,
//                      float l_4,float l_z,float m,float K_p_T,float K_p_M,float *Omega_sens,float *b_sens,
//                      float *g_sens,float *Omega_dot_sens,float *b_dot_sens,float *g_dot_sens, float *Euler_rad,
//                      float *pqr_rad_s){
//    //Transpose pointer to local variables:
//    float Omega_1, Omega_2, Omega_3, Omega_4;
//    Omega_1 = 800;
//    Omega_2 = 800;
//    Omega_3 = 800;
//    Omega_4 = 800;
//    float b_1, b_2, b_3, b_4, g_1, g_2, g_3, g_4;
//    b_1 = 0;
//    b_2 = 0;
//    b_3 = 0;
//    b_4 = 0;
//    g_1 = 0;
//    g_2 = 0;
//    g_3 = 0;
//    g_4 = 0;
//
//    float Omega_1_dot, Omega_2_dot, Omega_3_dot, Omega_4_dot;
//    Omega_1_dot = 0;
//    Omega_2_dot = 0;
//    Omega_3_dot = 0;
//    Omega_4_dot = 0;
//    float b_1_dot, b_2_dot, b_3_dot, b_4_dot, g_1_dot, g_2_dot, g_3_dot, g_4_dot;
//    b_1_dot = 0;
//    b_2_dot = 0;
//    b_3_dot = 0;
//    b_4_dot = 0;
//    g_1_dot = 0;
//    g_2_dot = 0;
//    g_3_dot = 0;
//    g_4_dot = 0;
//
//
//    float Phi, Theta, Psi, p, q, r;
//    Phi = 0;
//    Theta = 0;
//    Psi = M_PI/3;
//    p = 0;
//    q = 0;
//    r = 0;
//
//    //Motor disposition
//    float l_1_x = -l_4;
//    float l_2_x = -l_4;
//    float l_3_x = l_3;
//    float l_4_x = l_3;
//    float l_1_y = l_1;
//    float l_2_y = -l_1;
//    float l_3_y = -l_2;
//    float l_4_y = l_2;
//    float l_1_z = l_z;
//    float l_2_z = l_z;
//    float l_3_z = l_z;
//    float l_4_z = l_z;
//
//    //Assumptions:
//    float I_xx_tilt, I_yy_tilt;
//    I_xx_tilt = 0;
//    I_yy_tilt = 0;
//    float b_1_ddot, b_2_ddot, b_3_ddot, b_4_ddot;
//    float g_1_ddot, g_2_ddot, g_3_ddot, g_4_ddot;
//    b_1_ddot = 0;
//    b_2_ddot = 0;
//    b_3_ddot = 0;
//    b_4_ddot = 0;
//    g_1_ddot = 0;
//    g_2_ddot = 0;
//    g_3_ddot = 0;
//    g_4_ddot = 0;
//
//    // First row
//    B_matrix[0][0] = 0;
//    B_matrix[0][1] = 0;
//    B_matrix[0][2] = 0;
//    B_matrix[0][3] = 0;
//
//    B_matrix[0][4] = (K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
//    B_matrix[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
//    B_matrix[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
//    B_matrix[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
//
//    B_matrix[0][8] = -(K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
//    B_matrix[0][9] = -(K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
//    B_matrix[0][10] = -(K_p_T*Omega_3*Omega_3*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
//    B_matrix[0][11] = -(K_p_T*Omega_4*Omega_4*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
//
//    // Second row
//    B_matrix[1][0] = 0;
//    B_matrix[1][1] = 0;
//    B_matrix[1][2] = 0;
//    B_matrix[1][3] = 0;
//
//    B_matrix[1][4] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix[1][5] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_2*Omega_2*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix[1][6] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_3*Omega_3*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix[1][7] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_4*Omega_4*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
//
//    B_matrix[1][8] = (K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
//
//    // Third row
//    B_matrix[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
//    B_matrix[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
//    B_matrix[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
//    B_matrix[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;
//
//    B_matrix[2][4] = 0;
//    B_matrix[2][5] = 0;
//    B_matrix[2][6] = 0;
//    B_matrix[2][7] = 0;
//
//    B_matrix[2][8] = 0;
//    B_matrix[2][9] = 0;
//    B_matrix[2][10] = 0;
//    B_matrix[2][11] = 0;
//
//    // Fourth row
//    B_matrix[3][0] = (2*K_p_M*Omega_1*sin(b_1) + J_r*b_1_dot*cos(b_1) + J_r*q*cos(b_1)*cos(g_1) + J_r*r*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
//    B_matrix[3][1] = -(2*K_p_M*Omega_2*sin(b_2) + J_r*b_2_dot*cos(b_2) + J_r*q*cos(b_2)*cos(g_2) + J_r*r*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
//    B_matrix[3][2] = (2*K_p_M*Omega_3*sin(b_3) + J_r*b_3_dot*cos(b_3) + J_r*q*cos(b_3)*cos(g_3) + J_r*r*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
//    B_matrix[3][3] = -(2*K_p_M*Omega_4*sin(b_4) + J_r*b_4_dot*cos(b_4) + J_r*q*cos(b_4)*cos(g_4) + J_r*r*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;
//
//    B_matrix[3][4] = 0;
//    B_matrix[3][5] = 0;
//    B_matrix[3][6] = 0;
//    B_matrix[3][7] = 0;
//
//    B_matrix[3][8] = 0;
//    B_matrix[3][9] = 0;
//    B_matrix[3][10] = 0;
//    B_matrix[3][11] = 0;
//
//    // Fifth row
//    B_matrix[4][0] = -(J_r*g_1_dot*cos(g_1) - J_r*r*sin(b_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_M*Omega_1*cos(b_1)*sin(g_1) + J_r*p*cos(b_1)*cos(g_1) - J_r*b_1_dot*sin(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
//    B_matrix[4][1] = (J_r*g_2_dot*cos(g_2) - J_r*r*sin(b_2) + 2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) + J_r*p*cos(b_2)*cos(g_2) - J_r*b_2_dot*sin(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
//    B_matrix[4][2] = -(J_r*g_3_dot*cos(g_3) - J_r*r*sin(b_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_M*Omega_3*cos(b_3)*sin(g_3) + J_r*p*cos(b_3)*cos(g_3) - J_r*b_3_dot*sin(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
//    B_matrix[4][3] = (J_r*g_4_dot*cos(g_4) - J_r*r*sin(b_4) + 2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) + J_r*p*cos(b_4)*cos(g_4) - J_r*b_4_dot*sin(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;
//
//    B_matrix[4][4] = 0;
//    B_matrix[4][5] = 0;
//    B_matrix[4][6] = 0;
//    B_matrix[4][7] = 0;
//
//    B_matrix[4][8] = 0;
//    B_matrix[4][9] = 0;
//    B_matrix[4][10] = 0;
//    B_matrix[4][11] = 0;
//
//    // Sixth row
//    B_matrix[5][0] = -(J_r*g_1_dot*sin(g_1) + J_r*q*sin(b_1) + 2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + J_r*b_1_dot*cos(g_1)*sin(b_1) + J_r*p*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
//    B_matrix[5][1] = (J_r*g_2_dot*sin(g_2) + J_r*q*sin(b_2) - 2*K_p_T*Omega_2*l_2_y*sin(b_2) - 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + J_r*b_2_dot*cos(g_2)*sin(b_2) + J_r*p*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
//    B_matrix[5][2] = -(J_r*g_3_dot*sin(g_3) + J_r*q*sin(b_3) + 2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + J_r*b_3_dot*cos(g_3)*sin(b_3) + J_r*p*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
//    B_matrix[5][3] = (J_r*g_4_dot*sin(g_4) + J_r*q*sin(b_4) - 2*K_p_T*Omega_4*l_4_y*sin(b_4) - 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + J_r*b_4_dot*cos(g_4)*sin(b_4) + J_r*p*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;
//
////    B_matrix[5][4] = -(J_r*Omega_1*q*cos(b_1) + K_p_T*Omega_1^2*l_1_y*cos(b_1) + J_r*Omega_1_dot*cos(g_1)*sin(b_1) + I_xx_tilt*g_1_ddot*cos(b_1)*cos(g_1) + K_p_M*Omega_1^2*cos(g_1)*sin(b_1) - J_r*Omega_1*p*sin(b_1)*sin(g_1) - K_p_T*Omega_1^2*l_1_x*sin(b_1)*sin(g_1) + J_r*Omega_1*b_1_dot*cos(b_1)*cos(g_1))/I_zz;
////    B_matrix[5][5] = (J_r*Omega_2*q*cos(b_2) - K_p_T*Omega_2^2*l_2_y*cos(b_2) - J_r*Omega_2_dot*cos(g_2)*sin(b_2) - I_xx_tilt*g_2_ddot*cos(b_2)*cos(g_2) + K_p_M*Omega_2^2*cos(g_2)*sin(b_2) - J_r*Omega_2*p*sin(b_2)*sin(g_2) + K_p_T*Omega_2^2*l_2_x*sin(b_2)*sin(g_2) + J_r*Omega_2*b_2_dot*cos(b_2)*cos(g_2))/I_zz;
////    B_matrix[5][6] = -(J_r*Omega_3*q*cos(b_3) + K_p_T*Omega_3^2*l_3_y*cos(b_3) + J_r*Omega_3_dot*cos(g_3)*sin(b_3) + I_xx_tilt*g_3_ddot*cos(b_3)*cos(g_3) + K_p_M*Omega_3^2*cos(g_3)*sin(b_3) - J_r*Omega_3*p*sin(b_3)*sin(g_3) - K_p_T*Omega_3^2*l_3_x*sin(b_3)*sin(g_3) + J_r*Omega_3*b_3_dot*cos(b_3)*cos(g_3))/I_zz;
////    B_matrix[5][7] = (J_r*Omega_4*q*cos(b_4) - K_p_T*Omega_4^2*l_4_y*cos(b_4) - J_r*Omega_4_dot*cos(g_4)*sin(b_4) - I_xx_tilt*g_4_ddot*cos(b_4)*cos(g_4) + K_p_M*Omega_4^2*cos(g_4)*sin(b_4) - J_r*Omega_4*p*sin(b_4)*sin(g_4) + K_p_T*Omega_4^2*l_4_x*sin(b_4)*sin(g_4) + J_r*Omega_4*b_4_dot*cos(b_4)*cos(g_4))/I_zz;
//
//    B_matrix[5][4] = 0;
//    B_matrix[5][5] = 0;
//    B_matrix[5][6] = 0;
//    B_matrix[5][7] = 0;
//
//    B_matrix[5][8] = -(J_r*Omega_1*g_1_dot*cos(g_1) - I_yy_tilt*b_1_ddot*cos(g_1) + J_r*Omega_1_dot*cos(b_1)*sin(g_1) - I_xx_tilt*g_1_ddot*sin(b_1)*sin(g_1) + K_p_M*Omega_1*Omega_1*cos(b_1)*sin(g_1) - J_r*Omega_1*b_1_dot*sin(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*cos(g_1) + J_r*Omega_1*p*cos(b_1)*cos(g_1))/I_zz;
//    B_matrix[5][9] = (I_yy_tilt*b_2_ddot*cos(g_2) + J_r*Omega_2*g_2_dot*cos(g_2) - J_r*Omega_2_dot*cos(b_2)*sin(g_2) + I_xx_tilt*g_2_ddot*sin(b_2)*sin(g_2) + K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - J_r*Omega_2*b_2_dot*sin(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2) + J_r*Omega_2*p*cos(b_2)*cos(g_2))/I_zz;
//    B_matrix[5][10] = -(J_r*Omega_3*g_3_dot*cos(g_3) - I_yy_tilt*b_3_ddot*cos(g_3) + J_r*Omega_3_dot*cos(b_3)*sin(g_3) - I_xx_tilt*g_3_ddot*sin(b_3)*sin(g_3) + K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) - J_r*Omega_3*b_3_dot*sin(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3) + J_r*Omega_3*p*cos(b_3)*cos(g_3))/I_zz;
//    B_matrix[5][11] = (I_yy_tilt*b_4_ddot*cos(g_4) + J_r*Omega_4*g_4_dot*cos(g_4) - J_r*Omega_4_dot*cos(b_4)*sin(g_4) + I_xx_tilt*g_4_ddot*sin(b_4)*sin(g_4) + K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - J_r*Omega_4*b_4_dot*sin(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4) + J_r*Omega_4*p*cos(b_4)*cos(g_4))/I_zz;
//
//
////    //Print the B matrix
////    printf("B_matrix = \n");
////    int i = 0;
////    int j = 0;
////    for (i = 0; i < INDI_INPUTS ; i++) {
////        for (j = 0; j < INDI_NUM_ACT; j++) {
////            printf("%f, ", B_matrix[i][j]);
////        }
////        printf("\n");
////    }
//
//}

/**
 * Function for the message B_MATRIX
 */
static void send_B_matrix( struct transport_tx *trans , struct link_device * dev ) {

    pprz_msg_send_B_MATRIX_SEND(trans , dev , AC_ID ,
                           12, B_matrix_in[0],
                           12, B_matrix_in[1],
                           12, B_matrix_in[2],
                           12, B_matrix_in[3],
                           1, B_matrix_in[4],
                           1, B_matrix_transposed[9],
                           1, B_matrix_transposed[10],
                           1, B_matrix_transposed[11]);
}

/**
 * Function for the message INDI_CMD
 */
static void send_indi_cmd( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message
    float INDI_increment[12];
    for(uint8_t j = 0; j < 12; j++){
        if(j<4){
            INDI_increment[j] = indi_du[j];
        }
        else{
            INDI_increment[j] = indi_du[j];
        }
    }

    pprz_msg_send_INDI_CMD(trans , dev , AC_ID ,
                                         & INDI_acceleration_inputs[0],& INDI_acceleration_inputs[1],& INDI_acceleration_inputs[2],
                                         & INDI_acceleration_inputs[3],& INDI_acceleration_inputs[4],& INDI_acceleration_inputs[5],
                                         & INDI_increment[0],& INDI_increment[1],& INDI_increment[2],& INDI_increment[3],
                                         & INDI_increment[4],& INDI_increment[5],& INDI_increment[6],& INDI_increment[7],
                                         & INDI_increment[8],& INDI_increment[9],& INDI_increment[10],& INDI_increment[11]);
}

/**
 * Function for the message overactuated_variables
 */
static void send_overactuated_variables( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message
    float euler_deg[3], euler_des_deg[3];
    for(uint8_t i = 0 ; i < 3 ; i++){
        //Updated from filtered to unfiltered values the 21/11/2021.
        rate_vect_filt_deg[i] = rate_vect[i]*180/3.14;
        rate_vect_filt_dot_deg[i] = rate_vect[i]*180/3.14;
        euler_deg[i] = euler_vect[i]*180/3.14;
        euler_des_deg[i] = euler_setpoint[i]*180/3.14;
    }


    pprz_msg_send_OVERACTUATED_VARIABLES(trans , dev , AC_ID ,
                                         & wind_speed,
                                         & pos_vect[0], & pos_vect[1], & pos_vect[2],
                                         & speed_vect[0], & speed_vect[1], & speed_vect[2],
                                         & acc_vect_filt[0], & acc_vect_filt[1], & acc_vect_filt[2],
                                         & rate_vect_filt_dot_deg[0], & rate_vect_filt_dot_deg[1], & rate_vect_filt_dot_deg[2],
                                         & rate_vect_filt_deg[0], & rate_vect_filt_deg[1], & rate_vect_filt_deg[2],
                                         & euler_deg[0], & euler_deg[1], & euler_deg[2],
                                         & euler_des_deg[0], & euler_des_deg[1], & euler_des_deg[2],
                                         & pos_setpoint[0], & pos_setpoint[1], & pos_setpoint[2],
                                         & alt_cmd, & pitch_cmd, & roll_cmd, & yaw_motor_cmd, & yaw_tilt_cmd, & elevation_cmd, & azimuth_cmd);
}

/**
 * Function for the message ACTUATORS_OUTPUT
 */
static void send_actuator_variables( struct transport_tx *trans , struct link_device * dev ) {

    pprz_msg_send_ACTUATORS_OUTPUT(trans , dev , AC_ID ,
                                         & overactuated_mixing.commands[0],
                                         & overactuated_mixing.commands[1],
                                         & overactuated_mixing.commands[2],
                                         & overactuated_mixing.commands[3],
                                         & overactuated_mixing.commands[4],
                                         & overactuated_mixing.commands[5],
                                         & overactuated_mixing.commands[6],
                                         & overactuated_mixing.commands[7],
                                         & overactuated_mixing.commands[8],
                                         & overactuated_mixing.commands[9],
                                         & overactuated_mixing.commands[10],
                                         & overactuated_mixing.commands[11]);
}

/**
 * Function for the message PID_variables
 */
static void send_PID_variables( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message containing the PID values

    pprz_msg_send_GAINS_OVERACTUATED(trans , dev , AC_ID ,
                                         & pid_gains_over.p.phi, & pid_gains_over.i.phi, & pid_gains_over.d.phi,
                                         & pid_gains_over.p.theta, & pid_gains_over.i.theta, & pid_gains_over.d.theta,
                                         & pid_gains_over.p.psi, & pid_gains_over.i.psi, & pid_gains_over.d.psi,
                                         & pid_gain_psi_motor.p, & pid_gain_psi_motor.i, & pid_gain_psi_motor.d,
                                         & pid_gains_over.p.x, & pid_gains_over.i.x, & pid_gains_over.d.x,
                                         & pid_gains_over.p.y, & pid_gains_over.i.y, & pid_gains_over.d.y,
                                         & pid_gains_over.p.z, & pid_gains_over.i.z, & pid_gains_over.d.z,
                                         & indi_gains_over.p.phi, & indi_gains_over.d.phi,
                                         & indi_gains_over.p.theta, & indi_gains_over.d.theta,
                                         & indi_gains_over.p.psi, & indi_gains_over.d.psi,
                                         & indi_gains_over.p.x, & indi_gains_over.d.x,
                                         & indi_gains_over.p.y, & indi_gains_over.d.y,
                                         & indi_gains_over.p.z, & indi_gains_over.d.z);
}

/**
 * Initialize the filters
 */
void init_filters(void){
    float sample_time = 1.0 / PERIODIC_FREQUENCY;
    //Sensors cutoff frequency
    float tau_p = 1.0 / ( OVERACTUATED_MIXING_FILT_CUTOFF_P);
    float tau_q = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_Q);
    float tau_r = 1.0 / ( OVERACTUATED_MIXING_FILT_CUTOFF_R);
    float tau_acc_x = 1.0 / ( OVERACTUATED_MIXING_FILT_CUTOFF_ACC_X);
    float tau_acc_y = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ACC_Y);
    float tau_acc_z = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ACC_Z);
    float tau_el = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_EL);
    float tau_az = 1.0 / ( OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_AZ);
    float tau_motor = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_MOTOR);
    float tau_ang_err = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ANG_ERR);
    float tau_pos_err = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_POS_ERR);

    // Initialize filters for the actuators
    for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
        if(i < 4){ //Case motor filter
            init_butterworth_2_low_pass(&actuator_state_filters[i], tau_motor, sample_time, 0.0);
        }
        else if(i > 3 && i < 8){ //Case elevation servos filter
            init_butterworth_2_low_pass(&actuator_state_filters[i], tau_el, sample_time, 0.0);
        }
        else{   //Case azimuth servos filter
            init_butterworth_2_low_pass(&actuator_state_filters[i], tau_az, sample_time, 0.0);
        }
    }

    // Initialize filters for the rates and accelerations
    init_butterworth_2_low_pass(&measurement_rates_filters[0], tau_p, sample_time, 0.0);
    init_butterworth_2_low_pass(&measurement_rates_filters[1], tau_q, sample_time, 0.0);
    init_butterworth_2_low_pass(&measurement_rates_filters[2], tau_r, sample_time, 0.0);
    init_butterworth_2_low_pass(&measurement_acc_filters[0], tau_acc_x, sample_time, 0.0);
    init_butterworth_2_low_pass(&measurement_acc_filters[1], tau_acc_y, sample_time, 0.0);
    init_butterworth_2_low_pass(&measurement_acc_filters[2], tau_acc_z, sample_time, 0.0);

    // Initialize filters for the derivative of the position and angular error
    init_butterworth_2_low_pass(&angular_error_dot_filters[0], tau_ang_err, sample_time, 0.0);
    init_butterworth_2_low_pass(&angular_error_dot_filters[1], tau_ang_err, sample_time, 0.0);
    init_butterworth_2_low_pass(&angular_error_dot_filters[2], tau_ang_err, sample_time, 0.0);
    init_butterworth_2_low_pass(&position_error_dot_filters[0], tau_pos_err, sample_time, 0.0);
    init_butterworth_2_low_pass(&position_error_dot_filters[1], tau_pos_err, sample_time, 0.0);
    init_butterworth_2_low_pass(&position_error_dot_filters[2], tau_pos_err, sample_time, 0.0);
}

/**
 * Get actuator state based on first order dynamics
 */
void get_actuator_state(void)
{
    //actuator dynamics
    for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
        actuator_state[i] = actuator_state[i] + act_dyn[i] * (indi_u[i] - actuator_state[i]);
        //Motors
        if(i < 4){
            Bound(actuator_state[i],OVERACTUATED_MIXING_MOTOR_MIN_OMEGA,OVERACTUATED_MIXING_MOTOR_MAX_OMEGA);
        }
        // Elevator angles
        else if(i < 8){
//            Bound(actuator_state[i],OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE,OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE);
            Bound(actuator_state[i],-40 * M_PI/180, 25 * M_PI/180);
        }
        //Azimuth angles
        else {
            Bound(actuator_state[i],OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE,OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
        }
        //Propagate the actuator values into the filters and calculate the derivative
        update_butterworth_2_low_pass(&actuator_state_filters[i], actuator_state[i]);
        actuator_state_filt[i] = actuator_state_filters[i].o[0];
        actuator_state_filt_dot[i] = (actuator_state_filters[i].o[0]
                                      - actuator_state_filters[i].o[1]) * PERIODIC_FREQUENCY;
    }

}

/**
 * Initialize the overactuated mixing module
 */
void overactuated_mixing_init() {

    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_OVERACTUATED_VARIABLES , send_overactuated_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_GAINS_OVERACTUATED , send_PID_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_ACTUATORS_OUTPUT , send_actuator_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_INDI_CMD , send_indi_cmd );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_B_MATRIX_SEND , send_B_matrix );

    //Startup the init variables of the INDI
    init_filters();

    //Assign pointers:
    for (int i = 0; i < INDI_INPUTS; i++) {
        B_matrix_in_[i] = &B_matrix_in[i][0];
    }
    for (int j = 0; j < INDI_NUM_ACT; j++) {
        B_matrix_[j] = &B_matrix_transposed[j][0];
    }

}

/**
 * Set of commands acting on global variables to compute the pseudoinverse
 */
void compute_pseudoinverse(void){

    uint8_t i,j,k;

    //Transpose matrix B_in
    for (i = 0; i < INDI_INPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            B_matrix_transposed[j][i] = B_matrix_in[i][j];
        }
    }

    // Pre-assign the matrices for the SVD decomposition
    for (i = 0; i < INDI_INPUTS; i++) {
        v_in_[i] = &v_in[i][0];
    }

    //Decompose the B matrix with the SVD decomposition module
    pprz_svd_float(B_matrix_, &w_in[0], v_in_, INDI_NUM_ACT, INDI_INPUTS);

    //Transpose matrix U
    for (i = 0; i < INDI_NUM_ACT; i++) {
        for (j = 0; j < INDI_INPUTS; j++) {
            U_transposed[j][i] = B_matrix_[i][j];
        }
    }

    // Invert the diag values
    for (i = 0; i < INDI_INPUTS; i++) {
        w_inverted[i][i] = 1/w_in[i];
    }

    //Multiply the diagonal matrix with U_transposed
    for (i = 0; i < INDI_INPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            out_1_multiply[i][j] = 0.;
            for (k = 0; k < INDI_INPUTS; k++) {
                out_1_multiply[i][j] += w_inverted[i][k] * U_transposed[k][j];
            }
        }
    }

    //Multiply V with out_1
    for (i = 0; i < INDI_INPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            Pseudoinverse_over[i][j] = 0.;
            for (k = 0; k < INDI_INPUTS; k++) {
                Pseudoinverse_over[i][j] += v_in_[i][k] * out_1_multiply[k][j];
            }
        }
    }
}

/**
 * Run the overactuated mixing
 */
void overactuated_mixing_run(pprz_t in_cmd[], bool in_flight)
{
    //Assign variables
    uint8_t i, j, k;
    rate_vect[0] = stateGetBodyRates_f()->p;
    rate_vect[1] = stateGetBodyRates_f()->q;
    rate_vect[2] = stateGetBodyRates_f()->r;
    euler_vect[0] = stateGetNedToBodyEulers_f()->phi;
    euler_vect[1] = stateGetNedToBodyEulers_f()->theta;
    euler_vect[2] = stateGetNedToBodyEulers_f()->psi;
    acc_vect[0] = stateGetAccelNed_f()->x;
    acc_vect[1] = stateGetAccelNed_f()->y;
    acc_vect[2] = stateGetAccelNed_f()->z;
    speed_vect[0] = stateGetSpeedNed_f()->x;
    speed_vect[1] = stateGetSpeedNed_f()->y;
    speed_vect[2] = stateGetSpeedNed_f()->z;
    pos_vect[0] = stateGetPositionNed_f()->x;
    pos_vect[1] = stateGetPositionNed_f()->y;
    pos_vect[2] = stateGetPositionNed_f()->z;
    for ( i=0 ; i<4 ; i++){
        act_dyn[i] = act_dyn_struct.motor;
        act_dyn[i+4] = act_dyn_struct.elevation;
        act_dyn[i+8] = act_dyn_struct.azimuth;
    }


    /* Propagate the filter on the gyroscopes and accelerometers */
    for (i = 0; i < 3; i++) {
        update_butterworth_2_low_pass(&measurement_rates_filters[i], rate_vect[i]);
        update_butterworth_2_low_pass(&measurement_acc_filters[i], acc_vect[i]);

        //Calculate the angular acceleration via finite difference
        rate_vect_filt_dot[i] = (measurement_rates_filters[i].o[0]
                                 - measurement_rates_filters[i].o[1]) * PERIODIC_FREQUENCY;
        rate_vect_filt[i] = measurement_rates_filters[i].o[0];
        acc_vect_filt[i] = measurement_acc_filters[i].o[0];
    }

    //Computation of the matrix to pass from euler to body rates
    R_matrix[0][0]=1;
    R_matrix[1][0]=0;
    R_matrix[2][0]=-sin(euler_vect[1]);
    R_matrix[0][1]=0;
    R_matrix[1][1]=cos(euler_vect[0]);
    R_matrix[2][1]=sin(euler_vect[1]) * cos(euler_vect[1]);
    R_matrix[0][2]=0;
    R_matrix[1][2]=-sin(euler_vect[0]);
    R_matrix[2][2]=cos(euler_vect[0]) * cos(euler_vect[1]);

    x_stb = waypoint_get_y(WP_STDBY);
    y_stb = waypoint_get_x(WP_STDBY);
    z_stb = waypoint_get_alt(WP_STDBY);


    /// Case of Manual direct mode [FAILSAFE]
    if(radio_control.values[RADIO_MODE] < -500){

        //INIT AND BOOLEAN RESET
        if(FAILSAFE_engaged == 0){
            /*
            INIT CODE FOR THE FAILSAFE GOES HERE
             */
            FAILSAFE_engaged = 1;
            INDI_engaged = 0;
            PID_engaged = 0;
        }

//        //Compute cmd in manual mode:
        elevation_cmd = radio_control.values[RADIO_PITCH];
        azimuth_cmd = radio_control.values[RADIO_ROLL];
        yaw_tilt_cmd = in_cmd[COMMAND_YAW];
        // Compute cmd in auto mode

        // Calculate the desired position considering the STANDBY point
//        pos_setpoint[0] = x_stb; //Get the wp goal x-position in m
//        pos_setpoint[1] = y_stb; //Get the wp goal y-position in m
//
//        float longitudinal_cmd = cos(euler_vect[2]) * (pos_setpoint[0] - pos_vect[0]) + sin(euler_vect[2]) * (pos_setpoint[1] - pos_vect[1]);
//        float longitudinal_speed = cos(euler_vect[2]) * speed_vect[0] + sin(euler_vect[2]) * speed_vect[1];
//
//        float lateral_cmd = cos(euler_vect[2]) * (pos_setpoint[1] - pos_vect[1]) - sin(euler_vect[2]) * (pos_setpoint[0] - pos_vect[0]);
//        float lateral_speed = cos(euler_vect[2]) * speed_vect[1] - sin(euler_vect[2]) * speed_vect[0];
//
//        //Applying PID to command to get the servo deflection values:
//        azimuth_cmd = P_az_gain * lateral_cmd - D_az_gain * lateral_speed;
//        elevation_cmd = -1 * (P_el_gain * longitudinal_cmd - D_el_gain * longitudinal_speed);

        // Write values to servos and motors
        for (i = 0; i < N_ACT; i++) {
            if(i<4){ //MOTORS
                overactuated_mixing.commands[i] = motor_mixing.commands[i];
                Bound(overactuated_mixing.commands[i], 0, MAX_PPRZ);
            }
            else if(i>3 && i<8){ //ELEVATOR SERVOS
                overactuated_mixing.commands[i] = elevation_cmd;
                BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
            }
            else if(i>7 && i<10){ //AZIMUTH SERVOS 1,2
                overactuated_mixing.commands[i] = azimuth_cmd + yaw_tilt_cmd;
                BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
            }
            else {  //AZIMUTH SERVOS 3,4
                overactuated_mixing.commands[i] = azimuth_cmd - yaw_tilt_cmd;
                BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
            }
        }

    }

    /// Case of PID control as on simulink
    if(radio_control.values[RADIO_MODE] < 500 && radio_control.values[RADIO_MODE] > -500) {
        //INIT AND BOOLEAN RESET
        if (PID_engaged == 0) {
            /*
            INIT CODE FOR THE PID GOES HERE
             */

            //Reset the integrators:

            //Reset the status boolean in accordance with the actual flight state
            PID_engaged = 1;
            INDI_engaged = 0;
            FAILSAFE_engaged = 0;
        }

        ////Position error computation
//        //Compute the position setpoints through slider and RC stick:
//         if ( abs(radio_control.values[RADIO_PITCH]) > deadband_stick_throttle){
//             pos_setpoint[0] = pos_setpoint[0] + stick_gain_throttle * radio_control.values[RADIO_PITCH] * .0001;
//         }
//         if ( abs(radio_control.values[RADIO_ROLL]) > deadband_stick_throttle){
//             pos_setpoint[1] = pos_setpoint[1] + stick_gain_throttle * radio_control.values[RADIO_ROLL] * .0001;
//         }
//        if( abs(radio_control.values[RADIO_THROTTLE] - 4800) > deadband_stick_throttle && abs(pos_error[2]) < max_value_error.z ){
//            pos_setpoint[2]  = pos_setpoint[2]  + stick_gain_throttle * (radio_control.values[RADIO_THROTTLE] - 4800) * .00001;
//            Bound(pos_setpoint[2] ,-1,1000);
//        }
        //Get the position setpoints through STB point:
        pos_setpoint[0] = x_stb;
        pos_setpoint[1] = y_stb;
        pos_setpoint[2] = z_stb;

        //Calculate the error on the position
        pos_error[0] = pos_setpoint[0] - pos_vect[0];
        pos_error[1] = pos_setpoint[1] - pos_vect[1];
        pos_error[2] = pos_setpoint[2] + pos_vect[2];

        //Calculate the position error derivative and integration
        for (i = 0; i < 3; i++) {
            pos_error_dot[i] = (pos_error[i] - pos_error_old[i]) * PERIODIC_FREQUENCY;
            pos_error_old[i] = pos_error[i];
            pos_error_integrated[i] += pos_error[i] / PERIODIC_FREQUENCY;
        }

        /* Propagate the filter on the position error derivative therms */
        for (i = 0; i < 3; i++) {
            update_butterworth_2_low_pass(&position_error_dot_filters[i], pos_error_dot[i]);
            pos_error_dot_filt[i] = position_error_dot_filters[i].o[0];
        }

        //Now bound the error within the defined ranges:
        BoundAbs(pos_error[0], max_value_error.x);
        BoundAbs(pos_error[1], max_value_error.y);
        BoundAbs(pos_error[2], max_value_error.z);
        for (i = 0; i < 3; i++) {
            BoundAbs(pos_error_integrated[i], OVERACTUATED_MIXING_PID_MAX_POS_ERR_INTEGRATIVE);
            BoundAbs(pos_error_dot_filt[i], OVERACTUATED_MIXING_PID_MAX_SPEED_ORD);
        }

        //Calculate the orders with the PID gain defined:
        if(soft_PID){
            if(position_with_attitude){
                pos_order_earth[0] = pid_pos_x_att.p * pos_error[0] + pid_pos_x_att.i * pos_error_integrated[0] -
                                     pid_pos_x_att.d * speed_vect[0];
                pos_order_earth[1] = pid_pos_y_att.p * pos_error[1] + pid_pos_y_att.i * pos_error_integrated[1] -
                                     pid_pos_y_att.d * speed_vect[1];
            }
            else{
                pos_order_earth[0] = pid_gains_over.p.x * pos_error[0] + pid_gains_over.i.x * pos_error_integrated[0] -
                                     pid_gains_over.d.x * speed_vect[0];
                pos_order_earth[1] = pid_gains_over.p.y * pos_error[1] + pid_gains_over.i.y * pos_error_integrated[1] -
                                     pid_gains_over.d.y * speed_vect[1];
            }
            pos_order_earth[2] = pid_gains_over.p.z * pos_error[2] + pid_gains_over.i.z * pos_error_integrated[2] +
                                 pid_gains_over.d.z * speed_vect[2];
        }
        else{
            if(position_with_attitude){
                pos_order_earth[0] = pid_pos_x_att.p * pos_error[0] + pid_pos_x_att.i * pos_error_integrated[0] +
                                     pid_pos_x_att.d * pos_error_dot_filt[0];
                pos_order_earth[1] = pid_pos_y_att.p * pos_error[1] + pid_pos_y_att.i * pos_error_integrated[1] +
                                     pid_pos_y_att.d * pos_error_dot_filt[1];
            }
            else{
                pos_order_earth[0] = pid_gains_over.p.x * pos_error[0] + pid_gains_over.i.x * pos_error_integrated[0] -
                                     pid_gains_over.d.x * pos_error_dot_filt[0];
                pos_order_earth[1] = pid_gains_over.p.y * pos_error[1] + pid_gains_over.i.y * pos_error_integrated[1] +
                                     pid_gains_over.d.y * pos_error_dot_filt[1];
            }
            pos_order_earth[2] = pid_gains_over.p.z * pos_error[2] + pid_gains_over.i.z * pos_error_integrated[2] +
                                 pid_gains_over.d.z * pos_error_dot_filt[2];
        }


        //Transpose the position errors in the body frame:
        pos_order_body[0] = cos(euler_vect[2]) * pos_order_earth[0] + sin(euler_vect[2]) * pos_order_earth[1];
        pos_order_body[1] = cos(euler_vect[2]) * pos_order_earth[1] - sin(euler_vect[2]) * pos_order_earth[0];
        pos_order_body[2] = pos_order_earth[2];

        ////Angular error computation
        //Calculate the setpoints:
        if (position_with_attitude) {
            //The idea of the position body order is to have a radiant change of tilting, use the same idea with theta and psi
            euler_setpoint[0] = pos_order_body[1]; //A roll change is referred to a body y variation
            euler_setpoint[1] = -pos_order_body[0]; //A pitch change is referred to a body x variation
        } else {
            euler_setpoint[0] = max_value_error.phi * radio_control.values[RADIO_ROLL] / 9600;
            euler_setpoint[1] = max_value_error.theta * radio_control.values[RADIO_PITCH] / 9600;
//        euler_setpoint[0] = 0;
//        euler_setpoint[1] = 0;
        }
        //Integrate the stick yaw position to get the psi set point including psi saturation value
        if(manual_heading){
            euler_setpoint[2] = manual_heading_value_rad;
        }
        else if (abs(radio_control.values[RADIO_YAW]) > deadband_stick_yaw && abs(euler_error[2]) < max_value_error.psi) {
            euler_setpoint[2] =
                    euler_setpoint[2] + stick_gain_yaw * radio_control.values[RADIO_YAW] * M_PI / 180 * .001;
            //Correct the setpoint in order to always be within -pi and pi
            if (euler_setpoint[2] > M_PI) {
                euler_setpoint[2] -= 2 * M_PI;
            } else if (euler_setpoint[2] < -M_PI) {
                euler_setpoint[2] += 2 * M_PI;
            }
        }


        //Bound the setpoints within maximum angular values
        BoundAbs(euler_setpoint[0], max_value_error.phi);
        BoundAbs(euler_setpoint[1], max_value_error.theta);

        euler_error[0] = euler_setpoint[0] - euler_vect[0];
        euler_error[1] = euler_setpoint[1] - euler_vect[1];
        euler_error[2] = euler_setpoint[2] - euler_vect[2];

//        //Calculate the error on the euler angles
//        float euler_error_local[3];
//        euler_error_local[0] = euler_setpoint[0] - euler_vect[0];
//        euler_error_local[1] = euler_setpoint[1] - euler_vect[1];
//        euler_error_local[2] = euler_setpoint[2] - euler_vect[2];
//        //Transpose the error into the body frame:
//        for (j = 0; j < 3; j++) {
//            //Cleanup previous value
//            euler_error[j] = 0.;
//            for (k = 0; k < 3; k++) {
//                euler_error[j] += euler_error_local[k] * R_matrix[k][j];
//            }
//        }


        //Add logic for the psi control:
        if (euler_error[2] > M_PI) {
            euler_error[2] -= 2 * M_PI;
        } else if (euler_error[2] < -M_PI) {
            euler_error[2] += 2 * M_PI;
        }
        //Calculate the angular error derivative and integration term for the PID
        for (i = 0; i < 3; i++) {
            euler_error_dot[i] = (euler_error[i] - euler_error_old[i]) * PERIODIC_FREQUENCY;
            euler_error_old[i] = euler_error[i];
            euler_error_integrated[i] += euler_error[i] / PERIODIC_FREQUENCY;
        }
        // Propagate the filter on the angular error derivative terms
        for (i = 0; i < 3; i++) {
            update_butterworth_2_low_pass(&angular_error_dot_filters[i], euler_error_dot[i]);
            euler_error_dot_filt[i] = angular_error_dot_filters[i].o[0];
        }

        //Now bound the error within the defined ranges:
        BoundAbs(euler_error[2], max_value_error.psi);
        //Bound integrative terms
        for (i = 0; i < 3; i++) {
            BoundAbs(euler_error_integrated[i], OVERACTUATED_MIXING_PID_MAX_EULER_ERR_INTEGRATIVE);
        }

        psi_order_motor = 0;
        euler_order[2] = 0;
        if(soft_PID){
            euler_order[0] = pid_gains_over.p.phi * euler_error[0] + pid_gains_over.i.phi * euler_error_integrated[0] -
                             pid_gains_over.d.phi * rate_vect[0];
            euler_order[1] = pid_gains_over.p.theta * euler_error[1] + pid_gains_over.i.theta * euler_error_integrated[1] -
                             pid_gains_over.d.theta * rate_vect[1];
            //Compute the yaw order:
            if (yaw_with_tilting) {
                euler_order[2] = pid_gains_over.p.psi * euler_error[2] + pid_gains_over.i.psi * euler_error_integrated[2] -
                                 pid_gains_over.d.psi * rate_vect[2];
            }
            if (yaw_with_motors) {
                psi_order_motor = pid_gain_psi_motor.p * euler_error[2] + pid_gain_psi_motor.i * euler_error_integrated[2] -
                                  pid_gain_psi_motor.d * rate_vect[2];
            }
        }
        else{
            euler_order[0] = pid_gains_over.p.phi * euler_error[0] + pid_gains_over.i.phi * euler_error_integrated[0] +
                             pid_gains_over.d.phi * euler_error_dot_filt[0];
            euler_order[1] = pid_gains_over.p.theta * euler_error[1] + pid_gains_over.i.theta * euler_error_integrated[1] +
                             pid_gains_over.d.theta * euler_error_dot_filt[1];
            //Compute the yaw order:
            if (yaw_with_tilting) {
                euler_order[2] = pid_gains_over.p.psi * euler_error[2] + pid_gains_over.i.psi * euler_error_integrated[2] +
                                 pid_gains_over.d.psi * euler_error_dot_filt[2];
            }
            if (yaw_with_motors) {
                psi_order_motor = pid_gain_psi_motor.p * euler_error[2] + pid_gain_psi_motor.i * euler_error_integrated[2] +
                                  pid_gain_psi_motor.d * euler_error_dot_filt[2];
            }
        }

        //Bound euler angle orders:
        BoundAbs(euler_order[0], OVERACTUATED_MIXING_PID_MAX_ROLL_ORDER_PWM);
        BoundAbs(euler_order[1], OVERACTUATED_MIXING_PID_MAX_PITCH_ORDER_PWM);
        BoundAbs(euler_order[2], OVERACTUATED_MIXING_PID_MAX_YAW_ORDER_AZ);
        BoundAbs(psi_order_motor, OVERACTUATED_MIXING_PID_MAX_YAW_ORDER_MOTOR_PWM);


        //Submit motor orders:
        if(manual_motor_stick){
            //Motor 1:
            overactuated_mixing.commands[0] = (int32_t) (( euler_order[0] + euler_order[1] + psi_order_motor) ) * 9.6 + radio_control.values[RADIO_THROTTLE];
            //Motor 2:
            overactuated_mixing.commands[1] = (int32_t) ((-euler_order[0] + euler_order[1] - psi_order_motor) ) * 9.6 + radio_control.values[RADIO_THROTTLE];
            //Motor 3:
            overactuated_mixing.commands[2] = (int32_t) ((-euler_order[0] - euler_order[1] + psi_order_motor) ) * 9.6 + radio_control.values[RADIO_THROTTLE];
            //Motor 4:
            overactuated_mixing.commands[3] = (int32_t) (( euler_order[0] - euler_order[1] - psi_order_motor) ) * 9.6 + radio_control.values[RADIO_THROTTLE];
            //Compute the command for the PPZ message
            alt_cmd = radio_control.values[RADIO_THROTTLE];
        }
        else{
            //Protection control for altitude saturation on the pitch authority:
            //Compute the maximum allowable order considering as constraint the front motors:
            float max_alt_order = 1000 - Min(overactuated_mixing.commands[0]/9.6,overactuated_mixing.commands[1]/9.6);
            float order_alt_12 = Min(pos_order_body[2],max_alt_order);
            float order_alt_34 = 0;
            if(order_alt_12 >= 0){
                order_alt_34 = sqrt(order_alt_12 * order_alt_12 * VEHICLE_L4 / VEHICLE_L3);
            }
            else{
                order_alt_34 = -sqrt(order_alt_12 * order_alt_12 * VEHICLE_L3 / VEHICLE_L4);
            }
            //Compute the command for the PPZ message
            alt_cmd = ((order_alt_12 + order_alt_34)/2 + OVERACTUATED_MIXING_PID_THR_NEUTRAL_PWM - 1000) * 9.6;

            //Motor 1:
            overactuated_mixing.commands[0] =
                    (int32_t) (order_alt_12 + euler_order[0] + euler_order[1] + psi_order_motor + OVERACTUATED_MIXING_PID_THR_NEUTRAL_PWM - 1000) * 9.6;
            //Motor 2:
            overactuated_mixing.commands[1] =
                    (int32_t) (order_alt_12 - euler_order[0] + euler_order[1] - psi_order_motor + OVERACTUATED_MIXING_PID_THR_NEUTRAL_PWM - 1000) * 9.6;
            //Motor 3:
            overactuated_mixing.commands[2] =
                    (int32_t) (order_alt_34 - euler_order[0] - euler_order[1] + psi_order_motor + OVERACTUATED_MIXING_PID_THR_NEUTRAL_PWM - 1000) * 9.6;
            //Motor 4:
            overactuated_mixing.commands[3] =
                    (int32_t) (order_alt_34 + euler_order[0] - euler_order[1] - psi_order_motor + OVERACTUATED_MIXING_PID_THR_NEUTRAL_PWM - 1000) * 9.6;
        }

        //Bound the motor commands
        Bound(overactuated_mixing.commands[0], 0, 9600);
        Bound(overactuated_mixing.commands[1], 0, 9600);
        Bound(overactuated_mixing.commands[2], 0, 9600);
        Bound(overactuated_mixing.commands[3], 0, 9600);




        //Elevation servos:
        int32_t local_el_order = 0;

        if (activate_tilting_el) {
            if (pos_order_body[0] >= 0) {
                local_el_order = (int32_t)(pos_order_body[0] * 9600 / -OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE);
            } else {
                local_el_order = (int32_t)(pos_order_body[0] * 9600 / OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE);
            }
//            local_el_order = (int32_t)(- pos_order_body[0] * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE);
        }



        //Azimuth servos:
        int32_t local_az_order_12 = 0;
        int32_t local_az_order_34 = 0;

        if (activate_tilting_az) {
            if (pos_order_body[1] >= 0) {
                local_az_order_12 = (int32_t)(pos_order_body[1] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
                local_az_order_34 = (int32_t)(pos_order_body[1] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
            } else {
                local_az_order_12 = (int32_t)(pos_order_body[1] * 9600 / -OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE);
                local_az_order_34 = (int32_t)(pos_order_body[1] * 9600 / -OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE);
            }
//            local_az_order_12 = (int32_t)(pos_order_body[1] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
//            local_az_order_34 = (int32_t)(pos_order_body[1] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
        }



        if (yaw_with_tilting) {
            local_az_order_12 += euler_order[2] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE;
            local_az_order_34 -= euler_order[2] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE;
        }

        //Write the order on the message variables:
        roll_cmd = euler_order[0] * 9.6;
        pitch_cmd = euler_order[1] * 9.6;
        yaw_motor_cmd = psi_order_motor * 9.6f;
        yaw_tilt_cmd = euler_order[2] * 9600.0f / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE;
        elevation_cmd = local_el_order * 1.0f;
        azimuth_cmd = local_az_order_12 * 1.0f;


//        local_az_order_12 += OVERACTUATED_MIXING_NEUTRAL_AZ_ORDER + OVERACTUATED_MIXING_NEUTRAL_YAW_ORDER;
//        local_az_order_34 += OVERACTUATED_MIXING_NEUTRAL_AZ_ORDER - OVERACTUATED_MIXING_NEUTRAL_YAW_ORDER;
//        local_el_order += OVERACTUATED_MIXING_NEUTRAL_EL_ORDER;

        int local_el_order_1 = 0;
        int local_el_order_2 = 0;
        int local_el_order_3 = 0;
        int local_el_order_4 = 0;
        int local_az_order_1 = 0;
        int local_az_order_2 = 0;
        int local_az_order_3 = 0;
        int local_az_order_4 = 0;

        //Determination of the gamma angles for each rotor:
        float gamma_12 = atan(VEHICLE_L4/VEHICLE_L1);
        float gamma_34 = atan(VEHICLE_L3/VEHICLE_L2);

        if(static_tilt_motor_for_yaw && yaw_with_tilting == 0){
            local_el_order_1 = - static_tilt_angle * M_PI/180 * cos(gamma_12) * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE;
            local_az_order_1 = static_tilt_angle * M_PI/180 * sin(gamma_12) * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE;

            local_el_order_2 = - static_tilt_angle * M_PI/180 * cos(gamma_12) * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE;
            local_az_order_2 = - static_tilt_angle * M_PI/180 * sin(gamma_12) * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE;

            local_el_order_3 = - static_tilt_angle * M_PI/180 * cos(gamma_34) * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE;
            local_az_order_3 =  static_tilt_angle * M_PI/180 * sin(gamma_34) * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE;

            local_el_order_4 = - static_tilt_angle * M_PI/180 * cos(gamma_34) * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE;
            local_az_order_4 = - static_tilt_angle * M_PI/180 * sin(gamma_34) * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE;
        }

        local_el_order_1 += local_el_order + OVERACTUATED_MIXING_NEUTRAL_EL_ORDER_1;
        local_el_order_2 += local_el_order + OVERACTUATED_MIXING_NEUTRAL_EL_ORDER_2;
        local_el_order_3 += local_el_order + OVERACTUATED_MIXING_NEUTRAL_EL_ORDER_3;
        local_el_order_4 += local_el_order + OVERACTUATED_MIXING_NEUTRAL_EL_ORDER_4;

        local_az_order_1 += local_az_order_12 + OVERACTUATED_MIXING_NEUTRAL_AZ_ORDER + OVERACTUATED_MIXING_NEUTRAL_YAW_ORDER;
        local_az_order_2 += local_az_order_12 + OVERACTUATED_MIXING_NEUTRAL_AZ_ORDER + OVERACTUATED_MIXING_NEUTRAL_YAW_ORDER;
        local_az_order_3 += local_az_order_34 + OVERACTUATED_MIXING_NEUTRAL_AZ_ORDER - OVERACTUATED_MIXING_NEUTRAL_YAW_ORDER;
        local_az_order_4 += local_az_order_34 + OVERACTUATED_MIXING_NEUTRAL_AZ_ORDER - OVERACTUATED_MIXING_NEUTRAL_YAW_ORDER;

//        BoundAbs(local_az_order_12, MAX_PPRZ);
//        BoundAbs(local_az_order_34, MAX_PPRZ);
//        BoundAbs(local_el_order, MAX_PPRZ);

        BoundAbs(local_el_order_1, MAX_PPRZ);
        BoundAbs(local_el_order_2, MAX_PPRZ);
        BoundAbs(local_el_order_3, MAX_PPRZ);
        BoundAbs(local_el_order_4, MAX_PPRZ);
        BoundAbs(local_az_order_1, MAX_PPRZ);
        BoundAbs(local_az_order_2, MAX_PPRZ);
        BoundAbs(local_az_order_3, MAX_PPRZ);
        BoundAbs(local_az_order_4, MAX_PPRZ);


        //Submit tilting orders:
//        overactuated_mixing.commands[4] = local_el_order;
//        overactuated_mixing.commands[5] = local_el_order;
//        overactuated_mixing.commands[6] = local_el_order;
//        overactuated_mixing.commands[7] = local_el_order;

        overactuated_mixing.commands[4] = local_el_order_1;
        overactuated_mixing.commands[5] = local_el_order_2;
        overactuated_mixing.commands[6] = local_el_order_3;
        overactuated_mixing.commands[7] = local_el_order_4;
        overactuated_mixing.commands[8] = local_az_order_1;
        overactuated_mixing.commands[9] = local_az_order_2;
        overactuated_mixing.commands[10] = local_az_order_3;
        overactuated_mixing.commands[11] = local_az_order_4;
    }

    /// Case of INDI control mode as on simulink
    if(radio_control.values[RADIO_MODE] > 500)
    {
        //INIT AND BOOLEAN RESET
        if(INDI_engaged == 0){
            /*
            INIT CODE FOR THE INDI GOES HERE
             */
            euler_setpoint[2] = euler_vect[2];
            pos_setpoint[0] = pos_vect[0];
            pos_setpoint[1] = pos_vect[1];
            pos_setpoint[2] = pos_vect[2];
            init_filters();
            INDI_engaged = 1;
            PID_engaged = 0;
            FAILSAFE_engaged = 0;

            //Reset actuator states position:
            for(i = 0; i < 12; i++) {
                actuator_state[i] = 0;
                indi_u[i] = 0;
            }
        }

        // Get an estimate of the actuator state using the first order dynamics given by the user
        get_actuator_state();

        //Calculate the euler angle error to be fed into the PD for the INDI loop
//        euler_setpoint[0] = 0;
//        euler_setpoint[1] = 0;
        euler_setpoint[0] = max_value_error.phi * radio_control.values[RADIO_ROLL] / 9600;
        euler_setpoint[1] = max_value_error.theta * radio_control.values[RADIO_PITCH] / 9600;
        //Integrate the stick yaw position to get the psi set point
        if( abs(radio_control.values[RADIO_YAW]) > deadband_stick_yaw && abs(euler_error[2]) < max_value_error.psi){
            euler_setpoint[2] = euler_setpoint[2] + stick_gain_yaw * radio_control.values[RADIO_YAW] * M_PI / 180 * .001;
            //Correct the setpoint in order to always be within -pi and pi
            if(euler_setpoint[2] > M_PI){
                euler_setpoint[2] -= 2 * M_PI;
            }
            else if(euler_setpoint[2] < - M_PI){
                euler_setpoint[2] += 2 * M_PI;
            }
        }

        BoundAbs(euler_setpoint[0],max_value_error.phi);
        BoundAbs(euler_setpoint[1],max_value_error.theta);
        euler_error[0] = euler_setpoint[0] - euler_vect[0];
        euler_error[1] = euler_setpoint[1] - euler_vect[1];
        euler_error[2] = euler_setpoint[2] - euler_vect[2];
        //Add logic for the psi control:
        if(euler_error[2] > M_PI){
            euler_error[2] -= 2 * M_PI;
        }
        else if(euler_error[2] < -M_PI){
            euler_error[2] += 2 * M_PI;
        }

        //Link the euler error with the angular change in the body frame and calculate the rate setpoints
        for (j = 0; j < 3; j++) {
            //Cleanup previous value
            angular_body_error[j] = 0.;
            for (k = 0; k < 3; k++) {
                angular_body_error[j] += euler_error[k] * R_matrix[k][j];
            }
        }

        rate_setpoint[0] = angular_body_error[0] * indi_gains_over.p.phi;
        rate_setpoint[1] = angular_body_error[1] * indi_gains_over.p.theta;
        rate_setpoint[2] = angular_body_error[2] * indi_gains_over.p.psi;
        BoundAbs(rate_setpoint[0],OVERACTUATED_MIXING_INDI_MAX_P_ORD);
        BoundAbs(rate_setpoint[1],OVERACTUATED_MIXING_INDI_MAX_Q_ORD);
//        BoundAbs(rate_setpoint[2],OVERACTUATED_MIXING_INDI_MAX_R_ORD);

        //Compute the angular acceleration setpoint:
        acc_setpoint[3] = (rate_setpoint[0] - rate_vect[0]) * indi_gains_over.d.phi;
        acc_setpoint[4] = (rate_setpoint[1] - rate_vect[1]) * indi_gains_over.d.theta;
        acc_setpoint[5] = (rate_setpoint[2] - rate_vect[2]) * indi_gains_over.d.psi;
        BoundAbs(acc_setpoint[3],OVERACTUATED_MIXING_INDI_MAX_P_DOT_ORD * M_PI / 180);
        BoundAbs(acc_setpoint[4],OVERACTUATED_MIXING_INDI_MAX_Q_DOT_ORD * M_PI / 180);
        BoundAbs(acc_setpoint[5],OVERACTUATED_MIXING_INDI_MAX_R_DOT_ORD * M_PI / 180);

        //Compute the acceleration error and save it to the INDI input array in the right position:
        // ANGULAR ACCELERATION
        INDI_acceleration_inputs[3] = acc_setpoint[3] - rate_vect_filt_dot[0];
        INDI_acceleration_inputs[4] = acc_setpoint[4] - rate_vect_filt_dot[1];
        INDI_acceleration_inputs[5] = acc_setpoint[5] - rate_vect_filt_dot[2];

        //Calculate the position error to be fed into the PD for the INDI loop
        pos_setpoint[0] = x_stb;
        pos_setpoint[1] = y_stb;
//         if ( abs(radio_control.values[RADIO_PITCH]) > deadband_stick_throttle){
//             pos_setpoint[0] = pos_setpoint[0] + stick_gain_throttle * radio_control.values[RADIO_PITCH] * .0001;
//         }
//         if ( abs(radio_control.values[RADIO_ROLL]) > deadband_stick_throttle){
//             pos_setpoint[1] = pos_setpoint[1] + stick_gain_throttle * radio_control.values[RADIO_ROLL] * .0001;
//         }

//        pos_setpoint[2] = -z_stb;
        //Integrate the stick engine position to get the z set point WARNING NED CONVENTION
        if( abs(radio_control.values[RADIO_THROTTLE] - 4800) > deadband_stick_throttle ){
            pos_setpoint[2]  = pos_setpoint[2]  - stick_gain_throttle * (radio_control.values[RADIO_THROTTLE] - 4800) * .00001;
        }
        Bound(pos_setpoint[2] ,-1000,1);

        pos_error[0] = pos_setpoint[0] - pos_vect[0];
        pos_error[1] = pos_setpoint[1] - pos_vect[1];
        pos_error[2] = pos_setpoint[2] - pos_vect[2];
//        pos_error[0] = pos_setpoint[0];
//        pos_error[1] = pos_setpoint[1];
//        pos_error[2] = pos_setpoint[2];
//        BoundAbs(pos_error[0],max_value_error.x);
//        BoundAbs(pos_error[1],max_value_error.y);
//        BoundAbs(pos_error[2],max_value_error.z);

        //Compute the speed setpoint
        speed_setpoint[0] = pos_error[0] * indi_gains_over.p.x;
        speed_setpoint[1] = pos_error[1] * indi_gains_over.p.y;
        speed_setpoint[2] = pos_error[2] * indi_gains_over.p.z;
//        BoundAbs(speed_setpoint[0],OVERACTUATED_MIXING_INDI_MAX_VX_ORD);
//        BoundAbs(speed_setpoint[1],OVERACTUATED_MIXING_INDI_MAX_VY_ORD);
//        BoundAbs(speed_setpoint[2],OVERACTUATED_MIXING_INDI_MAX_VZ_ORD);

        //Compute the linear accel setpoint
        acc_setpoint[0] = (speed_setpoint[0] - speed_vect[0]) * indi_gains_over.d.x;
        acc_setpoint[1] = (speed_setpoint[1] - speed_vect[1]) * indi_gains_over.d.y;
        acc_setpoint[2] = (speed_setpoint[2] - speed_vect[2]) * indi_gains_over.d.z;
//        BoundAbs(acc_setpoint[0],OVERACTUATED_MIXING_INDI_MAX_AX_ORD);
//        BoundAbs(acc_setpoint[1],OVERACTUATED_MIXING_INDI_MAX_AY_ORD);
//        BoundAbs(acc_setpoint[2],OVERACTUATED_MIXING_INDI_MAX_AZ_ORD);

        //Compute the acceleration error and save it to the INDI input array in the right position:
        // LINEAR ACCELERATION
        INDI_acceleration_inputs[0] = acc_setpoint[0] - acc_vect_filt[0];
        INDI_acceleration_inputs[1] = acc_setpoint[1] - acc_vect_filt[1];
        INDI_acceleration_inputs[2] = acc_setpoint[2] - acc_vect_filt[2];



        // Compute the B matrix and invert it
        Compute_B_matrix(B_matrix_in_,VEHICLE_I_XX,VEHICLE_I_YY,VEHICLE_I_ZZ,VEHICLE_PROPELLER_INERTIA,VEHICLE_L1,
                         VEHICLE_L2,VEHICLE_L3,VEHICLE_L4,VEHICLE_LZ,VEHICLE_MASS,OVERACTUATED_MIXING_MOTOR_K_T_OMEGASQ,
                         OVERACTUATED_MIXING_MOTOR_K_M_OMEGASQ, &actuator_state_filt[0],&actuator_state_filt[4],
                         &actuator_state_filt[8], &actuator_state_filt_dot[0],&actuator_state_filt_dot[4],
                         &actuator_state_filt_dot[8],&euler_vect[0],&rate_vect_filt[0]);

//        float B_matrix[INDI_INPUTS][INDI_NUM_ACT] = {
//                {0, 0, 0, 0, -2.4525, -2.4525, -2.4525, -2.4525, 0, 0, 0, 0},
//                {0, 0, 0, 0, 0, 0, 0, 0, 2.4525, 2.4525, 2.4525, 2.4525},
//                {-0.0062, -0.0062, -0.0062, -0.0062, 0, 0, 0, 0, 0, 0, 0, 0},
//                {0.0265, -0.0265, -0.0265, 0.0265, 0, 0, 0, 0, 0, 0, 0, 0},
//                {0.0277, 0.0277, -0.0344, -0.0344, 0, 0, 0, 0, 0, 0, 0, 0},
//                {0.0010, -0.0010, 0.0010, -0.0010, 0, 0, 0, 0, 8.1791, 8.1791, -10.1533, -10.1533}};
//        float * B_matrix_in_[INDI_INPUTS];
//        for (i = 0; i < INDI_INPUTS; i++) {
//            B_matrix_in_[i] = &B_matrix[i][0];
//        }

//        INDI_acceleration_inputs[0] = 0;
//        INDI_acceleration_inputs[1] = 1;
//        INDI_acceleration_inputs[2] = 0;
//        INDI_acceleration_inputs[3] = 0;
//        INDI_acceleration_inputs[4] = 0;
//        INDI_acceleration_inputs[5] = 0;

        compute_pseudoinverse();

        //Compute the actuation increment command by multiplying desired acceleration with the pseudoinverse matrix
        for (j = 0; j < INDI_NUM_ACT; j++) {
            //Cleanup previous value
            indi_du[j] = 0.;
            for (k = 0; k < INDI_INPUTS; k++) {
                indi_du[j] += INDI_acceleration_inputs[k] * Pseudoinverse_over[k][j];
            }
        }

        //Compute the actuator value (incremental part of the INDI)
        for (i = 0; i < INDI_NUM_ACT; i++) {
            indi_u[i] = indi_du[i] + actuator_state_filt[i];
        }

        // Write values to servos and motor
        //Motors:
        overactuated_mixing.commands[0] = (int32_t) (indi_u[0] * 1/OVERACTUATED_MIXING_MOTOR_K_PWM_OMEGA * 9.6f);
        overactuated_mixing.commands[1] = (int32_t) (indi_u[1] * 1/OVERACTUATED_MIXING_MOTOR_K_PWM_OMEGA * 9.6f);
        overactuated_mixing.commands[2] = (int32_t) (indi_u[2] * 1/OVERACTUATED_MIXING_MOTOR_K_PWM_OMEGA * 9.6f);
        overactuated_mixing.commands[3] = (int32_t) (indi_u[3] * 1/OVERACTUATED_MIXING_MOTOR_K_PWM_OMEGA * 9.6f);

        //Elevator servos:
        if(activate_tilting_el) {
            overactuated_mixing.commands[4] = (int32_t) (indi_u[4] * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE + OVERACTUATED_MIXING_NEUTRAL_EL_ORDER_1);
            overactuated_mixing.commands[5] = (int32_t) (indi_u[5] * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE + OVERACTUATED_MIXING_NEUTRAL_EL_ORDER_2);
            overactuated_mixing.commands[6] = (int32_t) (indi_u[6] * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE + OVERACTUATED_MIXING_NEUTRAL_EL_ORDER_3);
            overactuated_mixing.commands[7] = (int32_t) (indi_u[7] * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE + OVERACTUATED_MIXING_NEUTRAL_EL_ORDER_4);
        }

        //Azimuth servos:
        if(activate_tilting_az) {
            overactuated_mixing.commands[8] = (int32_t) (indi_u[8] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
            overactuated_mixing.commands[9] = (int32_t) (indi_u[9] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
            overactuated_mixing.commands[10] = (int32_t) (indi_u[10] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
            overactuated_mixing.commands[11] = (int32_t) (indi_u[11] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
        }
        else if(yaw_with_tilting){
            overactuated_mixing.commands[8] = (int32_t) (in_cmd[COMMAND_YAW]);
            overactuated_mixing.commands[9] = (int32_t) (in_cmd[COMMAND_YAW]);
            overactuated_mixing.commands[10] = (int32_t) (- in_cmd[COMMAND_YAW]);
            overactuated_mixing.commands[11] = (int32_t) (- in_cmd[COMMAND_YAW]);
        }

        //Bound values:
        Bound(overactuated_mixing.commands[0], 0, MAX_PPRZ);
        Bound(overactuated_mixing.commands[1], 0, MAX_PPRZ);
        Bound(overactuated_mixing.commands[2], 0, MAX_PPRZ);
        Bound(overactuated_mixing.commands[3], 0, MAX_PPRZ);

        BoundAbs(overactuated_mixing.commands[4], MAX_PPRZ);
        BoundAbs(overactuated_mixing.commands[5], MAX_PPRZ);
        BoundAbs(overactuated_mixing.commands[6], MAX_PPRZ);
        BoundAbs(overactuated_mixing.commands[7], MAX_PPRZ);

        BoundAbs(overactuated_mixing.commands[8], MAX_PPRZ);
        BoundAbs(overactuated_mixing.commands[9], MAX_PPRZ);
        BoundAbs(overactuated_mixing.commands[10], MAX_PPRZ);
        BoundAbs(overactuated_mixing.commands[11], MAX_PPRZ);


//        for (i = 0; i < N_ACT; i++) {
//            overactuated_mixing.commands[i] = 0;
//            if(i < 4){ //Motors
//                //Transpose the motor command from SI to PPZ:
//                overactuated_mixing.commands[i] = (int32_t) (indi_u[i] * 1/OVERACTUATED_MIXING_MOTOR_K_PWM_OMEGA * 9.6f);
//            }
//            else if(i < 8){ //Elevator servos
//                if(activate_tilting_el) {
//                    overactuated_mixing.commands[i] = (int32_t)(
//                            indi_u[i] * 9600 / OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE);
//                }
//            }
//            else{ //Azimuth servos
//                if(activate_tilting_az) {
//                    overactuated_mixing.commands[i] = (int32_t)(
//                            indi_u[i] * 9600 / OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
//                }
//                else{
//                    if(i < 10){
//                        overactuated_mixing.commands[i] = in_cmd[COMMAND_YAW];
//                    }
//                    else{
//                        overactuated_mixing.commands[i] = - in_cmd[COMMAND_YAW];
//                    }
//                }
//            }
//            Bound(overactuated_mixing.commands[i],0, MAX_PPRZ);
//        }

    }

}

