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
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "paparazzi.h"
#include "modules/datalink/telemetry.h"
#include "modules/nav/waypoints.h"
#include "generated/flight_plan.h"
#include "modules/actuators/motor_mixing.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.c"
#include "modules/sensors/ca_am7.h"
#include "modules/core/abi.h"
//#include "wls/wls_alloc.h"

/**
 * Variables declaration
 */

//Communication with AM& variables
static abi_event AM7_in;
float extra_data_in_local[255];
struct am7_data_in myam7_data_in_local;

//Array which contains all the actuator values (sent to motor and servos)
struct overactuated_mixing_t overactuated_mixing;

//General state variables:
float rate_vect[3], rate_vect_filt[3], rate_vect_filt_dot[3], euler_vect[3], acc_vect[3], acc_vect_filt[3];
float speed_vect[3], pos_vect[3], airspeed = 0, flight_path_angle = 0, total_V = 0;
float actuator_state[N_ACT_REAL];
float actuator_state_filt[INDI_NUM_ACT];
float actuator_state_filt_dot[N_ACT_REAL];
float actuator_state_filt_ddot[N_ACT_REAL];
float actuator_state_filt_dot_old[N_ACT_REAL];
float euler_error[3];
float euler_error_integrated[3];
float angular_body_error[3];
float pos_error[3];
float pos_error_integrated[3];
float pos_order_body[3];
float pos_order_earth[3];
float euler_order[3];
float psi_order_motor;


//Flight states variables:
bool INDI_engaged = 0, FAILSAFE_engaged = 0, PID_engaged = 0;

// PID and general settings from slider
float pos_setpoint_body[2] = {0,0};
int deadband_stick_yaw = 500, deadband_stick_throttle = 2500;
float stick_gain_yaw = 0.01, stick_gain_throttle = 0.03; //  Stick to yaw and throttle gain (for the integral part)
bool yaw_with_motors_PID = 0, position_with_attitude = 0, manual_motor_stick = 1, activate_tilting_az_PID = 0;
bool activate_tilting_el_PID = 0, yaw_with_tilting_PID = 1, mode_1_control = 0;

bool manual_heading = 0;
int manual_heading_value_rad = 0;

float wind_speed = 0;
float x_stb, y_stb, z_stb;

float alt_cmd = 0, pitch_cmd = 0, roll_cmd = 0, yaw_motor_cmd = 0, yaw_tilt_cmd = 0, elevation_cmd = 0, azimuth_cmd = 0;

// Actuators variables:
float K_ppz_rads_motor = 9.6 / OVERACTUATED_MIXING_MOTOR_K_PWM_OMEGA;
float K_ppz_angle_el = (9600 * 2) / (OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE);
float K_ppz_angle_az = (9600 * 2) / (OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE);

// INDI VARIABLES
float prioritized_actuator_states[INDI_NUM_ACT] = {0, 0, 0, 0,
                                                   0, 0, 0, 0,
                                                   0, 0, 0, 0,
                                                   0, 0};

float indi_du[INDI_NUM_ACT], indi_u[INDI_NUM_ACT], indi_u_scaled[INDI_NUM_ACT];
float R_matrix[3][3];

//Setpoints and pseudocontrol
float pos_setpoint[3];
float speed_setpoint[3];
float euler_setpoint[3];
float rate_setpoint[3];
float acc_setpoint[6];
float INDI_acceleration_inputs[INDI_INPUTS];


float B_matrix[INDI_INPUTS][INDI_NUM_ACT];

// Optimization CA options
int max_iter = 6;
int N_iter = 1;

//RSPI specific global variables:
float sensibility_pseudo_control = 0.01;
float sensibility_locked_actuator = 0.01;
float airspeed_transition = 3;

//Variables needed for the filters:
Butterworth2LowPass measurement_rates_filters[3]; //Filter of pqr
Butterworth2LowPass measurement_acc_filters[3];   //Filter of acceleration
Butterworth2LowPass actuator_state_filters[N_ACT_REAL];   //Filter of actuators

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
float act_dyn[N_ACT_REAL];

static void data_AM7_abi_in(uint8_t sender_id __attribute__((unused)), float * myam7_data_in_ptr, float * extra_data_in_ptr){
    memcpy(&myam7_data_in_local,myam7_data_in_ptr,sizeof(struct am7_data_in));
    memcpy(&extra_data_in_local,extra_data_in_ptr,sizeof(extra_data_in_local));
}

// This function computes the full B matrix extended 6x14 in earth reference frame
void compute_B_matrix_extended_fcn_try(float * B_matrix, float I_xx, float I_yy, float I_zz, float m, float K_p_T, float K_p_M,
                                   float Phi, float Theta, float Psi, float l_1, float l_2, float l_3, float l_4, float l_z,
                                   float Cl_alpha, float Cm_alpha, float rho, float Cd_zero, float K_Cd, float S, float wing_chord,
                                   float V, float Omega_1, float Omega_2, float Omega_3, float Omega_4,
                                   float b_1, float b_2, float b_3, float b_4, float g_1, float g_2, float g_3, float g_4){

    float B_matrix_local[6][14];
    for(int i = 0; i < 14; i++){
        for(int j = 0; j < 6; j++){
            B_matrix_local[j][i] = 0;
        }
    }
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

    //First row
    B_matrix_local[0][0] = -(2*K_p_T*Omega_1*cos(Psi)*cos(Theta)*sin(b_1) + 2*K_p_T*Omega_1*cos(b_1)*cos(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][1] = -(2*K_p_T*Omega_2*cos(Psi)*cos(Theta)*sin(b_2) + 2*K_p_T*Omega_2*cos(b_2)*cos(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][2] = -(2*K_p_T*Omega_3*cos(Psi)*cos(Theta)*sin(b_3) + 2*K_p_T*Omega_3*cos(b_3)*cos(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][3] = -(2*K_p_T*Omega_4*cos(Psi)*cos(Theta)*sin(b_4) + 2*K_p_T*Omega_4*cos(b_4)*cos(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][4] = (K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][8] = -(K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][9] = -(K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][10] = -(K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][11] = -(K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;

//    B_matrix_local[0][12] = -(cos(Phi)*cos(Psi)*cos(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - cos(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Psi)*cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/2 - (S*V*V*rho*cos(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 + Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Psi)*cos(Theta) + (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Psi)*cos(Theta))/2)/m;
//    B_matrix_local[0][13] = -((Cl_alpha*S*Theta*rho*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*V*V)/2 + (cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;


    //Second row
    B_matrix_local[1][0] = (2*K_p_T*Omega_1*cos(b_1)*cos(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_1*cos(Theta)*sin(Psi)*sin(b_1) + 2*K_p_T*Omega_1*cos(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][1] = (2*K_p_T*Omega_2*cos(b_2)*cos(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_2*cos(Theta)*sin(Psi)*sin(b_2) + 2*K_p_T*Omega_2*cos(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][2] = (2*K_p_T*Omega_3*cos(b_3)*cos(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_3*cos(Theta)*sin(Psi)*sin(b_3) + 2*K_p_T*Omega_3*cos(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][3] = (2*K_p_T*Omega_4*cos(b_4)*cos(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_4*cos(Theta)*sin(Psi)*sin(b_4) + 2*K_p_T*Omega_4*cos(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][4] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][5] = -(K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Psi)*cos(b_2) + K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][6] = -(K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Psi)*cos(b_3) + K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][7] = -(K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Psi)*cos(b_4) + K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][8] = (K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;

//    B_matrix_local[1][12] = (sin(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*sin(Psi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/2 + (S*V*V*rho*sin(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Theta)*sin(Psi) - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi))/2)/m;
//    B_matrix_local[1][13] = ((Cl_alpha*S*Theta*rho*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*V*V)/2 + (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;


    //Third row
    B_matrix_local[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
    B_matrix_local[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
    B_matrix_local[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
    B_matrix_local[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;

//    B_matrix_local[2][4] = (K_p_T*Omega_1*Omega_1*cos(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*cos(Phi)*cos(Theta)*cos(g_1)*sin(b_1) - K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Phi)*sin(b_1)*sin(g_1))/m;
//    B_matrix_local[2][5] = (K_p_T*Omega_2*Omega_2*cos(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*cos(Phi)*cos(Theta)*cos(g_2)*sin(b_2) - K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Phi)*sin(b_2)*sin(g_2))/m;
//    B_matrix_local[2][6] = (K_p_T*Omega_3*Omega_3*cos(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*cos(Phi)*cos(Theta)*cos(g_3)*sin(b_3) - K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Phi)*sin(b_3)*sin(g_3))/m;
//    B_matrix_local[2][7] = (K_p_T*Omega_4*Omega_4*cos(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*cos(Phi)*cos(Theta)*cos(g_4)*sin(b_4) - K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Phi)*sin(b_4)*sin(g_4))/m;
//
//    B_matrix_local[2][8] = (K_p_T*Omega_1*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*cos(g_1))/m;
//    B_matrix_local[2][9] = (K_p_T*Omega_2*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*sin(g_2) + K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*cos(g_2))/m;
//    B_matrix_local[2][10] = (K_p_T*Omega_3*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*cos(g_3))/m;
//    B_matrix_local[2][11] = (K_p_T*Omega_4*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*sin(g_4) + K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*cos(g_4))/m;

//    B_matrix_local[2][12] = -(sin(Phi)*sin(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) - cos(Phi)*sin(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*cos(Phi)*cos(Theta))/2 + (S*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*sin(Theta))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix_local[2][13] = ((cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + (S*V*V*rho*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/2 + (Cl_alpha*S*Theta*V*V*rho*cos(Theta)*sin(Phi))/2)/m;


    //Fourth row
    B_matrix_local[3][0] = (2*K_p_M*Omega_1*sin(b_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
    B_matrix_local[3][1] = (2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_M*Omega_2*sin(b_2) + 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
    B_matrix_local[3][2] = (2*K_p_M*Omega_3*sin(b_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
    B_matrix_local[3][3] = (2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_M*Omega_4*sin(b_4) + 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;

    B_matrix_local[3][4] = -(K_p_T*Omega_1*Omega_1*l_1_y*cos(g_1)*sin(b_1) - K_p_M*Omega_1*Omega_1*cos(b_1) + K_p_T*Omega_1*Omega_1*l_1_z*sin(b_1)*sin(g_1))/I_xx;
    B_matrix_local[3][5] = -(K_p_M*Omega_2*Omega_2*cos(b_2) + K_p_T*Omega_2*Omega_2*l_2_y*cos(g_2)*sin(b_2) + K_p_T*Omega_2*Omega_2*l_2_z*sin(b_2)*sin(g_2))/I_xx;
    B_matrix_local[3][6] = -(K_p_T*Omega_3*Omega_3*l_3_y*cos(g_3)*sin(b_3) - K_p_M*Omega_3*Omega_3*cos(b_3) + K_p_T*Omega_3*Omega_3*l_3_z*sin(b_3)*sin(g_3))/I_xx;
    B_matrix_local[3][7] = -(K_p_M*Omega_4*Omega_4*cos(b_4) + K_p_T*Omega_4*Omega_4*l_4_y*cos(g_4)*sin(b_4) + K_p_T*Omega_4*Omega_4*l_4_z*sin(b_4)*sin(g_4))/I_xx;

    B_matrix_local[3][8] = (K_p_T*Omega_1*Omega_1*l_1_z*cos(b_1)*cos(g_1) - K_p_T*Omega_1*Omega_1*l_1_y*cos(b_1)*sin(g_1))/I_xx;
    B_matrix_local[3][9] = (K_p_T*Omega_2*Omega_2*l_2_z*cos(b_2)*cos(g_2) - K_p_T*Omega_2*Omega_2*l_2_y*cos(b_2)*sin(g_2))/I_xx;
    B_matrix_local[3][10] = (K_p_T*Omega_3*Omega_3*l_3_z*cos(b_3)*cos(g_3) - K_p_T*Omega_3*Omega_3*l_3_y*cos(b_3)*sin(g_3))/I_xx;
    B_matrix_local[3][11] = (K_p_T*Omega_4*Omega_4*l_4_z*cos(b_4)*cos(g_4) - K_p_T*Omega_4*Omega_4*l_4_y*cos(b_4)*sin(g_4))/I_xx;

//    B_matrix_local[3][12] = 0.f;
//    B_matrix_local[3][13] = 0.f;

    //Fifth row
    B_matrix_local[4][0] = -(2*K_p_M*Omega_1*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
    B_matrix_local[4][1] = (2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
    B_matrix_local[4][2] = -(2*K_p_M*Omega_3*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
    B_matrix_local[4][3] = (2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;

    B_matrix_local[4][4] = (K_p_M*Omega_1*Omega_1*sin(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_z*cos(b_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(g_1)*sin(b_1))/I_yy;
    B_matrix_local[4][5] = (K_p_T*Omega_2*Omega_2*l_2_z*cos(b_2) - K_p_M*Omega_2*Omega_2*sin(b_2)*sin(g_2) + K_p_T*Omega_2*Omega_2*l_2_x*cos(g_2)*sin(b_2))/I_yy;
    B_matrix_local[4][6] = (K_p_M*Omega_3*Omega_3*sin(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_z*cos(b_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(g_3)*sin(b_3))/I_yy;
    B_matrix_local[4][7] = (K_p_T*Omega_4*Omega_4*l_4_z*cos(b_4) - K_p_M*Omega_4*Omega_4*sin(b_4)*sin(g_4) + K_p_T*Omega_4*Omega_4*l_4_x*cos(g_4)*sin(b_4))/I_yy;

    B_matrix_local[4][8] = -(K_p_M*Omega_1*Omega_1*cos(b_1)*cos(g_1) - K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_yy;
    B_matrix_local[4][9] = (K_p_M*Omega_2*Omega_2*cos(b_2)*cos(g_2) + K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_yy;
    B_matrix_local[4][10] = -(K_p_M*Omega_3*Omega_3*cos(b_3)*cos(g_3) - K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_yy;
    B_matrix_local[4][11] = (K_p_M*Omega_4*Omega_4*cos(b_4)*cos(g_4) + K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_yy;

//    B_matrix_local[4][12] = (Cm_alpha * S * V*V* rho * wing_chord) / (2 * I_yy);
//    B_matrix_local[4][13] = 0.f;


    // Sixth row
    B_matrix_local[5][0] = -(2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][1] = -(2*K_p_T*Omega_2*l_2_y*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][2] = -(2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][3] = -(2*K_p_T*Omega_4*l_4_y*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][4] = -(K_p_T*Omega_1*Omega_1*l_1_y*cos(b_1) + K_p_M*Omega_1*Omega_1*cos(g_1)*sin(b_1) - K_p_T*Omega_1*Omega_1*l_1_x*sin(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][5] = (K_p_M*Omega_2*Omega_2*cos(g_2)*sin(b_2) - K_p_T*Omega_2*Omega_2*l_2_y*cos(b_2) + K_p_T*Omega_2*Omega_2*l_2_x*sin(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][6] = -(K_p_T*Omega_3*Omega_3*l_3_y*cos(b_3) + K_p_M*Omega_3*Omega_3*cos(g_3)*sin(b_3) - K_p_T*Omega_3*Omega_3*l_3_x*sin(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][7] = (K_p_M*Omega_4*Omega_4*cos(g_4)*sin(b_4) - K_p_T*Omega_4*Omega_4*l_4_y*cos(b_4) + K_p_T*Omega_4*Omega_4*l_4_x*sin(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][8] = -(K_p_M*Omega_1*Omega_1*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_zz;
    B_matrix_local[5][9] = (K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_zz;
    B_matrix_local[5][10] = -(K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_zz;
    B_matrix_local[5][11] = (K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_zz;

//    B_matrix_local[5][12] = 0.f;
//    B_matrix_local[5][13] = 0.f;

    memcpy(& B_matrix[0], & B_matrix_local[0], 14*6*sizeof(float) );
}

// This function computes the full B matrix extended 6x14 in earth reference frame
void compute_B_matrix_extended_fcn_with_tilt_inertia(float * B_matrix, float I_xx, float I_yy, float I_zz, float I_xx_tilt, float I_yy_tilt, float m, float K_p_T, float K_p_M,
                                       float Phi, float Theta, float Psi, float l_1, float l_2, float l_3, float l_4, float l_z,
                                       float Cl_alpha, float Cm_alpha, float rho, float Cd_zero, float K_Cd, float S, float wing_chord,
                                       float V, float Omega_1, float Omega_2, float Omega_3, float Omega_4,
                                       float b_1, float b_2, float b_3, float b_4, float g_1, float g_2, float g_3, float g_4,
                                       float b_1_ddot, float b_2_ddot, float b_3_ddot, float b_4_ddot,
                                       float g_1_ddot, float g_2_ddot, float g_3_ddot, float g_4_ddot){

    float B_matrix_local[6][14];
    for(int i = 0; i < 14; i++){
        for(int j = 0; j < 6; j++){
            B_matrix_local[j][i] = 0;
        }
    }
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

    //First row
    B_matrix_local[0][0] = -(2*K_p_T*Omega_1*cos(Psi)*cos(Theta)*sin(b_1) + 2*K_p_T*Omega_1*cos(b_1)*cos(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][1] = -(2*K_p_T*Omega_2*cos(Psi)*cos(Theta)*sin(b_2) + 2*K_p_T*Omega_2*cos(b_2)*cos(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][2] = -(2*K_p_T*Omega_3*cos(Psi)*cos(Theta)*sin(b_3) + 2*K_p_T*Omega_3*cos(b_3)*cos(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][3] = -(2*K_p_T*Omega_4*cos(Psi)*cos(Theta)*sin(b_4) + 2*K_p_T*Omega_4*cos(b_4)*cos(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][4] = (K_p_T*Omega_1*Omega_1 *cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1 *cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1 *sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][8] = -(K_p_T*Omega_1*Omega_1 *cos(b_1)*cos(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_1*Omega_1 *cos(b_1)*sin(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][9] = -(K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][10] = -(K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][11] = -(K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;

//    B_matrix_local[0][12] = -(cos(Phi)*cos(Psi)*cos(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - cos(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1  + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Psi)*cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/2 - (S*V*V*rho*cos(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 + Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Psi)*cos(Theta) + (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Psi)*cos(Theta))/2)/m;
//    B_matrix_local[0][13] = -((Cl_alpha*S*Theta*rho*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*V*V)/2 + (cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;

    //Second row
    B_matrix_local[1][0] = (2*K_p_T*Omega_1*cos(b_1)*cos(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_1*cos(Theta)*sin(Psi)*sin(b_1) + 2*K_p_T*Omega_1*cos(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][1] = (2*K_p_T*Omega_2*cos(b_2)*cos(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_2*cos(Theta)*sin(Psi)*sin(b_2) + 2*K_p_T*Omega_2*cos(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][2] = (2*K_p_T*Omega_3*cos(b_3)*cos(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_3*cos(Theta)*sin(Psi)*sin(b_3) + 2*K_p_T*Omega_3*cos(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][3] = (2*K_p_T*Omega_4*cos(b_4)*cos(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_4*cos(Theta)*sin(Psi)*sin(b_4) + 2*K_p_T*Omega_4*cos(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][4] = -(K_p_T*Omega_1*Omega_1 *cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_1*Omega_1 *cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1 *sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][5] = -(K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Psi)*cos(b_2) + K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][6] = -(K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Psi)*cos(b_3) + K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][7] = -(K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Psi)*cos(b_4) + K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][8] = (K_p_T*Omega_1*Omega_1 *cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1 *cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;

//    B_matrix_local[1][12] = (sin(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1  + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*sin(Psi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/2 + (S*V*V*rho*sin(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Theta)*sin(Psi) - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi))/2)/m;
//    B_matrix_local[1][13] = ((Cl_alpha*S*Theta*rho*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*V*V)/2 + (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;

    //Third row
    B_matrix_local[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
    B_matrix_local[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
    B_matrix_local[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
    B_matrix_local[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;

//    B_matrix_local[2][4] = (K_p_T*Omega_1*Omega_1 *cos(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1 *cos(Phi)*cos(Theta)*cos(g_1)*sin(b_1) - K_p_T*Omega_1*Omega_1 *cos(Theta)*sin(Phi)*sin(b_1)*sin(g_1))/m;
//    B_matrix_local[2][5] = (K_p_T*Omega_2*Omega_2*cos(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*cos(Phi)*cos(Theta)*cos(g_2)*sin(b_2) - K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Phi)*sin(b_2)*sin(g_2))/m;
//    B_matrix_local[2][6] = (K_p_T*Omega_3*Omega_3*cos(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*cos(Phi)*cos(Theta)*cos(g_3)*sin(b_3) - K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Phi)*sin(b_3)*sin(g_3))/m;
//    B_matrix_local[2][7] = (K_p_T*Omega_4*Omega_4*cos(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*cos(Phi)*cos(Theta)*cos(g_4)*sin(b_4) - K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Phi)*sin(b_4)*sin(g_4))/m;
//
//    B_matrix_local[2][8] = (K_p_T*Omega_1*Omega_1 *cos(Phi)*cos(Theta)*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1 *cos(Theta)*sin(Phi)*cos(b_1)*cos(g_1))/m;
//    B_matrix_local[2][9] = (K_p_T*Omega_2*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*sin(g_2) + K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*cos(g_2))/m;
//    B_matrix_local[2][10] = (K_p_T*Omega_3*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*cos(g_3))/m;
//    B_matrix_local[2][11] = (K_p_T*Omega_4*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*sin(g_4) + K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*cos(g_4))/m;

//    B_matrix_local[2][12] = -(sin(Phi)*sin(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) - cos(Phi)*sin(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*sin(b_1)*Omega_1*Omega_1  + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*cos(Phi)*cos(Theta))/2 + (S*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*sin(Theta))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix_local[2][13] = ((cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*sin(b_1)*Omega_1*Omega_1  + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1  + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + (S*V*V*rho*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/2 + (Cl_alpha*S*Theta*V*V*rho*cos(Theta)*sin(Phi))/2)/m;

    //Fourth row:
    B_matrix_local[3][0] = (2*K_p_M*Omega_1*sin(b_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
    B_matrix_local[3][1] = (2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_M*Omega_2*sin(b_2) + 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
    B_matrix_local[3][2] = (2*K_p_M*Omega_3*sin(b_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
    B_matrix_local[3][3] = (2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_M*Omega_4*sin(b_4) + 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;

    B_matrix_local[3][4] = -(I_xx_tilt*g_1_ddot*sin(b_1) - K_p_M*Omega_1*Omega_1 *cos(b_1) + K_p_T*Omega_1*Omega_1 *l_1_y*cos(g_1)*sin(b_1) + K_p_T*Omega_1*Omega_1 *l_1_z*sin(b_1)*sin(g_1))/I_xx;
    B_matrix_local[3][5] = -(K_p_M*Omega_2*Omega_2*cos(b_2) + I_xx_tilt*g_2_ddot*sin(b_2) + K_p_T*Omega_2*Omega_2*l_2_y*cos(g_2)*sin(b_2) + K_p_T*Omega_2*Omega_2*l_2_z*sin(b_2)*sin(g_2))/I_xx;
    B_matrix_local[3][6] = -(I_xx_tilt*g_3_ddot*sin(b_3) - K_p_M*Omega_3*Omega_3*cos(b_3) + K_p_T*Omega_3*Omega_3*l_3_y*cos(g_3)*sin(b_3) + K_p_T*Omega_3*Omega_3*l_3_z*sin(b_3)*sin(g_3))/I_xx;
    B_matrix_local[3][7] = -(K_p_M*Omega_4*Omega_4*cos(b_4) + I_xx_tilt*g_4_ddot*sin(b_4) + K_p_T*Omega_4*Omega_4*l_4_y*cos(g_4)*sin(b_4) + K_p_T*Omega_4*Omega_4*l_4_z*sin(b_4)*sin(g_4))/I_xx;

    B_matrix_local[3][8] = (K_p_T*Omega_1*Omega_1 *l_1_z*cos(b_1)*cos(g_1) - K_p_T*Omega_1*Omega_1 *l_1_y*cos(b_1)*sin(g_1))/I_xx;
    B_matrix_local[3][9] = (K_p_T*Omega_2*Omega_2*l_2_z*cos(b_2)*cos(g_2) - K_p_T*Omega_2*Omega_2*l_2_y*cos(b_2)*sin(g_2))/I_xx;
    B_matrix_local[3][10] = (K_p_T*Omega_3*Omega_3*l_3_z*cos(b_3)*cos(g_3) - K_p_T*Omega_3*Omega_3*l_3_y*cos(b_3)*sin(g_3))/I_xx;
    B_matrix_local[3][11] = (K_p_T*Omega_4*Omega_4*l_4_z*cos(b_4)*cos(g_4) - K_p_T*Omega_4*Omega_4*l_4_y*cos(b_4)*sin(g_4))/I_xx;

//    B_matrix_local[3][12] = 0;
//    B_matrix_local[3][13] = 0;

    //Fifth row:
    B_matrix_local[4][0] = -(2*K_p_M*Omega_1*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
    B_matrix_local[4][1] = (2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
    B_matrix_local[4][2] = -(2*K_p_M*Omega_3*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
    B_matrix_local[4][3] = (2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;

    B_matrix_local[4][4] = (K_p_M*Omega_1*Omega_1 *sin(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1 *l_1_z*cos(b_1) + I_xx_tilt*g_1_ddot*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1 *l_1_x*cos(g_1)*sin(b_1))/I_yy;
    B_matrix_local[4][5] = (K_p_T*Omega_2*Omega_2*l_2_z*cos(b_2) - K_p_M*Omega_2*Omega_2*sin(b_2)*sin(g_2) + I_xx_tilt*g_2_ddot*cos(b_2)*sin(g_2) + K_p_T*Omega_2*Omega_2*l_2_x*cos(g_2)*sin(b_2))/I_yy;
    B_matrix_local[4][6] = (K_p_M*Omega_3*Omega_3*sin(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_z*cos(b_3) + I_xx_tilt*g_3_ddot*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(g_3)*sin(b_3))/I_yy;
    B_matrix_local[4][7] = (K_p_T*Omega_4*Omega_4*l_4_z*cos(b_4) - K_p_M*Omega_4*Omega_4*sin(b_4)*sin(g_4) + I_xx_tilt*g_4_ddot*cos(b_4)*sin(g_4) + K_p_T*Omega_4*Omega_4*l_4_x*cos(g_4)*sin(b_4))/I_yy;

    B_matrix_local[4][8] = -(I_yy_tilt*b_1_ddot*sin(g_1) - I_xx_tilt*g_1_ddot*cos(g_1)*sin(b_1) + K_p_M*Omega_1*Omega_1 *cos(b_1)*cos(g_1) - K_p_T*Omega_1*Omega_1 *l_1_x*cos(b_1)*sin(g_1))/I_yy;
    B_matrix_local[4][9] = (I_xx_tilt*g_2_ddot*cos(g_2)*sin(b_2) - I_yy_tilt*b_2_ddot*sin(g_2) + K_p_M*Omega_2*Omega_2*cos(b_2)*cos(g_2) + K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_yy;
    B_matrix_local[4][10] = -(I_yy_tilt*b_3_ddot*sin(g_3) - I_xx_tilt*g_3_ddot*cos(g_3)*sin(b_3) + K_p_M*Omega_3*Omega_3*cos(b_3)*cos(g_3) - K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_yy;
    B_matrix_local[4][11] = (I_xx_tilt*g_4_ddot*cos(g_4)*sin(b_4) - I_yy_tilt*b_4_ddot*sin(g_4) + K_p_M*Omega_4*Omega_4*cos(b_4)*cos(g_4) + K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_yy;

//    B_matrix_local[4][12] = (Cm_alpha*S*V*V*rho*wing_chord)/(2*I_yy);
//    B_matrix_local[4][13] = 0;

    //Sixth row:
    B_matrix_local[5][0] = -(2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][1] = -(2*K_p_T*Omega_2*l_2_y*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][2] = -(2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][3] = -(2*K_p_T*Omega_4*l_4_y*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][4] = -(K_p_T*Omega_1*Omega_1*l_1_y*cos(b_1) + I_xx_tilt*g_1_ddot*cos(b_1)*cos(g_1) + K_p_M*Omega_1*Omega_1*cos(g_1)*sin(b_1) - K_p_T*Omega_1*Omega_1*l_1_x*sin(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][5] = -(K_p_T*Omega_2*Omega_2*l_2_y*cos(b_2) + I_xx_tilt*g_2_ddot*cos(b_2)*cos(g_2) - K_p_M*Omega_2*Omega_2*cos(g_2)*sin(b_2) - K_p_T*Omega_2*Omega_2*l_2_x*sin(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][6] = -(K_p_T*Omega_3*Omega_3*l_3_y*cos(b_3) + I_xx_tilt*g_3_ddot*cos(b_3)*cos(g_3) + K_p_M*Omega_3*Omega_3*cos(g_3)*sin(b_3) - K_p_T*Omega_3*Omega_3*l_3_x*sin(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][7] = -(K_p_T*Omega_4*Omega_4*l_4_y*cos(b_4) + I_xx_tilt*g_4_ddot*cos(b_4)*cos(g_4) - K_p_M*Omega_4*Omega_4*cos(g_4)*sin(b_4) - K_p_T*Omega_4*Omega_4*l_4_x*sin(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][8] = (I_yy_tilt*b_1_ddot*cos(g_1) + I_xx_tilt*g_1_ddot*sin(b_1)*sin(g_1) - K_p_M*Omega_1*Omega_1 *cos(b_1)*sin(g_1) - K_p_T*Omega_1*Omega_1 *l_1_x*cos(b_1)*cos(g_1))/I_zz;
    B_matrix_local[5][9] = (I_yy_tilt*b_2_ddot*cos(g_2) + I_xx_tilt*g_2_ddot*sin(b_2)*sin(g_2) + K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_zz;
    B_matrix_local[5][10] = (I_yy_tilt*b_3_ddot*cos(g_3) + I_xx_tilt*g_3_ddot*sin(b_3)*sin(g_3) - K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) - K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_zz;
    B_matrix_local[5][11] = (I_yy_tilt*b_4_ddot*cos(g_4) + I_xx_tilt*g_4_ddot*sin(b_4)*sin(g_4) + K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_zz;

//    B_matrix_local[5][12] = 0;
//    B_matrix_local[5][13] = 0;

    memcpy(& B_matrix[0], & B_matrix_local[0], 14*6*sizeof(float) );
}

// This function computes the full B matrix extended 6x14 in earth reference frame [upd 16-02-2022]
void compute_B_matrix_extended_fcn(float * B_matrix, float I_xx, float I_yy, float I_zz, float m, float K_p_T, float K_p_M,
                                   float Phi, float Theta, float Psi, float l_1, float l_2, float l_3, float l_4, float l_z,
                                   float Cl_alpha, float Cm_alpha, float rho, float Cd_zero, float K_Cd, float S, float wing_chord,
                                   float V, float Omega_1, float Omega_2, float Omega_3, float Omega_4,
                                   float b_1, float b_2, float b_3, float b_4, float g_1, float g_2, float g_3, float g_4){

    float B_matrix_local[6][14];
    for(int i = 0; i < 14; i++){
        for(int j = 0; j < 6; j++){
            B_matrix_local[j][i] = 0;
        }
    }

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

    //First row
    B_matrix_local[0][0] = -(2*K_p_T*Omega_1*cos(Psi)*cos(Theta)*sin(b_1) + 2*K_p_T*Omega_1*cos(b_1)*cos(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][1] = -(2*K_p_T*Omega_2*cos(Psi)*cos(Theta)*sin(b_2) + 2*K_p_T*Omega_2*cos(b_2)*cos(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][2] = -(2*K_p_T*Omega_3*cos(Psi)*cos(Theta)*sin(b_3) + 2*K_p_T*Omega_3*cos(b_3)*cos(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][3] = -(2*K_p_T*Omega_4*cos(Psi)*cos(Theta)*sin(b_4) + 2*K_p_T*Omega_4*cos(b_4)*cos(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][4] = (K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][8] = -(K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][9] = -(K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][10] = -(K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][11] = -(K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;

//    B_matrix_local[0][12] = -(cos(Phi)*cos(Psi)*cos(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - cos(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Psi)*cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/2 - (S*V*V*rho*cos(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 + Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Psi)*cos(Theta) + (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Psi)*cos(Theta))/2)/m;
//    B_matrix_local[0][13] = -((Cl_alpha*S*Theta*rho*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*V*V)/2 + (cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;


    //Second row
    B_matrix_local[1][0] = (2*K_p_T*Omega_1*cos(b_1)*cos(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_1*cos(Theta)*sin(Psi)*sin(b_1) + 2*K_p_T*Omega_1*cos(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][1] = (2*K_p_T*Omega_2*cos(b_2)*cos(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_2*cos(Theta)*sin(Psi)*sin(b_2) + 2*K_p_T*Omega_2*cos(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][2] = (2*K_p_T*Omega_3*cos(b_3)*cos(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_3*cos(Theta)*sin(Psi)*sin(b_3) + 2*K_p_T*Omega_3*cos(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][3] = (2*K_p_T*Omega_4*cos(b_4)*cos(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_4*cos(Theta)*sin(Psi)*sin(b_4) + 2*K_p_T*Omega_4*cos(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][4] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][5] = -(K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Psi)*cos(b_2) + K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][6] = -(K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Psi)*cos(b_3) + K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][7] = -(K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Psi)*cos(b_4) + K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][8] = (K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;

//    B_matrix_local[1][12] = (sin(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*sin(Psi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/2 + (S*V*V*rho*sin(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Theta)*sin(Psi) - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi))/2)/m;
//    B_matrix_local[1][13] = ((Cl_alpha*S*Theta*rho*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*V*V)/2 + (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;

    //Third row
    B_matrix_local[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
    B_matrix_local[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
    B_matrix_local[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
    B_matrix_local[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;

    B_matrix_local[2][4] = (K_p_T*Omega_1*Omega_1*cos(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*cos(Phi)*cos(Theta)*cos(g_1)*sin(b_1) - K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Phi)*sin(b_1)*sin(g_1))/m;
    B_matrix_local[2][5] = (K_p_T*Omega_2*Omega_2*cos(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*cos(Phi)*cos(Theta)*cos(g_2)*sin(b_2) - K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Phi)*sin(b_2)*sin(g_2))/m;
    B_matrix_local[2][6] = (K_p_T*Omega_3*Omega_3*cos(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*cos(Phi)*cos(Theta)*cos(g_3)*sin(b_3) - K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Phi)*sin(b_3)*sin(g_3))/m;
    B_matrix_local[2][7] = (K_p_T*Omega_4*Omega_4*cos(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*cos(Phi)*cos(Theta)*cos(g_4)*sin(b_4) - K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Phi)*sin(b_4)*sin(g_4))/m;

    B_matrix_local[2][8] = (K_p_T*Omega_1*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*cos(g_1))/m;
    B_matrix_local[2][9] = (K_p_T*Omega_2*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*sin(g_2) + K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*cos(g_2))/m;
    B_matrix_local[2][10] = (K_p_T*Omega_3*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*cos(g_3))/m;
    B_matrix_local[2][11] = (K_p_T*Omega_4*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*sin(g_4) + K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*cos(g_4))/m;

//    B_matrix_local[2][12] = -(sin(Phi)*sin(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) - cos(Phi)*sin(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*cos(Phi)*cos(Theta))/2 + (S*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*sin(Theta))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix_local[2][13] = ((cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + (S*V*V*rho*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/2 + (Cl_alpha*S*Theta*V*V*rho*cos(Theta)*sin(Phi))/2)/m;

    //Fourth row
    B_matrix_local[3][0] = (2*K_p_M*Omega_1*sin(b_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
    B_matrix_local[3][1] = (2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_M*Omega_2*sin(b_2) + 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
    B_matrix_local[3][2] = (2*K_p_M*Omega_3*sin(b_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
    B_matrix_local[3][3] = (2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_M*Omega_4*sin(b_4) + 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;

    B_matrix_local[3][4] = -(K_p_T*Omega_1*Omega_1*l_1_y*cos(g_1)*sin(b_1) - K_p_M*Omega_1*Omega_1*cos(b_1) + K_p_T*Omega_1*Omega_1*l_1_z*sin(b_1)*sin(g_1))/I_xx;
    B_matrix_local[3][5] = -(K_p_M*Omega_2*Omega_2*cos(b_2) + K_p_T*Omega_2*Omega_2*l_2_y*cos(g_2)*sin(b_2) + K_p_T*Omega_2*Omega_2*l_2_z*sin(b_2)*sin(g_2))/I_xx;
    B_matrix_local[3][6] = -(K_p_T*Omega_3*Omega_3*l_3_y*cos(g_3)*sin(b_3) - K_p_M*Omega_3*Omega_3*cos(b_3) + K_p_T*Omega_3*Omega_3*l_3_z*sin(b_3)*sin(g_3))/I_xx;
    B_matrix_local[3][7] = -(K_p_M*Omega_4*Omega_4*cos(b_4) + K_p_T*Omega_4*Omega_4*l_4_y*cos(g_4)*sin(b_4) + K_p_T*Omega_4*Omega_4*l_4_z*sin(b_4)*sin(g_4))/I_xx;

    B_matrix_local[3][8] = (K_p_T*Omega_1*Omega_1*l_1_z*cos(b_1)*cos(g_1) - K_p_T*Omega_1*Omega_1*l_1_y*cos(b_1)*sin(g_1))/I_xx;
    B_matrix_local[3][9] = (K_p_T*Omega_2*Omega_2*l_2_z*cos(b_2)*cos(g_2) - K_p_T*Omega_2*Omega_2*l_2_y*cos(b_2)*sin(g_2))/I_xx;
    B_matrix_local[3][10] = (K_p_T*Omega_3*Omega_3*l_3_z*cos(b_3)*cos(g_3) - K_p_T*Omega_3*Omega_3*l_3_y*cos(b_3)*sin(g_3))/I_xx;
    B_matrix_local[3][11] = (K_p_T*Omega_4*Omega_4*l_4_z*cos(b_4)*cos(g_4) - K_p_T*Omega_4*Omega_4*l_4_y*cos(b_4)*sin(g_4))/I_xx;

//    B_matrix_local[3][12] = 0;
//    B_matrix_local[3][13] = 0;

    //Fifth row
    B_matrix_local[4][0] = -(2*K_p_M*Omega_1*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
    B_matrix_local[4][1] = (2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
    B_matrix_local[4][2] = -(2*K_p_M*Omega_3*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
    B_matrix_local[4][3] = (2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;

    B_matrix_local[4][4] = (K_p_M*Omega_1*Omega_1*sin(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_z*cos(b_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(g_1)*sin(b_1))/I_yy;
    B_matrix_local[4][5] = (K_p_T*Omega_2*Omega_2*l_2_z*cos(b_2) - K_p_M*Omega_2*Omega_2*sin(b_2)*sin(g_2) + K_p_T*Omega_2*Omega_2*l_2_x*cos(g_2)*sin(b_2))/I_yy;
    B_matrix_local[4][6] = (K_p_M*Omega_3*Omega_3*sin(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_z*cos(b_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(g_3)*sin(b_3))/I_yy;
    B_matrix_local[4][7] = (K_p_T*Omega_4*Omega_4*l_4_z*cos(b_4) - K_p_M*Omega_4*Omega_4*sin(b_4)*sin(g_4) + K_p_T*Omega_4*Omega_4*l_4_x*cos(g_4)*sin(b_4))/I_yy;

    B_matrix_local[4][8] = -(K_p_M*Omega_1*Omega_1*cos(b_1)*cos(g_1) - K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_yy;
    B_matrix_local[4][9] = (K_p_M*Omega_2*Omega_2*cos(b_2)*cos(g_2) + K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_yy;
    B_matrix_local[4][10] = -(K_p_M*Omega_3*Omega_3*cos(b_3)*cos(g_3) - K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_yy;
    B_matrix_local[4][11] = (K_p_M*Omega_4*Omega_4*cos(b_4)*cos(g_4) + K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_yy;

//    B_matrix_local[4][12] = (Cm_alpha*S*V*V*rho*wing_chord)/(2*I_yy);
//    B_matrix_local[4][13] = 0;

    //Sixth row
    B_matrix_local[5][0] = -(2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][1] = -(2*K_p_T*Omega_2*l_2_y*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][2] = -(2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][3] = -(2*K_p_T*Omega_4*l_4_y*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][4] = -(K_p_T*Omega_1*Omega_1*l_1_y*cos(b_1) + K_p_M*Omega_1*Omega_1*cos(g_1)*sin(b_1) - K_p_T*Omega_1*Omega_1*l_1_x*sin(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][5] = (K_p_M*Omega_2*Omega_2*cos(g_2)*sin(b_2) - K_p_T*Omega_2*Omega_2*l_2_y*cos(b_2) + K_p_T*Omega_2*Omega_2*l_2_x*sin(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][6] = -(K_p_T*Omega_3*Omega_3*l_3_y*cos(b_3) + K_p_M*Omega_3*Omega_3*cos(g_3)*sin(b_3) - K_p_T*Omega_3*Omega_3*l_3_x*sin(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][7] = (K_p_M*Omega_4*Omega_4*cos(g_4)*sin(b_4) - K_p_T*Omega_4*Omega_4*l_4_y*cos(b_4) + K_p_T*Omega_4*Omega_4*l_4_x*sin(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][8] = -(K_p_M*Omega_1*Omega_1*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_zz;
    B_matrix_local[5][9] = (K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_zz;
    B_matrix_local[5][10] = -(K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_zz;
    B_matrix_local[5][11] = (K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_zz;

//    B_matrix_local[5][12] = 0;
//    B_matrix_local[5][13] = 0;

    memcpy(& B_matrix[0], & B_matrix_local[0], 14*6*sizeof(float) );
}

// This function computes the old full B matrix extended 6x14 in earth reference frame
void compute_B_matrix_extended_fcn_old(float * B_matrix, float I_xx, float I_yy, float I_zz, float m, float K_p_T, float K_p_M,
                                   float Phi, float Theta, float Psi, float l_1, float l_2, float l_3, float l_4, float l_z,
                                   float Cl_alpha, float Cm_alpha, float rho, float Cd_zero, float K_Cd, float S, float wing_chord,
                                   float V, float Omega_1, float Omega_2, float Omega_3, float Omega_4,
                                   float b_1, float b_2, float b_3, float b_4, float g_1, float g_2, float g_3, float g_4){

    float B_matrix_local[6][14];

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

    float b_1_dot, b_2_dot, b_3_dot, b_4_dot, g_1_dot, g_2_dot, g_3_dot, g_4_dot;
    b_1_dot = actuator_state_filt_dot[4];
    b_2_dot = actuator_state_filt_dot[5];
    b_3_dot = actuator_state_filt_dot[6];
    b_4_dot = actuator_state_filt_dot[7];
    g_1_dot = actuator_state_filt_dot[8];
    g_2_dot = actuator_state_filt_dot[9];
    g_3_dot = actuator_state_filt_dot[10];
    g_4_dot = actuator_state_filt_dot[11];

    float J_r = VEHICLE_PROPELLER_INERTIA;
    float p, q, r;
    Phi = euler_vect[0];
    Theta = euler_vect[1];
    Psi = euler_vect[2];
    p = rate_vect[0];
    q = rate_vect[1];
    r = rate_vect[2];

    float Omega_1_dot, Omega_2_dot, Omega_3_dot, Omega_4_dot;
    Omega_1_dot = actuator_state_filt_dot[0];
    Omega_2_dot = actuator_state_filt_dot[1];
    Omega_3_dot = actuator_state_filt_dot[2];
    Omega_4_dot = actuator_state_filt_dot[3];

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
    B_matrix_local[0][0] = 0;
    B_matrix_local[0][1] = 0;
    B_matrix_local[0][2] = 0;
    B_matrix_local[0][3] = 0;

    B_matrix_local[0][4] = (K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][8] = -(K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][9] = -(K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][10] = -(K_p_T*Omega_3*Omega_3*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][11] = -(K_p_T*Omega_4*Omega_4*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;

    // Second row
    B_matrix_local[1][0] = 0;
    B_matrix_local[1][1] = 0;
    B_matrix_local[1][2] = 0;
    B_matrix_local[1][3] = 0;

    B_matrix_local[1][4] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][5] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_2*Omega_2*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][6] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_3*Omega_3*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][7] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_4*Omega_4*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][8] = (K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;

    // Third row
    B_matrix_local[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
    B_matrix_local[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
    B_matrix_local[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
    B_matrix_local[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;

    B_matrix_local[2][4] = 0;
    B_matrix_local[2][5] = 0;
    B_matrix_local[2][6] = 0;
    B_matrix_local[2][7] = 0;

    B_matrix_local[2][8] = 0;
    B_matrix_local[2][9] = 0;
    B_matrix_local[2][10] = 0;
    B_matrix_local[2][11] = 0;

    // Fourth row
    B_matrix_local[3][0] = (2*K_p_M*Omega_1*sin(b_1) + J_r*b_1_dot*cos(b_1) + J_r*q*cos(b_1)*cos(g_1) + J_r*r*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
    B_matrix_local[3][1] = -(2*K_p_M*Omega_2*sin(b_2) + J_r*b_2_dot*cos(b_2) + J_r*q*cos(b_2)*cos(g_2) + J_r*r*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
    B_matrix_local[3][2] = (2*K_p_M*Omega_3*sin(b_3) + J_r*b_3_dot*cos(b_3) + J_r*q*cos(b_3)*cos(g_3) + J_r*r*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
    B_matrix_local[3][3] = -(2*K_p_M*Omega_4*sin(b_4) + J_r*b_4_dot*cos(b_4) + J_r*q*cos(b_4)*cos(g_4) + J_r*r*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;

    B_matrix_local[3][4] = 0;
    B_matrix_local[3][5] = 0;
    B_matrix_local[3][6] = 0;
    B_matrix_local[3][7] = 0;

    B_matrix_local[3][8] = 0;
    B_matrix_local[3][9] = 0;
    B_matrix_local[3][10] = 0;
    B_matrix_local[3][11] = 0;

    // Fifth row
    B_matrix_local[4][0] = -(J_r*g_1_dot*cos(g_1) - J_r*r*sin(b_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_M*Omega_1*cos(b_1)*sin(g_1) + J_r*p*cos(b_1)*cos(g_1) - J_r*b_1_dot*sin(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
    B_matrix_local[4][1] = (J_r*g_2_dot*cos(g_2) - J_r*r*sin(b_2) + 2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) + J_r*p*cos(b_2)*cos(g_2) - J_r*b_2_dot*sin(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
    B_matrix_local[4][2] = -(J_r*g_3_dot*cos(g_3) - J_r*r*sin(b_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_M*Omega_3*cos(b_3)*sin(g_3) + J_r*p*cos(b_3)*cos(g_3) - J_r*b_3_dot*sin(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
    B_matrix_local[4][3] = (J_r*g_4_dot*cos(g_4) - J_r*r*sin(b_4) + 2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) + J_r*p*cos(b_4)*cos(g_4) - J_r*b_4_dot*sin(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;

    B_matrix_local[4][4] = 0;
    B_matrix_local[4][5] = 0;
    B_matrix_local[4][6] = 0;
    B_matrix_local[4][7] = 0;

    B_matrix_local[4][8] = 0;
    B_matrix_local[4][9] = 0;
    B_matrix_local[4][10] = 0;
    B_matrix_local[4][11] = 0;

    // Sixth row
    B_matrix_local[5][0] = -(J_r*g_1_dot*sin(g_1) + J_r*q*sin(b_1) + 2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + J_r*b_1_dot*cos(g_1)*sin(b_1) + J_r*p*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][1] = (J_r*g_2_dot*sin(g_2) + J_r*q*sin(b_2) - 2*K_p_T*Omega_2*l_2_y*sin(b_2) - 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + J_r*b_2_dot*cos(g_2)*sin(b_2) + J_r*p*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][2] = -(J_r*g_3_dot*sin(g_3) + J_r*q*sin(b_3) + 2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + J_r*b_3_dot*cos(g_3)*sin(b_3) + J_r*p*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][3] = (J_r*g_4_dot*sin(g_4) + J_r*q*sin(b_4) - 2*K_p_T*Omega_4*l_4_y*sin(b_4) - 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + J_r*b_4_dot*cos(g_4)*sin(b_4) + J_r*p*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;

//    B_matrix_local[5][4] = -(J_r*Omega_1*q*cos(b_1) + K_p_T*Omega_1^2*l_1_y*cos(b_1) + J_r*Omega_1_dot*cos(g_1)*sin(b_1) + I_xx_tilt*g_1_ddot*cos(b_1)*cos(g_1) + K_p_M*Omega_1^2*cos(g_1)*sin(b_1) - J_r*Omega_1*p*sin(b_1)*sin(g_1) - K_p_T*Omega_1^2*l_1_x*sin(b_1)*sin(g_1) + J_r*Omega_1*b_1_dot*cos(b_1)*cos(g_1))/I_zz;
//    B_matrix_local[5][5] = (J_r*Omega_2*q*cos(b_2) - K_p_T*Omega_2^2*l_2_y*cos(b_2) - J_r*Omega_2_dot*cos(g_2)*sin(b_2) - I_xx_tilt*g_2_ddot*cos(b_2)*cos(g_2) + K_p_M*Omega_2^2*cos(g_2)*sin(b_2) - J_r*Omega_2*p*sin(b_2)*sin(g_2) + K_p_T*Omega_2^2*l_2_x*sin(b_2)*sin(g_2) + J_r*Omega_2*b_2_dot*cos(b_2)*cos(g_2))/I_zz;
//    B_matrix_local[5][6] = -(J_r*Omega_3*q*cos(b_3) + K_p_T*Omega_3^2*l_3_y*cos(b_3) + J_r*Omega_3_dot*cos(g_3)*sin(b_3) + I_xx_tilt*g_3_ddot*cos(b_3)*cos(g_3) + K_p_M*Omega_3^2*cos(g_3)*sin(b_3) - J_r*Omega_3*p*sin(b_3)*sin(g_3) - K_p_T*Omega_3^2*l_3_x*sin(b_3)*sin(g_3) + J_r*Omega_3*b_3_dot*cos(b_3)*cos(g_3))/I_zz;
//    B_matrix_local[5][7] = (J_r*Omega_4*q*cos(b_4) - K_p_T*Omega_4^2*l_4_y*cos(b_4) - J_r*Omega_4_dot*cos(g_4)*sin(b_4) - I_xx_tilt*g_4_ddot*cos(b_4)*cos(g_4) + K_p_M*Omega_4^2*cos(g_4)*sin(b_4) - J_r*Omega_4*p*sin(b_4)*sin(g_4) + K_p_T*Omega_4^2*l_4_x*sin(b_4)*sin(g_4) + J_r*Omega_4*b_4_dot*cos(b_4)*cos(g_4))/I_zz;

    B_matrix_local[5][4] = 0;
    B_matrix_local[5][5] = 0;
    B_matrix_local[5][6] = 0;
    B_matrix_local[5][7] = 0;

    B_matrix_local[5][8] = -(J_r*Omega_1*g_1_dot*cos(g_1) - I_yy_tilt*b_1_ddot*cos(g_1) + J_r*Omega_1_dot*cos(b_1)*sin(g_1) - I_xx_tilt*g_1_ddot*sin(b_1)*sin(g_1) + K_p_M*Omega_1*Omega_1*cos(b_1)*sin(g_1) - J_r*Omega_1*b_1_dot*sin(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*cos(g_1) + J_r*Omega_1*p*cos(b_1)*cos(g_1))/I_zz;
    B_matrix_local[5][9] = (I_yy_tilt*b_2_ddot*cos(g_2) + J_r*Omega_2*g_2_dot*cos(g_2) - J_r*Omega_2_dot*cos(b_2)*sin(g_2) + I_xx_tilt*g_2_ddot*sin(b_2)*sin(g_2) + K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - J_r*Omega_2*b_2_dot*sin(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2) + J_r*Omega_2*p*cos(b_2)*cos(g_2))/I_zz;
    B_matrix_local[5][10] = -(J_r*Omega_3*g_3_dot*cos(g_3) - I_yy_tilt*b_3_ddot*cos(g_3) + J_r*Omega_3_dot*cos(b_3)*sin(g_3) - I_xx_tilt*g_3_ddot*sin(b_3)*sin(g_3) + K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) - J_r*Omega_3*b_3_dot*sin(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3) + J_r*Omega_3*p*cos(b_3)*cos(g_3))/I_zz;
    B_matrix_local[5][11] = (I_yy_tilt*b_4_ddot*cos(g_4) + J_r*Omega_4*g_4_dot*cos(g_4) - J_r*Omega_4_dot*cos(b_4)*sin(g_4) + I_xx_tilt*g_4_ddot*sin(b_4)*sin(g_4) + K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - J_r*Omega_4*b_4_dot*sin(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4) + J_r*Omega_4*p*cos(b_4)*cos(g_4))/I_zz;


    //Adding extra states
    B_matrix_local[0][12] = 0;
    B_matrix_local[0][13] = 0;

    B_matrix_local[1][12] = 0;
    B_matrix_local[1][13] = 0;

    B_matrix_local[2][12] = 0;
    B_matrix_local[2][13] = 0;

    B_matrix_local[3][12] = 0.f;
    B_matrix_local[3][13] = 0.f;

    B_matrix_local[4][12] = 0;
    B_matrix_local[4][13] = 0;

    B_matrix_local[5][12] = 0.f;
    B_matrix_local[5][13] = 0.f;

    memcpy(& B_matrix[0], & B_matrix_local[0], 14*6*sizeof(float) );
}

// Computation using the same elements used in the old B but with the new neglected variables
void compute_B_matrix_extended_fcn_hybrid(float * B_matrix, float I_xx, float I_yy, float I_zz, float m, float K_p_T, float K_p_M,
                                   float Phi, float Theta, float Psi, float l_1, float l_2, float l_3, float l_4, float l_z,
                                   float Cl_alpha, float Cm_alpha, float rho, float Cd_zero, float K_Cd, float S, float wing_chord,
                                   float V, float Omega_1, float Omega_2, float Omega_3, float Omega_4,
                                   float b_1, float b_2, float b_3, float b_4, float g_1, float g_2, float g_3, float g_4){

    float B_matrix_local[6][14];
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

    //First row
    B_matrix_local[0][0] = 0;
    B_matrix_local[0][1] = 0;
    B_matrix_local[0][2] = 0;
    B_matrix_local[0][3] = 0;

    B_matrix_local[0][4] = (K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][8] = -(K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][9] = -(K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][10] = -(K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][11] = -(K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;

//    B_matrix_local[0][12] = -(cos(Phi)*cos(Psi)*cos(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - cos(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Psi)*cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/2 - (S*V*V*rho*cos(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 + Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Psi)*cos(Theta) + (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Psi)*cos(Theta))/2)/m;
//    B_matrix_local[0][13] = -((Cl_alpha*S*Theta*rho*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*V*V)/2 + (cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;

    B_matrix_local[0][12] = 0;
    B_matrix_local[0][13] = 0;

    //Second row
    B_matrix_local[1][0] = 0;
    B_matrix_local[1][1] = 0;
    B_matrix_local[1][2] = 0;
    B_matrix_local[1][3] = 0;

    B_matrix_local[1][4] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][5] = -(K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Psi)*cos(b_2) + K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][6] = -(K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Psi)*cos(b_3) + K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][7] = -(K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Psi)*cos(b_4) + K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][8] = (K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;

//    B_matrix_local[1][12] = (sin(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*sin(Psi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/2 + (S*V*V*rho*sin(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Theta)*sin(Psi) - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi))/2)/m;
//    B_matrix_local[1][13] = ((Cl_alpha*S*Theta*rho*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*V*V)/2 + (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;

    B_matrix_local[1][12] = 0;
    B_matrix_local[1][13] = 0;

    //Third row
    B_matrix_local[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
    B_matrix_local[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
    B_matrix_local[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
    B_matrix_local[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;

    B_matrix_local[2][4] = 0;
    B_matrix_local[2][5] = 0;
    B_matrix_local[2][6] = 0;
    B_matrix_local[2][7] = 0;

    B_matrix_local[2][8] = 0;
    B_matrix_local[2][9] = 0;
    B_matrix_local[2][10] = 0;
    B_matrix_local[2][11] = 0;

//    B_matrix_local[2][12] = -(sin(Phi)*sin(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) - cos(Phi)*sin(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*cos(Phi)*cos(Theta))/2 + (S*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*sin(Theta))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
//    B_matrix_local[2][13] = ((cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + (S*V*V*rho*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/2 + (Cl_alpha*S*Theta*V*V*rho*cos(Theta)*sin(Phi))/2)/m;

    B_matrix_local[2][12] = 0;
    B_matrix_local[2][13] = 0;

    //Fourth row
    B_matrix_local[3][0] = (2*K_p_M*Omega_1*sin(b_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
    B_matrix_local[3][1] = (2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_M*Omega_2*sin(b_2) + 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
    B_matrix_local[3][2] = (2*K_p_M*Omega_3*sin(b_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
    B_matrix_local[3][3] = (2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_M*Omega_4*sin(b_4) + 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;

    B_matrix_local[3][4] = 0;
    B_matrix_local[3][5] = 0;
    B_matrix_local[3][6] = 0;
    B_matrix_local[3][7] = 0;

    B_matrix_local[3][8] = 0;
    B_matrix_local[3][9] = 0;
    B_matrix_local[3][10] = 0;
    B_matrix_local[3][11] = 0;

    B_matrix_local[3][12] = 0.f;
    B_matrix_local[3][13] = 0.f;

    //Fifth row
    B_matrix_local[4][0] = -(2*K_p_M*Omega_1*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
    B_matrix_local[4][1] = (2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
    B_matrix_local[4][2] = -(2*K_p_M*Omega_3*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
    B_matrix_local[4][3] = (2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;

    B_matrix_local[4][4] = 0;
    B_matrix_local[4][5] = 0;
    B_matrix_local[4][6] = 0;
    B_matrix_local[4][7] = 0;

    B_matrix_local[4][8] = 0;
    B_matrix_local[4][9] = 0;
    B_matrix_local[4][10] = 0;
    B_matrix_local[4][11] = 0;

//    B_matrix_local[4][12] = (Cm_alpha * S * V*V* rho * wing_chord) / (2 * I_yy);
//    B_matrix_local[4][13] = 0.f;

    B_matrix_local[4][12] = 0;
    B_matrix_local[4][13] = 0;


    // Sixth row
    B_matrix_local[5][0] = -(2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][1] = -(2*K_p_T*Omega_2*l_2_y*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][2] = -(2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][3] = -(2*K_p_T*Omega_4*l_4_y*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][4] = 0;
    B_matrix_local[5][5] = 0;
    B_matrix_local[5][6] = 0;
    B_matrix_local[5][7] = 0;

    B_matrix_local[5][8] = -(K_p_M*Omega_1*Omega_1*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_zz;
    B_matrix_local[5][9] = (K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_zz;
    B_matrix_local[5][10] = -(K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_zz;
    B_matrix_local[5][11] = (K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_zz;

    B_matrix_local[5][12] = 0.f;
    B_matrix_local[5][13] = 0.f;

    memcpy(& B_matrix[0], & B_matrix_local[0], 14*6*sizeof(float) );
}

/** Moore–Penrose pseudo-inverse
 *
 * Given a matrix B(num_row,num_column), with num_row < num_column, this routine computes its Moore–Penrose inverse,
 * The Moore–Penrose inverse is based on the SVD decomposition.
 *
 * @param num_row number of rows of input the matrix B_matrix
 * @param num_column number of column of input the matrix B_matrix
 * @param B_matrix pointer to the Matrix to invert
 * @param B_inv pointer of the resulting inverted matrix. B_inv(num_column,num_row)
 */
void compute_pseudoinverse_fcn(int num_row, int num_column, float * B_matrix, float * B_inv){

    //Print the B matrix
    float B_matrix_local[num_row][num_column];
    //Make a local copy of the B matrix
    memcpy(&B_matrix_local[0], &B_matrix[0], num_row * num_column * sizeof(float));

    //Declare all the necessary variables:
    float W_diag_array[num_row];
    float V_matrix[num_row][num_row];
    float B_matrix_transposed[num_column][num_row];
    float * B_matrix_ptr[num_column];
    float * V_matrix_ptr[num_row];
    float out_local[num_row][num_row];
    float B_inv_local[num_column][num_row];
    float W_diag_inverted[num_row][num_row];
    float U_transposed[num_row][num_column];

    //Transpose matrix B_in
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_column; j++) {
            B_matrix_transposed[j][i] = B_matrix_local[i][j];
        }
    }

    //Assign the pointer of B_matrix_transposed into B_matrix_ptr
    for (int j = 0; j < num_column; j++) {
        B_matrix_ptr[j] = &B_matrix_transposed[j][0];
    }

    // Pre-assign the matrices for the SVD decomposition
    for (int i = 0; i < num_row; i++) {
        V_matrix_ptr[i] = &V_matrix[i][0];
    }

    //Decompose the B matrix with the SVD decomposition module
    pprz_svd_float(B_matrix_ptr, & W_diag_array[0] , V_matrix_ptr, num_column, num_row);

    //Sort the eigenvalues and the V and U matrices with bubblesort algorithm:
    //Note that U is B_matrix_transposed;
    //FIXME need to improve speed!!!!
    float temp_W = 0;
    float temp_U = 0;
    float temp_V = 0;
    for (int j = 0; j < num_row; j++)
    {
        for (int i = 0; i < num_row - 1; i++)
        {
            if (W_diag_array[i] < W_diag_array[i + 1])
            {
                //Swap W
                temp_W = W_diag_array[i + 1];
                W_diag_array[i + 1] = W_diag_array[i];
                W_diag_array[i] = temp_W;
                //Swap U
                for(int k = 0; k < num_column ; k++){
                    temp_U = B_matrix_transposed[k][i + 1];
                    B_matrix_transposed[k][i + 1] = B_matrix_transposed[k][i];
                    B_matrix_transposed[k][i] = temp_U;
                }
                //Swap V
                for(int k = 0; k < num_row ; k++){
                    temp_V = V_matrix[k][i + 1];
                    V_matrix[k][i + 1] = V_matrix[k][i];
                    V_matrix[k][i] = temp_V;
                }
            }
        }
    }

    //Transpose matrix U from the SVD output
    for (int i = 0; i < num_column; i++) {
        for (int j = 0; j < num_row; j++) {
            U_transposed[j][i] = B_matrix_transposed[i][j];
        }
    }


    int num_row_mod = num_row;
    //Decrease the number of row if some eigenvalues are too small to avoid instabilities:
    for (int i = num_row - 1; i > 0; i--) {
        if(W_diag_array[i] < 0.0000001 * W_diag_array[0]){
            num_row_mod--;
        }
    }

    // Invert the diag values and create the diagonal matrix
    for (int i = 0; i < num_row_mod; i++) {
        for (int j = 0; j < num_row_mod; j++) {
            if(i == j){
                W_diag_inverted[j][i] = 1/W_diag_array[i];
            }
            else{
                W_diag_inverted[j][i] = 0;
            }
        }
    }

    //Multiply V_matrix with the diagonal matrix W_diag_inverted
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_row_mod; j++) {
            out_local[i][j] = 0.;
            for (int k = 0; k < num_row_mod; k++) {
                out_local[i][j] += V_matrix[i][k] * W_diag_inverted[j][k];
            }
        }
    }

    //Multiply out_local with the matrix U_transposed
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_column; j++) {
            B_inv_local[j][i] = 0.;
            for (int k = 0; k < num_row_mod; k++) {
                B_inv_local[j][i] += out_local[i][k] * U_transposed[k][j];
            }
        }
    }

    memcpy(B_inv, & B_inv_local[0], num_row * num_column * sizeof(float));
}

int RSPI_optimization(float * B_matrix_in, int num_row, int num_column, float * max_u, float * min_u, float * out_du,
                      float * out_u, float * des_u, float * actual_u, int max_iter, float * pseudo_control){

    float B_matrix_local[num_row][num_column];
    memcpy(& B_matrix_local[0], B_matrix_in, num_row * num_column * sizeof(float));
    //Initialize some variables for the RSPI
    int locked_actuator[num_column];
    float scaling_array[num_column];
    for (int k = 0; k < num_column; k++){
        locked_actuator[k] = 0;
        scaling_array[k] = 1;
    }
    float min_scaling_array = 1;
    int sum_locked_actuator = 0;
    float sensibility_lock_actuator = 0.01;
    float sensibility_pseudo_control = 0.01;

    float max_du_scaled_iter[num_column];
    float min_du_scaled_iter[num_column];


    float pseudo_control_with_increment[num_row];
    //Make a copy of the pseudo control array to add the increment inside the inversion:
    memcpy(& pseudo_control_with_increment[0], & pseudo_control[0], num_row * sizeof(float));

    //Add the actual actuator position and the prioritized actuator states to the pseudo-control input through the effectiveness matrix:
    for (int k = 0; k < num_row; k++) {
        for (int i = 0; i < num_column; i++) {
            pseudo_control_with_increment[k] += (actual_u[i] - des_u[i]) * B_matrix_local[k][i];
        }
    }

    //Compute the gains for the B matrix with minimum and maximum actuator position
    float gain_motor = (max_u[0] - min_u[0])/2;
    float gain_el = (max_u[4] - min_u[4])/2;
    float gain_az = (max_u[8] - min_u[8])/2;
    float gain_aoa = (max_u[12] - min_u[12])/2;
    float gain_phi = max_u[13];


    //Scale the effectiveness matrix with the actuator gains
    float B_matrix_scaled[num_row][num_column];
    for (int j = 0; j < num_row; j++) {
        for (int i = 0; i < 4; i++) {
            B_matrix_scaled[j][i] = B_matrix_local[j][i] * gain_motor;
            B_matrix_scaled[j][i + 4] = B_matrix_local[j][i + 4] * gain_el;
            B_matrix_scaled[j][i + 8] = B_matrix_local[j][i + 8] * gain_az;
        }
        B_matrix_scaled[j][12] = B_matrix_local[j][12] * gain_aoa;
        B_matrix_scaled[j][13] = B_matrix_local[j][13] * gain_phi;
    }

    //Save a copy of the scaled effectiveness matrix, so then later we can modify it for the RSPI
    float B_matrix_scaled_iter[num_row][num_column];
    memcpy(&B_matrix_scaled_iter[0], &B_matrix_scaled[0], num_row * num_column * sizeof(float));

    //Compute the maximum and minimum scaled actuator values:
    float max_u_scaled[num_column];
    float min_u_scaled[num_column];
    for (int i = 0; i < 4; i++) {
        max_u_scaled[i] = max_u[0] / gain_motor;
        max_u_scaled[i + 4] = max_u[4] / gain_el;
        max_u_scaled[i + 8] = max_u[8] / gain_az;
        min_u_scaled[i] = min_u[0] / gain_motor;
        min_u_scaled[i + 4] = min_u[4] / gain_el;
        min_u_scaled[i + 8] = min_u[8] / gain_az;
    }
    max_u_scaled[12] = max_u[12] / gain_aoa;
    max_u_scaled[13] = max_u[13] / gain_phi;
    min_u_scaled[12] = min_u[12] / gain_aoa;
    min_u_scaled[13] = -max_u[13] / gain_phi;

    //Compute the pseudoinverse for the scaled B matrix
    float B_matrix_inv[num_column][num_row];
    compute_pseudoinverse_fcn(num_row, num_column, B_matrix_scaled[0], B_matrix_inv[0]);


    //Compute the initial actuation increment scaled command by multiplying desired acceleration with the pseudo-inverse matrix
    float indi_u_scaled_init[num_column];
    for (int j = 0; j < num_column; j++) {
        //Cleanup previous value
        indi_u_scaled_init[j] = 0.;
        for (int k = 0; k < num_row; k++) {
            indi_u_scaled_init[j] += pseudo_control_with_increment[k] * B_matrix_inv[j][k];
        }
    }

    //Build the scaling array:
    for (int i = 0; i < num_column; i++) {
        if(indi_u_scaled_init[i] > 0){
            scaling_array[i] = fmin(1,max_u_scaled[i] / indi_u_scaled_init[i]);
        }
        else if(indi_u_scaled_init[i] < 0){
            scaling_array[i] = fmin(1,min_u_scaled[i] / indi_u_scaled_init[i]);
        }
        if(scaling_array[i] < 0){
            scaling_array[i] = 0;
        }
        //Find the minimum of the scaling_array to get the scaling parameter for the whole control input array:
        if (scaling_array[i] < min_scaling_array) {
            min_scaling_array = scaling_array[i];
        }
    }

    // Check for saturation and compute the scaling array accordingly
    for (int i = 0; i < num_column; i++) {
        if ( fabs(scaling_array[i] - min_scaling_array) < sensibility_lock_actuator && min_scaling_array < 1) {
            locked_actuator[i] = 1;
            sum_locked_actuator ++;
            for (int j = 0; j < num_row; j++) {
                B_matrix_scaled_iter[j][i] = 0;
            }
        }
    }

    //If we do have saturation, scale the actuator position to the allowed one and recalculate the achieved pseudo-control:
    float indi_u_scaled_iter[num_column];
    float indi_u_scaled[num_column];
    float achieved_dv[num_row];
    float residual_dv[num_row];
    float sum_residual_dv = 0;
    if (min_scaling_array < 1) {
        for (int i = 0; i < num_column; i++) {
            indi_u_scaled_iter[i] = indi_u_scaled_init[i] * min_scaling_array;
            indi_u_scaled[i] = indi_u_scaled_iter[i];
        }

        //Calculate the associated pseudo control achieved with the constrained control input:
        for (int j = 0; j < num_row; j++) {
            //Cleanup previous value
            achieved_dv[j] = 0.;
            for (int k = 0; k < num_column; k++) {
                achieved_dv[j] += B_matrix_scaled[j][k] * indi_u_scaled[k];
            }
            //Compute the residual with the main pseudo-control commanded:
            residual_dv[j] = pseudo_control_with_increment[j] - achieved_dv[j];
            sum_residual_dv += fabs(residual_dv[j]);
        }

    }
    //If no saturation occurs just return the control inputs as they are.
    else {
        for (int i = 0; i < num_column; i++) {
            indi_u_scaled[i] = indi_u_scaled_init[i];
        }
    }

    //Initialize the variable for the iterative part of the RSPI
    int iter_count = 1;

    //If we are saturated, and we didn't reach the pseudo-control target, begin the iterative process:
    while (min_scaling_array < 1 && iter_count < max_iter &&
           sum_residual_dv > sensibility_pseudo_control) {

        //Increase the counter:
        iter_count++;

        //Compute the pseudoinverse with the modified B matrix
        compute_pseudoinverse_fcn(num_row, num_column, B_matrix_scaled_iter[0], B_matrix_inv[0]);

        //Compute the new control input associated with the residual of the pseudo-control of the previous iteration with the modified B matrix inverted
        for (int j = 0; j < num_column; j++) {
            //Cleanup previous value
            indi_u_scaled_init[j] = 0.;
            for (int k = 0; k < num_row; k++) {
                indi_u_scaled_init[j] += residual_dv[k] * B_matrix_inv[j][k];
            }
        }

        //Reduce the new maximum control input position of the previously allocated value.
        //Notice that we pass from a global to an incremental problem.
        for (int i = 0; i < num_column; i++) {
            max_du_scaled_iter[i] = max_u_scaled[i] - indi_u_scaled[i];
            min_du_scaled_iter[i] = min_u_scaled[i] - indi_u_scaled[i];
            scaling_array[i] = 1;
        }
        min_scaling_array = 1;

        //Build again the scaling array, this time with the new maximum actuator values:
        for (int i = 0; i < num_column; i++) {
            if (indi_u_scaled_init[i] > 0 && locked_actuator[i] == 0) {
                scaling_array[i] = fmin(1, max_du_scaled_iter[i] / indi_u_scaled_init[i]);
            } else if (indi_u_scaled_init[i] < 0 && locked_actuator[i] == 0) {
                scaling_array[i] = fmin(1, min_du_scaled_iter[i] / indi_u_scaled_init[i]);
            }
            if (scaling_array[i] < 0) {
                scaling_array[i] = 0;
            }
            //Find the minimum of the scaling_array to get the scaling parameter for the whole control input array:
            if (scaling_array[i] < min_scaling_array) {
                min_scaling_array = scaling_array[i];
            }
        }

        //Understand which actuators are locked(saturated) and register them into the locked_actuator array, also update the new B matrix:
        for (int i = 0; i < num_column; i++) {
            if (fabs(scaling_array[i] - min_scaling_array) < sensibility_locked_actuator && min_scaling_array < 1) {
                locked_actuator[i] = 1;
                sum_locked_actuator ++;
                for (int j = 0; j < num_row; j++) {
                    B_matrix_scaled_iter[j][i] = 0;
                }
            }
        }

        //Scale the actuator position to the allowed one and re-calculate the achieved pseudo-control:
        for (int i = 0; i < num_column; i++) {
            indi_u_scaled_iter[i] = indi_u_scaled_init[i] * min_scaling_array;
            indi_u_scaled[i] += indi_u_scaled_iter[i];
        }

        //Calculate the associated pseudo control achieved with the constrained control input:
        sum_residual_dv = 0.; //Cleanup previous value
        for (int j = 0; j < num_row; j++) {
            achieved_dv[j] = 0.; //Cleanup previous value
            for (int k = 0; k < num_column; k++) {
                achieved_dv[j] += B_matrix_scaled[j][k] * indi_u_scaled[k];
            }
            //Compute the residual with the main pseudo-control commanded:
            residual_dv[j] = pseudo_control_with_increment[j] - achieved_dv[j];
            sum_residual_dv += fabs(residual_dv[j]);
        }

    }

    //Multiply the scaled actuator position with the gains and get the real actuator commands, also subtract the prioritized position:
    for (int i = 0; i < 4; i++) {
        out_u[i] = indi_u_scaled[i] * gain_motor;
        out_u[i+4] = indi_u_scaled[i+4] * gain_el;
        out_u[i+8] = indi_u_scaled[i+8] * gain_az;
    }
    out_u[12] = indi_u_scaled[12] * gain_aoa;
    out_u[13] = indi_u_scaled[13] * gain_phi;

    //Now subtract the prioritized actuator position to the computed actuator value and also compute the increment.
    //ALSO FILTER NAN ISSUES
    for (int i = 0; i < num_column; i++) {
        out_u[i] += des_u[i];
        if(isnan(out_u[i])){
            out_u[i] = 0;
        }
        out_du[i] = out_u[i] - actual_u[i];
    }

    return iter_count;
}

int RSPI_optimization_incremental(float * B_matrix_in, int num_row, int num_column, float * max_u, float * min_u, float * out_du,
                      float * out_u, float * des_u, float * actual_u, int max_iter, float * pseudo_control){

    float B_matrix_local[num_row][num_column];
    memcpy(& B_matrix_local[0], B_matrix_in, num_row * num_column * sizeof(float));
    //Initialize some variables for the RSPI
    int locked_actuator[num_column];
    float scaling_array[num_column];
    for (int k = 0; k < num_column; k++){
        locked_actuator[k] = 0;
        scaling_array[k] = 1;
    }
    float min_scaling_array = 1;
    int sum_locked_actuator = 0;
    float sensibility_lock_actuator = 0.01;
    float sensibility_pseudo_control = 0.01;

    float max_du[num_column];
    float min_du[num_column];
    float max_du_scaled_iter[num_column];
    float min_du_scaled_iter[num_column];


    //Compute the gains for the B matrix with minimum and maximum actuator position
    float gain_motor = (max_u[0] - min_u[0])/2;
    float gain_el = (max_u[4] - min_u[4])/2;
    float gain_az = (max_u[8] - min_u[8])/2;
    float gain_aoa = (max_u[12] - min_u[12])/2;
    float gain_phi = max_u[13];


    //Scale the effectiveness matrix with the actuator gains
    float B_matrix_scaled[num_row][num_column];
    for (int j = 0; j < num_row; j++) {
        for (int i = 0; i < 4; i++) {
            B_matrix_scaled[j][i] = B_matrix_local[j][i] * gain_motor;
            B_matrix_scaled[j][i + 4] = B_matrix_local[j][i + 4] * gain_el;
            B_matrix_scaled[j][i + 8] = B_matrix_local[j][i + 8] * gain_az;
        }
        B_matrix_scaled[j][12] = B_matrix_local[j][12] * gain_aoa;
        B_matrix_scaled[j][13] = B_matrix_local[j][13] * gain_phi;
    }

    //Save a copy of the scaled effectiveness matrix, so then later we can modify it for the RSPI
    float B_matrix_scaled_iter[num_row][num_column];
    memcpy(&B_matrix_scaled_iter[0], &B_matrix_scaled[0], num_row * num_column * sizeof(float));

    //Compute the maximum and minimum local increment:
    for (int i = 0; i < num_column; i++) {
        max_du[i] = max_u[i] - actual_u[i];
        min_du[i] = min_u[i] - actual_u[i];
    }

//    //Compute the maximum and minimum local increment with servo rates:
//    for (int i = 0; i < num_column; i++) {
//        if(i>3){
//            max_du[i] = max_u[i] - actual_u[i];
//            min_du[i] = min_u[i] - actual_u[i];
//        }
//        else{
//            max_du[i] = max_u[i] - actual_u[i];
//            min_du[i] = min_u[i] - actual_u[i];
//        }
//
//    }

    //Now scale the max and min values with the gains:
    float max_du_scaled[num_column];
    float min_du_scaled[num_column];
    for (int i = 0; i < 4; i++) {
        max_du_scaled[i] = max_du[0] / gain_motor;
        max_du_scaled[i + 4] = max_du[4] / gain_el;
        max_du_scaled[i + 8] = max_du[8] / gain_az;
        min_du_scaled[i] = min_du[0] / gain_motor;
        min_du_scaled[i + 4] = min_du[4] / gain_el;
        min_du_scaled[i + 8] = min_du[8] / gain_az;
    }
    max_du_scaled[12] = max_du[12] / gain_aoa;
    max_du_scaled[13] = max_du[13] / gain_phi;
    min_du_scaled[12] = min_du[12] / gain_aoa;
    min_du_scaled[13] = -max_du[13] / gain_phi;

    //Compute the pseudoinverse for the scaled B matrix
    float B_matrix_inv[num_column][num_row];
    compute_pseudoinverse_fcn(num_row, num_column, B_matrix_scaled[0], B_matrix_inv[0]);


    //Compute the initial actuation increment scaled command by multiplying desired acceleration with the pseudo-inverse matrix
    float indi_du_scaled_init[num_column];
    for (int j = 0; j < num_column; j++) {
        //Cleanup previous value
        indi_du_scaled_init[j] = 0.;
        for (int k = 0; k < num_row; k++) {
            indi_du_scaled_init[j] += pseudo_control[k] * B_matrix_inv[j][k];
        }
    }

    //Build the scaling array:
    for (int i = 0; i < num_column; i++) {
        if(indi_du_scaled_init[i] > 0){
            scaling_array[i] = fmin(1,max_du_scaled[i] / indi_du_scaled_init[i]);
        }
        else if(indi_du_scaled_init[i] < 0){
            scaling_array[i] = fmin(1,min_du_scaled[i] / indi_du_scaled_init[i]);
        }
        if(scaling_array[i] < 0){
            scaling_array[i] = 0;
        }
        //Find the minimum of the scaling_array to get the scaling parameter for the whole control input array:
        if (scaling_array[i] < min_scaling_array) {
            min_scaling_array = scaling_array[i];
        }
    }

    // Check for saturation and compute the scaling array accordingly
    for (int i = 0; i < num_column; i++) {
        if ( fabs(scaling_array[i] - min_scaling_array) < sensibility_lock_actuator && min_scaling_array < 1) {
            locked_actuator[i] = 1;
            sum_locked_actuator ++;
            for (int j = 0; j < num_row; j++) {
                B_matrix_scaled_iter[j][i] = 0;
            }
        }
    }

    //If we do have saturation, scale the actuator position to the allowed one and recalculate the achieved pseudo-control:
    float indi_du_scaled_iter[num_column];
    float indi_du_scaled[num_column];
    float achieved_dv[num_row];
    float residual_dv[num_row];
    float sum_residual_dv = 0;
    if (min_scaling_array < 1) {
        for (int i = 0; i < num_column; i++) {
            indi_du_scaled_iter[i] = indi_du_scaled_init[i] * min_scaling_array;
            indi_du_scaled[i] = indi_du_scaled_iter[i];
        }

        //Calculate the associated pseudo control achieved with the constrained control input:
        for (int j = 0; j < num_row; j++) {
            //Cleanup previous value
            achieved_dv[j] = 0.;
            for (int k = 0; k < num_column; k++) {
                achieved_dv[j] += B_matrix_scaled[j][k] * indi_du_scaled[k];
            }
            //Compute the residual with the main pseudo-control commanded:
            residual_dv[j] = pseudo_control[j] - achieved_dv[j];
            sum_residual_dv += fabs(residual_dv[j]);
        }

    }
        //If no saturation occurs just return the control inputs as they are.
    else {
        for (int i = 0; i < num_column; i++) {
            indi_du_scaled[i] = indi_du_scaled_init[i];
        }
    }

    //Initialize the variable for the iterative part of the RSPI
    int iter_count = 1;

    //If we are saturated, and we didn't reach the pseudo-control target, begin the iterative process:
    while (min_scaling_array < 1 && iter_count < max_iter &&
           sum_residual_dv > sensibility_pseudo_control) {

        //Increase the counter:
        iter_count++;

        //Compute the pseudoinverse with the modified B matrix
        compute_pseudoinverse_fcn(num_row, num_column, B_matrix_scaled_iter[0], B_matrix_inv[0]);

        //Compute the new control input associated with the residual of the pseudo-control of the previous iteration with the modified B matrix inverted
        for (int j = 0; j < num_column; j++) {
            //Cleanup previous value
            indi_du_scaled_init[j] = 0.;
            for (int k = 0; k < num_row; k++) {
                indi_du_scaled_init[j] += residual_dv[k] * B_matrix_inv[j][k];
            }
        }

        //Reduce the new maximum control input position of the previously allocated value.
        //Notice that we pass from a global to an incremental problem.
        for (int i = 0; i < num_column; i++) {
            max_du_scaled_iter[i] = max_du_scaled[i] - indi_du_scaled[i];
            min_du_scaled_iter[i] = min_du_scaled[i] - indi_du_scaled[i];
            scaling_array[i] = 1;
        }
        min_scaling_array = 1;

        //Build again the scaling array, this time with the new maximum actuator values:
        for (int i = 0; i < num_column; i++) {
            if (indi_du_scaled_init[i] > 0 && locked_actuator[i] == 0) {
                scaling_array[i] = fmin(1, max_du_scaled_iter[i] / indi_du_scaled_init[i]);
            } else if (indi_du_scaled_init[i] < 0 && locked_actuator[i] == 0) {
                scaling_array[i] = fmin(1, min_du_scaled_iter[i] / indi_du_scaled_init[i]);
            }
            if (scaling_array[i] < 0) {
                scaling_array[i] = 0;
            }
            //Find the minimum of the scaling_array to get the scaling parameter for the whole control input array:
            if (scaling_array[i] < min_scaling_array) {
                min_scaling_array = scaling_array[i];
            }
        }

        //Understand which actuators are locked(saturated) and register them into the locked_actuator array, also update the new B matrix:
        for (int i = 0; i < num_column; i++) {
            if (fabs(scaling_array[i] - min_scaling_array) < sensibility_locked_actuator && min_scaling_array < 1) {
                locked_actuator[i] = 1;
                sum_locked_actuator ++;
                for (int j = 0; j < num_row; j++) {
                    B_matrix_scaled_iter[j][i] = 0;
                }
            }
        }

        //Scale the actuator position to the allowed one and re-calculate the achieved pseudo-control:
        for (int i = 0; i < num_column; i++) {
            indi_du_scaled_iter[i] = indi_du_scaled_init[i] * min_scaling_array;
            indi_du_scaled[i] += indi_du_scaled_iter[i];
        }

        //Calculate the associated pseudo control achieved with the constrained control input:
        sum_residual_dv = 0.; //Cleanup previous value
        for (int j = 0; j < num_row; j++) {
            achieved_dv[j] = 0.; //Cleanup previous value
            for (int k = 0; k < num_column; k++) {
                achieved_dv[j] += B_matrix_scaled[j][k] * indi_du_scaled[k];
            }
            //Compute the residual with the main pseudo-control commanded:
            residual_dv[j] = pseudo_control[j] - achieved_dv[j];
            sum_residual_dv += fabs(residual_dv[j]);
        }

    }

    //Multiply the scaled actuator position with the gains and get the real actuator commands, also subtract the prioritized position:
    for (int i = 0; i < 4; i++) {
        out_du[i] = indi_du_scaled[i] * gain_motor;
        out_du[i+4] = indi_du_scaled[i+4] * gain_el;
        out_du[i+8] = indi_du_scaled[i+8] * gain_az;
    }
    out_du[12] = indi_du_scaled[12] * gain_aoa;
    out_du[13] = indi_du_scaled[13] * gain_phi;

    //Now subtract the prioritized actuator position to the computed actuator value and also compute the increment.
    //ALSO FILTER NAN ISSUES
    for (int i = 0; i < num_column; i++) {
        if(isnan(out_du[i])){
            out_du[i] = 0;
        }
        out_u[i] = out_du[i] + actual_u[i];
    }

    return iter_count;
}

void basic_inversion(float * B_matrix_in, int num_row, int num_column, float * out_du, float * out_u,
                     float * des_u, float * actual_u, float * pseudo_control){

    float B_matrix[num_row][num_column];
    float B_matrix_inv[num_column][num_row];
    memcpy( & B_matrix[0], & B_matrix_in[0], num_row * num_column * sizeof(float));

    //Compute the pseudoinverse
    compute_pseudoinverse_fcn(num_row, num_column, B_matrix[0], B_matrix_inv[0]);

    float pseudo_control_with_increment[num_row];
    //Make a copy of the pseudo control array to add the increment inside the inversion:
    memcpy(& pseudo_control_with_increment[0], & pseudo_control[0], num_row * sizeof(float));

    //Add the actual actuator position and the prioritized actuator states to the pseudo-control input through the effectiveness matrix:
    for (int k = 0; k < num_row; k++) {
        for (int i = 0; i < num_column; i++) {
            pseudo_control_with_increment[k] += (actual_u[i] - des_u[i]) * B_matrix[k][i];
        }
    }

    //Compute the initial actuation increment command by multiplying desired acceleration with the pseudo-inverse matrix
    for (int j = 0; j < num_column; j++) {
        //Cleanup previous value
        out_u[j] = 0.;
        for (int k = 0; k < num_row; k++) {
            out_u[j] += pseudo_control_with_increment[k] * B_matrix_inv[j][k];
        }
        out_u[j] += des_u[j];
        out_du[j] = out_u[j] - actual_u[j];
    }

    //Compute and print the residuals of the conventional inversion
    float residuals_conventional[num_column];
    for (int j = 0; j < num_row; j++) {
        residuals_conventional[j] = 0.f;
        for (int i = 0; i < num_column; i++) {
            residuals_conventional[j] += out_du[i] * B_matrix[j][i];
        }
        residuals_conventional[j] = pseudo_control[j] - residuals_conventional[j];
    }

}

void basic_inversion_scaled_B(float * B_matrix_in, int num_row, int num_column,float * max_u, float * min_u,
                              float * out_du, float * out_u, float * des_u, float * actual_u, float * pseudo_control){

    float B_matrix_local[num_row][num_column];
    memcpy(& B_matrix_local[0], B_matrix_in, num_row * num_column * sizeof(float));


    float pseudo_control_with_increment[num_row];
    //Make a copy of the pseudo control array to add the increment inside the inversion:
    memcpy(& pseudo_control_with_increment[0], & pseudo_control[0], num_row * sizeof(float));

    //Add the actual actuator position and the prioritized actuator states to the pseudo-control input through the effectiveness matrix:
    for (int k = 0; k < num_row; k++) {
        for (int i = 0; i < num_column; i++) {
            pseudo_control_with_increment[k] += (actual_u[i] - des_u[i]) * B_matrix[k][i];
        }
    }

    //Compute the gains for the B matrix with minimum and maximum actuator position
    float gain_motor = (max_u[0] - min_u[0])/2;
    float gain_el = (max_u[4] - min_u[4])/2;
    float gain_az = (max_u[8] - min_u[8])/2;
    float gain_aoa = (max_u[12] - min_u[12])/2;
    float gain_phi = max_u[13];


    //Scale the effectiveness matrix with the actuator gains
    float B_matrix_scaled[num_row][num_column];
    for (int j = 0; j < num_row; j++) {
        for (int i = 0; i < 4; i++) {
            B_matrix_scaled[j][i] = B_matrix_local[j][i] * gain_motor;
            B_matrix_scaled[j][i + 4] = B_matrix_local[j][i + 4] * gain_el;
            B_matrix_scaled[j][i + 8] = B_matrix_local[j][i + 8] * gain_az;
        }
        B_matrix_scaled[j][12] = B_matrix_local[j][12] * gain_aoa;
        B_matrix_scaled[j][13] = B_matrix_local[j][13] * gain_phi;
    }


    //Compute the maximum and minimum scaled actuator values:
    float max_u_scaled[num_column];
    float min_u_scaled[num_column];
    for (int i = 0; i < 4; i++) {
        max_u_scaled[i] = max_u[0] / gain_motor;
        max_u_scaled[i + 4] = max_u[4] / gain_el;
        max_u_scaled[i + 8] = max_u[8] / gain_az;
        min_u_scaled[i] = min_u[0] / gain_motor;
        min_u_scaled[i + 4] = min_u[4] / gain_el;
        min_u_scaled[i + 8] = min_u[8] / gain_az;
    }
    max_u_scaled[12] = max_u[12] / gain_aoa;
    max_u_scaled[13] = max_u[13] / gain_phi;
    min_u_scaled[12] = min_u[12] / gain_aoa;
    min_u_scaled[13] = -max_u[13] / gain_phi;

    //Compute the pseudoinverse for the scaled B matrix
    float B_matrix_inv[num_column][num_row];
    compute_pseudoinverse_fcn(num_row, num_column, B_matrix_scaled[0], B_matrix_inv[0]);


    //Compute the initial actuation increment scaled command by multiplying desired acceleration with the pseudo-inverse matrix
    float indi_u_scaled[num_column];
    for (int j = 0; j < num_column; j++) {
        //Cleanup previous value
        indi_u_scaled[j] = 0.;
        for (int k = 0; k < num_row; k++) {
            indi_u_scaled[j] += pseudo_control_with_increment[k] * B_matrix_inv[j][k];
        }
    }

    //Multiply the scaled actuator position with the gains and get the real actuator commands, also subtract the prioritized position:
    for (int i = 0; i < 4; i++) {
        out_u[i] = indi_u_scaled[i] * gain_motor;
        out_u[i+4] = indi_u_scaled[i+4] * gain_el;
        out_u[i+8] = indi_u_scaled[i+8] * gain_az;
    }
    out_u[12] = indi_u_scaled[12] * gain_aoa;
    out_u[13] = indi_u_scaled[13] * gain_phi;


    //Compute the initial actuation increment command by multiplying desired acceleration with the pseudo-inverse matrix
    for (int j = 0; j < num_column; j++) {
        out_u[j] += des_u[j];
        out_du[j] = out_u[j] - actual_u[j];
    }

}

int wls_optimization(float * B_matrix_in, int num_row, int num_column, float * max_u, float * min_u, float * out_du,
                     float * out_u, float * des_u, float * actual_u, float * pseudo_control, float * Wv, int max_iter){

    float  B_matrix[num_row][num_column];
    memcpy( & B_matrix[0], & B_matrix_in[0], num_row * num_column * sizeof(float));

    float * B_matrix_ptr[num_row];
    for (int j = 0; j < num_row; j++) {
        B_matrix_ptr[j] = & B_matrix[j][0];
    }

    float du_max[num_column];
    float du_min[num_column];
    float des_du[num_column];
    //Create the incremental problem:
    for (int j = 0; j < num_column; j++) {
        //Cleanup previous value
        du_max[j] = max_u[j] - actual_u[j];
        du_min[j] = min_u[j] - actual_u[j];
        des_du[j] = des_u[j] - actual_u[j];
    }

    int num_iter = wls_alloc(out_du, pseudo_control, du_min, du_max, B_matrix_ptr, 0, 0, Wv, 0, des_du, 10000, max_iter);

    //Compute and print the residuals of the conventional inversion
    float residuals[num_column];
    for (int j = 0; j < num_row; j++) {
        residuals[j] = 0.f;
        for (int i = 0; i < num_column; i++) {
            residuals[j] += out_du[i] * B_matrix[j][i];
        }
        residuals[j] = pseudo_control[j] - residuals[j];
    }

    //Create control input array and filter for NAN in the computed actuator set
    for (int j = 0; j < num_column; j++) {
        if(isnan(out_du[j])){
            out_du[j] = 0;
        }
        out_u[j] = out_du[j] + actual_u[j];
    }

    return num_iter;
}

/**
 * Function for the message B_MATRIX
 */
static void send_B_matrix_linear( struct transport_tx *trans , struct link_device * dev ) {

    pprz_msg_send_B_MATRIX_SEND_LINEAR(trans , dev , AC_ID ,
                           14, B_matrix[0],
                           14, B_matrix[1],
                           14, B_matrix[2]);
}
static void send_B_matrix_angular( struct transport_tx *trans , struct link_device * dev ) {

    pprz_msg_send_B_MATRIX_SEND_ANGULAR(trans , dev , AC_ID ,
                                       14, B_matrix[3],
                                       14, B_matrix[4],
                                       14, B_matrix[5]);
}

/**
 * Function for the message INDI_CMD
 */
static void send_indi_cmd( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message
    int32_t N_iter_local;
    N_iter_local = (int32_t) N_iter;

    pprz_msg_send_INDI_CMD(trans , dev , AC_ID ,
                           & INDI_acceleration_inputs[0],& INDI_acceleration_inputs[1],& INDI_acceleration_inputs[2],
                           & INDI_acceleration_inputs[3],& INDI_acceleration_inputs[4],& INDI_acceleration_inputs[5],
                           & indi_du[0],& indi_du[1],& indi_du[2],& indi_du[3],
                           & indi_du[4],& indi_du[5],& indi_du[6],& indi_du[7],
                           & indi_du[8],& indi_du[9],& indi_du[10],& indi_du[11],
                           & indi_du[12],& indi_du[13], & N_iter_local);
}

/**
 * Function for the message overactuated_variables
 */
static void send_overactuated_variables( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message

    pprz_msg_send_OVERACTUATED_VARIABLES(trans , dev , AC_ID ,
                                         & wind_speed,
                                         & pos_vect[0], & pos_vect[1], & pos_vect[2],
                                         & speed_vect[0], & speed_vect[1], & speed_vect[2],
                                         & acc_vect_filt[0], & acc_vect_filt[1], & acc_vect_filt[2],
                                         & rate_vect_filt_dot[0], & rate_vect_filt_dot[1], & rate_vect_filt_dot[2],
                                         & rate_vect_filt[0], & rate_vect_filt[1], & rate_vect_filt[2],
                                         & euler_vect[0], & euler_vect[1], & euler_vect[2],
                                         & euler_setpoint[0], & euler_setpoint[1], & euler_setpoint[2],
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
 * Initialize the filters
 */
void init_filters(void){
    float sample_time = 1.0 / PERIODIC_FREQUENCY;
    //Sensors cutoff frequency
    float tau_ang_acc = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ANG_RATES);
    float tau_lin_acc = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_LIN_ACC);
    float tau_el = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_EL);
    float tau_az = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_AZ);
    float tau_motor = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_ACTUATORS_MOTOR);

    // Initialize filters for the actuators
    for (uint8_t i = 0; i < N_ACT_REAL; i++) {
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

    // Initialize filters for the rates derivative and accelerations
    for (int i = 0; i < 3; i++) {
        init_butterworth_2_low_pass(&measurement_rates_filters[i], tau_ang_acc, sample_time, 0.0);
        init_butterworth_2_low_pass(&measurement_acc_filters[i], tau_lin_acc, sample_time, 0.0);
    }

}

/**
 * Get actuator state based on first order dynamics
 */
void get_actuator_state(void)
{
    //actuator dynamics
    for (uint8_t i = 0; i < N_ACT_REAL; i++) {

        //Motors
        if(i < 4){
            actuator_state[i] = actuator_state[i] + act_dyn[i] * (indi_u[i] - actuator_state[i]);
            Bound(actuator_state[i],OVERACTUATED_MIXING_MOTOR_MIN_OMEGA,OVERACTUATED_MIXING_MOTOR_MAX_OMEGA);
        }
        // Elevator angles
        else if(i < 8){
            actuator_state[i] = actuator_state[i] + act_dyn[i] * (indi_u[i] - actuator_state[i]);
            Bound(actuator_state[i],OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE,OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE);
        }
        //Azimuth angles
        else{
            actuator_state[i] = actuator_state[i] + act_dyn[i] * (indi_u[i] - actuator_state[i]);
            Bound(actuator_state[i],OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE,OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE);
        }

        //Propagate the actuator values into the filters and calculate the derivative
        update_butterworth_2_low_pass(&actuator_state_filters[i], actuator_state[i]);
        actuator_state_filt[i] = actuator_state_filters[i].o[0];
        actuator_state_filt_dot[i] = (actuator_state_filters[i].o[0]
                                      - actuator_state_filters[i].o[1]) * PERIODIC_FREQUENCY;
        actuator_state_filt_ddot[i] = (actuator_state_filt_dot[i]
                                      - actuator_state_filt_dot_old[i]) * PERIODIC_FREQUENCY;
        actuator_state_filt_dot_old[i] = actuator_state_filt_dot[i];
        //Add extra states (theta and phi)
        actuator_state_filt[12] = euler_vect[1];
        actuator_state_filt[13] = euler_vect[0];
    }
}

/**
 * Initialize the overactuated mixing module
 */
void overactuated_mixing_init(void) {

    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_OVERACTUATED_VARIABLES , send_overactuated_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_ACTUATORS_OUTPUT , send_actuator_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_INDI_CMD , send_indi_cmd );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_B_MATRIX_SEND_LINEAR , send_B_matrix_linear );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_B_MATRIX_SEND_ANGULAR , send_B_matrix_angular );

    //Startup the init variables of the INDI
    init_filters();

    //Init abi bind msg:
    AbiBindMsgAM7_DATA_IN(ABI_BROADCAST, &AM7_in, data_AM7_abi_in);

}

/**
 * Ad each iteration upload global variables
 */
void init_variables(void){
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
    airspeed = stateGetAirspeed_f();
    total_V = sqrt(speed_vect[0]*speed_vect[0] + speed_vect[1]*speed_vect[1] + speed_vect[2]*speed_vect[2]);
    if(total_V > 1){
        flight_path_angle = asin(-speed_vect[2]/total_V);
    }
    else{
        flight_path_angle = 0;
    }


    for (int i = 0 ; i < 4 ; i++){
        act_dyn[i] = act_dyn_struct.motor;
        act_dyn[i+4] = act_dyn_struct.elevation;
        act_dyn[i+8] = act_dyn_struct.azimuth;
    }

    /* Propagate the filter on the gyroscopes and accelerometers */
    for (int i = 0; i < 3; i++) {
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

    //Initialize actuator commands
    for(int i = 0; i < 12; i++){
        overactuated_mixing.commands[0] = 0;
    }
}

/**
 * Run the overactuated mixing
 */
void overactuated_mixing_run(pprz_t in_cmd[])
{
    //Assign variables
    uint8_t i, j, k;
    init_variables();

    /// Case of PID control as on simulink [FAILSAFE]
    if(radio_control.values[RADIO_MODE] < 500) {
//    if(0) {
        //INIT AND BOOLEAN RESET
        if (PID_engaged == 0) {
            /*
            INIT CODE FOR THE PID GOES HERE
             */
            pos_setpoint[0] = pos_vect[0];
            pos_setpoint[1] = pos_vect[1];
            pos_setpoint[2] = pos_vect[2];
            //Reset the status boolean in accordance with the actual flight state
            PID_engaged = 1;
            INDI_engaged = 0;
            FAILSAFE_engaged = 0;
        }

        ////Position error computation
        //Compute the position setpoints through slider and RC stick:
         if ( abs(radio_control.values[RADIO_PITCH]) > deadband_stick_throttle){
             pos_setpoint_body[0] += stick_gain_throttle * radio_control.values[RADIO_PITCH] * .0001;
         }
         if ( abs(radio_control.values[RADIO_ROLL]) > deadband_stick_throttle){
             pos_setpoint_body[1] += stick_gain_throttle * radio_control.values[RADIO_ROLL] * .0001;
         }
        if( abs(radio_control.values[RADIO_THROTTLE] - 4800) > deadband_stick_throttle && abs(pos_error[2]) < max_value_error.z ){
            pos_setpoint[2]  += stick_gain_throttle * (radio_control.values[RADIO_THROTTLE] - 4800) * .00001;
            Bound(pos_setpoint[2] ,-5,1000);
        }

        pos_setpoint[0] = pos_setpoint_body[0] * cos(euler_vect[2]) - pos_setpoint_body[1] * sin(euler_vect[2]);
        pos_setpoint[1] = pos_setpoint_body[0] * sin(euler_vect[2]) + pos_setpoint_body[1] * cos(euler_vect[2]);

        //Get the position setpoints through STB point:
        pos_setpoint[0] = x_stb;
        pos_setpoint[1] = y_stb;
        pos_setpoint[2] = z_stb;

        //Calculate the error on the position
        pos_error[0] = pos_setpoint[0] - pos_vect[0];
        pos_error[1] = pos_setpoint[1] - pos_vect[1];
        pos_error[2] = pos_setpoint[2] + pos_vect[2];

        //Calculate and bound the position error integration
        for (i = 0; i < 3; i++) {
            pos_error_integrated[i] += pos_error[i] / PERIODIC_FREQUENCY;
            BoundAbs(pos_error_integrated[i], OVERACTUATED_MIXING_PID_MAX_POS_ERR_INTEGRATIVE);
        }

        //Now bound the error within the defined ranges:
        BoundAbs(pos_error[0], max_value_error.x);
        BoundAbs(pos_error[1], max_value_error.y);
        BoundAbs(pos_error[2], max_value_error.z);

        //Calculate the orders with the PID gain defined:
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
        }
        else {
            if(mode_1_control){
                euler_setpoint[0] = max_value_error.phi * radio_control.values[RADIO_ROLL] / 9600;
                euler_setpoint[1] = max_value_error.theta * radio_control.values[RADIO_PITCH] / 9600;
            }
            else{
                euler_setpoint[0] = 0;
                euler_setpoint[1] = 0;
            }

        }
        //Integrate the stick yaw position to get the psi set point including psi saturation value
        if(manual_heading){
            euler_setpoint[2] = manual_heading_value_rad;
        }
        else if (abs(radio_control.values[RADIO_YAW]) > deadband_stick_yaw && fabs(euler_error[2]) < max_value_error.psi) {
            euler_setpoint[2] =
                    euler_setpoint[2] + stick_gain_yaw * radio_control.values[RADIO_YAW] * M_PI / 180 * .001;
            //Correct the setpoint in order to always be within -pi and pi
            if (euler_setpoint[2] > M_PI) {
                euler_setpoint[2] -= 2 * M_PI;
            }
            else if (euler_setpoint[2] < -M_PI) {
                euler_setpoint[2] += 2 * M_PI;
            }
        }

        //Bound the setpoints within maximum angular values
        BoundAbs(euler_setpoint[0], max_value_error.phi);
        BoundAbs(euler_setpoint[1], max_value_error.theta);

        euler_error[0] = euler_setpoint[0] - euler_vect[0];
        euler_error[1] = euler_setpoint[1] - euler_vect[1];
        euler_error[2] = euler_setpoint[2] - euler_vect[2];

        //Add logic for the psi control:
        if (euler_error[2] > M_PI) {
            euler_error[2] -= 2 * M_PI;
        }
        else if (euler_error[2] < -M_PI) {
            euler_error[2] += 2 * M_PI;
        }

        //Calculate and bound the angular error integration term for the PID
        for (i = 0; i < 3; i++) {
            euler_error_integrated[i] += euler_error[i] / PERIODIC_FREQUENCY;
            BoundAbs(euler_error_integrated[i], OVERACTUATED_MIXING_PID_MAX_EULER_ERR_INTEGRATIVE);
        }

        //Now bound the error within the defined ranges:
        BoundAbs(euler_error[2], max_value_error.psi);

        euler_order[0] = pid_gains_over.p.phi * euler_error[0] + pid_gains_over.i.phi * euler_error_integrated[0] -
                         pid_gains_over.d.phi * rate_vect[0];
        euler_order[1] = pid_gains_over.p.theta * euler_error[1] + pid_gains_over.i.theta * euler_error_integrated[1] -
                         pid_gains_over.d.theta * rate_vect[1];

        euler_order[2] = 0;
        //Compute the yaw order for the tilting system (if needed):
        if (yaw_with_tilting_PID) {
            euler_order[2] = pid_gains_over.p.psi * euler_error[2] + pid_gains_over.i.psi * euler_error_integrated[2] -
                             pid_gains_over.d.psi * rate_vect[2];
        }

        //Compute the yaw order for the differential motor (if needed)
        psi_order_motor = 0;
        if (yaw_with_motors_PID) {
            psi_order_motor = pid_gain_psi_motor.p * euler_error[2] + pid_gain_psi_motor.i * euler_error_integrated[2] -
                              pid_gain_psi_motor.d * rate_vect[2];
        }

        //Bound euler angle orders:
        BoundAbs(euler_order[0], OVERACTUATED_MIXING_PID_MAX_ROLL_ORDER_PWM);
        BoundAbs(euler_order[1], OVERACTUATED_MIXING_PID_MAX_PITCH_ORDER_PWM);
        BoundAbs(euler_order[2], OVERACTUATED_MIXING_PID_MAX_YAW_ORDER_AZ);
        BoundAbs(psi_order_motor, OVERACTUATED_MIXING_PID_MAX_YAW_ORDER_MOTOR_PWM);

        //Submit motor orders:
        if(manual_motor_stick){
            //Motor 1:
            overactuated_mixing.commands[0] = (int32_t) (( euler_order[0] + euler_order[1] + psi_order_motor)) * 9.6 + radio_control.values[RADIO_THROTTLE];
            //Motor 2:
            overactuated_mixing.commands[1] = (int32_t) ((-euler_order[0] + euler_order[1] - psi_order_motor)) * 9.6 + radio_control.values[RADIO_THROTTLE];
            //Motor 3:
            overactuated_mixing.commands[2] = (int32_t) ((-euler_order[0] - euler_order[1] + psi_order_motor)) * 9.6 + radio_control.values[RADIO_THROTTLE];
            //Motor 4:
            overactuated_mixing.commands[3] = (int32_t) (( euler_order[0] - euler_order[1] - psi_order_motor)) * 9.6 + radio_control.values[RADIO_THROTTLE];
            //Compute the command for the PPZ message
            alt_cmd = radio_control.values[RADIO_THROTTLE];
            //Also add position computation if we are in this mode:
            pos_order_body[0] = - radio_control.values[RADIO_PITCH]/K_ppz_angle_el;
            pos_order_body[1] =  radio_control.values[RADIO_ROLL]/K_ppz_angle_az;
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

        //Elevation servos:
        if (activate_tilting_el_PID || mode_1_control == 0) {
            overactuated_mixing.commands[4] = (int32_t) ((-pos_order_body[0] - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[5] = (int32_t) ((-pos_order_body[0] - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[6] = (int32_t) ((-pos_order_body[0] - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[7] = (int32_t) ((-pos_order_body[0] - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE) * K_ppz_angle_el);
        }
        else{
            overactuated_mixing.commands[4] = (int32_t) (-OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE * K_ppz_angle_el);
            overactuated_mixing.commands[5] = (int32_t) (-OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE * K_ppz_angle_el);
            overactuated_mixing.commands[6] = (int32_t) (-OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE * K_ppz_angle_el);
            overactuated_mixing.commands[7] = (int32_t) (-OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE * K_ppz_angle_el);
        }

        //Azimuth servos:
        if (activate_tilting_az_PID || mode_1_control == 0) {
            overactuated_mixing.commands[8] = (int32_t) ((pos_order_body[1] - OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[9] = (int32_t) ((pos_order_body[1] - OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[10] = (int32_t) ((pos_order_body[1] - OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[11] = (int32_t) ((pos_order_body[1] - OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE) * K_ppz_angle_az);
        }
        else{
            overactuated_mixing.commands[8] = (int32_t) (-OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE * K_ppz_angle_az);
            overactuated_mixing.commands[9] = (int32_t) (-OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE * K_ppz_angle_az);
            overactuated_mixing.commands[10] = (int32_t) (-OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE * K_ppz_angle_az);
            overactuated_mixing.commands[11] = (int32_t) (-OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE * K_ppz_angle_az);
        }

        if (yaw_with_tilting_PID) {
            overactuated_mixing.commands[8] += (int32_t) (euler_order[2] * K_ppz_angle_az);
            overactuated_mixing.commands[9] += (int32_t) (euler_order[2] * K_ppz_angle_az);
            overactuated_mixing.commands[10] -= (int32_t) (euler_order[2] * K_ppz_angle_az);
            overactuated_mixing.commands[11] -= (int32_t) (euler_order[2] * K_ppz_angle_az);
        }

        //Write the orders on the message variables:
        roll_cmd = euler_order[0] * 180 / M_PI;
        pitch_cmd = euler_order[1] * 180 / M_PI;
        yaw_motor_cmd = psi_order_motor;
        yaw_tilt_cmd = euler_order[2];
        elevation_cmd = pos_order_body[0] * 180 / M_PI;
        azimuth_cmd = pos_order_body[1] * 180 / M_PI;

    }

    /// Case of INDI control mode as on simulink
    if(radio_control.values[RADIO_MODE] > 500){
//    if(1){

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
            indi_u[0] = 100;
            indi_u[1] = 100;
            indi_u[2] = 100;
            indi_u[3] = 100;

            indi_u[4] = 0;
            indi_u[5] = 0;
            indi_u[6] = 0;
            indi_u[7] = 0;

            indi_u[8] = 0;
            indi_u[9] = 0;
            indi_u[10] = 0;
            indi_u[11] = 0;

            rate_vect_filt[0] = 0;
            rate_vect_filt[1] = 0;
            rate_vect_filt[2] = 0;

            indi_u[12] = 0;
            indi_u[13] = 0;
        }

        // Get an estimate of the actuator state using the first order dynamics given by the user
        get_actuator_state();

        //Calculate the euler angle error to be fed into the PD for the INDI loop

        float manual_motor_value = 0;
        float manual_el_value = 0;
        float manual_az_value = 0;
        float manual_phi_value = max_value_error.phi * radio_control.values[RADIO_ROLL] / 9600;
        float manual_theta_value = max_value_error.theta * radio_control.values[RADIO_PITCH] / 9600;

        euler_setpoint[0] = indi_u[13];
        euler_setpoint[1] = indi_u[12];
        //Give a specific heading value to keep
        if(manual_heading){
            euler_setpoint[2] = manual_heading_value_rad;
        }
        //Integrate the stick yaw position to get the psi set point
        else if( abs(radio_control.values[RADIO_YAW]) > deadband_stick_yaw && fabs(euler_error[2]) < max_value_error.psi){
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
//        BoundAbs(rate_setpoint[0],OVERACTUATED_MIXING_INDI_MAX_P_ORD);
//        BoundAbs(rate_setpoint[1],OVERACTUATED_MIXING_INDI_MAX_Q_ORD);
//        BoundAbs(rate_setpoint[2],OVERACTUATED_MIXING_INDI_MAX_R_ORD);

        //Compute the angular acceleration setpoint:
//        acc_setpoint[3] = (rate_setpoint[0] - rate_vect_filt[0]) * indi_gains_over.d.phi;
//        acc_setpoint[4] = (rate_setpoint[1] - rate_vect_filt[1]) * indi_gains_over.d.theta;
//        acc_setpoint[5] = (rate_setpoint[2] - rate_vect_filt[2]) * indi_gains_over.d.psi;

        acc_setpoint[3] = (rate_setpoint[0] - rate_vect_filt[0]) * indi_gains_over.d.phi;
        acc_setpoint[4] = (rate_setpoint[1] - rate_vect_filt[1]) * indi_gains_over.d.theta;
        acc_setpoint[5] = (rate_setpoint[2] - rate_vect_filt[2]) * indi_gains_over.d.psi;


//        BoundAbs(acc_setpoint[3],OVERACTUATED_MIXING_INDI_MAX_P_DOT_ORD * M_PI / 180);
//        BoundAbs(acc_setpoint[4],OVERACTUATED_MIXING_INDI_MAX_Q_DOT_ORD * M_PI / 180);
//        BoundAbs(acc_setpoint[5],OVERACTUATED_MIXING_INDI_MAX_R_DOT_ORD * M_PI / 180);

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

//        //Local testing the CA algorithms:
//        euler_vect[0] = 0; euler_vect[1] = 0; euler_vect[2] = 0;
//        actuator_state_filt[0] = 300; actuator_state_filt[1] = 300; actuator_state_filt[2] = 300; actuator_state_filt[3] = 300;
//        actuator_state_filt[4] = 0; actuator_state_filt[5] = 0; actuator_state_filt[6] = 0; actuator_state_filt[7] = 0;
//        actuator_state_filt[8] = 0; actuator_state_filt[9] = 0; actuator_state_filt[10] = 0; actuator_state_filt[11] = 0;
//        actuator_state_filt[12] = 0; actuator_state_filt[13] = 0;
//        INDI_acceleration_inputs[0] = 0;
//        INDI_acceleration_inputs[1] = 0;
//        INDI_acceleration_inputs[2] = -15;
//        INDI_acceleration_inputs[3] = 0;
//        INDI_acceleration_inputs[4] = 0;
//        INDI_acceleration_inputs[5] = 0;

        //Compute and transmit the messages to the AM7 module:
        struct am7_data_out am7_data_out_local;
        float extra_data_out_local[255];
        am7_data_out_local.motor_1_state_int = (int16_t) (actuator_state_filt[0] * 1e1);
        am7_data_out_local.motor_2_state_int = (int16_t) (actuator_state_filt[1] * 1e1);
        am7_data_out_local.motor_3_state_int = (int16_t) (actuator_state_filt[2] * 1e1);
        am7_data_out_local.motor_4_state_int = (int16_t) (actuator_state_filt[3] * 1e1);

        am7_data_out_local.el_1_state_int = (int16_t) (actuator_state_filt[4] * 1e2 * 180/M_PI);
        am7_data_out_local.el_2_state_int = (int16_t) (actuator_state_filt[5] * 1e2 * 180/M_PI);
        am7_data_out_local.el_3_state_int = (int16_t) (actuator_state_filt[6] * 1e2 * 180/M_PI);
        am7_data_out_local.el_4_state_int = (int16_t) (actuator_state_filt[7] * 1e2 * 180/M_PI);

        am7_data_out_local.az_1_state_int = (int16_t) (actuator_state_filt[8] * 1e2 * 180/M_PI);
        am7_data_out_local.az_2_state_int = (int16_t) (actuator_state_filt[9] * 1e2 * 180/M_PI);
        am7_data_out_local.az_3_state_int = (int16_t) (actuator_state_filt[10] * 1e2 * 180/M_PI);
        am7_data_out_local.az_4_state_int = (int16_t) (actuator_state_filt[11] * 1e2 * 180/M_PI);

        am7_data_out_local.phi_state_int = (int16_t) (euler_vect[0] * 1e2 * 180/M_PI);
        am7_data_out_local.theta_state_int = (int16_t) (euler_vect[1] * 1e2 * 180/M_PI);
        am7_data_out_local.psi_state_int = (int16_t) (euler_vect[2] * 1e2 * 180/M_PI);

        am7_data_out_local.gamma_state_int = (int16_t) (flight_path_angle * 1e2 * 180/M_PI);

        am7_data_out_local.p_state_int = (int16_t) (rate_vect_filt[0] * 1e1 * 180/M_PI);
        am7_data_out_local.q_state_int = (int16_t) (rate_vect_filt[1] * 1e1 * 180/M_PI);
        am7_data_out_local.r_state_int = (int16_t) (rate_vect_filt[2] * 1e1 * 180/M_PI);

        am7_data_out_local.airspeed_state_int = (int16_t) (airspeed * 1e2);

        am7_data_out_local.pseudo_control_ax_int = (int16_t) (INDI_acceleration_inputs[0] * 1e2);
        am7_data_out_local.pseudo_control_ay_int = (int16_t) (INDI_acceleration_inputs[1] * 1e2);
        am7_data_out_local.pseudo_control_az_int = (int16_t) (INDI_acceleration_inputs[2] * 1e2);
        am7_data_out_local.pseudo_control_p_dot_int = (int16_t) (INDI_acceleration_inputs[3] * 1e1 * 180/M_PI);
        am7_data_out_local.pseudo_control_q_dot_int = (int16_t) (INDI_acceleration_inputs[4] * 1e1 * 180/M_PI);
        am7_data_out_local.pseudo_control_r_dot_int = (int16_t) (INDI_acceleration_inputs[5] * 1e1 * 180/M_PI);

        am7_data_out_local.desired_motor_value_int = (int16_t) (manual_motor_value * 1e1);
        am7_data_out_local.desired_el_value_int = (int16_t) (manual_el_value * 1e2 * 180/M_PI);
        am7_data_out_local.desired_az_value_int = (int16_t) (manual_az_value * 1e2 * 180/M_PI);
        am7_data_out_local.desired_theta_value_int = (int16_t) (manual_theta_value * 1e2 * 180/M_PI);
        am7_data_out_local.desired_phi_value_int = (int16_t) (manual_phi_value * 1e2 * 180/M_PI);

        extra_data_out_local[0] = OVERACTUATED_MIXING_MOTOR_K_T_OMEGASQ;
        extra_data_out_local[1] = OVERACTUATED_MIXING_MOTOR_K_M_OMEGASQ;
        extra_data_out_local[2] = VEHICLE_MASS;
        extra_data_out_local[3] = VEHICLE_I_XX;
        extra_data_out_local[4] = VEHICLE_I_YY;
        extra_data_out_local[5] = VEHICLE_I_ZZ;
        extra_data_out_local[6] = VEHICLE_L1;
        extra_data_out_local[7] = VEHICLE_L2;
        extra_data_out_local[8] = VEHICLE_L3;
        extra_data_out_local[9] = VEHICLE_L4;
        extra_data_out_local[10] = VEHICLE_LZ;
        extra_data_out_local[11] = OVERACTUATED_MIXING_MOTOR_MAX_OMEGA;
        extra_data_out_local[12] = OVERACTUATED_MIXING_MOTOR_MIN_OMEGA;
        extra_data_out_local[13] = (OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE * 180/M_PI);
        extra_data_out_local[14] = (OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE * 180/M_PI);
        extra_data_out_local[15] = (OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE * 180/M_PI);
        extra_data_out_local[16] = (OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE * 180/M_PI);
        extra_data_out_local[17] = (OVERACTUATED_MIXING_MAX_THETA_INDI * 180/M_PI);
        extra_data_out_local[18] = (OVERACTUATED_MIXING_MIN_THETA_INDI * 180/M_PI);
        extra_data_out_local[19] = (OVERACTUATED_MIXING_MAX_AOA_INDI * 180/M_PI);
        extra_data_out_local[20] = (OVERACTUATED_MIXING_MIN_AOA_INDI * 180/M_PI);
        extra_data_out_local[21] = (OVERACTUATED_MIXING_MAX_PHI_INDI * 180/M_PI);
        extra_data_out_local[22] = VEHICLE_CM_ZERO;
        extra_data_out_local[23] = VEHICLE_CM_ALPHA;
        extra_data_out_local[24] = VEHICLE_CL_ALPHA;
        extra_data_out_local[25] = VEHICLE_CD_ZERO;
        extra_data_out_local[26] = VEHICLE_K_CD;
        extra_data_out_local[27] = VEHICLE_S;
        extra_data_out_local[28] = VEHICLE_WING_CHORD;
        extra_data_out_local[29] = 1.225; //rho value at MSL

        extra_data_out_local[30] = OVERACTUATED_MIXING_W_ACT_MOTOR_CONST;
        extra_data_out_local[31] = OVERACTUATED_MIXING_W_ACT_MOTOR_SPEED;
        extra_data_out_local[32] = OVERACTUATED_MIXING_W_ACT_EL_CONST;
        extra_data_out_local[33] = OVERACTUATED_MIXING_W_ACT_EL_SPEED;
        extra_data_out_local[34] = OVERACTUATED_MIXING_W_ACT_AZ_CONST;
        extra_data_out_local[35] = OVERACTUATED_MIXING_W_ACT_AZ_SPEED;
        extra_data_out_local[36] = OVERACTUATED_MIXING_W_ACT_THETA_CONST;
        extra_data_out_local[37] = OVERACTUATED_MIXING_W_ACT_THETA_SPEED;
        extra_data_out_local[38] = OVERACTUATED_MIXING_W_ACT_PHI_CONST;
        extra_data_out_local[39] = OVERACTUATED_MIXING_W_ACT_PHI_SPEED;

        extra_data_out_local[40] = OVERACTUATED_MIXING_W_DV_1;
        extra_data_out_local[41] = OVERACTUATED_MIXING_W_DV_2;
        extra_data_out_local[42] = OVERACTUATED_MIXING_W_DV_3;
        extra_data_out_local[43] = OVERACTUATED_MIXING_W_DV_4;
        extra_data_out_local[44] = OVERACTUATED_MIXING_W_DV_5;
        extra_data_out_local[45] = OVERACTUATED_MIXING_W_DV_6;

        extra_data_out_local[46] = OVERACTUATED_MIXING_GAMMA_QUADRATIC;

        AbiSendMsgAM7_DATA_OUT(ABI_AM7_DATA_OUT_ID, &am7_data_out_local, &extra_data_out_local);


        //Collect the last available data on the AM7 bus to be communicated to the servos.
        indi_u[0] =  (myam7_data_in_local.motor_1_cmd_int * 1e-1);
        indi_u[1] =  (myam7_data_in_local.motor_2_cmd_int * 1e-1);
        indi_u[2] =  (myam7_data_in_local.motor_3_cmd_int * 1e-1);
        indi_u[3] =  (myam7_data_in_local.motor_4_cmd_int * 1e-1);

        indi_u[4] =  (myam7_data_in_local.el_1_cmd_int * 1e-2 * M_PI/180);
        indi_u[5] =  (myam7_data_in_local.el_2_cmd_int * 1e-2 * M_PI/180);
        indi_u[6] =  (myam7_data_in_local.el_3_cmd_int * 1e-2 * M_PI/180);
        indi_u[7] =  (myam7_data_in_local.el_4_cmd_int * 1e-2 * M_PI/180);

        indi_u[8] =  (myam7_data_in_local.az_1_cmd_int * 1e-2 * M_PI/180);
        indi_u[9] =  (myam7_data_in_local.az_2_cmd_int * 1e-2 * M_PI/180);
        indi_u[10] =  (myam7_data_in_local.az_3_cmd_int * 1e-2 * M_PI/180);
        indi_u[11] =  (myam7_data_in_local.az_4_cmd_int * 1e-2 * M_PI/180);

        indi_u[12] =  (myam7_data_in_local.theta_cmd_int * 1e-2 * M_PI/180);
        indi_u[13] =  (myam7_data_in_local.phi_cmd_int * 1e-2 * M_PI/180);


        // Write values to servos and motor
        //Motors:
        overactuated_mixing.commands[0] = (int32_t) (indi_u[0] * K_ppz_rads_motor);
        overactuated_mixing.commands[1] = (int32_t) (indi_u[1] * K_ppz_rads_motor);
        overactuated_mixing.commands[2] = (int32_t) (indi_u[2] * K_ppz_rads_motor);
        overactuated_mixing.commands[3] = (int32_t) (indi_u[3] * K_ppz_rads_motor);

        //Elevator servos:
        overactuated_mixing.commands[4] = (int32_t) ( (indi_u[4] - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE) * K_ppz_angle_el );
        overactuated_mixing.commands[5] = (int32_t) ( (indi_u[5] - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE) * K_ppz_angle_el );
        overactuated_mixing.commands[6] = (int32_t) ( (indi_u[6] - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE) * K_ppz_angle_el );
        overactuated_mixing.commands[7] = (int32_t) ( (indi_u[7] - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE) * K_ppz_angle_el );

        //Azimuth servos:
        overactuated_mixing.commands[8] = (int32_t) (indi_u[8] * K_ppz_angle_az);
        overactuated_mixing.commands[9] = (int32_t) (indi_u[9] * K_ppz_angle_az);
        overactuated_mixing.commands[10] = (int32_t) (indi_u[10] * K_ppz_angle_az);
        overactuated_mixing.commands[11] = (int32_t) (indi_u[11] * K_ppz_angle_az);

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

}

