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
 * @author Alessandro Mancinelli (a.mancinelli@tudelft.nl)
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
#include "mcu_periph/sys_time.h"

/**
 * Variables declaration
 */

// Variable for the autonomous maneuver in Cyberzoo to compare different CA algorithm
int start_auto = 0;
float start_time_auto = 0;
int32_t bool_start_auto_on = 0;

float ax_point = -2;
float ay_point = 0;
float bx_point = -2;
float by_point = -0.5;
float cx_point = 1.5;
float cy_point = 0.5;
float ox_point = 0;
float oy_point = 0;
float z_test = 1.2;
float z_end = 3.2;
float pitch_angle_test = 20;
float roll_angle_test = 20;
int controller_id = 1;

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
float psi_order_motor = 0;

//Flight states variables:
bool INDI_engaged = 0, FAILSAFE_engaged = 0, PID_engaged = 0;

// PID and general settings from slider
int deadband_stick_yaw = 500, deadband_stick_throttle = 2500;
float stick_gain_yaw = 0.01, stick_gain_throttle = 0.03; //  Stick to yaw and throttle gain (for the integral part)
bool yaw_with_motors_PID = 0, position_with_attitude = 0, manual_motor_stick = 1, activate_tilting_az_PID = 0;
bool activate_tilting_el_PID = 0, yaw_with_tilting_PID = 1, mode_1_control = 0;

bool manual_heading = 0;
int manual_heading_value_rad = 0;

float wind_speed = 0;

float alt_cmd = 0, pitch_cmd = 0, roll_cmd = 0, yaw_motor_cmd = 0, yaw_tilt_cmd = 0, elevation_cmd = 0, azimuth_cmd = 0;

// Actuators gains:
float K_ppz_rads_motor = 9.6 / OVERACTUATED_MIXING_MOTOR_K_PWM_OMEGA;
float K_ppz_angle_el = (9600 * 2) / (OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE);
float K_ppz_angle_az = (9600 * 2) / (OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE);

// Priotirized actuator states with extra theta and phi states 
float prioritized_actuator_states[INDI_NUM_ACT] = {0, 0, 0, 0,
                                                   0, 0, 0, 0,
                                                   0, 0, 0, 0,
                                                   0, 0};
//Incremental INDI variables
float indi_du[INDI_NUM_ACT], indi_u[INDI_NUM_ACT], indi_u_scaled[INDI_NUM_ACT];

//Variables for the actuator model v2:
#define actuator_mem_buf_size 10
float indi_u_memory[INDI_NUM_ACT][actuator_mem_buf_size];
float actuator_state_old[INDI_NUM_ACT];
float actuator_state_old_old[INDI_NUM_ACT];
int delay_ts_motor = (int) (OVERACTUATED_MIXING_INDI_MOTOR_FIRST_ORD_DELAY * PERIODIC_FREQUENCY);
int delay_ts_az = (int) (OVERACTUATED_MIXING_INDI_AZ_SECOND_ORD_DELAY * PERIODIC_FREQUENCY);
int delay_ts_el = (int) (OVERACTUATED_MIXING_INDI_EL_SECOND_ORD_DELAY * PERIODIC_FREQUENCY);
float max_rate_az = OVERACTUATED_MIXING_INDI_AZ_SECOND_ORD_RATE_LIMIT / PERIODIC_FREQUENCY;
float max_rate_el = OVERACTUATED_MIXING_INDI_EL_SECOND_ORD_RATE_LIMIT / PERIODIC_FREQUENCY;

//Matrix for the coordinate transformation from euler angle derivative to rates:
float R_matrix[3][3];

//Setpoints and pseudocontrol
float pos_setpoint[3];
float speed_setpoint[3];
float euler_setpoint[3];
float rate_setpoint[3];
float acc_setpoint[6];
float INDI_pseudocontrol[INDI_INPUTS];

float B_matrix[INDI_INPUTS][INDI_NUM_ACT];

//AM7 variables:
float manual_motor_value = 150, manual_el_value = 0, manual_az_value = 0, manual_phi_value = 0, manual_theta_value = 0;
struct am7_data_out am7_data_out_local;
float extra_data_out_local[255];
static abi_event AM7_in;
float extra_data_in_local[255];
struct am7_data_in myam7_data_in_local;

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
        } };
struct PID_over_simple pid_gain_psi_motor = {
        .p = OVERACTUATED_MIXING_PID_P_GAIN_PSI_MOTOR,
        .i = OVERACTUATED_MIXING_PID_I_GAIN_PSI_MOTOR,
        .d = OVERACTUATED_MIXING_PID_D_GAIN_PSI_MOTOR };
struct PID_over_simple pid_pos_x_att = {
        .p = OVERACTUATED_MIXING_PID_P_GAIN_POS_X_ATT,
        .i = OVERACTUATED_MIXING_PID_I_GAIN_POS_X_ATT,
        .d = OVERACTUATED_MIXING_PID_D_GAIN_POS_X_ATT };
struct PID_over_simple pid_pos_y_att = {
        .p = OVERACTUATED_MIXING_PID_P_GAIN_POS_Y_ATT,
        .i = OVERACTUATED_MIXING_PID_I_GAIN_POS_Y_ATT,
        .d = OVERACTUATED_MIXING_PID_D_GAIN_POS_Y_ATT };
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
        } };
struct FloatEulers max_value_error = {
        OVERACTUATED_MIXING_MAX_PHI,
        OVERACTUATED_MIXING_MAX_THETA,
        OVERACTUATED_MIXING_MAX_PSI_ERR };
struct ActuatorsStruct act_dyn_struct = {
        OVERACTUATED_MIXING_INDI_ACT_DYN_MOTOR,
        OVERACTUATED_MIXING_INDI_ACT_DYN_EL,
        OVERACTUATED_MIXING_INDI_ACT_DYN_AZ };

// Variables needed for the actuators:
float act_dyn[N_ACT_REAL];

static void data_AM7_abi_in(uint8_t sender_id __attribute__((unused)), struct am7_data_in * myam7_data_in_ptr, float * extra_data_in_ptr){
    memcpy(&myam7_data_in_local,myam7_data_in_ptr,sizeof(struct am7_data_in));
    memcpy(&extra_data_in_local,extra_data_in_ptr,255 * sizeof(float));
}

/**
 * Function for the message INDI_CMD
 */
static void send_indi_cmd( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message

    pprz_msg_send_INDI_CMD(trans , dev , AC_ID ,
                           & INDI_pseudocontrol[0],& INDI_pseudocontrol[1],& INDI_pseudocontrol[2],
                           & INDI_pseudocontrol[3],& INDI_pseudocontrol[4],& INDI_pseudocontrol[5],
                           & indi_du[0],& indi_du[1],& indi_du[2],& indi_du[3],
                           & indi_du[4],& indi_du[5],& indi_du[6],& indi_du[7],
                           & indi_du[8],& indi_du[9],& indi_du[10],& indi_du[11],
                           & indi_du[12],& indi_du[13]);
}

/**
 * Function for the message overactuated_variables
 */
static void send_overactuated_variables( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message

    pprz_msg_send_OVERACTUATED_VARIABLES(trans , dev , AC_ID ,
                                         & bool_start_auto_on,
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

    //Initialize to zero the variables of get_actuator_state_v2:
    for(int i = 0; i < INDI_NUM_ACT; i++){
        for(int j = 0; j < actuator_mem_buf_size; j++ ){
            indi_u_memory[i][j] = 0;
        }
        actuator_state_old_old[i] = 0;
        actuator_state_old[i] = 0;
    }
}

/**
 * Get actuator state based on first order dynamics
 */
void get_actuator_state(void)
{
    //actuator dynamics
    for (int i = 0; i < N_ACT_REAL; i++) {

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
 * Get actuator state based on second order dynamics
 */
void get_actuator_state_v2(void)
{
    //actuator dynamics
    for (int i = 0; i < N_ACT_REAL; i++) {

        //Motors
        if(i < 4){
            actuator_state[i] = - OVERACTUATED_MIXING_INDI_MOTOR_FIRST_ORD_DEN_2 * actuator_state_old[i] +
                    OVERACTUATED_MIXING_INDI_MOTOR_FIRST_ORD_NUM_2 * indi_u_memory[i][actuator_mem_buf_size - delay_ts_motor - 1];
            Bound(actuator_state[i],OVERACTUATED_MIXING_MOTOR_MIN_OMEGA,OVERACTUATED_MIXING_MOTOR_MAX_OMEGA);
        }
        // Elevator angles
        else if(i < 8){
            actuator_state[i] = - OVERACTUATED_MIXING_INDI_EL_SECOND_ORD_DEN_2 * actuator_state_old[i] -
                                  OVERACTUATED_MIXING_INDI_EL_SECOND_ORD_DEN_3 * actuator_state_old_old[i] +
                                  OVERACTUATED_MIXING_INDI_EL_SECOND_ORD_NUM_2 * indi_u_memory[i][actuator_mem_buf_size - delay_ts_el - 1] +
                                  OVERACTUATED_MIXING_INDI_EL_SECOND_ORD_NUM_3 * indi_u_memory[i][actuator_mem_buf_size - delay_ts_el - 2];
            //Apply saturation
            if(actuator_state[i] - actuator_state_old[i] > max_rate_el){
                actuator_state[i] = actuator_state_old[i] + max_rate_el;
            } else if(actuator_state[i] - actuator_state_old[i] < -max_rate_el){
                actuator_state[i] = actuator_state_old[i] - max_rate_el;
            }
            // Bound for max and minimum physical values:
            Bound(actuator_state[i],OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE,OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE);
        }
        //Azimuth angles
        else{
            actuator_state[i] = - OVERACTUATED_MIXING_INDI_AZ_SECOND_ORD_DEN_2 * actuator_state_old[i] -
                                OVERACTUATED_MIXING_INDI_AZ_SECOND_ORD_DEN_3 * actuator_state_old_old[i] +
                                OVERACTUATED_MIXING_INDI_AZ_SECOND_ORD_NUM_2 * indi_u_memory[i][actuator_mem_buf_size - delay_ts_az - 1] +
                                OVERACTUATED_MIXING_INDI_AZ_SECOND_ORD_NUM_3 * indi_u_memory[i][actuator_mem_buf_size - delay_ts_az - 2];
            //Apply saturation
            if(actuator_state[i] - actuator_state_old[i] > max_rate_az){
                actuator_state[i] = actuator_state_old[i] + max_rate_az;
            } else if(actuator_state[i] - actuator_state_old[i] < -max_rate_az){
                actuator_state[i] = actuator_state_old[i] - max_rate_az;
            }
            // Bound for max and minimum physical values:
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

        //Assign the memory variables:
        actuator_state_old_old[i] = actuator_state_old[i];
        actuator_state_old[i] = actuator_state[i];
        for (int j = 1; j < actuator_mem_buf_size ; j++){
            indi_u_memory[i][j-1] = indi_u_memory[i][j];
        }
        indi_u_memory[i][actuator_mem_buf_size-1] = indi_u[i];

    }
}

/**
 * Initialize the overactuated mixing module
 */
void overactuated_mixing_init(void) {

    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_OVERACTUATED_VARIABLES , send_overactuated_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_ACTUATORS_OUTPUT , send_actuator_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_INDI_CMD , send_indi_cmd );

    //Startup the init variables of the INDI
    init_filters();

    //Init abi bind msg:
    AbiBindMsgAM7_DATA_IN(ABI_BROADCAST, &AM7_in, data_AM7_abi_in);

}

/**
 * Ad each iteration upload global variables
 */
void assign_variables(void){
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
    if(total_V > 2){
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
        
        acc_vect_filt[i] = measurement_acc_filters[i].o[0];

        // rate_vect_filt[i] = measurement_rates_filters[i].o[0];

        //Use first order filter for the rates: 
        rate_vect_filt[i] = rate_vect_filt[i] + OVERACTUATED_MIXING_FIRST_ORDER_FILTER_COEFF_ANG_RATES * (rate_vect[i] - rate_vect_filt[i]);
    }

    //Computation of the matrix to pass from euler to body rates
    R_matrix[0][0]=1;
    R_matrix[1][0]=0;
    R_matrix[2][0]=-sin(euler_vect[1]);
    R_matrix[0][1]=0;
    R_matrix[1][1]=cos(euler_vect[0]);
    R_matrix[2][1]=sin(euler_vect[0]) * cos(euler_vect[1]);
    R_matrix[0][2]=0;
    R_matrix[1][2]=-sin(euler_vect[0]);
    R_matrix[2][2]=cos(euler_vect[0]) * cos(euler_vect[1]);

    //Initialize actuator commands
    for(int i = 0; i < 12; i++){
        overactuated_mixing.commands[0] = 0;
    }
}

/**
 * Run the overactuated mixing
 */
void overactuated_mixing_run()
{
    //Assign variables
    assign_variables();

    pos_setpoint[0] = ox_point;
    pos_setpoint[1] = oy_point;
    pos_setpoint[2] = - z_test;
    manual_theta_value = 0;
    manual_phi_value = 0;
    euler_setpoint[2] = 0;

    if(start_auto){
        if(bool_start_auto_on == 0) {
            start_time_auto = get_sys_time_float();
            bool_start_auto_on = 1;
        }
        //Switch case for the position setpoints:
        if(get_sys_time_float() - start_time_auto <= 5){
            pos_setpoint[0] = 0;
            pos_setpoint[1] = 0;
            pos_setpoint[2] = - z_test;
            manual_theta_value = 0;
            manual_phi_value = 0;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 5 && get_sys_time_float() - start_time_auto <= 10){
            pos_setpoint[0] = ax_point;
            pos_setpoint[1] = ay_point;
            pos_setpoint[2] = - z_test;
            manual_theta_value = 0;
            manual_phi_value = 0;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 10 && get_sys_time_float() - start_time_auto <= 15){
            pos_setpoint[0] = bx_point;
            pos_setpoint[1] = by_point;
            pos_setpoint[2] = - z_test;
            manual_theta_value = 0;
            manual_phi_value = 0;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 15 && get_sys_time_float() - start_time_auto <= 20){
            pos_setpoint[0] = bx_point;
            pos_setpoint[1] = by_point;
            pos_setpoint[2] = - z_test;
            manual_theta_value = pitch_angle_test * M_PI/180;
            manual_phi_value = roll_angle_test * M_PI/180;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 20 && get_sys_time_float() - start_time_auto <= 25){
            pos_setpoint[0] = cx_point;
            pos_setpoint[1] = cy_point;
            pos_setpoint[2] = - z_end;
            manual_theta_value = pitch_angle_test * M_PI/180;
            manual_phi_value = roll_angle_test * M_PI/180;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 25 && get_sys_time_float() - start_time_auto <= 30){
            pos_setpoint[0] = cx_point;
            pos_setpoint[1] = cy_point;
            pos_setpoint[2] = - z_end;
            manual_theta_value = pitch_angle_test * M_PI/180;
            manual_phi_value = roll_angle_test * M_PI/180;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 30 && get_sys_time_float() - start_time_auto <= 35){
            pos_setpoint[0] = cx_point;
            pos_setpoint[1] = cy_point;
            pos_setpoint[2] = - z_end;
            manual_theta_value = 0;
            manual_phi_value = 0;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 35 && get_sys_time_float() - start_time_auto <= 40){
            pos_setpoint[0] = cx_point;
            pos_setpoint[1] = cy_point;
            pos_setpoint[2] = - z_end;
            manual_theta_value = 0;
            manual_phi_value = 0;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 40 && get_sys_time_float() - start_time_auto <= 45){
            pos_setpoint[0] = cx_point;
            pos_setpoint[1] = cy_point;
            pos_setpoint[2] = - z_end;
            manual_theta_value = 0;
            manual_phi_value = 0;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 45 && get_sys_time_float() - start_time_auto <= 55){
            pos_setpoint[0] = cx_point;
            pos_setpoint[1] = cy_point;
            pos_setpoint[2] = - z_end;
            manual_theta_value = 0;
            manual_phi_value = 0;
            euler_setpoint[2] = 0;
        }
        if(get_sys_time_float() - start_time_auto > 55 ){
            pos_setpoint[0] = cx_point;
            pos_setpoint[1] = cy_point;
            pos_setpoint[2] = - z_end;
            manual_theta_value = 0;
            manual_phi_value = 0;
            euler_setpoint[2] = 0;
        }
    } else{
        bool_start_auto_on = 0;
    }


    /// Case of manual PID control [FAILSAFE]
    if(radio_control.values[RADIO_MODE] < -500) {

        //INIT AND BOOLEAN RESET
        if(FAILSAFE_engaged == 0 ){
            PID_engaged = 0;
            INDI_engaged = 0;
            FAILSAFE_engaged = 1;
            for (int i = 0; i < 3; i++) {
                euler_error_integrated[i] = 0;
                pos_error_integrated[i] = 0;
            }
        }

        ////Angular error computation
        //Calculate the setpoints manually:
        euler_setpoint[0] = 0;
        euler_setpoint[1] = 0;
        if (abs(radio_control.values[RADIO_YAW]) > deadband_stick_yaw ) {
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
        for (int i = 0; i < 3; i++) {
            euler_error_integrated[i] += euler_error[i] / PERIODIC_FREQUENCY;
            BoundAbs(euler_error_integrated[i], OVERACTUATED_MIXING_PID_MAX_EULER_ERR_INTEGRATIVE);
        }

        euler_order[0] = pid_gains_over.p.phi * euler_error[0] + pid_gains_over.i.phi * euler_error_integrated[0] -
                         pid_gains_over.d.phi * rate_vect[0];
        euler_order[1] = pid_gains_over.p.theta * euler_error[1] + pid_gains_over.i.theta * euler_error_integrated[1] -
                         pid_gains_over.d.theta * rate_vect[1];
        euler_order[2] = pid_gains_over.p.psi * euler_error[2] + pid_gains_over.i.psi * euler_error_integrated[2] -
                         pid_gains_over.d.psi * rate_vect[2];

        //Bound euler angle orders:
        BoundAbs(euler_order[0], OVERACTUATED_MIXING_PID_MAX_ROLL_ORDER_PWM);
        BoundAbs(euler_order[1], OVERACTUATED_MIXING_PID_MAX_PITCH_ORDER_PWM);
        BoundAbs(euler_order[2], OVERACTUATED_MIXING_PID_MAX_YAW_ORDER_AZ);

        //Submit motor orders:
        overactuated_mixing.commands[0] = (int32_t) (( euler_order[0] + euler_order[1])) * 9.6 + radio_control.values[RADIO_THROTTLE];
        overactuated_mixing.commands[1] = (int32_t) ((-euler_order[0] + euler_order[1])) * 9.6 + radio_control.values[RADIO_THROTTLE];
        overactuated_mixing.commands[2] = (int32_t) ((-euler_order[0] - euler_order[1])) * 9.6 + radio_control.values[RADIO_THROTTLE];
        overactuated_mixing.commands[3] = (int32_t) (( euler_order[0] - euler_order[1])) * 9.6 + radio_control.values[RADIO_THROTTLE];

        //Submit servo elevation orders:
        overactuated_mixing.commands[4] = (int32_t) ((radio_control.values[RADIO_PITCH]/K_ppz_angle_el - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE) * K_ppz_angle_el);
        overactuated_mixing.commands[5] = (int32_t) ((radio_control.values[RADIO_PITCH]/K_ppz_angle_el - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE) * K_ppz_angle_el);
        overactuated_mixing.commands[6] = (int32_t) ((radio_control.values[RADIO_PITCH]/K_ppz_angle_el - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE) * K_ppz_angle_el);
        overactuated_mixing.commands[7] = (int32_t) ((radio_control.values[RADIO_PITCH]/K_ppz_angle_el - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE) * K_ppz_angle_el);

        //Submit servo azimuth orders:
        overactuated_mixing.commands[8] = (int32_t) ((radio_control.values[RADIO_ROLL]/K_ppz_angle_az - OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE) * K_ppz_angle_az);
        overactuated_mixing.commands[9] = (int32_t) ((radio_control.values[RADIO_ROLL]/K_ppz_angle_az - OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE) * K_ppz_angle_az);
        overactuated_mixing.commands[10] = (int32_t) ((radio_control.values[RADIO_ROLL]/K_ppz_angle_az - OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE) * K_ppz_angle_az);
        overactuated_mixing.commands[11] = (int32_t) ((radio_control.values[RADIO_ROLL]/K_ppz_angle_az - OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE) * K_ppz_angle_az);

        //Add yaw commands on top of the azimuth commands.
        overactuated_mixing.commands[8] += (int32_t) (euler_order[2] * K_ppz_angle_az);
        overactuated_mixing.commands[9] += (int32_t) (euler_order[2] * K_ppz_angle_az);
        overactuated_mixing.commands[10] -= (int32_t) (euler_order[2] * K_ppz_angle_az);
        overactuated_mixing.commands[11] -= (int32_t) (euler_order[2] * K_ppz_angle_az);

    }

    /// Case of auto PID control
    if(radio_control.values[RADIO_MODE] < 500 && radio_control.values[RADIO_MODE] > -500 )
//    if(1)
    {
        //INIT AND BOOLEAN RESET
        if(PID_engaged == 0 ){
            PID_engaged = 1;
            INDI_engaged = 0;
            FAILSAFE_engaged = 0;
            for (int i = 0; i < 3; i++) {
                euler_error_integrated[i] = 0;
                pos_error_integrated[i] = 0;
            }
        }

        ////Position error computation
        pos_error[0] = pos_setpoint[0] - pos_vect[0];
        pos_error[1] = pos_setpoint[1] - pos_vect[1];
        pos_error[2] = -pos_setpoint[2] + pos_vect[2];

        //Calculate the position error integration
        for (int i = 0; i < 3; i++) {
            if(radio_control.values[RADIO_TH_HOLD] < - 4800) {
                pos_error_integrated[i] += pos_error[i] / PERIODIC_FREQUENCY;
            }
        }

        //Calculate the orders with the PID gain defined:
        pos_order_earth[0] = pid_gains_over.p.x * pos_error[0] + pid_gains_over.i.x * pos_error_integrated[0] -
                             pid_gains_over.d.x * speed_vect[0];
        pos_order_earth[1] = pid_gains_over.p.y * pos_error[1] + pid_gains_over.i.y * pos_error_integrated[1] -
                             pid_gains_over.d.y * speed_vect[1];
        pos_order_earth[2] = pid_gains_over.p.z * pos_error[2] + pid_gains_over.i.z * pos_error_integrated[2] +
                             pid_gains_over.d.z * speed_vect[2];

        //Transpose the position errors in the body frame:
        pos_order_body[0] = cos(euler_vect[2]) * pos_order_earth[0] + sin(euler_vect[2]) * pos_order_earth[1];
        pos_order_body[1] = cos(euler_vect[2]) * pos_order_earth[1] - sin(euler_vect[2]) * pos_order_earth[0];
        pos_order_body[2] = pos_order_earth[2];

        ////Angular error computation
        euler_setpoint[0] = manual_phi_value;
        euler_setpoint[1] = manual_theta_value;
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
        for (int i = 0; i < 3; i++) {
            if(radio_control.values[RADIO_TH_HOLD] < - 4800) {
                euler_error_integrated[i] += euler_error[i] / PERIODIC_FREQUENCY;
            }
        }

        //Now bound the error within the defined ranges:
        BoundAbs(euler_error[2], max_value_error.psi);

        euler_order[0] = pid_gains_over.p.phi * euler_error[0] + pid_gains_over.i.phi * euler_error_integrated[0] -
                         pid_gains_over.d.phi * rate_vect[0];
        euler_order[1] = pid_gains_over.p.theta * euler_error[1] + pid_gains_over.i.theta * euler_error_integrated[1] -
                         pid_gains_over.d.theta * rate_vect[1];
        euler_order[2] = pid_gains_over.p.psi * euler_error[2] + pid_gains_over.i.psi * euler_error_integrated[2] -
                         pid_gains_over.d.psi * rate_vect[2];

        //Bound euler angle orders and servos max angle:
        BoundAbs(euler_order[0], OVERACTUATED_MIXING_PID_MAX_ROLL_ORDER_PWM);
        BoundAbs(euler_order[1], OVERACTUATED_MIXING_PID_MAX_PITCH_ORDER_PWM);
        BoundAbs(euler_order[2], OVERACTUATED_MIXING_PID_MAX_YAW_ORDER_AZ);
        BoundAbs(pos_order_body[0], OVERACTUATED_MIXING_PID_MAX_EL_ORDER);
        BoundAbs(pos_order_body[1], OVERACTUATED_MIXING_PID_MAX_AZ_ORDER);

        float alt_order = pos_order_body[2] + OVERACTUATED_MIXING_PID_THR_NEUTRAL_PWM - 1000;
        Bound(alt_order,0,1000);
        //Submit motor orders:
        overactuated_mixing.commands[0] =
                (int32_t) (alt_order + euler_order[0] + euler_order[1]) * 9.6;
        overactuated_mixing.commands[1] =
                (int32_t) (alt_order - euler_order[0] + euler_order[1]) * 9.6;
        overactuated_mixing.commands[2] =
                (int32_t) (alt_order - euler_order[0] - euler_order[1]) * 9.6;
        overactuated_mixing.commands[3] =
                (int32_t) (alt_order + euler_order[0] - euler_order[1]) * 9.6;

        //Elevation servos:
        overactuated_mixing.commands[4] = (int32_t) ((-pos_order_body[0] - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE) * K_ppz_angle_el);
        overactuated_mixing.commands[5] = (int32_t) ((-pos_order_body[0] - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE) * K_ppz_angle_el);
        overactuated_mixing.commands[6] = (int32_t) ((-pos_order_body[0] - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE) * K_ppz_angle_el);
        overactuated_mixing.commands[7] = (int32_t) ((-pos_order_body[0] - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE) * K_ppz_angle_el);

        //Azimuth servos:
        overactuated_mixing.commands[8] = (int32_t) ((pos_order_body[1] - OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE) * K_ppz_angle_az);
        overactuated_mixing.commands[9] = (int32_t) ((pos_order_body[1] - OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE) * K_ppz_angle_az);
        overactuated_mixing.commands[10] = (int32_t) ((pos_order_body[1] - OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE) * K_ppz_angle_az);
        overactuated_mixing.commands[11] = (int32_t) ((pos_order_body[1] - OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE) * K_ppz_angle_az);

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

    /// Case of INDI control mode with external am7 function:
    if(radio_control.values[RADIO_MODE] > 500 )
//    if(1)
    {

        //INIT AND BOOLEAN RESET
        if(INDI_engaged == 0 ){
            /*
            INIT CODE FOR THE INDI GOES HERE
             */

            actuator_state_filt[0] = 100;
            actuator_state_filt[1] = 100;
            actuator_state_filt[2] = 100;
            actuator_state_filt[3] = 100;

            actuator_state_filt[4] = 0;
            actuator_state_filt[5] = 0;
            actuator_state_filt[6] = 0;
            actuator_state_filt[7] = 0;

            actuator_state_filt[8] = 0;
            actuator_state_filt[9] = 0;
            actuator_state_filt[10] = 0;
            actuator_state_filt[11] = 0;

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
        }

//        // Get an estimate of the actuator state using the first order dynamics given by the user
//        get_actuator_state();

        // Get an estimate of the actuator state using the second order dynamics
        get_actuator_state_v2();

        //Calculate the euler angle error to be fed into the PD for the INDI loop

//        manual_phi_value = max_value_error.phi * radio_control.values[RADIO_ROLL] / 9600;
//        manual_theta_value = max_value_error.theta * radio_control.values[RADIO_PITCH] / 9600;

        euler_setpoint[0] = manual_phi_value;
        euler_setpoint[1] = manual_theta_value;
        //Give a specific heading value to keep
        if(manual_heading){
            euler_setpoint[2] = manual_heading_value_rad;
        }

        if(euler_setpoint[2] > M_PI){
            euler_setpoint[2] -= 2 * M_PI;
        }
        else if(euler_setpoint[2] < - M_PI){
            euler_setpoint[2] += 2 * M_PI;
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
        for (int j = 0; j < 3; j++) {
            //Cleanup previous value
            angular_body_error[j] = 0.;
            for (int k = 0; k < 3; k++) {
                angular_body_error[j] += euler_error[k] * R_matrix[k][j];
            }
        }

        rate_setpoint[0] = angular_body_error[0] * indi_gains_over.p.phi;
        rate_setpoint[1] = angular_body_error[1] * indi_gains_over.p.theta;
        rate_setpoint[2] = angular_body_error[2] * indi_gains_over.p.psi;

        //Compute the angular acceleration setpoint:
        acc_setpoint[3] = (rate_setpoint[0] - rate_vect_filt[0]) * indi_gains_over.d.phi;
        acc_setpoint[4] = (rate_setpoint[1] - rate_vect_filt[1]) * indi_gains_over.d.theta;
        acc_setpoint[5] = (rate_setpoint[2] - rate_vect_filt[2]) * indi_gains_over.d.psi;


        //Compute the acceleration error and save it to the INDI input array in the right position:
        // ANGULAR ACCELERATION
        INDI_pseudocontrol[3] = acc_setpoint[3] - rate_vect_filt_dot[0];
        INDI_pseudocontrol[4] = acc_setpoint[4] - rate_vect_filt_dot[1];
        INDI_pseudocontrol[5] = acc_setpoint[5] - rate_vect_filt_dot[2];

        //Calculate the position error to be fed into the PD for the INDI loop

        pos_error[0] = pos_setpoint[0] - pos_vect[0];
        pos_error[1] = pos_setpoint[1] - pos_vect[1];
        pos_error[2] = pos_setpoint[2] - pos_vect[2];

        //Compute the speed setpoint
        speed_setpoint[0] = pos_error[0] * indi_gains_over.p.x;
        speed_setpoint[1] = pos_error[1] * indi_gains_over.p.y;
        speed_setpoint[2] = pos_error[2] * indi_gains_over.p.z;

        //Compute the linear accel setpoint
        acc_setpoint[0] = (speed_setpoint[0] - speed_vect[0]) * indi_gains_over.d.x;
        acc_setpoint[1] = (speed_setpoint[1] - speed_vect[1]) * indi_gains_over.d.y;
        acc_setpoint[2] = (speed_setpoint[2] - speed_vect[2]) * indi_gains_over.d.z;

        //Compute the acceleration error and save it to the INDI input array in the right position:
        // LINEAR ACCELERATION
        INDI_pseudocontrol[0] = acc_setpoint[0] - acc_vect_filt[0];
        INDI_pseudocontrol[1] = acc_setpoint[1] - acc_vect_filt[1];
        INDI_pseudocontrol[2] = acc_setpoint[2] - acc_vect_filt[2];

//        actuator_state_filt[0] = 100;
//        actuator_state_filt[1] = 100;
//        actuator_state_filt[2] = 100;
//        actuator_state_filt[3] = 100;
//
//        actuator_state_filt[4] = 0;
//        actuator_state_filt[5] = 0;
//        actuator_state_filt[6] = 0;
//        actuator_state_filt[7] = 0;
//
//        actuator_state_filt[8] = 0;
//        actuator_state_filt[9] = 0;
//        actuator_state_filt[10] = 0;
//        actuator_state_filt[11] = 0;

        //Compute and transmit the messages to the AM7 module:
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

        am7_data_out_local.pseudo_control_ax_int = (int16_t) (INDI_pseudocontrol[0] * 1e2);
        am7_data_out_local.pseudo_control_ay_int = (int16_t) (INDI_pseudocontrol[1] * 1e2);
        am7_data_out_local.pseudo_control_az_int = (int16_t) (INDI_pseudocontrol[2] * 1e2);
        am7_data_out_local.pseudo_control_p_dot_int = (int16_t) (INDI_pseudocontrol[3] * 1e1 * 180/M_PI);
        am7_data_out_local.pseudo_control_q_dot_int = (int16_t) (INDI_pseudocontrol[4] * 1e1 * 180/M_PI);
        am7_data_out_local.pseudo_control_r_dot_int = (int16_t) (INDI_pseudocontrol[5] * 1e1 * 180/M_PI);


        am7_data_out_local.desired_motor_value_int = (int16_t) (manual_motor_value * 1e1);
        am7_data_out_local.desired_el_value_int = (int16_t) (manual_el_value * 1e2 * 180/M_PI);
        am7_data_out_local.desired_az_value_int = (int16_t) (manual_az_value * 1e2 * 180/M_PI);
        am7_data_out_local.desired_theta_value_int = (int16_t) (manual_theta_value * 1e2 * 180/M_PI);
        am7_data_out_local.desired_phi_value_int = (int16_t) (manual_phi_value * 1e2 * 180/M_PI);

        // Hardcoding the maximum elevator angle into the optimizer
        float min_el_angle_constrained = -90; //Degrees

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

//        extra_data_out_local[14] = (OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE * 180/M_PI);
        extra_data_out_local[14] = (min_el_angle_constrained);

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

        extra_data_out_local[46] = OVERACTUATED_MIXING_GAMMA_QUADRATIC_DV;
        extra_data_out_local[47] = OVERACTUATED_MIXING_GAMMA_QUADRATIC_DU;
        extra_data_out_local[48] = OVERACTUATED_MIXING_GAMMA_QUADRATIC_WLS;
        extra_data_out_local[49] = OVERACTUATED_MIXING_MAX_ITER_WLS;

        extra_data_out_local[50] = controller_id;

        //Collect the last available data on the AM7 bus to be communicated to the servos.
        AbiSendMsgAM7_DATA_OUT(ABI_AM7_DATA_OUT_ID, &am7_data_out_local, extra_data_out_local);

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


        // Write values to servos and motor

        if(RadioControlValues(RADIO_TH_HOLD) > - 4500) {
            //Motors:
            overactuated_mixing.commands[0] = (int32_t)(indi_u[0] * K_ppz_rads_motor);
            overactuated_mixing.commands[1] = (int32_t)(indi_u[1] * K_ppz_rads_motor);
            overactuated_mixing.commands[2] = (int32_t)(indi_u[2] * K_ppz_rads_motor);
            overactuated_mixing.commands[3] = (int32_t)(indi_u[3] * K_ppz_rads_motor);

            //Elevator servos:
            overactuated_mixing.commands[4] = (int32_t)(
                    (indi_u[4] - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[5] = (int32_t)(
                    (indi_u[5] - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[6] = (int32_t)(
                    (indi_u[6] - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[7] = (int32_t)(
                    (indi_u[7] - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE) * K_ppz_angle_el);

            //Azimuth servos:
            overactuated_mixing.commands[8] = (int32_t)(
                    (indi_u[8]  - OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[9] = (int32_t)(
                    (indi_u[9]  - OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[10] = (int32_t)(
                    (indi_u[10]  - OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[11] = (int32_t)(
                    (indi_u[11]  - OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE) * K_ppz_angle_az);
        }
        else{
            //Motors:
            overactuated_mixing.commands[0] = 0;
            overactuated_mixing.commands[1] = 0;
            overactuated_mixing.commands[2] = 0;
            overactuated_mixing.commands[3] = 0;

            //Elevator servos:
            overactuated_mixing.commands[4] = (int32_t)((-OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[5] = (int32_t)((-OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[6] = (int32_t)((-OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE) * K_ppz_angle_el);
            overactuated_mixing.commands[7] = (int32_t)((-OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE) * K_ppz_angle_el);

            //Azimuth servos:
            overactuated_mixing.commands[8] = (int32_t)((-OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[9] = (int32_t)((-OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[10] = (int32_t)((-OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE) * K_ppz_angle_az);
            overactuated_mixing.commands[11] = (int32_t)((-OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE) * K_ppz_angle_az);
        }
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

