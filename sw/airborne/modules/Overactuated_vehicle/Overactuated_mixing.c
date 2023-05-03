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
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.c"
#include "modules/sensors/ca_am7.h"
#include "modules/sensors/serial_act_t4.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
// #include "modules/sensors/aoa_pwm.h"
#include "modules/adcs/adc_generic.h"
#include "modules/energy/electrical.h"

#include "modules/core/sys_mon_rtos.h"

/**
 * Variables declaration
 */

#define FBW_ACTUATORS
#define MAX_DSHOT_VALUE 1999.0
#define RPM_CONTROL

// Decide if you want INDI RPM control or PID RPM control. Default is PID. 
#define RPM_INDI_CONTROL


// #define TEST_PWM_SERVOS
// #define TEST_RPM_CONTROL
// #define TEST_DSHOT_CONTROL
// #define TEST_ROTOR_ANGLES

// #define TEST_RASMUS_SERVO
// float time_old = 0; 
// float test_frequency = 0.5;

// struct ESC_status myESC_status;

//Array which contains all the actuator values (sent to motor and servos)
struct overactuated_mixing_t overactuated_mixing;

//General state variables:
float rate_vect[3], rate_vect_filt[3], rate_vect_filt_dot[3], euler_vect[3], acc_vect[3], acc_vect_filt[3], accel_vect_filt_control_rf[3],accel_vect_control_rf[3], speed_vect_control_rf[3];
float rate_vect_ned[3], acc_body_real[3], acc_body_real_filt[3];
float speed_vect[3], pos_vect[3], airspeed = 0, beta_deg = 0, beta_rad = 0, flight_path_angle = 0, total_V = 0;
float actuator_state[INDI_NUM_ACT];
float actuator_state_filt[INDI_NUM_ACT];
float euler_error[3];
float euler_error_integrated[3];
float angular_body_error[3];
float pos_error[3];
float pos_error_integrated[3];
float pos_order_body[3];
float pos_order_earth[3];
float euler_order[3];
float psi_order_motor = 0;


//Rotors test: 
float des_az_angle_test = 0; 
float des_el_angle_test = 0;

#ifdef RPM_CONTROL

//PID RPM controller specific 
float K_p_rad_s_dshot = RPM_CONTROL_FBW_K_P_RAD_S_DSHOT;
float K_i_rad_s_dshot = RPM_CONTROL_FBW_K_I_RAD_S_DSHOT;
float K_d_rad_s_dshot = RPM_CONTROL_FBW_K_D_RAD_S_DSHOT;
float motor_rad_s_dot_filtered[4], motor_rad_s_error_integrated[4], motor_rad_s_filtered_old[4];

//Indi RPM controller specific variables:
float dshot_cmd_ppz[4], dshot_cmd_ppz_filtered[4], dshot_cmd_ppz_filtered_delayed[4][RPM_CONTROL_FBW_MOTOR_DYN_DELAY_TS];
float dshot_cmd_state_filtered[4];
float K_indi_rad_s_dshot = RPM_CONTROL_FBW_K_INDI_RAD_S_DSHOT;

//General RPM control variables: 
float Des_RPM_motor_1 = 0;
float Des_dshot_steps_motor_1 = 0;
float motor_rad_s_filtered[4];

#endif

//Test PWM servos external to teensy: 
int min_pwm_servo_9 = FBW_T4_SERVO_9_MIN_PWM;
int max_pwm_servo_9 = FBW_T4_SERVO_9_MAX_PWM; 
int neutral_pwm_servo_9 = FBW_T4_SERVO_9_NEUTRAL_PWM; 

int min_pwm_servo_10 = FBW_T4_SERVO_10_MIN_PWM; 
int max_pwm_servo_10 = FBW_T4_SERVO_10_MAX_PWM; 
int neutral_pwm_servo_10 = FBW_T4_SERVO_10_NEUTRAL_PWM; 


float desired_angle_servo_9 = 0;
float desired_angle_servo_10 = 0;

//Sideslip gains
float K_beta = 0.1;

float Dynamic_MOTOR_K_T_OMEGASQ;

//Ailerons variables 
float CL_ailerons = VEHICLE_CL_AILERONS;
float roll_pwm_cmd; 

//Variables for the NONLINEAR_CA_DEBUG message: 
float feed_fwd_term_yaw, feed_back_term_yaw;

//Flight states variables:
bool INDI_engaged = 0, FAILSAFE_engaged = 0;

// PID and general settings from slider
int deadband_stick_yaw = 300, deadband_stick_throttle = 300;
float stick_gain_yaw = 0.05, stick_gain_throttle = 0.03; //  Stick to yaw and throttle gain (for the integral part)
bool yaw_with_tilting_PID = 1;

bool manual_heading = 0;
int manual_heading_value_rad = 0;

float des_pos_earth_x = 0;
float des_pos_earth_y = 0;

float alt_cmd = 0, pitch_cmd = 0, roll_cmd = 0, yaw_motor_cmd = 0, yaw_tilt_cmd = 0, elevation_cmd = 0, azimuth_cmd = 0;

//External servos variables: 
int16_t neutral_servo_1_pwm = 1417;
int16_t neutral_servo_2_pwm = 1492;

// int16_t neutral_servo_1_pwm = 1527;
// int16_t neutral_servo_2_pwm = 1365;

int servo_right_cmd = 0;
int servo_left_cmd = 0;

// Actuators gains:
#ifdef RPM_CONTROL
float K_ppz_rads_motor = 9.6;
#else
float K_ppz_rads_motor = 9.6 / OVERACTUATED_MIXING_MOTOR_K_PWM_OMEGA;
#endif

float K_ppz_angle_el = (9600 * 2) / (OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE);
float K_ppz_angle_az = (9600 * 2) / (OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE);

//Global variable for the ACTUATOR_OUTPUT messge: 
int32_t actuator_output[INDI_NUM_ACT], actuator_state_int[INDI_NUM_ACT];


//Incremental INDI variables
float indi_u[INDI_NUM_ACT], indi_u_scaled[INDI_NUM_ACT];

//Variables for the actuator model v2:
#ifndef FBW_ACTUATORS
#define actuator_mem_buf_size 100
float indi_u_memory[INDI_NUM_ACT][actuator_mem_buf_size];
float actuator_state_old[INDI_NUM_ACT];
float actuator_state_old_old[INDI_NUM_ACT];
int delay_ts_motor = (int) (OVERACTUATED_MIXING_INDI_MOTOR_FIRST_ORD_DELAY * PERIODIC_FREQUENCY_OVERACTUATED);
int delay_ts_az = (int) (OVERACTUATED_MIXING_INDI_AZ_SECOND_ORD_DELAY * PERIODIC_FREQUENCY_OVERACTUATED);
int delay_ts_el = (int) (OVERACTUATED_MIXING_INDI_EL_SECOND_ORD_DELAY * PERIODIC_FREQUENCY_OVERACTUATED);
int delay_ts_ailerons = (int) (OVERACTUATED_MIXING_INDI_AILERONS_FIRST_ORD_DELAY * PERIODIC_FREQUENCY_OVERACTUATED);
float max_rate_az = OVERACTUATED_MIXING_INDI_AZ_SECOND_ORD_RATE_LIMIT / PERIODIC_FREQUENCY_OVERACTUATED;
float max_rate_el = OVERACTUATED_MIXING_INDI_EL_SECOND_ORD_RATE_LIMIT / PERIODIC_FREQUENCY_OVERACTUATED;
#endif


//Setpoints and pseudocontrol
float pos_setpoint[3];
float speed_setpoint_control_rf[3];
float speed_error_vect[3];
float speed_error_vect_control_rf[3];
float euler_setpoint[3];
float rate_setpoint[3];
float acc_setpoint[6];
float INDI_pseudocontrol[INDI_INPUTS];


//AM7 variables:
float manual_motor_value = 0, manual_el_value = 0, manual_az_value = 0, manual_phi_value = 0, manual_theta_value = 0, manual_ailerons_value = 0;
struct am7_data_out am7_data_out_local;
float extra_data_out_local[255] __attribute__((aligned));
static abi_event AM7_in;
float extra_data_in_local[255] __attribute__((aligned));
struct am7_data_in myam7_data_in_local;

// serial_act_t4 variables:
struct serial_act_t4_out myserial_act_t4_out_local;
float serial_act_t4_extra_data_out_local[255] __attribute__((aligned));
static abi_event SERIAL_ACT_T4_IN;
float serial_act_t4_extra_data_in_local[255] __attribute__((aligned));
struct serial_act_t4_in myserial_act_t4_in_local;


//Variables needed for the filters:
Butterworth2LowPass measurement_rates_filters[3]; //Filter of pqr
Butterworth2LowPass measurement_acc_filters[3];   //Filter of acceleration
Butterworth2LowPass actuator_state_filters[INDI_NUM_ACT];   //Filter of actuators

//Filter of lateral acceleration for turn correction
Butterworth2LowPass accely_filt;

//Filter for the airspeed: 
Butterworth2LowPass airspeed_filt;

//Filters for flight path angle : 
Butterworth2LowPass flight_path_angle_filtered;

//Variables for the auto test: 
float auto_test_time_start, des_Vx, des_Vy, des_Vz, des_phi, des_theta, des_psi_dot;
uint8_t auto_test_start = 0; 

//Variables for the waypoint contol: 
int waypoint_mode = 0; 
float x_stb, y_stb, z_stb;

// Variables for the speed to derivative gain slider and thrust coefficient: 
float K_d_speed = 0.03; 
float K_T_airspeed = 0.025;

//Variables for the sysmon file write: 
float time_old_sys_mon = 0;
int debug_enable = 0; 

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
float act_dyn[INDI_NUM_ACT];

static void data_AM7_abi_in(uint8_t sender_id __attribute__((unused)), struct am7_data_in * myam7_data_in_ptr, float * extra_data_in_ptr){
    memcpy(&myam7_data_in_local,myam7_data_in_ptr,sizeof(struct am7_data_in));
    memcpy(&extra_data_in_local,extra_data_in_ptr,255 * sizeof(float));
}

static void serial_act_t4_abi_in(uint8_t sender_id __attribute__((unused)), struct serial_act_t4_in * myserial_act_t4_in_ptr, float * serial_act_t4_extra_data_in_ptr){
    memcpy(&myserial_act_t4_in_local,myserial_act_t4_in_ptr,sizeof(struct serial_act_t4_in));
    memcpy(&serial_act_t4_extra_data_in_local,serial_act_t4_extra_data_in_ptr,255 * sizeof(float));
}


/**
 * Function for the message NONLINEAR_CA_DEBUG
 */
static void send_nonlinear_ca_debug( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message

    pprz_msg_send_NONLINEAR_CA_DEBUG(trans , dev , AC_ID ,
                           & acc_setpoint[0],& acc_setpoint[1],& acc_setpoint[2],
                           & acc_setpoint[3],& acc_setpoint[4],& acc_setpoint[5],
                           & speed_setpoint_control_rf[0],& speed_setpoint_control_rf[1],& speed_setpoint_control_rf[2],
                           & rate_setpoint[0],& rate_setpoint[1],& rate_setpoint[2],
                           & pos_setpoint[0],& pos_setpoint[1],& pos_setpoint[2],
                           & euler_setpoint[0],& euler_setpoint[1], & euler_setpoint[2],
                           & feed_fwd_term_yaw, & feed_back_term_yaw);
}

/**
 * Function for the message OVERACTUATED_VARIABLES
 */
static void send_overactuated_variables( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message
    pprz_msg_send_OVERACTUATED_VARIABLES(trans , dev , AC_ID ,
                                         & airspeed, & auto_test_start , & beta_deg,
                                         & pos_vect[0], & pos_vect[1], & pos_vect[2],
                                         & speed_vect[0], & speed_vect[1], & speed_vect[2],
                                         & accel_vect_filt_control_rf[0], & accel_vect_filt_control_rf[1], & accel_vect_filt_control_rf[2],
                                         & rate_vect_filt_dot[0], & rate_vect_filt_dot[1], & rate_vect_filt_dot[2],
                                         & rate_vect_filt[0], & rate_vect_filt[1], & rate_vect_filt[2],
                                         & euler_vect[0], & euler_vect[1], & euler_vect[2],
                                         & euler_setpoint[0], & euler_setpoint[1], & euler_setpoint[2],
                                         & pos_setpoint[0], & pos_setpoint[1], & pos_setpoint[2],
                                         & alt_cmd, & pitch_cmd, & roll_cmd, & yaw_motor_cmd, & yaw_tilt_cmd, & elevation_cmd, & azimuth_cmd);
}

/**
 * Function that computes a value linearly passing from 0 to 1. 
 * if current_speed < start_speed the output is 0
 * if start_speed < current_speed < end_speed the output is linearly increasing from 0 to 1
 * if current_speed > end_speed the output is 1
 */
float compute_lat_speed_multiplier(float start_speed, float end_speed, float current_speed){

    float lat_speed_multiplier = (current_speed - start_speed) / (end_speed - start_speed);
    Bound(lat_speed_multiplier , 0, 1);
    return lat_speed_multiplier;
}

/**
 * Transpose an array from propeller reference frame to body reference frame
 */
void from_propeller_to_body(float * out_array, float * in_array, float b, float g){
    float R_pb_matrix[3][3];
    R_pb_matrix[0][0] = cos(b);
    R_pb_matrix[0][1] = 0;
    R_pb_matrix[0][2] = sin(b);
    R_pb_matrix[1][0] = sin(g)*sin(b) ;
    R_pb_matrix[1][1] = cos(g) ;
    R_pb_matrix[1][2] = -sin(g)*cos(b) ;
    R_pb_matrix[2][0] = -cos(g)*sin(b) ;
    R_pb_matrix[2][1] = sin(g) ;
    R_pb_matrix[2][2] = cos(g)*cos(b);

    //Do the multiplication between the income array and the matrix:
    for (int j = 0; j < 3; j++) {
        //Initialize value to zero:
        out_array[j] = 0.;
        for (int k = 0; k < 3; k++) {
            out_array[j] += in_array[k] * R_pb_matrix[j][k];
        }
    }
}

/**
 * Transpose the euler rates array into body rates array
 */
void from_euler_rates_to_body_rates(float * out_array, float * in_array, float Theta, float Phi){
    float R_matrix[3][3];
    R_matrix[0][0] = 1;
    R_matrix[0][1] = 0;
    R_matrix[0][2] = -sin(Theta);
    R_matrix[1][0] = 0 ;
    R_matrix[1][1] = cos(Phi) ;
    R_matrix[1][2] = sin(Phi)*cos(Theta);
    R_matrix[2][0] = 0 ;
    R_matrix[2][1] = -sin(Phi) ;
    R_matrix[2][2] = cos(Phi)*cos(Theta);

    //Do the multiplication between the income array and the matrix:
    for (int j = 0; j < 3; j++) {
        //Initialize value to zero:
        out_array[j] = 0.;
        for (int k = 0; k < 3; k++) {
            out_array[j] += in_array[k] * R_matrix[j][k];
        }
    }
}

/**
 * Transpose an array from earth reference frame to control reference frame
 */
void from_earth_to_control(float * out_array, float * in_array, float Psi){
    float R_gc_matrix[3][3];
    R_gc_matrix[0][0] = cos(Psi);
    R_gc_matrix[0][1] = -sin(Psi);
    R_gc_matrix[0][2] = 0;
    R_gc_matrix[1][0] = sin(Psi) ;
    R_gc_matrix[1][1] = cos(Psi) ;
    R_gc_matrix[1][2] = 0 ;
    R_gc_matrix[2][0] = 0 ;
    R_gc_matrix[2][1] = 0 ;
    R_gc_matrix[2][2] = 1 ;

    //Do the multiplication between the income array and the transposition matrix:
    for (int j = 0; j < 3; j++) {
        //Initialize value to zero:
        out_array[j] = 0.;
        for (int k = 0; k < 3; k++) {
            out_array[j] += in_array[k] * R_gc_matrix[k][j];
        }
    }
}


/**
 * Function which computes the thrust in the body reference frame using the inflow angle 
 * characteristics determined in the wind tunnel:
 */
struct BodyCoord_f compute_propeller_thrust_in_body_frame(float propeller_speed, float b_angle_rotor, float g_angle_rotor, float omega_prop_rad_s){
    struct BodyCoord_f myBodyCoord_f;

    float p_kt_0 = (PROP_MODEL_P00_KT + 
                    PROP_MODEL_P10_KT*propeller_speed + 
                    PROP_MODEL_P20_KT*propeller_speed*propeller_speed + 
                    PROP_MODEL_P30_KT*propeller_speed*propeller_speed*propeller_speed + 
                    PROP_MODEL_P40_KT*propeller_speed*propeller_speed*propeller_speed*propeller_speed)*PROP_MODEL_KT_REF;
    float p_kt_1 = (PROP_MODEL_P01_KT +
                    PROP_MODEL_P11_KT*propeller_speed +
                    PROP_MODEL_P21_KT*propeller_speed*propeller_speed + 
                    PROP_MODEL_P31_KT*propeller_speed*propeller_speed*propeller_speed)*PROP_MODEL_KT_REF;
    float p_kt_2 = (PROP_MODEL_P02_KT + 
                    PROP_MODEL_P12_KT*propeller_speed + 
                    PROP_MODEL_P22_KT*propeller_speed*propeller_speed)*PROP_MODEL_KT_REF;

    float p_qt_0 = (PROP_MODEL_P00_QT + 
                    PROP_MODEL_P10_QT*propeller_speed + 
                    PROP_MODEL_P20_QT*propeller_speed*propeller_speed + 
                    PROP_MODEL_P30_QT*propeller_speed*propeller_speed*propeller_speed + 
                    PROP_MODEL_P40_QT*propeller_speed*propeller_speed*propeller_speed*propeller_speed)*PROP_MODEL_MAX_THR_LOSS_REAL;
    float p_qt_1 = (PROP_MODEL_P01_QT +
                    PROP_MODEL_P11_QT*propeller_speed +
                    PROP_MODEL_P21_QT*propeller_speed*propeller_speed + 
                    PROP_MODEL_P31_QT*propeller_speed*propeller_speed*propeller_speed)*PROP_MODEL_MAX_THR_LOSS_REAL;
    float p_qt_2 = (PROP_MODEL_P02_QT + 
                    PROP_MODEL_P12_QT*propeller_speed + 
                    PROP_MODEL_P22_QT*propeller_speed*propeller_speed)*PROP_MODEL_MAX_THR_LOSS_REAL;

    float thr_value = omega_prop_rad_s * omega_prop_rad_s * (p_kt_0 + p_kt_1 * b_angle_rotor + p_kt_2 * b_angle_rotor * b_angle_rotor) +
                      (p_qt_0 + p_qt_1 * b_angle_rotor + p_qt_2 * b_angle_rotor * b_angle_rotor);       

    float thr_array_prop_rf[3] = {0 , 0, thr_value};
    float thr_array_body_rf[3]; 

    from_propeller_to_body(thr_array_body_rf, thr_array_prop_rf, b_angle_rotor, g_angle_rotor);

    //Transpose the thrust value from propeller RF to body RF: 
    myBodyCoord_f.x = thr_array_body_rf[0];
    myBodyCoord_f.y = thr_array_body_rf[1];
    myBodyCoord_f.z = thr_array_body_rf[2];
    return myBodyCoord_f;
}

/**
 * Function that computes the yaw rate for the coordinate turn:
 */
float compute_yaw_rate_turn(void){
            //Compute the yaw rate for the coordinate turn:
        float yaw_rate_setpoint_turn = 0;

        float airspeed_turn = airspeed;
        //We are dividing by the airspeed, so a lower bound is important
        Bound(airspeed_turn,10.0,30.0);

        float accel_y_filt_corrected = 0;

        // accel_y_filt_corrected = accely_filt.o[0] 
        //                         - actuator_state_filt[0]*actuator_state_filt[0]* Dynamic_MOTOR_K_T_OMEGASQ * sin(actuator_state_filt[8])/VEHICLE_MASS
        //                         - actuator_state_filt[1]*actuator_state_filt[1]* Dynamic_MOTOR_K_T_OMEGASQ * sin(actuator_state_filt[9])/VEHICLE_MASS
        //                         - actuator_state_filt[2]*actuator_state_filt[2]* Dynamic_MOTOR_K_T_OMEGASQ * sin(actuator_state_filt[10])/VEHICLE_MASS
        //                         - actuator_state_filt[3]*actuator_state_filt[3]* Dynamic_MOTOR_K_T_OMEGASQ * sin(actuator_state_filt[11])/VEHICLE_MASS;
                                
        accel_y_filt_corrected = accely_filt.o[0] 
                                - compute_propeller_thrust_in_body_frame(airspeed_filt.o[0],actuator_state_filt[4],actuator_state_filt[8],actuator_state_filt[0]).y/VEHICLE_MASS
                                - compute_propeller_thrust_in_body_frame(airspeed_filt.o[0],actuator_state_filt[5],actuator_state_filt[9],actuator_state_filt[1]).y/VEHICLE_MASS
                                - compute_propeller_thrust_in_body_frame(airspeed_filt.o[0],actuator_state_filt[6],actuator_state_filt[10],actuator_state_filt[2]).y/VEHICLE_MASS
                                - compute_propeller_thrust_in_body_frame(airspeed_filt.o[0],actuator_state_filt[7],actuator_state_filt[11],actuator_state_filt[3]).y/VEHICLE_MASS;

        // Creating the setpoint using the bank angle and the body acceleration correction for the sideslip:
        yaw_rate_setpoint_turn = 9.81*tan(euler_vect[0])/airspeed_turn - K_beta * accel_y_filt_corrected;

        feed_fwd_term_yaw = 9.81*tan(euler_vect[0])/airspeed_turn;
        feed_back_term_yaw = - K_beta * accel_y_filt_corrected;

        yaw_rate_setpoint_turn = yaw_rate_setpoint_turn * compute_lat_speed_multiplier(OVERACTUATED_MIXING_MIN_SPEED_TRANSITION,OVERACTUATED_MIXING_REF_SPEED_TRANSITION,airspeed);

        return yaw_rate_setpoint_turn;
}

/**
 * Function that computes the speed reference in the control reference frame, taking as input 
 * the current vehicle position and the desired position in the ground reference frame:
 */
void compute_speed_ref_from_waypoint(float * speed_reference_control_rf, float * dest_pos_ground_rf, float * current_pos_ground_rf, float airspeed_vehicle, float Psi){
    float pos_error_earth_rf[3], pos_error_control_rf[3];
    //Compute position error in ground reference frame 
    for( int i=0; i<3; i++){
        pos_error_earth_rf[i] = dest_pos_ground_rf[i] - current_pos_ground_rf[i];
    }
    //Transpose position error from ground rf to control rf: 
    from_earth_to_control(pos_error_control_rf, pos_error_earth_rf, Psi);

    //Before applying gains and bounds, let's compute the dynamic saturation point for the fwd speed, based on the control-x distance of the WP: 
    float max_fwd_speed_approach_wp = sqrt( fabs(pos_error_control_rf[0]) * 2 * WP_CONTROL_MAX_DECEL_WP_APPROACH );

    //Apply saturation block to the maximum position error: 
    BoundAbs(pos_error_control_rf[0],WP_CONTROL_MAX_POS_XY_ERROR);
    BoundAbs(pos_error_control_rf[1],WP_CONTROL_MAX_POS_XY_ERROR);
    BoundAbs(pos_error_control_rf[2],WP_CONTROL_MAX_POS_Z_ERROR);

    //Now apply static gains to the position error to generate the speed references in the control rf: 
    speed_reference_control_rf[0] = pos_error_control_rf[0] * WP_CONTROL_VX_CONTROL_STATIC_GAIN;
    speed_reference_control_rf[1] = pos_error_control_rf[1] * WP_CONTROL_VY_CONTROL_STATIC_GAIN;
    speed_reference_control_rf[2] = pos_error_control_rf[2] * WP_CONTROL_VZ_CONTROL_STATIC_GAIN;

    //Now compute and apply the Vy and Vz gains with the airspeed dependency: 
    float Vy_dyn_gain = 1 + WP_CONTROL_VY_AIRSPEED_GAIN_COEFF * airspeed_vehicle;
    float Vz_dyn_gain = 1 + WP_CONTROL_VZ_AIRSPEED_GAIN_COEFF * airspeed_vehicle;
    Bound(Vy_dyn_gain,WP_CONTROL_VY_GAIN_MIN_VAL,WP_CONTROL_VY_GAIN_MAX_VAL);
    Bound(Vz_dyn_gain,WP_CONTROL_VZ_GAIN_MIN_VAL,WP_CONTROL_VZ_GAIN_MAX_VAL);

    speed_reference_control_rf[1] = speed_reference_control_rf[1] * Vy_dyn_gain;
    speed_reference_control_rf[2] = speed_reference_control_rf[2] * Vz_dyn_gain;

    //Apply approach constrain to fwd speed: 
    Bound(speed_reference_control_rf[0],LIMITS_FWD_MIN_FWD_SPEED,max_fwd_speed_approach_wp);

}

/**
 * Function for the message ACTUATORS_OUTPUT
 */
static void send_actuator_variables( struct transport_tx *trans , struct link_device * dev ) {
    pprz_msg_send_ACTUATORS_OUTPUT(trans , dev , AC_ID ,
                                         & actuator_output[0],
                                         & actuator_output[1],
                                         & actuator_output[2],
                                         & actuator_output[3],
                                         & actuator_output[4],
                                         & actuator_output[5],
                                         & actuator_output[6],
                                         & actuator_output[7],
                                         & actuator_output[8],
                                         & actuator_output[9],
                                         & actuator_output[10],
                                         & actuator_output[11],
                                         & actuator_output[12],
                                         & actuator_state_int[0],
                                         & actuator_state_int[1],
                                         & actuator_state_int[2],
                                         & actuator_state_int[3],
                                         & actuator_state_int[4],
                                         & actuator_state_int[5],
                                         & actuator_state_int[6],
                                         & actuator_state_int[7],
                                         & actuator_state_int[8],
                                         & actuator_state_int[9],
                                         & actuator_state_int[10],
                                         & actuator_state_int[11],
                                         & actuator_state_int[12]);
}

/**
 * Initialize the filters
 */
void init_filters(void){
    float sample_time = 1.0 / PERIODIC_FREQUENCY_OVERACTUATED;
    //Sensors cutoff frequency
    float tau_indi = 1.0 / (OVERACTUATED_MIXING_FILT_CUTOFF_INDI);

    // Initialize filters for the actuators
    for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
        init_butterworth_2_low_pass(&actuator_state_filters[i], tau_indi, sample_time, 0.0);
    }

    // Initialize filters for the rates derivative and accelerations
    for (int i = 0; i < 3; i++) {
        init_butterworth_2_low_pass(&measurement_rates_filters[i], tau_indi, sample_time, 0.0);
        init_butterworth_2_low_pass(&measurement_acc_filters[i], tau_indi, sample_time, 0.0);
    }
    
    //Initialize filter for the lateral acceleration to correct the turn and the flight path angle: 
    init_butterworth_2_low_pass(&accely_filt, tau_indi, sample_time, 0.0);
    init_butterworth_2_low_pass(&flight_path_angle_filtered, tau_indi, sample_time, 0.0);
    init_butterworth_2_low_pass(&airspeed_filt, tau_indi, sample_time, 0.0);

    //Initialize to zero the variables of get_actuator_state_v2:
    #ifndef FBW_ACTUATORS
        for(int i = 0; i < INDI_NUM_ACT; i++){
            for(int j = 0; j < actuator_mem_buf_size; j++ ){
                indi_u_memory[i][j] = 0;
            }
            actuator_state_old_old[i] = 0;
            actuator_state_old[i] = 0;
        }
    #endif
}

/**
 * @brief Check the system performance
 * 
 */
static void status_nederdrone_sysmon(void) {
  static uint8_t cnt = 0;

  if(rtos_mon.cpu_load > 85 || (debug_enable && cnt++ > 10)) {
    sdLogWriteLog(pprzLogFile, "Data reported in the RTOS_MON message:\r\n");
    sdLogWriteLog(pprzLogFile, " core free mem: %lu\r\n", rtos_mon.core_free_memory);
    sdLogWriteLog(pprzLogFile, " heap free mem: %lu\r\n", rtos_mon.heap_free_memory);
    sdLogWriteLog(pprzLogFile, " heap fragments: %lu\r\n", rtos_mon.heap_fragments);
    sdLogWriteLog(pprzLogFile, " heap largest: %lu\r\n", rtos_mon.heap_largest);
    sdLogWriteLog(pprzLogFile, " CPU load: %d %%\r\n", rtos_mon.cpu_load);
    sdLogWriteLog(pprzLogFile, " number of threads: %d\r\n", rtos_mon.thread_counter);
    sdLogWriteLog(pprzLogFile, " thread names: %s\r\n", rtos_mon.thread_names);
    for (int i = 0; i < rtos_mon.thread_counter; i++) {
      sdLogWriteLog(pprzLogFile, " thread %d load: %0.1f, free stack: %d\r\n", i,
              (float)rtos_mon.thread_load[i] / 10.f, rtos_mon.thread_free_stack[i]);
    }
    sdLogWriteLog(pprzLogFile, " CPU time: %.2f\r\n", rtos_mon.cpu_time);

    cnt = 0;
  }
}

/**
 * Get actuator state based on second order dynamics with rate limiter for servos and first order dynamics for motor
 */
void get_actuator_state_v2(void)
{
    #ifndef FBW_ACTUATORS
    //actuator dynamics
    for (int i = 0; i < INDI_NUM_ACT; i++) {        
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
        //Ailerons: 
        actuator_state[14] = - OVERACTUATED_MIXING_INDI_AILERONS_FIRST_ORD_DEN_2 * actuator_state_old[14] +
                    OVERACTUATED_MIXING_INDI_AILERONS_FIRST_ORD_NUM_2 * indi_u_memory[14][actuator_mem_buf_size - delay_ts_ailerons - 1];
        Bound(actuator_state[14],OVERACTUATED_MIXING_MIN_DELTA_AILERONS,OVERACTUATED_MIXING_MAX_DELTA_AILERONS);
        
        //Attitude angles: 
        actuator_state[12] = euler_vect[1]; //Theta
        actuator_state[13] = euler_vect[0]; //Phi        

        //Propagate the actuator values into the filters and calculate the derivative
        update_butterworth_2_low_pass(&actuator_state_filters[i], actuator_state[i]);
        actuator_state_filt[i] = actuator_state_filters[i].o[0];


        //Assign the memory variables:
        actuator_state_old_old[i] = actuator_state_old[i];
        actuator_state_old[i] = actuator_state[i];
        for (int j = 1; j < actuator_mem_buf_size ; j++){
            indi_u_memory[i][j-1] = indi_u_memory[i][j];
        }
        indi_u_memory[i][actuator_mem_buf_size-1] = indi_u[i];

    }
    #else //Case of FWB through Teensy 4.0

    //Motors:
    actuator_state[0] = 2.0*M_PI*myserial_act_t4_in_local.motor_1_rpm_int/60.0;
    actuator_state[1] = 2.0*M_PI*myserial_act_t4_in_local.motor_2_rpm_int/60.0;
    actuator_state[2] = 2.0*M_PI*myserial_act_t4_in_local.motor_3_rpm_int/60.0;
    actuator_state[3] = 2.0*M_PI*myserial_act_t4_in_local.motor_4_rpm_int/60.0;

    
    //Elevatoion angles: 
    actuator_state[4] = ((myserial_act_t4_in_local.servo_2_angle_int/100.0)/FBW_T4_K_RATIO_GEAR_EL) * M_PI/180.0 + OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE/FBW_T4_K_RATIO_GEAR_EL; 
    actuator_state[5] = ((myserial_act_t4_in_local.servo_6_angle_int/100.0)/FBW_T4_K_RATIO_GEAR_EL) * M_PI/180.0 + OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE/FBW_T4_K_RATIO_GEAR_EL; 
    actuator_state[6] = ((myserial_act_t4_in_local.servo_8_angle_int/100.0)/FBW_T4_K_RATIO_GEAR_EL) * M_PI/180.0 + OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE/FBW_T4_K_RATIO_GEAR_EL; 
    actuator_state[7] = ((myserial_act_t4_in_local.servo_4_angle_int/100.0)/FBW_T4_K_RATIO_GEAR_EL) * M_PI/180.0 + OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE/FBW_T4_K_RATIO_GEAR_EL; 

    //Azimuth angles: 
    actuator_state[8] = ((myserial_act_t4_in_local.servo_1_angle_int/100.0)/FBW_T4_K_RATIO_GEAR_AZ) * M_PI/180.0 + OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE/FBW_T4_K_RATIO_GEAR_AZ;  
    actuator_state[9] = ((myserial_act_t4_in_local.servo_5_angle_int/100.0)/FBW_T4_K_RATIO_GEAR_AZ) * M_PI/180.0 + OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE/FBW_T4_K_RATIO_GEAR_AZ;  
    actuator_state[10] = ((-myserial_act_t4_in_local.servo_7_angle_int/100.0)/FBW_T4_K_RATIO_GEAR_AZ) * M_PI/180.0 + OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE/FBW_T4_K_RATIO_GEAR_AZ;  
    actuator_state[11] = ((-myserial_act_t4_in_local.servo_3_angle_int/100.0)/FBW_T4_K_RATIO_GEAR_AZ) * M_PI/180.0 + OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE/FBW_T4_K_RATIO_GEAR_AZ;   

    //Ailerons: 
    actuator_state[14] = (myserial_act_t4_in_local.servo_9_angle_int + myserial_act_t4_in_local.servo_10_angle_int)/200.0 * M_PI/180.0 ; 

    //Attitude angles: 
    actuator_state[12] = euler_vect[1]; //Theta
    actuator_state[13] = euler_vect[0]; //Phi   

    //Filter the actuator states: 
    for (int i = 0; i < INDI_NUM_ACT; i++) {  
        update_butterworth_2_low_pass(&actuator_state_filters[i], actuator_state[i]);
        actuator_state_filt[i] = actuator_state_filters[i].o[0];
    }

    // Collect extra packet from extra_data_in rolling message: 
    // myESC_status.ESC_1_rpm = myserial_act_t4_in_local.motor_1_rpm_int; 
    // myESC_status.ESC_2_rpm = myserial_act_t4_in_local.motor_2_rpm_int; 
    // myESC_status.ESC_3_rpm = myserial_act_t4_in_local.motor_3_rpm_int; 
    // myESC_status.ESC_4_rpm = myserial_act_t4_in_local.motor_4_rpm_int; 
    
    // myESC_status.ESC_1_voltage = serial_act_t4_extra_data_in_local[0];
    // myESC_status.ESC_2_voltage = serial_act_t4_extra_data_in_local[1];
    // myESC_status.ESC_3_voltage = serial_act_t4_extra_data_in_local[2];
    // myESC_status.ESC_4_voltage = serial_act_t4_extra_data_in_local[3];

    // myESC_status.ESC_1_current = serial_act_t4_extra_data_in_local[4];
    // myESC_status.ESC_2_current = serial_act_t4_extra_data_in_local[5];
    // myESC_status.ESC_3_current = serial_act_t4_extra_data_in_local[6];
    // myESC_status.ESC_4_current = serial_act_t4_extra_data_in_local[7];

    // myESC_status.ESC_1_consumption = serial_act_t4_extra_data_in_local[8];
    // myESC_status.ESC_2_consumption = serial_act_t4_extra_data_in_local[9];
    // myESC_status.ESC_3_consumption = serial_act_t4_extra_data_in_local[10];
    // myESC_status.ESC_4_consumption = serial_act_t4_extra_data_in_local[11];

    #endif

}

/**
 * Initialize the overactuated mixing module
 */
void overactuated_mixing_init(void) {

    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_OVERACTUATED_VARIABLES , send_overactuated_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_ACTUATORS_OUTPUT , send_actuator_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_NONLINEAR_CA_DEBUG , send_nonlinear_ca_debug );

    //Startup the init variables of the INDI
    init_filters();

    //Init abi bind msg to Raspberry Pi:
    AbiBindMsgAM7_DATA_IN(ABI_BROADCAST, &AM7_in, data_AM7_abi_in);
    //Init abi bind msg to Teensy 4.0:
    AbiBindMsgSERIAL_ACT_T4_IN(ABI_BROADCAST, &SERIAL_ACT_T4_IN, serial_act_t4_abi_in);
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
    airspeed = ms45xx.airspeed;
    // beta_deg = - aoa_pwm.angle * 180/M_PI;
    beta_rad = beta_deg * M_PI / 180;

    total_V = sqrt(speed_vect[0]*speed_vect[0] + speed_vect[1]*speed_vect[1] + speed_vect[2]*speed_vect[2]);
    if(total_V > 5){
        flight_path_angle = asin(-speed_vect[2]/total_V);
        BoundAbs(flight_path_angle, M_PI/2);
    }
    else{
        flight_path_angle = 0;
    }

    for (int i = 0 ; i < 4 ; i++){
        act_dyn[i] = act_dyn_struct.motor;
        act_dyn[i+4] = act_dyn_struct.elevation;
        act_dyn[i+8] = act_dyn_struct.azimuth;
    }

    //Determine the acceleration set in the body reference frame with gravity:

    /* Propagate the filter on the gyroscopes and accelerometers */
    for (int i = 0; i < 3; i++) {
        update_butterworth_2_low_pass(&measurement_rates_filters[i], rate_vect[i]);
        update_butterworth_2_low_pass(&measurement_acc_filters[i], acc_vect[i]);

        //Calculate the angular acceleration via finite difference
        rate_vect_filt_dot[i] = (measurement_rates_filters[i].o[0]
                                 - measurement_rates_filters[i].o[1]) * PERIODIC_FREQUENCY_OVERACTUATED;

        // rate_vect_filt[i] = measurement_rates_filters[i].o[0];

        acc_vect_filt[i] = measurement_acc_filters[i].o[0];

        rate_vect_filt[i] = rate_vect_filt[i] + OVERACTUATED_MIXING_FIRST_ORDER_FILTER_COEFF_ANG_RATES * (rate_vect[i] - rate_vect_filt[i]);

    }

    // Update the filter for the body lateral acceleration 
    float accely = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
    update_butterworth_2_low_pass(&accely_filt, accely);

    //Flight path angle 
    update_butterworth_2_low_pass(&flight_path_angle_filtered, flight_path_angle);

    //Airspeed 
    update_butterworth_2_low_pass(&airspeed_filt, airspeed);

    //Determination of the accelerations in the control rf:
    from_earth_to_control( accel_vect_control_rf, acc_vect, euler_vect[2]);
    from_earth_to_control( accel_vect_filt_control_rf, acc_vect_filt, euler_vect[2]);
    from_earth_to_control( speed_vect_control_rf, speed_vect, euler_vect[2]);
}

/**
 * Run the overactuated mixing
 */
void overactuated_mixing_run(void)
{
    //Assign variables
    assign_variables();

    //Write to sysmon every 1 second if required by debug enable
    if(get_sys_time_float() - time_old_sys_mon >= 1 && debug_enable){
        status_nederdrone_sysmon();
        time_old_sys_mon = get_sys_time_float();
    }

    //Retrieve the position of the STB WP: 
    x_stb = waypoint_get_y(WP_STDBY);
    y_stb = waypoint_get_x(WP_STDBY);
    z_stb = -waypoint_get_alt(WP_STDBY); 

    // //Prepare the reference for the AUTO test with the nonlinear controller: 
        
    // if(radio_control.values[none] < -500){
    //     auto_test_start = 1; 
    // }
    // else{
    //     auto_test_start = 0;
    // }

    // if(auto_test_start){
        
    //     if( get_sys_time_float() - auto_test_time_start >= 0 && get_sys_time_float() - auto_test_time_start < 2){
    //         des_Vx = 0;
    //         des_Vy = 0;
    //         des_Vz = 0;
    //         des_phi = 0;
    //         des_theta = 0;
    //         des_psi_dot = 0;  
    //     }
    //     else if( get_sys_time_float() - auto_test_time_start >= 2 && get_sys_time_float() - auto_test_time_start < 7){
    //         des_Vx = 15;
    //         des_Vy = 0;
    //         des_Vz = 0;
    //         des_phi = 0;
    //         des_theta = 0;
    //         des_psi_dot = 0;  
    //     }
    //     else if( get_sys_time_float() - auto_test_time_start >= 7 && get_sys_time_float() - auto_test_time_start < 12){
    //         des_Vx = 15;
    //         des_Vy = 0;
    //         des_Vz = -4;
    //         des_phi = 0;
    //         des_theta = 0;
    //         des_psi_dot = 0;  
    //     }
    //     else if( get_sys_time_float() - auto_test_time_start >= 12 && get_sys_time_float() - auto_test_time_start < 15){
    //         des_Vx = 15;
    //         des_Vy = 0;
    //         des_Vz = 0;
    //         des_phi = 0;
    //         des_theta = 0;
    //         des_psi_dot = 0;  
    //     }
    //     else if( get_sys_time_float() - auto_test_time_start >= 15 && get_sys_time_float() - auto_test_time_start < 22){
    //         des_Vx = 15;
    //         des_Vy = 4;
    //         des_Vz = 0;
    //         des_phi = 0;
    //         des_theta = 0;
    //         des_psi_dot = 0;   
    //     }
    //     else if( get_sys_time_float() - auto_test_time_start >= 22 && get_sys_time_float() - auto_test_time_start < 26){
    //         des_Vx = 15;
    //         des_Vy = 0;
    //         des_Vz = 0;
    //         des_phi = 0;
    //         des_theta = 0;
    //         des_psi_dot = 0;   
    //     }
    //     else{
    //         des_Vx = 0;
    //         des_Vy = 0;
    //         des_Vz = 0;
    //         des_phi = 0;
    //         des_theta = 0;
    //         des_psi_dot = 0;   
    //     }                

    // } else{
    //     auto_test_time_start = get_sys_time_float();
    // }

    /// Case of manual PID control [FAILSAFE]
    // if(0){
    if(radio_control.values[RADIO_MODE] < -500) {


        //INIT AND BOOLEAN RESET
        if(FAILSAFE_engaged == 0 ){
            INDI_engaged = 0;
            FAILSAFE_engaged = 1;
            for (int i = 0; i < 3; i++) {
                euler_error_integrated[i] = 0;
                pos_error_integrated[i] = 0;
            }
            euler_setpoint[2] = euler_vect[2];

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
            euler_error_integrated[i] += euler_error[i] / PERIODIC_FREQUENCY_OVERACTUATED;
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
        if(yaw_with_tilting_PID){
            overactuated_mixing.commands[8] += (int32_t) (euler_order[2] * K_ppz_angle_az);
            overactuated_mixing.commands[9] += (int32_t) (euler_order[2] * K_ppz_angle_az);
            overactuated_mixing.commands[10] -= (int32_t) (euler_order[2] * K_ppz_angle_az);
            overactuated_mixing.commands[11] -= (int32_t) (euler_order[2] * K_ppz_angle_az);
        }

        //Add servos values:
        servo_right_cmd = neutral_servo_1_pwm;
        servo_left_cmd = neutral_servo_2_pwm;

        indi_u[14] = 0;

        Bound(servo_right_cmd,1000,2000);
        Bound(servo_left_cmd,1000,2000);

    }

    /// Case of INDI control mode with external nonlinear function:
    //    if(1)
    if(radio_control.values[RADIO_MODE] >= -500 )
    {
        //Manual Speed reference mode
        if(radio_control.values[RADIO_MODE] < 500){
            waypoint_mode = 0;
        }
        // Waypoint reference mode
        if(radio_control.values[RADIO_MODE] >= 500){
            waypoint_mode = 1;
        }

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

            actuator_state_filt[12] = 0;
            actuator_state_filt[13] = 0;

            actuator_state_filt[14] = 0;

            init_filters();

            INDI_engaged = 1;
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

            indi_u[12] = 0;
            indi_u[13] = 0;

            indi_u[14] = 0;

            rate_vect_filt[0] = 0;
            rate_vect_filt[1] = 0;
            rate_vect_filt[2] = 0;

            pos_setpoint[2] = pos_vect[2];

            //Init dshot RPM control cmds
            #ifdef RPM_CONTROL
            if(RadioControlValues(RADIO_TH_HOLD) > 0){
                for (int i = 0; i < 4; i++){

                    #ifdef RPM_INDI_CONTROL

                    dshot_cmd_ppz_filtered[i] = overactuated_mixing.commands[i];
                    for (int j = 0; j < RPM_CONTROL_FBW_MOTOR_DYN_DELAY_TS; j++){
                        dshot_cmd_ppz_filtered_delayed[i][j] = overactuated_mixing.commands[i];
                    }
                    dshot_cmd_state_filtered[i] = overactuated_mixing.commands[i];
                    dshot_cmd_ppz[i] = overactuated_mixing.commands[i];

                    #else //PID RPM CONTROL

                    motor_rad_s_filtered_old[i] = actuator_state[i];
                    motor_rad_s_error_integrated[i] = 0;

                    #endif 

                    motor_rad_s_filtered[i] = actuator_state[i];

                }
            }
            #endif
        }

        //Correct the K_T with speed: 
        float local_gain_K_T = 1 - airspeed*K_T_airspeed ;
        Bound( local_gain_K_T, 0.1, 1);

        //Compute the actual k_motor_omegasq with voltage value: 
        Dynamic_MOTOR_K_T_OMEGASQ = local_gain_K_T * (electrical.vsupply * OVERACTUATED_MIXING_MOTOR_K_T_OMEGASQ_P1 + OVERACTUATED_MIXING_MOTOR_K_T_OMEGASQ_P2);

        // Dynamic_MOTOR_K_T_OMEGASQ = Dynamic_MOTOR_K_T_OMEGASQ * local_gain_K_T;

        // Get an estimate of the actuator state using the second order dynamics
        get_actuator_state_v2();

        //Calculate the desired euler angles from the auxiliary joystick channels:
        manual_phi_value = MANUAL_CONTROL_MAX_CMD_ROLL_ANGLE * radio_control.values[RADIO_AUX3] / 9600;
        manual_theta_value = MANUAL_CONTROL_MAX_CMD_PITCH_ANGLE * radio_control.values[RADIO_AUX4] / 9600;

        //Add the setpoint in case of AUTO test
        if(auto_test_start){ 
            manual_phi_value = des_phi * M_PI/180; 
            manual_theta_value = des_theta * M_PI/180; 
        }

        manual_motor_value = OVERACTUATED_MIXING_MOTOR_MIN_OMEGA;

        euler_setpoint[0] = indi_u[13];
        euler_setpoint[1] = indi_u[12];

        // euler_setpoint[0] = manual_phi_value;
        // euler_setpoint[1] = manual_theta_value;       

        BoundAbs(euler_setpoint[0],max_value_error.phi);
        BoundAbs(euler_setpoint[1],max_value_error.theta);
        euler_error[0] = euler_setpoint[0] - euler_vect[0];
        euler_error[1] = euler_setpoint[1] - euler_vect[1];

        // For the yaw, we can directly control the rates:
        float yaw_rate_setpoint_manual = MANUAL_CONTROL_MAX_CMD_YAW_RATE * radio_control.values[RADIO_YAW] / 9600;

        //Add the setpoint in case of AUTO test
        if(auto_test_start){ 
            yaw_rate_setpoint_manual = des_psi_dot * M_PI/180; 
        }

        euler_error[2] = yaw_rate_setpoint_manual + compute_yaw_rate_turn();

        float gain_to_speed_constant = 1 - airspeed * K_d_speed; 
        Bound(gain_to_speed_constant, 0.1, 1);

        //Apply euler angle gains: 
        float phi_dot = euler_error[0]  * indi_gains_over.p.phi * gain_to_speed_constant;
        float theta_dot = euler_error[1]  * indi_gains_over.p.theta * gain_to_speed_constant;
        float psi_dot = euler_error[2]  * indi_gains_over.p.psi * gain_to_speed_constant;

        float phi_value = euler_vect[0];
        float theta_value = euler_vect[1];

        //Calculate the body error manually: 
        angular_body_error[0] = phi_dot - sin(theta_value) * psi_dot;
        angular_body_error[1] = cos(phi_value) * theta_dot + sin(phi_value) * cos(theta_value) * psi_dot;
        angular_body_error[2] = -sin(phi_value) * theta_dot + cos(phi_value) * cos(theta_value) * psi_dot;

        // rate_setpoint[0] = angular_body_error[0] * indi_gains_over.p.phi * gain_to_speed_constant;
        // rate_setpoint[1] = angular_body_error[1] * indi_gains_over.p.theta * gain_to_speed_constant;
        // rate_setpoint[2] = angular_body_error[2] * indi_gains_over.p.psi * gain_to_speed_constant;

        rate_setpoint[0] = angular_body_error[0] ;
        rate_setpoint[1] = angular_body_error[1] ;
        rate_setpoint[2] = angular_body_error[2] ;

        //Compute the angular acceleration setpoint using the filtered rates:
        acc_setpoint[3] = (rate_setpoint[0] - rate_vect_filt[0]) * indi_gains_over.d.phi * gain_to_speed_constant;
        acc_setpoint[4] = (rate_setpoint[1] - rate_vect_filt[1]) * indi_gains_over.d.theta * gain_to_speed_constant;
        acc_setpoint[5] = (rate_setpoint[2] - rate_vect_filt[2]) * indi_gains_over.d.psi * gain_to_speed_constant;

        // //Compute the angular acceleration setpoint using the unfiltered rates:
        // acc_setpoint[3] = (rate_setpoint[0] - rate_vect[0]) * indi_gains_over.d.phi;
        // acc_setpoint[4] = (rate_setpoint[1] - rate_vect[1]) * indi_gains_over.d.theta;
        // acc_setpoint[5] = (rate_setpoint[2] - rate_vect[2]) * indi_gains_over.d.psi;


        //Compute the acceleration error and save it to the INDI input array in the right position:
        // ANGULAR ACCELERATION
        INDI_pseudocontrol[3] = acc_setpoint[3] - rate_vect_filt_dot[0];
        INDI_pseudocontrol[4] = acc_setpoint[4] - rate_vect_filt_dot[1];
        INDI_pseudocontrol[5] = acc_setpoint[5] - rate_vect_filt_dot[2];

        //Calculate the speed error to be fed into the PD for the INDI loop
        pos_setpoint[0] = des_pos_earth_x;
        pos_setpoint[1] = des_pos_earth_y;
        if( abs(radio_control.values[RADIO_THROTTLE] - 4800) > deadband_stick_throttle ){
            pos_setpoint[2]  += - stick_gain_throttle * (radio_control.values[RADIO_THROTTLE] - 4800) * .00001;
        }
        Bound(pos_setpoint[2] ,-1000,1);

        pos_error[0] = pos_setpoint[0] - pos_vect[0];
        pos_error[1] = pos_setpoint[1] - pos_vect[1];
        pos_error[2] = pos_setpoint[2] - pos_vect[2];

        //Compute the speed setpoints in the control reference frame:
        speed_setpoint_control_rf[0] = - MANUAL_CONTROL_MAX_CMD_FWD_SPEED * radio_control.values[RADIO_PITCH]/9600.0;
        speed_setpoint_control_rf[1] = MANUAL_CONTROL_MAX_CMD_LAT_SPEED * radio_control.values[RADIO_ROLL]/9600.0;

        if( abs(radio_control.values[RADIO_THROTTLE] - 4800) > deadband_stick_throttle ) {
            speed_setpoint_control_rf[2] =
                    -MANUAL_CONTROL_MAX_CMD_VERT_SPEED * (radio_control.values[RADIO_THROTTLE] - 4800.0) / 4800.0;
        } else { speed_setpoint_control_rf[2] = 0; }

        // If wanted, use the position control for the altitude: 
        if(0){
            speed_setpoint_control_rf[2] = pos_error[2] * indi_gains_over.p.z;
        }
        
        //Add the setpoint in case of AUTO test
        if(auto_test_start){ 
            speed_setpoint_control_rf[0] = des_Vx; 
            speed_setpoint_control_rf[1] = des_Vy; 
            speed_setpoint_control_rf[2] = des_Vz; 
        }

        // If we are in the waypoint control mode, overwrite the speed setpoints with the one distated by the waypoint: 
        if(waypoint_mode){
            pos_setpoint[0] = x_stb; 
            pos_setpoint[1] = y_stb; 
            pos_setpoint[2] = z_stb;       
            compute_speed_ref_from_waypoint(speed_setpoint_control_rf, pos_setpoint, pos_vect, airspeed, euler_vect[2]);
        }


        //Apply saturation blocks to speed setpoints in control reference frame:
        Bound(speed_setpoint_control_rf[0],LIMITS_FWD_MIN_FWD_SPEED,LIMITS_FWD_MAX_FWD_SPEED);
        BoundAbs(speed_setpoint_control_rf[1],LIMITS_FWD_MAX_LAT_SPEED);
        BoundAbs(speed_setpoint_control_rf[2],LIMITS_FWD_MAX_VERT_SPEED);

        //Compute the speed error in the control rf:
        speed_error_vect_control_rf[0] = speed_setpoint_control_rf[0] - speed_vect_control_rf[0];
        speed_error_vect_control_rf[1] = speed_setpoint_control_rf[1] - speed_vect_control_rf[1] * (1 - compute_lat_speed_multiplier(OVERACTUATED_MIXING_MIN_SPEED_TRANSITION,OVERACTUATED_MIXING_REF_SPEED_TRANSITION,airspeed));
        speed_error_vect_control_rf[2] = speed_setpoint_control_rf[2] - speed_vect_control_rf[2];

        //Compute the acceleration setpoints in the control rf:
        acc_setpoint[0] = speed_error_vect_control_rf[0] * indi_gains_over.d.x;
        acc_setpoint[1] = speed_error_vect_control_rf[1] * indi_gains_over.d.y;
        acc_setpoint[2] = speed_error_vect_control_rf[2] * indi_gains_over.d.z;

        //Apply saturation points for the accelerations in the control rf:
        BoundAbs(acc_setpoint[0],LIMITS_FWD_MAX_FWD_ACC);
        BoundAbs(acc_setpoint[1],LIMITS_FWD_MAX_LAT_ACC);
        BoundAbs(acc_setpoint[2],LIMITS_FWD_MAX_VERT_ACC);

        //Compute the acceleration error and save it to the INDI input array in the right position:
        // LINEAR ACCELERATION IN CONTROL RF
        INDI_pseudocontrol[0] = acc_setpoint[0] - accel_vect_filt_control_rf[0];
        INDI_pseudocontrol[1] = acc_setpoint[1] - accel_vect_filt_control_rf[1];
        INDI_pseudocontrol[2] = acc_setpoint[2] - accel_vect_filt_control_rf[2];


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

        am7_data_out_local.phi_state_int = (int16_t) (actuator_state_filt[13] * 1e2 * 180/M_PI);
        am7_data_out_local.theta_state_int = (int16_t) (actuator_state_filt[12] * 1e2 * 180/M_PI);
        am7_data_out_local.ailerons_state_int = (int16_t) (actuator_state_filt[14] * 1e2 * 180/M_PI);

        am7_data_out_local.gamma_state_int = (int16_t) (flight_path_angle_filtered.o[0] * 1e2 * 180/M_PI);

        am7_data_out_local.p_state_int = (int16_t) (measurement_rates_filters[0].o[0] * 1e1 * 180/M_PI);
        am7_data_out_local.q_state_int = (int16_t) (measurement_rates_filters[1].o[0] * 1e1 * 180/M_PI);
        am7_data_out_local.r_state_int = (int16_t) (measurement_rates_filters[2].o[0] * 1e1 * 180/M_PI);

        am7_data_out_local.airspeed_state_int = (int16_t) (airspeed_filt.o[0] * 1e2);

        float fake_beta = 0;

        am7_data_out_local.beta_state_int = (int16_t) (fake_beta * 1e2);

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
        // am7_data_out_local.desired_theta_value_int = (int16_t) (0 * 1e2 * 180/M_PI);
        // am7_data_out_local.desired_phi_value_int = (int16_t) (0 * 1e2 * 180/M_PI);

        am7_data_out_local.desired_ailerons_value_int = (int16_t) (manual_ailerons_value * 1e2 * 180/M_PI);

        extra_data_out_local[0] = PROP_MODEL_KT_REF;
        extra_data_out_local[1] = PROP_MODEL_KM_REF;
        extra_data_out_local[2] = VEHICLE_MASS;
        extra_data_out_local[3] = VEHICLE_I_XX;
        extra_data_out_local[4] = VEHICLE_I_YY;
        extra_data_out_local[5] = VEHICLE_I_ZZ;
        extra_data_out_local[6] = VEHICLE_L1;
        extra_data_out_local[7] = VEHICLE_L2;
        extra_data_out_local[8] = VEHICLE_L3;
        extra_data_out_local[9] = VEHICLE_L4;
        extra_data_out_local[10] = VEHICLE_LZ;
        #ifdef RPM_CONTROL
        extra_data_out_local[11] = RPM_CONTROL_FBW_MAX_OMEGA_RAD_S;
        #else    
        extra_data_out_local[11] = OVERACTUATED_MIXING_MOTOR_MAX_OMEGA;
        #endif
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

        extra_data_out_local[46] = OVERACTUATED_MIXING_GAMMA_QUADRATIC_DU;

        extra_data_out_local[47] = VEHICLE_CY_BETA;
        extra_data_out_local[48] = VEHICLE_CL_BETA;
        extra_data_out_local[49] = VEHICLE_WING_SPAN;

        extra_data_out_local[50] = OVERACTUATED_MIXING_SPEED_AOA_PROTECTION;

        //Aileron addon: 
        extra_data_out_local[51] = OVERACTUATED_MIXING_W_ACT_AILERONS_CONST;
        extra_data_out_local[52] = OVERACTUATED_MIXING_W_ACT_AILERONS_SPEED;
        extra_data_out_local[53] = (OVERACTUATED_MIXING_MIN_DELTA_AILERONS * 180/M_PI);
        extra_data_out_local[54] = (OVERACTUATED_MIXING_MAX_DELTA_AILERONS * 180/M_PI);
        extra_data_out_local[55] = CL_ailerons ;

        //Inflow angle aerodynamic model: 
        extra_data_out_local[56] = PROP_MODEL_MAX_THR_LOSS_OPTIMIZER;
        extra_data_out_local[57] = PROP_MODEL_C_DR;

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

        indi_u[14] =  (myam7_data_in_local.ailerons_cmd_int * 1e-2 * M_PI/180);

        // Write values to servos and motor
        if(RadioControlValues(RADIO_TH_HOLD) > 0) {
            //Motors:
            #ifndef RPM_CONTROL
            overactuated_mixing.commands[0] = (int32_t)(indi_u[0] * K_ppz_rads_motor);
            overactuated_mixing.commands[1] = (int32_t)(indi_u[1] * K_ppz_rads_motor);
            overactuated_mixing.commands[2] = (int32_t)(indi_u[2] * K_ppz_rads_motor);
            overactuated_mixing.commands[3] = (int32_t)(indi_u[3] * K_ppz_rads_motor);
            #else //RPM CONTROL ON TEENSY
                //TESTING:
                #ifdef TEST_RPM_CONTROL

                indi_u[0] = (2 * M_PI) * Des_RPM_motor_1 / 60;
                indi_u[1] = 0;
                indi_u[2] = 0;
                indi_u[3] = 0;
                indi_u[4] = 0;
                indi_u[5] = 0;
                indi_u[6] = 0;
                indi_u[7] = 0;
                indi_u[8] = 0;
                indi_u[9] = 0;
                indi_u[10] = 0;
                indi_u[11] = 0;
                indi_u[12] = 0;
                indi_u[13] = 0;
                indi_u[14] = 0;

                #endif

                #ifdef TEST_DSHOT_CONTROL

                indi_u[0] = Des_dshot_steps_motor_1*600;
                indi_u[1] = 0;
                indi_u[2] = 0;
                indi_u[3] = 0;
                indi_u[4] = 0;
                indi_u[5] = 0;
                indi_u[6] = 0;
                indi_u[7] = 0;
                indi_u[8] = 0;
                indi_u[9] = 0;
                indi_u[10] = 0;
                indi_u[11] = 0;
                indi_u[12] = 0;
                indi_u[13] = 0;
                indi_u[14] = 0;

                #endif

                #ifdef TEST_PWM_SERVOS

                        indi_u[0] = 0;
                        indi_u[1] = 0;
                        indi_u[2] = 0;
                        indi_u[3] = 0;
                        indi_u[4] = 0;
                        indi_u[5] = 0;
                        indi_u[6] = 0;
                        indi_u[7] = 0;
                        indi_u[8] = 0;
                        indi_u[9] = 0;
                        indi_u[10] = 0;
                        indi_u[11] = 0;
                        indi_u[12] = 0;
                        indi_u[13] = 0;
                        indi_u[14] = 0;

                #endif  

                #ifdef TEST_DSHOT_CONTROL
                overactuated_mixing.commands[0] = (int32_t) (indi_u[0]);
                overactuated_mixing.commands[1] = (int32_t) (indi_u[1]);
                overactuated_mixing.commands[2] = (int32_t) (indi_u[2]);
                overactuated_mixing.commands[3] = (int32_t) (indi_u[3]);
                #endif 

                #ifdef TEST_ROTOR_ANGLES
                    indi_u[0] = 0;
                    indi_u[1] = 0;
                    indi_u[2] = 0;
                    indi_u[3] = 0;
                    indi_u[4] = des_el_angle_test*M_PI/180;
                    indi_u[5] = des_el_angle_test*M_PI/180;
                    indi_u[6] = des_el_angle_test*M_PI/180;
                    indi_u[7] = des_el_angle_test*M_PI/180;
                    indi_u[8] = des_az_angle_test*M_PI/180;
                    indi_u[9] = des_az_angle_test*M_PI/180;
                    indi_u[10] = des_az_angle_test*M_PI/180;
                    indi_u[11] = des_az_angle_test*M_PI/180;
                    indi_u[12] = 0;
                    indi_u[13] = 0;
                    indi_u[14] = 0;
                #endif

                #ifdef TEST_RASMUS_SERVO
                    if(get_sys_time_float() - time_old > 1.5 && get_sys_time_float() > 40){
                        test_frequency +=0.5;
                        time_old = get_sys_time_float();
                    }
                    indi_u[0] = 0;
                    indi_u[1] = 0;
                    indi_u[2] = 0;
                    indi_u[3] = 0;
                    indi_u[4] = 5*sin(get_sys_time_float()*test_frequency*2*M_PI)*M_PI/180;
                    indi_u[5] = 0;
                    indi_u[6] = 0;
                    indi_u[7] = 0;
                    indi_u[8] = 0;
                    indi_u[9] = 0;
                    indi_u[10] = 0;
                    indi_u[11] = 0;
                    indi_u[12] = 0;
                    indi_u[13] = 0;
                    indi_u[14] = 0;
                    actuator_output[12] = (int32_t) (test_frequency*100);
                #endif

                //END TESTING

                #ifndef TEST_DSHOT_CONTROL
                    //Apply first order filter to rotational speed to remove noise
                    for(int i = 0; i<4; i++){
                        motor_rad_s_filtered[i] = motor_rad_s_filtered[i] + RPM_CONTROL_FBW_FILT_FIRST_ORDER_RPM_COEFF * (actuator_state[i] - motor_rad_s_filtered[i]);
                    }

                    #ifdef RPM_INDI_CONTROL

                        for (int i = 0; i < 4; i++){
                            
                            //Apply the same filter on the cmd that the one of the rotational speed:
                            dshot_cmd_ppz_filtered[i] = dshot_cmd_ppz_filtered[i] + RPM_CONTROL_FBW_FILT_FIRST_ORDER_RPM_COEFF * (dshot_cmd_ppz[i] - dshot_cmd_ppz_filtered[i]);

                            //Shift the delayed cmd array:
                            for (int j = 0; j < RPM_CONTROL_FBW_MOTOR_DYN_DELAY_TS - 1; j++){
                                dshot_cmd_ppz_filtered_delayed[i][j] = dshot_cmd_ppz_filtered_delayed[i][j+1];
                            }
                            //Assign current value to the delayed array of state in the last position: 
                            dshot_cmd_ppz_filtered_delayed[i][RPM_CONTROL_FBW_MOTOR_DYN_DELAY_TS-1] = dshot_cmd_ppz_filtered[i];

                            //Estimate the cmd evolution based on the provided motor dynamics:
                            dshot_cmd_state_filtered[i] = dshot_cmd_state_filtered[i] + RPM_CONTROL_FBW_MOTOR_DYN_COEFF * (dshot_cmd_ppz_filtered_delayed[i][0] - dshot_cmd_state_filtered[i]);

                            //Compute the incremental ppz cmd for the motors:
                            dshot_cmd_ppz[i] = ( dshot_cmd_state_filtered[i] + K_indi_rad_s_dshot * (indi_u[i] - motor_rad_s_filtered[i]));

                            //Assign computed cmd to the output array: 
                            overactuated_mixing.commands[i] = (int32_t) (dshot_cmd_ppz[i]);

                        }

                    #endif

                    #ifndef RPM_INDI_CONTROL //PID RPM control

                    //Derivative term:
                    motor_rad_s_dot_filtered[0] = (motor_rad_s_filtered_old[0] - motor_rad_s_filtered_old[0])*PERIODIC_FREQUENCY_OVERACTUATED;
                    motor_rad_s_dot_filtered[1] = (motor_rad_s_filtered_old[1] - motor_rad_s_filtered_old[1])*PERIODIC_FREQUENCY_OVERACTUATED;
                    motor_rad_s_dot_filtered[2] = (motor_rad_s_filtered_old[2] - motor_rad_s_filtered_old[2])*PERIODIC_FREQUENCY_OVERACTUATED;
                    motor_rad_s_dot_filtered[3] = (motor_rad_s_filtered_old[3] - motor_rad_s_filtered_old[3])*PERIODIC_FREQUENCY_OVERACTUATED;

                    motor_rad_s_filtered_old[0] = motor_rad_s_filtered[0];
                    motor_rad_s_filtered_old[1] = motor_rad_s_filtered[1];
                    motor_rad_s_filtered_old[2] = motor_rad_s_filtered[2];
                    motor_rad_s_filtered_old[3] = motor_rad_s_filtered[3];

                    //Integral term:
                    motor_rad_s_error_integrated[0] += indi_u[0] - motor_rad_s_filtered[0];
                    motor_rad_s_error_integrated[1] += indi_u[1] - motor_rad_s_filtered[1];
                    motor_rad_s_error_integrated[2] += indi_u[2] - motor_rad_s_filtered[2];
                    motor_rad_s_error_integrated[3] += indi_u[3] - motor_rad_s_filtered[3];

                    //Feed fwd term
                    float feed_fwd_term_rpm[4];
                    feed_fwd_term_rpm[0] = 9600 * indi_u[0] / OVERACTUATED_MIXING_MOTOR_MAX_OMEGA;
                    feed_fwd_term_rpm[1] = 9600 * indi_u[1] / OVERACTUATED_MIXING_MOTOR_MAX_OMEGA;
                    feed_fwd_term_rpm[2] = 9600 * indi_u[2] / OVERACTUATED_MIXING_MOTOR_MAX_OMEGA;
                    feed_fwd_term_rpm[3] = 9600 * indi_u[3] / OVERACTUATED_MIXING_MOTOR_MAX_OMEGA;

                    overactuated_mixing.commands[0] = (int32_t) (feed_fwd_term_rpm[0] + (indi_u[0] - motor_rad_s_filtered[0]) * K_p_rad_s_dshot + 
                                                                 motor_rad_s_error_integrated[0] * K_i_rad_s_dshot - 
                                                                 motor_rad_s_dot_filtered[0] * K_d_rad_s_dshot);
                    overactuated_mixing.commands[1] = (int32_t) (feed_fwd_term_rpm[1] + (indi_u[1] - motor_rad_s_filtered[1]) * K_p_rad_s_dshot + 
                                                                 motor_rad_s_error_integrated[1] * K_i_rad_s_dshot - 
                                                                 motor_rad_s_dot_filtered[1] * K_d_rad_s_dshot);
                    overactuated_mixing.commands[2] = (int32_t) (feed_fwd_term_rpm[2] + (indi_u[2] - motor_rad_s_filtered[2]) * K_p_rad_s_dshot + 
                                                                motor_rad_s_error_integrated[2] * K_i_rad_s_dshot - 
                                                                motor_rad_s_dot_filtered[2] * K_d_rad_s_dshot);
                    overactuated_mixing.commands[3] = (int32_t) (feed_fwd_term_rpm[3] + (indi_u[3] - motor_rad_s_filtered[3]) * K_p_rad_s_dshot + 
                                                                 motor_rad_s_error_integrated[3] * K_i_rad_s_dshot - 
                                                                 motor_rad_s_dot_filtered[3] * K_d_rad_s_dshot);

                    #endif

                #endif

            #endif             

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

        //Add servos values:
        roll_pwm_cmd = indi_u[14] * OVERACTUATED_MIXING_AILERONS_K_PWM_ANGLE;
        servo_right_cmd = neutral_servo_1_pwm - roll_pwm_cmd;
        servo_left_cmd = neutral_servo_2_pwm - roll_pwm_cmd;

        Bound(servo_right_cmd,1000,2000);
        Bound(servo_left_cmd,1000,2000);

        //Submit value to external servo: 
        am7_data_out_local.pwm_servo_1_int = (int16_t) (indi_u[14]*18000/M_PI);
        am7_data_out_local.pwm_servo_2_int = (int16_t) (indi_u[14]*18000/M_PI);
        
        roll_cmd = servo_right_cmd;
        pitch_cmd = servo_left_cmd;

    }

    //Send to the raspberry pi the values for the next optimization run.
    AbiSendMsgAM7_DATA_OUT(ABI_AM7_DATA_OUT_ID, &am7_data_out_local, extra_data_out_local);

    #ifdef FBW_ACTUATORS
        //Pre compute the actuator values to be sent to the FBW T4: 
        //Motor cmds
        if(RadioControlValues(RADIO_TH_HOLD) > 0) {
            //Arm motor:
            myserial_act_t4_out_local.motor_arm_int = 1;
            //Bound motor values before submitting:
            Bound(overactuated_mixing.commands[0], 0, MAX_PPRZ);
            Bound(overactuated_mixing.commands[1], 0, MAX_PPRZ);
            Bound(overactuated_mixing.commands[2], 0, MAX_PPRZ);
            Bound(overactuated_mixing.commands[3], 0, MAX_PPRZ);
            //Motors cmds:
            myserial_act_t4_out_local.motor_1_dshot_cmd_int =  (int16_t) (overactuated_mixing.commands[0] * MAX_DSHOT_VALUE / 9600.0); 
            myserial_act_t4_out_local.motor_2_dshot_cmd_int =  (int16_t) (overactuated_mixing.commands[1] * MAX_DSHOT_VALUE / 9600.0); 
            myserial_act_t4_out_local.motor_3_dshot_cmd_int =  (int16_t) (overactuated_mixing.commands[2] * MAX_DSHOT_VALUE / 9600.0); 
            myserial_act_t4_out_local.motor_4_dshot_cmd_int =  (int16_t) (overactuated_mixing.commands[3] * MAX_DSHOT_VALUE / 9600.0); 
            //Aileron servos PWM cmd: 
            myserial_act_t4_out_local.servo_9_cmd_int = (int16_t) (indi_u[14] * 100.0 * 180.0/M_PI );
            myserial_act_t4_out_local.servo_10_cmd_int = (int16_t) (indi_u[14] * 100.0 * 180.0/M_PI );     
        }
        else{
            //Disarm motor:
            myserial_act_t4_out_local.motor_arm_int = 0;
            //Motors cmds:
            myserial_act_t4_out_local.motor_1_dshot_cmd_int =  (int16_t) (0); 
            myserial_act_t4_out_local.motor_2_dshot_cmd_int =  (int16_t) (0); 
            myserial_act_t4_out_local.motor_3_dshot_cmd_int =  (int16_t) (0); 
            myserial_act_t4_out_local.motor_4_dshot_cmd_int =  (int16_t) (0); 
            //Aileron servos cmd:
            myserial_act_t4_out_local.servo_9_cmd_int = (int16_t) (10 * 100); //10 degrees up 
            myserial_act_t4_out_local.servo_10_cmd_int = (int16_t) (10 * 100); //10 degrees down ;  
        }

        // Bound actuator values for the FBW system before submitting:
        Bound(overactuated_mixing.commands[4],(OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE)*K_ppz_angle_el,(OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE)*K_ppz_angle_el);
        Bound(overactuated_mixing.commands[5],(OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE)*K_ppz_angle_el,(OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE)*K_ppz_angle_el);
        Bound(overactuated_mixing.commands[6],(OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE)*K_ppz_angle_el,(OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE)*K_ppz_angle_el);
        Bound(overactuated_mixing.commands[7],(OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE)*K_ppz_angle_el,(OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE)*K_ppz_angle_el);

        Bound(overactuated_mixing.commands[8],(OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE)*K_ppz_angle_az,(OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE)*K_ppz_angle_az);
        Bound(overactuated_mixing.commands[9],(OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE)*K_ppz_angle_az,(OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE)*K_ppz_angle_az);
        Bound(overactuated_mixing.commands[10],(OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE)*K_ppz_angle_az,(OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE)*K_ppz_angle_az);
        Bound(overactuated_mixing.commands[11],(OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE)*K_ppz_angle_az,(OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE)*K_ppz_angle_az);
        
        //Elevator servos cmd: 
        myserial_act_t4_out_local.servo_2_cmd_int = (int16_t) ( ( overactuated_mixing.commands[4] / K_ppz_angle_el ) * 100 * FBW_T4_K_RATIO_GEAR_EL * 180/M_PI );
        myserial_act_t4_out_local.servo_6_cmd_int = (int16_t) ( ( overactuated_mixing.commands[5] / K_ppz_angle_el ) * 100 * FBW_T4_K_RATIO_GEAR_EL * 180/M_PI );
        myserial_act_t4_out_local.servo_8_cmd_int = (int16_t) ( ( overactuated_mixing.commands[6] / K_ppz_angle_el ) * 100 * FBW_T4_K_RATIO_GEAR_EL * 180/M_PI );
        myserial_act_t4_out_local.servo_4_cmd_int = (int16_t) ( ( overactuated_mixing.commands[7] / K_ppz_angle_el ) * 100 * FBW_T4_K_RATIO_GEAR_EL * 180/M_PI );
        //Azimuth servos cmd: 
        myserial_act_t4_out_local.servo_1_cmd_int = (int16_t) ( ( overactuated_mixing.commands[8] / K_ppz_angle_az ) * 100 * FBW_T4_K_RATIO_GEAR_AZ * 180/M_PI );
        myserial_act_t4_out_local.servo_5_cmd_int = (int16_t) ( ( overactuated_mixing.commands[9] / K_ppz_angle_az ) * 100 * FBW_T4_K_RATIO_GEAR_AZ * 180/M_PI );
        myserial_act_t4_out_local.servo_7_cmd_int = (int16_t) ( ( -overactuated_mixing.commands[10] / K_ppz_angle_az ) * 100 * FBW_T4_K_RATIO_GEAR_AZ * 180/M_PI );
        myserial_act_t4_out_local.servo_3_cmd_int = (int16_t) ( ( -overactuated_mixing.commands[11] / K_ppz_angle_az ) * 100 * FBW_T4_K_RATIO_GEAR_AZ * 180/M_PI );

        #ifdef TEST_PWM_SERVOS
            myserial_act_t4_out_local.servo_9_cmd_int = (int16_t) (desired_angle_servo_9 * 100);
            myserial_act_t4_out_local.servo_10_cmd_int = (int16_t) (desired_angle_servo_10 * 100);
        #endif

        //Assign extra data: 
        serial_act_t4_extra_data_out_local[0] = FBW_T4_AILERONS_FIRST_ORD_DEN;
        serial_act_t4_extra_data_out_local[1] = FBW_T4_AILERONS_FIRST_ORD_NUM;
        
        serial_act_t4_extra_data_out_local[2] = max_pwm_servo_9;
        serial_act_t4_extra_data_out_local[3] = min_pwm_servo_9;
        serial_act_t4_extra_data_out_local[4] = neutral_pwm_servo_9;
        serial_act_t4_extra_data_out_local[5] = FBW_T4_SERVO_9_MIN_ANGLE_DEG;
        serial_act_t4_extra_data_out_local[6] = FBW_T4_SERVO_9_MAX_ANGLE_DEG;
        serial_act_t4_extra_data_out_local[7] = FBW_T4_SERVO_9_DELAY_TS;
  
        serial_act_t4_extra_data_out_local[8] = FBW_T4_AILERONS_FIRST_ORD_DEN;
        serial_act_t4_extra_data_out_local[9] = FBW_T4_AILERONS_FIRST_ORD_NUM;  
        
        serial_act_t4_extra_data_out_local[10] = max_pwm_servo_10;
        serial_act_t4_extra_data_out_local[11] = min_pwm_servo_10;
        serial_act_t4_extra_data_out_local[12] = neutral_pwm_servo_10;
        serial_act_t4_extra_data_out_local[13] = FBW_T4_SERVO_10_MIN_ANGLE_DEG;
        serial_act_t4_extra_data_out_local[14] = FBW_T4_SERVO_10_MAX_ANGLE_DEG;
        serial_act_t4_extra_data_out_local[15] = FBW_T4_SERVO_10_DELAY_TS;

        //SEND MESSAGE:
        AbiSendMsgSERIAL_ACT_T4_OUT(ABI_SERIAL_ACT_T4_OUT_ID, &myserial_act_t4_out_local, &serial_act_t4_extra_data_out_local[0]);

    #endif

    //Bound values if we are not in FBW mode:
    #ifndef FBW_ACTUATORS
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
    #endif

    actuator_output[0] = (int32_t) (overactuated_mixing.commands[0] / K_ppz_rads_motor);
    actuator_output[1] = (int32_t) (overactuated_mixing.commands[1] / K_ppz_rads_motor);
    actuator_output[2] = (int32_t) (overactuated_mixing.commands[2] / K_ppz_rads_motor);
    actuator_output[3] = (int32_t) (overactuated_mixing.commands[3] / K_ppz_rads_motor);

    actuator_output[4] = (int32_t) ((overactuated_mixing.commands[4] / K_ppz_angle_el + OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE) * 18000/M_PI);
    actuator_output[5] = (int32_t) ((overactuated_mixing.commands[5] / K_ppz_angle_el + OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE) * 18000/M_PI);
    actuator_output[6] = (int32_t) ((overactuated_mixing.commands[6] / K_ppz_angle_el + OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE) * 18000/M_PI);
    actuator_output[7] = (int32_t) ((overactuated_mixing.commands[7] / K_ppz_angle_el + OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE) * 18000/M_PI);

    actuator_output[8] = (int32_t) ((overactuated_mixing.commands[8] / K_ppz_angle_az + OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE) * 18000/M_PI);
    actuator_output[9] = (int32_t) ((overactuated_mixing.commands[9] / K_ppz_angle_az + OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE) * 18000/M_PI);
    actuator_output[10] = (int32_t) ((overactuated_mixing.commands[10] / K_ppz_angle_az + OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE) * 18000/M_PI);
    actuator_output[11] = (int32_t) ((overactuated_mixing.commands[11] / K_ppz_angle_az + OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE) * 18000/M_PI);

    //Actuator state message:
    actuator_state_int[0] = (int32_t) (actuator_state[0]);
    actuator_state_int[1] = (int32_t) (actuator_state[1]);
    actuator_state_int[2] = (int32_t) (actuator_state[2]);
    actuator_state_int[3] = (int32_t) (actuator_state[3]);

    actuator_state_int[4] = (int32_t) (actuator_state[4] * 18000.0/M_PI);
    actuator_state_int[5] = (int32_t) (actuator_state[5] * 18000.0/M_PI);
    actuator_state_int[6] = (int32_t) (actuator_state[6] * 18000.0/M_PI);
    actuator_state_int[7] = (int32_t) (actuator_state[7] * 18000.0/M_PI);

    actuator_state_int[8] = (int32_t) (actuator_state[8] * 18000.0/M_PI);
    actuator_state_int[9] = (int32_t) (actuator_state[9] * 18000.0/M_PI);
    actuator_state_int[10] = (int32_t) (actuator_state[10] * 18000.0/M_PI);
    actuator_state_int[11] = (int32_t) (actuator_state[11] * 18000.0/M_PI);

    //Ailerons
    actuator_state_int[12] = (int32_t) (actuator_state[14] * 18000.0/M_PI);

}

