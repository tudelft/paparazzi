#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <sys/time.h>
#include <math.h>
#include "MATLAB_generated_files/rtwtypes.h"
#include <stddef.h>

#ifndef AM7_H
#define AM7_H

#define first_opt_iter_max_time 2.4e-3
#define second_opt_iter_max_time 4.9e-3

#define refresh_time_optimizer 5e-3 //Must be equal or bigger than second_opt_iter_max_time
#define filter_cutoff_frequency 12 //rad/s

//Define the baudrate for the module and the starting byte 
#define START_BYTE 0x9B  //1st start block identifier byte
#define BAUDRATE_AM7 921600 //Define the baudrate
// #define MAX_FREQUENCY_MSG_OUT 513 //Define the maximum message output frequency
#define MAX_FREQUENCY_MSG_OUT 550 //Define the maximum message output frequency

#define BAUDRATE_TF_MINI 115200 //Baudrate of the TF mini lidar sensor

//Communication structures
struct  __attribute__((__packed__)) am7_data_out {
    //Motor command
	int16_t motor_1_cmd_int;
	int16_t motor_2_cmd_int;
	int16_t motor_3_cmd_int;
	int16_t motor_4_cmd_int;
	int16_t el_1_cmd_int;
	int16_t el_2_cmd_int;
	int16_t el_3_cmd_int;
    int16_t el_4_cmd_int;
    int16_t az_1_cmd_int;
    int16_t az_2_cmd_int;
    int16_t az_3_cmd_int;
    int16_t az_4_cmd_int;
    int16_t theta_cmd_int;
    int16_t phi_cmd_int;
    int16_t ailerons_cmd_int;
    //Optimization info
    uint16_t n_iteration;
    uint16_t n_evaluation;
    uint16_t elapsed_time_us;
    int16_t exit_flag_optimizer;
    //Residuals
    int16_t residual_ax_int;
    int16_t residual_ay_int;
    int16_t residual_az_int;
    int16_t residual_p_dot_int;
    int16_t residual_q_dot_int;
    int16_t residual_r_dot_int;
    //Lidar status
    int16_t lidar_value_cm; 
    int16_t lidar_strength; 
    //Aruco infos: 
    float aruco_detection_timestamp;
    float aruco_NED_pos_x;
    float aruco_NED_pos_y;
    float aruco_NED_pos_z;
    //Rolling_msg
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;
	uint8_t checksum_out;
};

struct  __attribute__((__packed__)) am7_data_in {
    //Actuator state
    int16_t motor_1_state_int;
    int16_t motor_2_state_int;
    int16_t motor_3_state_int;
    int16_t motor_4_state_int;
    int16_t el_1_state_int;
    int16_t el_2_state_int;
    int16_t el_3_state_int;
    int16_t el_4_state_int;
    int16_t az_1_state_int;
    int16_t az_2_state_int;
    int16_t az_3_state_int;
    int16_t az_4_state_int;
    int16_t ailerons_state_int;
    //Variable states
    int16_t theta_state_int;
    int16_t phi_state_int;
    int16_t psi_state_int;
    int16_t gamma_state_int;
    int16_t p_state_int;
    int16_t q_state_int;
    int16_t r_state_int;
    int16_t airspeed_state_int;
    int16_t beta_state_int;
    //Approach boolean and lidar corrected altitude for the rotor constraint application 
    int16_t approach_boolean; 
    int16_t lidar_alt_corrected_int;
    //Pseudo-control cmd
    int16_t pseudo_control_ax_int;
    int16_t pseudo_control_ay_int;
    int16_t pseudo_control_az_int;
    int16_t pseudo_control_p_dot_int;
    int16_t pseudo_control_q_dot_int;
    int16_t pseudo_control_r_dot_int;
    //Desired actuator value:
    int16_t desired_motor_value_int;
    int16_t desired_el_value_int;
    int16_t desired_az_value_int;
    int16_t desired_theta_value_int;
    int16_t desired_phi_value_int;
    int16_t desired_ailerons_value_int;
    //UAV position NED: 
    float UAV_NED_pos_x;
    float UAV_NED_pos_y;
    float UAV_NED_pos_z;
    //Extra variables for the cascaded Nonlinear CA
    int16_t p_body_current_int; //degrees/sec value * 10
    int16_t q_body_current_int; //degrees/sec value * 10
    int16_t r_body_current_int; //degrees/sec value * 10
    int16_t p_dot_current_int; //degrees/sec^2 value * 10
    int16_t q_dot_current_int; //degrees/sec^2 value * 10
    int16_t r_dot_current_int; //degrees/sec^2 value * 10
    int16_t theta_current_int; //degrees value * 100
    int16_t phi_current_int; //value * 100 
    int16_t theta_gain_int; //value * 100 
    int16_t phi_gain_int; //value * 100 
    int16_t p_body_gain_int; //value * 100 
    int16_t q_body_gain_int; //value * 100 
    int16_t r_body_gain_int; //value * 100 
    int16_t des_psi_dot_int; //degrees value * 100 
    //Rolling msg
    float rolling_msg_in;
    uint8_t rolling_msg_in_id;  
	uint8_t checksum_in;
};

struct aruco_detection_t {
    float timestamp_detection; 
    float NED_pos_x; 
    float NED_pos_y;
    float NED_pos_z;
};

#endif
