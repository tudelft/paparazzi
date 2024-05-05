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

#define max_time_optimizer_outer_loop 2.45e-3
#define max_time_optimizer_inner_loop 2.45e-3
#define max_iterations_outer_loop 70
#define max_iterations_inner_loop 70
#define max_function_eval_outer_loop 200
#define max_function_eval_inner_loop 200
#define refresh_time_optimizer 5e-3 //Must be equal or bigger than max_time_optimizer
#define filter_cutoff_frequency_init 12 //rad/s

//Define the baudrate for the module and the starting byte 
#define START_BYTE 0x9B  //1st start block identifier byte
#define BAUDRATE_AM7 921600 //Define the baudrate
// #define MAX_FREQUENCY_MSG_OUT 513 //Define the maximum message output frequency
#define MAX_FREQUENCY_MSG_OUT 550 //Define the maximum message output frequency

#define BAUDRATE_TF_MINI 115200 //Baudrate of the TF mini lidar sensor

#define NUM_ACT_IN_U_IN 15
#define NUM_ACT_IN_U_OUT 15

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
    uint16_t n_iteration_outer;
    uint16_t n_iteration_inner;
    uint16_t n_evaluation_outer;
    uint16_t n_evaluation_inner;
    uint16_t elapsed_time_us;
    int16_t exit_flag_optimizer_inner;
    //Modeled acc filtered
    int16_t modeled_ax_int;
    int16_t modeled_ay_int;
    int16_t modeled_az_int;
    int16_t modeled_p_dot_int;
    int16_t modeled_q_dot_int;
    int16_t modeled_r_dot_int;
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
    //Actuator state - unfiltered
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
    //Variable states - unfiltered 
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
    //Pseudo-control cmd - unfiltered
    int16_t pseudo_control_ax_int;
    int16_t pseudo_control_ay_int;
    int16_t pseudo_control_az_int;
    int16_t ax_state_int;
    int16_t ay_state_int;
    int16_t az_state_int;
    //Error Controller variables:
    int16_t psi_dot_cmd_int;
    int16_t p_dot_state_int;
    int16_t q_dot_state_int;
    int16_t r_dot_state_int;
    int16_t p_state_filt_int;
    int16_t q_state_filt_int;
    int16_t r_state_filt_int;
    //Desired theta and phi value:
    int16_t desired_theta_value_int;
    int16_t desired_phi_value_int;
    //UAV position NED: 
    float UAV_NED_pos_x;
    float UAV_NED_pos_y;
    float UAV_NED_pos_z;
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
