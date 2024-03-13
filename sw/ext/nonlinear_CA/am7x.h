#include <stdio.h>
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

#define max_time_optimizer 4.9e-3

#define refresh_time_optimizer 5e-3 //Must be equal or bigger than optimizer_max_time
#define filter_cutoff_frequency 13 //rad/s

//Define the baudrate for the module and the starting byte 
#define START_BYTE 0x9B  //1st start block identifier byte
#define BAUDRATE_AM7 460800 //Define the baudrate
#define MAX_FREQUENCY_MSG_OUT 550 //Define the maximum message output frequency

#define SEND_MODELED_ACC_ON_RESIDUALS

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
    uint16_T n_iteration;
    uint16_T n_evaluation;
    uint16_T elapsed_time_us;
    int16_t exit_flag_optimizer;
    //Residuals
    int16_t residual_ax_int;
    int16_t residual_ay_int;
    int16_t residual_az_int;
    int16_t residual_p_dot_int;
    int16_t residual_q_dot_int;
    int16_t residual_r_dot_int;
    float rolling_msg_out;
    uint8_T rolling_msg_out_id;
	uint8_T checksum_out;
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
    int16_t gamma_state_int;
    int16_t p_state_int;
    int16_t q_state_int;
    int16_t r_state_int;
    int16_t airspeed_state_int;
    int16_t beta_state_int;
    //Extra servos messages 
    int16_t pwm_servo_1_int;
    int16_t pwm_servo_2_int;   
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
    float rolling_msg_in;
    uint8_T rolling_msg_in_id;  
	uint8_T checksum_in;
};

#endif
