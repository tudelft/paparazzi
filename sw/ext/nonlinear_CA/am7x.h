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

//Outer loop settings
#define NUM_ACT_IN_U_IN 15
#define max_time_outer_loop 5e-3
#define max_iterations_outer_loop 100
#define max_function_eval_outer_loop 300
#define refresh_time_outer_loop 5e-3 // It should be higher or equal to max_time_outer_loop

//Inner loop settings
#define NUM_ACT_IN_U_IN_INNER 13
#define max_time_inner_loop 5e-3
#define max_iterations_inner_loop 100
#define max_function_eval_inner_loop 300
#define refresh_time_inner_loop 5e-3 // It should be higher or equal to max_time_inner_loop

//Filter settings
#define refresh_time_filters 5e-3 //200 Hz 
#define filter_cutoff_frequency_init 12.0 //rad/s - second order butterworth filter
#define filter_cutoff_first_order_pqr_init 20.0 //rad/s - first order filter

//Define the baudrate for the module and the starting byte 
#define START_BYTE 0x9B
#define BAUDRATE_AM7 921600

//Define the maximum message output frequency
#define MAX_FREQUENCY_MSG_OUT 550

// Define the baudrate of the TF mini lidar sensor
#define BAUDRATE_TF_MINI 115200

//Deifne the length of the output array of the inner loop
#define NUM_ACT_IN_U_OUT 13

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
    uint16_t n_evaluation_outer;
    uint16_t elapsed_time_us_outer;
    int16_t exit_flag_optimizer_outer;
    uint16_t n_iteration_inner;
    uint16_t n_evaluation_inner;
    uint16_t elapsed_time_us_inner;
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
    int8_t aruco_system_status;
    //Sixdof infos: 
    float sixdof_detection_timestamp;
    float sixdof_NED_pos_x;
    float sixfod_NED_pos_y;
    float sixdof_NED_pos_z;
    int8_t sixdof_system_status;
    //Rolling_msg
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;
	uint8_t checksum_out;
};

struct  __attribute__((__packed__)) am7_data_in {
    //Actuator state - unfiltered
    int16_t motor_1_state_int; //rad/s * 10
    int16_t motor_2_state_int; //rad/s * 10
    int16_t motor_3_state_int; //rad/s * 10
    int16_t motor_4_state_int; //rad/s * 10
    int16_t el_1_state_int; //degrees * 100
    int16_t el_2_state_int; //degrees * 100
    int16_t el_3_state_int; //degrees * 100
    int16_t el_4_state_int; //degrees * 100
    int16_t az_1_state_int; //degrees * 100
    int16_t az_2_state_int; //degrees * 100
    int16_t az_3_state_int; //degrees * 100
    int16_t az_4_state_int; //degrees * 100
    int16_t ailerons_state_int; //degrees * 100
    //Variable states - unfiltered 
    int16_t theta_state_int; //degrees * 100
    int16_t phi_state_int; //degrees * 100
    int16_t psi_state_int; //degrees * 100
    int16_t gamma_state_int; //degrees * 100
    int16_t airspeed_state_int; //m/s * 100
    int16_t beta_state_int; //degrees * 100
    //Body rates - unfiltered
    int16_t p_state_int; //degrees/sec value * 10
    int16_t q_state_int; //degrees/sec value * 10
    int16_t r_state_int; //degrees/sec value * 10
    //pqr_dot filtered
    int16_t p_dot_filt_int; //degrees/sec^2 value * 10
    int16_t q_dot_filt_int; //degrees/sec^2 value * 10
    int16_t r_dot_filt_int; //degrees/sec^2 value * 10
    int16_t psi_dot_cmd_int; //degrees value * 100 
    //Approach boolean and lidar corrected altitude for the rotor constraint application 
    int16_t approach_boolean; //Boolean value
    int16_t lidar_alt_corrected_int; //meters * 100
    //Pseudo-control increments linear (from filtered accelerations)
    int16_t pseudo_control_ax_int; //m/s^2 * 100
    int16_t pseudo_control_ay_int; //m/s^2 * 100
    int16_t pseudo_control_az_int; //m/s^2 * 100
    //Desired theta and phi value:
    int16_t desired_theta_value_int; //degrees * 100
    int16_t desired_phi_value_int; //degrees * 100
    //UAV position NED: 
    float UAV_NED_pos_x; //meters
    float UAV_NED_pos_y; //meters
    float UAV_NED_pos_z; //meters
    //Failure info: 
    uint8_t failure_mode; //0 -> no failure; 1 -> Rotor 1 fail; 2 -> Rotor 2 fail; 3 -> Rotor 3 fail; 4 -> Rotor 4 fail; 
    //Rolling msg
    float rolling_msg_in;
    uint8_t rolling_msg_in_id;  
	uint8_t checksum_in;
};

struct __attribute__((__packed__)) marker_detection_t {
    float timestamp_detection; 
    float NED_pos_x; 
    float NED_pos_y;
    float NED_pos_z;
    int8_t system_status; 
};

struct __attribute__((__packed__)) outer_loop_output {
    double Theta_cmd_rad;
    double Phi_cmd_rad;
    double p_dot_cmd_rad_s;
    double q_dot_cmd_rad_s;
    double r_dot_cmd_rad_s;
    double residual_ax;
    double residual_ay;
    double residual_az;
    double residual_p_dot;
    double residual_q_dot;
    double residual_r_dot;
    double exit_flag; 
    double n_iterations;
    double n_evaluations;
    double elapsed_time;
    double acc_decrement_aero_ax; 
    double acc_decrement_aero_ay;
    double acc_decrement_aero_az;
    double acc_decrement_aero_p_dot;
    double acc_decrement_aero_q_dot;
    double acc_decrement_aero_r_dot;
}

struct __attribute__((__packed__)) data_in_optimizer {
    //Real time data in, filtered with the indi filters: 
    float motor_1_state_filtered;
    float motor_2_state_filtered;
    float motor_3_state_filtered;
    float motor_4_state_filtered;
    float el_1_state_filtered;
    float el_2_state_filtered;
    float el_3_state_filtered;
    float el_4_state_filtered;
    float az_1_state_filtered;
    float az_2_state_filtered;
    float az_3_state_filtered;
    float az_4_state_filtered;
    float ailerons_state_filtered;
    float theta_state_filtered;
    float phi_state_filtered;
    float psi_state_filtered;
    float gamma_state_filtered;
    float p_state_filtered;
    float q_state_filtered;
    float r_state_filtered;
    float theta_state_ec; 
    float phi_state_ec;
    float psi_dot_cmd_ec;
    float p_state_ec;
    float q_state_ec;
    float r_state_ec;
    float p_dot_state_ec;
    float q_dot_state_ec;
    float r_dot_state_ec;
    float airspeed_state_filtered;
    float beta_state_filtered;
    float approach_boolean;
    float lidar_alt_corrected;
    float pseudo_control_ax;
    float pseudo_control_ay;
    float pseudo_control_az;
    float desired_theta_value;
    float desired_phi_value;

    //Failure: 
    float failure_mode; 

    //Computed filtered modeled accellerations 
    float modeled_ax_filtered;
    float modeled_ay_filtered;
    float modeled_az_filtered;
    float modeled_p_dot_filtered;
    float modeled_q_dot_filtered;
    float modeled_r_dot_filtered;

    //Non-real-time data:
    float K_p_T;
    float K_p_M;
    float m;
    float I_xx;
    float I_yy;
    float I_zz;
    float l_1;
    float l_2;
    float l_3;
    float l_4;
    float l_z;
    float max_omega;
    float min_omega;
    float max_b;
    float min_b;
    float max_g;
    float min_g;
    float max_theta;
    float min_theta;
    float max_alpha;
    float min_alpha;
    float max_phi;
    float Cm_zero;
    float Cm_alpha;
    float Cl_alpha;
    float Cd_zero;
    float K_Cd;
    float S;
    float wing_chord;
    float rho;
    float W_act_motor_const;
    float W_act_motor_speed;
    float W_act_tilt_el_const;
    float W_act_tilt_el_speed;
    float W_act_tilt_az_const;
    float W_act_tilt_az_speed;
    float W_act_theta_const;
    float W_act_theta_speed;
    float W_act_phi_const;
    float W_act_phi_speed;
    float W_dv_1;
    float W_dv_2;
    float W_dv_3;
    float W_dv_4;
    float W_dv_5;
    float W_dv_6;
    float gamma_quadratic_du;
    float Cy_beta;
    float Cl_beta;
    float wing_span;
    float aoa_protection_speed;
    float W_act_ailerons_const;
    float W_act_ailerons_speed;
    float min_delta_ailerons;
    float max_delta_ailerons;
    float CL_aileron;
    float k_alt_tilt_constraint;
    float min_alt_tilt_constraint;
    float transition_speed;
    float desired_motor_value;
    float desired_el_value;
    float desired_az_value;
    float desired_ailerons_value;
    float theta_gain;
    float phi_gain;
    float p_body_gain;
    float q_body_gain;
    float r_body_gain;
    float k_d_airspeed;
    float min_theta_hard;
    float max_theta_hard;
    float min_phi_hard;
    float max_phi_hard;
    float disable_acc_decrement_inner_loop;
    float filter_cutoff_frequency_telem;
    float max_airspeed;
    float vert_acc_margin;
    float power_Cd_0;
    float power_Cd_a;
    float prop_R;
    float prop_Cd_0;
    float prop_Cl_0;
    float prop_Cd_a;
    float prop_Cl_a;
    float prop_delta;
    float prop_sigma;
    float prop_theta;
    float beacon_tracking_id;
    float desired_sixdof_mode;
    float use_u_init_outer_loop;
    float use_u_init_inner_loop;
};
    
#endif
