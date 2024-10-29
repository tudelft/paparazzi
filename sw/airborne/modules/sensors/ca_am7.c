/*
 * Copyright (C) 2022 OpenUAS
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
 * @file "modules/ca_am7.c"
 * @author Alessandro Mancinelli
 */

#include "modules/sensors/ca_am7.h"
#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include <time.h>
#include <sys/time.h>
#include "modules/core/abi.h"
#include "modules/sensors/serial_act_t4.h"
#include "generated/flight_plan.h"
#include "state.h"
#include "modules/overactuated_vehicle/overactuated_mixing.h"

static abi_event AM7_receive_from_modules;
uint8_t sending_msg_id;
struct am7_data_in myam7_data_in;
struct am7_data_out myam7_data_out;
float extra_data_in[255]__attribute__((aligned)), extra_data_out[255]__attribute__((aligned));
uint16_t buffer_in_counter = 0;
uint32_t missed_packets = 0;
uint16_t ca7_message_frequency_RX = 0;
uint32_t received_packets = 0;
float last_ts = 0;
static uint8_t am7_msg_buf_in[sizeof(struct am7_data_in)*2]  __attribute__((aligned));   

//Filters variables: 
float tau_body_rates_filter; 
float p_filtered, q_filtered, r_filtered, p_old, q_old, r_old;
Butterworth2LowPass body_p_dot_second_order_filter;
Butterworth2LowPass body_q_dot_second_order_filter;
Butterworth2LowPass body_r_dot_second_order_filter;
float p_dot_filtered, q_dot_filtered, r_dot_filtered;

//Struct to store the received data from the other modules:
struct am7_data_t received_am7_data, my_am7_data; 
//Assign default am7_data_t structure, to use in case no module sends data:
struct am7_data_t default_am7_data = {
    .packet_timestamp = 0,
    //Pseudocontrols:
    .pseudocontrol_ax = 0,
    .pseudocontrol_ay = 0,
    .pseudocontrol_az = 0,
    .pseudocontrol_p_dot = 0,
    .pseudocontrol_q_dot = 0,
    .pseudocontrol_r_dot = 0,
    //Estimated variables:
    .est_flight_path_angle_rad = 0,
    .est_airspeed = 0,
    .est_beta_rad = 0,
    //Psi dot command:
    .psi_dot_cmd_rad_s = 0,
    //Desired variables:
    .desired_motor_rad_s = 0,
    .desired_el_rad = 0,
    .desired_az_rad = 0,
    .desired_theta_rad = 0,
    .desired_phi_rad = 0,
    .desired_ail_rad = 0,
};

//Gains and waight matrices for the control law:
float w_mot_const, w_mot_speed, w_el_const, w_el_speed, w_az_const, w_az_speed, w_theta_const, w_theta_speed, w_phi_const, w_phi_speed, w_ail_const, w_ail_speed;
float w_dv_1, w_dv_2, w_dv_3, w_dv_4, w_dv_5, w_dv_6;
float gamma_quadratic_du;
static bool gains_changed_app = false, gains_changed_cruise = false; 

//Decision variables and miscellaneous: 
uint8_t failure_mode = 0;
float aoa_protection_speed, transition_speed, min_speed_transition, ref_speed_transition, k_gain_airspeed; 
float vert_acc_margin = AM7_SETTINGS_VERT_ACC_MARGIN;
float K_T_airspeed = AM7_SETTINGS_VEHICLE_MOTOR_K_T_AIRSPEED; 
int disable_acc_decrement_inner_loop = AM7_SETTINGS_DISABLE_ACC_DECREMENT_INNER_LOOP;
int use_u_init_outer_loop = AM7_SETTINGS_USE_U_INIT_OUTER_LOOP;
int use_u_init_inner_loop = AM7_SETTINGS_USE_U_INIT_INNER_LOOP;
int single_loop_controller = AM7_SETTINGS_SINGLE_LOOP_CONTROLLER;
int use_new_aero_model = AM7_SETTINGS_USE_NEW_AERO_MODEL;
int use_received_ang_ref_in_inner_loop = AM7_SETTINGS_USE_RECEIVED_ANG_REF_IN_INNER_LOOP;

//SIXDOF variables: 
int sixdof_mode = 1; 
int selected_beacon = 2; 
float alt_offset_beacon = 5.0f; 

//Propeller model: 
float power_cd_0 = PROP_MODEL_POWER_CD_0;
float power_cd_a = PROP_MODEL_POWER_CD_A;
float prop_r = PROP_MODEL_PROP_R;
float prop_cd_0 = PROP_MODEL_PROP_CD_0;
float prop_cl_0 = PROP_MODEL_PROP_CL_0;
float prop_cd_a = PROP_MODEL_PROP_CD_A;
float prop_cl_a = PROP_MODEL_PROP_CL_A;
float prop_delta = PROP_MODEL_PROP_DELTA;
float prop_sigma = PROP_MODEL_PROP_SIGMA;
float prop_theta = PROP_MODEL_PROP_THETA;

//Extra variables: 
float lidar_alt;
static abi_event get_lidar_value_ev;

#if PERIODIC_TELEMETRY
    #include "modules/datalink/telemetry.h"
    static void am7_downlink(struct transport_tx *trans, struct link_device *dev)
    {
        float motors_cmd_float_telemetry[4] = {myam7_data_in.motor_1_cmd_int*0.1f,
                                              myam7_data_in.motor_2_cmd_int*0.1f,
                                              myam7_data_in.motor_3_cmd_int*0.1f,
                                              myam7_data_in.motor_4_cmd_int*0.1f};

        float elevation_tilt_cmd_float_telemetry[4] = {myam7_data_in.el_1_cmd_int*0.01f,
                                                       myam7_data_in.el_2_cmd_int*0.01f,
                                                       myam7_data_in.el_3_cmd_int*0.01f,
                                                       myam7_data_in.el_4_cmd_int*0.01f};

        float azimuth_tilt_cmd_float_telemetry[4] = {myam7_data_in.az_1_cmd_int*0.01f,
                                                     myam7_data_in.az_2_cmd_int*0.01f,
                                                     myam7_data_in.az_3_cmd_int*0.01f,
                                                     myam7_data_in.az_4_cmd_int*0.01f};

        float theta_cmd_float_telemetry = myam7_data_in.theta_cmd_int*0.01f;
        float phi_cmd_float_telemetry = myam7_data_in.phi_cmd_int*0.01f;

        float ailerons_cmd_float_telemetry = myam7_data_in.ailerons_cmd_int*0.01f;

        uint16_t n_iteration_outer_telemetry = myam7_data_in.n_iteration_outer;
        uint16_t n_evaluation_outer_telemetry = myam7_data_in.n_evaluation_outer;
        uint16_t elapsed_time_us_outer_telemetry = myam7_data_in.elapsed_time_us_outer;
        int16_t exit_flag_optimizer_outer_telemetry = myam7_data_in.exit_flag_optimizer_outer;

        uint16_t n_iteration_inner_telemetry = myam7_data_in.n_iteration_inner;
        uint16_t n_evaluation_inner_telemetry = myam7_data_in.n_evaluation_inner;
        uint16_t elapsed_time_us_inner_telemetry = myam7_data_in.elapsed_time_us_inner;
        int16_t exit_flag_optimizer_inner_telemetry = myam7_data_in.exit_flag_optimizer_inner;

        float modeled_acc_float_telemetry[6] = {myam7_data_in.modeled_ax_int*0.01f,
                                                    myam7_data_in.modeled_ay_int*0.01f,
                                                    myam7_data_in.modeled_az_int*0.01f,
                                                    myam7_data_in.modeled_p_dot_int*0.1f,
                                                    myam7_data_in.modeled_q_dot_int*0.1f,
                                                    myam7_data_in.modeled_r_dot_int*0.1f};

        float residuals_array_float_telemetry[6] = {myam7_data_in.residual_ax_int*0.01f,
                                                    myam7_data_in.residual_ay_int*0.01f,
                                                    myam7_data_in.residual_az_int*0.01f,
                                                    myam7_data_in.residual_p_dot_int*0.1f,
                                                    myam7_data_in.residual_q_dot_int*0.1f,
                                                    myam7_data_in.residual_r_dot_int*0.1f};



        float lidar_altitude_m_float_telemetry = myam7_data_in.lidar_value_cm*0.01;
        int16_t lidar_strength_telemetry = myam7_data_in.lidar_strength;

        float aruco_detection_timestamp_telemetry = myam7_data_in.aruco_detection_timestamp;
        float NED_aruco_pos_x_telemetry = myam7_data_in.aruco_NED_pos_x; 
        float NED_aruco_pos_y_telemetry = myam7_data_in.aruco_NED_pos_y; 
        float NED_aruco_pos_z_telemetry = myam7_data_in.aruco_NED_pos_z; 
        float NED_aruco_relative_phi_telemetry = (float) (myam7_data_in.aruco_relative_phi*0.01f);
        float NED_aruco_relative_theta_telemetry = (float) (myam7_data_in.aruco_relative_theta*0.01f);
        float NED_aruco_relative_psi_telemetry = (float) (myam7_data_in.aruco_relative_psi*0.01f);
        int8_t aruco_sys_status_telemetry = myam7_data_in.aruco_system_status;

        float sixdof_detection_timestamp_telemetry = myam7_data_in.sixdof_detection_timestamp;
        float NED_sixdof_pos_x_telemetry = myam7_data_in.sixdof_NED_pos_x;
        float NED_sixdof_pos_y_telemetry = myam7_data_in.sixdof_NED_pos_y;
        float NED_sixdof_pos_z_telemetry = myam7_data_in.sixdof_NED_pos_z;
        float NED_sixdof_relative_phi_telemetry = (float) (myam7_data_in.sixdof_relative_phi*0.01f);
        float NED_sixdof_relative_theta_telemetry = (float) (myam7_data_in.sixdof_relative_theta*0.01f);
        float NED_sixdof_relative_psi_telemetry = (float) (myam7_data_in.sixdof_relative_psi*0.01f);
        int8_t sixdof_sys_status_telemetry = myam7_data_in.sixdof_system_status;

        float rolling_msg_in_telemetry = myam7_data_in.rolling_msg_in;
        uint8_t rolling_msg_in_id_telemetry = myam7_data_in.rolling_msg_in_id;

        pprz_msg_send_AM7_IN(trans, dev, AC_ID, &motors_cmd_float_telemetry[0],
                &elevation_tilt_cmd_float_telemetry[0], &azimuth_tilt_cmd_float_telemetry[0],
                &theta_cmd_float_telemetry, &phi_cmd_float_telemetry,
                &ailerons_cmd_float_telemetry,
                &missed_packets, &ca7_message_frequency_RX,
                &lidar_altitude_m_float_telemetry, &lidar_strength_telemetry,
                &aruco_detection_timestamp_telemetry, &NED_aruco_pos_x_telemetry, &NED_aruco_pos_y_telemetry, &NED_aruco_pos_z_telemetry,
                &NED_aruco_relative_phi_telemetry, &NED_aruco_relative_theta_telemetry, &NED_aruco_relative_psi_telemetry, &aruco_sys_status_telemetry,
                &sixdof_detection_timestamp_telemetry, &NED_sixdof_pos_x_telemetry, &NED_sixdof_pos_y_telemetry, &NED_sixdof_pos_z_telemetry,
                &NED_sixdof_relative_phi_telemetry, &NED_sixdof_relative_theta_telemetry, &NED_sixdof_relative_psi_telemetry, &sixdof_sys_status_telemetry,
                &modeled_acc_float_telemetry[0],
                &residuals_array_float_telemetry[0],
                &n_iteration_outer_telemetry, &n_evaluation_outer_telemetry, &elapsed_time_us_outer_telemetry, &exit_flag_optimizer_outer_telemetry,
                &n_iteration_inner_telemetry, &n_evaluation_inner_telemetry, &elapsed_time_us_inner_telemetry, &exit_flag_optimizer_inner_telemetry,
                &rolling_msg_in_telemetry, &rolling_msg_in_id_telemetry);
    }
    static void am7_uplink(struct transport_tx *trans, struct link_device *dev)
    {
        float motors_state_float_telemetry[4] = {myam7_data_out.motor_1_state_int*0.1f,
                                                myam7_data_out.motor_2_state_int*0.1f,
                                                myam7_data_out.motor_3_state_int*0.1f,
                                                myam7_data_out.motor_4_state_int*0.1f};

        float tilt_elevation_state_float_telemetry[4] = {myam7_data_out.el_1_state_int*0.01f,
                                                         myam7_data_out.el_2_state_int*0.01f,
                                                         myam7_data_out.el_3_state_int*0.01f,
                                                         myam7_data_out.el_4_state_int*0.01f};

        float tilt_azimuth_state_float_telemetry[4] = {myam7_data_out.az_1_state_int*0.01f,
                                                       myam7_data_out.az_2_state_int*0.01f,
                                                       myam7_data_out.az_3_state_int*0.01f,
                                                       myam7_data_out.az_4_state_int*0.01f};

        //Variable states and ailerons
        int16_t theta_state_int_telemetry = myam7_data_out.theta_state_int;
        int16_t phi_state_int_telemetry = myam7_data_out.phi_state_int;
        int16_t psi_state_int_telemetry = myam7_data_out.psi_state_int;

        int16_t ailerons_state_int_telemetry = myam7_data_out.ailerons_state_int;

        int16_t gamma_state_int_telemetry = myam7_data_out.gamma_state_int;
        int16_t airspeed_state_int_telemetry = myam7_data_out.airspeed_state_int;
        int16_t beta_state_int_telemetry = myam7_data_out.beta_state_int;

        int16_t p_state_int_telemetry = myam7_data_out.p_state_int;
        int16_t q_state_int_telemetry = myam7_data_out.q_state_int;
        int16_t r_state_int_telemetry = myam7_data_out.r_state_int;

        int16_t p_dot_filt_int_telemetry = myam7_data_out.p_dot_filt_int;
        int16_t q_dot_filt_int_telemetry = myam7_data_out.q_dot_filt_int;
        int16_t r_dot_filt_int_telemetry = myam7_data_out.r_dot_filt_int;

        int16_t psi_dot_cmd_int_telemetry = myam7_data_out.psi_dot_cmd_int;
        
        //Approach boolean and lidar corrected altitude for the rotor constraint application 
        int16_t approach_boolean_telemetry = myam7_data_out.approach_boolean;
        int16_t lidar_alt_corrected_int_telemetry = myam7_data_out.lidar_alt_corrected_int;       

        //Pseudo-control cmd and unfiltered linear accelleration readings
        int16_t pseudo_control_ax_int_telemetry = myam7_data_out.pseudo_control_ax_int;
        int16_t pseudo_control_ay_int_telemetry = myam7_data_out.pseudo_control_ay_int;
        int16_t pseudo_control_az_int_telemetry = myam7_data_out.pseudo_control_az_int;

        int16_t pseudo_control_p_dot_telemetry = myam7_data_out.pseudo_control_p_dot_int;
        int16_t pseudo_control_q_dot_telemetry = myam7_data_out.pseudo_control_q_dot_int;
        int16_t pseudo_control_r_dot_telemetry = myam7_data_out.pseudo_control_r_dot_int;

        //Desired actuator value:
        int16_t desired_theta_value_int_telemetry = myam7_data_out.desired_theta_value_int;
        int16_t desired_phi_value_int_telemetry = myam7_data_out.desired_phi_value_int;

        //UAV POSITION:
        float UAV_NED_pos_x_telemetry = myam7_data_out.UAV_NED_pos_x;
        float UAV_NED_pos_y_telemetry = myam7_data_out.UAV_NED_pos_y;
        float UAV_NED_pos_z_telemetry = myam7_data_out.UAV_NED_pos_z;

        float rolling_msg_out_telemetry = myam7_data_out.rolling_msg_out;
        uint8_t rolling_msg_out_id_telemetry = myam7_data_out.rolling_msg_out_id;

        pprz_msg_send_AM7_OUT(trans, dev, AC_ID, &motors_state_float_telemetry[0], &tilt_elevation_state_float_telemetry[0],
                &tilt_azimuth_state_float_telemetry[0], &theta_state_int_telemetry, &phi_state_int_telemetry, &psi_state_int_telemetry,
                &ailerons_state_int_telemetry,

                &gamma_state_int_telemetry, &airspeed_state_int_telemetry, &beta_state_int_telemetry,
                &p_state_int_telemetry, &q_state_int_telemetry, &r_state_int_telemetry,

                &p_dot_filt_int_telemetry, &q_dot_filt_int_telemetry, &r_dot_filt_int_telemetry,

                &psi_dot_cmd_int_telemetry,

                &approach_boolean_telemetry, &lidar_alt_corrected_int_telemetry,

                &pseudo_control_ax_int_telemetry, &pseudo_control_ay_int_telemetry, &pseudo_control_az_int_telemetry,
                &pseudo_control_p_dot_telemetry, &pseudo_control_q_dot_telemetry, &pseudo_control_r_dot_telemetry,

                &desired_theta_value_int_telemetry, &desired_phi_value_int_telemetry,

                &UAV_NED_pos_x_telemetry, &UAV_NED_pos_y_telemetry, &UAV_NED_pos_z_telemetry,

                &failure_mode,

                &rolling_msg_out_telemetry, &rolling_msg_out_id_telemetry);
    }
#endif

/*Function to receive the data from the other modules using the ABI communication: */
static void AM7_receive_from_module_fcn(uint8_t sender_id __attribute__((unused)), struct am7_data_t * myam7_data_t_ptr){
    memcpy(&received_am7_data,myam7_data_t_ptr,sizeof(struct am7_data_t));
}

/**ABI callback that obtains lidar corrected AGL altitude */
static void get_lidar_alt(uint8_t sender_id __attribute__((unused)), uint32_t timestamp_lidar, float distance_lidar_meter)
{   
    timestamp_lidar = timestamp_lidar;
    //Capy distance to global variable: 
    lidar_alt = distance_lidar_meter;
}

/*Assign the data to the struct and prepare extra_data_out array: */
void assign_am7_data(void){ 

    //First, retrieve the data from the other modules and assign the values to the myam7_data_out struct:
    //If the command is older than 0.5 seconds, then use the default command:
    if(get_sys_time_float() - received_am7_data.packet_timestamp > 0.5f){
        memcpy(&my_am7_data, &default_am7_data, sizeof(struct am7_data_t));
    }
    else{
        memcpy(&my_am7_data, &received_am7_data, sizeof(struct am7_data_t));
    }

    //Start filling the myam7_data_out struct with the data from the other modules:
    myam7_data_out.pseudo_control_ax_int = (int16_t) (my_am7_data.pseudocontrol_ax * 1e2);
    myam7_data_out.pseudo_control_ay_int = (int16_t) (my_am7_data.pseudocontrol_ay * 1e2);
    myam7_data_out.pseudo_control_az_int = (int16_t) (my_am7_data.pseudocontrol_az * 1e2);
    myam7_data_out.pseudo_control_p_dot_int = (int16_t) (my_am7_data.pseudocontrol_p_dot * 1e1 * 180/M_PI);
    myam7_data_out.pseudo_control_q_dot_int = (int16_t) (my_am7_data.pseudocontrol_q_dot * 1e1 * 180/M_PI);
    myam7_data_out.pseudo_control_r_dot_int = (int16_t) (my_am7_data.pseudocontrol_r_dot * 1e1 * 180/M_PI);
    myam7_data_out.gamma_state_int = (int16_t) (my_am7_data.est_flight_path_angle_rad * 1e2 * 180/M_PI);
    myam7_data_out.airspeed_state_int = (int16_t) (my_am7_data.est_airspeed * 1e2);
    myam7_data_out.beta_state_int = (int16_t) (my_am7_data.est_beta_rad * 1e2);
    myam7_data_out.psi_dot_cmd_int = (int16_t) (my_am7_data.psi_dot_cmd_rad_s * 1e2 * 180/M_PI);
    myam7_data_out.desired_theta_value_int = (int16_t) (my_am7_data.desired_theta_rad * 1e2 * 180/M_PI);
    myam7_data_out.desired_phi_value_int = (int16_t) (my_am7_data.desired_phi_rad * 1e2 * 180/M_PI);
    

    //Add the data retrived from the FBW module: 
    myam7_data_out.motor_1_state_int = (int16_t) (get_act_states_T4()->motor_1_rad_s_filt * 1e1);
    myam7_data_out.motor_2_state_int = (int16_t) (get_act_states_T4()->motor_2_rad_s_filt * 1e1);
    myam7_data_out.motor_3_state_int = (int16_t) (get_act_states_T4()->motor_3_rad_s_filt * 1e1);
    myam7_data_out.motor_4_state_int = (int16_t) (get_act_states_T4()->motor_4_rad_s_filt * 1e1);
    myam7_data_out.el_1_state_int = (int16_t) (get_act_states_T4()->el_1_angle_deg_corrected * 1e2);
    myam7_data_out.el_2_state_int = (int16_t) (get_act_states_T4()->el_2_angle_deg_corrected * 1e2);
    myam7_data_out.el_3_state_int = (int16_t) (get_act_states_T4()->el_3_angle_deg_corrected * 1e2);
    myam7_data_out.el_4_state_int = (int16_t) (get_act_states_T4()->el_4_angle_deg_corrected * 1e2);
    myam7_data_out.az_1_state_int = (int16_t) (get_act_states_T4()->az_1_angle_deg_corrected * 1e2);
    myam7_data_out.az_2_state_int = (int16_t) (get_act_states_T4()->az_2_angle_deg_corrected * 1e2);
    myam7_data_out.az_3_state_int = (int16_t) (get_act_states_T4()->az_3_angle_deg_corrected * 1e2);
    myam7_data_out.az_4_state_int = (int16_t) (get_act_states_T4()->az_4_angle_deg_corrected * 1e2);
    myam7_data_out.ailerons_state_int = (int16_t) ((get_act_states_T4()->flaperon_left_angle_deg + get_act_states_T4()->flaperon_right_angle_deg)/2 * 1e2);

    //Add states retrieved from autopilot functions or from the filters: 
    myam7_data_out.theta_state_int = (int16_t) (stateGetNedToBodyEulers_f()->theta * 1e2 * 180/M_PI);
    myam7_data_out.phi_state_int = (int16_t) (stateGetNedToBodyEulers_f()->phi * 1e2 * 180/M_PI);
    myam7_data_out.psi_state_int = (int16_t) (stateGetNedToBodyEulers_f()->psi * 1e2 * 180/M_PI);
    myam7_data_out.p_state_int = (int16_t) (stateGetBodyRates_f()->p * 1e1 * 180/M_PI);
    myam7_data_out.q_state_int = (int16_t) (stateGetBodyRates_f()->q * 1e1 * 180/M_PI);
    myam7_data_out.r_state_int = (int16_t) (stateGetBodyRates_f()->r * 1e1 * 180/M_PI);
    myam7_data_out.p_dot_filt_int = (int16_t) (p_dot_filtered * 1e1 * 180/M_PI);
    myam7_data_out.q_dot_filt_int = (int16_t) (q_dot_filtered * 1e1 * 180/M_PI);
    myam7_data_out.r_dot_filt_int = (int16_t) (r_dot_filtered * 1e1 * 180/M_PI);

    myam7_data_out.failure_mode = (int16_t) (failure_mode);
    myam7_data_out.approach_boolean = (int16_t) (approach_state);
    myam7_data_out.lidar_alt_corrected_int = (int16_t) (lidar_alt * 1e2);
    myam7_data_out.UAV_NED_pos_x = (float) (stateGetPositionNed_f()->x);
    myam7_data_out.UAV_NED_pos_y = (float) (stateGetPositionNed_f()->y);
    myam7_data_out.UAV_NED_pos_z = (float) (stateGetPositionNed_f()->z);

    //Add the variables in the extra_data_out array:
    extra_data_out[0] = AM7_SETTINGS_VEHICLE_MOTOR_K_T_OMEGASQ;
    extra_data_out[1] = AM7_SETTINGS_VEHICLE_MOTOR_K_M_OMEGASQ;
    extra_data_out[2] = AM7_SETTINGS_VEHICLE_MASS;
    extra_data_out[3] = AM7_SETTINGS_VEHICLE_I_XX;
    extra_data_out[4] = AM7_SETTINGS_VEHICLE_I_YY;
    extra_data_out[5] = AM7_SETTINGS_VEHICLE_I_ZZ;
    extra_data_out[6] = AM7_SETTINGS_VEHICLE_L1;
    extra_data_out[7] = AM7_SETTINGS_VEHICLE_L2;
    extra_data_out[8] = AM7_SETTINGS_VEHICLE_L3;
    extra_data_out[9] = AM7_SETTINGS_VEHICLE_L4;
    extra_data_out[10] = AM7_SETTINGS_VEHICLE_LZ;
    extra_data_out[11] = AM7_SETTINGS_MOTOR_MAX_OMEGA_RAD_S;
    extra_data_out[12] = AM7_SETTINGS_MOTOR_MIN_OMEGA_RAD_S;
    extra_data_out[13] = AM7_SETTINGS_SERVO_EL_MAX_ANGLE_DEG;
    extra_data_out[14] = AM7_SETTINGS_SERVO_EL_MIN_ANGLE_DEG;
    extra_data_out[15] = AM7_SETTINGS_SERVO_AZ_MAX_ANGLE_DEG;
    extra_data_out[16] = AM7_SETTINGS_SERVO_AZ_MIN_ANGLE_DEG;
    extra_data_out[17] = AM7_SETTINGS_MAX_THETA_DEG;
    extra_data_out[18] = AM7_SETTINGS_MIN_THETA_DEG;
    extra_data_out[19] = AM7_SETTINGS_MAX_AOA_DEG;
    extra_data_out[20] = AM7_SETTINGS_MIN_AOA_DEG;
    extra_data_out[21] = AM7_SETTINGS_MAX_PHI_DEG;
    extra_data_out[22] = AM7_SETTINGS_VEHICLE_CM_ZERO;
    extra_data_out[23] = AM7_SETTINGS_VEHICLE_CM_ALPHA;
    extra_data_out[24] = AM7_SETTINGS_VEHICLE_CL_ALPHA;
    extra_data_out[25] = AM7_SETTINGS_VEHICLE_CD_ZERO;
    extra_data_out[26] = AM7_SETTINGS_VEHICLE_K_CD;
    extra_data_out[27] = AM7_SETTINGS_VEHICLE_S;
    extra_data_out[28] = AM7_SETTINGS_VEHICLE_WING_CHORD;
    extra_data_out[29] = 1.225; //rho value at MSL
    extra_data_out[30] = w_mot_const;
    extra_data_out[31] = w_mot_speed;
    extra_data_out[32] = w_el_const;
    extra_data_out[33] = w_el_speed;
    extra_data_out[34] = w_az_const;
    extra_data_out[35] = w_az_speed;
    extra_data_out[36] = w_theta_const;
    extra_data_out[37] = w_theta_speed;
    extra_data_out[38] = w_phi_const;
    extra_data_out[39] = w_phi_speed;
    extra_data_out[40] = w_dv_1;
    extra_data_out[41] = w_dv_2;
    extra_data_out[42] = w_dv_3;
    extra_data_out[43] = w_dv_4;
    extra_data_out[44] = w_dv_5;
    extra_data_out[45] = w_dv_6;
    extra_data_out[46] = gamma_quadratic_du * 1e-8;

    extra_data_out[47] = AM7_SETTINGS_VEHICLE_CY_BETA;
    extra_data_out[48] = AM7_SETTINGS_VEHICLE_CL_BETA;
    extra_data_out[49] = AM7_SETTINGS_VEHICLE_WING_SPAN;

    extra_data_out[50] = aoa_protection_speed;

    //Aileron addon: 
    extra_data_out[51] = w_ail_const;
    extra_data_out[52] = w_ail_speed;
    extra_data_out[53] = AM7_SETTINGS_MIN_DELTA_AILERONS_DEG;
    extra_data_out[54] = AM7_SETTINGS_MAX_DELTA_AILERONS_DEG;
    extra_data_out[55] = AM7_SETTINGS_VEHICLE_CL_AILERONS;

    //Approach tilting angle constraint: 
    extra_data_out[56] = AM7_SETTINGS_K_ALT_TILT_CONSTRAINT;     
    extra_data_out[57] = AM7_SETTINGS_MIN_ALT_TILT_CONSTRAINT;   

    extra_data_out[58] = transition_speed;  
    
    extra_data_out[59] = my_am7_data.desired_motor_rad_s;  
    extra_data_out[60] = my_am7_data.desired_el_rad;  
    extra_data_out[61] = my_am7_data.desired_az_rad; 
    extra_data_out[62] = my_am7_data.desired_ail_rad;
    
    extra_data_out[63] = active_gains.p.theta;
    extra_data_out[64] = active_gains.p.phi;
    extra_data_out[65] = active_gains.d.theta;
    extra_data_out[66] = active_gains.d.phi;
    extra_data_out[67] = active_gains.d.psi;
    extra_data_out[68] = k_gain_airspeed;

    extra_data_out[69] = -OVERACTUATED_GAINS_MAX_THETA;
    extra_data_out[70] = OVERACTUATED_GAINS_MAX_THETA;
    extra_data_out[71] = -OVERACTUATED_GAINS_MAX_PHI;
    extra_data_out[72] = OVERACTUATED_GAINS_MAX_PHI;

    extra_data_out[73] = disable_acc_decrement_inner_loop;
    extra_data_out[74] = AM7_SETTINGS_INDI_SECOND_ORDER_CUTOFF_RAD_S;
    extra_data_out[75] = max_airspeed_am7;
    extra_data_out[76] = vert_acc_margin;

    extra_data_out[77] = power_cd_0;
    extra_data_out[78] = power_cd_a;
    extra_data_out[79] = prop_r;
    extra_data_out[80] = prop_cd_0;
    extra_data_out[81] = prop_cl_0;
    extra_data_out[82] = prop_cd_a;
    extra_data_out[83] = prop_cl_a;
    extra_data_out[84] = prop_delta;
    extra_data_out[85] = prop_sigma;
    extra_data_out[86] = prop_theta;

    if(selected_beacon == 1){
        extra_data_out[87] = 1640.0;     
    }
    if(selected_beacon == 2){
        extra_data_out[87] = 1636.0;     
    }
    if(selected_beacon == 3){
        extra_data_out[87] = 1645.0;     
    }
    if(selected_beacon == 4){
        extra_data_out[87] = 1633.0;     
    }
    if(selected_beacon == 5){
        extra_data_out[87] = 1632.0;     
    }
    
    extra_data_out[88] = sixdof_mode; 

    extra_data_out[89] = use_u_init_outer_loop;
    extra_data_out[90] = use_u_init_inner_loop;

    extra_data_out[91] = K_T_airspeed;

    extra_data_out[92] = AM7_SETTINGS_BODY_RATES_FIRST_ORDER_CUTOFF_RAD_S;

    extra_data_out[93] = single_loop_controller;
    extra_data_out[94] = use_new_aero_model;
    extra_data_out[95] = use_received_ang_ref_in_inner_loop;
    
}

/*Update the filters: */
void update_am7_filters(void){
    //Update body rates dot filters:
    update_butterworth_2_low_pass(&body_p_dot_second_order_filter, (float) ((stateGetBodyRates_f()->p - p_old) * (float) AM7_FREQUENCY));
    update_butterworth_2_low_pass(&body_q_dot_second_order_filter, (float) ((stateGetBodyRates_f()->q - q_old) * (float) AM7_FREQUENCY));
    update_butterworth_2_low_pass(&body_r_dot_second_order_filter, (float) ((stateGetBodyRates_f()->r - r_old) * (float) AM7_FREQUENCY));
    //Update the old values:
    p_old = stateGetBodyRates_f()->p;
    q_old = stateGetBodyRates_f()->q;
    r_old = stateGetBodyRates_f()->r;
    //Assign the filtered body rates derivative to the variables:
    p_dot_filtered = body_p_dot_second_order_filter.o[0];
    q_dot_filtered = body_q_dot_second_order_filter.o[0];
    r_dot_filtered = body_r_dot_second_order_filter.o[0];
}

/*Send the message over serial to the Raspberry pi: */
void send_am7_packet_over_serial(void){
    //Increase the counter to track the sending messages:
    myam7_data_out.rolling_msg_out = extra_data_out[sending_msg_id];
    myam7_data_out.rolling_msg_out_id = sending_msg_id;
    sending_msg_id++;
    if(sending_msg_id == 255){
        sending_msg_id = 0;
    }

    //Send the message over serial to the Raspberry pi:
    uint8_t *buf_send = (uint8_t *)&myam7_data_out;
    //Calculating the checksum
    uint8_t checksum_out_local = 0;
    for(uint16_t i = 0; i < sizeof(struct am7_data_out) - 1; i++){
        checksum_out_local += buf_send[i];
    }
    myam7_data_out.checksum_out = checksum_out_local;
    //Send bytes
    uart_put_byte(&(AM7_PORT), 0, START_BYTE);
    for(uint8_t i = 0; i < sizeof(struct am7_data_out) ; i++){
        uart_put_byte(&(AM7_PORT), 0, buf_send[i]);
    }
}

/*Update the tfmini lidar through the am7 module: */
void tfmini_lidar_update(void){
    int16_t raw_value_lidar_cm = myam7_data_in.lidar_value_cm;
    if(raw_value_lidar_cm >= 0){
        // compensate AGL measurement for body rotation
        float phi = stateGetNedToBodyEulers_f()->phi;
        float theta = stateGetNedToBodyEulers_f()->theta;
        float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
        float tf_mini_distance_compensated = raw_value_lidar_cm * gain * 0.01f;

        uint32_t now_ts = get_sys_time_usec();
        AbiSendMsgAGL(AGL_LIDAR_TFMINI_ID, now_ts, tf_mini_distance_compensated);
    }
}

/*Update the gains and weights based on the approach state: */
void update_gains_weights(void){
    //Assign gains according to the approach state: 
    if(approach_state && gains_changed_app == false){
        //Booleans to avoid changing the gains multiple times, to allow the sliders to be used:
        gains_changed_app = true;
        gains_changed_cruise = false;

        aoa_protection_speed = AM7_SETTINGS_SPEED_AOA_PROTECTION_APP;
        transition_speed = AM7_SETTINGS_TRANSITION_SPEED_APP;
        min_speed_transition = AM7_SETTINGS_MIN_SPEED_TRANSITION_APP;
        ref_speed_transition = AM7_SETTINGS_REF_SPEED_TRANSITION_APP;

        w_mot_const = AM7_SETTINGS_W_ACT_MOTOR_CONST_APP; 
        w_mot_speed = AM7_SETTINGS_W_ACT_MOTOR_SPEED_APP; 
        w_el_const = AM7_SETTINGS_W_ACT_EL_CONST_APP; 
        w_el_speed = AM7_SETTINGS_W_ACT_EL_SPEED_APP; 
        w_az_const = AM7_SETTINGS_W_ACT_AZ_CONST_APP; 
        w_az_speed = AM7_SETTINGS_W_ACT_AZ_SPEED_APP;  
        w_theta_const = AM7_SETTINGS_W_ACT_THETA_CONST_APP; 
        w_theta_speed = AM7_SETTINGS_W_ACT_THETA_SPEED_APP; 
        w_phi_const = AM7_SETTINGS_W_ACT_PHI_CONST_APP; 
        w_phi_speed = AM7_SETTINGS_W_ACT_PHI_SPEED_APP;
        w_ail_const = AM7_SETTINGS_W_ACT_AILERONS_CONST_APP; 
        w_ail_speed = AM7_SETTINGS_W_ACT_AILERONS_SPEED_APP;
        w_dv_1 = AM7_SETTINGS_W_DV_1_APP;
        w_dv_2 = AM7_SETTINGS_W_DV_2_APP;
        w_dv_3 = AM7_SETTINGS_W_DV_3_APP;
        w_dv_4 = AM7_SETTINGS_W_DV_4_APP;
        w_dv_5 = AM7_SETTINGS_W_DV_5_APP;
        w_dv_6 = AM7_SETTINGS_W_DV_6_APP;
        gamma_quadratic_du = AM7_SETTINGS_GAMMA_QUADRATIC_DU_APP_e_minus_8;
        k_gain_airspeed = AM7_SETTINGS_K_GAIN_AIRSPEED_APP;
    }

    if(approach_state == 0 && gains_changed_cruise == false){
        //Booleans to avoid changing the gains multiple times, to allow the sliders to be used:
        gains_changed_cruise = true;
        gains_changed_app = false;

        aoa_protection_speed = AM7_SETTINGS_SPEED_AOA_PROTECTION_CRUISE;
        transition_speed = AM7_SETTINGS_TRANSITION_SPEED_CRUISE;
        min_speed_transition = AM7_SETTINGS_MIN_SPEED_TRANSITION_CRUISE;
        ref_speed_transition = AM7_SETTINGS_REF_SPEED_TRANSITION_CRUISE;

        w_mot_const = AM7_SETTINGS_W_ACT_MOTOR_CONST_CRUISE;
        w_mot_speed = AM7_SETTINGS_W_ACT_MOTOR_SPEED_CRUISE;
        w_el_const = AM7_SETTINGS_W_ACT_EL_CONST_CRUISE;
        w_el_speed = AM7_SETTINGS_W_ACT_EL_SPEED_CRUISE;
        w_az_const = AM7_SETTINGS_W_ACT_AZ_CONST_CRUISE;
        w_az_speed = AM7_SETTINGS_W_ACT_AZ_SPEED_CRUISE;
        w_theta_const = AM7_SETTINGS_W_ACT_THETA_CONST_CRUISE;
        w_theta_speed = AM7_SETTINGS_W_ACT_THETA_SPEED_CRUISE;
        w_phi_const = AM7_SETTINGS_W_ACT_PHI_CONST_CRUISE;
        w_phi_speed = AM7_SETTINGS_W_ACT_PHI_SPEED_CRUISE;
        w_ail_const = AM7_SETTINGS_W_ACT_AILERONS_CONST_CRUISE;
        w_ail_speed = AM7_SETTINGS_W_ACT_AILERONS_SPEED_CRUISE;
        w_dv_1 = AM7_SETTINGS_W_DV_1_CRUISE;
        w_dv_2 = AM7_SETTINGS_W_DV_2_CRUISE;
        w_dv_3 = AM7_SETTINGS_W_DV_3_CRUISE;
        w_dv_4 = AM7_SETTINGS_W_DV_4_CRUISE;
        w_dv_5 = AM7_SETTINGS_W_DV_5_CRUISE;
        w_dv_6 = AM7_SETTINGS_W_DV_6_CRUISE;
        k_gain_airspeed = AM7_SETTINGS_K_GAIN_AIRSPEED_CRUISE;
        gamma_quadratic_du = AM7_SETTINGS_GAMMA_QUADRATIC_DU_CRUISE_e_minus_8;
    }
}

/*Update the external waypoints: from the readings of the SIXDOF and the ARUCO */
void update_external_WP(void){
    //Retrieve the position of the beacons and update the waypoints: 
    struct EnuCoor_f target_pos_sixdof = {myam7_data_in.sixdof_NED_pos_y, myam7_data_in.sixdof_NED_pos_x, -myam7_data_in.sixdof_NED_pos_z + alt_offset_beacon}; 
    waypoint_set_enu(WP_SIXDOF, &target_pos_sixdof); 
    // Send to the GCS that the waypoint has been moved
    static uint8_t wp_id = WP_SIXDOF;
    RunOnceEvery(AM7_FREQUENCY / 2.0f, { //Update SIXDOF waypoint every 0.5 seconds
        DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                                &waypoints[WP_SIXDOF].enu_i.x,
                                &waypoints[WP_SIXDOF].enu_i.y,
                                &waypoints[WP_SIXDOF].enu_i.z);
    });

    //Do the same for the aruco marker: 
    struct EnuCoor_f target_pos_aruco = {myam7_data_in.aruco_NED_pos_y, myam7_data_in.aruco_NED_pos_x, -myam7_data_in.aruco_NED_pos_z + alt_offset_beacon};
    waypoint_set_enu(WP_ARUCO, &target_pos_aruco);
    static uint8_t wp_id_aruco = WP_ARUCO;
    RunOnceEvery(AM7_FREQUENCY / 2.0f, { //Update ARUCO waypoint every 0.5 seconds
        DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id_aruco,
                                &waypoints[WP_ARUCO].enu_i.x,
                                &waypoints[WP_ARUCO].enu_i.y,
                                &waypoints[WP_ARUCO].enu_i.z);
    });
}

/*Routine to be called repetitively: */
void am7_routine(void){ 
    //Update tfmini lidar though the am7 module:
    #if USE_AM7_TFMINI_LIDAR
    tfmini_lidar_update();
    #endif 
    //Update filters: 
    update_am7_filters();
    //Update external WPs from the SIXDOF and ARUCO readings: 
    update_external_WP();
    //Update the gains and weights based on the approach state: 
    update_gains_weights();
    //Assign the data to the struct and prepare extra_data_out array:
    assign_am7_data();
    //Send the message over serial to the Raspberry pi:
    send_am7_packet_over_serial();
}

/*Function call to report the am7 data in to external modules: */
inline struct am7_data_in * get_am7_data_in(void){
    return &myam7_data_in;
}

/*Init the serial communication, filters and the ABI bind message: */
void am7_init(void) 
{
    //Init variables for communication:
    buffer_in_counter = 0;
    sending_msg_id = 0;

    //Init abi bind msg:
    AbiBindMsgAM7_DATA_OUT(ABI_BROADCAST, &AM7_receive_from_modules, AM7_receive_from_module_fcn);
    AbiBindMsgAGL(ABI_BROADCAST, &get_lidar_value_ev, get_lidar_alt);

    //If requested, register the periodic telemetry messages:
    #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_IN, am7_downlink);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_OUT, am7_uplink);
    #endif

    //Init filters: 
    init_butterworth_2_low_pass(&body_p_dot_second_order_filter, 1.0f /(float) AM7_SETTINGS_BODY_RATES_DOT_SECOND_ORDER_CUTOFF_RAD_S, 1.0f/(float) AM7_FREQUENCY, 0.0);
    init_butterworth_2_low_pass(&body_q_dot_second_order_filter, 1.0f /(float) AM7_SETTINGS_BODY_RATES_DOT_SECOND_ORDER_CUTOFF_RAD_S, 1.0f/(float) AM7_FREQUENCY, 0.0);
    init_butterworth_2_low_pass(&body_r_dot_second_order_filter, 1.0f /(float) AM7_SETTINGS_BODY_RATES_DOT_SECOND_ORDER_CUTOFF_RAD_S, 1.0f/(float) AM7_FREQUENCY, 0.0);
}

/* We need to wait for incoming messages */
void am7_event(void)
{
    if(fabs(get_sys_time_float() - last_ts) > 5){
        received_packets = 0;
        last_ts = get_sys_time_float();
    }
    while(uart_char_available(&(AM7_PORT)) > 0) {
        uint8_t am7_byte_in;
        am7_byte_in = uart_getch(&(AM7_PORT));
        if ((am7_byte_in == START_BYTE) || (buffer_in_counter > 0)) {
            am7_msg_buf_in[buffer_in_counter] = am7_byte_in;
            buffer_in_counter++;
        }
        if (buffer_in_counter > sizeof(struct am7_data_in) ) {
            buffer_in_counter = 0;
            uint8_t checksum_in_local = 0;
            for(uint16_t i = 1; i < sizeof(struct am7_data_in) ; i++){
                checksum_in_local += am7_msg_buf_in[i];
            }
            if(checksum_in_local == am7_msg_buf_in[sizeof(struct am7_data_in)]){
                //Copy received data to the struct:
                memcpy(&myam7_data_in, &am7_msg_buf_in[1], sizeof(struct am7_data_in));
                received_packets++;
            }
            else {
                missed_packets++;
            }
        }
    }
    ca7_message_frequency_RX = (uint16_t) received_packets/(get_sys_time_float() - last_ts);
}

