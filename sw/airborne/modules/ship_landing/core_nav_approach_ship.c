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
 * @file "modules/ship_landing/core_nav_approach_ship.c"
 * @author Alessandro Mancinelli (a.mancinelli@tudelft.nl)
 * Nav approach ship module
 */

#include "core_nav_approach_ship.h"
#include "generated/flight_plan.h"
#include "modules/datalink/telemetry.h"
#include "modules/nav/waypoints.h"
#include "generated/airframe.h"
#include "state.h"
#include "paparazzi.h"
#include <math.h>
#include "modules/overactuated_vehicle/overactuated_mixing.h"

//Variables declaration (mainly for messages):
float P0_UAV_NED_float[3], V0_UAV_NED_float[3], P0_SHIP_NED_float[3], V0_SHIP_NED_float[3], average_speed_NED_ship_float[3]; 
float PhiThetaPsi_rad_SHIP_float[3];
float expected_landing_time_relative_float, optimal_coeffs_x_float[6], optimal_coeffs_y_float[6], optimal_coeffs_z_float[6];
float exitflag_approach_path_float;
int8_t approach_ship_mode_int_8, V_OOB_int8[6], A_OOB_int8[6];
float UAV_to_SHIP_dist_NED_float[3];
float UAV_to_SHIP_azimuth_angle_rad_float, UAV_to_SHIP_elevation_angle_rad_float;
float delta_psi_float, psi_UAV_to_ship_float;

//External outputs for nav: 
float V_target_control_ship_approach[3] = {0.0, 0.0, 0.0};
float Desired_theta_rad_ship_approach = 0.0f , Desired_phi_rad_ship_approach = 0.0f;

struct ship_msg ship_state;
float x_speed_control_coeff_array[10], y_speed_control_coeff_array[10], z_speed_coeff_array[10];
double average_speed_NED_ship[3] = {0.0, 0.0, 0.0};
int counter_speed_ship = 0;
double approach_ship_mode_old = (double) 0.0f;

//Prepare the outputs: 
double expected_landing_time_relative = 0.0f;
double optimal_coeffs[18]; 
double V_target_control[3] = {0.0, 0.0, 0.0};
double exitflag_approach_path = 0.0;
double V_OOB[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double A_OOB[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double UAV_to_SHIP_dist_NED[3] = {0.0, 0.0, 0.0};
double Desired_phi_rad = 0.0;
double Desired_theta_rad = 0.0;
double UAV_to_SHIP_azimuth_angle_rad = 0.0;
double UAV_to_SHIP_elevation_angle_rad = 0.0;
double delta_psi = 0.0;
double psi_UAV_to_ship = 0.0;


//Inputs for the function (modify through sliders):
float sliding_window_seconds = 10.0;
int8_t approach_ship_engaged = 0; 
float Vx_max_control = 15.0, Vx_min_control = -5.0;
float Vy_max_control = 5.0, Vy_min_control = -5.0;
float Vz_max_control = 3.0, Vz_min_control = -3.0;
float Ax_max_control = 10.0, Ax_min_control = -4.0;
float Ay_max_control = 10.0, Ay_min_control = -10.0;
float Az_max_control = 5.0, Az_min_control = -5.0;
float max_time_valid_prediction = 7.0;
float Px_gain = 1.0, Py_gain = 1.0, Pz_gain = 1.0;
float flare_low_distance_m_float = 0.5;
float v_speed_docking_m_s_float = 0.1;
float pos_tracking_distance_m_float = 6.0;
float diag_approach_speed_m_s_float = 3.0;
float Px_APP_point_offset = 0.0, Py_APP_point_offset = 0.0, Pz_APP_point_offset = -2.0;
float approach_heading_ship_deg = 30.0;
float dist_line_gain_float = 6.0;
float max_line_gain_float = 0.5;


// Ivy callback to collect info about the SHIP_INFO_MSG:
void nav_approach_ship_parse_SHIP_INFO_MSG(uint8_t *buf) {
    if(DL_SHIP_INFO_MSG_ac_id(buf) != AC_ID)
    return;
    ship_state.tow_ship = DL_SHIP_INFO_MSG_tow_ship(buf);
    ship_state.timestamp = DL_SHIP_INFO_MSG_packet_timestamp(buf);
    ship_state.phi = DL_SHIP_INFO_MSG_phi(buf);  
    ship_state.theta = DL_SHIP_INFO_MSG_theta(buf);  
    ship_state.psi = DL_SHIP_INFO_MSG_psi(buf);  
    ship_state.phi_dot = DL_SHIP_INFO_MSG_phi_dot(buf);  
    ship_state.theta_dot = DL_SHIP_INFO_MSG_theta_dot(buf); 
    ship_state.lat = DL_SHIP_INFO_MSG_lat(buf);  
    ship_state.lon = DL_SHIP_INFO_MSG_lon(buf); 
    ship_state.alt = DL_SHIP_INFO_MSG_alt(buf);      
    ship_state.x_dot = DL_SHIP_INFO_MSG_x_dot(buf);  
    ship_state.y_dot = DL_SHIP_INFO_MSG_y_dot(buf); 
    ship_state.z_dot = DL_SHIP_INFO_MSG_z_dot(buf);  
}

// Ivy callback to collect info about the SHIP_PREDICTION_MSG:
void nav_approach_ship_parse_SHIP_PREDICTION_MSG(uint8_t *buf) {
    if(DL_SHIP_INFO_MSG_ac_id(buf) != AC_ID)
    return;
    memcpy(x_speed_control_coeff_array, DL_SHIP_PREDICTION_MSG_speed_x_control_coeffs(buf), sizeof(x_speed_control_coeff_array));
    memcpy(y_speed_control_coeff_array, DL_SHIP_PREDICTION_MSG_speed_y_control_coeffs(buf), sizeof(y_speed_control_coeff_array));
    memcpy(z_speed_coeff_array, DL_SHIP_PREDICTION_MSG_speed_z_control_coeffs(buf), sizeof(z_speed_coeff_array));
}

/**
 * Function for the message SHIP_INFO_MSG_GROUND
 */
static void send_ship_info_msg_ground( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message
    pprz_msg_send_SHIP_INFO_MSG_GROUND(trans , dev , AC_ID ,
                & ship_state.tow_ship,
                & ship_state.timestamp, & ship_state.phi,& ship_state.theta,& ship_state.psi,
                & ship_state.phi_dot,& ship_state.theta_dot,
                & ship_state.lat,& ship_state.lon,& ship_state.alt,
                & ship_state.x_dot,& ship_state.y_dot,& ship_state.z_dot, 
                & x_speed_control_coeff_array[0], & y_speed_control_coeff_array[0], & z_speed_coeff_array[0]);
}

/**
 * Function for the message NAV_APPROACH_SHIP_OUTPUTS
 */
static void send_nav_approach_ship_outputs( struct transport_tx *trans , struct link_device * dev ) {

    // Send telemetry message
    pprz_msg_send_NAV_APPROACH_SHIP_OUTPUTS(trans , dev , AC_ID ,
                & expected_landing_time_relative_float,
                & optimal_coeffs_x_float[0], &optimal_coeffs_y_float[0], &optimal_coeffs_z_float[0],
                & V_target_control_ship_approach[0], & exitflag_approach_path_float,
                & approach_ship_mode_int_8, & V_OOB_int8[0], & A_OOB_int8[0],
                & UAV_to_SHIP_dist_NED_float[0], & Desired_phi_rad_ship_approach, & Desired_theta_rad_ship_approach,
                & UAV_to_SHIP_azimuth_angle_rad_float, & UAV_to_SHIP_elevation_angle_rad_float,
                & delta_psi_float, & psi_UAV_to_ship_float);
}

/**
 * Function for the message NAV_APPROACH_SHIP_INPUTS
 */
static void send_nav_approach_ship_inputs( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message
    pprz_msg_send_NAV_APPROACH_SHIP_INPUTS(trans , dev , AC_ID ,
                & approach_ship_mode_int_8, & P0_UAV_NED_float[0], 
                & V0_UAV_NED_float[0], & P0_SHIP_NED_float[0], 
                & V0_SHIP_NED_float[0], & average_speed_NED_ship_float[0], & PhiThetaPsi_rad_SHIP_float[0]);
}

/**
 * @brief Function to initialize the nav approach ship module
 */
void nav_approach_ship_init(void){
  register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_SHIP_INFO_MSG_GROUND , send_ship_info_msg_ground );
  register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_NAV_APPROACH_SHIP_OUTPUTS , send_nav_approach_ship_outputs );
  register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_NAV_APPROACH_SHIP_INPUTS , send_nav_approach_ship_inputs );
}

/**
 * @brief Call the Function to approach the ship
 */
void nav_approach_ship_run(void){

    // Retrieve all variables needed for the function:
    double P0_UAV_NED[3] = {(double) stateGetPositionNed_f()->x,(double) stateGetPositionNed_f()->y,(double) stateGetPositionNed_f()->z};
    double V0_UAV_NED[3] = {(double) stateGetSpeedNed_f()->x,(double) stateGetSpeedNed_f()->y,(double) stateGetSpeedNed_f()->z};
    double V0_SHIP_NED[3] = {(double)ship_state.x_dot,(double) ship_state.y_dot,(double) ship_state.z_dot};

    //calculate the average of the speeds of the ship using a sliding window:
    average_speed_NED_ship[0] = (average_speed_NED_ship[0] * counter_speed_ship + V0_SHIP_NED[0]) / (counter_speed_ship + 1);
    average_speed_NED_ship[1] = (average_speed_NED_ship[1] * counter_speed_ship + V0_SHIP_NED[1]) / (counter_speed_ship + 1);
    average_speed_NED_ship[2] = (double) 0.0f;
    counter_speed_ship++;
    if(counter_speed_ship > sliding_window_seconds*PERIODIC_FREQUENCY_NAV_APPROACH_SHIP){
      counter_speed_ship = 0;
    }

    double psi_rad_UAV = (double) stateGetNedToBodyEulers_f()->psi;
    double PhiThetaPsi_SHIP_rad[3] = {(double) (ship_state.phi * M_PI/180),(double) (ship_state.theta * M_PI/180),(double) (ship_state.psi * M_PI/180)};

    double coeffs_ship_prediction_speed_7[24] = 
      {x_speed_control_coeff_array[2], x_speed_control_coeff_array[3], x_speed_control_coeff_array[4], x_speed_control_coeff_array[5], 
      x_speed_control_coeff_array[6], x_speed_control_coeff_array[7], x_speed_control_coeff_array[8], x_speed_control_coeff_array[9], 
      y_speed_control_coeff_array[2], y_speed_control_coeff_array[3], y_speed_control_coeff_array[4], y_speed_control_coeff_array[5],
      y_speed_control_coeff_array[6], y_speed_control_coeff_array[7], y_speed_control_coeff_array[8], y_speed_control_coeff_array[9],
      z_speed_coeff_array[2], z_speed_coeff_array[3], z_speed_coeff_array[4], z_speed_coeff_array[5],
      z_speed_coeff_array[6], z_speed_coeff_array[7], z_speed_coeff_array[8], z_speed_coeff_array[9]};

    double P0_SHIP_NED[3] = {0.0, 0.0, 0.0};
    if(state.ned_initialized_i){
      struct NedCoor_i ship_pos_NED_cm;
      struct LlaCoor_i ship_lla = {ship_state.lat, ship_state.lon, ship_state.alt};
      ned_of_lla_point_i(&ship_pos_NED_cm, &state.ned_origin_i, &ship_lla);
      P0_SHIP_NED[0] = (double) ship_pos_NED_cm.x / 100.;
      P0_SHIP_NED[1] = (double) ship_pos_NED_cm.y / 100.;
      P0_SHIP_NED[2] = (double) ship_pos_NED_cm.z / 100.;
    }

    double t_delay_ship_prediction = ship_state.timestamp - x_speed_control_coeff_array[1];
    
    double approach_ship_mode_local; 


    //Prepare inputs from sliders: 
    double v_max_control_rf[3] = {(double) Vx_max_control, (double) Vy_max_control, (double) Vz_max_control};
    double v_min_control_rf[3] = {(double) Vx_min_control, (double) Vy_min_control, (double) Vz_min_control};
    double a_max_control_rf[3] = {(double) Ax_max_control, (double) Ay_max_control, (double) Az_max_control};
    double a_min_control_rf[3] = {(double) Ax_min_control, (double) Ay_min_control, (double) Az_min_control};
    double max_time_of_landing_seconds = ((double) max_time_valid_prediction)  - t_delay_ship_prediction; 
    double pos_gain_landing_array[3] = {(double) Px_gain, (double) Py_gain, (double) Pz_gain};
    double flare_low_distance_m = (double) flare_low_distance_m_float;
    double v_speed_docking_m_s = (double) v_speed_docking_m_s_float;
    double diag_approach_speed_m_s = (double) diag_approach_speed_m_s_float;
    double c_NED_offset_end_point_diag_lan[3] = {(double) Px_APP_point_offset, (double) Py_APP_point_offset, (double) Pz_APP_point_offset};
    double pos_tracking_distance_m = (double) pos_tracking_distance_m_float;
    double approach_heading_ship_rad = (double) (approach_heading_ship_deg*M_PI/180.0);
    double dist_line_gain = (double) dist_line_gain_float;
    double max_line_gain = (double) max_line_gain_float;

    // nav_approach_ship( P0_UAV_NED, V0_UAV_NED, 
    //                    V0_SHIP_NED, PhiThetaPsi_SHIP_rad,
    //                    P0_SHIP_NED, psi_rad_UAV,
    //                    v_max_control_rf, v_min_control_rf, 
    //                    a_max_control_rf, a_min_control_rf,
    //                    coeffs_ship_prediction_speed_7, 
    //                    t_delay_ship_prediction, max_time_of_landing_seconds,
    //                    pos_gain_landing_array, flare_low_distance_m,
    //                    v_speed_docking_m_s, average_speed_NED_ship,
    //                    diag_approach_speed_m_s, approach_ship_mode_old, 
    //                    c_NED_offset_end_point_diag_lan,
    //                    pos_tracking_distance_m, approach_heading_ship_rad,
    //                    dist_line_gain, max_line_gain,
    //                    &expected_landing_time_relative, optimal_coeffs,
    //                    V_target_control, &exitflag_approach_path, 
    //                    &approach_ship_mode_local, V_OOB, A_OOB, 
    //                    UAV_to_SHIP_dist_NED, &Desired_phi_rad, 
    //                    &Desired_theta_rad, &UAV_to_SHIP_azimuth_angle_rad, 
    //                    &UAV_to_SHIP_elevation_angle_rad, &delta_psi, 
    //                    &psi_UAV_to_ship);

    //Assign approach_ship_mode_old: 
    approach_ship_mode_old = approach_ship_mode_local;

    //Prepare the msg outputs:    
    P0_UAV_NED_float[0] = (float) P0_UAV_NED[0]; 
    P0_UAV_NED_float[1] = (float) P0_UAV_NED[1];
    P0_UAV_NED_float[2] = (float) P0_UAV_NED[2];
    V0_UAV_NED_float[0] = (float) V0_UAV_NED[0];
    V0_UAV_NED_float[1] = (float) V0_UAV_NED[1];
    V0_UAV_NED_float[2] = (float) V0_UAV_NED[2];
    P0_SHIP_NED_float[0] = (float) P0_SHIP_NED[0];
    P0_SHIP_NED_float[1] = (float) P0_SHIP_NED[1];
    P0_SHIP_NED_float[2] = (float) P0_SHIP_NED[2];
    V0_SHIP_NED_float[0] = (float) V0_SHIP_NED[0];
    V0_SHIP_NED_float[1] = (float) V0_SHIP_NED[1];
    V0_SHIP_NED_float[2] = (float) V0_SHIP_NED[2];
    average_speed_NED_ship_float[0] = (float) average_speed_NED_ship[0];
    average_speed_NED_ship_float[1] = (float) average_speed_NED_ship[1];
    average_speed_NED_ship_float[2] = (float) average_speed_NED_ship[2];
    PhiThetaPsi_rad_SHIP_float[0] = (float) PhiThetaPsi_SHIP_rad[0];
    PhiThetaPsi_rad_SHIP_float[1] = (float) PhiThetaPsi_SHIP_rad[1];
    PhiThetaPsi_rad_SHIP_float[2] = (float) PhiThetaPsi_SHIP_rad[2];
    expected_landing_time_relative_float = (float) expected_landing_time_relative;
    optimal_coeffs_x_float[0] = (float) optimal_coeffs[0]; optimal_coeffs_x_float[1] = (float) optimal_coeffs[1]; 
    optimal_coeffs_x_float[2] = (float) optimal_coeffs[2]; optimal_coeffs_x_float[3] = (float) optimal_coeffs[3]; 
    optimal_coeffs_x_float[4] = (float) optimal_coeffs[4]; optimal_coeffs_x_float[5] = (float) optimal_coeffs[5]; 
    optimal_coeffs_y_float[0] = (float) optimal_coeffs[6]; optimal_coeffs_y_float[1] = (float) optimal_coeffs[7];
    optimal_coeffs_y_float[2] = (float) optimal_coeffs[8]; optimal_coeffs_y_float[3] = (float) optimal_coeffs[9];
    optimal_coeffs_y_float[4] = (float) optimal_coeffs[10]; optimal_coeffs_y_float[5] = (float) optimal_coeffs[11];
    optimal_coeffs_z_float[0] = (float) optimal_coeffs[12]; optimal_coeffs_z_float[1] = (float) optimal_coeffs[13];
    optimal_coeffs_z_float[2] = (float) optimal_coeffs[14]; optimal_coeffs_z_float[3] = (float) optimal_coeffs[15];
    V_target_control_ship_approach[0] = (float) V_target_control[0]; V_target_control_ship_approach[1] = (float) V_target_control[1];
    V_target_control_ship_approach[2] = (float) V_target_control[2];
    V_OOB_int8[0] = (int8_t) round(V_OOB[0]); 
    V_OOB_int8[1] = (int8_t) round(V_OOB[1]); 
    V_OOB_int8[2] = (int8_t) round(V_OOB[2]);
    V_OOB_int8[3] = (int8_t) round(V_OOB[3]);
    V_OOB_int8[4] = (int8_t) round(V_OOB[4]);
    V_OOB_int8[5] = (int8_t) round(V_OOB[5]);
    A_OOB_int8[0] = (int8_t) round(A_OOB[0]);
    A_OOB_int8[1] = (int8_t) round(A_OOB[1]);
    A_OOB_int8[2] = (int8_t) round(A_OOB[2]);
    A_OOB_int8[3] = (int8_t) round(A_OOB[3]);
    A_OOB_int8[4] = (int8_t) round(A_OOB[4]);
    A_OOB_int8[5] = (int8_t) round(A_OOB[5]);
    UAV_to_SHIP_dist_NED_float[0] = (float) UAV_to_SHIP_dist_NED[0]; UAV_to_SHIP_dist_NED_float[1] = (float) UAV_to_SHIP_dist_NED[1];
    UAV_to_SHIP_dist_NED_float[2] = (float) UAV_to_SHIP_dist_NED[2];
    Desired_theta_rad_ship_approach = (float) Desired_phi_rad;
    Desired_phi_rad_ship_approach = (float) Desired_theta_rad;
    UAV_to_SHIP_azimuth_angle_rad_float = (float) UAV_to_SHIP_azimuth_angle_rad;
    UAV_to_SHIP_elevation_angle_rad_float = (float) UAV_to_SHIP_elevation_angle_rad;
    delta_psi_float = (float) delta_psi;
    psi_UAV_to_ship_float = (float) psi_UAV_to_ship;
    approach_ship_mode_int_8 = (int8_t) round(approach_ship_mode_local);
    exitflag_approach_path_float = (float) exitflag_approach_path;

}

