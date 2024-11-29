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

//Variables declaration: 
struct ship_msg ship_state;
float x_speed_control_coeff_array[10], y_speed_control_coeff_array[10], z_speed_coeff_array[10];
double average_speed_NED_ship[3] = {0.0, 0.0, 0.0};
int counter_speed_ship = 0;
double approach_ship_mode_old = (double) 0.0f;

//Prepare the outputs: 
double A_err_control_rf[3] = {0.0, 0.0, 0.0};
double expected_landing_time_relative = 0.0;
double optimal_coeffs[18]; 
double A_target_control[3] = {0.0, 0.0, 0.0};
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

//Inputs for the function:
float sliding_window_seconds = 15.0;
double v_max_control_rf[3] = {0.0, 0.0, 0.0};
double v_min_control_rf[3] = {0.0, 0.0, 0.0};
double a_max_control_rf[3] = {0.0, 0.0, 0.0};
double a_min_control_rf[3] = {0.0, 0.0, 0.0};
double max_time_of_landing_seconds = 0.0; 
double pos_gain_landing = (double) 1.0f; 
double speed_gain_landing = (double) 4.0f;
double assume_zero_UAV_acc = (double) 1.0f; 
double flare_low_distance_m = (double) 0.0f;
double v_speed_docking_m_s = (double) 0.0f;
double diag_approach_speed_m_s = (double) 0.0f;
double c_NED_offset_end_point_diag_lan[3] = {0.0, 0.0, -2.0};
double pos_tracking_distance_m = (double) 0.0f;
double approach_heading_ship_rad = (double) 0.0f;
double dist_line_gain = (double) 0.0f;
double max_line_gain = (double) 0.0f;


    

/**
 * Transpose an array from control reference frame to earth reference frame
 */
void from_control_to_earth(float * out_array, float * in_array, float Psi){
    float R_cg_matrix[3][3];
    R_cg_matrix[0][0] = cos(Psi);
    R_cg_matrix[0][1] = sin(Psi);
    R_cg_matrix[0][2] = 0;
    R_cg_matrix[1][0] = -sin(Psi) ;
    R_cg_matrix[1][1] = cos(Psi) ;
    R_cg_matrix[1][2] = 0 ;
    R_cg_matrix[2][0] = 0 ;
    R_cg_matrix[2][1] = 0 ;
    R_cg_matrix[2][2] = 1 ;

    //Do the multiplication between the income array and the transposition matrix:
    for (int j = 0; j < 3; j++) {
        //Initialize value to zero:
        out_array[j] = 0.;
        for (int k = 0; k < 3; k++) {
            out_array[j] += in_array[k] * R_cg_matrix[k][j];
        }
    }
}

// Ivy callback to collect info about the SHIP_INFO_MSG:
void nav_approach_ship_parse_SHIP_INFO_MSG(uint8_t *buf) {
    if(DL_SHIP_INFO_MSG_ac_id(buf) != AC_ID)
    return;
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
                & ship_state.timestamp, & ship_state.phi,& ship_state.theta,& ship_state.psi,
                & ship_state.phi_dot,& ship_state.theta_dot,
                & ship_state.lat,& ship_state.lon,& ship_state.alt,
                & ship_state.x_dot,& ship_state.y_dot,& ship_state.z_dot, 
                & x_speed_control_coeff_array[0], & y_speed_control_coeff_array[0], & z_speed_coeff_array[0]);
}

/**
 * Function for the message NAV_APPROACH_SHIP_MSG
 */
static void send_nav_approach_ship_msg( struct transport_tx *trans , struct link_device * dev ) {

  //Prepare variables from function output: 
  float A_err_control_rf_float_temetery[3] = {(float) A_err_control_rf[0],(float)  A_err_control_rf[1],(float)  A_err_control_rf[2]};
  float A_target_control_float_temetery[3] = {(float) A_target_control[0],(float)  A_target_control[1],(float)  A_target_control[2]};
  float V_target_control_float_temetery[3] = {(float) V_target_control[0],(float)  V_target_control[1],(float)  V_target_control[2]};
  float expected_landing_time_relative_temetetry = (float) expected_landing_time_relative;
  float UAV_to_SHIP_dist_NED_float_temetery[3] = {(float) UAV_to_SHIP_dist_NED[0],(float)  UAV_to_SHIP_dist_NED[1],(float)  UAV_to_SHIP_dist_NED[2]};
  float optimal_coeffs_x[6] = {(float) optimal_coeffs[0],(float) optimal_coeffs[1],(float) optimal_coeffs[2],(float) optimal_coeffs[3],(float) optimal_coeffs[4],(float) optimal_coeffs[5]};
  float optimal_coeffs_y[6] = {(float) optimal_coeffs[6],(float) optimal_coeffs[7],(float) optimal_coeffs[8],(float) optimal_coeffs[9],(float) optimal_coeffs[10],(float) optimal_coeffs[11]};
  float optimal_coeffs_z[6] = {(float) optimal_coeffs[12],(float) optimal_coeffs[13],(float) optimal_coeffs[14],(float) optimal_coeffs[15],(float) optimal_coeffs[16],(float) optimal_coeffs[17]};
  float exitflag_approach_path_temetery = (float) exitflag_approach_path;
  float V_OOB_float_temetery[6] = {(float) V_OOB[0],(float)  V_OOB[1],(float)  V_OOB[2],(float)  V_OOB[3],(float)  V_OOB[4],(float)  V_OOB[5]};
  float A_OOB_float_temetery[6] = {(float) A_OOB[0],(float)  A_OOB[1],(float)  A_OOB[2],(float)  A_OOB[3],(float)  A_OOB[4],(float)  A_OOB[5]};
  float Desired_phi_rad_float_temetery = (float) Desired_phi_rad;
  float Desired_theta_rad_float_temetery = (float) Desired_theta_rad;
  float UAV_to_SHIP_azimuth_angle_rad_float_temetery = (float) UAV_to_SHIP_azimuth_angle_rad;
  float UAV_to_SHIP_elevation_angle_rad_float_temetery = (float) UAV_to_SHIP_elevation_angle_rad;
  float delta_psi_float_temetery = (float) delta_psi;
  float psi_UAV_to_ship_float_temetery = (float) psi_UAV_to_ship;

    // Send telemetry message
    pprz_msg_send_NAV_APPROACH_SHIP_MSG(trans , dev , AC_ID ,
                & A_err_control_rf_float_temetery[0], & A_target_control_float_temetery[0], & V_target_control_float_temetery[0], & expected_landing_time_relative_temetetry,
                & optimal_coeffs_x[0], & optimal_coeffs_y[0], & optimal_coeffs_z[0], & exitflag_approach_path_temetery,
                & V_OOB_float_temetery[0], & A_OOB_float_temetery[0], & UAV_to_SHIP_dist_NED_float_temetery[0],
                & Desired_phi_rad_float_temetery, & Desired_theta_rad_float_temetery, & UAV_to_SHIP_azimuth_angle_rad_float_temetery,
                & UAV_to_SHIP_elevation_angle_rad_float_temetery, & delta_psi_float_temetery, & psi_UAV_to_ship_float_temetery);
}



/**
 * @brief Function to initialize the nav approach ship module
 */
void nav_approach_ship_init(void){
  register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_SHIP_INFO_MSG_GROUND , send_ship_info_msg_ground );
  register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_NAV_APPROACH_SHIP_MSG , send_nav_approach_ship_msg );
}

/**
 * @brief Call the Function to approach the ship
 */
void nav_approach_ship_run(void){
    // Retrieve all variables needed for the function:
    double P0_UAV_NED[3] = {(double) stateGetPositionEnu_f()->x,(double) stateGetPositionEnu_f()->y,(double) stateGetPositionEnu_f()->z};
    double V0_UAV_NED[3] = {(double)stateGetSpeedNed_f()->x,(double) stateGetSpeedNed_f()->y,(double) stateGetSpeedNed_f()->z};
    double V0_SHIP_NED[3] = {(double)ship_state.x_dot,(double) ship_state.y_dot,(double) ship_state.z_dot};

    //calculate the average of the speeds of the ship using a sliding window:
    average_speed_NED_ship[0] = (average_speed_NED_ship[0] * counter_speed_ship + V0_SHIP_NED[0]) / (counter_speed_ship + 1);
    average_speed_NED_ship[1] = (average_speed_NED_ship[1] * counter_speed_ship + V0_SHIP_NED[1]) / (counter_speed_ship + 1);
    average_speed_NED_ship[2] = (average_speed_NED_ship[2] * counter_speed_ship + V0_SHIP_NED[2]) / (counter_speed_ship + 1);
    counter_speed_ship++;
    if(counter_speed_ship > sliding_window_seconds*PERIODIC_FREQUENCY_NAV_APPROACH_SHIP){
      counter_speed_ship = 0;
    }

    double A0_UAV_NED[3] = {(double)stateGetAccelNed_f()->x,(double) stateGetAccelNed_f()->y,(double) stateGetAccelNed_f()->z};
    float A0_UAV_NED_FILT_float[3]; 
    from_control_to_earth(A0_UAV_NED_FILT_float, accel_vect_filt_control_rf, stateGetNedToBodyEulers_f()->psi);
    double psi_rad_UAV = (double) stateGetNedToBodyEulers_f()->psi;
    double A0_UAV_NED_FILT[3] = {(double) A0_UAV_NED_FILT_float[0],(double) A0_UAV_NED_FILT_float[1],(double) A0_UAV_NED_FILT_float[2]};

    double PhiThetaPsi_SHIP_rad[3] = {(double) ship_state.phi * M_PI/180,(double) ship_state.theta * M_PI/180,(double) ship_state.psi * M_PI/180};

    double coeffs_ship_prediction_speed_7[24] = 
      {x_speed_control_coeff_array[2], x_speed_control_coeff_array[3], x_speed_control_coeff_array[4], x_speed_control_coeff_array[5], 
      x_speed_control_coeff_array[6], x_speed_control_coeff_array[7], x_speed_control_coeff_array[8], x_speed_control_coeff_array[9], 
      y_speed_control_coeff_array[2], y_speed_control_coeff_array[3], y_speed_control_coeff_array[4], y_speed_control_coeff_array[5],
      y_speed_control_coeff_array[6], y_speed_control_coeff_array[7], y_speed_control_coeff_array[8], y_speed_control_coeff_array[9],
      z_speed_coeff_array[2], z_speed_coeff_array[3], z_speed_coeff_array[4], z_speed_coeff_array[5],
      z_speed_coeff_array[6], z_speed_coeff_array[7], z_speed_coeff_array[8], z_speed_coeff_array[9]};

    double P0_SHIP_NED[3];
    if(state.ned_initialized_i){
      struct NedCoor_i ship_pos_NED_cm;
      struct LlaCoor_i ship_lla = {ship_state.lat, ship_state.lon, ship_state.alt};
      ned_of_lla_point_i(&ship_pos_NED_cm, &state.ned_origin_i, &ship_lla);
      P0_SHIP_NED[0] = (double) ship_pos_NED_cm.x / 100.;
      P0_SHIP_NED[1] = (double) ship_pos_NED_cm.y / 100.;
      P0_SHIP_NED[2] = (double) ship_pos_NED_cm.z / 100.;
    }
    else{
      P0_SHIP_NED[0] = 0;
      P0_SHIP_NED[1] = 0;
      P0_SHIP_NED[2] = 0;
    }

    double t_delay_ship_prediction = ship_state.timestamp - x_speed_control_coeff_array[1];
    
    double approach_ship_mode_local; 
    nav_approach_ship( P0_UAV_NED, V0_UAV_NED, V0_SHIP_NED, A0_UAV_NED, A0_UAV_NED_FILT, PhiThetaPsi_SHIP_rad,
                       P0_SHIP_NED, psi_rad_UAV,
                       v_max_control_rf, v_min_control_rf, a_max_control_rf, a_min_control_rf,
                       coeffs_ship_prediction_speed_7, t_delay_ship_prediction, max_time_of_landing_seconds,
                       pos_gain_landing, speed_gain_landing, assume_zero_UAV_acc, flare_low_distance_m,
                       v_speed_docking_m_s, average_speed_NED_ship,
                       diag_approach_speed_m_s, approach_ship_mode_old, c_NED_offset_end_point_diag_lan,
                       pos_tracking_distance_m, approach_heading_ship_rad,
                       dist_line_gain, max_line_gain, A_err_control_rf,
                       &expected_landing_time_relative, optimal_coeffs,
                       A_target_control, V_target_control, 
                       &exitflag_approach_path, &approach_ship_mode_local, V_OOB, A_OOB,
                       UAV_to_SHIP_dist_NED, &Desired_phi_rad, &Desired_theta_rad,
                       &UAV_to_SHIP_azimuth_angle_rad, &UAV_to_SHIP_elevation_angle_rad, &delta_psi, &psi_UAV_to_ship);

    //Assign approach_ship_mode_old: 
    approach_ship_mode_old = approach_ship_mode_local;

}

