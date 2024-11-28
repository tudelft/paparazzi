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

struct ship_msg ship_state;
float x_speed_control_coeff_array[10], y_speed_control_coeff_array[10], z_speed_coeff_array[10];

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
    // x_speed_control_coeff_array = DL_SHIP_PREDICTION_MSG_speed_x_control_coeffs(buf);
    // y_speed_control_coeff_array = DL_SHIP_PREDICTION_MSG_speed_y_control_coeffs(buf);
    // z_speed_coeff_array = DL_SHIP_PREDICTION_MSG_speed_z_control_coeffs(buf);
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
 * @brief Function to initialize the nav approach ship module
 */
void nav_approach_ship_init(void){
  register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_SHIP_INFO_MSG_GROUND , send_ship_info_msg_ground );
}

/**
 * @brief Call the Function to approach the ship
 */
void nav_approach_ship_run(void){
    //Call the function to approach the ship
    // nav_approach_ship();
}

