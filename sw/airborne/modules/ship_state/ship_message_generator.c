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
 * @file "modules/ship_state/ship_message_generator.c"
 * @author Alessandro Mancinelli
 */

#include "modules/ship_state/ship_message_generator.h"
#include "pprzlink/pprz_transport.h"
#include <time.h>
#include <sys/time.h>
#include "modules/core/abi.h"
#include "modules/datalink/telemetry.h"
#include "modules/gps/gps.h"

float phi_state, phi_dot_state, theta_state, theta_dot_state, psi_state, psi_dot_state; 

static void send_ship_info_message(struct transport_tx *trans, struct link_device *dev)
{
    uint32_t itow_ship_gps = get_sys_time_tow();
    float packet_timestamp_telemetry = get_sys_time_float();
    float phi_telemetry = stateGetNedToBodyEulers_f()->phi * 180/M_PI;
    float theta_telemetry = stateGetNedToBodyEulers_f()->theta * 180/M_PI;
    float psi_telemetry = stateGetNedToBodyEulers_f()->psi * 180/M_PI;
    float x_dot_telemetry = stateGetSpeedNed_f()->x;
    float y_dot_telemetry = stateGetSpeedNed_f()->y;
    float z_dot_telemetry = stateGetSpeedNed_f()->z;
    int32_t lat_state_telemetry = stateGetPositionLla_i()->lat; //degrees *1e-7
    int32_t long_state_telemetry = stateGetPositionLla_i()->lon; //degrees *1e-7
    int32_t alt_state_telemetry = stateGetPositionLla_i()->alt;  //millimeters    
    //Add the prediction coefficients with zeros: [not used in this message]
    float speed_empty_coeffs_telemetry[10] = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};
    //For the euler rates let's use the body rates: 
    float phi_dot_telemetry = (stateGetBodyRates_f()->p + 
                               stateGetBodyRates_f()->q * sinf(stateGetNedToBodyEulers_f()->phi)*tanf(stateGetNedToBodyEulers_f()->theta) +
                               stateGetBodyRates_f()->r * cosf(stateGetNedToBodyEulers_f()->phi)*tanf(stateGetNedToBodyEulers_f()->theta)) * 180/M_PI;
    float theta_dot_telemetry = (stateGetBodyRates_f()->q * cosf(stateGetNedToBodyEulers_f()->phi) - 
                                 stateGetBodyRates_f()->r * sinf(stateGetNedToBodyEulers_f()->phi)) * 180/M_PI;
    float psi_dot_telemetry = (stateGetBodyRates_f()->q * sinf(stateGetNedToBodyEulers_f()->phi)/cosf(stateGetNedToBodyEulers_f()->theta) +
                               stateGetBodyRates_f()->r * cosf(stateGetNedToBodyEulers_f()->phi)/cosf(stateGetNedToBodyEulers_f()->theta)) * 180/M_PI;
                               
    pprz_msg_send_SHIP_INFO_MSG_GROUND(trans, dev, AC_ID, 
                                &itow_ship_gps, &packet_timestamp_telemetry, 
                                &phi_telemetry, &theta_telemetry, &psi_telemetry,
                                &phi_dot_telemetry, &theta_dot_telemetry, &psi_dot_telemetry,
                                &lat_state_telemetry, &long_state_telemetry, &alt_state_telemetry,
                                &x_dot_telemetry, &y_dot_telemetry, &z_dot_telemetry,
                                speed_empty_coeffs_telemetry, speed_empty_coeffs_telemetry, speed_empty_coeffs_telemetry);
}


void ship_message_generator_init(void) 
{
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SHIP_INFO_MSG_GROUND, send_ship_info_message);
}

    
