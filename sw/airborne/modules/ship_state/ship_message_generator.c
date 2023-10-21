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

float phi_state, phi_dot_state, theta_state, theta_dot_state, psi_state, psi_dot_state; 

static void send_ship_info_message(struct transport_tx *trans, struct link_device *dev)
{
    float phi_telemetry = stateGetNedToBodyEulers_f()->phi * 180/M_PI;
    float theta_telemetry = stateGetNedToBodyEulers_f()->theta * 180/M_PI;
    float psi_telemetry = stateGetNedToBodyEulers_f()->psi * 180/M_PI;
    float phi_dot_telemetry = phi_dot_state * 180/M_PI;
    float theta_dot_telemetry = theta_dot_state * 180/M_PI;
    float psi_dot_telemetry = psi_dot_state * 180/M_PI;
    float x_telemetry = stateGetPositionNed_f()->x; 
    float y_telemetry = stateGetPositionNed_f()->y; 
    float z_telemetry = stateGetPositionNed_f()->z; 
    float x_dot_telemetry = stateGetSpeedNed_f()->x;
    float y_dot_telemetry = stateGetSpeedNed_f()->y;
    float z_dot_telemetry = stateGetSpeedNed_f()->z;
    float x_ddot_telemetry = stateGetAccelNed_f()->x;
    float y_ddot_telemetry = stateGetAccelNed_f()->y;
    float z_ddot_telemetry = stateGetAccelNed_f()->z;       
    pprz_msg_send_SHIP_INFO_MSG(trans, dev, AC_ID, &phi_telemetry, &theta_telemetry, &psi_telemetry,
                                &phi_dot_telemetry, &theta_dot_telemetry, &psi_dot_telemetry,
                                &x_telemetry, &y_telemetry, &z_telemetry,
                                &x_dot_telemetry, &y_dot_telemetry, &z_dot_telemetry,
                                &x_ddot_telemetry, &y_ddot_telemetry, &z_ddot_telemetry);
}


void ship_message_generator_init(void) 
{
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SHIP_INFO_MSG, send_ship_info_message);
}

void ship_message_generator_periodic(void) 
{
    phi_dot_state = (stateGetNedToBodyEulers_f()->phi - phi_state)*500; 
    theta_dot_state = (stateGetNedToBodyEulers_f()->theta - theta_state)*500; 
    psi_dot_state = (stateGetNedToBodyEulers_f()->psi - psi_state)*500; 
    phi_state = stateGetNedToBodyEulers_f()->phi;
    theta_state = stateGetNedToBodyEulers_f()->theta;
    psi_state = stateGetNedToBodyEulers_f()->psi;
    
}
    
