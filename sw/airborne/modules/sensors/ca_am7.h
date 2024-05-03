/*
 * Copyright (C) Paparazzi Team
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
 * @file "modules/sensors/ca_am7.h"
 * @author Alessandro Mancinelli
 * Converts telemtry data from a CA device AM7 type to the autopilot
 */

#ifndef AM7_H
#define AM7_H

#define START_BYTE 0x9B  //1st start block identifier byte


#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"


struct __attribute__((__packed__)) am7_data_in {
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
    float rolling_msg_in;
    uint8_t rolling_msg_in_id;
    uint8_t checksum_in;
};

struct __attribute__((__packed__)) am7_data_out {
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
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;
    uint8_t checksum_out;
};

extern void am7_init(void);
extern void am7_event(void);

#endif

