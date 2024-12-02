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
 * @file "modules/ship_landing/core_nav_approach_ship.h"
 * @author Alessandro Mancinelli (a.mancinelli@tudelft.nl)
 * Nav approach ship module
 */

#ifndef CORE_NAV_APPROACH_SHIP_H
#define CORE_NAV_APPROACH_SHIP_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

struct ship_msg { 
    uint32_t tow_ship;
    float timestamp; 
    float phi; 
    float theta; 
    float psi; 
    float phi_dot; 
    float theta_dot; 
    int32_t lat; 
    int32_t lon; 
    int32_t alt;     
    float x_dot; 
    float y_dot; 
    float z_dot; 
};

extern void nav_approach_ship_init(void); 
extern void nav_approach_ship_run(void);

extern void nav_approach_ship_parse_SHIP_INFO_MSG(uint8_t *buf);
extern void nav_approach_ship_parse_SHIP_PREDICTION_MSG(uint8_t *buf);

extern float V_target_control_ship_approach[3];
extern float Desired_theta_rad_ship_approach, Desired_phi_rad_ship_approach;

//Extra variables to be accessed from outside or from sliders: 
extern float sliding_window_seconds;
extern int8_t approach_ship_engaged; 
extern float Vx_max_control, Vx_min_control, Vy_max_control, Vy_min_control, Vz_max_control, Vz_min_control;
extern float Ax_max_control, Ax_min_control, Ay_max_control, Ay_min_control, Az_max_control, Az_min_control;
extern float max_time_valid_prediction; 
extern float Px_gain, Py_gain, Pz_gain; 
extern float flare_low_distance_m_float;
extern float v_speed_docking_m_s_float;
extern float diag_approach_speed_m_s_float; 
extern float pos_tracking_distance_m_float;
extern float Px_APP_point_offset, Py_APP_point_offset, Pz_APP_point_offset;
extern float approach_heading_ship_deg, dist_line_gain_float, max_line_gain_float;

extern float SHIP_pos_NED_float[3];
extern float UAV_speed_NED_float[3];
extern float SHIP_speed_NED_float[3];
extern float Avg_speed_ship_NED_float[3];

#endif // CORE_NAV_APPROACH_SHIP_H