/*
 * Copyright (C) Jelle Westenberger
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
 * @file "modules/ctrl_circle/ctrl_circle.c"
 * @author Jelle Westenberger
 * This controller will guide the drone into a coordinated circle turn.
 */

#include "modules/ctrl_circle/ctrl_circle.h"
#include "subsystems/imu.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/gps.h"




double radius;

double pos_x;
double pos_y; 
double pos_z; 
double z_des = 1.5; 
double z_cmd;

double est_phi;
double est_theta;
double est_yaw; 

double pitch_cmd;
double roll_cmd;
double yaw_cmd;

kp_z = 0.1 ;
kd_z = 0.001;

#define PITCHMAX 10 * D2R
#define PITCHMIN = 0

// void circle_ctrl_init() {}
void circle_ctrl_period() {

    pos_x = GpsState.NeCoor_i.x * 100; // in cm? 
    pos_y = GpsState.NeCoor_i.y * 100; 
    pos_z = GpsState.NeCoor_i.z * 100; 

    
    // z control

    z_cmd = kp_z * (z_des-pos_z);
    


}



// void circle_ctrl_event() {}
// void circle_ctrl_callback() {}


