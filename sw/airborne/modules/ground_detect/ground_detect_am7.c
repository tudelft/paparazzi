/*
 * Copyright (C) 2024 Alessandro Mancinelli <alessandro.mancinelli@outlook.com>
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
 * @file "modules/ground_detect/ground_detect_am7.c"
 * @author Alessandro Mancinelli <alessandro.mancinelli@outlook.com>
 * Detection of the ground using the AM7 device
 */
#include <stdio.h>
#include "modules/ground_detect/ground_detect_am7.h"
#include "modules/sensors/ca_am7.h"
#include "generated/flight_plan.h"

//Abi event to get the lidar value
static abi_event get_agl_corrected_value_ev;
float altitude_lidar_agl_meters;

//Detect_ground variables: 
uint8_t ground_detected_am = 0;
float time_of_ground_not_detected; 
float min_lidar_alt_ground_detect = 0.3; 
float time_tolerance_land = 0.5;
float az_tolerance_land = 4; //Vertical acceleration tolerance for ground detection


/**
 * ABI callback that obtains lidar corrected AGL altitude
  */
static void get_agl_corrected_value(uint8_t sender_id __attribute__((unused)), uint32_t timestamp_lidar, float distance_lidar_meter)
{   
    timestamp_lidar = timestamp_lidar;
    //Capy distance to global variable: 
    altitude_lidar_agl_meters = distance_lidar_meter;
}

void detect_ground_on_landing_am7_init(void){
    //Init abi for the lidar module: 
    AbiBindMsgAGL(ABI_BROADCAST, &get_agl_corrected_value_ev, get_agl_corrected_value);
}

/**
 * @brief Detects the ground on landing
 * @return 1 if the ground is detected, 0 otherwise
 */
uint8_t detect_ground_on_landing(void){
  if(altitude_lidar_agl_meters <= min_lidar_alt_ground_detect &&
      get_am7_data_in()->lidar_strength >= 200 && 
      get_am7_data_in()->modeled_az_int*0.01f > az_tolerance_land && 
      approach_state == 1 ){

      if(get_sys_time_float() - time_of_ground_not_detected >= time_tolerance_land){
          ground_detected_am = 1;
      }
      else{
          ground_detected_am = 0; 
      }
      
  }
  else{
      time_of_ground_not_detected = get_sys_time_float();
      ground_detected_am = 0; 
  }

  return ground_detected_am;
}