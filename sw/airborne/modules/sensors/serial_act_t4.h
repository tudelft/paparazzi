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
 * Uses a teensy 4.0 as fly by wire system. The teensy can control serial bus servos and KISS ESC providing real time telemetry.
 */

#ifndef SERIAL_ACT_T4
#define SERIAL_ACT_T4

#define START_BYTE_SERIAL_ACT_T4 0x9A  //1st start block identifier byte


#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"

struct __attribute__((__packed__)) serial_act_t4_in {  
    //SERVOS telemetry & update rate 
    int16_t servo_1_angle_int; //Degrees * 100 
    int16_t servo_2_angle_int; //Degrees * 100 
    int16_t servo_3_angle_int; //Degrees * 100 
    int16_t servo_4_angle_int; //Degrees * 100 
    int16_t servo_5_angle_int; //Degrees * 100 
    int16_t servo_6_angle_int; //Degrees * 100 
    int16_t servo_1_feedback_update_time_us; //MicroSeconds
    int16_t servo_2_feedback_update_time_us; //MicroSeconds
    int16_t servo_3_feedback_update_time_us; //MicroSeconds
    int16_t servo_4_feedback_update_time_us; //MicroSeconds
    int16_t servo_5_feedback_update_time_us; //MicroSeconds
    int16_t servo_6_feedback_update_time_us; //MicroSeconds
    int16_t servo_1_load_int; 
    int16_t servo_2_load_int; 
    int16_t servo_3_load_int; 
    int16_t servo_4_load_int; 
    int16_t servo_5_load_int; 
    int16_t servo_6_load_int; 
    int16_t servo_1_speed_int; 
    int16_t servo_2_speed_int; 
    int16_t servo_3_speed_int;     
    int16_t servo_4_speed_int; 
    int16_t servo_5_speed_int; 
    int16_t servo_6_speed_int;   
    int16_t servo_1_volt_int; 
    int16_t servo_2_volt_int; 
    int16_t servo_3_volt_int;        
    int16_t servo_4_volt_int; 
    int16_t servo_5_volt_int; 
    int16_t servo_6_volt_int;   
    int16_t servo_1_temp_int; 
    int16_t servo_2_temp_int; 
    int16_t servo_3_temp_int;   
    int16_t servo_4_temp_int; 
    int16_t servo_5_temp_int; 
    int16_t servo_6_temp_int; 
    //Rolling message in 
    float rolling_msg_in;
    uint8_t rolling_msg_in_id;   
    //CHECKSUM
    uint8_t checksum_in;
};


struct __attribute__((__packed__)) serial_act_t4_out {
    //ARM cmd
    int8_t servo_arm_int; //Arm servo boolean
    //Servo cmd
    int16_t servo_1_cmd_int; //Degrees * 100 
    int16_t servo_2_cmd_int; //Degrees * 100 
    int16_t servo_3_cmd_int; //Degrees * 100 
    int16_t servo_4_cmd_int; //Degrees * 100 
    int16_t servo_5_cmd_int; //Degrees * 100 
    int16_t servo_6_cmd_int; //Degrees * 100 
    //Rolling message out
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;
    //CHECKSUM
    uint8_t checksum_out;
};

extern void serial_act_t4_init(void);
extern void serial_act_t4_event(void);

#endif

