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
    //ESC telemetry & error code
	int16_t motor_1_rpm_int; //RPM motor 1
	int16_t motor_2_rpm_int; //RPM motor 2
	int16_t motor_3_rpm_int; //RPM motor 3
	int16_t motor_4_rpm_int; //RPM motor 4
    int16_t motor_1_error_code_int; //ESC 1 error code 
    int16_t motor_2_error_code_int; //ESC 2 error code 
    int16_t motor_3_error_code_int; //ESC 3 error code 
    int16_t motor_4_error_code_int; //ESC 4 error code 
    int16_t motor_1_current_int; //ESC 1 current mA
    int16_t motor_2_current_int; //ESC 2 current mA
    int16_t motor_3_current_int; //ESC 3 current mA
    int16_t motor_4_current_int; //ESC 4 current mA   
    int16_t motor_1_voltage_int; //ESC 1 voltage mV
    int16_t motor_2_voltage_int; //ESC 2 voltage mV
    int16_t motor_3_voltage_int; //ESC 3 voltage mV
    int16_t motor_4_voltage_int; //ESC 4 voltage mV       
    //SERVOS telemetry & update rate 
	int16_t servo_1_angle_int; //Degrees * 100 
	int16_t servo_2_angle_int; //Degrees * 100 
	int16_t servo_3_angle_int; //Degrees * 100 
    int16_t servo_4_angle_int; //Degrees * 100 
    int16_t servo_5_angle_int; //Degrees * 100 
    int16_t servo_6_angle_int; //Degrees * 100 
    int16_t servo_7_angle_int; //Degrees * 100 
    int16_t servo_8_angle_int; //Degrees * 100 
    int16_t servo_9_angle_int; //Degrees * 100 
    int16_t servo_10_angle_int; //Degrees * 100 
	int16_t servo_1_update_time_us; //MicroSeconds
	int16_t servo_2_update_time_us; //MicroSeconds
	int16_t servo_3_update_time_us; //MicroSeconds
    int16_t servo_4_update_time_us; //MicroSeconds
    int16_t servo_5_update_time_us; //MicroSeconds
    int16_t servo_6_update_time_us; //MicroSeconds 
    int16_t servo_7_update_time_us; //MicroSeconds 
    int16_t servo_8_update_time_us; //MicroSeconds 
    int16_t servo_9_update_time_us; //MicroSeconds 
    int16_t servo_10_update_time_us; //MicroSeconds    
    //Rolling message in 
    float rolling_msg_in;
    uint8_t rolling_msg_in_id;   
    //CHECKSUM
    uint8_t checksum_in;
};

struct __attribute__((__packed__)) serial_act_t4_out {
    //ARM cmd
    int8_t motor_arm_int; //Arm motor boolean
    int8_t servo_arm_int; //Arm servo boolean
    //ESC cmd
    int16_t motor_1_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_2_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_3_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_4_dshot_cmd_int; //Motor cmd 0 - 1999
    //Servo cmd
    int16_t servo_1_cmd_int; //Degrees * 100 
    int16_t servo_2_cmd_int; //Degrees * 100 
    int16_t servo_3_cmd_int; //Degrees * 100 
    int16_t servo_4_cmd_int; //Degrees * 100 
    int16_t servo_5_cmd_int; //Degrees * 100 
    int16_t servo_6_cmd_int; //Degrees * 100 
    int16_t servo_7_cmd_int; //Degrees * 100 
    int16_t servo_8_cmd_int; //Degrees * 100 
    int16_t servo_9_cmd_int; //Degrees * 100 
    int16_t servo_10_cmd_int; //Degrees * 100   
    //Rolling message out
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;
    //CHECKSUM
    uint8_t checksum_out;
};

struct ActCmd_t {
    float cmd_timestamp; 
    uint8_t motor_arm; 
    uint8_t servo_arm; 
    uint8_t motor_control_mode; //1 --> dshot command, 2 --> rotation command in rad/s using INDI controller;
    float motor_1_cmd; //CMD is either in dshot or rotation in rad/s depending on the mode 
    float motor_2_cmd; //CMD is either in dshot or rotation in rad/s depending on the mode 
    float motor_3_cmd; //CMD is either in dshot or rotation in rad/s depending on the mode 
    float motor_4_cmd; //CMD is either in dshot or rotation in rad/s depending on the mode 
    float servo_el_1_angle_deg;
    float servo_el_2_angle_deg;
    float servo_el_3_angle_deg;
    float servo_el_4_angle_deg;
    float servo_az_1_angle_deg;
    float servo_az_2_angle_deg;
    float servo_az_3_angle_deg;
    float servo_az_4_angle_deg;
    float flaperon_right_angle_deg;
    float flaperon_left_angle_deg;
};

struct ActStates_t { 
    float timestamp; 
    float motor_1_rad_s;
    float motor_2_rad_s;
    float motor_3_rad_s;
    float motor_4_rad_s;
    float el_1_angle_deg;
    float el_2_angle_deg;
    float el_3_angle_deg;
    float el_4_angle_deg;
    float az_1_angle_deg;
    float az_2_angle_deg;
    float az_3_angle_deg;
    float az_4_angle_deg;
    float flaperon_right_angle_deg;
    float flaperon_left_angle_deg;
    float el_1_angle_deg_corrected; 
    float el_2_angle_deg_corrected;
    float el_3_angle_deg_corrected;
    float el_4_angle_deg_corrected;
    float az_1_angle_deg_corrected;
    float az_2_angle_deg_corrected;
    float az_3_angle_deg_corrected;
    float az_4_angle_deg_corrected;
    float flap_right_angle_deg_corrected;
    float flap_left_angle_deg_corrected;
}

extern void serial_act_t4_init(void);
extern void serial_act_t4_event(void);
extern void serial_act_t4_control(void);

//Sliders variables
extern float K_indi_rad_s_dshot; 
extern int test_rpm_control; 
extern float motor_1_rad_s_slider, motor_2_rad_s_slider, motor_3_rad_s_slider, motor_4_rad_s_slider;
extern int test_dshot_cmd;
extern float motor_1_dshot_slider, motor_2_dshot_slider, motor_3_dshot_slider, motor_4_dshot_slider;
extern int test_servo_angles;
extern float el_1_angle_deg_slider, el_2_angle_deg_slider, el_3_angle_deg_slider, el_4_angle_deg_slider;
extern float az_1_angle_deg_slider, az_2_angle_deg_slider, az_3_angle_deg_slider, az_4_angle_deg_slider;
extern float flaperon_right_angle_deg_slider, flaperon_left_angle_deg_slider;
extern float max_pwm_servo_9, neutral_pwm_servo_9, min_pwm_servo_9;
extern float max_pwm_servo_10, neutral_pwm_servo_10, min_pwm_servo_10;

#endif

