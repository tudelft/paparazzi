/*
 * Copyright (C) 2015 C. De Wagter
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/Overactuated_vehicle/Overactuated_vehicle.h"
 * @author Alessandro Mancinelli
 * Control laws for Overactuated Vehicle
 */

#ifndef OVERACTUATED_MIXING_H
#define OVERACTUATED_MIXING_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"


/**
 * DEFINE VEHICLE PROPERTIES
 */
#define N_ACT_REAL 13
#define INDI_INPUTS 6
#define INDI_NUM_ACT 15

/**
 * DEFINE SOME USEFUL STRUCT
 */
struct FloatEulersPosition {
    float phi;
    float theta;
    float psi;
    float x;
    float y;
    float z;
};

struct BodyCoord_f {
    float x;
    float y;
    float z;
};

struct ActuatorsStruct {
    float motor;
    float elevation;
    float azimuth;
};

struct PID_over_simple {
    float p;
    float i;
    float d;
};

struct PID_over {
    struct FloatEulersPosition p;
    struct FloatEulersPosition i;
    struct FloatEulersPosition d;
};

struct PD_indi_over {
    struct FloatEulersPosition p;
    struct FloatEulersPosition d;
};

struct ESC_status {
    float ESC_1_rpm; 
    float ESC_1_voltage; 
    float ESC_1_current;
    float ESC_1_consumption;  
    float ESC_2_rpm; 
    float ESC_2_voltage; 
    float ESC_2_current;
    float ESC_2_consumption;  
    float ESC_3_rpm; 
    float ESC_3_voltage; 
    float ESC_3_current;
    float ESC_3_consumption;    
    float ESC_4_rpm; 
    float ESC_4_voltage; 
    float ESC_4_current;
    float ESC_4_consumption;            
};

/* overactuated mixing structure */
struct overactuated_mixing_t {
    int32_t commands[N_ACT_REAL];      ///< The output commands
};

extern struct overactuated_mixing_t overactuated_mixing;
extern struct PID_over pid_gains_over;
extern struct PID_over_simple pid_pos_x_att;
extern struct PID_over_simple pid_pos_y_att;
extern struct PID_over_simple pid_gain_psi_motor;
extern struct PD_indi_over indi_gains_over;
extern struct ActuatorsStruct act_dyn_struct;

// Variables for slider
extern float K_beta;
extern float K_T_airspeed;
extern float K_d_speed;

extern int16_t neutral_servo_1_pwm; 
extern int16_t neutral_servo_2_pwm; 

extern float CL_ailerons;

extern float des_pos_earth_x;
extern float des_pos_earth_y;
extern float stick_gain_yaw;
extern float stick_gain_throttle;
extern bool yaw_with_tilting_PID;

extern bool manual_heading;
extern int manual_heading_value_rad;

//Variables for the FBW controller: 
extern float K_indi_rad_s_dshot, Des_RPM_motor_1, Des_dshot_steps_motor_1;

extern float K_p_rad_s_dshot, K_i_rad_s_dshot, K_d_rad_s_dshot;

extern int min_pwm_servo_9, max_pwm_servo_9, neutral_pwm_servo_9, min_pwm_servo_10, max_pwm_servo_10, neutral_pwm_servo_10; 
extern float desired_angle_servo_9, desired_angle_servo_10;

extern float des_az_angle_test, des_el_angle_test;

/* External used functions */
extern void overactuated_mixing_init(void);
extern void overactuated_mixing_run(void);

#endif
