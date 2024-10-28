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
 * @file "modules/overactuated_vehicle/overactuated_vehicle.h"
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
struct FloatEulers {
    float phi;
    float theta;
    float psi;
};
struct PID_over {
    struct FloatEulers p;
    struct FloatEulers i;
    struct FloatEulers d;
};
struct PD_indi_over {
    struct FloatEulersPosition p;
    struct FloatEulersPosition d;
};
struct ship_info_msg { 
    float timestamp; 
    float phi; 
    float theta; 
    float psi; 
    float phi_dot; 
    float theta_dot; 
    float psi_dot; 
    float x; 
    float y; 
    float z; 
    float lat; 
    float lon; 
    float alt;     
    float x_dot; 
    float y_dot; 
    float z_dot; 
    float x_ddot; 
    float y_ddot; 
    float z_ddot; 
};


/* overactuated mixing structure */
struct overactuated_mixing_t {
    int32_t commands[N_ACT_REAL];      ///< The output commands
};

extern struct overactuated_mixing_t overactuated_mixing;
extern struct PID_over pid_gains_over;
extern struct PD_indi_over active_gains;


// Variables for slider
extern float K_beta;

extern float fpa_off_deg; 

extern float extra_lat_gain;


extern float stick_gain_yaw;
extern bool yaw_with_tilting_PID;

//Variable for the lateral acceleration and yaw rate control:
extern float overestimation_coeff;

extern int approach_state;


//attitude setpoint test: 
extern int use_slider_attitude; 
extern float pitch_target_slider; 
extern float roll_target_slider; 

extern int use_u_init_outer_loop;
extern int use_u_init_inner_loop;
extern int single_loop_controller;
extern int use_new_aero_model;
extern int use_received_ang_ref_in_inner_loop;
extern int dv_contains_modeled_accelerations; 

extern float alt_offset_beacon; 
extern int selected_beacon;
extern int sixdof_mode; 

/* External used functions */
extern void overactuated_mixing_init(void);
extern void overactuated_mixing_run(void);

extern void overactuated_mixing_parse_SHIP_INFO_MSG(uint8_t *buf);

#endif