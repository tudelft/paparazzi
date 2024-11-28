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
 * @file "modules/overactuated_vehicle/overactuated_mixing.h"
 * @author Alessandro Mancinelli
 * Control laws for Overactuated Vehicle
 */

#ifndef OVERACTUATED_MIXING_H
#define OVERACTUATED_MIXING_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"


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
// struct FloatEulers {
//     float phi;
//     float theta;
//     float psi;
// };
struct PID_over {
    struct FloatEulers p;
    struct FloatEulers i;
    struct FloatEulers d;
};
struct PD_indi_over {
    struct FloatEulersPosition p;
    struct FloatEulersPosition d;
};

// extern struct overactuated_mixing_t overactuated_mixing;
extern struct PID_over pid_gains_over;
extern struct PD_indi_over active_gains;

extern struct ActCmd_t act_cmd_to_t4;
extern struct am7_data_t data_to_am7_module;

// Variables for slider
extern float K_beta;
extern float fpa_off_deg; 
extern float extra_lat_gain;
extern float overestimation_coeff;

//Variables for limits:
extern float max_fwd_speed, max_airspeed_am7, min_fwd_speed, max_lat_speed, max_vert_speed;
extern float max_fwd_acc, min_fwd_acc, max_lat_acc, max_vert_acc;

// Variable for the gain and weight change:
extern int approach_state;

//Variables to be picked from other modules: 
extern float accel_vect_filt_control_rf[3]; 

/* External used functions */
extern void overactuated_mixing_init(void);
extern void overactuated_mixing_run(void);

#endif