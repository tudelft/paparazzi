
/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/gcnet/gcnet_main.h"
 * @brief example empty controller
 *
 * Implements an example simple rate controller in a module.
 */

#ifndef GCNET_MAIN_H
#define GCNET_MAIN_H

// paparazzi library with standard definitions
#include "std.h" 

// paparazzi-based libraries
#include "math/pprz_algebra_float.h"
#include "state.h"

// neural network library
#include "modules/gcnet/nn_operations.h"

extern float control_nn[NUM_CONTROLS];

/*
extern float state_nn[NUM_STATES];

//extern float ev_pos[3];
//extern float ev_att[3];

extern float Mx_measured;
extern float My_measured;
extern float Mz_measured;
extern float az_measured;

extern float Mx_modeled;
extern float My_modeled;
extern float Mz_modeled;
extern float az_modeled;
*/


extern void gcnet_init(void);
extern void gcnet_run(void);


// Logging
extern void gnc_net_log_header(FILE *file);
extern void gnc_net_log_data(FILE *file);


//extern void external_vision_update(uint8_t *buf);

#endif
