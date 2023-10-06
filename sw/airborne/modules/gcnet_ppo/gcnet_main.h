
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
 * @file "modules/gcnet_ppo/gcnet_main.h"
 * @brief example empty controller
 *
 * Implements an example simple rate controller in a module.
 */

#ifndef GCNET_MAIN_H
#define GCNET_MAIN_H

#include <stdio.h>

// paparazzi library with standard definitions
#include "std.h" 

// paparazzi-based libraries
#include "math/pprz_algebra_float.h"
#include "state.h"

// ppo neural network code
#include "modules/gcnet_ppo/nn_controller.h"

float control_nn[4];
bool gcnet_fake_att_mode;

void gcnet_activate(void);

void go_to_start(void);
void go_to_next_gate(void);

extern void gcnet_init(void);
extern void gcnet_run(void);

// Logging
extern void gnc_net_log_header(FILE *file);
extern void gnc_net_log_data(FILE *file);


#endif
