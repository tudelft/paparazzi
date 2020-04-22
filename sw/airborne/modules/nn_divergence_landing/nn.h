/*
 * Copyright (C) Kirk Scheper
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
 * @file "modules/event_based_flow/nn.h"
 * @author Kirk Scheper
 * This module is generates a command to avoid other vehicles based on their relative gps location
 */

#ifndef EVENT_OPTICAL_FLOW_NN_H
#define EVENT_OPTICAL_FLOW_NN_H

#include "std.h"

// modules
extern void nn_init(void);
extern void nn_cntrl(void);

// Divergence + derivative and thrust for logging
// And recording variable to easily identify descents
// TODO: is extern here dangerous?
extern float divergence, divergence_dot, acc_lp, thrust_lp, thrust;
extern float acceleration_sp;
extern float div_gt, divdot_gt;
extern uint16_t spike_count; // not used!
extern uint8_t record;

// settings
extern float thrust_effectiveness;
extern float nn_thrust_p_gain;
extern float nn_thrust_i_gain;

#endif  // EVENT_OPTICAL_FLOW_NN_H
