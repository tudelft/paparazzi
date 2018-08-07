/*
 * Copyright (C) 2018 Joost Meulenbeld
 *
 * This file is part of paparazzi.
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
 * @file state_space.h
 * @brief Discrete state-space system simulation
 *
 * A discrete state space maintains a current state x, and output y, and calculates
 * a new state based on old state and input u:
 *  x[new] = A*x[old] + B*u
 *  y = C*x
 * 
 * Matrices A and B are thus discrete-time state-space matrices, can be made with 
 * i.e. the command c2d in matlab
 */

#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include <std.h>
#include <stdlib.h>
#include <stdint.h>

#include "math/pprz_algebra_float.h"

struct state_space_t {
  int n_states;
  int n_inputs;
  int n_outputs;

  // System matrices
  float** A;
  float** B;
  float** C;

  float** x; // Current state
  float** u; // Previous input, i.e. the input used to calculate x and y
  float** y; // Current output

  // temporary matrices used in propagate()
  float** x_due_to_x;
  float** x_due_to_u;
  float** y_due_to_x;
};

/** Initialize a state-space system and return pointer to it
 *  @param A: System matrix [n_states  x n_states]
 *  @param B: Input matrix  [n_states  x n_inputs]
 *  @param C: Output matrix [n_outputs x n_states]
 */
struct state_space_t state_space_init(uint8_t n_states, uint8_t n_inputs, uint8_t n_outputs, float** A, float** B, float** C, float** ssx, float** ssu, float** ssy, float** xdx, float** xdu, float** ydx);

/** Propagate the previous state to new state
 * @param ss: pointer to state-space system
 * @param u: [n_inputs x 1] matrix containing the new input
 */
void state_space_propagate(struct state_space_t* ss, float** u);

#endif // STATE_SPACE_H
