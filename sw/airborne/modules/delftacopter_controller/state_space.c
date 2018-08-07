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
 * @file state_space.c. See state_space.h for documentation
 */

#include "state_space.h"

struct state_space_t state_space_init(uint8_t n_states, uint8_t n_inputs, uint8_t n_outputs, float** A, float** B, float** C, float** ssx, float** ssu, float** ssy, float** xdx, float** xdu, float** ydx) {
  struct state_space_t ss;

  ss.n_states = n_states;
  ss.n_inputs = n_inputs;
  ss.n_outputs = n_outputs;

  ss.A = A;
  ss.B = B;
  ss.C = C;

  ss.x = ssx;
  ss.u = ssu;
  ss.y = ssy;

  ss.x_due_to_x = xdx;
  ss.x_due_to_u = xdu;
  ss.y_due_to_x = ydx;

  return ss;
}

void state_space_propagate(struct state_space_t* ss, float** u) {
  float_mat_copy(ss->u, u, ss->n_inputs, 1);

  float_mat_mul(ss->x_due_to_x, ss->A, ss->x, ss->n_states, ss->n_states, 1);
  float_mat_mul(ss->x_due_to_u, ss->B, ss->u, ss->n_states, ss->n_inputs, 1);
  float_mat_sum(ss->x, ss->x_due_to_x, ss->x_due_to_u, ss->n_states, 1);

  float_mat_mul(ss->y_due_to_x, ss->C, ss->x, ss->n_outputs, ss->n_states, 1);
  float_mat_copy(ss->y, ss->y_due_to_x, ss->n_outputs, 1);
}

