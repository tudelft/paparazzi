
















/*
 * Copyright (C) 2023 Tomaso De Ponti <t.m.l.deponti@tudelft.nl>
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

/** @file "firmwares/rotorcraft/oneloop/navigation_oneloop.h"
 * @author Tomaso De Ponti <t.m.l.deponti@tudelft.nl>
 * A collection of Navigation functions to test the oneloop controller
 */

#ifndef NAVIGATION_ONELOOP_H
#define NAVIGATION_ONELOOP_H

#include "firmwares/rotorcraft/oneloop/navigation_oneloop.h"

extern void  init_nav_oneloop(void);
extern void  straight_oval(float s, float r, float l, float psi_i, float v_route, float a_route, float j_route, float p[3], float p0[3],  float v[3], float a[3], float j[3], float psi_vec[4], float* lap);
extern void  nav_speed_controller(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_d_des, float x_d_actual, float x_2d_bound, float k1_rm, float k2_rm);

#endif  // NAVIGATION_ONELOOP_H
