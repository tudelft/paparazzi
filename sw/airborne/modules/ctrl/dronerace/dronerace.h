/*
 * Copyright (C) MAVLab
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
 * @file "modules/ctrl/dronerace//dronerace.h"
 * @author MAVLab
 * Autonomous Drone Race
 */


#include <stdio.h>

#ifndef DRONERACE_H
#define DRONERACE_H

#define KDX 0.57 
#define KDY 0.56

// run
extern void dronerace_init(void);

extern void dronerace_enter(void);  // 1 time
extern void dronerace_periodic(void);

// export
extern void dronerace_set_rc(float t, float x, float y, float z);
extern void dronerace_get_cmd(float* alt, float* phi, float* theta, float* psi);


void ahrsblah();

void find_optimal(float *x0, float *v0, 
                  float *xd, float *vd,
                  float *xt, float *vt,
                  float *phi0, float *phi1, 
                  float *switch_time, float psi0);

float pathPredict(float x0[2], float v0[2],
                  float xd[2], float vd[2], 
                  float phi0, float phi1, float t1, 
                  float *xt, float *vt, float psi0);


//void get_state(dronerace_state_struct *var) ;

#endif

