/*
 * Copyright (C) 2021 Guido de Croon <g.c.h.e.decroon@tudelft.nl>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/ins/ins_flow.h
 *
 * "Inertial" navigation system.
 */

#ifndef INS_FLOW_H
#define INS_FLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#define CONSTANT_ALT_FILTER 0
#define OF_DRAG 1
#define OF_TWO_DIM 0
#define OF_THRUST_BIAS 0

#if CONSTANT_ALT_FILTER == 1

  #if OF_TWO_DIM == 0
    #define N_STATES_OF_KF 3

    #define OF_V_IND 0
    #define OF_ANGLE_IND 1
    #define OF_Z_IND 2
    #define N_MEAS_OF_KF 2
    #define OF_THETA_IND -1
    #define OF_VX_IND -1
  #else
    #define N_STATES_OF_KF 5
    #define OF_V_IND 0
    #define OF_ANGLE_IND 1
    #define OF_Z_IND 2
    #define OF_THETA_IND 3
    #define OF_VX_IND 4
    #define N_MEAS_OF_KF 3
  #endif

  #define OF_ANGLE_DOT_IND -1
  #define OF_Z_DOT_IND -1
#else
  #if OF_THRUST_BIAS == 0
    #define N_STATES_OF_KF 5
    #define OF_THRUST_BIAS_IND -1
  #else
    #define N_STATES_OF_KF 6
    #define OF_THRUST_BIAS_IND 5
  #endif

  // TODO: make these parameters in the estimation scheme:
  #define OF_TB_Q 0.02
  #define OF_TB_P 0.5

  #define OF_V_IND 0
  #define OF_ANGLE_IND 1
  #define OF_ANGLE_DOT_IND 2
  #define OF_Z_IND 3
  #define OF_Z_DOT_IND 4

  #define OF_THETA_IND -1
  #define OF_VX_IND -1



  #define N_MEAS_OF_KF 3
#endif

#define OF_LAT_FLOW_IND 0
#define OF_DIV_FLOW_IND 1
#define OF_RATE_IND 2
#define OF_LAT_FLOW_X_IND 2


// use filter to different extents:
#define USE_ANGLE 1
#define USE_VELOCITY 2
#define USE_HEIGHT 3


#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_int_cmpl_quat.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/ins.h"

extern void ins_flow_init(void);
extern void ins_flow_update(void);

extern float OF_X[N_STATES_OF_KF];
extern bool reset_filter;
extern bool run_filter;
extern int use_filter;
extern float thrust_factor;

#ifdef __cplusplus
}
#endif

#endif /* INS_FLOW_H */
