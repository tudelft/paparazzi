/*
 * Copyright (C) 2023 MAVLab
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
 * @file modules/ins/ins_ext_pose.h
 * Integrated Navigation System interface.
 */

/*
Extened kalman filter based on https://en.wikipedia.org/wiki/Extended_Kalman_filter
Section 5.3: Non-additive noise formulation and equations
*/

#ifndef INS_EXT_POSE_H
#define INS_EXT_POSE_H

#define EKF_NUM_STATES 15
#define EKF_NUM_INPUTS 6
#define EKF_NUM_OUTPUTS 6

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include <stdio.h>


extern float ekf_X[EKF_NUM_STATES];

extern void ins_ext_pose_init(void);
extern void ins_ext_pose_run(void);

extern void ins_ext_pose_msg_update(uint8_t *buf);

// Logging
extern void ins_ext_pos_log_header(FILE *file);
extern void ins_ext_pos_log_data(FILE *file);

struct InsExtPose {
  /* Inputs */
  struct FloatRates gyros_f;
  struct FloatVect3 accels_f;
  bool   has_new_gyro;
  bool   has_new_acc;

  struct FloatVect3 ev_pos;
  struct FloatVect3 ev_vel;
  struct FloatEulers ev_att;
  struct FloatQuat ev_quat;
  bool   has_new_ext_pose;
  float  ev_time;

  /* Origin */
  struct LtpDef_i  ltp_def;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;
};
extern struct InsExtPose ins_ext_pos;

#endif
