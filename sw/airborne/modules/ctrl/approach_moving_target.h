/*
 * Copyright (C) 2021 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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
 * @file "modules/ctrl/approach_moving_target.h"
 * @author Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * Approach a moving target (e.g. ship)
 */

#ifndef APPROACH_MOVING_TARGET_H
#define APPROACH_MOVING_TARGET_H

#include "std.h"
#include "math/pprz_algebra_float.h"

// Angle of the approach in degrees
extern float approach_moving_target_angle_deg;

struct Amt {
  struct FloatVect3 rel_unit_vec;
  float distance;
  float speed;
  float pos_gain;
  float psi_ref;
  float slope_ref;
  float speed_gain;
  float relvel_gain;
  float approach_speed_gain;
  int32_t enabled_time;
  uint8_t wp_ship_id;
  uint8_t wp_approach_id;
  float last_lag;
  float lag;
  float integral_gain_carrot;
  float pid_error;
  float pid_errorSum;
  float pid_errorLast;
  uint32_t pid_last_timestamp;

};

struct Wave {
  struct FloatVect3 RollPitchYaw;       // [deg]  the attitude of the deck of the ship
  struct FloatVect3 WaveOff2StaticPos;  // [m]    the offset from the average/static position of the ship
  struct FloatVect3 bufRPY[20];
  struct FloatVect3 bufOff[20];
  float estmInterval;                   // [sec]  the estimated interval in beteween the waves
  float estmAmp;                        // [m]    the estimated amplitude of the waves
  int certainty;                        // [%]    how certain the system is of the wave shape/interval
};

extern struct Amt amt;
extern struct Wave WaveInfl;
extern float amt_err_slowdown_gain;


extern void approach_moving_target_init(void);
extern void follow_diagonal_approach(void);
extern void approach_moving_target_enable(uint8_t wp_ship_id, uint8_t wp_approach_id);
extern void reset_moving_target_distance(void);
extern void waveEstimation(void);

#endif // APPROACH_MOVING_TARGET_H
