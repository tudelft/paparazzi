/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/ctrl/target_pos.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Create a target position derived from and RTK gps or TARGET_POS message
 */

#ifndef TARGET_POS_H
#define TARGET_POS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "filters/target_pos_kalman.h"

struct target_pos_t {
  bool valid;               ///< If the data of the target position is valid
  uint32_t recv_time;       ///< Time of when the target position message was received [msec]
  uint32_t tow;             ///< Time of week of the target position measurement
  struct LlaCoor_i lla;     ///< Lat, lon and altitude position of the target
  struct FloatVect3 vel;     ///< Speed of target in target local NED frame [m/s]
  struct FloatQuat quat;    ///< Attitude quaternion of the target body to target local NED frame
  struct FloatRates rates;  ///< Body rates of the target in [rad/s]
  float ground_speed;       ///< Ground speed of the target [m/s]
  float course;             ///< Ground course of the target [deg]
  float heading;            ///< Heading of the target [deg]
  float climb;              ///< Climb speed, z-up [m/s]
};

struct target_offset_t {
  float x;              ///< Target offset in x axis in the target body frame [m]
  float y;              ///< Target offset in y axis in the target body frame [m]
  float z;              ///< Target offset in z axis in the target body frame [m]
};

struct target_t {
  struct target_pos_t pos;                  ///< The target position message
  struct target_offset_t offset;            ///< The target offset relative to ground heading
  uint32_t target_pos_timeout;              ///< Ground target position message timeout [msec]
  uint32_t rtk_timeout;                     ///< RTK message timeout [msec]
  struct LlaCoor_i gps_lla;                 ///< GPS LLA position
};

struct falcon_sensor_t {
  uint8_t valid;
  bool manual;
  uint8_t mode;
  uint16_t beacon_id;
  struct FloatRMat sensor_to_body_rotation;
  struct FloatVect3 sensor_to_cg_translation;
  struct FloatVect3 p_out;
  struct FloatVect3 p_in;
  struct FloatQuat q;
  struct FloatVect3 p_var;
  struct FloatVect3 q_var;
  struct FloatEulers angles;
  float intensity;
  float width;
  float distance;
  struct KalmanSensor kalman;
};

extern float dist;
extern struct falcon_sensor_t falcon;
extern struct target_t target;
extern void target_pos_init(void);
extern void target_pos_periodic(void);
extern void target_parse_target_pos(uint8_t *buf);
extern void target_pos_parse_falcon_sixdof(uint8_t *buf);
extern void target_pos_parse_falcon_relangle(uint8_t *buf);
extern void target_pos_send_falcon_cmd(float unk);
extern bool target_get_pos(struct NedCoor_f *pos, float *heading);
extern bool target_get_vel(struct NedCoor_f *vel);
extern void target_set_wp(uint8_t wp_id);
extern void target_pos_kalman_filter_init(float r);
extern bool target_pos_set_current_offset(float unk);


#endif
