/*
 * Copyright (C) 2025 Alessandro Mancinelli
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
#include "std.h"
#include "math/pprz_algebra_float.h"

#ifndef REMOTE_SENSING_AM_H
#define REMOTE_SENSING_AM_H

enum falcon_mode_t {
  FALCON_MODE_NONE,       
  FALCON_MODE_SIXDOF,     
  FALCON_MODE_RELANGLE,   
  FALCON_MODE_RELBEACON  
};

//define the AzimuthElevation structure: 
struct AzimuthElevation {
  float azimuth; // azimuth angle in rad
  float elevation; // elevation angle in rad
};

struct sixdof_t {
  uint32_t tow; // Time of week of the sixdof measurement
  struct FloatVect3 pos; // position in Sensor frame
  struct FloatVect3 pos_var; // position variance in Sensor frame
  struct FloatQuat quat; // attitude of the sixdof platform relative to the sensor
  struct FloatVect3 quat_var; // attitude variance of the sixdof platform relative to the sensor
};

struct relangle_t {
  uint32_t tow; // Time of week of the relangle measurement
  uint16_t beacon_id; // Falcon sensor beacon id.
  struct AzimuthElevation angles; // Falcon sensor azimuth and elevation angles. 
  float intensity; // Falcon sensor intensity.
  float width; // Falcon sensor width.
  float distance; // Distance derived from angles and intensity.
  struct FloatVect3 pos; // position in NED frame
};

struct relbeacon_t {
  uint32_t tow; // Time of week of the relbeacon measurement
  uint16_t beacon_id; // Falcon sensor beacon id.
  struct FloatVect3 pos; // position in falcon sensor frame
};

struct target_pos_t {
  uint32_t tow; // Time of week of the target position measurement
  struct FloatVect3 pos; // position in NED frame
  struct FloatVect3 vel; // velocity in NED frame
  struct FloatQuat quat; // attitude in NED frame
};

struct falcon_t {
  uint8_t mode; // Falcon sensor mode
  struct sixdof_t sixdof; // Sixdof sensor data
  struct relangle_t relangle; // Relangle sensor data
  struct relbeacon_t relbeacon; // Relbeacon sensor data
};

struct aruco_t {
  uint32_t tow; // Time of week of the aruco measurement
  uint16_t id; // Aruco marker id
  struct FloatVect3 pos; // position in Camera frame
  struct FloatQuat quat; // Attitude of drone at the time of measurement
};

extern int track_aruco_id;

extern bool aruco_use_current_att;
extern bool aruco_filter_ids;
extern bool use_sixdof;
extern bool use_relangle;
extern bool use_relbeacon;
extern bool use_aruco;
extern bool use_rtk;

extern struct falcon_t falcon; // for settings

extern void remote_sensing_AM_init(void); 
extern void remote_sensing_AM_periodic(void);
extern void remote_sensing_AM_kalman_filter_init(float r);
extern void remote_sensing_parse_target_pos(uint8_t *buf);
extern void remote_sensing_AM_send_falcon_cmd(uint8_t mode);
extern void remote_sensing_parse_falcon_sixdof(uint8_t *buf);
extern void remote_sensing_parse_falcon_relangle(uint8_t *buf);
extern void remote_sensing_parse_falcon_relbeacon(uint8_t *buf);
extern void remote_sensing_parse_opencv_aruco(uint8_t *buf);
extern void remote_sensing_send_aruco_attitude(void);

#endif /* REMOTE_SENSING_H */

