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
#include "filters/target_pos_kalman.h"

#ifndef REMOTE_SENSING_AM_H
#define REMOTE_SENSING_AM_H

#define FALCON_MODE_NONE 0x00       ///< 
#define FALCON_MODE_SIXDOF 0x01     ///< 
#define FALCON_MODE_RELANGLE 0x02   ///< 
#define FALCON_MODE_RELBEACON 0x03  ///< 

//define the AzimuthElevation structure: 
struct AzimuthElevation {
  float azimuth; // azimuth angle in rad
  float elevation; // elevation angle in rad
};

struct sixdof_t {
  uint32_t tow; // Time of week of the sixdof measurement
  struct FloatVect3 pos; // position in NED frame
  struct FloatVect3 pos_var; // position variance in NED frame
  struct FloatQuat quat; // attitude in NED frame
  struct FloatVect3 quat_var; // attitude variance in NED frame
  struct KalmanSensor kalman_sensor; // Kalman filter for the falcon sensor
};

struct relangle_t {
  uint32_t tow; // Time of week of the relangle measurement
  uint16_t beacon_id; // Falcon sensor beacon id.
  struct AzimuthElevation angles; // Falcon sensor azimuth and elevation angles. 
  float intensity; // Falcon sensor intensity.
  float width; // Falcon sensor width.
  float distance; // Distance derived from angles and intensity.
  struct FloatVect3 pos; // position in NED frame
  struct KalmanSensor kalman_sensor; // Kalman filter for the falcon sensor
};

struct relbeacon_t {
  uint32_t tow; // Time of week of the relbeacon measurement
  uint16_t beacon_id; // Falcon sensor beacon id.
  struct FloatVect3 pos; // position in NED frame
  struct KalmanSensor kalman_sensor; // Kalman filter for the falcon sensor
};

struct target_pos_t {
  uint32_t tow; // Time of week of the target position measurement
  struct FloatVect3 pos; // position in NED frame
  struct FloatVect3 vel; // velocity in NED frame
  struct FloatQuat quat; // attitude in NED frame
  struct KalmanSensor kalman_sensor; // Kalman filter for target position
};

struct falcon_t {
  uint8_t mode; // Falcon sensor mode
  struct FloatQuat sensor_to_body; // Rotation of the body relative to the sensor
  struct FloatVect3 body_to_sensor_offset; // Position of the sensor in the body frame
  struct sixdof_t sixdof; // Sixdof sensor data
  struct relangle_t relangle; // Relangle sensor data
  struct relbeacon_t relbeacon; // Relbeacon sensor data
};

struct aruco_t {
  uint32_t tow; // Time of week of the aruco measurement
  uint16_t id; // Aruco marker id
  struct FloatQuat sensor_to_body; // Rotation of the body relative to the sensor
  struct FloatVect3 body_to_sensor_offset; // Position of the sensor in the body frame
  struct FloatVect3 pos; // position in Ned frame
  struct FloatQuat quat; // attitude in Ned frame
  struct KalmanSensor kalman_sensor; // Kalman filter for the opencv aruco sensor
};

struct nps_target_t {
  struct FloatVect3 pos;  // position in NED frame
  struct FloatVect3 vel;  // velocity in NED frame
  struct FloatEulers rpy; // attitude
  struct FloatRates pqr;  // angular rates
};

extern struct falcon_t falcon; // for settings

extern void remote_sensing_AM_init(void); 
extern void remote_sensing_AM_periodic(void);
extern void remote_sensing_AM_kalman_filter_init(float r);
extern void remote_sensing_parse_target_pos(uint8_t *buf);
extern void remote_sensing_AM_send_falcon_cmd(uint8_t unk);
extern void remote_sensing_parse_falcon_sixdof(uint8_t *buf);
extern void remote_sensing_parse_falcon_relangle(uint8_t *buf);
extern void remote_sensing_parse_falcon_relbeacon(uint8_t *buf);
extern void remote_sensing_parse_opencv_aruco(uint8_t *buf);
void test_request_landing_path(void);

#endif /* REMOTE_SENSING_H */

