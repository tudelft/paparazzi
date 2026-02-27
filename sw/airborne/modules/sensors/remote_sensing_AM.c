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
#include "modules/sensors/remote_sensing_AM.h"
#include "state.h"
#include <math.h>
#include "modules/datalink/extra_pprz_dl.h"
#include "remote_sensing_ekf.h"
#include "modules/datalink/telemetry.h"
#include "modules/datalink/downlink.h"
#include "pprzlink/intermcu_msg.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/core/abi.h"
#include <stdio.h>

int track_aruco_id = 16;

// Check if the sensors are enabled in the airframe file
#ifndef REMOTE_SENSING_KALMAN_USE_GROUND_STATION 
#warning "Ground Station GPS is not fused in Remote Sensing Kalman"
#define REMOTE_SENSING_KALMAN_USE_GROUND_STATION FALSE
#endif
#ifndef REMOTE_SENSING_KALMAN_USE_FALCON_SIXDOF
#warning "Falcon SIXDOF mode is not fused in Remote Sensing Kalman"
#define REMOTE_SENSING_KALMAN_USE_FALCON_SIXDOF FALSE
#endif
#ifndef REMOTE_SENSING_KALMAN_USE_FALCON_RELANGLE
#warning "Falcon Relative Angle mode is not fused in Remote Sensing Kalman"
#define REMOTE_SENSING_KALMAN_USE_FALCON_RELANGLE FALSE
#endif
#ifndef REMOTE_SENSING_KALMAN_USE_FALCON_RELBEACON
#warning "Falcon Relative Beacon mode is not fused in Remote Sensing Kalman"
#define REMOTE_SENSING_KALMAN_USE_FALCON_RELBEACON FALSE
#endif
#ifndef REMOTE_SENSING_KALMAN_USE_OPENCV_ARUCO
#warning "Aruco is not fused in Remote Sensing Kalman"
#define REMOTE_SENSING_KALMAN_USE_OPENCV_ARUCO FALSE
#endif

PRINT_CONFIG_VAR(REMOTE_SENSING_LOG_ON_ARRIVAL);

bool use_sixdof = REMOTE_SENSING_KALMAN_USE_FALCON_SIXDOF;
bool use_relangle = REMOTE_SENSING_KALMAN_USE_FALCON_RELANGLE;
bool use_relbeacon = REMOTE_SENSING_KALMAN_USE_FALCON_RELBEACON;
bool use_aruco = REMOTE_SENSING_KALMAN_USE_OPENCV_ARUCO;
bool use_rtk = REMOTE_SENSING_KALMAN_USE_GROUND_STATION;

#define FLOATVECT3_TO_ARRAY(v) (float[3]){v.x, v.y, v.z}
#define FLOATQUAT_TO_ARRAY(v) (float[4]){v.qi, v.qx, v.qy, v.qz}
#define FLOATANGLES_TO_ARRAY(v) (float[2]){v.azimuth, v.elevation}
#define POS_SPEED_TO_ARRAY(p, s) (float[6]){p.x, s.x, p.y, s.y, p.z, s.z}

// function prototypes
static void update_waypoint(uint8_t wp_id, struct FloatVect3 *pos);
static void send_waypoint(uint8_t wp_id);
static void ekf_init_defaults(void);

// Global variables
struct target_pos_t target = {0};
struct falcon_t falcon = {0};
struct aruco_t aruco = {0};

/* Initialize the new EKF parameters */
static float P0[9] = REMOTE_SENSING_KALMAN_P0;
static float Q0[9] = REMOTE_SENSING_KALMAN_Q0;

bool aruco_use_current_att = false;
bool aruco_filter_ids = true;

#if PERIODIC_TELEMETRY
static void send_remote_sensing_am_periodic(struct transport_tx *trans, struct link_device *dev) {  
  pprz_msg_send_REMOTE_SENSING_AM(trans, dev, AC_ID,
            &target.tow,
            FLOATVECT3_TO_ARRAY(target.vel),
            FLOATVECT3_TO_ARRAY(target.pos),
            FLOATQUAT_TO_ARRAY(target.quat),
            &falcon.sixdof.tow,
            FLOATVECT3_TO_ARRAY(falcon.sixdof.pos),
            FLOATVECT3_TO_ARRAY(falcon.sixdof.pos_var),
            FLOATQUAT_TO_ARRAY(falcon.sixdof.quat),
            &falcon.relangle.tow,
            &falcon.relangle.beacon_id,
            FLOATANGLES_TO_ARRAY(falcon.relangle.angles),
            &falcon.relangle.intensity,
            &falcon.relangle.width,
            &falcon.relbeacon.tow,
            &falcon.relbeacon.beacon_id,
            FLOATVECT3_TO_ARRAY(falcon.relbeacon.pos),
            &aruco.tow,
            &aruco.id,
            FLOATVECT3_TO_ARRAY(aruco.pos),
            FLOATQUAT_TO_ARRAY(aruco.quat));
}

#endif


#if !USE_NPS
//Function to upload telemetry: 
void sdlog_remote_sensing_am(void){ 
  // Send the current state of the remote_sensing module
  pprz_msg_send_REMOTE_SENSING_AM(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
            &target.tow,
            FLOATVECT3_TO_ARRAY(target.vel),
            FLOATVECT3_TO_ARRAY(target.pos),
            FLOATQUAT_TO_ARRAY(target.quat),
            &falcon.sixdof.tow,
            FLOATVECT3_TO_ARRAY(falcon.sixdof.pos),
            FLOATVECT3_TO_ARRAY(falcon.sixdof.pos_var),
            FLOATQUAT_TO_ARRAY(falcon.sixdof.quat),
            &falcon.relangle.tow,
            &falcon.relangle.beacon_id,
            FLOATANGLES_TO_ARRAY(falcon.relangle.angles),
            &falcon.relangle.intensity,
            &falcon.relangle.width,
            &falcon.relbeacon.tow,
            &falcon.relbeacon.beacon_id,
            FLOATVECT3_TO_ARRAY(falcon.relbeacon.pos),
            &aruco.tow,
            &aruco.id,
            FLOATVECT3_TO_ARRAY(aruco.pos),
            FLOATQUAT_TO_ARRAY(aruco.quat));
}
#endif

// Send mode to the falcon system: 
void remote_sensing_AM_send_falcon_cmd(uint8_t mode) 
{
  falcon.mode = mode;
  #if !USE_NPS
  pprz_msg_send_IMCU_FALCON_CMD(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, &falcon.mode);
  #else
  pprz_msg_send_FALCON_CMD(&(DefaultChannel).trans_tx, &(DefaultDevice).device, AC_ID, &falcon.mode);
  #endif
}

/**
 * Receive a TARGET_POS message from the ground and update the kalman filter if required
 */
void remote_sensing_parse_target_pos(uint8_t *buf)
{
  if(DL_TARGET_POS_ac_id(buf) != AC_ID)
    return;

  // Save the received values on local structures
  // uint32_t target_pos_recv_tow = get_sys_time_tow();
  target.tow = DL_TARGET_POS_tow(buf);
  struct LlaCoor_i target_pos_lla = {DL_TARGET_POS_lat(buf), DL_TARGET_POS_lon(buf), DL_TARGET_POS_alt(buf)};
  target.vel.x = DL_TARGET_POS_vnorth(buf);
  target.vel.y = DL_TARGET_POS_veast(buf);
  target.vel.z = DL_TARGET_POS_vdown(buf);
  target.quat.qi = DL_TARGET_POS_body_qi(buf);
  target.quat.qx = DL_TARGET_POS_body_qx(buf);
  target.quat.qy = DL_TARGET_POS_body_qy(buf);
  target.quat.qz = DL_TARGET_POS_body_qz(buf);
  
  struct NedCoor_i target_pos_cm;
  ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target_pos_lla);
  target.pos.x = target_pos_cm.x / 100.;
  target.pos.y = target_pos_cm.y / 100.;
  target.pos.z = target_pos_cm.z / 100.;

  // Convert absolute position and velocity to relative position and velocity
  VECT3_SUB(target.pos, *stateGetPositionNed_f());
  VECT3_SUB(target.vel, *stateGetSpeedNed_f());

  // Update EKF with GPS (RTK) measurement in NED-relative
  if (use_rtk) {
    ekf_update_gps(target.pos, target.vel);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif
}

/**
 * Receive a SIXDOF message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_sixdof(uint8_t *buf)
{
  float *pos = pprzlink_get_DL_IMCU_FALCON_SIXDOF_pos(buf);
  float *quat = pprzlink_get_DL_IMCU_FALCON_SIXDOF_quat(buf);
  float *pos_var = pprzlink_get_DL_IMCU_FALCON_SIXDOF_pos_var(buf);
  float *quat_var = pprzlink_get_DL_IMCU_FALCON_SIXDOF_quat_var(buf);
  struct FloatQuat q = {quat[0], quat[1], quat[2], quat[3]}; // Rotation of the platform relative to the sensor
  struct FloatVect3 p = {pos[0], pos[1], pos[2]}; // Position of the drone relative to the platform
  struct FloatVect3 p_var = {pos_var[0], pos_var[1], pos_var[2]}; // Variance of p
  struct FloatVect3 q_var = {quat_var[0], quat_var[1], quat_var[2]}; // Variance of q

  // Fill up the falcon structure
  falcon.sixdof.tow = get_sys_time_tow();
  falcon.sixdof.pos = p;
  falcon.sixdof.quat = q;
  falcon.sixdof.pos_var = p_var;
  falcon.sixdof.quat_var = q_var;

  if (use_sixdof) {
    // Pass raw sensor-frame measurement to EKF
    ekf_update_sixdof_pos_var(p_var);
    ekf_update_sixdof(p, q);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif
}

/**
 * Receive a RELANGLE message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_relangle(uint8_t *buf) 
{
  falcon.relangle.tow = get_sys_time_tow();
  falcon.relangle.beacon_id = pprzlink_get_DL_IMCU_FALCON_RELANGLE_id(buf);
  falcon.relangle.intensity = pprzlink_get_DL_IMCU_FALCON_RELANGLE_intensity(buf);
  falcon.relangle.width = pprzlink_get_DL_IMCU_FALCON_RELANGLE_width(buf);
  float *rel_angles = pprzlink_get_DL_IMCU_FALCON_RELANGLE_angles(buf);
  falcon.relangle.angles.azimuth = rel_angles[0];
  falcon.relangle.angles.elevation = rel_angles[1];

  // Not Implemented

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif
}

/**
 * Receive a RELBEACON message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_relbeacon(uint8_t *buf) 
{
  falcon.relbeacon.tow = get_sys_time_tow();
  falcon.relbeacon.beacon_id = pprzlink_get_DL_IMCU_FALCON_RELBEACON_id(buf);
  float *pos = pprzlink_get_DL_IMCU_FALCON_RELBEACON_pos(buf);
  struct FloatVect3 pos_cam = {pos[0], pos[1], pos[2]}; // In the falcon sensor frame
  falcon.relbeacon.pos = pos_cam; 

  if (use_relbeacon) {
    ekf_update_relbeacon(falcon.relbeacon.pos);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif
}

/**
 * Receive an OPENCV aruco message from the camera and update the kalman filter if required
 */
void remote_sensing_parse_opencv_aruco(uint8_t *buf) 
{
  aruco.tow = get_sys_time_msec() - pprzlink_get_DL_IMCU_OPENCV_ARUCO_timestamp(buf);
  aruco.id = pprzlink_get_DL_IMCU_OPENCV_ARUCO_id(buf);
  float *pos = pprzlink_get_DL_IMCU_OPENCV_ARUCO_pos(buf);
  float *quat = pprzlink_get_DL_IMCU_OPENCV_ARUCO_body_to_ned(buf);
  
  if (aruco.id != track_aruco_id && aruco_filter_ids) return;

  aruco.pos = (struct FloatVect3){pos[0], pos[1], pos[2]}; // In the camera frame
  aruco.quat = (struct FloatQuat){quat[0], quat[1], quat[2], quat[3]}; // Rotation of the platform relative to the sensor

  if (use_aruco) {
    // Pass raw sensor-frame measurement to EKF
    ekf_update_aruco(aruco.pos);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif
}

void remote_sensing_AM_kalman_filter_init(float r) {
  (void)r;
  ekf_init_defaults();
}

void remote_sensing_AM_init(void) {

  // Init function
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_REMOTE_SENSING_AM, send_remote_sensing_am_periodic);
  #endif

  /* Initialize the EKF */
  ekf_init_defaults();

  /* Map airframe noise into EKF measurement R for each sensor */
  // GPS: expect pos(3) + speed(3)
  float R_gps_airframe[6] = REMOTE_SENSING_RTK_MEASUREMENT_NOISE;
  float R_gps[9] = {0};
  for (int i = 0; i < 6; i++) {
    R_gps[i] = R_gps_airframe[i];
  }
  ekf_set_R_gps(R_gps);

  // Aruco: first 3 entries used in update
  float R_aruco_airframe[3] = REMOTE_SENSING_ARUCO_MEASUREMENT_NOISE;
  float R_aruco[9] = {0};
  for (int i = 0; i < 3; i++) {
    R_aruco[i] = R_aruco_airframe[i];
  }
  ekf_set_R_aruco(R_aruco);

  // Falcon SIXDOF: first 3 entries for position, quaternion noise optional
  float R_sixdof_airframe[3] = REMOTE_SENSING_SIXDOF_MEASUREMENT_NOISE;
  float R_sixdof[9] = {0};
  for (int i = 0; i < 3; i++) {
    R_sixdof[i] = R_sixdof_airframe[i];
  }
  ekf_set_R_sixdof(R_sixdof);

  falcon.mode = FALCON_MODE_SIXDOF; // Default mode
}

void remote_sensing_send_aruco_attitude(void) {
  struct FloatQuat q;
  float_quat_invert(&q, stateGetNedToBodyQuat_f());
  uint32_t time_msec = get_sys_time_msec();
  #if !USE_NPS
  pprz_msg_send_IMCU_ARUCO_ATTITUDE(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, &time_msec, FLOATQUAT_TO_ARRAY(q));
  #else
  (void)time_msec;
  #endif
}

void remote_sensing_AM_periodic(void) {
  // Predict EKF with body acceleration input
  struct NedCoor_f *accel_ned = stateGetAccelNed_f(); // Might want to filter this
  float U[3] = {accel_ned->x, accel_ned->y, accel_ned->z};
  ekf_predict(U, REMOTE_SENSING_AM_PERIODIC_PERIOD);

  // Get Kalman state
  float *X = ekf_get_X();
  struct FloatVect3 pos = {X[0], X[1], X[2]};
  struct FloatVect3 speed = {X[3], X[4], X[5]};

  // Check for NaN
  if (isnan(pos.x) || isnan(pos.y) || isnan(pos.z) || isnan(speed.x) || isnan(speed.y) || isnan(speed.z)) {
    // Reset the EKF
    ekf_init_defaults();
  }

#if REMOTE_SENSING_EKF_CLIP_P
  ekf_clip_covariance();
#endif

  update_waypoint(WP_KALMAN, &pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_KALMAN);});

  #if !USE_NPS

  pprz_msg_send_TARGET_POS_KALMAN(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                                  &pos.x, &pos.y, &pos.z,
                                  &speed.x, &speed.y, &speed.z);
  #endif
  RunOnceEvery(100, {
  DOWNLINK_SEND_TARGET_POS_KALMAN(DefaultChannel, DefaultDevice,
                                  &pos.x, &pos.y, &pos.z,
                                  &speed.x, &speed.y, &speed.z);
  });

  // Send absolute positions and speed in ENU frame to navigation
  VECT3_ADD(pos, *stateGetPositionNed_f());
  VECT3_ADD(speed, *stateGetSpeedNed_f());
  AbiSendMsgMOVING_BASE(REMOTE_SENSING_ID, &(struct EnuCoor_f) {pos.y, pos.x, -pos.z}, &(struct EnuCoor_f) {speed.y, speed.x, -speed.z}, NULL);

  return;
}

static void update_waypoint(uint8_t wp_id, struct FloatVect3 *pos) {
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, *pos);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);
}

static void send_waypoint(uint8_t wp_id) {
  DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
      &waypoints[wp_id].enu_i.x,
      &waypoints[wp_id].enu_i.y,
      &waypoints[wp_id].enu_i.z);
}

static void ekf_init_defaults(void) {
  // Process noise U: use first 3 entries of legacy Q0
  ekf_set_Q(Q0);

  // Initial state X: zeros
  float X0[9] = {0};
  ekf_set_X(X0);

  ekf_set_P_diag(P0);
}
