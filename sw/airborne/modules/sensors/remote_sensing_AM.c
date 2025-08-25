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
#include "modules/datalink/telemetry.h"
#include "modules/datalink/downlink.h"
#include "pprzlink/intermcu_msg.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/nav/nav_moving_base.h"
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

#ifndef REMOTE_SENSING_KALMAN_MAX_REPEATED_FAILURE_CNT
#define REMOTE_SENSING_KALMAN_MAX_REPEATED_FAILURE_CNT 5
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
static void load_kalman_sensor_from_airframe(struct KalmanSensor *ks, const struct KalmanSensor *ks_airframe);
static void load_sensor_rotation_from_airframe(struct FloatQuat *q, struct FloatRMat *rmat_airframe);
static void load_sensor_offset_from_airframe(struct FloatVect3 *offset, const struct FloatVect3 *offset_airframe);
static void sensor_to_NED(struct FloatVect3 *ned, struct FloatVect3 *sensor, struct FloatQuat *sensor_to_body, struct FloatVect3 *offset, struct FloatQuat *body_to_ned);
static void falcon_auto_mode(void);
static void handle_kalman_update_result(enum TargetPosKalmanUpdateResult result);

// Global variables
struct target_pos_t target = {0};
struct falcon_t falcon = {0};
struct aruco_t aruco = {0};

/* Initialize the kalman filter struct */
struct TargetPosKalman remote_sensing_kalman = {0};
static float P0[6] = REMOTE_SENSING_KALMAN_P0;
static float Q0[6] = REMOTE_SENSING_KALMAN_Q0;

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

  // struct FloatRates target_pos_rates = {DL_TARGET_POS_p(buf), DL_TARGET_POS_q(buf), DL_TARGET_POS_r(buf)};
  
  struct NedCoor_i target_pos_cm;
  ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target_pos_lla);
  target.pos.x = target_pos_cm.x / 100.;
  target.pos.y = target_pos_cm.y / 100.;
  target.pos.z = target_pos_cm.z / 100.;

  // Convert absolute position and velocity to relative position and velocity
  VECT3_SUB(target.pos, *stateGetPositionNed_f());
  VECT3_SUB(target.vel, *stateGetSpeedNed_f());

  // Save the relative position and velocity in the target structure
  if (use_rtk) {
    target_pos_kalman_set_measurement(&target.kalman_sensor, POS_SPEED_TO_ARRAY(target.pos, target.vel));
    enum TargetPosKalmanUpdateResult result = target_pos_kalman_update(&remote_sensing_kalman, &target.kalman_sensor);
    handle_kalman_update_result(result);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_BOX, &target.pos);
  RunOnceEvery(4, {send_waypoint(WP_BOX);});
}

/**
 * Receive a SIXDOF message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_sixdof(uint8_t *buf)
{
  float *pos = pprzlink_get_DL_IMCU_FALCON_SIXDOF_pos(buf);
  float *quat = pprzlink_get_DL_IMCU_FALCON_SIXDOF_quat(buf);
  float *pos_var = pprzlink_get_DL_IMCU_FALCON_SIXDOF_pos_var(buf);
  // float *quat_var = pprzlink_get_DL_IMCU_falcon.sixdof.quat_var(buf);
  struct FloatQuat q = {quat[0], quat[1], quat[2], quat[3]}; // Rotation of the platform relative to the sensor
  struct FloatVect3 p = {pos[0], pos[1], pos[2]}; // Position of the drone relative to the platform
  struct FloatVect3 p_var = {pos_var[0], pos_var[1], pos_var[2]}; // Variance of p
  // struct FloatVect3 q_var = {quat_var[0], quat_var[1], quat_var[2]}; // Variance of q
  struct FloatVect3 p_rot, p_var_rot;

  // Calculate the position of the platform relative the the UAV
  float_quat_vmult(&p_rot, &q, &p); // Rotate the position around the platform
  float_quat_vmult(&p_var_rot, &q, &p_var); // Rotate the position variance around the platform

  // Invert the position
  VECT3_SMUL(p_rot, p_rot, -1.f);

  // Rotate the position from sensor frame to NED
  sensor_to_NED(&falcon.sixdof.pos, &p_rot, &falcon.sensor_to_body, &falcon.body_to_sensor_offset, NULL);
  sensor_to_NED(&falcon.sixdof.pos_var, &p_var_rot, &falcon.sensor_to_body, &(struct FloatVect3){0, 0, 0}, NULL);

  //Fill up the falcon structure
  falcon.sixdof.tow = get_sys_time_tow();

  if (use_sixdof) {
    // Update the kalman filter with the new position and position variance
    target_pos_kalman_set_measurement(&falcon.sixdof.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.sixdof.pos));
    target_pos_kalman_set_noise(&falcon.sixdof.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.sixdof.pos_var));
    enum TargetPosKalmanUpdateResult result = target_pos_kalman_update(&remote_sensing_kalman, &falcon.sixdof.kalman_sensor);
    handle_kalman_update_result(result);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_SIXDOF, &falcon.sixdof.pos);
  RunOnceEvery(50, {send_waypoint(WP_SIXDOF);});
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

  struct FloatEulers angles = {falcon.relangle.angles.azimuth, 0.f, falcon.relangle.angles.elevation};
  
  /* Implement logic to go from distance and x/z angles to a relative position */
  // Temp relation for distance and intensity, depends on environment and beacon
  // float falcon.relangle.distance = 5.4165 + 75.5979 / falcon.relangle.intensity - 80.3343 / (falcon.relangle.intensity*falcon.relangle.intensity);
  
  // Use the KF distance to estimate a distance for the relative position
  struct FloatVect3 relangle_pos =  target_pos_kalman_get_pos(&remote_sensing_kalman);
  falcon.relangle.distance = sqrtf(VECT3_DOT_PRODUCT(relangle_pos, relangle_pos));

  // Obtain the relative position in sensor frame
  struct FloatVect3 p_out_sensor;
  struct FloatQuat rel_angles_sensor;
  
  // Convert the relative angles to quaternions and multiple with the estimated distance along the sensor y axis to get the relative position
  // in the sensor frame
  float_quat_of_eulers(&rel_angles_sensor, &angles);
  float_quat_vmult(&p_out_sensor, &rel_angles_sensor, &(struct FloatVect3){0, falcon.relangle.distance, 0});
  sensor_to_NED(&falcon.relangle.pos, &p_out_sensor, &falcon.sensor_to_body, &falcon.body_to_sensor_offset, NULL); // Rotate the position to body frame
  
  if (use_relangle) {
    // Update the kalman filter with the new angles and intensity.
    target_pos_kalman_set_measurement(&falcon.relangle.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.relangle.pos));
    enum TargetPosKalmanUpdateResult result = target_pos_kalman_update(&remote_sensing_kalman, &falcon.relangle.kalman_sensor);
    handle_kalman_update_result(result);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_RELANGLE, &falcon.relangle.pos);
  RunOnceEvery(50, {send_waypoint(WP_RELANGLE);});
}

/**
 * Receive a RELBEACON message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_relbeacon(uint8_t *buf) 
{
  falcon.relbeacon.tow = get_sys_time_tow();
  falcon.relbeacon.beacon_id = pprzlink_get_DL_IMCU_FALCON_RELBEACON_id(buf);
  float *pos = pprzlink_get_DL_IMCU_FALCON_RELBEACON_pos(buf);

  sensor_to_NED(&falcon.relbeacon.pos, &(struct FloatVect3){pos[0], pos[1], pos[2]}, &falcon.sensor_to_body, &falcon.body_to_sensor_offset, NULL); // Rotate the position to NED frame

  if (use_relbeacon) {
    // Update the kalman filter with the beacon position.
    target_pos_kalman_set_measurement(&falcon.relbeacon.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.relbeacon.pos));
    enum TargetPosKalmanUpdateResult result = target_pos_kalman_update(&remote_sensing_kalman, &falcon.relbeacon.kalman_sensor);
    handle_kalman_update_result(result);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_RELBEACON, &falcon.relbeacon.pos);
  RunOnceEvery(50, {send_waypoint(WP_RELBEACON);});
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

  if (aruco_use_current_att) {
    sensor_to_NED(&aruco.pos, &(struct FloatVect3){pos[0], pos[1], pos[2]}, &aruco.sensor_to_body, &aruco.body_to_sensor_offset, NULL); 
  } else {
    sensor_to_NED(&aruco.pos, &(struct FloatVect3){pos[0], pos[1], pos[2]}, &aruco.sensor_to_body, &aruco.body_to_sensor_offset, &(struct FloatQuat){quat[0], quat[1], quat[2], quat[3]}); 
  }
  if (use_aruco) {
    // Update the kalman filter with the aruco position.
    target_pos_kalman_set_measurement(&aruco.kalman_sensor, FLOATVECT3_TO_ARRAY(aruco.pos));
    enum TargetPosKalmanUpdateResult result = target_pos_kalman_update(&remote_sensing_kalman, &aruco.kalman_sensor);
    handle_kalman_update_result(result);
  }

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_ARUCO, &aruco.pos);
  RunOnceEvery(5, {send_waypoint(WP_ARUCO);}); 
}

void remote_sensing_AM_kalman_filter_init(float r __attribute__((unused))) {
  target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, REMOTE_SENSING_AM_PERIODIC_PERIOD);
}

void remote_sensing_AM_init(void)
{

  // Init function
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_REMOTE_SENSING_AM, send_remote_sensing_am_periodic);
  #endif

  /* Initialize the linear Kalman filter */
  target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, REMOTE_SENSING_AM_PERIODIC_PERIOD);

  // Load the sensors from the airframe file
  struct KalmanSensor target_ks = TARGET_KALMAN_SENSOR;
  struct KalmanSensor falcon_ks = FALCON_KALMAN_SENSOR;
  struct KalmanSensor aruco_ks = ARUCO_KALMAN_SENSOR;

  struct FloatRMat falcon_rmat = FALCON_BODY_TO_SENSOR;
  struct FloatVect3 falcon_offset = FALCON_OFFSET_UAV_BODY;
  
  struct FloatRMat aruco_rmat = ARUCO_BODY_TO_SENSOR;
  struct FloatVect3 aruco_offset = ARUCO_OFFSET_UAV_BODY;

  float sixdof_measurement_noise[3] = FALCON_SIXDOF_MEASUREMENT_NOISE;
  float relangle_measurement_noise[3] = FALCON_RELANGLE_MEASUREMENT_NOISE;
  float relbeacon_measurement_noise[3] = FALCON_RELBEACON_MEASUREMENT_NOISE;

  /* Configure the Kalman sensors */
  load_kalman_sensor_from_airframe(&target.kalman_sensor, &target_ks);
  load_kalman_sensor_from_airframe(&aruco.kalman_sensor, &aruco_ks);
  load_kalman_sensor_from_airframe(&falcon.sixdof.kalman_sensor, &falcon_ks);
  load_kalman_sensor_from_airframe(&falcon.relangle.kalman_sensor, &falcon_ks);
  load_kalman_sensor_from_airframe(&falcon.relbeacon.kalman_sensor, &falcon_ks);

  /* Set the right measurement noise for the different falcon modes */
  for (int i = 0; i < 3; i++) {
    falcon.sixdof.kalman_sensor.noise[i] = sixdof_measurement_noise[i];
    falcon.relangle.kalman_sensor.noise[i] = relangle_measurement_noise[i];
    falcon.relbeacon.kalman_sensor.noise[i] = relbeacon_measurement_noise[i];
  }
  
  /* Store sensor to body rotation for falcon and aruco camera */
  load_sensor_rotation_from_airframe(&falcon.sensor_to_body, &falcon_rmat);
  load_sensor_rotation_from_airframe(&aruco.sensor_to_body, &aruco_rmat);

  /* Store sensor to body offset for falcon and aruco camera */
  load_sensor_offset_from_airframe(&falcon.body_to_sensor_offset, &falcon_offset);
  load_sensor_offset_from_airframe(&aruco.body_to_sensor_offset, &aruco_offset);

  falcon.mode = FALCON_MODE_SIXDOF; // Default mode
  falcon.auto_mode = false;
}

void remote_sensing_send_aruco_attitude(void) {
  struct FloatQuat q;
  float_quat_invert(&q, stateGetNedToBodyQuat_f());
  uint32_t time_msec = get_sys_time_msec();
  #if !USE_NPS
  pprz_msg_send_IMCU_ARUCO_ATTITUDE(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, &time_msec, FLOATQUAT_TO_ARRAY(q));
  #endif
}

void remote_sensing_AM_periodic(void) {
  if (falcon.auto_mode == true) {
    falcon_auto_mode();
  }

  // Here we can run the periodic KF update
  target_pos_kalman_predict(&remote_sensing_kalman);

  // Get Kalman state
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  target_pos_kalman_get_state(&remote_sensing_kalman, &pos, &speed);

  // Check for NaN
  if (isnan(pos.x) || isnan(pos.y) || isnan(pos.z) || isnan(speed.x) || isnan(speed.y) || isnan(speed.z)) {
    // Reset the kalman filter
    target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, REMOTE_SENSING_AM_PERIODIC_PERIOD);
  } 

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
  
  nav_moving_base_set_pos(&(struct EnuCoor_f){pos.y, pos.x, -pos.z});
  nav_moving_base_set_speed(&(struct EnuCoor_f){speed.y, speed.x, -speed.z});
  nav_moving_base_set_accel(&(struct EnuCoor_f){0.0f, 0.0f, 0.0f});

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

static void load_kalman_sensor_from_airframe(struct KalmanSensor *ks, const struct KalmanSensor *ks_airframe) {
  for (int i = 0; i < ks_airframe->n_meas; i++) {
    ks->noise[i] = ks_airframe->noise[i];
    ks->Hmat[i] = ks_airframe->Hmat[i];
  }

  ks->n_meas = ks_airframe->n_meas;
};

static void load_sensor_rotation_from_airframe(struct FloatQuat *q, struct FloatRMat *rmat_airframe) {  
  struct FloatRMat sensor_to_body;
  float_rmat_transp(&sensor_to_body, rmat_airframe);
  float_quat_of_rmat(q, &sensor_to_body);
}

static void load_sensor_offset_from_airframe(struct FloatVect3 *offset, const struct FloatVect3 *offset_airframe) {
  offset->x = offset_airframe->x;
  offset->y = offset_airframe->y;
  offset->z = offset_airframe->z;
}

static void sensor_to_NED(struct FloatVect3 *ned, struct FloatVect3 *sensor, struct FloatQuat *sensor_to_body, struct FloatVect3 *offset, struct FloatQuat *body_to_ned) {
  #if !USE_NPS
  // From sensor to body frame
  struct FloatVect3 body;
  float_quat_vmult(&body, sensor_to_body, sensor); // Rotate the position to the body frame
  VECT3_ADD(body, *offset);
  
  // From body to NED frame
  if (body_to_ned == NULL) {
    struct FloatQuat body_to_ned;
    float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
    float_quat_vmult(ned, &body_to_ned, &body); // Rotate the position to the NED frame
  } else {
    float_quat_vmult(ned, body_to_ned, &body); // Rotate the position to the NED frame
  }
  
  #else
  VECT3_COPY(*ned, *sensor); // For NPS we send NED position so no need to convert
  #endif
}

// Choose falcon mode automatically based on the current state of the system
static void falcon_auto_mode(void) {
  static float t_switch = 0;
  static enum falcon_mode_t mode = FALCON_MODE_SIXDOF;
  
  enum falcon_mode_t new_mode = FALCON_MODE_SIXDOF;
  float current_time = get_sys_time_float();
  uint32_t current_tow = get_sys_time_tow();
  
  // Prevent constant switching of modes
  if (current_time - t_switch < 3.0f) {
    return;
  }

  struct FloatVect3 pos = target_pos_kalman_get_pos(&remote_sensing_kalman);
  
  // If distance to target is large, always prefer RELANGLE mode
  if (pos.z > 5.0f) { // 6 meters is the cut-off for SIXDOF mode, give a little margin
    new_mode = FALCON_MODE_RELANGLE;
  }

  // Prefer sixdof mode
  new_mode = FALCON_MODE_SIXDOF;

  // If sixdof can't find platform try relbeacon and vice-versa
  if (mode == FALCON_MODE_SIXDOF && current_tow - falcon.sixdof.tow > 5.0) new_mode = FALCON_MODE_RELBEACON;
  else if (mode == FALCON_MODE_RELBEACON && current_tow - falcon.relbeacon.tow > 5.0) new_mode = FALCON_MODE_SIXDOF;
  else if (mode != FALCON_MODE_RELANGLE && current_tow - falcon.sixdof.tow > 10.0 && current_tow - falcon.relbeacon.tow > 10.0) new_mode = FALCON_MODE_RELANGLE; // Fallback to relangle

  if (new_mode == mode) return;
  t_switch = current_time;
  remote_sensing_AM_send_falcon_cmd(new_mode);
}

static void handle_kalman_update_result(enum TargetPosKalmanUpdateResult result) {
  static uint8_t failure_cnt = 0;
  char error[30];
  int rc;
  switch (result) {
    case TARGET_POS_KALMAN_UPDATE_SUCCESS:
      failure_cnt = 0;
      return;
    case TARGET_POS_KALMAN_UPDATE_NULL_PTR:
      rc = snprintf(error, sizeof(error), "TPK: Null Ptr");
      failure_cnt++;
      break;
    case TARGET_POS_KALMAN_UPDATE_DIM:
      rc = snprintf(error, sizeof(error), "TPK: n_meas > TPK_DIM");
      failure_cnt++;
      break;
    case TARGET_POS_KALMAN_UPDATE_OOB:
      rc = snprintf(error, sizeof(error), "TPK: Hmat OOB");
      failure_cnt++;
      break;
    case TARGET_POS_KALMAN_UPDATE_SVD:
      rc = snprintf(error, sizeof(error), "TPK: SVD error");
      failure_cnt++;
      break;
    case TARGET_POS_KALMAN_UPDATE_RCOND:
      rc = snprintf(error, sizeof(error), "TPK: RCond error");
      failure_cnt++;
      break;
    default:
      rc = snprintf(error, sizeof(error), "TPK: Unknown error");
      failure_cnt++;
      break;
  }

  #if TARGET_POS_KALMAN_DEBUG
  #if !USE_NPS 
    pprz_msg_send_INFO_MSG(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, rc, error);
  #endif
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, error);
  #endif

  // Last resort reset of Kalman filter after repeated update failures
  if (failure_cnt > REMOTE_SENSING_KALMAN_MAX_REPEATED_FAILURE_CNT) {
    // Reset the kalman filter if we have too many failures
    target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, REMOTE_SENSING_AM_PERIODIC_PERIOD);
    failure_cnt = 0;
  }
}