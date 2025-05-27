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

#define FLOATVECT3_TO_ARRAY(v) (float[3]){v.x, v.y, v.z}
#define FLOATQUAT_TO_ARRAY(v) (float[4]){v.qi, v.qx, v.qy, v.qz}
#define FLOATANGLES_TO_ARRAY(v) (float[2]){v.azimuth, v.elevation}
#define POS_SPEED_TO_ARRAY(p, s) (float[6]){p.x, s.x, p.y, s.y, p.z, s.z}

#if USE_NPS
#include "math/pprz_random.h"
static float rand_norm_float(float mu, float sigma) {
  return mu + sigma * rand_gaussian();
}
#endif

// function prototypes
static void update_waypoint(uint8_t wp_id, struct FloatVect3 pos);
static void send_waypoint(uint8_t wp_id);
static void load_kalman_sensor_from_airframe(struct KalmanSensor *ks, const struct KalmanSensor *ks_airframe);
static void load_sensor_rotation_from_airframe(struct FloatQuat *q, struct FloatRMat *rmat_airframe);
static void load_sensor_offset_from_airframe(struct FloatVect3 *offset, const struct FloatVect3 *offset_airframe);
static void sensor_to_NED(struct FloatVect3 *ned, struct FloatVect3 *sensor, struct FloatQuat *sensor_to_body, struct FloatVect3 offset);

// Global variables
struct target_pos_t target = {0};
struct falcon_t falcon = {0};
struct aruco_t aruco = {0};

/* Initialize the kalman filter structs */
struct TargetPosKalman remote_sensing_kalman;
static float P0[6] = REMOTE_SENSING_KALMAN_P0;
static float Q0[6] = REMOTE_SENSING_KALMAN_Q0;

// Load the sensors from the airframe file
static struct KalmanSensor target_ks = TARGET_KALMAN_SENSOR;
static struct KalmanSensor falcon_ks = FALCON_KALMAN_SENSOR;
static struct KalmanSensor aruco_ks = ARUCO_KALMAN_SENSOR;

static struct FloatRMat falcon_rmat = FALCON_BODY_TO_SENSOR;
static struct FloatVect3 falcon_offset = FALCON_OFFSET_UAV_BODY;

static struct FloatRMat aruco_rmat = ARUCO_BODY_TO_SENSOR;
static struct FloatVect3 aruco_offset = ARUCO_OFFSET_UAV_BODY;

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

//Send mode to the falcon system: 
void remote_sensing_AM_send_falcon_cmd(uint8_t mode) 
{
  uint8_t falcon_mode = mode;
  float noise[falcon.kalman_sensor.n_meas];

  // Reset the kalman filter noise for the falcon sensor on mode switch, TODO: update once we have estimates of the real noise
  switch (falcon_mode) {
    case FALCON_MODE_SIXDOF:
      for (int i = 0; i < falcon.kalman_sensor.n_meas; i++) {
        noise[i] = 0.1f;
      }
      break;
    case FALCON_MODE_RELANGLE: 
      for (int i = 0; i < falcon.kalman_sensor.n_meas; i++) {
        noise[i] = 0.1f;
      }
      break;
    case FALCON_MODE_RELBEACON: 
      for (int i = 0; i < falcon.kalman_sensor.n_meas; i++) {
        noise[i] = 0.1f;
      }
      break;
    default: // No mode
      break;

    target_pos_kalman_set_noise(&falcon.kalman_sensor, noise);
  }

  #if !USE_NPS
  pprz_msg_send_IMCU_FALCON_CMD(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, &falcon_mode);
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

  // Now we have the target position in the UAV NED frame (target.pos) and the target velocity in the UAV NED frame (target.vel).


  // Save the relative position and velocity in the target structure
  #if REMOTE_SENSING_KALMAN_USE_GROUND_STATION
    target_pos_kalman_set_measurement(&target.kalman_sensor, POS_SPEED_TO_ARRAY(target.pos, target.vel));
    target_pos_kalman_update(&remote_sensing_kalman, &target.kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_BOX, target.pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_BOX);});
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
  sensor_to_NED(&falcon.sixdof.pos, &p_rot, &falcon.sensor_to_body, falcon.body_to_sensor_offset);
  sensor_to_NED(&falcon.sixdof.pos_var, &p_var_rot, &falcon.sensor_to_body, (struct FloatVect3){0, 0, 0});

  //Fill up the falcon structure
  falcon.sixdof.tow = get_sys_time_tow();

  #if REMOTE_SENSING_KALMAN_USE_FALCON_SIXDOF
  // Update the kalman filter with the new position and position variance
  target_pos_kalman_set_measurement(&falcon.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.sixdof.pos));
  target_pos_kalman_set_noise(&falcon.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.sixdof.pos_var));
  target_pos_kalman_update(&remote_sensing_kalman, &falcon.kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_SIXDOF, falcon.sixdof.pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_SIXDOF);});
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
  // float falcon.relangle_distance = 5.4165 + 75.5979 / falcon.relangle_intensity - 80.3343 / (falcon.relangle_intensity*falcon.relangle_intensity);
  
  // // Obtain the relative position in sensor frame
  struct FloatVect3 p_out_sensor;
  struct FloatQuat rel_angles_sensor;
  
  // Convert the relative angles to quaternions and multiple with the estimated distance along the sensor y axis to get the relative position
  // in the sensor frame
  float_quat_of_eulers(&rel_angles_sensor, &angles);
  float_quat_vmult(&p_out_sensor, &rel_angles_sensor, &(struct FloatVect3){0, falcon.relangle.distance, 0});
  sensor_to_NED(&falcon.relangle.pos, &p_out_sensor, &falcon.sensor_to_body, falcon.body_to_sensor_offset); // Rotate the position to body frame
  
  #if REMOTE_SENSING_KALMAN_USE_FALCON_RELANGLE
  // Update the kalman filter with the new angles and intensity.
  target_pos_kalman_set_measurement(&falcon.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.relangle.pos));
  target_pos_kalman_update(&remote_sensing_kalman, &falcon.kalman_sensor); 
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_RELANGLE, falcon.relangle.pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_RELANGLE);});
}

/**
 * Receive a RELBEACON message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_relbeacon(uint8_t *buf) 
{
  falcon.relbeacon.tow = get_sys_time_tow();
  falcon.relbeacon.beacon_id = pprzlink_get_DL_IMCU_FALCON_RELBEACON_id(buf);
  float *pos = pprzlink_get_DL_IMCU_FALCON_RELBEACON_pos(buf);

  sensor_to_NED(&falcon.relbeacon.pos, &(struct FloatVect3){pos[0], pos[1], pos[2]}, &falcon.sensor_to_body, falcon.body_to_sensor_offset); // Rotate the position to NED frame

  #if REMOTE_SENSING_KALMAN_USE_FALCON_RELBEACON
  // Update the kalman filter with the beacon position.
  target_pos_kalman_set_measurement(&falcon.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.relbeacon.pos));
  target_pos_kalman_update(&remote_sensing_kalman, &falcon.kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_RELBEACON, falcon.relbeacon.pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_RELBEACON);});
}

#if !USE_NPS
void test_request_landing_path(void){
  // Send also another message for debug: 
  float min_time_landing = 0.1f;
  float max_time_landing = 10.0f;
  float landing_time_resolution = 0.1f;
  float V_bound_max_control[3] = {5.0f, 5.0f, 5.0f};
  float V_bound_min_control[3] = {-5.0f, -5.0f, -5.0f};
  float A_bound_max_control[3] = {5.0f, 5.0f, 5.0f};
  float A_bound_min_control[3] = {-5.0f, -5.0f, -5.0f};
  uint16_t num_points = 13;

  float coeffs_ship_prediction[24] = {
    0.1f, 0.2f, 0.3f, 0.4f,
    0.5f, 0.6f, 0.7f, 0.8f,
    0.9f, 1.0f, 1.1f, 1.2f,
    1.3f, 1.4f, 1.5f, 1.6f,
    1.7f, 1.8f, 1.9f, 2.0f,
  };
  float init_NED_path_pos[3] = {1.0f, 2.0f, 3.0f};
  float init_NED_path_speed[3] = {4.0f, 5.0f, 6.0f};
  float init_NED_path_acc[3] = {0.0f, 0.0f, 0.0f};
  float psi_ship_rad = 0.0f;
  float P0_ship_NED[3] = {0, 0, 0}; // Current ship position in the NED frame
  float time_delay_prediction = 0.0f; // Ship prediction delay

  pprz_msg_send_IMCU_LANDING_PATH_SETTINGS(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, 
    &min_time_landing, &max_time_landing, &landing_time_resolution,
    V_bound_max_control, V_bound_min_control, A_bound_max_control, A_bound_min_control,
    &num_points);

  uint32_t current_time_ms = get_sys_time_tow();

  pprz_msg_send_IMCU_REQUEST_PATH_COEFF(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID,
      &current_time_ms, coeffs_ship_prediction, init_NED_path_pos, init_NED_path_speed, init_NED_path_acc,
      &psi_ship_rad, P0_ship_NED, &time_delay_prediction);
}
#endif

/**
 * Receive a RELBEACON message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_opencv_aruco(uint8_t *buf) 
{
  aruco.tow = get_sys_time_tow();
  aruco.id = pprzlink_get_DL_IMCU_OPENCV_ARUCO_id(buf);
  float *pos = pprzlink_get_DL_IMCU_OPENCV_ARUCO_pos(buf);

  sensor_to_NED(&aruco.pos, &(struct FloatVect3){pos[0], pos[1], pos[2]}, &aruco.sensor_to_body, aruco.body_to_sensor_offset); // Rotate the position to NED frame

  #if REMOTE_SENSING_KALMAN_USE_OPENCV_ARUCO
  // Update the kalman filter with the aruco position.
  target_pos_kalman_set_measurement(&aruco.kalman_sensor, FLOATVECT3_TO_ARRAY(aruco.pos));
  target_pos_kalman_update(&remote_sensing_kalman, &aruco.kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_ARUCO, aruco.pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_ARUCO);});
}

void remote_sensing_AM_kalman_filter_init(float r __attribute__((unused))) {
  target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);
}

void remote_sensing_AM_init(void)
{

  // Initialize the RNG for simulations
  #if USE_NPS
  init_random();
  #endif

  //Init function
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_REMOTE_SENSING_AM, send_remote_sensing_am_periodic);
  #endif

  /* Initialize the linear Kalman filter */
  target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);

  /* Configure the Kalman sensors */
  load_kalman_sensor_from_airframe(&target.kalman_sensor, &target_ks);
  load_kalman_sensor_from_airframe(&falcon.kalman_sensor, &falcon_ks);
  load_kalman_sensor_from_airframe(&aruco.kalman_sensor, &aruco_ks);
  
  /* Store sensor to body rotation for falcon and aruco camera */
  load_sensor_rotation_from_airframe(&falcon.sensor_to_body, &falcon_rmat);
  load_sensor_rotation_from_airframe(&aruco.sensor_to_body, &aruco_rmat);

  /* Store sensor to body offset for falcon and aruco camera */
  load_sensor_offset_from_airframe(&falcon.body_to_sensor_offset, &falcon_offset);
  load_sensor_offset_from_airframe(&aruco.body_to_sensor_offset, &aruco_offset);

  falcon.mode = FALCON_MODE_SIXDOF; // Default mode
}

void remote_sensing_AM_periodic(void) {
  
#if !TARTGET_POS_GROUND_STATION
#if USE_NPS
  // Fake some target pos data
  // Falcon every 50 Hz
  struct FloatVect3 falcon_meas = {rand_norm_float(0, falcon.kalman_sensor.noise[0]),
    rand_norm_float(0, falcon.kalman_sensor.noise[1]), 
    rand_norm_float(0, falcon.kalman_sensor.noise[2]) - stateGetPositionNed_f()->z};

  target_pos_kalman_set_measurement(&falcon.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon_meas));

  RunOnceEvery(1, {
    target_pos_kalman_update(&remote_sensing_kalman, &falcon.kalman_sensor);
  });

  struct FloatVect3 aruco_meas = {rand_norm_float(0, aruco.kalman_sensor.noise[0]),
    rand_norm_float(0, aruco.kalman_sensor.noise[1]),
    rand_norm_float(0, aruco.kalman_sensor.noise[2]) - stateGetPositionNed_f()->z};

  target_pos_kalman_set_measurement(&aruco.kalman_sensor, FLOATVECT3_TO_ARRAY(aruco_meas));

  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ / 10, {
    target_pos_kalman_update(&remote_sensing_kalman, &aruco.kalman_sensor);
  });
  
  struct FloatVect3 target_pos = {rand_norm_float(0, target.kalman_sensor.noise[0]),
    rand_norm_float(0, target.kalman_sensor.noise[2]),
    rand_norm_float(0, target.kalman_sensor.noise[4]) - stateGetPositionNed_f()->z};
  
    struct FloatVect3 target_speed = {rand_norm_float(0, target.kalman_sensor.noise[1]),
    rand_norm_float(0, target.kalman_sensor.noise[3]),
    rand_norm_float(0, target.kalman_sensor.noise[5]) - stateGetSpeedNed_f()->z};

  target_pos_kalman_set_measurement(&target.kalman_sensor, POS_SPEED_TO_ARRAY(target_pos, target_speed));
    
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ / 8, {
    target_pos_kalman_update(&remote_sensing_kalman, &target.kalman_sensor);
  });

  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {
    // Update a position in the flight plan for now
    update_waypoint(WP_SIXDOF, falcon_meas);
    send_waypoint(WP_SIXDOF);

    update_waypoint(WP_ARUCO, aruco_meas);
    send_waypoint(WP_ARUCO);

    update_waypoint(WP_BOX, target_pos);
    send_waypoint(WP_BOX);
  });
#endif
#endif
  
  // Here we can run the periodic KF update
  target_pos_kalman_predict(&remote_sensing_kalman);

  // Get Kalman state
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  target_pos_kalman_get_state(&remote_sensing_kalman, &pos, &speed);

  // Check for NaN
  if (isnan(pos.x) || isnan(pos.y) || isnan(pos.z) || isnan(speed.x) || isnan(speed.y) || isnan(speed.z)) {
    // Reset the kalman filter
    target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);
  } 

  update_waypoint(WP_KALMAN, pos);
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

  return;
}

static void update_waypoint(uint8_t wp_id, struct FloatVect3 pos) {
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, pos);
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
  float_quat_of_rmat(q, rmat_airframe);
}

static void load_sensor_offset_from_airframe(struct FloatVect3 *offset, const struct FloatVect3 *offset_airframe) {
  offset->x = offset_airframe->x;
  offset->y = offset_airframe->y;
  offset->z = offset_airframe->z;
}

static void sensor_to_NED(struct FloatVect3 *ned, struct FloatVect3 *sensor, struct FloatQuat *sensor_to_body, struct FloatVect3 offset) {
  
  // From sensor to body frame
  struct FloatVect3 body;
  float_quat_vmult(&body, sensor_to_body, sensor); // Rotate the position to body frame
  VECT3_ADD(body, offset);
  
  // From body to NED frame
  struct FloatQuat body_to_ned;
  float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  float_quat_vmult(ned, &body_to_ned, &body); // Rotate the position to earth frame NED
}