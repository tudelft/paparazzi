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
#include "modules/nav/nav_rotorcraft_hybrid.h"

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

// function prototypes
static void update_waypoint(uint8_t wp_id, struct FloatVect3 *pos);
static void send_waypoint(uint8_t wp_id);
static void load_kalman_sensor_from_airframe(struct KalmanSensor *ks, const struct KalmanSensor *ks_airframe);
static void load_sensor_rotation_from_airframe(struct FloatQuat *q, struct FloatRMat *rmat_airframe);
static void load_sensor_offset_from_airframe(struct FloatVect3 *offset, const struct FloatVect3 *offset_airframe);
static void sensor_to_NED(struct FloatVect3 *ned, struct FloatVect3 *sensor, struct FloatQuat *sensor_to_body, struct FloatVect3 *offset);
static void falcon_auto_mode(void);

// Global variables
struct target_pos_t target = {0};
struct falcon_t falcon = {0};
struct aruco_t aruco = {0};

/* Initialize the kalman filter struct */
struct TargetPosKalman remote_sensing_kalman;
static float P0[6] = REMOTE_SENSING_KALMAN_P0;
static float Q0[6] = REMOTE_SENSING_KALMAN_Q0;

/* Initialize the landing algorithm outputs struct*/
struct landing_algorithm_outputs_t landing_algorithm_outputs;

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

static void send_landing_algorithm_outputs_periodic(struct transport_tx *trans, struct link_device *dev) {  
  pprz_msg_send_LANDING_ALGORITHM_OUTPUT(trans, dev, AC_ID,
            &landing_algorithm_outputs.timestamp_output,
            landing_algorithm_outputs.UAV_acc_target_NED,
            landing_algorithm_outputs.UAV_desired_phi_theta_rad,
            &landing_algorithm_outputs.landing_algorithm_mode,
            &landing_algorithm_outputs.exitflag_path_planner,
            &landing_algorithm_outputs.expected_landing_time,
            landing_algorithm_outputs.V_out_of_bounds_array,
            landing_algorithm_outputs.A_out_of_bounds_array);
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

  // Convert absolute position to relative position
  struct NedCoor_f *uav_pos = stateGetPositionNed_f();
  VECT3_SUB(target.pos, *uav_pos);

  // Save the relative position and velocity in the target structure
  #if REMOTE_SENSING_KALMAN_USE_GROUND_STATION
    target_pos_kalman_set_measurement(&target.kalman_sensor, POS_SPEED_TO_ARRAY(target.pos, target.vel));
    target_pos_kalman_update(&remote_sensing_kalman, &target.kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_BOX, &target.pos);
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
  sensor_to_NED(&falcon.sixdof.pos, &p_rot, &falcon.sensor_to_body, &falcon.body_to_sensor_offset);
  sensor_to_NED(&falcon.sixdof.pos_var, &p_var_rot, &falcon.sensor_to_body, &(struct FloatVect3){0, 0, 0});

  //Fill up the falcon structure
  falcon.sixdof.tow = get_sys_time_tow();

  #if REMOTE_SENSING_KALMAN_USE_FALCON_SIXDOF
  // Update the kalman filter with the new position and position variance
  target_pos_kalman_set_measurement(&falcon.sixdof.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.sixdof.pos));
  target_pos_kalman_set_noise(&falcon.sixdof.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.sixdof.pos_var));
  target_pos_kalman_update(&remote_sensing_kalman, &falcon.sixdof.kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_SIXDOF, &falcon.sixdof.pos);
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
  sensor_to_NED(&falcon.relangle.pos, &p_out_sensor, &falcon.sensor_to_body, &falcon.body_to_sensor_offset); // Rotate the position to body frame
  
  #if REMOTE_SENSING_KALMAN_USE_FALCON_RELANGLE
  // Update the kalman filter with the new angles and intensity.
  target_pos_kalman_set_measurement(&falcon.relangle.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.relangle.pos));
  target_pos_kalman_update(&remote_sensing_kalman, &falcon.relangle.kalman_sensor); 
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_RELANGLE, &falcon.relangle.pos);
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

  sensor_to_NED(&falcon.relbeacon.pos, &(struct FloatVect3){pos[0], pos[1], pos[2]}, &falcon.sensor_to_body, &falcon.body_to_sensor_offset); // Rotate the position to NED frame

  #if REMOTE_SENSING_KALMAN_USE_FALCON_RELBEACON
  // Update the kalman filter with the beacon position.
  target_pos_kalman_set_measurement(&falcon.relbeacon.kalman_sensor, FLOATVECT3_TO_ARRAY(falcon.relbeacon.pos));
  target_pos_kalman_update(&remote_sensing_kalman, &falcon.relbeacon.kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_RELBEACON, &falcon.relbeacon.pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_RELBEACON);});
}

/**
 * Receive a IMCU_LANDING_ALGORITHM_OUTPUT message from the falcon and update the kalman filter if required
 */
void receive_landing_algorithm_outputs(uint8_t *buf) 
{
  uint32_t timestamp_output = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_timestamp_output(buf);
  float *UAV_acc_target_NED = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_UAV_acc_target_NED(buf);
  float *UAV_desired_phi_theta_rad = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_UAV_desired_phi_theta_rad(buf);
  int8_t landing_algorithm_mode = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_landing_algorithm_mode(buf);
  int8_t exitflag_path_planner = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_exitflag_path_planner(buf);
  float expected_landing_time = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_expected_landing_time(buf);
  uint8_t *V_out_of_bounds_array = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_V_out_of_bounds_array(buf);
  uint8_t *A_out_of_bounds_array = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_A_out_of_bounds_array(buf);

  //Fill up the landing algorithm outputs structure
  landing_algorithm_outputs.timestamp_output = timestamp_output;
  landing_algorithm_outputs.UAV_acc_target_NED[0] = UAV_acc_target_NED[0];
  landing_algorithm_outputs.UAV_acc_target_NED[1] = UAV_acc_target_NED[1];
  landing_algorithm_outputs.UAV_acc_target_NED[2] = UAV_acc_target_NED[2];
  landing_algorithm_outputs.UAV_desired_phi_theta_rad[0] = UAV_desired_phi_theta_rad[0];
  landing_algorithm_outputs.UAV_desired_phi_theta_rad[1] = UAV_desired_phi_theta_rad[1];
  landing_algorithm_outputs.landing_algorithm_mode = landing_algorithm_mode;
  landing_algorithm_outputs.exitflag_path_planner = exitflag_path_planner;
  landing_algorithm_outputs.expected_landing_time = expected_landing_time;
  for (int i = 0; i < 6; i++) {
    landing_algorithm_outputs.V_out_of_bounds_array[i] = V_out_of_bounds_array[i];
    landing_algorithm_outputs.A_out_of_bounds_array[i] = A_out_of_bounds_array[i];
  }

  // Send the current state of the remote_sensing module
  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
  pprz_msg_send_LANDING_ALGORITHM_OUTPUT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
            &timestamp_output,
            UAV_acc_target_NED,
            UAV_desired_phi_theta_rad,
            &landing_algorithm_mode,
            &exitflag_path_planner,
            &expected_landing_time,
            V_out_of_bounds_array,
            A_out_of_bounds_array);
  #endif
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

void send_landing_algorithm_params(void){
  // Send the landing algorithm parameters
  float PH_offset_ship_ctr[3] = {0.0f, 0.0f, 0.0f}; // Offset of the PH waypoint in the ship control reference frame
  float line_approach_speed = 1.0f; // Line approach speed in m/s
  float approach_line_angle_rad = 0.0f; // Angle with respect to the ship stern where the approach will be started
  float dist_line_gain = 1.0f; // Distance line gain
  float max_line_gain = 1.0f; // Maximum line gain
  float vel_gain_approach[3] = {1.0f, 1.0f, 1.0f}; // Velocity gain of the approach landing phase
  float pos_gain_hovering[3] = {1.0f, 1.0f, 1.0f}; // Position gain of the hovering landing phase
  float vel_gain_hovering[3] = {1.0f, 1.0f, 1.0f}; // Velocity gain of the hovering landing phase
  float hovering_engage_dist = 1.0f; // Euclidean distance for engagement of the PH hovering mode
  float min_time_landing_trajectory = 0.1f; // Minimum time for landing
  float max_time_landing_trajectory = 10.0f; // Maximum time for landing
  float landing_time_resolution_trajectory = 0.1f; // Resolution of the landing trajectopry generation 
  float V_bound_max_control[3] = {5.0f, 5.0f, 5.0f}; // Upper speed bounds for the trajectory generation in the control RF
  float V_bound_min_control[3] = {-5.0f, -5.0f, -5.0f}; // Lower speed bounds for the trajectory generation in the control RF
  float A_bound_max_control[3] = {5.0f, 5.0f, 5.0f}; // Upper acc bounds for the trajectory generation in the control RF
  float A_bound_min_control[3] = {-5.0f, -5.0f, -5.0f}; // Lower acc bounds for the trajectory generation in the control RF
  uint16_t num_points = 13; // Number of points where the speed and acc boundaries are checked
  float pos_gain_trajectory[3] = {1.0f, 1.0f, 1.0f}; // Position gain of the trajectory landing phase
  float vel_gain_trajectory[3] = {1.0f, 1.0f, 1.0f}; // Velocity gain of the trajectory landing phase
  float flare_engage_height = 1.0f; // Engaging height for the flare manoeuvre
  float flare_vertical_speed = 1.0f; // Vertical speed of the flare manoeuvre
  float pos_gain_flare[3] = {1.0f, 1.0f, 1.0f}; // Position gain of the flare landing phase
  float vel_gain_flare[3] = {1.0f, 1.0f, 1.0f}; // Velocity gain of the flare landing phase

  // Send the landing algorithm parameters
  pprz_msg_send_IMCU_LANDING_ALGORITHM_PARAMS(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, 
    PH_offset_ship_ctr, &line_approach_speed, &approach_line_angle_rad, &dist_line_gain, &max_line_gain,
    vel_gain_approach, pos_gain_hovering, vel_gain_hovering, &hovering_engage_dist,
    &min_time_landing_trajectory, &max_time_landing_trajectory, &landing_time_resolution_trajectory,
    V_bound_max_control, V_bound_min_control, A_bound_max_control, A_bound_min_control,
    &num_points, pos_gain_trajectory, vel_gain_trajectory,
    &flare_engage_height, &flare_vertical_speed, pos_gain_flare, vel_gain_flare);
}

void request_landing_algorithm_outputs(void){

  // To request landing commands we just need to send the landing algorithm states message
  uint32_t timestamp_states = get_sys_time_tow();

  // Current UAV position and speed in the NED frame
  float P0_UAV_NED[3] = {stateGetPositionNed_f()->x, stateGetPositionNed_f()->y, stateGetPositionNed_f()->z}; // Current UAV position in the NED frame
  float V0_UAV_NED[3] = {stateGetSpeedNed_f()->x, stateGetSpeedNed_f()->y, stateGetSpeedNed_f()->z}; // Current UAV speed in the NED frame

  //Ship states obtained from the kalman filter:
  float P0_ship_NED[3] = {0, 0, 0}; // Current ship position in the NED frame
  float V0_ship_NED[3] = {0, 0, 0}; // Current ship speed in the NED frame
  float V0_ship_NED_filt[3] = {0, 0, 0}; // Filtered ship speed in the NED frame
  float SHIP_att_rad[3] = {0, 0, 0}; // Ship attitude in radians
  float UAV_psi_rad = 0.0f; // UAV psi angle in radians

  //Coefficients for the ship speed prediction obtained from the ship speed prediction algorithm
  float coeffs_ship_prediction[24] = {
    0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f,
    0.9f, 1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f,
    1.7f, 1.8f, 1.9f, 2.0f, 2.1f, 2.2f, 2.3f, 2.4f};
  float t_delay_ship_prediction_seconds = 0.0f; // Time delay of ship predictions in seconds

  // Send the landing algorithm parameters
  pprz_msg_send_IMCU_LANDING_ALGORITHM_STATES(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, 
    &timestamp_states, P0_UAV_NED, V0_UAV_NED,
    P0_ship_NED, V0_ship_NED, V0_ship_NED_filt,
    SHIP_att_rad, &UAV_psi_rad,
    coeffs_ship_prediction, &t_delay_ship_prediction_seconds);
}
#endif

/**
 * Receive an OPENCV aruco message from the camera and update the kalman filter if required
 */
void remote_sensing_parse_opencv_aruco(uint8_t *buf) 
{
  aruco.tow = get_sys_time_tow();
  aruco.id = pprzlink_get_DL_IMCU_OPENCV_ARUCO_id(buf);
  float *pos = pprzlink_get_DL_IMCU_OPENCV_ARUCO_pos(buf);

  sensor_to_NED(&aruco.pos, &(struct FloatVect3){pos[0], pos[1], pos[2]}, &aruco.sensor_to_body, &aruco.body_to_sensor_offset); // Rotate the position to NED frame

  #if REMOTE_SENSING_KALMAN_USE_OPENCV_ARUCO
  // Update the kalman filter with the aruco position.
  target_pos_kalman_set_measurement(&aruco.kalman_sensor, FLOATVECT3_TO_ARRAY(aruco.pos));
  target_pos_kalman_update(&remote_sensing_kalman, &aruco.kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  update_waypoint(WP_ARUCO, &aruco.pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_ARUCO);});
}

void remote_sensing_AM_kalman_filter_init(float r __attribute__((unused))) {
  target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);
}

void remote_sensing_AM_init(void)
{

  //Init function
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_REMOTE_SENSING_AM, send_remote_sensing_am_periodic);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LANDING_ALGORITHM_OUTPUT, send_landing_algorithm_outputs_periodic);
  #endif

  /* Initialize the linear Kalman filter */
  target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);

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
  nav_hybrid_set_wp_speed(&speed);

  // Check for NaN
  if (isnan(pos.x) || isnan(pos.y) || isnan(pos.z) || isnan(speed.x) || isnan(speed.y) || isnan(speed.z)) {
    // Reset the kalman filter
    target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);
  } 

  update_waypoint(WP_KALMAN, &pos);
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {send_waypoint(WP_KALMAN);});

  #if !USE_NPS
  //Test the landing algorithm: 
  RunOnceEvery(50*REMOTE_SENSING_AM_PERIODIC_FREQ, {send_landing_algorithm_params();});
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {request_landing_algorithm_outputs();});

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

static void sensor_to_NED(struct FloatVect3 *ned, struct FloatVect3 *sensor, struct FloatQuat *sensor_to_body, struct FloatVect3 *offset) {
  #if !USE_NPS
  // From sensor to body frame
  struct FloatVect3 body;
  float_quat_vmult(&body, sensor_to_body, sensor); // Rotate the position to the body frame
  VECT3_ADD(body, *offset);
  
  // From body to NED frame
  struct FloatQuat body_to_ned;
  float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  float_quat_vmult(ned, &body_to_ned, &body); // Rotate the position to the NED frame
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
  float dist_to_target = sqrtf(VECT3_DOT_PRODUCT(pos, pos));
  
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
  remote_sensing_AM_send_falcon_cmd(mode);
}