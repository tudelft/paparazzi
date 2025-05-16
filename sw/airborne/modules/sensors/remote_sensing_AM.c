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
#include "filters/target_pos_kalman.h"
#include "generated/flight_plan.h"

#if USE_NPS
#include <stdlib.h>
#endif

uint8_t falcon_mode = FALCON_MODE_SIXDOF; // 0: no mode, 1: sixdof_mode, 2: relangle_mode, 3: relbeacon_mode

// Target_pos global variables: 
uint32_t target_pos_tow = 0; // Time of week of the target position measurement
struct FloatVect3 target_pos_vel_NED = {0, 0, 0}; // Ground station velocity in the Ground station NED frame assumed to be matching with the UAV one.
struct FloatVect3 target_pos_NED = {0, 0, 0}; // Ground station position in the UAV NED frame.
struct FloatQuat target_pos_quat = {0, 0, 0, 0}; // Ground station attitude in the Ground station NED frame.

// Generic falcon sensor defines
struct FloatQuat falcon_sensor_to_body = {0}; // Rotation of the body relative to the sensor

// Falcon sixdof sensor global variables:
uint32_t falcon_sixdof_tow = 0; // Time of week of the falcon sixdof measurement
struct FloatVect3 falcon_sixdof_pos_NED = {0, 0, 0}; // Falcon sensor position in the UAV NED frame.
struct FloatVect3 falcon_sixdof_pos_NED_var = {0, 0, 0}; // Falcon sensor position variance in the UAV NED frame.
struct FloatQuat falcon_sixdof_quat = {0, 0, 0, 0}; // Falcon sensor attitude in the UAV NED frame.

// Falcon relangle sensor global variables:
uint32_t falcon_relangle_tow = 0; // Time of week of the falcon relangle measurement
uint16_t falcon_relangle_beacon_id = 0; // Falcon sensor beacon id.
struct AzimuthElevation falcon_relangle = {0, 0}; // Falcon sensor azimuth and elevation angles. 
float falcon_relangle_intensity = 0; // Falcon sensor intensity.
float falcon_relangle_width = 0; // Falcon sensor width.
float falcon_relangle_distance = 1; // Relangle distance slider in meters

// Falcon relbeacon sensor global variables:
uint32_t falcon_relbeacon_tow = 0; // Time of week of the falcon relbeacon measurement
uint16_t falcon_relbeacon_beacon_id = 0; // Falcon sensor beacon id.
struct FloatVect3 falcon_relbeacon_pos = {0, 0, 0}; // Falcon sensor position in the sensor body frame.

// Opencv_aruco sensor global variables:
uint32_t opencv_aruco_tow = 0; // Time of week of the opencv aruco measurement
uint16_t opencv_aruco_id = 0; // Opencv aruco id.
struct FloatVect3 opencv_aruco_pos = {0, 0, 0}; // Opencv aruco position in the UAV NED frame.
struct FloatQuat opencv_aruco_quat = {0, 0, 0, 0}; // Opencv aruco attitude in the UAV NED frame.
struct FloatQuat opencv_aruco_sensor_to_body = {0}; // Rotation of the body relative to the sensor

/* Initialize the kalman filter structs */
struct TargetPosKalman remote_sensing_kalman;
float P0[6] = {10., 10., 10., 10., 10., 10.};
float Q0[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

struct KalmanSensor target_pos_kalman_sensor = { // Kalman sensor for the target position
  .noise = {0.1, 1.f, 0.1, 1.f, 0.1, 1.f}, // Process noise
  .meas = {0, 0, 0, 0, 0, 0}, // Measurement
  .n_meas = 6, // Number of measurements
  .Hmat = {{1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // Measurement matrix
           {0.f, 1.f, 0.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
           {0.f, 0.f, 0.f, 1.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}}
};

struct KalmanSensor falcon_kalman_sensor = {
  .noise = {0.1, 0.1, 0.1, 0.0, 0.0, 0.0}, // Process noise
  .meas = {0, 0, 0, 0, 0, 0}, // Measurement
  .n_meas = 3, // Number of measurements
  .Hmat = {{1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // Measurement matrix
           {0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 1.f, 0.f},
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}
};

struct KalmanSensor aruco_kalman_sensor = {
  .noise = {0.1, 0.1, 0.1, 0.0, 0.0, 0.0}, // Process noise
  .meas = {0, 0, 0, 0, 0, 0}, // Measurement
  .n_meas = 3, // Number of measurements
  .Hmat = {{1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // Measurement matrix
           {0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 1.f, 0.f},
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}
};

#if PERIODIC_TELEMETRY
static void send_remote_sensing_am_periodic(struct transport_tx *trans, struct link_device *dev)
{
  float target_pos_vel_NED_T[3] = {target_pos_vel_NED.x, target_pos_vel_NED.y, target_pos_vel_NED.z};
  float target_pos_NED_T[3] = {target_pos_NED.x, target_pos_NED.y, target_pos_NED.z};
  float target_pos_quat_T[4] = {target_pos_quat.qi, target_pos_quat.qx, target_pos_quat.qy, target_pos_quat.qz};

  float falcon_sixdof_pos_NED_T[3] = {falcon_sixdof_pos_NED.x, falcon_sixdof_pos_NED.y, falcon_sixdof_pos_NED.z};
  float falcon_sixdof_pos_NED_var_T[3] = {falcon_sixdof_pos_NED_var.x, falcon_sixdof_pos_NED_var.y, falcon_sixdof_pos_NED_var.z};
  float falcon_sixdof_quat_T[4] = {falcon_sixdof_quat.qi, falcon_sixdof_quat.qx, falcon_sixdof_quat.qy, falcon_sixdof_quat.qz};

  float falcon_relangle_T[2] = {falcon_relangle.azimuth, falcon_relangle.elevation};

  float falcon_relbecon_pos_T[3] = {falcon_relbeacon_pos.x, falcon_relbeacon_pos.y, falcon_relbeacon_pos.z};

  float opencv_aruco_pos_T[3] = {opencv_aruco_pos.x, opencv_aruco_pos.y, opencv_aruco_pos.z};
  float opencv_aruco_quat_T[4] = {opencv_aruco_quat.qi, opencv_aruco_quat.qx, opencv_aruco_quat.qy, opencv_aruco_quat.qz};
  
  pprz_msg_send_REMOTE_SENSING_AM(trans, dev, AC_ID,
            &target_pos_tow,
            target_pos_vel_NED_T,
            target_pos_NED_T,
            target_pos_quat_T,
            &falcon_sixdof_tow,
            falcon_sixdof_pos_NED_T,
            falcon_sixdof_pos_NED_var_T,
            falcon_sixdof_quat_T,
            &falcon_relangle_tow,
            &falcon_relangle_beacon_id,
            falcon_relangle_T,
            &falcon_relangle_intensity,
            &falcon_relangle_width,
            &falcon_relbeacon_tow,
            &falcon_relbeacon_beacon_id,
            falcon_relbecon_pos_T,
            &opencv_aruco_tow,
            &opencv_aruco_id,
            opencv_aruco_pos_T,
            opencv_aruco_quat_T);
}

#endif


#if !USE_NPS
//Function to upload telemetry: 
void sdlog_remote_sensing_am(void){ 

  float target_pos_vel_NED_T[3] = {target_pos_vel_NED.x, target_pos_vel_NED.y, target_pos_vel_NED.z};
  float target_pos_NED_T[3] = {target_pos_NED.x, target_pos_NED.y, target_pos_NED.z};
  float target_pos_quat_T[4] = {target_pos_quat.qi, target_pos_quat.qx, target_pos_quat.qy, target_pos_quat.qz};

  float falcon_sixdof_pos_NED_T[3] = {falcon_sixdof_pos_NED.x, falcon_sixdof_pos_NED.y, falcon_sixdof_pos_NED.z};
  float falcon_sixdof_pos_NED_var_T[3] = {falcon_sixdof_pos_NED_var.x, falcon_sixdof_pos_NED_var.y, falcon_sixdof_pos_NED_var.z};
  float falcon_sixdof_quat_T[4] = {falcon_sixdof_quat.qi, falcon_sixdof_quat.qx, falcon_sixdof_quat.qy, falcon_sixdof_quat.qz};

  float falcon_relangle_T[2] = {falcon_relangle.azimuth, falcon_relangle.elevation};

  float falcon_relbecon_pos_T[3] = {falcon_relbeacon_pos.x, falcon_relbeacon_pos.y, falcon_relbeacon_pos.z};

  float opencv_aruco_pos_T[3] = {opencv_aruco_pos.x, opencv_aruco_pos.y, opencv_aruco_pos.z};
  float opencv_aruco_quat_T[4] = {opencv_aruco_quat.qi, opencv_aruco_quat.qx, opencv_aruco_quat.qy, opencv_aruco_quat.qz};

  // Send the current state of the remote_sensing module
  pprz_msg_send_REMOTE_SENSING_AM(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
    &target_pos_tow,
    target_pos_vel_NED_T,
    target_pos_NED_T,
    target_pos_quat_T,
    &falcon_sixdof_tow,
    falcon_sixdof_pos_NED_T,
    falcon_sixdof_pos_NED_var_T,
    falcon_sixdof_quat_T,
    &falcon_relangle_tow,
    &falcon_relangle_beacon_id,
    falcon_relangle_T,
    &falcon_relangle_intensity,
    &falcon_relangle_width,
    &falcon_relbeacon_tow,
    &falcon_relbeacon_beacon_id,
    falcon_relbecon_pos_T,
    &opencv_aruco_tow,
    &opencv_aruco_id,
    opencv_aruco_pos_T,
    opencv_aruco_quat_T);
}
#endif

//Send mode to the falcon system: 
void remote_sensing_AM_send_falcon_cmd(uint8_t mode) 
{
  falcon_mode = mode;

  // Reset the kalman filter noise for the falcon sensor on mode switch
  switch (falcon_mode) {
    case FALCON_MODE_SIXDOF:
      for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
        falcon_kalman_sensor.noise[i] = 0.1f;
      }
      break;
    case FALCON_MODE_RELANGLE: 
      for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
        falcon_kalman_sensor.noise[i] = 0.1f;
      }
      break;
    case FALCON_MODE_RELBEACON: 
      for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
        falcon_kalman_sensor.noise[i] = 0.1f;
      }
      break;
    default: // No mode
      break;
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
  target_pos_tow = DL_TARGET_POS_tow(buf);
  struct LlaCoor_i target_pos_lla = {DL_TARGET_POS_lat(buf), DL_TARGET_POS_lon(buf), DL_TARGET_POS_alt(buf)};
  target_pos_vel_NED.x = DL_TARGET_POS_vnorth(buf);
  target_pos_vel_NED.y = DL_TARGET_POS_veast(buf);
  target_pos_vel_NED.z = DL_TARGET_POS_vdown(buf);
  target_pos_quat.qi = DL_TARGET_POS_body_qi(buf);
  target_pos_quat.qx = DL_TARGET_POS_body_qx(buf);
  target_pos_quat.qy = DL_TARGET_POS_body_qy(buf);
  target_pos_quat.qz = DL_TARGET_POS_body_qz(buf);

  // struct FloatRates target_pos_rates = {DL_TARGET_POS_p(buf), DL_TARGET_POS_q(buf), DL_TARGET_POS_r(buf)};
  
  struct NedCoor_i target_pos_cm;
  ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target_pos_lla);
  target_pos_NED.x = target_pos_cm.x / 100.;
  target_pos_NED.y = target_pos_cm.y / 100.;
  target_pos_NED.z = target_pos_cm.z / 100.;

  // Now we have the target position in the UAV NED frame (target_pos_NED) and the target velocity in the UAV NED frame (target_pos_vel_NED).


  // Save the relative position and velocity in the target structure
  #if REMOTE_SENSING_KALMAN_USE_GROUND_STATION
    target_pos_kalman_sensor.meas[0] = target_pos_NED.x;
    target_pos_kalman_sensor.meas[2] = target_pos_NED.y;
    target_pos_kalman_sensor.meas[4] = target_pos_NED.z;
    target_pos_kalman_sensor.meas[1] = target_pos_vel_NED.x;
    target_pos_kalman_sensor.meas[3] = target_pos_vel_NED.y;
    target_pos_kalman_sensor.meas[5] = target_pos_vel_NED.z;
    target_pos_kalman_update(&remote_sensing_kalman, &target_pos_kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  uint8_t wp_id = WP_BOX;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, target_pos_NED);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);
}

/**
 * Receive a SIXDOF message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_sixdof(uint8_t *buf)
{
  float *pos = pprzlink_get_DL_IMCU_FALCON_SIXDOF_pos(buf);
  float *quat = pprzlink_get_DL_IMCU_FALCON_SIXDOF_quat(buf);
  float *pos_var = pprzlink_get_DL_IMCU_FALCON_SIXDOF_pos_var(buf);
  // float *quat_var = pprzlink_get_DL_IMCU_FALCON_SIXDOF_quat_var(buf);
  struct FloatQuat q = {quat[0], quat[1], quat[2], quat[3]}; // Rotation of the platform relative to the sensor
  struct FloatVect3 p = {pos[0], pos[1], pos[2]}; // Position of the drone relative to the platform
  struct FloatVect3 p_var = {pos_var[0], pos_var[1], pos_var[2]}; // Variance of p
  // struct FloatVect3 q_var = {quat_var[0], quat_var[1], quat_var[2]}; // Variance of q
  struct FloatVect3 p_rot, p_inv, p_out;
  struct FloatVect3 p_var_rot, p_var_inv, p_var_out;

  // Calculate the position of the platform relative the the UAV
  float_quat_vmult(&p_rot, &q, &p); // Rotate the position around the platform
  float_quat_vmult(&p_var_rot, &q, &p_var); // Rotate the position variance around the platform
  // Invert position and change from Y-Down to Z-Down reference frame
  p_inv.x = -p_rot.x + REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_X;
  p_inv.y = p_rot.z + REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_Y;
  p_inv.z = -p_rot.y + REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_Z;
  p_var_inv.x = -p_var_rot.x;
  p_var_inv.y = p_var_rot.z;
  p_var_inv.z = -p_var_rot.y;

  struct FloatQuat body_to_ned;
  float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  float_quat_vmult(&p_out, &body_to_ned, &p_inv); // Rotate the position to earth frame NED
  float_quat_vmult(&p_var_out, &body_to_ned, &p_var_inv); // Rotate the position variance to earth frame NED

  //Fill up the falcon structure
  falcon_sixdof_tow = get_sys_time_tow();
  falcon_sixdof_pos_NED.x = p_out.x;
  falcon_sixdof_pos_NED.y = p_out.y;
  falcon_sixdof_pos_NED.z = p_out.z;
  falcon_sixdof_pos_NED_var.x = p_var_out.x;
  falcon_sixdof_pos_NED_var.y = p_var_out.y;
  falcon_sixdof_pos_NED_var.z = p_var_out.z;


  #if REMOTE_SENSING_KALMAN_USE_FALCON_SIXDOF
  // Update the kalman filter with the new position and position variance
  falcon_kalman_sensor.meas[0] = falcon_sixdof_pos_NED.x;
  falcon_kalman_sensor.meas[2] = falcon_sixdof_pos_NED.y;
  falcon_kalman_sensor.meas[4] = falcon_sixdof_pos_NED.z;
  falcon_kalman_sensor.noise[0] = falcon_sixdof_pos_NED_var.x;
  falcon_kalman_sensor.noise[2] = falcon_sixdof_pos_NED_var.y;
  falcon_kalman_sensor.noise[4] = falcon_sixdof_pos_NED_var.z;
  target_pos_kalman_update(&remote_sensing_kalman, &falcon_kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  uint8_t wp_id = WP_SIXDOF;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, falcon_sixdof_pos_NED);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);

}

/**
 * Receive a RELANGLE message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_relangle(uint8_t *buf) 
{
  falcon_relangle_tow = get_sys_time_tow();
  falcon_relangle_beacon_id = pprzlink_get_DL_IMCU_FALCON_RELANGLE_id(buf);
  falcon_relangle_intensity = pprzlink_get_DL_IMCU_FALCON_RELANGLE_intensity(buf);
  falcon_relangle_width = pprzlink_get_DL_IMCU_FALCON_RELANGLE_width(buf);
  float *rel_angles = pprzlink_get_DL_IMCU_FALCON_RELANGLE_angles(buf);
  falcon_relangle.azimuth = rel_angles[0];
  falcon_relangle.elevation = rel_angles[1];

  struct FloatEulers angles = {falcon_relangle.elevation, 0.f, falcon_relangle.azimuth};
  
  /* Implement logic to go from distance and x/z angles to a relative position */
  // Temp relation for distance and intensity, depends on environment and beacon
  // float falcon_relangle_distance = 5.4165 + 75.5979 / falcon_relangle_intensity - 80.3343 / (falcon_relangle_intensity*falcon_relangle_intensity);
  
  // // Obtain the relative position in sensor frame
  struct FloatVect3 p_out_sensor;
  struct FloatVect3 p_out_body;
  struct FloatVect3 p_out_ned;
  struct FloatQuat body_to_ned;
  struct FloatQuat rel_angles_sensor;
  
  // Convert the relative angles to quaternions and multiple with the estimated distance along the sensor y axis to get the relative position
  // in the sensor frame
  float_quat_of_eulers(&rel_angles_sensor, &angles);
  float_quat_vmult(&p_out_sensor, &rel_angles_sensor, &(struct FloatVect3){0, falcon_relangle_distance, 0});

  // // Rotate the relative position to the body frame and add the sensor to c.g. translation
  float_quat_vmult(&p_out_body, &falcon_sensor_to_body, &p_out_sensor);
  
  p_out_body.x += REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_X;
  p_out_body.y += REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_Y;
  p_out_body.z += REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_Z;

  // // Rotate the relative position to the NED frame
  float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  float_quat_vmult(&p_out_ned, &body_to_ned, &p_out_body);
  
  #if REMOTE_SENSING_KALMAN_USE_FALCON_RELANGLE
  // Update the kalman filter with the new angles and intensity.
  falcon_kalman_sensor.meas[0] = p_out_ned.x;
  falcon_kalman_sensor.meas[2] = p_out_ned.y;
  falcon_kalman_sensor.meas[4] = p_out_ned.z;
  target_pos_kalman_update(&remote_sensing_kalman, &falcon_kalman_sensor); 
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  uint8_t wp_id = WP_RELANGLE;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, p_out_ned);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);

}

/**
 * Receive a RELBEACON message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_falcon_relbeacon(uint8_t *buf) 
{
  falcon_relbeacon_tow = get_sys_time_tow();
  falcon_relbeacon_beacon_id = pprzlink_get_DL_IMCU_FALCON_RELBEACON_id(buf);
  float *pos = pprzlink_get_DL_IMCU_FALCON_RELBEACON_pos(buf);
  falcon_relbeacon_pos.x = pos[0];
  falcon_relbeacon_pos.y = pos[1];
  falcon_relbeacon_pos.z = pos[2];

  //Transpose it to real drone body frame: 
  struct FloatVect3 falcon_relbeacon_pos_UAV_body;
  float_quat_vmult(&falcon_relbeacon_pos_UAV_body, &falcon_sensor_to_body, &falcon_relbeacon_pos); // Rotate the position to body frame

  falcon_relbeacon_pos_UAV_body.x += REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_X;
  falcon_relbeacon_pos_UAV_body.y += REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_Y;
  falcon_relbeacon_pos_UAV_body.z += REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_Z;

  // Rotate the relative position to the NED frame
  struct FloatQuat body_to_ned;
  struct FloatVect3 falcon_relbeacon_pos_NED;
  float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  float_quat_vmult(&falcon_relbeacon_pos_NED, &body_to_ned, &falcon_relbeacon_pos_UAV_body); // Rotate the position to earth frame NED

  #if REMOTE_SENSING_KALMAN_USE_FALCON_RELBEACON
  // Update the kalman filter with the beacon position.
  falcon_kalman_sensor.meas[0] = falcon_relbeacon_pos_NED.x;
  falcon_kalman_sensor.meas[2] = falcon_relbeacon_pos_NED.y;
  falcon_kalman_sensor.meas[4] = falcon_relbeacon_pos_NED.z;
  target_pos_kalman_update(&remote_sensing_kalman, &falcon_kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  uint8_t wp_id = WP_RELBEACON;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, falcon_relbeacon_pos_NED);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);

}

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

/**
 * Receive a RELBEACON message from the falcon and update the kalman filter if required
 */
void remote_sensing_parse_opencv_aruco(uint8_t *buf) 
{
  opencv_aruco_tow = get_sys_time_tow();
  opencv_aruco_id = pprzlink_get_DL_IMCU_OPENCV_ARUCO_id(buf);
  float *pos = pprzlink_get_DL_IMCU_OPENCV_ARUCO_pos(buf);
  opencv_aruco_pos.x = pos[0];
  opencv_aruco_pos.y = pos[1];
  opencv_aruco_pos.z = pos[2];

  //Transpose it to real drone body frame: 
  struct FloatVect3 opencv_aruco_pos_UAV_body;
  float_quat_vmult(&opencv_aruco_pos_UAV_body, &opencv_aruco_sensor_to_body, &opencv_aruco_pos); // Rotate the position to body frame

  opencv_aruco_pos_UAV_body.x += REMOTE_SENSING_OPENCV_ARUCO_OFFSET_UAV_BODY_X;
  opencv_aruco_pos_UAV_body.y += REMOTE_SENSING_OPENCV_ARUCO_OFFSET_UAV_BODY_Y;
  opencv_aruco_pos_UAV_body.z += REMOTE_SENSING_OPENCV_ARUCO_OFFSET_UAV_BODY_Z;

  // Rotate the relative position to the NED frame
  struct FloatQuat body_to_ned;
  struct FloatVect3 opencv_aruco_pos_NED;
  float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  float_quat_vmult(&opencv_aruco_pos_NED, &body_to_ned, &opencv_aruco_pos_UAV_body); // Rotate the position to earth frame NED

  #if REMOTE_SENSING_KALMAN_USE_OPENCV_ARUCO
  // Update the kalman filter with the aruco position.
  aruco_kalman_sensor.meas[0] = opencv_aruco_pos_NED.x;
  aruco_kalman_sensor.meas[2] = opencv_aruco_pos_NED.y;
  aruco_kalman_sensor.meas[4] = opencv_aruco_pos_NED.z;
  target_pos_kalman_update(&remote_sensing_kalman, &aruco_kalman_sensor);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL && !USE_NPS
    sdlog_remote_sensing_am();
  #endif

  // Update a position in the flight plan for now
  uint8_t wp_id = WP_ARUCO;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, opencv_aruco_pos_NED);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);

}

void remote_sensing_AM_kalman_filter_init(float r __attribute__((unused))) {
  target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);
}

void remote_sensing_AM_init(void)
{
  //Init function
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_REMOTE_SENSING_AM, send_remote_sensing_am_periodic);
  #endif

  /* Initialize the linear Kalman filter */
  target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);
  
  /* Store sensor to body rotation for falcon and aruco camera */
  float_quat_of_eulers(&falcon_sensor_to_body, // ZYX order
    &(struct FloatEulers) {RadOfDeg(REMOTE_SENSING_FALCON_ROTATION_SENSOR_TO_BODY_X), 
                           RadOfDeg(REMOTE_SENSING_FALCON_ROTATION_SENSOR_TO_BODY_Y), 
                           RadOfDeg(REMOTE_SENSING_FALCON_ROTATION_SENSOR_TO_BODY_Z)});

  float_quat_of_eulers(&opencv_aruco_sensor_to_body, // ZYX order
    &(struct FloatEulers) {RadOfDeg(REMOTE_SENSING_OPENCV_ARUCO_ROTATION_SENSOR_TO_BODY_X), 
                           RadOfDeg(REMOTE_SENSING_OPENCV_ARUCO_ROTATION_SENSOR_TO_BODY_Y), 
                           RadOfDeg(REMOTE_SENSING_OPENCV_ARUCO_ROTATION_SENSOR_TO_BODY_Z)});
}

void remote_sensing_AM_periodic(void) {
  
#if !TARTGET_POS_GROUND_STATION
#if USE_NPS
  // Fake some target pos data
  // Falcon every 50 Hz
  static int N = 10;
  falcon_kalman_sensor.meas[0] = stateGetPositionNed_f()->x + ((float)(rand() % (N + 1)) / N);
  falcon_kalman_sensor.meas[2] = stateGetPositionNed_f()->y + ((float)(rand() % (N + 1)) / N);
  falcon_kalman_sensor.meas[4] = stateGetPositionNed_f()->z + ((float)(rand() % (N + 1)) / N);

  RunOnceEvery(1, {
    target_pos_kalman_update(&remote_sensing_kalman, &falcon_kalman_sensor);
  });

  aruco_kalman_sensor.meas[0] = stateGetPositionNed_f()->x + ((float)(rand() % (N + 1)) / N * 3);
  aruco_kalman_sensor.meas[2] = stateGetPositionNed_f()->y + ((float)(rand() % (N + 1)) / N * 3);
  aruco_kalman_sensor.meas[4] = stateGetPositionNed_f()->z + ((float)(rand() % (N + 1)) / N * 3);

  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ / 10, {
    target_pos_kalman_update(&remote_sensing_kalman, &aruco_kalman_sensor);
  });
  
  target_pos_kalman_sensor.meas[0] = stateGetPositionNed_f()->x + ((float)(rand() % (N + 1)) / N / 2);
  target_pos_kalman_sensor.meas[2] = stateGetPositionNed_f()->y + ((float)(rand() % (N + 1)) / N / 2);
  target_pos_kalman_sensor.meas[4] = stateGetPositionNed_f()->z + ((float)(rand() % (N + 1)) / N / 2);
  target_pos_kalman_sensor.meas[1] = stateGetSpeedNed_f()->x + ((float)(rand() % (N + 1)) / N);
  target_pos_kalman_sensor.meas[3] = stateGetSpeedNed_f()->y + ((float)(rand() % (N + 1)) / N);
  target_pos_kalman_sensor.meas[5] = stateGetSpeedNed_f()->z + ((float)(rand() % (N + 1)) / N);
    
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ / 8, {
    target_pos_kalman_update(&remote_sensing_kalman, &target_pos_kalman_sensor);
  });
#endif
#endif
  
  // Here we can run the periodic KF update
  target_pos_kalman_predict(&remote_sensing_kalman);

  // Get Kalman state
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  target_pos_kalman_get_state(&remote_sensing_kalman, &pos, &speed);

  uint8_t wp_id;

  // Check for NaN
  if (isnan(pos.x) || isnan(pos.y) || isnan(pos.z) || isnan(speed.x) || isnan(speed.y) || isnan(speed.z)) {
    // Reset the kalman filter
    target_pos_kalman_init(&remote_sensing_kalman, P0, Q0, 1/REMOTE_SENSING_AM_PERIODIC_FREQ);
  } else {
    // Update a position in the flight plan for now
    wp_id = WP_KALMAN;
    struct EnuCoor_f target_enu;
    struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
    ENU_OF_TO_NED(target_enu, pos);
    VECT3_ADD(target_enu, *uav_pos);
    target_enu.z = waypoints[wp_id].enu_f.z;
    waypoint_set_enu(wp_id, &target_enu);
  }

  // Send waypoint update every half second
  RunOnceEvery(REMOTE_SENSING_AM_PERIODIC_FREQ, {

    // test_request_landing_path();
    
    // Send to the GCS that the waypoint has been moved
    wp_id = WP_KALMAN;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
      &waypoints[wp_id].enu_i.x,
      &waypoints[wp_id].enu_i.y,
      &waypoints[wp_id].enu_i.z);
    
    wp_id = WP_SIXDOF;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
      &waypoints[wp_id].enu_i.x,
      &waypoints[wp_id].enu_i.y,
      &waypoints[wp_id].enu_i.z);

    wp_id = WP_RELANGLE;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
      &waypoints[wp_id].enu_i.x,
      &waypoints[wp_id].enu_i.y,
      &waypoints[wp_id].enu_i.z);

    wp_id = WP_RELBEACON;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
      &waypoints[wp_id].enu_i.x,
      &waypoints[wp_id].enu_i.y,
      &waypoints[wp_id].enu_i.z);
    
    wp_id = WP_ARUCO;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
      &waypoints[wp_id].enu_i.x,
      &waypoints[wp_id].enu_i.y,
      &waypoints[wp_id].enu_i.z);

    wp_id = WP_BOX;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
      &waypoints[wp_id].enu_i.x,
      &waypoints[wp_id].enu_i.y,
      &waypoints[wp_id].enu_i.z);
    });

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

