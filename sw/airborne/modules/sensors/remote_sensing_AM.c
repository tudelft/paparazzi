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

uint8_t falcon_mode = 1; // 0: no mode, 1: sixdof_mode, 2: relangle_mode, 3: relbeacon_mode

//Target_pos global variables: 
uint32_t target_pos_tow = 0; // Time of week of the target position measurement
struct FloatVect3 target_pos_vel_NED = {0, 0, 0}; // Ground station velocity in the Ground station NED frame assumed to be matching with the UAV one.
struct FloatVect3 target_pos_NED = {0, 0, 0}; // Ground station position in the UAV NED frame.
struct FloatQuat target_pos_quat = {0, 0, 0, 0}; // Ground station attitude in the Ground station NED frame.

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

// Falcon relbeacon sensor global variables:
uint32_t falcon_relbeacon_tow = 0; // Time of week of the falcon relbeacon measurement
uint16_t falcon_relbeacon_beacon_id = 0; // Falcon sensor beacon id.
struct FloatVect3 falcon_relbeacon_pos = {0, 0, 0}; // Falcon sensor position in the sensor body frame.

// Opencv_aruco sensor global variables:
uint32_t opencv_aruco_tow = 0; // Time of week of the opencv aruco measurement
uint16_t opencv_aruco_id = 0; // Opencv aruco id.
struct FloatVect3 opencv_aruco_pos = {0, 0, 0}; // Opencv aruco position in the UAV NED frame.
struct FloatQuat opencv_aruco_quat = {0, 0, 0, 0}; // Opencv aruco attitude in the UAV NED frame.


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

//Send mode to the falcon system: 
void remote_sensing_AM_send_falcon_cmd(uint8_t mode) 
{
  falcon_mode = mode;
  pprz_msg_send_IMCU_FALCON_CMD(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, &falcon_mode);
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

  //Now we have the target position in the UAV NED frame (target_pos_NED) and the target velocity in the UAV NED frame (target_pos_vel_NED).


  //Save the relative position and velocity in the target structure
  #if REMOTE_SENSING_KALMAN_USE_GROUND_STATION
  static struct KalmanSensor ground_station_kalman = {
    .noise = {0.1f, 1.f, 0.1f, 1.f, 0.1f, 1.f}, // not determined yet!
    .meas = {0, 0, 0, 0, 0, 0},
    .Hmat = {{1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
            {0.f, 1.f, 0.f, 0.f, 0.f, 0.f}, 
            {0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
            {0.f, 0.f, 0.f, 1.f, 0.f, 0.f}, 
            {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, 
            {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}}
  };

  ground_station_kalman.meas[0] = pos.x;
  ground_station_kalman.meas[2] = pos.y;
  ground_station_kalman.meas[4] = pos.z;
  ground_station_kalman.meas[1] = vel.x;
  ground_station_kalman.meas[3] = vel.y;
  ground_station_kalman.meas[5] = vel.z;
  target_pos_kalman_update(&target_pos_kalman, &ground_station_kalman);
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL
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
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL
    sdlog_remote_sensing_am();
  #endif

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


  // struct FloatEulers angles = {falcon_relangle.elevation, 0.f, falcon_relangle.azimuth};
  
  /* Implement logic to go from distance and x/z angles to a relative position */
  // Temp relation for distance and intensity, depends on environment and beacon
  // float falcon_distance = 5.4165 + 75.5979 / falcon_relangle_intensity - 80.3343 / (falcon_relangle_intensity*falcon_relangle_intensity);
  
  // // Obtain the relative position in sensor frame
  // struct FloatVect3 p_out_sensor;
  // struct FloatVect3 p_out_body;
  // struct FloatVect3 p_out_ned;
  // struct FloatRMat rel_angles_sensor;
  // float_rmat_of_eulers_321(&rel_angles_sensor, &angles);
  // float_rmat_vmult(&p_out_sensor, &rel_angles_sensor, &(struct FloatVect3){0, falcon_distance, 0});

  // // Rotate the relative position to the body frame and add the sensor to c.g. translation
  // float_rmat_vmult(&p_out_body, &falcon.sensor_to_body_rotation, &p_out_sensor);
  // VECT3_ADD(p_out_body, falcon.body_to_sensor_translation);

  // // Rotate the relative position to the NED frame
  // struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
  // float_rmat_transp_vmult(&p_out_ned, ned_to_body, &p_out_body);
  
  #if REMOTE_SENSING_KALMAN_USE_FALCON_RELANGLE
  // Update the kalman filter with the new angles and intensity. 
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL
    sdlog_remote_sensing_am();
  #endif

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
  falcon_relbeacon_pos_UAV_body.x = -falcon_relbeacon_pos.x + REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_X;
  falcon_relbeacon_pos_UAV_body.y = falcon_relbeacon_pos.z + REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_Y;
  falcon_relbeacon_pos_UAV_body.z = -falcon_relbeacon_pos.y + REMOTE_SENSING_FALCON_OFFSET_UAV_BODY_Z;


  // Rotate the relative position to the NED frame
  // struct FloatQuat body_to_ned;
  // struct FloatVect3 falcon_relbeacon_pos_NED;
  // float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  // float_quat_vmult(&falcon_relbeacon_pos_NED, &body_to_ned, &falcon_relbeacon_pos_UAV_body); // Rotate the position to earth frame NED

  #if REMOTE_SENSING_KALMAN_USE_FALCON_RELBEACON
  // Update the kalman filter with the beacon position.
  falcon_relbeacon_pos_UAV_body.x = falcon_relbeacon_pos_UAV_body.x + 1;

  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL
    sdlog_remote_sensing_am();
  #endif
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
  opencv_aruco_pos_UAV_body.x = opencv_aruco_pos.x + REMOTE_SENSING_OPENCV_ARUCO_OFFSET_UAV_BODY_X;
  opencv_aruco_pos_UAV_body.y = opencv_aruco_pos.z + REMOTE_SENSING_OPENCV_ARUCO_OFFSET_UAV_BODY_Y;
  opencv_aruco_pos_UAV_body.z = opencv_aruco_pos.y + REMOTE_SENSING_OPENCV_ARUCO_OFFSET_UAV_BODY_Z;

  // Rotate the relative position to the NED frame
  // struct FloatQuat body_to_ned;
  // struct FloatVect3 opencv_aruco_pos_NED;
  // float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  // float_quat_vmult(&opencv_aruco_pos_NED, &body_to_ned, &opencv_aruco_pos_UAV_body); // Rotate the position to earth frame NED


  #if REMOTE_SENSING_KALMAN_USE_OPENCV_ARUCO
    // Update the kalman filter with the aruco position.
    opencv_aruco_pos_UAV_body.x = opencv_aruco_pos_UAV_body.x + 1;
  #endif

  #if REMOTE_SENSING_LOG_ON_ARRIVAL
    sdlog_remote_sensing_am();
  #endif

}


void remote_sensing_AM_init(void)
{
  //Init function
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_REMOTE_SENSING_AM, send_remote_sensing_am_periodic);
  #endif
}

void remote_sensing_AM_periodic(void) {
  // Here we can run the periodic KF update
}

