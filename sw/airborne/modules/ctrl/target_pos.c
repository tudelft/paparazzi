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
 * @file "modules/ctrl/target_pos.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Control a rotorcraft to follow at a defined distance from the target
 */

#include "target_pos.h"
#include <math.h>

#include "pprzlink/intermcu_msg.h"
#include "modules/datalink/telemetry.h"
#include "modules/core/abi.h"
#include "modules/datalink/downlink.h"
#include "filters/target_pos_kalman.h"

#ifndef TARGET_POS_GROUND_STATION
#define TARGET_POS_GROUND_STATION false
#endif

// The timeout when receiving GPS messages from the ground in ms
#ifndef TARGET_POS_TIMEOUT
#define TARGET_POS_TIMEOUT 5000
#endif

// The timeout when receiving an RTK gps message from the GPS
#ifndef TARGET_RTK_TIMEOUT
#define TARGET_RTK_TIMEOUT 1000
#endif

/* Falcon sensor to body rotation angles */
#ifndef FALCON_X_ANGLE 
#define FALCON_X_ANGLE 0
#endif

#ifndef FALCON_Y_ANGLE
#define FALCON_Y_ANGLE 0
#endif

#ifndef FALCON_Z_ANGLE
#define FALCON_Z_ANGLE 0
#endif

/* Body frame to sensor frame translation in body frame */
#ifndef BODY_FALCON_X_DIST 
#define BODY_FALCON_X_DIST 0
#endif

#ifndef BODY_FALCON_Y_DIST
#define BODY_FALCON_Y_DIST 0
#endif

#ifndef BODY_FALCON_Z_DIST
#define BODY_FALCON_Z_DIST 0
#endif

#ifndef TARGET_POS_RELHEADING_REF_ID
#define TARGET_POS_RELHEADING_REF_ID 0 
#endif

#ifndef TARGET_POS_KALMAN_USE_FALCON
#define TARGET_POS_KALMAN_USE_FALCON false
#endif

#ifndef TARGET_POS_KALMAN_USE_ARUCO
#define TARGET_POS_KALMAN_USE_ARUCO false
#endif

#ifndef TARGET_POS_KALMAN_USE_GROUND_STATION
#define TARGET_POS_KALMAN_USE_GROUND_STATION false
#endif

#ifndef TARGET_POS_KALMAN_USE_LIDAR
#define TARGET_POS_KALMAN_USE_LIDAR false
#endif

/* Initialize the main structure */
struct target_t target = {
  .pos = {0},
  .offset = {0},
  .target_pos_timeout = TARGET_POS_TIMEOUT,
  .rtk_timeout = TARGET_RTK_TIMEOUT,
};

/* Initialize falcon sensor structure */
const struct KalmanSensor falcon_kalman = {
  .noise = {0.1, 999.f, 0.1, 999.f, 0.1, 999.f}, // not determined yet!
  .meas = {0, 0, 0, 0, 0, 0},
  .Hmat = {{1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, 
           {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}
};



struct falcon_sensor_t falcon = {
  .valid = false,     // Assume invalid data on start up
  .manual = false,    // By default the tracking mode should be determined automatically
  .mode = 0,          // Initialize falcon sensor tracking mode to off
  .beacon_id = 0,
  .body_to_sensor_translation = {
    .x = BODY_FALCON_X_DIST,
    .y = BODY_FALCON_Y_DIST,
    .z = BODY_FALCON_Z_DIST,
  },
  .p_out = {0},
  .p_in = {0},
  .q = {0},
  .p_var = {0},
  .q_var = {0},
  .angles = {0},
  .intensity = 0,
  .width = 0,
  .distance = 0.f,
  .kalman = falcon_kalman
};

/* Initialize the linear kalman filter struct */
struct TargetPosKalman target_pos_kalman;
float P0[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
float Q0[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

/* GPS abi callback */
static abi_event gps_ev;
static abi_event relpos_ev;
static abi_event lidar_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
static void relpos_cb(uint8_t sender_id, uint32_t stamp, struct RelPosNED *relpos);
static void lidar_cb(uint8_t sender_id, uint32_t stamp, float distance);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_target_pos_info(struct transport_tx *trans, struct link_device *dev)
{
#if TARGET_POS_GROUND_STATION
  // Send the current state of the ground station
  static const int32_t zero = 0;
  struct LlaCoor_i *pos = stateGetPositionLla_i();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  struct FloatRates *rates = stateGetBodyRates_f();
  uint32_t tow = get_sys_time_tow();

  DOWNLINK_SEND_TARGET_POS_INFO(DefaultChannel, DefaultDevice,
                              &tow,
                              &zero,
                              &pos->lat,
                              &pos->lon,
                              &pos->alt,
                              &vel->x,
                              &vel->y,
                              &vel->z,
                              &quat->qi,
                              &quat->qx,
                              &quat->qy,
                              &quat->qz,
                              &rates->p,
                              &rates->q,
                              &rates->r,
                              &target.offset.x,
                              &target.offset.y,
                              &target.offset.z);
#else
  pprz_msg_send_TARGET_POS_INFO(trans, dev, AC_ID,
                              &target.pos.tow,
                              &target.pos.recv_time,
                              &target.pos.lla.lat,
                              &target.pos.lla.lon,
                              &target.pos.lla.alt,
                              &target.pos.vel.x,
                              &target.pos.vel.y,
                              &target.pos.vel.z,
                              &target.pos.quat.qi,
                              &target.pos.quat.qx,
                              &target.pos.quat.qy,
                              &target.pos.quat.qz,
                              &target.pos.rates.p,
                              &target.pos.rates.q,
                              &target.pos.rates.r,
                              &target.offset.x,
                              &target.offset.y,
                              &target.offset.z);
#endif
}

static void send_falcon_sensor(struct transport_tx *trans, struct link_device *dev)
{
  float p_out[3] = {falcon.p_out.x, falcon.p_out.y, falcon.p_out.z};
  float p_in[3] = {falcon.p_in.x, falcon.p_in.y, falcon.p_in.z};
  float q[4] = {falcon.q.qi, falcon.q.qx, falcon.q.qy, falcon.q.qz};
  float p_var[3] = {falcon.p_var.x, falcon.p_var.y, falcon.p_var.z};
  float q_var[3] = {falcon.q_var.x, falcon.q_var.y, falcon.q_var.z};
  float angles[2] = {falcon.angles.phi, falcon.angles.psi};
  float distance = falcon.distance;
  
  pprz_msg_send_FALCON_SENSOR(trans, dev, AC_ID,
                              &falcon.valid,
                              &falcon.mode,
                              &falcon.beacon_id, 
                              p_out,
                              p_in, 
                              q, 
                              p_var, 
                              q_var,
                              &falcon.intensity, 
                              &falcon.width, 
                              angles,
                              &distance);
}
#endif

void target_pos_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TARGET_POS_INFO, send_target_pos_info);
#if !TARGET_POS_GROUND_STATION
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FALCON_SENSOR, send_falcon_sensor);
#endif
#endif

  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
  AbiBindMsgRELPOS(ABI_BROADCAST, &relpos_ev, relpos_cb);
  AbiBindMsgAGL(ABI_BROADCAST, &lidar_ev, lidar_cb);

  /* Initialize the linear Kalman filter */
  target_pos_kalman_init(&target_pos_kalman, P0, Q0, 1/TARGET_POS_PERIODIC_FREQ);
  float_rmat_of_eulers_321(&falcon.body_to_sensor_rotation, &(struct FloatEulers) {RadOfDeg(FALCON_X_ANGLE), RadOfDeg(FALCON_Y_ANGLE), RadOfDeg(FALCON_Z_ANGLE)});
}

/* Get the GPS lla position */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  target.gps_lla.lat = gps_s->lla_pos.lat;
  target.gps_lla.lon = gps_s->lla_pos.lon;
  target.gps_lla.alt = gps_s->lla_pos.alt;
}

/* Update the local relative position information */
static void relpos_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct RelPosNED *relpos)
{
  // Verify if we received a valid heading
  if(
#ifdef TARGET_POS_RELHEADING_REF_ID
    relpos->reference_id != TARGET_POS_RELHEADING_REF_ID ||
#endif
    !isfinite(relpos->heading)
  ) {
    return;
  }

  // Compensate for offset gps antenna to c.g.
  struct FloatVect3 relpos_cg_compensated = {0};
#if defined(INS_EKF2_GPS_POS_X) && defined(INS_EKF2_GPS_POS_Y) && defined(INS_EKF2_GPS_POS_Z)
  struct FloatVect3 gps_to_cg_offset = {INS_EKF2_GPS_POS_X, INS_EKF2_GPS_POS_Y, INS_EKF2_GPS_POS_Z};
  struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
  struct FloatVect3 relpos_pos = {(float)relpos->pos.x, (float)relpos->pos.y, (float)relpos->pos.z};
  struct FloatVect3 relpos_cg_compensation;
  float_rmat_transp_vmult(&relpos_cg_compensation, ned_to_body, &gps_to_cg_offset);
  VECT3_DIFF(relpos_cg_compensated, relpos_pos, relpos_cg_compensation);
#endif

#if FALCON_LOG_ON_ARRIVAL
  pprz_msg_send_GPS_RELPOS(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &relpos->reference_id,
                              &relpos->tow,               
                              &relpos->pos.x,
                              &relpos->pos.y,
                              &relpos->pos.z,
                              &relpos_cg_compensated.x,
                              &relpos_cg_compensated.y,
                              &relpos_cg_compensated.z,    
                              &relpos->distance,          
                              &relpos->heading,           
                              &relpos->pos_acc.x,
                              &relpos->pos_acc.y,
                              &relpos->pos_acc.z,  
                              &relpos->distance_acc,        
                              &relpos->heading_acc);          
#endif
}

/**
 * Receive a TARGET_POS message from the ground
 */
void target_parse_target_pos(uint8_t *buf)
{
  if(DL_TARGET_POS_ac_id(buf) != AC_ID)
    return;

  // Save the received values
  target.pos.recv_time = get_sys_time_tow();
  target.pos.tow = DL_TARGET_POS_tow(buf);
  target.pos.lla.lat = DL_TARGET_POS_lat(buf);
  target.pos.lla.lon = DL_TARGET_POS_lon(buf);
  target.pos.lla.alt = DL_TARGET_POS_alt(buf);
  target.pos.vel.x = DL_TARGET_POS_vnorth(buf);
  target.pos.vel.y = DL_TARGET_POS_veast(buf);
  target.pos.vel.z = DL_TARGET_POS_vdown(buf);
  target.pos.quat.qi = DL_TARGET_POS_body_qi(buf);
  target.pos.quat.qx = DL_TARGET_POS_body_qx(buf);
  target.pos.quat.qy = DL_TARGET_POS_body_qy(buf);
  target.pos.quat.qz = DL_TARGET_POS_body_qz(buf);
  target.pos.rates.p = DL_TARGET_POS_p(buf);
  target.pos.rates.q = DL_TARGET_POS_q(buf);
  target.pos.rates.r = DL_TARGET_POS_r(buf);
  
  struct NedCoor_i target_pos_cm;
  struct FloatVect3 pos;
  ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target.pos.lla);
  pos.x = target_pos_cm.x / 100.;
  pos.y = target_pos_cm.y / 100.;
  pos.z = target_pos_cm.z / 100.;

  // Get the target position relative to the drone
  VECT3_ADD(pos, target.offset);
  struct NedCoor_f *uav_pos = stateGetPositionNed_f();
  VECT3_SUB(pos, *uav_pos);

  struct FloatVect3 vel = {target.pos.vel.x, target.pos.vel.y, target.pos.vel.z};
  struct NedCoor_f *uav_speed = stateGetSpeedNed_f();
  VECT3_SUB(vel, *uav_speed);

#if TARGET_POS_KALMAN_USE_GROUND_STATION
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

#ifdef FALCON_LOG_ON_ARRIVAL
  pprz_msg_send_TARGET_POS_INFO(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &target.pos.tow,
                              &target.pos.recv_time,
                              &target.pos.lla.lat,
                              &target.pos.lla.lon,
                              &target.pos.lla.alt,
                              &target.pos.vel.x,
                              &target.pos.vel.y,
                              &target.pos.vel.z,
                              &target.pos.quat.qi,
                              &target.pos.quat.qx,
                              &target.pos.quat.qy,
                              &target.pos.quat.qz,
                              &target.pos.rates.p,
                              &target.pos.rates.q,
                              &target.pos.rates.r,
                              &target.offset.x,
                              &target.offset.y,
                              &target.offset.z);
#endif
}

/* Update the lidar measurement */
static void lidar_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), float distance)
{
#if TARGET_POS_KALMAN_USE_LIDAR
  static struct KalmanSensor lidar_kalman = {
    .noise = {999.f, 999.f, 999.f, 999.f, 0.1f, 999.f}, // not determined yet!
    .meas = {0, 0, 0, 0, 0, 0},
    .Hmat = {{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
             {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
             {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
             {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
             {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, 
             {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}
  };

  // Check if lidar AGL is in valid range
  if (distance < 0.1f || distance > 5.0f) {
    return;
  }

  lidar_kalman.meas[4] = distance;
  target_pos_kalman_update(&target_pos_kalman, &lidar_kalman);
#endif
}

/**
 * Parse a Falcon sixdof message
 */
#include "generated/flight_plan.h"
#if TARGET_POS_GROUND_STATION
void target_pos_parse_falcon_sixdof(uint8_t *buf) {} // required for dummy flightplan
#else
void target_pos_parse_falcon_sixdof(uint8_t *buf)
{
  float *pos = pprzlink_get_DL_IMCU_FALCON_SIXDOF_pos(buf);
  float *quat = pprzlink_get_DL_IMCU_FALCON_SIXDOF_quat(buf);
  float *pos_var = pprzlink_get_DL_IMCU_FALCON_SIXDOF_pos_var(buf);
  float *quat_var = pprzlink_get_DL_IMCU_FALCON_SIXDOF_quat_var(buf);
  struct FloatQuat q = {quat[0], quat[1], quat[2], quat[3]}; // Rotation of the platform relative to the sensor
  struct FloatVect3 p = {pos[0], pos[1], pos[2]}; // Position of the drone relative to the platform
  struct FloatVect3 p_var = {pos_var[0], pos_var[1], pos_var[2]}; // Variance of p
  struct FloatVect3 q_var = {quat_var[0], quat_var[1], quat_var[2]}; // Variance of q
  struct FloatVect3 p_rot, p_inv, p_out;
  struct FloatQuat body_to_ned;

  // Calculate the position of the platform relative the the UAV
  float_quat_vmult(&p_rot, &q, &p); // Rotate the position around the platform
  // Invert position and change from Y-Down to Z-Down reference frame
  p_inv.x = -p_rot.x; // TODO: Add falcon position offset configuration
  p_inv.y = p_rot.z;
  p_inv.z = -p_rot.y + 0.13;

  float_quat_invert(&body_to_ned, stateGetNedToBodyQuat_f());
  float_quat_vmult(&p_out, &body_to_ned, &p_inv); // Rotate the position to earth frame NED

  // Update a position in the flight plan for now
  uint8_t wp_id = WP_FOLLOW;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, p_out);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);

  falcon.p_in = p;
  falcon.p_out = p_out;
  falcon.q = q;
  falcon.p_var = p_var;
  falcon.q_var = q_var;

#if (TARGET_POS_KALMAN_USE_FALCON)
  falcon.kalman.meas[0] = p_out.x;
  falcon.kalman.meas[2] = p_out.y;
  falcon.kalman.meas[4] = p_out.z;
  target_pos_kalman_update(&target_pos_kalman, &falcon.kalman);
#endif
  
#if FALCON_LOG_ON_ARRIVAL
  float p_out_arr[3] = {falcon.p_out.x, falcon.p_out.y, falcon.p_out.z};
  float p_in_arr[3] = {falcon.p_in.x, falcon.p_in.y, falcon.p_in.z};
  float zero_f = 0.f;
  uint16_t zero_i = 0;
  float zeros_2[2] = {0.f, 0.f};

  pprz_msg_send_FALCON_SENSOR(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &falcon.valid,
                              &falcon.mode,
                              &zero_i,      // Beacon id (unused in SIXDOF tracking mode)
                              p_out_arr,
                              p_in_arr,
                              quat,
                              pos_var,
                              quat_var,
                              &zero_f,      // Beam intensity (unused in SIXDOF tracking mode)
                              &zero_f,      // Beam width (unused in SIXDOF tracking mode)
                              zeros_2,      // Relative angles (unused in SIXDOF tracking mode)
                              &zero_f);     // Distance in relangle mode (unused in SIXDOF tracking mode)
#endif

  // Send waypoint update every half second
  RunOnceEvery(200 / 2, {
    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
  });
}
#endif

/**
 * Parse a Falcon relative angle message
 */
#if TARGET_POS_GROUND_STATION
void target_pos_parse_falcon_relangle(uint8_t *buf) {} // required for dummy flightplan
#else
void target_pos_parse_falcon_relangle(uint8_t *buf) 
{
  falcon.beacon_id = pprzlink_get_DL_IMCU_FALCON_RELANGLE_id(buf);
  falcon.intensity = pprzlink_get_DL_IMCU_FALCON_RELANGLE_intensity(buf);
  falcon.width = pprzlink_get_DL_IMCU_FALCON_RELANGLE_width(buf);

  float *rel_angles = pprzlink_get_DL_IMCU_FALCON_RELANGLE_angles(buf);
  struct FloatEulers angles = {rel_angles[1], 0.f, rel_angles[0]};
  falcon.angles = angles;
  
  /* Implement logic to go from distance and x/z angles to a relative position */
  // Temp relation for distance and intensity, depends on environment and beacon
  // falcon.distance = 5.4165 + 75.5979 / falcon.intensity - 80.3343 / (falcon.intensity*falcon.intensity);
  
  // Obtain the relative position in sensor frame
  struct FloatVect3 p_out_sensor;
  struct FloatVect3 p_out_body;
  struct FloatVect3 p_out_ned;
  struct FloatRMat rel_angles_sensor;
  float_rmat_of_eulers_321(&rel_angles_sensor, &falcon.angles);
  float_rmat_transp_vmult(&p_out_sensor, &rel_angles_sensor, &(struct FloatVect3){0, falcon.distance, 0});

  // Rotate the relative position to the body frame and add the sensor to c.g. translation
  float_rmat_transp_vmult(&p_out_body, &falcon.body_to_sensor_rotation, &p_out_sensor);
  VECT3_ADD(p_out_body, falcon.body_to_sensor_translation);

  // Rotate the relative position to the NED frame
  struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
  float_rmat_transp_vmult(&p_out_ned, ned_to_body, &p_out_body);
  
#if TARGET_POS_KALMAN_USE_FALCON
  falcon.kalman.meas[0] = p_out_ned.x;
  falcon.kalman.meas[2] = p_out_ned.y;
  falcon.kalman.meas[4] = p_out_ned.z;
  target_pos_kalman_update(&target_pos_kalman, &falcon.kalman);
#endif

#if FALCON_LOG_ON_ARRIVAL
  float p_out[3] = {p_out_ned.x, p_out_ned.y, p_out_ned.z};
  float zeros_3[3] = {0, 0, 0};
  float zeros_4[4] = {0, 0, 0, 0};

  pprz_msg_send_FALCON_SENSOR(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &falcon.valid,
                              &falcon.mode,
                              &falcon.beacon_id,
                              p_out, // Relative position out (unused in relative angle mode)
                              zeros_3, // Relative position in (unused in relative angle mode)
                              zeros_4, // Quaternion rotation (unused in relative angle mode)
                              zeros_3, // Position variance (unused in relative angle mode)
                              zeros_3, // Quaternion variance (unused in relative angle mode)
                              &falcon.intensity,
                              &falcon.width,
                              rel_angles,
                              &falcon.distance);
#endif

  uint8_t wp_id = WP_RELANGLE;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, p_out_ned);
  VECT3_ADD(target_enu, *uav_pos);
  // target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);

  // Send waypoint update every half second
  RunOnceEvery(200 / 2, {
    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
  });
}
#endif

/**
 * Send a falcon cmd message to the sensor
 */
#if USE_NPS || TARGET_POS_GROUND_STATION || !defined(EXTRA_DOWNLINK_DEVICE)
void target_pos_send_falcon_cmd(float mode) {
  falcon.mode = mode;
}
#else
#include "modules/datalink/extra_pprz_dl.h"
void target_pos_send_falcon_cmd(float mode) 
{
  falcon.mode = mode;
  pprz_msg_send_IMCU_FALCON_CMD(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, &falcon.mode);
}
#endif

/**
 * Get the current target position (NED) and heading
 */
bool target_get_pos(struct NedCoor_f *pos __attribute__((unused)), float *heading __attribute__((unused))) {
  return false;
}

/**
 * Get the current target velocity (NED)
 */
bool target_get_vel(struct NedCoor_f *vel __attribute__((unused))) {
  return false;
}

/**
 * Set the current measured distances as offset (not constant in NED! needs fix)
 */
bool target_pos_set_current_offset(__attribute__((unused)) bool set_offset) {
  if(target.pos.valid && state.ned_initialized_i) { // && (get_sys_time_tow() - target.pos.tow) < TARGET_RTK_TIMEOUT) // not working atm
    struct NedCoor_i target_pos_cm;

    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target.pos.lla);

    target.offset.x -= (float)target_pos_cm.x * 0.01;
    target.offset.y -= (float)target_pos_cm.y * 0.01;
    target.offset.z -= (float)target_pos_cm.z * 0.01;

    target.autoset_target_offset = true;
    return true;
  }

#if USE_NPS
  struct FloatVect3 pos;
  struct FloatVect3 vel;
  target_pos_kalman_get_state(&target_pos_kalman, &pos, &vel);
  target.offset.x -= pos.x;
  target.offset.y -= pos.y;
  target.offset.z -= pos.z;
  target.autoset_target_offset = true;
#endif
  return false;
}

void target_pos_kalman_filter_init(float r __attribute__((unused))) {
  target_pos_kalman_init(&target_pos_kalman, P0, Q0, 1/TARGET_POS_PERIODIC_FREQ);
}

void target_pos_periodic(void) {
#if !TARTGET_POS_GROUND_STATION
#if USE_NPS
  // Fake some target pos data
  
  // Lidar every 10 Hz
  RunOnceEvery(5, {
    lidar_cb(0, 0, stateGetPositionNed_f()->z);
  });

  // Falcon every 50 Hz
  static struct KalmanSensor falcon_kalman = {
    .noise = {0.1, 999.f, 0.1, 999.f, 0.1, 999.f}, // not determined yet!
    .meas = {0, 0, 0, 0, 0, 0},
    .Hmat = {{1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
             {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
             {0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
             {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 
             {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, 
             {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}
  };
  
  falcon_kalman.meas[0] = stateGetPositionNed_f()->x + target.offset.x;
  falcon_kalman.meas[2] = stateGetPositionNed_f()->y + target.offset.y;
  falcon_kalman.meas[4] = stateGetPositionNed_f()->z + target.offset.z;

  RunOnceEvery(1, {
    target_pos_kalman_update(&target_pos_kalman, &falcon_kalman);
  });

  // GPS every 1 Hz
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
  
  ground_station_kalman.meas[0] = stateGetPositionNed_f()->x + target.offset.x;
  ground_station_kalman.meas[2] = stateGetPositionNed_f()->y + target.offset.y;
  ground_station_kalman.meas[4] = stateGetPositionNed_f()->z + target.offset.z;
  ground_station_kalman.meas[1] = stateGetSpeedNed_f()->x;
  ground_station_kalman.meas[3] = stateGetSpeedNed_f()->y;
  ground_station_kalman.meas[5] = stateGetSpeedNed_f()->z;
    
  RunOnceEvery(50, {
    target_pos_kalman_update(&target_pos_kalman, &ground_station_kalman);
  });
#endif

  target_pos_kalman_predict(&target_pos_kalman);

  // Get Kalman state
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  target_pos_kalman_get_state(&target_pos_kalman, &pos, &speed);

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
#else
  return;
#endif

}
