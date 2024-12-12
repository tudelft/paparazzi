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
#include "filters/simple_kinematic_kalman.h"

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

#ifndef TARGET_OFFSET_X
#define TARGET_OFFSET_X 0.0
#endif

#ifndef TARGET_OFFSET_Y
#define TARGET_OFFSET_Y 0.0
#endif

#ifndef TARGET_OFFSET_Z
#define TARGET_OFFSET_Z 0.0
#endif

// Still need to find suitable initial values
#ifndef TARGET_POS_KALMAN_P0_POS
#define TARGET_POS_KALMAN_P0_POS 1.0
#endif

#ifndef TARGET_POS_KALMAN_P0_SPEED
#define TARGET_POS_KALMAN_P0_SPEED 1.0
#endif

#ifndef TARGET_POS_KALMAN_Q_SIGMA2
#define TARGET_POS_KALMAN_Q_SIGMA2 1.0
#endif

#ifndef TARGET_POS_KALMAN_R
#define TARGET_POS_KALMAN_R 1.0
#endif

/* Body to sensor angle offsets, usually 0 except for Y angle */
#ifndef FALCON_X_ANGLE 
#define FALCON_X_ANGLE 0
#endif

#ifndef FALCON_Y_ANGLE
#define FALCON_Y_ANGLE 0
#endif

#ifndef FALCON_Z_ANGLE
#define FALCON_Z_ANGLE 0
#endif

#ifndef TARGET_POS_RELHEADING_REF_ID
#define TARGET_POS_RELHEADING_REF_ID 0 
#endif

/* Initialize the main structure */
struct target_t target = {
  .pos = {0},
  .offset = {
    .x = TARGET_OFFSET_X,
    .y = TARGET_OFFSET_Y,
    .z = TARGET_OFFSET_Z,
  },
  .target_pos_timeout = TARGET_POS_TIMEOUT,
  .rtk_timeout = TARGET_RTK_TIMEOUT,
};

/* Initialize falcon sensor structure */
struct falcon_sensor_t falcon = {
  .valid = false,     // Assume invalid data on start up
  .manual = false,    // By default the tracking mode should be determined automatically
  .mode = 0,          // Initialize falcon sensor tracking mode to off
  .beacon_id = 0,
  .body_offset = {
    .phi = FALCON_X_ANGLE,
    .theta = FALCON_Y_ANGLE,
    .psi = FALCON_Z_ANGLE,
  },
  .p_out = {0},
  .p_in = {0},
  .q = {0},
  .p_var = {0},
  .q_var = {0},
  .angles = {0},
  .intensity = 0,
  .width = 0
};

/* Initialize the linear kalman filter struct */
struct SimpleKinematicKalman target_pos_kalman;
struct FloatRMat body_to_falcon_sensor;

/* GPS abi callback */
static abi_event gps_ev;
static abi_event relpos_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
static void relpos_cb(uint8_t sender_id, uint32_t stamp, struct RelPosNED *relpos);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_target_pos_info(struct transport_tx *trans, struct link_device *dev)
{
#if TARGET_POS_GROUND_STATION
  // Send the current state of the ground station
  struct LlaCoor_i *pos = stateGetPositionLla_i();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  struct FloatRates *rates = stateGetBodyRates_f();
  uint32_t tow = get_sys_time_tow();

  DOWNLINK_SEND_TARGET_POS_INFO(DefaultChannel, DefaultDevice,
                              &tow,
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
  float distance = 0.f;
  
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

  /* Initialize the linear Kalman filter */
  target_pos_kalman_filter_init(TARGET_POS_KALMAN_R);

  float_rmat_of_eulers_321(&body_to_falcon_sensor, &falcon.body_offset);

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

  // Calculate old properties for now to keep code compatible
  target.pos.course = atan2f(target.pos.vel.y, target.pos.vel.x);
  target.pos.ground_speed = sqrtf(target.pos.vel.x * target.pos.vel.x + target.pos.vel.y * target.pos.vel.y);
  target.pos.climb = -target.pos.vel.z;
  target.pos.valid = true;

  // To test time between messages
  target.pos.rates.p = (float)target.pos.recv_time - (float)target.pos.tow;
  
  struct NedCoor_i target_pos_cm;
  struct FloatVect3 pos;
  ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target.pos.lla);
  pos.x = target_pos_cm.x / 100.;
  pos.y = target_pos_cm.y / 100.;
  pos.z = target_pos_cm.z / 100.;

  // Add the target offset manually for now
  pos.x += target.offset.x;
  pos.y += target.offset.y;
  pos.z += target.offset.z;

  simple_kinematic_kalman_update_pos(&target_pos_kalman, pos);
  simple_kinematic_kalman_update_speed(&target_pos_kalman, target.pos.vel, SIMPLE_KINEMATIC_KALMAN_SPEED_3D);

#ifdef FALCON_LOG_ON_ARRIVAL
  pprz_msg_send_TARGET_POS_INFO(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &target.pos.tow,
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

  simple_kinematic_kalman_update_pos(&target_pos_kalman, p_out);
  
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
  struct FloatEulers angles = {rel_angles[0], 0.f, rel_angles[1]};
  falcon.angles = angles;
  
  /* Implement logic to go from distance and x/z angles to a relative position */
  // Temp relation for distance and intensity, depends on environment
  float distance = 5.4165 + 75.5979 / falcon.intensity - 80.3343 / (falcon.intensity*falcon.intensity);

  // Adjust for rotation between sensor and NED
  struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
  struct FloatRMat ned_to_falcon_sensor;
  float_rmat_comp(&ned_to_falcon_sensor, ned_to_body, &body_to_falcon_sensor);

  struct FloatEulers angles_ned;
  float_rmat_transp_mult(&angles_ned, &ned_to_falcon_sensor, &angles);

  struct FloatRMat falcon_rmat;
  float_rmat_of_eulers_321(&falcon_rmat, &angles_ned);
  
  // Obtain the relative position in sensor frame
  struct FloatVect3 p_out;
  float_rmat_vmult(&p_out, &falcon_rmat, &(struct FloatVect3){0, 0, distance});
  
  simple_kinematic_kalman_update_pos(&target_pos_kalman, p_out);

#if FALCON_LOG_ON_ARRIVAL
  float zeros_3[3] = {0, 0, 0};
  float zeros_4[4] = {0, 0, 0, 0};

  pprz_msg_send_FALCON_SENSOR(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &falcon.valid,
                              &falcon.mode,
                              &falcon.beacon_id,
                              zeros_3, // Relative position out (unused in relative angle mode)
                              zeros_3, // Relative position in (unused in relative angle mode)
                              zeros_4, // Quaternion rotation (unused in relative angle mode)
                              zeros_3, // Position variance (unused in relative angle mode)
                              zeros_3, // Quaternion variance (unused in relative angle mode)
                              &falcon.intensity,
                              &falcon.width,
                              rel_angles,
                              &distance);
#endif

  uint8_t wp_id = WP_RELANGLE;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, p_out);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
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
#if USE_NPS || TARGET_POS_GROUND_STATION
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
bool target_pos_set_current_offset(float unk __attribute__((unused))) {
    if(target.pos.valid && state.ned_initialized_i) { // && (get_sys_time_tow() - target.pos.tow) < TARGET_RTK_TIMEOUT) // not working atm
    struct NedCoor_i target_pos_cm;

    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target.pos.lla);

    // Convert to floating point (cm to meters)
    struct NedCoor_f pos;
    pos.x = target_pos_cm.x * 0.01;
    pos.y = target_pos_cm.y * 0.01;
    pos.z = target_pos_cm.z * 0.01;

    target.offset.x -= pos.x;
    target.offset.y -= pos.y;
    target.offset.z -= pos.z;
  }

  return false;
}

void target_pos_kalman_filter_init(float r __attribute__((unused))) {
  simple_kinematic_kalman_init(&target_pos_kalman, TARGET_POS_KALMAN_P0_POS, TARGET_POS_KALMAN_P0_SPEED, 
                                TARGET_POS_KALMAN_Q_SIGMA2, TARGET_POS_KALMAN_R, 1/TARGET_POS_PERIODIC_FREQ);
}

void target_pos_periodic(void) {
#if !TARTGET_POS_GROUND_STATION && !USE_NPS
  simple_kinematic_kalman_predict(&target_pos_kalman);

  // Get Kalman state
  struct FloatVect3 pos;
  struct FloatVect3 speed;
  simple_kinematic_kalman_get_state(&target_pos_kalman, &pos, &speed);

  pprz_msg_send_TARGET_POS_KALMAN(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                                  &pos.x, &pos.y, &pos.z,
                                  &speed.x, &speed.y, &speed.z);
  RunOnceEvery(100, {
  DOWNLINK_SEND_TARGET_POS_KALMAN(DefaultChannel, DefaultDevice,
                                  &pos.x, &pos.y, &pos.z,
                                  &speed.x, &speed.y, &speed.z);
  });
#else
  return;
#endif

}
