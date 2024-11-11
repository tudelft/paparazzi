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

// The timeout when receiving GPS messages from the ground in ms
#ifndef TARGET_POS_TIMEOUT
#define TARGET_POS_TIMEOUT 5000
#endif

// The timeout when recceiving an RTK gps message from the GPS
#ifndef TARGET_RTK_TIMEOUT
#define TARGET_RTK_TIMEOUT 1000
#endif

#ifndef TARGET_OFFSET_HEADING
#define TARGET_OFFSET_HEADING 180.0
#endif

#ifndef TARGET_OFFSET_DISTANCE
#define TARGET_OFFSET_DISTANCE 12.0
#endif

#ifndef TARGET_OFFSET_HEIGHT
#define TARGET_OFFSET_HEIGHT -1.2
#endif

#ifndef TARGET_INTEGRATE_XY
#define TARGET_INTEGRATE_XY true
#endif

#ifndef TARGET_INTEGRATE_Z
#define TARGET_INTEGRATE_Z false
#endif

/* Initialize the main structure */
struct target_t target = {
  .pos = {0},
  .offset = {
    .heading = TARGET_OFFSET_HEADING,
    .distance = TARGET_OFFSET_DISTANCE,
    .height = TARGET_OFFSET_HEIGHT,
  },
  .target_pos_timeout = TARGET_POS_TIMEOUT,
  .rtk_timeout = TARGET_RTK_TIMEOUT,
  .integrate_xy = TARGET_INTEGRATE_XY,
  .integrate_z = TARGET_INTEGRATE_Z
};

/* Initialize falcon sensor structure */
struct falcon_sensor_t falcon = {
  .manual = false,    // By default the tracking mode should be determined automatically
  .mode = 0,          // Initialize falcon sensor tracking mode to off
  .beacon_id = 0,
  .p_out = {0},
  .q = {0},
  .p_var = {0},
  .q_var = {0},
  .angles = {0},
  .intensity = 0,
  .width = 0
};

/* GPS abi callback */
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_target_pos_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_TARGET_POS_INFO(trans, dev, AC_ID,
                              &target.pos.lla.lat,
                              &target.pos.lla.lon,
                              &target.pos.lla.alt,
                              &target.pos.ground_speed,
                              &target.pos.climb,
                              &target.pos.course,
                              &target.pos.heading,
                              &target.offset.heading,
                              &target.offset.distance,
                              &target.offset.height);
}

static void send_falcon_sensor(struct transport_tx *trans, struct link_device *dev)
{
  float p_out[3] = {falcon.p_out.x, falcon.p_out.y, falcon.p_out.z};
  float q[4] = {falcon.q.qi, falcon.q.qx, falcon.q.qy, falcon.q.qz};
  float p_var[3] = {falcon.p_var.x, falcon.p_var.y, falcon.p_var.z};
  float q_var[3] = {falcon.q_var.x, falcon.q_var.y, falcon.q_var.z};
  float angles[2] = {falcon.angles.x, falcon.angles.y};
  
  pprz_msg_send_FALCON_SENSOR(trans, dev, AC_ID,
                              &falcon.mode,
                              &falcon.beacon_id, 
                              p_out, 
                              q, 
                              p_var, 
                              q_var,
                              &falcon.intensity, 
                              &falcon.width, 
                              angles);
}
#endif

void target_pos_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TARGET_POS_INFO, send_target_pos_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FALCON_SENSOR, send_falcon_sensor);
#endif

  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
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

/**
 * Receive a TARGET_POS message from the ground
 */
void target_parse_target_pos(uint8_t *buf)
{
  if(DL_TARGET_POS_ac_id(buf) != AC_ID)
    return;

  // Save the received values
  target.pos.recv_time = get_sys_time_msec();
  target.pos.tow = gps_tow_from_sys_ticks(sys_time.nb_tick); // FIXME: need to get from the real GPS
  target.pos.lla.lat = DL_TARGET_POS_lat(buf);
  target.pos.lla.lon = DL_TARGET_POS_lon(buf);
  target.pos.lla.alt = DL_TARGET_POS_alt(buf);
  target.pos.ground_speed = DL_TARGET_POS_speed(buf);
  target.pos.climb = DL_TARGET_POS_climb(buf);
  target.pos.course = DL_TARGET_POS_course(buf);
  target.pos.heading = DL_TARGET_POS_heading(buf);
  target.pos.valid = true;

#ifdef LOG_ON_ARRIVAL
  pprz_msg_send_TARGET_POS_INFO(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &target.pos.lla.lat,
                              &target.pos.lla.lon,
                              &target.pos.lla.alt,
                              &target.pos.ground_speed,
                              &target.pos.climb,
                              &target.pos.course,
                              &target.pos.heading,
                              &target.offset.heading,
                              &target.offset.distance,
                              &target.offset.height);
#endif
}

/**
 * Parse a Falcon sixdof message
 */
#include "generated/flight_plan.h"
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

  float_quat_invert(&body_to_ned ,stateGetNedToBodyQuat_f());
  float_quat_vmult(&p_out, &body_to_ned, &p_inv); // Rotate the position to earth frame NED

  // Update a position in the flight plan for now
  uint8_t wp_id = WP_FOLLOW;
  struct EnuCoor_f target_enu;
  struct EnuCoor_f *uav_pos = stateGetPositionEnu_f();
  ENU_OF_TO_NED(target_enu, p_out);
  VECT3_ADD(target_enu, *uav_pos);
  target_enu.z = waypoints[wp_id].enu_f.z;
  waypoint_set_enu(wp_id, &target_enu);

  float p_out_arr[3] = {falcon.p_out.x, falcon.p_out.y, falcon.p_out.z};
  float q_arr[4] = {falcon.q.qi, falcon.q.qx, falcon.q.qy, falcon.q.qz};
  float p_var_arr[3] = {falcon.p_var.x, falcon.p_var.y, falcon.p_var.z};
  float q_var_arr[3] = {falcon.q_var.x, falcon.q_var.y, falcon.q_var.z};
  
  pprz_msg_send_IMCU_FALCON_SIXDOF(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              p_out_arr,
                              q_arr,
                              p_var_arr,
                              q_var_arr);

  // Send waypoint update every half second
  RunOnceEvery(200 / 2, {
    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
  });

  falcon.p_out = p_out;
  falcon.q = q;
  falcon.p_var = p_var;
  falcon.q_var = q_var;
}

/**
 * Parse a Falcon relative angle message
 */
void target_pos_parse_falcon_relangle(uint8_t *buf) 
{
  falcon.beacon_id = pprzlink_get_DL_IMCU_FALCON_RELANGLE_id(buf);
  falcon.intensity = pprzlink_get_DL_IMCU_FALCON_RELANGLE_intensity(buf);
  falcon.width = pprzlink_get_DL_IMCU_FALCON_RELANGLE_width(buf);

  float *rel_angles = pprzlink_get_DL_IMCU_FALCON_RELANGLE_angles(buf);
  struct FloatVect2 angles = {rel_angles[0], rel_angles[1]};
  falcon.angles = angles;

  pprz_msg_send_IMCU_FALCON_RELANGLE(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &falcon.beacon_id,
                              rel_angles,
                              &falcon.intensity,
                              &falcon.width);
  
  /* TODO: Implement some logic */

}

/**
 * Send a falcon cmd message to the sensor
 */
#include "modules/datalink/extra_pprz_dl.h"
void target_pos_send_falcon_cmd(float unk __attribute__((unused))) 
{
  pprz_msg_send_IMCU_FALCON_CMD(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, &falcon.mode);
}

/**
 * Get the current target position (NED) and heading
 */
bool target_get_pos(struct NedCoor_f *pos, float *heading) {
  float time_diff = 0;

  /* When we have a valid target_pos message, state ned is initialized and no timeout */
  if(target.pos.valid && state.ned_initialized_i && (target.pos.recv_time+target.target_pos_timeout) > get_sys_time_msec()) {
    struct NedCoor_i target_pos_cm, drone_pos_cm;

    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target.pos.lla);
    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&drone_pos_cm, &state.ned_origin_i, &target.gps_lla);

    // Convert to floating point (cm to meters)
    pos->x = (target_pos_cm.x - drone_pos_cm.x) * 0.01;
    pos->y = (target_pos_cm.y - drone_pos_cm.y) * 0.01;
    pos->z = (target_pos_cm.z - drone_pos_cm.z) * 0.01;

    // In seconds, overflow uint32_t in 49,7 days
    time_diff = (get_sys_time_msec() - target.pos.recv_time) * 0.001; // FIXME: should be based on TOW of ground gps

    // Return the heading
    *heading = target.pos.heading;

    // If we have a velocity measurement try to integrate the x-y position when enabled
    struct NedCoor_f vel = {0};
    bool got_vel = target_get_vel(&vel);
    if(target.integrate_xy && got_vel) {
      pos->x = pos->x + vel.x * time_diff;
      pos->y = pos->y + vel.y * time_diff;
    }

    if(target.integrate_z && got_vel) {
      pos->z = pos->z + vel.z * time_diff;
    }

    // Offset the target
    pos->x += target.offset.distance * cosf((*heading + target.offset.heading)/180.*M_PI);
    pos->y += target.offset.distance * sinf((*heading + target.offset.heading)/180.*M_PI);
    pos->z -= target.offset.height;

    return true;
  }

  return false;
}

/**
 * Get the current target velocity (NED)
 */
bool target_get_vel(struct NedCoor_f *vel) {

  /* When we have a valid target_pos message, state ned is initialized and no timeout */
  if(target.pos.valid && state.ned_initialized_i && (target.pos.recv_time+target.target_pos_timeout) > get_sys_time_msec()) {
    // Calculate based on ground speed and course
    vel->x = target.pos.ground_speed * cosf(target.pos.course/180.*M_PI);
    vel->y = target.pos.ground_speed * sinf(target.pos.course/180.*M_PI);
    vel->z = -target.pos.climb;

    return true;
  }

  return false;
}

/**
 * Set the current measured distance and heading as offset
 */
bool target_pos_set_current_offset(float unk __attribute__((unused))) {
  if(target.pos.valid && state.ned_initialized_i && (target.pos.recv_time+target.target_pos_timeout) > get_sys_time_msec()) {
    struct NedCoor_i target_pos_cm;
    struct NedCoor_f uav_pos = *stateGetPositionNed_f();

    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target.pos.lla);

    // Convert to floating point (cm to meters)
    struct NedCoor_f pos;
    pos.x = target_pos_cm.x * 0.01;
    pos.y = target_pos_cm.y * 0.01;
    pos.z = target_pos_cm.z * 0.01;

    target.offset.distance = sqrtf(powf(uav_pos.x - pos.x, 2) + powf(uav_pos.y - pos.y, 2));
    target.offset.height = -(uav_pos.z - pos.z);
    target.offset.heading = atan2f((uav_pos.y - pos.y), (uav_pos.x - pos.x))*180.0/M_PI - target.pos.heading;
  }

  return false;
}
