/*
 * Copyright (C) OpenUAS
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/digital_cam/home_vector_ctrl.c
 *  Visual-homing CNN home-vector following through CATIA over the UART link to MORA.
 */

#include "home_vector_ctrl.h"

#include "modules/digital_cam/uart_cam_ctrl.h"
#include "modules/digital_cam/catia/protocol.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include <math.h>

#if FIXEDWING_FIRMWARE
#include "modules/nav/common_nav.h"
#include "firmwares/fixedwing/nav.h"
#else
#include "modules/nav/waypoints.h"
#endif

#ifndef HOME_VECTOR_PERIOD_S
#define HOME_VECTOR_PERIOD_S 0.5f
#endif
#define HOME_VECTOR_MIN_PERIOD_S 0.1f

#ifndef HOME_VECTOR_LEG_DISTANCE_M
#define HOME_VECTOR_LEG_DISTANCE_M 50.f
#endif

/** DC_SHOT telemetry for home-vector shots at most this often (0 = every sample); the
 *  UART reply carrying the actual prediction is unaffected and always arrives per shot. */
#ifndef HOME_VECTOR_REPORT_PERIOD_S
#define HOME_VECTOR_REPORT_PERIOD_S 1.0f
#endif

bool home_vector_result_valid = false;
bool home_vector_result_fresh = false;
float home_vector_dx_body = 0.f;
float home_vector_dy_body = 0.f;
float home_vector_dist_m = 0.f;

float home_vector_period_s = HOME_VECTOR_PERIOD_S;
float home_vector_leg_distance_m = HOME_VECTOR_LEG_DISTANCE_M;

uint8_t home_vector_wp_id;

static bool sampling_active = false;
static float last_shot_time = 0.f;
static float last_report_time = 0.f;

static void set_wp_xy(uint8_t wp_id, float x, float y);

static bool home_vector_rx_handler(const struct catia_transport *frame)
{
  if (frame->msg_id != CATIA_HOME_VECTOR_RESULT
      || frame->payload_len != CATIA_HOME_VECTOR_RESULT_MSG_SIZE) {
    return false;
  }
  union catia_home_vector_result_union result;
  for (int i = 0; i < CATIA_HOME_VECTOR_RESULT_MSG_SIZE; i++) {
    result.bin[i] = frame->payload[i];
  }
  if (result.data.status != CATIA_HOME_VECTOR_VALID) {
    // Keep the last good prediction; only the awaited reply is missing this cycle.
    home_vector_result_fresh = false;
    return true;
  }
  home_vector_dx_body = result.data.dx_scaled / 10000.f;
  home_vector_dy_body = result.data.dy_scaled / 10000.f;
  home_vector_dist_m = result.data.dist_mm / 1000.f;
  home_vector_result_valid = true;
  home_vector_result_fresh = true;
  return true;
}

void home_vector_init(void)
{
  digital_cam_uart_set_rx_handler(home_vector_rx_handler);
}

/** Rotate the body-frame (forward, right) prediction into world ENU (east, north) using
 * the aircraft's current heading (psi: 0 = North, positive clockwise, standard NED yaw).
 * Derived fresh for Paparazzi's ENU convention -- NOT a port of the Python research
 * pipeline's body_to_world (that one targets a north-up-GeoTIFF-pixel "world frame",
 * x=East/y=South, a different axis convention). Verify against a known heading and a
 * known real-world home direction on the bench before trusting this for a real flight
 * (see the deployment plan's Part 3 verification note). */
static void body_to_world_enu(float dx_body, float dy_body, float heading_rad,
                              float *east, float *north)
{
  float s = sinf(heading_rad);
  float c = cosf(heading_rad);
  *east  = dx_body * s + dy_body * c;
  *north = dx_body * c - dy_body * s;
}

void home_vector_periodic(void)
{
  float now = get_sys_time_float();
  if (!sampling_active) {
    return;
  }
  if (now - last_shot_time >= home_vector_period_s) {
    last_shot_time = now;
    bool report = (now - last_report_time >= HOME_VECTOR_REPORT_PERIOD_S);
    if (report) {
      last_report_time = now;
    }
    digital_cam_uart_shoot(CATIA_CAMERA_AICAM, report);
  }

  if (!home_vector_result_fresh) {
    return;
  }
  home_vector_result_fresh = false;

  float heading_rad = stateGetNedToBodyEulers_f()->psi;
  float east, north;
  body_to_world_enu(home_vector_dx_body, home_vector_dy_body, heading_rad, &east, &north);
  float norm = sqrtf(east * east + north * north);
  if (norm < 1e-6f) {
    return;
  }
  east /= norm;
  north /= norm;

  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  set_wp_xy(home_vector_wp_id,
            pos->x + east * home_vector_leg_distance_m,
            pos->y + north * home_vector_leg_distance_m);
}

uint8_t home_vector_start(uint8_t wp_id)
{
  home_vector_wp_id = wp_id;
  home_vector_result_clear();
  last_shot_time = 0.f;
  last_report_time = 0.f;
  sampling_active = true;
  return 0;
}

uint8_t home_vector_stop(void)
{
  sampling_active = false;
  return digital_cam_uart_stop(CATIA_CAMERA_AICAM, false);
}

uint8_t home_vector_result_clear(void)
{
  home_vector_result_valid = false;
  home_vector_result_fresh = false;
  home_vector_dx_body = 0.f;
  home_vector_dy_body = 0.f;
  home_vector_dist_m = 0.f;
  return 0;
}

static void set_wp_xy(uint8_t wp_id, float x, float y)
{
#if FIXEDWING_FIRMWARE
  nav_move_waypoint_enu(wp_id, x, y, WaypointAlt(wp_id));
#else
  struct EnuCoor_f enu = { x, y, waypoint_get_alt(wp_id) };
  waypoint_set_enu(wp_id, &enu);
#endif
}
