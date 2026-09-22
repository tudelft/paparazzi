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

/** @file modules/digital_cam/earcam_ctrl.c
 *  EARcam acoustic loud-spot search through CATIA over the UART link to MORA.
 */

#include "home_vector_ctrl.h"
#include "generated/airframe.h"

#include "modules/digital_cam/uart_cam_ctrl.h"
#include "modules/digital_cam/catia/protocol.h"
#include "mcu_periph/sys_time.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"
#include "state.h"
#include <math.h>

#if FIXEDWING_FIRMWARE
#include "modules/nav/common_nav.h"
#include "firmwares/fixedwing/nav.h"
#include "autopilot.h"
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

#ifndef HOME_VECTOR_REPORT_PERIOD_S
#define HOME_VECTOR_REPORT_PERIOD_S 1.0f
#endif

bool home_vector_result_valid = false;
bool home_vector_result_fresh = false;
float home_vector_ux_body = 0.f;
float home_vector_uy_body = 0.f;
float home_vector_dist_m = 0.f;

float home_vector_period_s = HOME_VECTOR_PERIOD_S;
float home_vector_leg_distance_m = HOME_VECTOR_LEG_DISTANCE_M;

uint8_t home_vector_wp_id;

static bool sampling_active = false;
static float last_shot_time = 0.f;
static float last_report_time = 0.f;
static float last_motor_time = 0.f;

static void set_wp_xy(uint8_t wp_id, float x, float y);

static void body_to_world_enu(float ux_body, float uy_body, float heading_rad,
                              float *east, float *north)
{
  float s = sinf(heading_rad);
  float c = cosf(heading_rad);
  *east  = ux_body * s + uy_body * c;
  *north = ux_body * c - uy_body * s;
}

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
  if (result.data.status != CATIA_HOME_VECTOR_RESULT_VALID) {
    // Keep the last good estimate; only the awaited reply is missing.
    home_vector_result_fresh = false;
    return true;
  }
  home_vector_ux_body = result.data.ux_scaled / 1000.f;
  home_vector_uy_body = result.data.uy_scaled / 1000.f;
  home_vector_dist_m = result.data.dist_mm / 1000.f;
  home_vector_result_valid = true;
  home_vector_result_fresh = true;
  return true;
}

void home_vector_init(void)
{
  digital_cam_uart_set_rx_handler(home_vector_rx_handler);
}

// shoots home vector requests at home_vector_period_s
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
    digital_cam_uart_shoot(CATIA_CAMERA_EARCAM, report); // request a home vector 
  }
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
  home_vector_ux_body = 0.f;
  home_vector_uy_body = 0.f;
  home_vector_dist_m = 0.f;
  return 0;
}
