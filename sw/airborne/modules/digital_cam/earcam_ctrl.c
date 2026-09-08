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
 *  EARcam acoustic loud-spot search over the MORA camera link.
 */

#include "earcam_ctrl.h"
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
#ifndef DEFAULT_CIRCLE_RADIUS
#define DEFAULT_CIRCLE_RADIUS 30.f
#endif
#ifndef nav_radius
#define nav_radius DEFAULT_CIRCLE_RADIUS
#endif
#endif

#ifndef EARCAM_USE_AGL_DIST
#define EARCAM_USE_AGL_DIST FALSE
#endif
#if EARCAM_USE_AGL_DIST
#include "modules/sonar/agl_dist.h"
#endif
#ifndef EARCAM_USE_NAV_DROP
#define EARCAM_USE_NAV_DROP FALSE
#endif
#if EARCAM_USE_NAV_DROP
#include "modules/nav/nav_drop.h"
#include "generated/flight_plan.h"
// nav_drop.c only builds its functions when the flight plan has a RELEASE waypoint.
#if defined(WP_RELEASE)
#define EARCAM_HAVE_NAV_DROP 1
#endif
#endif
#ifndef EARCAM_HAVE_NAV_DROP
#define EARCAM_HAVE_NAV_DROP 0
#endif

#ifndef EARCAM_PERIOD_S
#define EARCAM_PERIOD_S 0.1f
#endif
#define EARCAM_MIN_PERIOD_S 0.05f
/** DC_SHOT telemetry for EAR samples at most this often (0 = every sample). */
#ifndef EARCAM_REPORT_PERIOD_S
#define EARCAM_REPORT_PERIOD_S 1.0f
#endif

#ifndef EARCAM_REFINE_HALF_LENGTH_M
#define EARCAM_REFINE_HALF_LENGTH_M 50.f
#endif
#ifndef EARCAM_REFINE_CONVERGE_M
#define EARCAM_REFINE_CONVERGE_M 4.f
#endif
#ifndef EARCAM_REFINE_MAX_ITERATIONS
#define EARCAM_REFINE_MAX_ITERATIONS 3
#endif
#ifndef EARCAM_REFINE_LEGS
#define EARCAM_REFINE_LEGS 4
#endif
#ifndef EARCAM_REFINE_TURN_RADIUS_M
#define EARCAM_REFINE_TURN_RADIUS_M 0.f
#endif
#ifndef EARCAM_REFINE_HEIGHT_FROM_M
#define EARCAM_REFINE_HEIGHT_FROM_M 0.f
#endif
#ifndef EARCAM_REFINE_HEIGHT_TO_M
#define EARCAM_REFINE_HEIGHT_TO_M 0.f
#endif
#ifndef EARCAM_REFINE_HEIGHT_STEP_M
#define EARCAM_REFINE_HEIGHT_STEP_M 0.f
#endif
#ifndef EARCAM_REFINE_HEIGHT_MIN_M
#define EARCAM_REFINE_HEIGHT_MIN_M 4.f
#endif
#ifndef EARCAM_QUIET_ONLY
#define EARCAM_QUIET_ONLY FALSE
#endif
#ifndef EARCAM_QUIET_DELAY_S
#define EARCAM_QUIET_DELAY_S 1.5f
#endif
#ifndef EARCAM_DROP_RELEASE_AGL_M
#define EARCAM_DROP_RELEASE_AGL_M 1.5f
#endif
#ifndef EARCAM_DROP_MAX_AGL_M
#define EARCAM_DROP_MAX_AGL_M 2.0f
#endif
/** Rangefinder AGL trusted up to this height; go around below EARCAM_DROP_ABORT_AGL_M. */
#ifndef EARCAM_DROP_AGL_MAX_M
#define EARCAM_DROP_AGL_MAX_M 4.f
#endif
#ifndef EARCAM_DROP_ABORT_AGL_M
#define EARCAM_DROP_ABORT_AGL_M 0.6f
#endif
// C11 math.h does not define M_PI.
#define EARCAM_PI 3.14159265358979323846f

bool earcam_result_valid = false;
bool earcam_result_fresh = false;
float earcam_lat_deg = 0.f;
float earcam_lon_deg = 0.f;
float earcam_agl_m = 0.f;
float earcam_alt_m = 0.f;
float earcam_level_db = 0.f;
float earcam_confidence = 0.f;
uint16_t earcam_samples = 0;
float earcam_period_s = EARCAM_PERIOD_S;

float earcam_refine_half_length_m = EARCAM_REFINE_HALF_LENGTH_M;
float earcam_refine_converge_m = EARCAM_REFINE_CONVERGE_M;
uint8_t earcam_refine_max_iterations = EARCAM_REFINE_MAX_ITERATIONS;
uint8_t earcam_refine_legs = EARCAM_REFINE_LEGS;
uint8_t earcam_refine_iteration = 0;
float earcam_refine_shift_m = 0.f;
bool earcam_refine_converged = false;
float earcam_refine_turn_radius_m = EARCAM_REFINE_TURN_RADIUS_M;
float earcam_refine_height_from_m = EARCAM_REFINE_HEIGHT_FROM_M;
float earcam_refine_height_to_m = EARCAM_REFINE_HEIGHT_TO_M;
float earcam_refine_height_step_m = EARCAM_REFINE_HEIGHT_STEP_M;
float earcam_refine_height_min_m = EARCAM_REFINE_HEIGHT_MIN_M;

bool earcam_quiet_only = EARCAM_QUIET_ONLY;
float earcam_quiet_delay_s = EARCAM_QUIET_DELAY_S;
bool earcam_quiet_now = false;

float earcam_drop_release_agl_m = EARCAM_DROP_RELEASE_AGL_M;
float earcam_drop_max_agl_m = EARCAM_DROP_MAX_AGL_M;
float earcam_drop_agl_m = 0.f;
bool earcam_drop_released = false;
bool earcam_drop_missed = false;
uint8_t earcam_drop_attempts = 0;

static bool sampling_active = false;
static float last_shot_time = 0.f;
static float last_report_time = 0.f;
static float last_motor_time = 0.f;
#if FIXEDWING_FIRMWARE && EARCAM_USE_AGL_DIST
static float drop_last_agl_time = -1.f;
static float drop_last_agl_value = 0.f;
#endif
static struct LlaCoor_i result_lla;
static struct LlaCoor_i previous_result_lla;
static bool previous_result_valid = false;
static uint8_t refine_leg = 0;
static float refine_center_x = 0.f;
static float refine_center_y = 0.f;
static float refine_turn_radius = 0.f;
static float refine_exit_qdr_deg = 0.f;
static float refine_half_length = 0.f;   // effective half-length, see refine_setup_geometry()
static float refine_circle_radius = 0.f; // turn circle radius matching that half-length

static void result_to_local_xy(const struct LlaCoor_i *lla, float *x, float *y);
static void place_leg_waypoints(uint8_t wp_from, uint8_t wp_to, uint8_t wp_turn);
static void set_wp_xy(uint8_t wp_id, float x, float y);

static bool earcam_rx_handler(const struct mora_transport *frame)
{
  if (frame->msg_id != MORA_EAR_RESULT || frame->payload_len != MORA_EAR_RESULT_MSG_SIZE) {
    return false;
  }
  union mora_ear_result_union result;
  for (int i = 0; i < MORA_EAR_RESULT_MSG_SIZE; i++) {
    result.bin[i] = frame->payload[i];
  }
  earcam_samples = (result.data.sample_count < 0) ? 0 :
                   (result.data.sample_count > 65535 ? 65535 : (uint16_t)result.data.sample_count);
  earcam_level_db = result.data.level_cdb / 100.f;
  earcam_confidence = result.data.confidence / 1000.f;
  if (result.data.status != MORA_EAR_RESULT_VALID
      || result.data.lat < -900000000 || result.data.lat > 900000000
      || result.data.lon < -1800000000 || result.data.lon > 1800000000) {
    // Keep the last good estimate; only the awaited reply is missing.
    earcam_result_fresh = false;
    return true;
  }
  result_lla.lat = result.data.lat;
  result_lla.lon = result.data.lon;
  result_lla.alt = result.data.alt_mm;
  earcam_lat_deg = result.data.lat / 1e7f;
  earcam_lon_deg = result.data.lon / 1e7f;
  earcam_agl_m = result.data.agl_mm / 1000.f;
  earcam_alt_m = result.data.alt_mm / 1000.f;
  earcam_result_valid = true;
  earcam_result_fresh = true;
  return true;
}

void earcam_init(void)
{
  digital_cam_uart_set_rx_handler(earcam_rx_handler);
}

void earcam_periodic(void)
{
  float now = get_sys_time_float();
#if FIXEDWING_FIRMWARE
  if (!autopilot_throttle_killed()) {
    last_motor_time = now;
  }
  earcam_quiet_now = !earcam_quiet_only || (now - last_motor_time >= earcam_quiet_delay_s);
#else
  earcam_quiet_now = true;
#endif
  if (!sampling_active || !earcam_quiet_now) {
    return;
  }
  if (now - last_shot_time >= earcam_period_s) {
    last_shot_time = now;
    bool report = (now - last_report_time >= EARCAM_REPORT_PERIOD_S);
    if (report) {
      last_report_time = now;
    }
    digital_cam_uart_shoot(MORA_CAMERA_EARCAM, report);
  }
}

uint8_t earcam_start(void)
{
  if (earcam_period_s < EARCAM_MIN_PERIOD_S) {
    earcam_period_s = EARCAM_MIN_PERIOD_S;
  }
  earcam_result_clear();
  last_shot_time = 0.f;
  last_report_time = 0.f;
  last_motor_time = get_sys_time_float();
  sampling_active = true;
  earcam_refine_iteration = 0;
  earcam_refine_converged = false;
  previous_result_valid = false;
  earcam_drop_attempts = 0;
  earcam_drop_released = false;
  earcam_drop_missed = false;
  return 0;
}

uint8_t earcam_stop(void)
{
  sampling_active = false;
  return digital_cam_uart_stop(MORA_CAMERA_EARCAM, false);
}

uint8_t earcam_solve(void)
{
  // Sampling keeps running; MORA answers with an interim MORA_EAR_RESULT.
  // The last result stays available; only the "fresh" flag waits for the reply.
  if (earcam_result_valid) {
    previous_result_lla = result_lla;
    previous_result_valid = true;
  }
  earcam_result_fresh = false;
  return digital_cam_uart_stop(MORA_CAMERA_EARCAM, true);
}

uint8_t earcam_result_clear(void)
{
  earcam_result_valid = false;
  earcam_result_fresh = false;
  earcam_samples = 0;
  earcam_confidence = 0.f;
  return 0;
}

uint8_t earcam_result_to_waypoint(uint8_t wp_id)
{
  if (!earcam_result_valid) {
    return 1;
  }
#if FIXEDWING_FIRMWARE
  struct UtmCoor_f utm;
  struct LlaCoor_f lla;
  LLA_FLOAT_OF_BFP(lla, result_lla);
  utm.zone = nav_utm_zone0;
  utm_of_lla_f(&utm, &lla);
  nav_move_waypoint(wp_id, utm.east, utm.north, WaypointAlt(wp_id));
#else
  waypoint_set_latlon(wp_id, &result_lla);
#endif
  return 0;
}

uint8_t earcam_place_run_in(uint8_t wp_start, uint8_t wp_target, float distance_m)
{
  // Start point distance_m short of the target, opposite to the current course, so the
  // final leg start->target is a straight run-in flown on the present heading.
  return earcam_place_run_in_course(wp_start, wp_target, distance_m, stateGetHorizontalSpeedDir_f());
}

uint8_t earcam_place_run_in_course(uint8_t wp_start, uint8_t wp_target, float distance_m, float course)
{
  float target_x, target_y;
#if FIXEDWING_FIRMWARE
  target_x = WaypointX(wp_target);
  target_y = WaypointY(wp_target);
#else
  target_x = waypoint_get_x(wp_target);
  target_y = waypoint_get_y(wp_target);
#endif
  if (distance_m < 1.f) {
    distance_m = 1.f;
  }
  set_wp_xy(wp_start, target_x - sinf(course) * distance_m, target_y - cosf(course) * distance_m);
  return 0;
}

float earcam_run_in_course_deg = 0.f;
bool earcam_run_in_clear = false;

static bool corridor_clear(float tx, float ty, float course, float before_m, float after_m, float margin_m,
                           earcam_obstacle_fn obstacle)
{
  const float ax = sinf(course), ay = cosf(course);   // along track
  const float rx = ay, ry = -ax;                      // to the right of track
  for (float s = -before_m; s <= after_m; s += 5.f) {
    for (int k = -1; k <= 1; k++) {
      if (obstacle(tx + ax * s + rx * margin_m * k, ty + ay * s + ry * margin_m * k)) {
        return false;
      }
    }
  }
  return true;
}

bool earcam_place_run_in_lane(uint8_t wp_start, uint8_t wp_target, float before_m, float after_m,
                              float margin_m, float lane_deg, float max_tilt_deg, float step_deg,
                              float preferred_course, earcam_obstacle_fn obstacle)
{
  float target_x, target_y;
#if FIXEDWING_FIRMWARE
  target_x = WaypointX(wp_target);
  target_y = WaypointY(wp_target);
#else
  target_x = waypoint_get_x(wp_target);
  target_y = waypoint_get_y(wp_target);
#endif
  if (step_deg < 1.f) {
    step_deg = 1.f;
  }
  const float lane = RadOfDeg(lane_deg);
  // Which tilt sign brings the course nearer to the preferred (upwind) course.
  float diff = preferred_course - lane;
  NormRadAngle(diff);
  const float first_sign = (diff >= 0.f) ? 1.f : -1.f;

  earcam_run_in_clear = false;
  float chosen = lane;
  for (float tilt = 0.f; tilt <= max_tilt_deg + 0.01f && !earcam_run_in_clear; tilt += step_deg) {
    for (int side = 0; side < (tilt > 0.f ? 2 : 1); side++) {
      float course = lane + RadOfDeg(tilt) * first_sign * (side == 0 ? 1.f : -1.f);
      if (corridor_clear(target_x, target_y, course, before_m, after_m, margin_m, obstacle)) {
        chosen = course;
        earcam_run_in_clear = true;
        break;
      }
    }
  }
  NormRadAngle(chosen);
  earcam_run_in_course_deg = DegOfRad(chosen);
  earcam_place_run_in_course(wp_start, wp_target, before_m, chosen);
  return earcam_run_in_clear;
}

float earcam_place_climbout(uint8_t wp_start, uint8_t wp_target, uint8_t wp_climbout, uint8_t wp_exit,
                            float max_m, float min_m, float turn_radius_m, float margin_m, float side,
                            float alt, earcam_obstacle_fn keep_out)
{
#if FIXEDWING_FIRMWARE
  const float tx = WaypointX(wp_target), ty = WaypointY(wp_target);
  float ax = tx - WaypointX(wp_start), ay = ty - WaypointY(wp_start);
#else
  const float tx = waypoint_get_x(wp_target), ty = waypoint_get_y(wp_target);
  float ax = tx - waypoint_get_x(wp_start), ay = ty - waypoint_get_y(wp_start);
#endif
  const float run = sqrtf(ax * ax + ay * ay);
  if (run < 1.f) {
    return 0.f;
  }
  ax /= run;
  ay /= run;
  const float rx = ay, ry = -ax;   // right of track
  const float ring = turn_radius_m + margin_m;
  float length = 0.f;
  for (float s = 5.f; s <= max_m + 0.01f; s += 5.f) {
    // Exit circle centre for a climb-out ending here; nothing within ring of it may be keep-out.
    const float cx = tx + ax * s + rx * side * turn_radius_m;
    const float cy = ty + ay * s + ry * side * turn_radius_m;
    bool free = !keep_out(tx + ax * s, ty + ay * s);
    for (int k = 0; k < 12 && free; k++) {
      const float a = k * (2.f * M_PI / 12.f);
      free = !keep_out(cx + ring * sinf(a), cy + ring * cosf(a));
    }
    if (!free) {
      break;
    }
    length = s;
  }
  if (length < min_m) {
    length = min_m;
  }
  const float cx = tx + ax * length, cy = ty + ay * length;
  nav_move_waypoint_enu(wp_climbout, cx, cy, alt);
  nav_move_waypoint_enu(wp_exit, cx + rx * side * turn_radius_m, cy + ry * side * turn_radius_m, alt);
  return length;
}

/* ------------------------------------------------------------------ */
/* Adaptive star refinement                                            */
/* ------------------------------------------------------------------ */

static void result_to_local_xy(const struct LlaCoor_i *lla, float *x, float *y)
{
#if FIXEDWING_FIRMWARE
  struct UtmCoor_f utm;
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, *lla);
  utm.zone = nav_utm_zone0;
  utm_of_lla_f(&utm, &lla_f);
  *x = utm.east - nav_utm_east0;
  *y = utm.north - nav_utm_north0;
#else
  struct EnuCoor_i enu;
  enu_of_lla_point_i(&enu, stateGetNedOrigin_i(), (struct LlaCoor_i *)lla);
  // enu_of_lla_point_i returns centimetres
  *x = enu.x / 100.f;
  *y = enu.y / 100.f;
#endif
}

static float refine_turn_radius_abs(void)
{
  if (earcam_refine_turn_radius_m > 1.f) {
    return earcam_refine_turn_radius_m;
  }
  return fabsf(nav_radius) > 1.f ? fabsf(nav_radius) : DEFAULT_CIRCLE_RADIUS;
}

// Height above ground_alt for the current star, lowered per completed star.
static float refine_height(float base_height)
{
  float h = base_height - earcam_refine_height_step_m * (float)earcam_refine_iteration;
  return h < earcam_refine_height_min_m ? earcam_refine_height_min_m : h;
}

static void set_wp_xy_height(uint8_t wp_id, float x, float y, float height)
{
#if FIXEDWING_FIRMWARE
  if (earcam_refine_height_from_m > 0.f) {
    nav_move_waypoint_enu(wp_id, x, y, ground_alt + height);
    return;
  }
#endif
  (void)height;
  set_wp_xy(wp_id, x, y);
}

// Consecutive legs meet at the centre under 180/N degrees. A circle of radius R
// tangent to both leg lines touches them R/tan(90/N) from the centre. Legs end at
// that tangent point (extending the configured half-length when needed, or growing
// the circle when the half-length is longer), so the turn exits exactly onto the
// next leg with the right heading and every pass over the estimate is a straight,
// wings-level run-in.
static void refine_setup_geometry(void)
{
  float beta = EARCAM_PI / (2.f * (float)earcam_refine_legs);
  float tan_beta = tanf(beta);
  float fillet = refine_turn_radius_abs() / tan_beta;
  refine_half_length = earcam_refine_half_length_m > fillet ? earcam_refine_half_length_m : fillet;
  refine_circle_radius = refine_half_length * tan_beta;
}

// Leg k has heading k*180/N through the center. Alternating direction puts the
// end of leg k and the start of leg k+1 on the same side, so the join is one
// circle turn of (180 - 180/N) degrees, e.g. 135 deg for a 4-leg star.
static void leg_geometry(uint8_t leg, float *from_x, float *from_y, float *to_x, float *to_y)
{
  float heading = (float)leg * EARCAM_PI / (float)earcam_refine_legs;
  float dx = sinf(heading) * refine_half_length;
  float dy = cosf(heading) * refine_half_length;
  if (leg % 2 == 1) {
    dx = -dx;
    dy = -dy;
  }
  *from_x = refine_center_x - dx;
  *from_y = refine_center_y - dy;
  *to_x = refine_center_x + dx;
  *to_y = refine_center_y + dy;
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

static void place_leg_waypoints(uint8_t wp_from, uint8_t wp_to, uint8_t wp_turn)
{
  float from_x, from_y, to_x, to_y;
  leg_geometry(refine_leg, &from_x, &from_y, &to_x, &to_y);
  float height_from = refine_height(earcam_refine_height_from_m);
  float height_to = refine_height(earcam_refine_height_to_m);
  set_wp_xy_height(wp_from, from_x, from_y, height_from);
  set_wp_xy_height(wp_to, to_x, to_y, height_to);

  // Turn circle after this leg: tangent to the leg at its end, on the side of the
  // next leg's start, so the aircraft rolls straight into the turn and exits onto
  // the next run-in. Exit heading is the next leg's heading.
  uint8_t next = (uint8_t)(refine_leg + 1);
  if (next >= earcam_refine_legs) {
    next = 0;
  }
  float next_from_x, next_from_y, next_to_x, next_to_y;
  leg_geometry(next, &next_from_x, &next_from_y, &next_to_x, &next_to_y);
  float leg_dx = to_x - from_x;
  float leg_dy = to_y - from_y;
  float leg_len = sqrtf(leg_dx * leg_dx + leg_dy * leg_dy);
  if (leg_len < 1.f) {
    leg_len = 1.f;
  }
  // Left-hand normal of the leg direction; sign chosen towards the next start.
  float normal_x = -leg_dy / leg_len;
  float normal_y = leg_dx / leg_len;
  float side = (next_from_x - to_x) * normal_x + (next_from_y - to_y) * normal_y;
  float radius = refine_circle_radius;
  if (side < 0.f) {
    normal_x = -normal_x;
    normal_y = -normal_y;
    refine_turn_radius = radius;      // clockwise
  } else {
    refine_turn_radius = -radius;     // counter-clockwise
  }
  set_wp_xy_height(wp_turn, to_x + normal_x * radius, to_y + normal_y * radius, height_from);
  refine_exit_qdr_deg = DegOfRad(atan2f(next_to_x - next_from_x, next_to_y - next_from_y));
}

uint8_t earcam_refine_setup(uint8_t wp_from, uint8_t wp_to, uint8_t wp_turn)
{
  if (!earcam_result_valid) {
    return 1;
  }
  if (earcam_refine_legs < 2) {
    earcam_refine_legs = 2;
  } else if (earcam_refine_legs > 8) {
    earcam_refine_legs = 8;
  }
  refine_setup_geometry();
  result_to_local_xy(&result_lla, &refine_center_x, &refine_center_y);
  refine_leg = 0;
  place_leg_waypoints(wp_from, wp_to, wp_turn);
  earcam_result_fresh = false;  // consumed; refine_solve waits for the next reply
  return 0;
}

uint8_t earcam_refine_next_leg(uint8_t wp_from, uint8_t wp_to, uint8_t wp_turn)
{
  refine_leg++;
  if (refine_leg >= earcam_refine_legs) {
    return 1;
  }
  place_leg_waypoints(wp_from, wp_to, wp_turn);
  return 0;
}

float earcam_refine_turn_radius(void)
{
  return refine_turn_radius;
}

bool earcam_refine_turning(void)
{
#if FIXEDWING_FIRMWARE
  // On the circle, the aircraft heading equals QDR +/- 90 deg depending on turn
  // direction; exit when the heading matches the next leg's course.
  float heading_deg = DegOfRad(stateGetHorizontalSpeedDir_f());
  float diff = heading_deg - refine_exit_qdr_deg;
  while (diff > 180.f) { diff -= 360.f; }
  while (diff < -180.f) { diff += 360.f; }
  return fabsf(diff) > 6.f;
#else
  return false;
#endif
}

uint8_t earcam_refine_update(void)
{
  earcam_refine_iteration++;
  earcam_result_fresh = false;  // consumed
  if (!earcam_result_valid || !previous_result_valid) {
    earcam_refine_shift_m = 0.f;
    earcam_refine_converged = earcam_refine_iteration >= earcam_refine_max_iterations;
    return earcam_result_valid ? 0 : 1;
  }
  float x0, y0, x1, y1;
  result_to_local_xy(&previous_result_lla, &x0, &y0);
  result_to_local_xy(&result_lla, &x1, &y1);
  earcam_refine_shift_m = sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
  earcam_refine_converged = earcam_refine_shift_m <= earcam_refine_converge_m
                            || earcam_refine_iteration >= earcam_refine_max_iterations;
  return 0;
}

/* ------------------------------------------------------------------ */
/* Known search area                                                   */
/* ------------------------------------------------------------------ */

uint8_t earcam_refine_leg_throttle(float kill_before_m, float restore_after_m)
{
#if FIXEDWING_FIRMWARE
  float from_x, from_y, to_x, to_y;
  leg_geometry(refine_leg, &from_x, &from_y, &to_x, &to_y);
  float dx = to_x - from_x;
  float dy = to_y - from_y;
  float len = sqrtf(dx * dx + dy * dy);
  bool quiet = false;
  if (len > 1.f) {
    struct EnuCoor_f *pos = stateGetPositionEnu_f();
    // Along-track position relative to the centre, positive towards the leg end.
    float s = ((pos->x - refine_center_x) * dx + (pos->y - refine_center_y) * dy) / len;
    quiet = s >= -kill_before_m && s <= restore_after_m;
  }
  autopilot_set_kill_throttle(quiet);
#else
  (void)kill_before_m;
  (void)restore_after_m;
#endif
  return 0;
}

uint8_t earcam_result_from_waypoint(uint8_t wp_id)
{
#if FIXEDWING_FIRMWARE
  struct UtmCoor_f utm;
  struct LlaCoor_f lla;
  utm.east = nav_utm_east0 + WaypointX(wp_id);
  utm.north = nav_utm_north0 + WaypointY(wp_id);
  utm.alt = WaypointAlt(wp_id);
  utm.zone = nav_utm_zone0;
  lla_of_utm_f(&lla, &utm);
  LLA_BFP_OF_REAL(result_lla, lla);
#else
  result_lla = *waypoint_get_lla(wp_id);
#endif
  earcam_lat_deg = result_lla.lat / 1e7f;
  earcam_lon_deg = result_lla.lon / 1e7f;
  earcam_alt_m = result_lla.alt / 1000.f;
  earcam_agl_m = 0.f;
  earcam_level_db = 0.f;
  earcam_confidence = 0.f;
  earcam_samples = 0;
  earcam_result_valid = true;
  earcam_result_fresh = false;
  return 0;
}

bool earcam_result_within(uint8_t wp_id, float radius_m)
{
  if (!earcam_result_valid) {
    return false;
  }
  float x, y, wx, wy;
  result_to_local_xy(&result_lla, &x, &y);
#if FIXEDWING_FIRMWARE
  wx = WaypointX(wp_id);
  wy = WaypointY(wp_id);
#else
  wx = waypoint_get_x(wp_id);
  wy = waypoint_get_y(wp_id);
#endif
  float dx = x - wx;
  float dy = y - wy;
  return dx * dx + dy * dy <= radius_m * radius_m;
}

/* ------------------------------------------------------------------ */
/* Low release: complements nav_drop (approach, release point, hatch) */
/* ------------------------------------------------------------------ */

#if FIXEDWING_FIRMWARE

// Height above ground: rangefinder when in range, barometric over the target otherwise.
static float drop_agl(uint8_t wp_target, bool *rangefinder)
{
  *rangefinder = false;
#if EARCAM_USE_AGL_DIST
  float now = get_sys_time_float();
  if (agl_dist_valid) {
    drop_last_agl_time = now;
    drop_last_agl_value = agl_dist_value_filtered;
    *rangefinder = true;
    return agl_dist_value_filtered;
  }
  // Reading lost right after a low value: assume below the minimum range (go around).
  if (drop_last_agl_time >= 0.f && now - drop_last_agl_time < 1.f && drop_last_agl_value < 1.2f) {
    *rangefinder = true;
    return EARCAM_DROP_ABORT_AGL_M - 0.1f;
  }
#endif
  return stateGetPositionUtm_f()->alt - WaypointAlt(wp_target);
}

uint8_t earcam_drop_level_point(uint8_t wp_start, uint8_t wp_release, uint8_t wp_target,
                                uint8_t wp_level, float level_m)
{
  float dx = WaypointX(wp_release) - WaypointX(wp_start);
  float dy = WaypointY(wp_release) - WaypointY(wp_start);
  float run_in = sqrtf(dx * dx + dy * dy);
  if (run_in < 1.f) {
    return 1;
  }
  if (level_m > run_in - 1.f) {
    level_m = run_in - 1.f;
  }
  nav_move_waypoint_enu(wp_level, WaypointX(wp_release) - dx / run_in * level_m,
                        WaypointY(wp_release) - dy / run_in * level_m,
                        WaypointAlt(wp_target) + earcam_drop_release_agl_m);
  return 0;
}

float earcam_drop_altitude(uint8_t wp_target)
{
  bool rangefinder;
  float agl = drop_agl(wp_target, &rangefinder);
  earcam_drop_agl_m = agl;
  if (rangefinder && agl <= EARCAM_DROP_AGL_MAX_M) {
    // Hold the release height over the ground actually measured below.
    return stateGetPositionUtm_f()->alt - agl + earcam_drop_release_agl_m;
  }
  return WaypointAlt(wp_target) + earcam_drop_release_agl_m;
}

bool earcam_drop_too_low(uint8_t wp_target)
{
  bool rangefinder;
  float agl = drop_agl(wp_target, &rangefinder);
  return rangefinder && agl < EARCAM_DROP_ABORT_AGL_M;
}

uint8_t earcam_drop_shoot(uint8_t wp_target)
{
  bool rangefinder;
  earcam_drop_agl_m = drop_agl(wp_target, &rangefinder);
  if (earcam_drop_agl_m > earcam_drop_max_agl_m) {
    earcam_drop_missed = true;
    earcam_drop_released = false;
    return 1;
  }
#if EARCAM_HAVE_NAV_DROP
  nav_drop_shoot();
#endif
  earcam_drop_attempts++;
  earcam_drop_missed = false;
  earcam_drop_released = true;
  return 0;
}

uint8_t earcam_drop_mark_impact(uint8_t wp_target, uint8_t wp_release, uint8_t wp_mark)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  float speed = stateGetHorizontalSpeedNorm_f();
  float course = stateGetHorizontalSpeedDir_f();
  float delay = 0.f;
#if EARCAM_HAVE_NAV_DROP && !NAV_DROP_RELEASE_WITH_DELAY
  // Otherwise the delay travel is already inside the target-release offset.
  delay = nav_drop_trigger_delay;
#endif
  set_wp_xy(wp_mark, pos->x + speed * sinf(course) * delay + WaypointX(wp_target) - WaypointX(wp_release),
            pos->y + speed * cosf(course) * delay + WaypointY(wp_target) - WaypointY(wp_release));
  return 0;
}

#else /* not FIXEDWING_FIRMWARE */

uint8_t earcam_drop_level_point(uint8_t wp_start, uint8_t wp_release, uint8_t wp_target,
                                uint8_t wp_level, float level_m)
{
  (void)wp_start; (void)wp_release; (void)wp_target; (void)wp_level; (void)level_m;
  return 1;
}
float earcam_drop_altitude(uint8_t wp_target) { (void)wp_target; return 0.f; }
bool earcam_drop_too_low(uint8_t wp_target) { (void)wp_target; return false; }
uint8_t earcam_drop_shoot(uint8_t wp_target) { (void)wp_target; return 1; }
uint8_t earcam_drop_mark_impact(uint8_t wp_target, uint8_t wp_release, uint8_t wp_mark)
{
  (void)wp_target; (void)wp_release; (void)wp_mark;
  return 1;
}

#endif
