/*
 * Copyright (C) 2025 Noah Wechtler <noah.wechtler@tudelft.nl>
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
 * @file "modules/nav/nav_moving_base.c"
 * Specific navigation functions for landing on a moving base
 */

#include "modules/nav/nav_moving_base.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "modules/nav/waypoints.h"

#ifdef WP_MOVING_BASE
#include "modules/datalink/downlink.h"
#endif

#ifndef NAV_MOVING_BASE_OVERWRITE_POS_GAIN
#define NAV_MOVING_BASE_OVERWRITE_POS_GAIN FALSE
#endif

#ifndef NAV_MOVING_BASE_OVERWRITE_POS_GAINZ
#define NAV_MOVING_BASE_OVERWRITE_POS_GAINZ FALSE
#endif

#ifndef NAV_MOVING_BASE_OVERWRITE_SPEED_GAIN
#define NAV_MOVING_BASE_OVERWRITE_SPEED_GAIN FALSE
#endif

#ifndef NAV_MOVING_BASE_OVERWRITE_SPEED_GAINZ
#define NAV_MOVING_BASE_OVERWRITE_SPEED_GAINZ FALSE
#endif

#ifndef NAV_MOVING_BASE_DESCEND_SPEED
#define NAV_MOVING_BASE_DESCEND_SPEED 0.5f
#endif

#ifndef NAV_MOVING_BASE_MAX_H_ACCEL
#define NAV_MOVING_BASE_MAX_H_ACCEL (struct FloatVect2[2]) { \
  { .x = 3.0f, .y = -1.0f }, \
  { .x = 1.0f, .y = -1.0f } \
}
#endif

#ifndef NAV_MOVING_BASE_MAX_H_SPEED
#define NAV_MOVING_BASE_MAX_H_SPEED (struct FloatVect2[2]) { \
  { .x = 12.0f, .y = -12.0f }, \
  { .x = 5.0f, .y = -5.0f } \
}
#endif

#ifndef NAV_MOVING_BASE_MAX_V_ACCEL
// Use FloatVect2 for convenient math macros, x is upwards limit, y is downwards limit
#define NAV_MOVING_BASE_MAX_V_ACCEL (struct FloatVect2) { \
  .x = 2.0f, \
  .y = -2.0f \
}
#endif

#ifndef NAV_MOVING_BASE_MAX_V_SPEED
// Use FloatVect2 for convenient math macros, x is upwards limit, y is downwards limit
#define NAV_MOVING_BASE_MAX_V_SPEED (struct FloatVect2) { \
  .x = 2.0f, \
  .y = -1.0f \
}
#endif

struct NavMovingBase nav_moving_base;

#if defined(GUIDANCE_INDI_POS_GAIN) && !NAV_MOVING_BASE_OVERWRITE_POS_GAIN
float pos_gain_h = GUIDANCE_INDI_POS_GAIN;
#elif defined(NAV_MOVING_BASE_POS_GAIN)
float pos_gain_h = NAV_MOVING_BASE_POS_GAIN;
PRINT_CONFIG_VAR(NAV_MOVING_BASE_POS_GAIN);
#else
float pos_gain_h = 1.0f;
#endif

#if defined(GUIDANCE_INDI_POS_GAINZ) && !NAV_MOVING_BASE_OVERWRITE_POS_GAINZ
float pos_gain_v = GUIDANCE_INDI_POS_GAINZ;
#elif defined(NAV_MOVING_BASE_POS_GAINZ)
float pos_gain_v = NAV_MOVING_BASE_POS_GAINZ;
PRINT_CONFIG_VAR(NAV_MOVING_BASE_POS_GAINZ);
#else
float pos_gain_v = 1.0f;
#endif

#if defined(GUIDANCE_INDI_SPEED_GAIN) && !NAV_MOVING_BASE_OVERWRITE_SPEED_GAIN
float speed_gain_h = GUIDANCE_INDI_SPEED_GAIN;
#elif defined(NAV_MOVING_BASE_SPEED_GAIN)
float speed_gain_h = NAV_MOVING_BASE_SPEED_GAIN;
PRINT_CONFIG_VAR(NAV_MOVING_BASE_SPEED_GAIN);
#else
float speed_gain_h = 1.0f;
#endif

#if defined(GUIDANCE_INDI_SPEED_GAINZ) && !NAV_MOVING_BASE_OVERWRITE_SPEED_GAINZ
float speed_gain_v = GUIDANCE_INDI_SPEED_GAINZ;
#elif defined(NAV_MOVING_BASE_SPEED_GAINZ)
PRINT_CONFIG_VAR(NAV_MOVING_BASE_SPEED_GAINZ);
float speed_gain_v = NAV_MOVING_BASE_SPEED_GAINZ;
#else
float speed_gain_v = 1.0f;
#endif

PRINT_CONFIG_VAR(NAV_MOVING_BASE_DESCEND_SPEED);

static bool nav_moving_base_track(void);
static bool nav_moving_base_descend(void);
static bool nav_moving_base_land(void);
static void enforce_horizontal_bounds(struct EnuCoor_f* quantity, struct FloatVect2* bounds);

void nav_moving_base_init(void) {
  struct FloatVect2 max_h_accel[2] = NAV_MOVING_BASE_MAX_H_ACCEL;
  struct FloatVect2 max_h_speed[2] = NAV_MOVING_BASE_MAX_H_SPEED;
  struct FloatVect2 max_v_accel = NAV_MOVING_BASE_MAX_V_ACCEL;
  struct FloatVect2 max_v_speed = NAV_MOVING_BASE_MAX_V_SPEED;

  VECT2_COPY(nav_moving_base.max_accel_h[0], max_h_accel[0]);
  VECT2_COPY(nav_moving_base.max_accel_h[1], max_h_accel[1]);
  VECT2_COPY(nav_moving_base.max_speed_h[0], max_h_speed[0]);
  VECT2_COPY(nav_moving_base.max_speed_h[1], max_h_speed[1]);
  VECT2_COPY(nav_moving_base.max_accel_v, max_v_accel);
  VECT2_COPY(nav_moving_base.max_speed_v, max_v_speed);

  nav_moving_base.pos_gain = (struct FloatVect3) {pos_gain_h, pos_gain_h, pos_gain_v};
  nav_moving_base.speed_gain = (struct FloatVect3) {speed_gain_h, speed_gain_h, speed_gain_v};

  nav_moving_base.stay = false;
}

void nav_moving_base_setup(enum NavMovingBaseMode mode) {
  nav_moving_base.mode = mode;

  switch (nav_moving_base.mode) {
    case NAV_MOVING_BASE_MODE_TRACKING_NO_FF:
    case NAV_MOVING_BASE_MODE_TRACKING:
      nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;
      nav.vertical_mode = NAV_VERTICAL_MODE_ALT;
      nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
      break;

    case NAV_MOVING_BASE_MODE_DESCEND_NO_FF:
    case NAV_MOVING_BASE_MODE_DESCEND:
      nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;
      nav.vertical_mode = NAV_VERTICAL_MODE_CLIMB;
      nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
      break;

    case NAV_MOVING_BASE_MODE_LANDING:
      nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;
      nav.vertical_mode = NAV_VERTICAL_MODE_ALL;
      nav.setpoint_mode = NAV_SETPOINT_MODE_ALL;
      break;
  }
}

bool nav_moving_base_run(void) {
  bool complete = false;
  
  switch (nav_moving_base.mode) {
    case NAV_MOVING_BASE_MODE_TRACKING_NO_FF:
    case NAV_MOVING_BASE_MODE_TRACKING:
      complete = nav_moving_base_track();
      break;
    case NAV_MOVING_BASE_MODE_DESCEND_NO_FF:
    case NAV_MOVING_BASE_MODE_DESCEND:
      complete = nav_moving_base_descend();
      break;
    case NAV_MOVING_BASE_MODE_LANDING:
      complete = nav_moving_base_land();
      break;
  }

  if (nav_moving_base.stay) {
    return true;
  }

  return !complete;
}

static bool nav_moving_base_track(void) {
  bool complete = false;
  
  VECT2_COPY(nav.target, nav_moving_base.pos);

  // Calculate position error
  struct EnuCoor_f pos_error;
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  VECT2_DIFF(pos_error, nav.target, *pos);

  struct EnuCoor_f speed_sp;
  VECT2_EMUL(speed_sp, pos_error, nav_moving_base.pos_gain);
  
  if (nav_moving_base.mode == NAV_MOVING_BASE_MODE_TRACKING ||
      nav_moving_base.mode == NAV_MOVING_BASE_MODE_DESCEND) {
    VECT2_ADD(speed_sp, nav_moving_base.speed);
  }

  // Enforce general horizontal speed limits
  enforce_horizontal_bounds(&speed_sp, nav_moving_base.max_speed_h);
  VECT2_COPY(nav.speed, speed_sp);

  return complete;
}

// Tracking of target while descending/ascending
static bool nav_moving_base_descend(void) {
  // Horizontal navigation during descent is the same as during tracking
  bool vert_complete = false;
  bool hor_complete = nav_moving_base_track();

  // Vertical navigation, hardcoded descend speed or make some kind of reference?
  float speed_sp_v = -NAV_MOVING_BASE_DESCEND_SPEED;
  Bound(speed_sp_v, nav_moving_base.max_speed_v.y, nav_moving_base.max_speed_v.x);

  // nav.target.z = nav_moving_base.pos.z;
  nav.climb = speed_sp_v;

  return (vert_complete && hor_complete);
}

static bool nav_moving_base_land(void) {
  // Set the nav values based on the values
  bool complete = false;
  
  struct EnuCoor_f vel = nav_moving_base.speed;
  struct EnuCoor_f accel = nav_moving_base.accel;

  enforce_horizontal_bounds(&vel, nav_moving_base.max_speed_h);
  enforce_horizontal_bounds(&accel, nav_moving_base.max_accel_h);

  VECT3_COPY(nav.target, nav_moving_base.pos);
  VECT3_COPY(nav.speed, vel);
  VECT3_COPY(nav.accel, accel);

  return complete;
}

static void enforce_horizontal_bounds(struct EnuCoor_f* quantity, struct FloatVect2* bounds) {
  // Quantity (Velocity or Accel) is in ENU frame, but should be bounded in control frame
  struct FloatVect2 quantity_control_rf;
  struct FloatEulers eulers_zxy = *stateGetNedToBodyEulers_f();

  float sin_psi = sinf(eulers_zxy.psi);
  float cos_psi = cosf(eulers_zxy.psi);

  // From ENU to control frame
  quantity_control_rf.x = quantity->y * cos_psi + quantity->x * sin_psi;
  quantity_control_rf.y = -quantity->y * sin_psi + quantity->x * cos_psi;

  // Check if the quantity to be bound is positive or negative
  quantity_control_rf.x = (quantity_control_rf.x >= 0)? Min(quantity_control_rf.x, bounds[0].x) : Max(quantity_control_rf.x, bounds[0].y);
  quantity_control_rf.y = (quantity_control_rf.y >= 0)? Min(quantity_control_rf.y, bounds[1].x) : Max(quantity_control_rf.y, bounds[1].y);

  quantity->x = quantity_control_rf.y * cos_psi + quantity_control_rf.x * sin_psi;
  quantity->y = -quantity_control_rf.y * sin_psi + quantity_control_rf.x * cos_psi;
}

void nav_moving_base_set_pos(struct EnuCoor_f *pos) {
  VECT3_COPY(nav_moving_base.pos, *pos);
#ifdef WP_MOVING_BASE
  waypoint_set_enu(WP_MOVING_BASE, &nav_moving_base.pos);
  RunOnceEvery(50, {DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &(uint8_t) {WP_MOVING_BASE},
      &waypoints[WP_MOVING_BASE].enu_i.x,
      &waypoints[WP_MOVING_BASE].enu_i.y,
      &waypoints[WP_MOVING_BASE].enu_i.z);
  });
#endif
}

void nav_moving_base_set_speed(struct EnuCoor_f *speed) {
  VECT3_COPY(nav_moving_base.speed, *speed);
}

void nav_moving_base_set_accel(struct EnuCoor_f *accel) {
  VECT3_COPY(nav_moving_base.accel, *accel);
}

void nav_moving_base_set_max_accel_h(struct FloatVect2 longitudinal, struct FloatVect2 lateral) {
  VECT2_COPY(nav_moving_base.max_accel_h[0], longitudinal);
  VECT2_COPY(nav_moving_base.max_accel_h[1], lateral);
}

void nav_moving_base_set_max_speed_h(struct FloatVect2 longitudinal, struct FloatVect2 lateral) {
  VECT2_COPY(nav_moving_base.max_speed_h[0], longitudinal);
  VECT2_COPY(nav_moving_base.max_speed_h[1], lateral);
}