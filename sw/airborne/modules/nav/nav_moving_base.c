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
#include "math/pprz_algebra.h"

#ifndef NAV_MOVING_BASE_MAX_H_ACCEL
#define NAV_MOVING_BASE_MAX_H_ACCEL (struct FloatVect2) { \
  .x = 2.0f, \
  .y = 2.0f \
}
#endif

#ifndef NAV_MOVING_BASE_MAX_H_SPEED
#define NAV_MOVING_BASE_MAX_H_SPEED (struct FloatVect2) { \
  .x = 10.0f, \
  .y = 10.0f \
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
  .y = -2.0f \
}
#endif

static void nav_moving_base_track(void);
static void nav_moving_base_descend(void);
static void nav_moving_base_land(void);
static void enforce_horizontal_bounds(struct EnuCoor_f* quantity, struct FloatVect2* bounds);

static struct NavMovingBase nav_moving_base;
  
void nav_moving_base_init(void) {
  VECT2_COPY(nav_moving_base.max_accel_h, NAV_MOVING_BASE_MAX_H_ACCEL);
  VECT2_COPY(nav_moving_base.max_speed_h, NAV_MOVING_BASE_MAX_H_SPEED);
  VECT2_COPY(nav_moving_base.max_accel_v, NAV_MOVING_BASE_MAX_V_ACCEL);
  VECT2_COPY(nav_moving_base.max_speed_v, NAV_MOVING_BASE_MAX_V_SPEED);
  nav_moving_base.complete = false;
  nav_moving_base.stay_indefinitely = false;
}

void nav_moving_base_setup(int waypoint_id, enum NavMovingBaseMode mode) {
  nav_moving_base.complete = false;
  nav_moving_base.mode = mode;
  nav_moving_base.wp = waypoints[waypoint_id];
}

bool nav_moving_base_run(void) {
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
      NavStartDetectGround();
      nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;
      nav.vertical_mode = NAV_VERTICAL_MODE_GUIDED;
      nav.setpoint_mode = NAV_SETPOINT_MODE_ALL;
      break;
  }
}

bool nav_moving_base_run(void) {
  switch (nav_moving_base.mode) {
    case NAV_MOVING_BASE_MODE_TRACKING_NO_FF:
    case NAV_MOVING_BASE_MODE_TRACKING:
      nav_moving_base_track();
      break;
    case NAV_MOVING_BASE_MODE_DESCEND_NO_FF:
    case NAV_MOVING_BASE_MODE_DESCEND:
      nav_moving_base_descend();
      break;
    case NAV_MOVING_BASE_MODE_LANDING:
      nav_moving_base_land();
      break;
  }

  if (nav_moving_base.stay) {
    return false;
  }

  return false;
}

static void nav_moving_base_track(void) {
  VECT2_COPY(nav.target, nav_moving_base.pos);

  // Calculate position error
  struct EnuCoor_f pos_error;
  struct FloatVect2 pos_error_control_rf;
  struct FloatEulers eulers_zxy = *stateGetNedToBodyEulers_f();
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  VECT2_DIFF(pos_error, nav.target, *pos);

  float cos_psi = cosf(eulers_zxy.psi);
  float sin_psi = sinf(eulers_zxy.psi);

  pos_error_control_rf.x = pos_error.y * cos_psi + pos_error.x * sin_psi;
  pos_error_control_rf.y = -pos_error.y * sin_psi + pos_error.x * cos_psi;

  struct EnuCoor_f speed_sp;
  struct FloatVect2 wp_speed_control_rf = {0};
  VECT2_EMUL(speed_sp, pos_error, nav_moving_base.pos_gain);
  
  if (nav_moving_base.mode == NAV_MOVING_BASE_MODE_TRACKING ||
      nav_moving_base.mode == NAV_MOVING_BASE_MODE_DESCEND) {
    VECT2_ADD(speed_sp, nav_moving_base.speed);

    wp_speed_control_rf.x = speed_sp.y * cos_psi + speed_sp.x * sin_psi;
    wp_speed_control_rf.y = -speed_sp.y * sin_psi + speed_sp.x * cos_psi;
  }

  // Bound the setpoint velocity vector
  struct FloatVect2 max_h_accel;

  max_h_accel.x = (pos_error_control_rf.x >= 0) ? nav_moving_base.max_accel_h[0].x : nav_moving_base.max_accel_h[0].y;
  max_h_accel.y = (pos_error_control_rf.y >= 0) ? nav_moving_base.max_accel_h[1].x : nav_moving_base.max_accel_h[1].y;
  if (!force_forward) {
    // If not in force_forward, compute speed based on decceleration and nav_goto_max_speed
    // Calculate distance to waypoint

    // Calculate max speed when decelerating at MAX capacity a_max
    // distance travelled d = 1/2 a_max t^2
    // The time in which it does this is: T = V / a_max
    // The maximum speed at which to fly to still allow arriving with zero
    // speed at the waypoint given maximum deceleration is: V = sqrt(2 * a_max * d)
    struct FloatVect2 max_speed_decel;
    VECT2_EMUL(max_speed_decel, pos_error_control_rf, max_h_accel); // dist_to_wp can only be positive, but just in case
    VECT2_SMUL(max_speed_decel, max_speed_decel, 2.f * 0.8);
    max_speed_decel.x = sqrtf(fabsf(max_speed_decel.x));
    max_speed_decel.y = sqrtf(fabsf(max_speed_decel.y));

    // Feed forward speed in cases where the waypoint is moving, instead of arriving with V=0 arrive with V=V_wp
    if (nav_moving_base.mode == NAV_MOVING_BASE_MODE_TRACKING || 
        nav_moving_base.mode == NAV_MOVING_BASE_MODE_DESCEND) {
      VECT2_ADD(max_speed_decel, wp_speed_control_rf);
    }

    struct FloatVect2 max_speed_h;
    max_speed_h.x = max_speed_decel.y * cos_psi - max_speed_decel.x * sin_psi;
    max_speed_h.y = max_speed_decel.y * cos_psi + max_speed_decel.x * sin_psi;

    // Enforce deceleration limits
    Bound(speed_sp.x, -max_speed_h.x, max_speed_h.x);
    Bound(speed_sp.y, -max_speed_h.y, max_speed_h.y);
  }

  // Enforce general horizontal speed limits
  enforce_horizontal_bounds(&speed_sp, nav_moving_base.max_speed_h);
  VECT2_COPY(nav.speed, speed_sp);
}

// Tracking of target while descending/ascending
static void nav_moving_base_descend(void) {
  // Horizontal navigation during descent is the same as during tracking
  nav_moving_base_track();

  // Vertical navigation, hardcoded descend speed or make some kind of reference?
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  float pos_error_v = nav.target.z - pos->z;
  float speed_sp_v = pos_error_v * nav_moving_base.pos_gain.z;
  Bound(speed_sp_v, nav_moving_base.max_speed_v.y, nav_moving_base.max_speed_v.x);

  nav.target.z = nav_moving_base.pos.z;
  nav.speed.z = speed_sp_v;
  
}

static void nav_moving_base_land(void) {
  // Set the nav values based on the values
  struct EnuCoor_f vel = nav_moving_base.speed;
  struct EnuCoor_f accel = nav_moving_base.accel;

  enforce_horizontal_bounds(&vel, nav_moving_base.max_speed_h);
  enforce_horizontal_bounds(&accel, nav_moving_base.max_accel_h);

  VECT3_COPY(nav.target, nav_moving_base.pos);
  VECT3_COPY(nav.speed, vel);
  VECT3_COPY(nav.accel, accel);
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

  quantity->x = quantity_control_rf.x * cos_psi - quantity_control_rf.y * sin_psi;
  quantity->y = quantity_control_rf.x * sin_psi + quantity_control_rf.y * cos_psi;
}
