/*
 * Copyright (C) 2026 TU Delft
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

/** @file modules/audio/orbit_search.c
 *  Acoustic orbit search, see orbit_search.h.
 */

#include "modules/audio/orbit_search.h"
#include "modules/audio/quiet_listen.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/guidance/energy_ctrl.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include <math.h>

#ifndef V_CTL_GLIDE_RATIO
#define V_CTL_GLIDE_RATIO 8.
#endif

bool orbit_search_done = false;

static float orbit_t0 = 0.0f;
static float orbit_last_t = -1.0f;
static float orbit_dist = 0.0f;
static float orbit_run = 0.0f;
static bool orbit_quiet = false;
static uint8_t arcs_done = 0;

static float orbit_cx = 0.0f;
static float orbit_cy = 0.0f;
static float orbit_target = 0.0f;
static bool orbit_target_set = false;

/* Bearing of the aircraft from the orbit centre: radians, clockwise from
   north, in [0, 2 pi). */
static float orbit_bearing(void)
{
  float b = atan2f(GetPosX() - orbit_cx, GetPosY() - orbit_cy);
  return fmodf(b + 2.0f * M_PI, 2.0f * M_PI);
}

/* Within ORBIT_SEARCH_LISTEN_RADIUS_M of the orbit centre. */
static bool in_listen_zone(void)
{
  float dx = GetPosX() - orbit_cx;
  float dy = GetPosY() - orbit_cy;
  return dx * dx + dy * dy < ORBIT_SEARCH_LISTEN_RADIUS_M * ORBIT_SEARCH_LISTEN_RADIUS_M;
}

/* Signed angle the aircraft is past the target, in [-pi, pi). */
static float past_target(float brg)
{
  return fmodf(brg - orbit_target + 5.0f * M_PI, 2.0f * M_PI) - M_PI;
}

/* True when a listening arc may start here: at the planned start bearing,
   or up to ORBIT_SEARCH_START_WINDOW_DEG past it. If the aircraft is
   already further past, the plan steps on to the next start bearing ahead
   of it instead of waiting a whole lap. The first arc starts wherever the
   aircraft is and fixes the sequence. The circle is flown clockwise
   (positive radius), so bearings increase. */
static bool at_start_bearing(void)
{
  float brg = orbit_bearing();
  float step = RadOfDeg(ORBIT_SEARCH_STEP_DEG);
  float win = RadOfDeg(ORBIT_SEARCH_START_WINDOW_DEG);
  if (!orbit_target_set) {
    orbit_target = brg;
    orbit_target_set = true;
  }
  float d = past_target(brg);
  for (uint8_t guard = 0; d >= win && guard < 12; guard++) {
    orbit_target = fmodf(orbit_target + step, 2.0f * M_PI);
    d = past_target(brg);
  }
  if (d < 0.f || d >= win) { return false; }
  orbit_target = fmodf(orbit_target + step, 2.0f * M_PI);
  return true;
}

void orbit_search_reset(uint8_t centre_wp, uint8_t drop_wp)
{
  quiet_listen_off();
  orbit_t0 = get_sys_time_float();
  orbit_last_t = -1.0f;
  orbit_dist = 0.0f;
  orbit_run = 0.0f;
  orbit_quiet = false;
  orbit_search_done = false;
  arcs_done = 0;
  orbit_cx = waypoints[centre_wp].x;
  orbit_cy = waypoints[centre_wp].y;
  orbit_target_set = false;
  waypoints[drop_wp].x = waypoints[centre_wp].x;
  waypoints[drop_wp].y = waypoints[centre_wp].y;
}

/* A listening arc only starts once the climb-back has actually reached
   height (within ORBIT_SEARCH_REGAIN_TOL), and ends early if the glide
   reaches floor_m. A fixed 50 m of power is not enough on its own: the
   climb is capped near 2 m/s, so it regained ~9 m per cycle against
   ~10.5 m lost, and a SITL flight ratcheted from 45 m down to 20 m over
   nine arcs - under the floor, which only limits the COMMANDED height and
   cannot stop a glide with the motor off. */
void orbit_search_update(uint8_t n_arcs, float height, float floor_m, float listen_thr)
{
  float now = get_sys_time_float();
  v_ctl_auto_airspeed_setpoint = ORBIT_SEARCH_AIRSPEED;
  if (orbit_search_done) { return; }
  quiet_listen_session_hold();

  /* Listening ends after n_arcs completed arcs, or at ORBIT_SEARCH_MAX_S. */
  if ((!orbit_quiet && arcs_done >= n_arcs) || now - orbit_t0 >= ORBIT_SEARCH_MAX_S) {
    quiet_listen_off();
    orbit_quiet = false;
    orbit_search_done = true;
    return;
  }

  float dt = now - orbit_last_t;
  if (orbit_last_t < 0.f || dt >= 0.5f) { dt = 0.0f; }
  orbit_last_t = now;
  float ds = stateGetHorizontalSpeedNorm_f() * dt;
  orbit_dist += ds;

  float h = GetPosAlt() - GetAltRef();
  if (orbit_quiet) {
    orbit_run += ds;
    if (orbit_dist >= ORBIT_SEARCH_QUIET_M || h < floor_m || !in_listen_zone()) {
      orbit_quiet = false;
      orbit_dist = 0.0f;
      arcs_done++;
      quiet_listen_off();
    } else {
      quiet_listen_on(listen_thr);
    }
  } else if (orbit_dist >= ORBIT_SEARCH_POWER_M &&
             h >= height - ORBIT_SEARCH_REGAIN_TOL &&
             in_listen_zone() &&
             at_start_bearing()) {
    orbit_quiet = true;
    orbit_dist = 0.0f;
    orbit_run = 0.0f;
    quiet_listen_on(listen_thr);
  }
}

/* Level at the orbit height when powered; the natural glide line on a
   listening arc, where commanding level flight would only fight a motor
   that is off. Floored so a lagging climb-back cannot ratchet it down. */
float orbit_search_alt(float height, float floor_m)
{
  if (!orbit_quiet) { return GetAltRef() + height; }
  float h = height - orbit_run / (float)V_CTL_GLIDE_RATIO;
  if (h < floor_m) { h = floor_m; }
  return GetAltRef() + h;
}
