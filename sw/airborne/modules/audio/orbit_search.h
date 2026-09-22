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

/** @file modules/audio/orbit_search.h
 *  Acoustic orbit search: low-throttle listening arcs on a circle round a point.
 *
 *  A circle of ORBIT_SEARCH_RADIUS round the organisers' point, cycling
 *  ORBIT_SEARCH_QUIET_M of low-throttle descent and ORBIT_SEARCH_POWER_M of
 *  powered climb-back. Listening ends after n_arcs listening arcs (8: about
 *  150 s, every bearing heard from 4-5 arcs), or at ORBIT_SEARCH_MAX_S if the
 *  arcs cannot start. An arc only starts, and only continues, within
 *  ORBIT_SEARCH_LISTEN_RADIUS_M of the centre: the alarm is within 25 m of it,
 *  so nothing is listened to further out. 25 m at 10 m/s is about 22 deg of
 *  bank, below the airframe's declared MIN_CIRCLE_RADIUS of 30 m.
 *
 *  Distance round the orbit is integrated from ground speed in the block's
 *  pre_call, so the cycle does not depend on how the circle was joined. It
 *  starts powered, which gives the aircraft 50 m to settle onto the circle.
 *  EARcam samples on the arcs; the plan requests its result once listening is
 *  done (orbit_search_done).
 *
 *  Sweep: each listening arc starts ORBIT_SEARCH_STEP_DEG further round than
 *  the last, whatever the climb rate, so the arcs cover every bearing (an arc
 *  hears about 218 deg; all bearings are heard after 5 arcs). Left to the
 *  climb-back alone the shift could shrink to ~7 deg and leave a sector
 *  unheard for the whole search.
 *
 *  Flight plan use:
 *    call_once orbit_search_reset(WP_CENTER, WP_DROP), earcam_start(), then a block with
 *    pre_call="orbit_search_update(arcs, height, floor, listen_thr)" flying
 *    circle radius=ORBIT_SEARCH_RADIUS alt=orbit_search_alt(height, floor) round WP_CENTER,
 *    left on orbit_search_done.
 */

#ifndef ORBIT_SEARCH_H
#define ORBIT_SEARCH_H

#include "std.h"

/** Orbit radius, m. */
#ifndef ORBIT_SEARCH_RADIUS
#define ORBIT_SEARCH_RADIUS 25.0f
#endif
/** Distance flown on each low-throttle listening arc, m. */
#ifndef ORBIT_SEARCH_QUIET_M
#define ORBIT_SEARCH_QUIET_M 110.0f
#endif
/** Minimum powered climb-back between arcs, m. */
#ifndef ORBIT_SEARCH_POWER_M
#define ORBIT_SEARCH_POWER_M 50.0f
#endif
/** Airspeed setpoint on the orbit, m/s. */
#ifndef ORBIT_SEARCH_AIRSPEED
#define ORBIT_SEARCH_AIRSPEED 10.0f
#endif
/** Nominal cruise throttle for the climb to orbit height. */
#ifndef ORBIT_SEARCH_CRUISE_THR
#define ORBIT_SEARCH_CRUISE_THR 0.16f
#endif
/** Arcs only start and continue within this distance of the centre, m. */
#ifndef ORBIT_SEARCH_LISTEN_RADIUS_M
#define ORBIT_SEARCH_LISTEN_RADIUS_M 40.0f
#endif
/** Listening ends at the latest after this time, s. */
#ifndef ORBIT_SEARCH_MAX_S
#define ORBIT_SEARCH_MAX_S 300.0f
#endif
/** Shift of the start bearing from one arc to the next, deg. */
#ifndef ORBIT_SEARCH_STEP_DEG
#define ORBIT_SEARCH_STEP_DEG 40.0f
#endif
/** An arc may start up to this far past its start bearing, deg. */
#ifndef ORBIT_SEARCH_START_WINDOW_DEG
#define ORBIT_SEARCH_START_WINDOW_DEG 20.0f
#endif
/** The climb-back must be within this of the orbit height before an arc, m. */
#ifndef ORBIT_SEARCH_REGAIN_TOL
#define ORBIT_SEARCH_REGAIN_TOL 1.0f
#endif

/** TRUE once listening is done; hands over to the EARcam result blocks. */
extern bool orbit_search_done;

/** Reset the cycle round centre_wp and put drop_wp on the centre. */
extern void orbit_search_reset(uint8_t centre_wp, uint8_t drop_wp);

/** Block pre_call: owns the quiet/powered cycle for n_arcs arcs. */
extern void orbit_search_update(uint8_t n_arcs, float height, float floor_m, float listen_thr);

/** Altitude setpoint for the orbit circle. */
extern float orbit_search_alt(float height, float floor_m);

#endif /* ORBIT_SEARCH_H */
