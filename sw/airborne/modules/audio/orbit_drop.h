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

/** @file modules/audio/orbit_drop.h
 *  Site model, drop geometry and payload hatch for the orbit search drop
 *  (tudelft_include_imav2026_mission4_orbit.xml).
 *
 *  Site model: the flight plan's tree sectors M4_OW, M4_OE, M4_ON and the main
 *  plan's Geofence sector, as the obstacle callbacks for
 *  earcam_place_run_in_lane() and earcam_place_climbout(). They are only built
 *  when the active flight plan has those sectors.
 */

#ifndef ORBIT_DROP_H
#define ORBIT_DROP_H

#include "std.h"

/* Run-in lane: course flown over the target (deg, from north), max tilt
   and tilt step tried, corridor half width kept clear of the sectors, and
   the side (+1 right, east over the meadow) of the base turn circle. */
#ifndef ORBIT_DROP_LANE_COURSE_DEG
#define ORBIT_DROP_LANE_COURSE_DEG 0.f
#endif
#ifndef ORBIT_DROP_LANE_MAX_TILT_DEG
#define ORBIT_DROP_LANE_MAX_TILT_DEG 20.f
#endif
#ifndef ORBIT_DROP_LANE_TILT_STEP_DEG
#define ORBIT_DROP_LANE_TILT_STEP_DEG 2.f
#endif
#ifndef ORBIT_DROP_RUN_IN_M
#define ORBIT_DROP_RUN_IN_M 300.f
#endif
#ifndef ORBIT_DROP_LANE_MARGIN_M
#define ORBIT_DROP_LANE_MARGIN_M 8.f
#endif
#ifndef ORBIT_DROP_DROPTURN_SIDE
#define ORBIT_DROP_DROPTURN_SIDE 1.f
#endif
/* Climb-out straight ahead past the target, then the exit circle. */
#ifndef ORBIT_DROP_CLIMBOUT_MAX_M
#define ORBIT_DROP_CLIMBOUT_MAX_M 120.f
#endif
#ifndef ORBIT_DROP_CLIMBOUT_MIN_M
#define ORBIT_DROP_CLIMBOUT_MIN_M 40.f
#endif
#ifndef ORBIT_DROP_CLIMBOUT_RATE_MPS
#define ORBIT_DROP_CLIMBOUT_RATE_MPS 3.0f
#endif
#ifndef ORBIT_DROP_CLIMBOUT_AIRSPEED_MPS
#define ORBIT_DROP_CLIMBOUT_AIRSPEED_MPS 11.f
#endif
#ifndef ORBIT_DROP_EXIT_TURN_RADIUS_M
#define ORBIT_DROP_EXIT_TURN_RADIUS_M 30.f
#endif
/* The exit circle stays this far inside the geofence: turn-entry overshoot
   plus tailwind, GPS error and the fence's own accuracy. */
#ifndef ORBIT_DROP_FENCE_MARGIN_M
#define ORBIT_DROP_FENCE_MARGIN_M 40.f
#endif
/* Above this height nothing at the site can be hit. The orbit flies here. */
#ifndef ORBIT_DROP_SAFE_HEIGHT_M
#define ORBIT_DROP_SAFE_HEIGHT_M 45.f
#endif
/* Below this height inside an obstacle sector = emergency climb. */
#ifndef ORBIT_DROP_ESCAPE_HEIGHT_M
#define ORBIT_DROP_ESCAPE_HEIGHT_M 25.f
#endif

/** Trees, or outside the geofence: both are places the low run-in corridor may not touch. */
extern bool orbit_drop_obstacle(float x, float y);
/** Outside the geofence. */
extern bool orbit_drop_outside_fence(float x, float y);

/** Payload hatch closed (AUTO2: S_HATCH follows COMMAND_HATCH, AUTO1: the RC
 *  switch). It is opened by nav_drop_shoot() through earcam_drop_shoot(). */
extern void orbit_drop_hatch_close(void);
extern void orbit_drop_hatch_open(void);

#endif /* ORBIT_DROP_H */
