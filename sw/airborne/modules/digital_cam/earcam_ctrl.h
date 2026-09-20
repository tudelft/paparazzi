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

/** @file modules/digital_cam/earcam_ctrl.h
 *  @brief EARcam acoustic loud-spot search over the MORA camera link.
 *
 * Periodically sends EARcam-targeted shoots through digital_cam_uart during a
 * survey, asks MORA to finalize, and receives the loudest-spot result so the
 * flight plan can move a waypoint onto it.
 */

#ifndef DIGITAL_CAM_EARCAM_H
#define DIGITAL_CAM_EARCAM_H

#include "std.h"

extern void earcam_init(void);
extern void earcam_periodic(void);

/** Latest loudest-spot result received from MORA. */
extern bool earcam_result_valid;
/** True from the reply to the latest solve/stop until it is consumed by refine_setup/update. */
extern bool earcam_result_fresh;
extern float earcam_lat_deg;
extern float earcam_lon_deg;
extern float earcam_agl_m;
extern float earcam_alt_m;
extern float earcam_level_db;
extern float earcam_confidence;
extern uint16_t earcam_samples;

/** Sampling period used by earcam_start(), seconds; GCS adjustable. */
extern float earcam_period_s;

/** Clear the last result and start periodic EARcam shoots. */
extern uint8_t earcam_start(void);
/** Stop periodic shoots and ask MORA to compute the loudest spot. */
extern uint8_t earcam_stop(void);
/** Ask MORA for an interim result while sampling continues (refinement). */
extern uint8_t earcam_solve(void);
/** Forget the last result. */
extern uint8_t earcam_result_clear(void);
/** Move waypoint lat/lon to the result, keeping the waypoint altitude. Returns 0 on success. */
extern uint8_t earcam_result_to_waypoint(uint8_t wp_id);
/** Place wp_start distance_m short of wp_target, opposite to the current course (straight run-in). */
extern uint8_t earcam_place_run_in(uint8_t wp_start, uint8_t wp_target, float distance_m);
/** Same, run-in flown on the given course (rad, from north clockwise), e.g. wind_circle_upwind_course(). */
extern uint8_t earcam_place_run_in_course(uint8_t wp_start, uint8_t wp_target, float distance_m, float course);

/** Obstacle test in flight plan coordinates (m east, m north), e.g. a flight plan sector. */
typedef bool (*earcam_obstacle_fn)(float x, float y);
/** Course (deg) chosen by the last earcam_place_run_in_lane() and whether its corridor was free. */
extern float earcam_run_in_course_deg;
extern bool earcam_run_in_clear;
/**
 * Place wp_start before_m short of wp_target on the first obstacle-free course out of
 * lane_deg, lane_deg +- step_deg, ... up to +- max_tilt_deg (the sign nearer to
 * preferred_course, rad, is tried first). A course is free when no point of the corridor
 * from before_m before to after_m past the target, on the centre line and margin_m to either
 * side (5 m steps), is an obstacle. Returns true when such a course exists; otherwise the
 * run-in is laid on lane_deg and false is returned (earcam_run_in_clear false).
 */
extern bool earcam_place_run_in_lane(uint8_t wp_start, uint8_t wp_target, float before_m, float after_m,
                                     float margin_m, float lane_deg, float max_tilt_deg, float step_deg,
                                     float preferred_course, earcam_obstacle_fn obstacle);
/**
 * Place the climb-out on the run-in course (wp_start to wp_target) past the target: wp_climbout as far as
 * max_m, but only while the exit circle that would follow (centre turn_radius_m to the side, +1 right / -1
 * left, of the climb-out end) stays turn_radius_m + margin_m clear of keep_out all around; never closer
 * than min_m. wp_exit = that circle centre. Both get altitude alt. Returns the climb-out length in m.
 */
extern float earcam_place_climbout(uint8_t wp_start, uint8_t wp_target, uint8_t wp_climbout, uint8_t wp_exit,
                                   float max_m, float min_m, float turn_radius_m, float margin_m, float side,
                                   float alt, earcam_obstacle_fn keep_out);

/*
 * Adaptive star refinement, fixed-wing flyable: straight measurement legs
 * through the current estimate from several headings, joined by circle turns
 * tangent to both legs (fillets). Legs end at the tangent points, so every pass
 * over the estimate is a straight, wings-level run-in. Their intersection fixes
 * the 2-D position; iterate until estimates agree.
 */
/** Minimum leg half-length in m from the center. Extended to R/tan(90/N) when the
 *  turn circle of radius R = |nav_radius| needs it; if longer, the circle grows. */
extern float earcam_refine_half_length_m;
/** Convergence threshold in m between successive estimates. */
extern float earcam_refine_converge_m;
/** Maximum refinement iterations. */
extern uint8_t earcam_refine_max_iterations;
/** Number of star legs (2..8). */
extern uint8_t earcam_refine_legs;
/** Current iteration (0 = none yet) and shift of the last estimate in m. */
extern uint8_t earcam_refine_iteration;
extern float earcam_refine_shift_m;
/** True when the last result moved less than earcam_refine_converge_m or the
 *  iteration limit is reached. */
extern bool earcam_refine_converged;

/** Center a star on the current result and place leg 0 in wp_from/wp_to and the
 *  following turn circle in wp_turn. Returns 1 when there is no valid result. */
extern uint8_t earcam_refine_setup(uint8_t wp_from, uint8_t wp_to, uint8_t wp_turn);
/** Advance to the next leg, updating the three waypoints. Returns 1 when the star is complete. */
extern uint8_t earcam_refine_next_leg(uint8_t wp_from, uint8_t wp_to, uint8_t wp_turn);
/** True while circling wp_turn and the heading is not yet aligned with the next leg. */
extern bool earcam_refine_turning(void);
/** Signed circle radius for the current turn (sign gives turn direction). */
extern float earcam_refine_turn_radius(void);
/** After a star: update convergence state from the newest result. */
extern uint8_t earcam_refine_update(void);

/** Turn circle radius for the star in m; 0 = use |nav_radius| (default). */
extern float earcam_refine_turn_radius_m;
/** Leg heights above ground_alt (fixed-wing only). When earcam_refine_height_from_m is
 *  0 the waypoint altitudes from the flight plan are kept. Otherwise leg start (and
 *  turn circle) get height_from, leg end gets height_to (a gliding leg descends over
 *  the estimate), both lowered by height_step per completed star, not below height_min. */
extern float earcam_refine_height_from_m;
extern float earcam_refine_height_to_m;
extern float earcam_refine_height_step_m;
extern float earcam_refine_height_min_m;

/*
 * Known search area (IMAV Mission 4: the alarm lies within 25 m of a given point).
 */
/** Seed the result with a waypoint so a star can be flown around it without a survey.
 *  Call after earcam_start(). Returns 0. */
extern uint8_t earcam_result_from_waypoint(uint8_t wp_id);
/** True when a valid result lies within radius_m of wp_id. */
extern bool earcam_result_within(uint8_t wp_id, float radius_m);
/** Move wp_id to the result, projecting outside estimates 1 cm inside the circle.
 *  Preserves altitude and the raw result. Returns 1 without moving on invalid input
 *  or when the projected position exceeds fixed-wing HOME waypoint limits. */
extern uint8_t earcam_result_to_waypoint_in_circle(uint8_t wp_id, uint8_t wp_center, float radius_m);

/*
 * Quiet sampling (fixed-wing): a pusher propeller masks the alarm, so samples are
 * only taken while the throttle is killed for at least earcam_quiet_delay_s (prop
 * spin-down). Flight plan: kill the throttle on the legs, restore it in the turns.
 */
extern bool earcam_quiet_only;
extern float earcam_quiet_delay_s;
/** True when a sample would be taken now (for telemetry/debug). */
extern bool earcam_quiet_now;
/** Block pre_call for a star leg: kill the throttle from kill_before_m before the
 *  centre to restore_after_m past it (a short glide over the estimate), motor
 *  otherwise. Fixed-wing only. */
extern uint8_t earcam_refine_leg_throttle(float kill_before_m, float restore_after_m);

/*
 * Low release (fixed-wing). The approach, release point and hatch are the nav_drop
 * module (nav_drop_compute_approach / nav_drop_update_release / nav_drop_shoot, see
 * include_obc2014_mission.xml). This adds what a 2 m release needs on top:
 *  - a level-off point on the run-in: glide (vmode glide, correct pre-climb) from the
 *    START height to earcam_drop_release_agl_m at wp_level, then hold that height to
 *    the release point, on the rangefinder when EARCAM_USE_AGL_DIST and in range;
 *  - a release gate: no release above earcam_drop_max_agl_m (rule: 2 m, else no points);
 *  - a go-around test when the rangefinder shows less than EARCAM_DROP_ABORT_AGL_M.
 */
extern float earcam_drop_release_agl_m;    ///< release height above ground, m
extern float earcam_drop_max_agl_m;        ///< no release above this height, m
extern float earcam_drop_agl_m;            ///< height above ground used by the last call, m
extern bool earcam_drop_released;          ///< hatch commanded at the last earcam_drop_shoot()
extern bool earcam_drop_missed;            ///< release refused (too high) at the last earcam_drop_shoot()
extern uint8_t earcam_drop_attempts;       ///< releases commanded since earcam_start()

/** Place wp_level level_m before wp_release on the wp_start -> wp_release line at the
 *  release height over WaypointAlt(wp_target). Call after nav_drop_compute_approach(). */
extern uint8_t earcam_drop_level_point(uint8_t wp_start, uint8_t wp_release, uint8_t wp_target,
                                       uint8_t wp_level, float level_m);
/** Altitude setpoint holding the release height over ground level WaypointAlt(wp_target),
 *  referenced to the rangefinder when it is valid. */
extern float earcam_drop_altitude(uint8_t wp_target);
/** True when the rangefinder shows less than EARCAM_DROP_ABORT_AGL_M (go around). */
extern bool earcam_drop_too_low(uint8_t wp_target);
/** Release gate: opens the hatch through nav_drop_shoot() when the height above ground
 *  is at most earcam_drop_max_agl_m (EARCAM_USE_NAV_DROP) and returns 0; otherwise sets
 *  earcam_drop_missed and returns 1. Use it in the exception that detects crossing the
 *  release point so the hatch opens in the same navigation cycle. */
extern uint8_t earcam_drop_shoot(uint8_t wp_target);
/** Diagnostic: move wp_mark to where nav_drop expects the kit to land when released now
 *  (position after nav_drop_trigger_delay plus the target-release offset). Downlink it with
 *  DownlinkSendWpNr(wp_mark) to compare against the truth in simulation or a GPS fix. */
extern uint8_t earcam_drop_mark_impact(uint8_t wp_target, uint8_t wp_release, uint8_t wp_mark);

#endif // DIGITAL_CAM_EARCAM_H
