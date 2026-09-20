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

/** @file modules/digital_cam/home_vector_ctrl.h
 *  @brief Visual-homing CNN home-vector following through CATIA over the UART link to MORA.
 *
 * Periodically sends AI-cam-targeted shoots through digital_cam_uart at
 * home_vector_period_s; CATIA running with --aicam-home on the MORA companion computer
 * runs each captured frame through the on-sensor IMX500 visual-homing network and
 * replies with a fresh predicted body-frame direction-to-home every shot (unlike earcam,
 * which only replies on an explicit solve). This module rotates that body-frame vector
 * into world frame using the aircraft's current heading and nudges a dedicated HOME
 * waypoint home_vector_leg_distance_m ahead along it, so existing nav/guidance flying
 * toward that waypoint follows the CNN's prediction continuously.
 *
 * Typical flight plan sequence: home_vector_start(WP_HOME) at the start of a homing
 * block, then navigate toward WP_HOME (Goto/Stay) for the block's duration,
 * home_vector_stop() to end it. See digital_cam_earcam's earcam_ctrl.h for the sibling
 * acoustic-search module this one's UART/result-handling plumbing mirrors.
 */

#ifndef DIGITAL_CAM_HOME_VECTOR_H
#define DIGITAL_CAM_HOME_VECTOR_H

#include "std.h"

extern void home_vector_init(void);
extern void home_vector_periodic(void);

/** Latest body-frame prediction received from MORA. */
extern bool home_vector_result_valid;
/** True from the reply to the latest shot until home_vector_periodic() consumes it into
 *  a waypoint nudge. */
extern bool home_vector_result_fresh;
/** Body-frame direction-to-home, forward component (unit vector, dimensionless). */
extern float home_vector_dx_body;
/** Body-frame direction-to-home, right component (unit vector, dimensionless). */
extern float home_vector_dy_body;
/** Predicted distance to home, metres (only as informative as the deployed checkpoint's
 *  own distance-normalization; not currently used to size the waypoint nudge). */
extern float home_vector_dist_m;

/** Shoot period while active, seconds; GCS adjustable. */
extern float home_vector_period_s;
/** Distance the HOME waypoint is placed ahead of the aircraft along the predicted
 *  world-frame heading on every fresh result; GCS adjustable. */
extern float home_vector_leg_distance_m;

/** Waypoint nudged by every fresh result once home_vector_start() has been called. */
extern uint8_t home_vector_wp_id;

/** Clear the last result and start periodic --aicam-home shoots, nudging wp_id. */
extern uint8_t home_vector_start(uint8_t wp_id);
/** Stop periodic shoots. The HOME waypoint is left at its last nudged position. */
extern uint8_t home_vector_stop(void);
/** Forget the last result without stopping sampling. */
extern uint8_t home_vector_result_clear(void);

#endif // DIGITAL_CAM_HOME_VECTOR_H
