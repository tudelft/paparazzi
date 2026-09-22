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
 *  @brief EARcam acoustic loud-spot search over the MORA camera link.
 *
 * Periodically sends EARcam-targeted shoots through digital_cam_uart during a
 * survey, asks MORA to finalize, and receives the loudest-spot result so the
 * flight plan can move a waypoint onto it.
 */

#ifndef DIGITAL_CAM_HOME_VECTOR_H
#define DIGITAL_CAM_HOME_VECTOR_H

#include "std.h"

extern void home_vector_init(void);
extern void home_vector_periodic(void);

extern bool home_vector_result_valid;
extern bool home_vector_result_fresh;

extern float home_vector_ux_body;
extern float home_vector_uy_body;
extern float home_vector_dist_m;

extern float home_vector_period_s;
extern float home_vector_leg_distance_m;

extern uint8_t home_vector_start(uint8_t wp_id);
extern uint8_t home_vector_stop(void);
extern uint8_t home_vector_result_clear(void);

#endif