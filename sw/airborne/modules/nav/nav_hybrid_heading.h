/*
 * Copyright (C) MAVLab
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
 * @file "modules/nav/nav_hybrid_heading.h"
 * @author MAVLab
 * Move heading into the wind when hybrids are hovering.
 */

#ifndef NAV_HYBRID_HEADING_H
#define NAV_HYBRID_HEADING_H

// Align with wind
extern void nav_hybrid_heading_init(void);
extern void nav_hybrid_heading_set(void);
extern void nav_hybrid_heading_set_l(void);
extern void nav_hybrid_heading_set_r(void);

// Turn to waypoint
extern void nav_hybrid_heading_init_to_waypoint(int wp);
extern void nav_hybrid_heading_set_to_waypoint(void);

// Module
extern void nav_hybrid_heading_periodic(void);

extern float nav_hybrid_heading_max_yaw_rate;
extern float nav_hybrid_heading_max_yaw_rate_to_wp;
extern float nav_hybrid_heading_pitch_deadband;
extern float nav_hybrid_heading_tip_in_wind;


#endif

