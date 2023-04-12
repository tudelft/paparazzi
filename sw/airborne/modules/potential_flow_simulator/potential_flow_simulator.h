/*
 * Copyright (C) Sunyou Hwang <S.Hwang-1@tudelft.nl>
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

/** @file "modules/potential_flow_simulator/potential_flow_simulator.h"
 * @author Sunyou Hwang <S.Hwang-1@tudelft.nl>
 * Potential flow simulator for orographic soaring (Never Landing Drone project)

 */

#ifndef POTENTIAL_FLOW_SIMULATOR_H
#define POTENTIAL_FLOW_SIMULATOR_H

#include "std.h"

// wind speed m/s, dir degree CW from north
extern float ref_wind_spd;
extern float ref_wind_dir;

extern float obstacle_radius;
extern float obstacle_position_north;
extern float obstacle_position_east;
extern float obstacle_position_down;

extern void init_potential_flow_simulator(void);
extern void potential_flow_simulator_periodic(void);

//extern void parse_ground_gps(uint8_t *buf);




#endif  // POTENTIAL_FLOW_SIMULATOR_H
