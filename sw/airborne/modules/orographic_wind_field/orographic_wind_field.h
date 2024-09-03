/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/orographic_wind_field/orographic_wind_field.h"
 * @author Simon Cajagi <Cajagi@student.tudelft.nl>
 * This module controls the winds in the NPS simulation based on the current position of the aircraft, to simulate updrafts above a dune.
 * The wind field is based on a CSV file storing the CFD simulated flow field over a slope in the wind tunnel.
 * This data is extruded to form an infinite slope along the east-west axis.
 * The wind blows from the north towards the south.
 * Outside of the are described by the CSV file, the wind speed is set to the base wind speed, still coming from the north.
 */

#ifndef OROGRAPHIC_WIND_FIELD_H
#define OROGRAPHIC_WIND_FIELD_H

#include "std.h"

// External variables
extern float base_wind_speed;
extern uint8_t desired_slope_angle;
extern uint8_t desired_turbulence_severity;

// Function declarations
void init_orographic_wind_field(void);
void periodic_orographic_wind_field(void);

#endif  // OROGRAPHIC_WIND_FIELD_H
