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

/** @file "modules/deviation_from_wp/deviation_from_wp.h"
 * @author Simon Cajagi <Cajagi@student.tudelft.nl>
 * This module logs the distance from a waypoint to help analyse how turbulence affects NAV mode accuracy in different wind fields.
 */

#ifndef DEVIATION_FROM_WP_H
#define DEVIATION_FROM_WP_H

#include "std.h"

// External variables
extern float wp_e;
extern float wp_n;
extern float wp_u;
extern bool send_data;
extern bool automated_test;

// Function declarations
void init_deviation_from_wp(void);
void periodic_deviation_from_wp(void);

#endif  // DEVIATION_FROM_WP_H