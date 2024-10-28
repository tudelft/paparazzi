/*
 * Copyright (C) 2024 Alessandro Mancinelli <alessandro.mancinelli@outlook.com>
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
 * @file "modules/ground_detect/ground_detect_am7.h"
 * @author Alessandro Mancinelli <alessandro.mancinelli@outlook.com>
 * Detection of the ground using the AM7 device
 */

#ifndef GROUND_DETECT_AM7_H
#define GROUND_DETECT_AM7_H

#include "std.h"
#include <stdarg.h>

//Variables for the slider
extern float min_lidar_alt;
extern float time_tolerance_land;
extern float az_tolerance_land;
extern float min_lidar_alt_ground_detect; 

extern uint8_t detect_ground_on_landing(void); 
extern void detect_ground_on_landing_am7_init(void);

#endif // GROUND_DETECT_AM7_H