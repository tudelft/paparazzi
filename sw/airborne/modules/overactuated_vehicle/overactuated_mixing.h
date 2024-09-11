/*
 * Copyright (C) 2015 C. De Wagter
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/overactuated_vehicle/overactuated_vehicle.h"
 * @author Alessandro Mancinelli
 * Control laws for Overactuated Vehicle
 */

#ifndef OVERACTUATED_MIXING_H
#define OVERACTUATED_MIXING_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

/* External used functions */
extern void overactuated_mixing_init(void);
extern void overactuated_mixing_run(void);

#endif