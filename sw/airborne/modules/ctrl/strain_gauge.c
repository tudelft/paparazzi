/*
 * Copyright (C) 2021 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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
 * @file "modules/ctrl/approach_moving_target.h"
 * @author Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * Approach a moving target (e.g. ship)
 */
 
#ifndef STRAIN_GAUGE
#define STRAIN_GAUGE 0
#endif
 
void strain_gauge_init(void) {
// initializing the strain gauge 
};

void strain_gauge_periodic(void) {
// reading and processing the values from the strain gauge
};

void strain_gauge_event(void) {
// data coming in as event - here function for motor 
};

