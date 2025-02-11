/*
 * Copyright (C) 2025 Elize Alwash <e.alwash@student.tudelft.nl>
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
 * @file "modules/ctrl/strain_gauge.h"
 * @author Elize Alwash <e.alwash@student.tudelft.nl>
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

