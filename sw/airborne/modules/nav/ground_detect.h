/*
 * Copyright (C) 2023 Ewoud Smeur
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
 *
 */

/** \file ground_detect.h
 *
 * Ground detection module
 */

#ifndef GROUND_DETECT_H
#define GROUND_DETECT_H

#include "std.h"


extern void ground_detect_init(void);
extern void ground_detect_periodic(void);
extern void ground_detect_filter_accel(void);

extern bool ground_detect(void);

extern bool disarm_on_not_in_flight;

#endif
