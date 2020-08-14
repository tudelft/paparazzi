/*
 * Copyright (C) 2020 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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

/**
 * @file modules/ctrl/flap_wiggle.h
 */

#ifndef FLAP_WIGGLE_H
#define FLAP_WIGGLE_H

#include "stdbool.h"
#include "stdint.h"

extern bool flap_wiggle_state;
extern float flap_wiggle_gain;
extern int32_t wiggle_val[8];

/**
 * Initialises periodic loop;
 */
extern void flap_wiggle_init(void);

/**
 * Periodic function 
 */
extern void flap_wiggle_periodic(void);

#endif  /* FLAP_WIGGLE_H */

