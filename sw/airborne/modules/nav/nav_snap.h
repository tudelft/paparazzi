/*
 * Copyright (C) 2023  MAVLab
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
 * @file modules/nav/nav_snap.h
 *
 * Feed-forward a hardcoded minium snap trajectory.
 */

#ifndef NAV_SNAP_H
#define NAV_SNAP_H

#include "std.h"
#include <stdio.h>

// Settings
extern float min_snap_alpha;
extern float min_snap_a_ff;
extern float min_snap_v_ff;
extern float min_snap_pos_gain;
extern float min_snap_speed_gain;
extern int min_snap_abi;

// Functions
extern void nav_snap_init(void);
extern bool nav_snap_x0(int _wp);
extern bool nav_snap_run(void);

// Logging
extern void min_snap_log_header(FILE *file);
extern void min_snap_log_data(FILE *file);

#endif /* NAV_SNAP_H */
