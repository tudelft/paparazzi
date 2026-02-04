/*
 * Copyright (C) 2026
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

/** @file "modules/guidance/guidance_unified.h"
 * A unified guidance module.
 */

#ifndef GUIDANCE_UNIFIED_H
#define GUIDANCE_UNIFIED_H

#include <std.h>

// Settings
extern float pos_ref[3];
extern float vel_ref[3];
extern float accel_ref[3];
extern float accel_ref_with_gains[3];
extern float T;
extern float roll_rate_calc;
extern float pitch_rate_calc;
extern float dcmd[3];
extern struct ThrustSetpoint thr_sp;

extern void guidance_unified_init(void);
extern void guidance_unified_enter(void);
extern void guidance_unified_run(bool in_flight);
extern struct ThrustSetpoint get_thrust(void);

#endif  // GUIDANCE_UNIFIED_H
