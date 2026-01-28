/*
 * Copyright (C) 2026 Evangelos Ntouros <e.ntouros@tudelft.nl>
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

/** @file "modules/flatness/flatness_guidance.h"
 * @author Evangelos Ntouros <e.ntouros@tudelft.nl>
 * Differential flatness based guidance.
 */

#ifndef FLATNESS_GUIDANCE_H
#define FLATNESS_GUIDANCE_H

extern struct ThrustSetpoint thrust_sp;

extern void flatness_guidance_init(void);
extern struct ThrustSetpoint get_thrust(void);

#endif  // FLATNESS_GUIDANCE_H
