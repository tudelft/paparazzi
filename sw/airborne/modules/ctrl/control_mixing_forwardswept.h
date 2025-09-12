/*
 * Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/ctrl/control_mixing_heewing.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control mixing specific to the Heewing T1 Ranger
 */

#ifndef CONTROL_MIXING_FORWARDSWEPT_H
#define CONTROL_MIXING_FORWARDSWEPT_H

#include "stdbool.h"

extern void control_mixing_forwardswept_init(void);

/** Stabilization in attitude direct mode
 */
extern void control_mixing_forwardswept_attitude_direct_enter(void);
extern void control_mixing_forwardswept_attitude_direct_run(void);

/** Stabilization in rate mode
 */
extern void control_mixing_forwardswept_rate_enter(void);
extern void control_mixing_forwardswept_rate_run(void);

/** Nav mode
 */
extern void control_mixing_forwardswept_nav_enter(void);
extern void control_mixing_forwardswept_nav_run(void);

#endif  // CONTROL_MIXING_FORWARDSWEPT_H
