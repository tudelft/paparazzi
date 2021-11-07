/*
 * Copyright (C) 2021 Dennis van Wijngaarden <D.C.vanWIjngaarden@tudelft.nl>
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

/** @file modules/ctrl/ctrl_effectiveness_scheduling_rotating_wing_drone.h
 * Module that interpolates gainsets in flight based on the transition percentage
 */

#ifndef CTRL_EFFECTIVENESS_SCHEDULING_ROTATING_WING_DRONE_H
#define CTRL_EFFECTIVENESS_SCHEDULING_ROTATING_WING_DRONE_H

#include "stdint.h"

// Define module settings
extern uint8_t rot_wing_angle_deg;

// Define module functions
extern void ctrl_eff_scheduling_rotating_wing_drone_init(void);
extern void ctrl_eff_scheduling_rotating_wing_drone_periodic(void);

#endif