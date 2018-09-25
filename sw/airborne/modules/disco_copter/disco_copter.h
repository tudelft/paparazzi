/*
 * Copyright (C) Hacker
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
 * @file "modules/disco_copter/disco_copter.h"
 * @author Hacker
 * 
 */

#ifndef DISCO_COPTER_H
#define DISCO_COPTER_H


#define INTERMCU_SET_CMD_STATUS(X) {}
#define INTERMCU_CMD_FAILSAFE 0

#include "modules/helicopter/throttle_curve.h"
#include "modules/sensors/temp_adc.h"

#include "modules/nav/nav_heli_spinup.h"
#include "firmwares/rotorcraft/guidance/guidance_delftacopter.h"
#include "modules/boards/opa_controller_ap.h"


extern void disco_copter_init(void);



#endif

