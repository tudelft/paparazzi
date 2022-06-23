/*
 * Copyright (C) 2022 Tomaso De Ponti <tomasodp@gmail.com>
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

/** @file "modules/guidance/ctrl_interface.h"
 * @author Tomaso De Ponti <tomasodp@gmail.com>
 * This module is used to have a clean interface with sliders and booleans to gradually implement control changes that could jeopardize drone control.
 */

#ifndef CTRL_INTERFACE_H
#define CTRL_INTERFACE_H
#include "std.h"
// Booleans definition
extern bool separate_TL; //Boolean to activate Estimation of Thrust based on Lift Model
extern bool better_lpe ; //Boolean to activate Lift Pitch Effectiveness based on Lift Model
extern bool better_ail ; //Boolean to activate scheduling of aileron eff based on skew model
extern bool manual_eff ; //Boolean to set effectiveness values manually
extern bool better_eff ; //Boolean to switch complelty to new effectiveness values
extern bool motor_free ; //Bolean to turn off motors above a certain airspeed
// Values for sliders definition
extern float roll_ail_L_eff  ;  //Effectiveness value x10-3
extern float roll_ail_R_eff  ;  //Effectiveness value x10-3
extern float pitch_ail_L_eff ;  //Effectiveness value x10-3
extern float pitch_ail_R_eff ;  //Effectiveness value x10-3
extern float pitch_elev_eff  ;  //Effectiveness value x10-3
extern float yaw_rud_eff     ;  //Effectiveness value x10-3
extern float u_motor_free    ;  //Airspeed limit in [m/s]

extern void interface_init(void);

#endif  // CTRL_INTERFACE_H
