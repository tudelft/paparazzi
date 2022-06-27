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

/** @file "modules/guidance/ctrl_interface.c"
 * @author Tomaso De Ponti <tomasodp@gmail.com>
 * This module is used to have a clean interface with sliders and booleans to gradually implement control changes that could jeopardize drone control.
 */

#include "modules/guidance/ctrl_interface.h"
#include "stdio.h"

// 
bool separate_TL = false; //Boolean to activate Estimation of Thrust based on Lift Model
bool better_lpe  = false; //Boolean to activate Lift Pitch Effectiveness based on Lift Model
bool better_ail  = false; //Boolean to activate scheduling of aileron eff based on skew model
bool manual_eff  = false; //Boolean to set effectiveness values manually
bool better_eff  = false; //Boolean to switch complelty to new effectiveness values
bool motor_free  = true; //Bolean to turn off motors above a certain airspeed
float u_motor_free    = 16.0   ;  //Airspeed limit in [m/s]
float roll_ail_L_eff  = 0.013  ;  //Effectiveness value x10-3
float roll_ail_R_eff  = 0.013  ;  //Effectiveness value x10-3
float pitch_ail_L_eff = 0.000  ;  //Effectiveness value x10-3
float pitch_ail_R_eff = 0.000  ;  //Effectiveness value x10-3
float pitch_elev_eff  = -0.01  ;  //Effectiveness value x10-3
float yaw_rud_eff     = 0.006  ;  //Effectiveness value x10-3
 void interface_init(void)
{
  // init scheduling
  printf("Starting Control Interface\n");


  // float roll_ail_L_eff  = 0.013  ;  //Effectiveness value x10-3
  // float roll_ail_R_eff  = 0.013  ;  //Effectiveness value x10-3
  // float pitch_ail_L_eff = 0.000  ;  //Effectiveness value x10-3
  // float pitch_ail_R_eff = 0.000  ;  //Effectiveness value x10-3
  // float pitch_elev_eff  = -0.01  ;  //Effectiveness value x10-3
  // float yaw_rud_eff     = 0.006  ;  //Effectiveness value x10-3
}

