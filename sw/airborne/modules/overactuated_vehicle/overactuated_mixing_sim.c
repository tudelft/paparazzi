/*
 * Copyright (C) 2021 A. Mancinelli
 *
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
 * @file "modules/overactuated_vehicle/overactuated_vehicle.c"
 * @author Alessandro Mancinelli (a.mancinelli@tudelft.nl)
 * Control laws for Overactuated Vehicle
 */
#include "generated/airframe.h"
#include "overactuated_mixing.h"



/**
 * Function which detects ground
 * IT ONLY WORKS WITH THE NONLINEAR CONTROLLER!!! 
 */
uint8_t detect_ground_on_landing(void){

    return 0;
}


int approach_state = 0;
