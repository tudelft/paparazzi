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
 * @file "modules/ship_landing/core_nav_approach_ship.h"
 * @author Alessandro Mancinelli (a.mancinelli@tudelft.nl)
 * Nav approach ship module
 */

#ifndef CORE_NAV_APPROACH_SHIP_H
#define CORE_NAV_APPROACH_SHIP_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

struct ship_msg { 
    float timestamp; 
    float phi; 
    float theta; 
    float psi; 
    float phi_dot; 
    float theta_dot; 
    int32_t lat; 
    int32_t lon; 
    int32_t alt;     
    float x_dot; 
    float y_dot; 
    float z_dot; 
};

extern void nav_approach_ship_init(void); 
extern void nav_approach_ship_run(void);

extern void nav_approach_ship_parse_SHIP_INFO_MSG(uint8_t *buf);
extern void nav_approach_ship_parse_SHIP_PREDICTION_MSG(uint8_t *buf);


//Extra variables to be accessed from outside or from sliders: 

#endif // CORE_NAV_APPROACH_SHIP_H