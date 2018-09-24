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
 * @file "modules/disco_copter/disco_copter.c"
 * @author Hacker
 * 
 */

#include "modules/disco_copter/disco_copter.h"



struct throttle_curve_t throttle_curve;


void gpio_setup_output(uint32_t port, uint16_t gpios) {}

void gpio_set(uint32_t port, uint16_t pin) {}


void nav_throttle_curve_set(uint8_t mode)
{

}


float get_temp(uint8_t nr)
{
  return 20.5;
}


void disco_copter_init()
{

}


