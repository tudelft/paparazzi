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
#include "modules/vision_outback/vision_outback.h"



struct throttle_curve_t throttle_curve;
enum hybrid_mode_t dc_hybrid_mode;
void nav_heli_spinup_setup( uint16_t duration __attribute__((unused)), float throttle __attribute__((unused))){}
bool nav_heli_spinup_run(void)
{
  return false;
}

void opa_controller_ap_disarm(bool take __attribute__((unused)) ){
}

void uart8_init(void);
void uart8_init(void)
{
}

bool het_moment = false;
bool vision_timeout = false;
bool vision_outback_enable_landing = false;
bool vision_found_joe = false;


void enableVisionDescent(bool b __attribute__((unused))) {}
void enableVisionFindJoe(bool b __attribute__((unused))) {}





void gpio_setup_output(uint32_t port __attribute__((unused)), uint16_t gpios __attribute__((unused)));
void gpio_setup_output(uint32_t port __attribute__((unused)), uint16_t gpios __attribute__((unused)))
{}

void gpio_set(uint32_t port __attribute__((unused)), uint16_t pin __attribute__((unused)));
void gpio_set(uint32_t port __attribute__((unused)), uint16_t pin __attribute__((unused)))
{}

void nav_throttle_curve_set(uint8_t mode __attribute__((unused)))
{

}


float get_temp(uint8_t nr __attribute__((unused)))
{
  return 20.5;
}


void disco_copter_init()
{
  throttle_curve.rpm_meas = 1300;
}

bool hasFoundGoodJoe(void)
{
	return disco_vision_found_joe;
}


void enableVisionPower(void)
{
  disco_vision_is_on = true;
}

bool enableVisionShutdown(bool b __attribute__((unused)))
{
  disco_vision_is_on = false;
  return true;
}

void killVision(void)
{
  disco_vision_is_on = false;
}

bool getVisionReady(void)
{
  return disco_vision_is_on;
}


uint8_t disco_vision_is_on = false;
uint8_t disco_vision_found_joe = false;





