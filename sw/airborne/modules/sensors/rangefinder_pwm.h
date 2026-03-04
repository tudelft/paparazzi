/*
* Copyright (C) 2020 OpenuAS
*
* Thanks to Jean-François Erdelyi & Gautier Hattenberger for ADC one
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
* @file "modules/sensors/rangefinder_pwm.h"
* @author OpenUAS
* @brief Driver for Rangefinder sensors that output a PWM signal
*
* Reads a sensor using PWM input and outputs distance to object in meters
*
* Sensor example: Maxbotix EZ1
* https://www.maxbotix.com/033-using-pulse-width-pin-2.htm
*
*/

#ifndef RANGEFINDER_PWM_H
#define RANGEFINDER_PWM_H

#include "std.h"

struct RangefinderPwm {
  uint16_t raw;   ///< raw PWM value
  float offset;   ///< offset to zero distance in meters
  float scale;    ///< scale to convert raw to a real distance
  float distance; ///< Distance measured
};

extern struct RangefinderPwm rangefinder_pwm; // Contains the read rangefinder sensor data

extern void rangefinder_pwm_init(void);
extern void rangefinder_pwm_read(void);

#endif /* RANGEFINDER_PWM_H */

