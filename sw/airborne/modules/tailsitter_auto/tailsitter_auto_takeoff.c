/*
 * Copyright (C) 2023 Yamac Birol
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

#include "generated/airframe.h"
#include "state.h"
#include <stdio.h>
#include <math.h>
#include "math/pprz_algebra_float.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/autopilot_static.h"
#include "autopilot.h"
#include "modules/actuators/actuators.h"

#ifndef RADIO_THROTTLE
#define RADIO_THROTTLE   0
#endif

int16_t stage = 0;
int16_t counter = 0;
float Kq = TAKEOFF_Q_GAIN;

float t_scale_to_thrust = 3.5;
float theta_d;

#define TAKEOFF_MODULE_FREQ 500

//int16_t pwm2pprz(float pwm);
int16_t take_off_stage(float theta);
int16_t take_off_thrust(void);
void take_off_enter(void);

void take_off_enter(void){
  stage = 0;
  counter = 0;
}

int16_t take_off_stage(float theta){
  counter++;
  if(autopilot.mode == AP_MODE_NAV){
    if (!autopilot_get_motors_on()) {
      stage = 0;
      counter = 0;
    }
    else if(stage == 0 && counter/TAKEOFF_MODULE_FREQ > 1.0){
      stage = 1;
      counter = 0;
    }
    else if(stage == 1 && theta==theta_d && counter/TAKEOFF_MODULE_FREQ > 10.0 ){
      stage = 2;
      counter = 0;
    }
  } else {
    counter = 0;
    if(radio_control.values[RADIO_PIVOT_SWITCH] < -4500 ){
      stage = 0;
    }
    else if(radio_control.values[RADIO_PIVOT_SWITCH] < 4500){
      stage = 1;
    }
    else if(radio_control.values[RADIO_PIVOT_SWITCH] > 4500){
      stage = 2;
    }
  }
  return stage;
}
