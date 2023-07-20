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

int16_t stage = 0;
int16_t counter = 0;
float Kq = TAKEOFF_Q_GAIN;

#define TAKEOFF_MODULE_FREQ 500

int16_t pwm2pprz(float pwm);
int16_t take_off_stage(float theta);
int16_t take_off_thrust(void);
void take_off_enter(void);

void take_off_enter(void){
  stage = 0;
  counter = 0;
}

int16_t take_off_thrust(void){
  float thrust_pwm=1000;
  int16_t thrust_pprz=0;
  struct FloatEulers eulers_zxy;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  struct FloatRates * body_rates = stateGetBodyRates_f();
  float_eulers_of_quat_zxy(&eulers_zxy, statequat);

  if(autopilot.mode == AP_MODE_NAV){
    if(stage == 0){
      thrust_pwm = 1000;
      thrust_pprz = pwm2pprz(thrust_pwm);
    } else if(stage == 1){
      int16_t pitch_rate_cont = (int16_t) ceil(Kq*(0.3 - body_rates->q));
      thrust_pprz = 6000 - pitch_rate_cont + counter/2;
    } else{ // stage 2 (thrust not used actually?)
      thrust_pwm = 1542;
      thrust_pprz = pwm2pprz(thrust_pwm);
    }
  }
  else{
    thrust_pprz = radio_control.values[RADIO_THROTTLE];
  }
  return thrust_pprz;
}

int16_t take_off_stage(float theta){
  counter++;
  if(autopilot.mode == AP_MODE_NAV){
    if (!autopilot_get_motors_on()) {
      stage = 0;
      counter = 0;
    }
    if(stage == 0 && counter/TAKEOFF_MODULE_FREQ > 1.0){
      stage = 1;
      counter = 0;
    }
    else if(stage == 1 && fabsf(theta) < RadOfDeg(5.0)){
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
int16_t pwm2pprz(float pwm){
  int16_t pprz_cmd;
  pprz_cmd =  9600/900*pwm - 10666.67; //(int16_t)ceil((pwm - [1560 1350 1150 1150 1500 1500])./[-600 600 750 750 400 400]*9600);
  return pprz_cmd;
}
