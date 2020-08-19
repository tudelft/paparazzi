/*
 * Copyright (C) 2017 Ewoud Smeur <ewoud_smeur@msn.com>
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

#include "modules/ctrl/motor_vs_flap_slider.h"
#include "subsystems/actuators/motor_mixing.h"
#include "generated/airframe.h"
#include "state.h"
#include "autopilot.h"

int32_t pitch_coef_orig[MOTOR_MIXING_NB_MOTOR] = MOTOR_MIXING_PITCH_COEF;
int32_t yaw_coef_orig[MOTOR_MIXING_NB_MOTOR] = MOTOR_MIXING_YAW_COEF;

float pitch_slider = 1.0;
float yaw_slider = 1.0;

void motor_vs_flap_slider_init(void)
{
}

void motor_vs_flap_slider_periodic(void)
{
  float pitch_gain = 1.0;
  float yaw_gain = 1.0;

  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  float pitch_deg = DegOfRad(eulers_zxy.theta);
  float airspeed = stateGetAirspeed_f();

  if (airspeed > 15.0 &&
      pitch_deg < -60.0 &&
      autopilot.mode != AP_MODE_FORWARD &&
      autopilot.mode != AP_MODE_ATTITUDE_DIRECT)
  {
    pitch_gain = pitch_slider;
    yaw_gain = yaw_slider;
  }

  int i = 0;
  for (i = 0; i < MOTOR_MIXING_NB_MOTOR; i++) {
    pitch_coef[i] = (int32_t) (pitch_coef_orig[i] * pitch_gain);
    yaw_coef[i] = (int32_t) (yaw_coef_orig[i] * yaw_gain);
  }
}

