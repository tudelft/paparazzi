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

/** @file modules/ctrl/indi_simple_scheduling.c
 * Module that interpolates gainsets in flight based on the transition percentage
 */

#include "modules/ctrl/scheduling_indi_simple.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/radio_control.h"

#include "modules/ctrl/motor_vs_flap_slider.h"

int32_t use_scheduling = 1;

static float g_forward[3] = {STABILIZATION_INDI_FORWARD_G1_P, STABILIZATION_INDI_FORWARD_G1_Q, STABILIZATION_INDI_FORWARD_G1_R};

static float g_hover[3] = {STABILIZATION_INDI_G1_P, STABILIZATION_INDI_G1_Q, STABILIZATION_INDI_G1_R};

//Get the specified gains in the gainlibrary
void ctrl_eff_scheduling_init(void)
{
}

void ctrl_eff_scheduling_periodic(void)
{
  // Go from transition percentage to ratio
  /*float ratio = FLOAT_OF_BFP(transition_percentage, INT32_PERCENTAGE_FRAC) / 100;*/
  float ratio = 0.0;

  if (use_scheduling == 1) {
    ratio = fabs(stateGetNedToBodyEulers_f()->theta) / M_PI_2;
  } else {
    ratio = 0.0;
  }

  if (radio_control.values[RADIO_FMODE] > 4800) {
    g_forward[0] = STABILIZATION_INDI_FORWARD_G1_P_FEW_PROPS;
  } else {
    g_forward[0] = STABILIZATION_INDI_FORWARD_G1_P;
  }

  // When using the pitch slider to take the props out of the mix, adjust the pitch effectiveness
    g_forward[1] = STABILIZATION_INDI_FORWARD_G1_Q - (1.0 - pitch_slider) * STABILIZATION_INDI_MOT_PITCH_EFF;
  // When using the yaw slider to take the props out of the mix, adjust the yaw effectiveness
    g_forward[2] = STABILIZATION_INDI_FORWARD_G1_R - (1.0 - yaw_slider) * STABILIZATION_INDI_MOT_YAW_EFF;

  indi.g1.p = g_hover[0] * (1.0 - ratio) + g_forward[0] * ratio;
  indi.g1.q = g_hover[1] * (1.0 - ratio) + g_forward[1] * ratio;
  indi.g1.r = g_hover[2] * (1.0 - ratio) + g_forward[2] * ratio;

  float ratio_spec_force = 0.0;
  if (radio_control.values[RADIO_FMODE] > 4800) {
    ratio_spec_force = 1.0;
  } else {
    ratio_spec_force = 0.0;
  }
  thrust_in_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN * (1.0 - ratio_spec_force)
                                  + GUIDANCE_INDI_SPECIFIC_FORCE_GAIN_FWD * ratio;

}
