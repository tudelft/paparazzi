/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/ctrl/eff_scheduling_nederdrone.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Interpolation of control effectivenss matrix 
of the Nederdrone.
      
If instead using online adaptation is an option, be sure to 
not use this module at the same time!
 */

#include "modules/ctrl/eff_scheduling_nederdrone.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "state.h"
#include "autopilot.h"
#include "modules/radio_control/radio_control.h"
#include "generated/radio.h"
#define INDI_SCHEDULING_LOWER_BOUND_G1 0.0001

// Airspeed at which tip props should be turned on
#ifndef INDI_SCHEDULING_LOW_AIRSPEED
#define INDI_SCHEDULING_LOW_AIRSPEED 14.0
#endif

int32_t use_scheduling = 1;

static float g_forward[4][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL_FWD, STABILIZATION_INDI_G1_PITCH_FWD, STABILIZATION_INDI_G1_YAW_FWD, STABILIZATION_INDI_G1_THRUST_FWD};

static float g_hover[4][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL, STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST};

// Functions to schedule switching on and of of tip props on front wing
float sched_ratio_tip_props = 1.0;
// If pitch lower, pitch props gradually switch off till  sched_tip_prop_lower_pitch_limit_deg (1 > sched_ratio_tip_props > 0)
float sched_tip_prop_upper_pitch_limit_deg = -45;
// If pitch lower, pitch props switch fully off (sched_ratio_tip_props goes to 0)
float sched_tip_prop_lower_pitch_limit_deg = -80;
// Setting to not switch off tip props during forward flight
bool sched_tip_props_always_on = false;

void ctrl_eff_scheduling_init(void)
{
  // your init code here
  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    g_hover[0][i] = g_hover[0][i] / INDI_G_SCALING;
    g_hover[1][i] = g_hover[1][i] / INDI_G_SCALING;
    g_hover[2][i] = g_hover[2][i] / INDI_G_SCALING;
    g_hover[3][i] = g_hover[3][i] / INDI_G_SCALING;

    g_forward[0][i] = g_forward[0][i] / INDI_G_SCALING;
    g_forward[1][i] = g_forward[1][i] / INDI_G_SCALING;
    g_forward[2][i] = g_forward[2][i] / INDI_G_SCALING;
    g_forward[3][i] = g_forward[3][i] / INDI_G_SCALING;
  }
}

void ctrl_eff_scheduling_periodic(void)
{
  // your periodic code here.
  // freq = 20.0 Hz

  float airspeed = stateGetAirspeed_f();

  // Go from transition percentage to ratio
  /*float ratio = FLOAT_OF_BFP(transition_percentage, INT32_PERCENTAGE_FRAC) / 100;*/
  float ratio = 0.0;

  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  // Ratio is only based on pitch now, as the pitot tube is often not mounted.
  if (use_scheduling == 1) {
    ratio = fabs(eulers_zxy.theta) / M_PI_2;
  } else {
    ratio = 0.0;
  }
  Bound(ratio,0.0,1.0);

  float stall_speed = 14.0; // m/s
  float pitch_ratio = 0.0;
  // Assume hover or stalled conditions below 14 m/s
  if (use_scheduling == 1) {
    if ( (eulers_zxy.theta > -M_PI_4) || (airspeed < stall_speed) ) {
      if (eulers_zxy.theta > -M_PI_4) {
        pitch_ratio = 0.0;
      } else {
        pitch_ratio = fabs(1.0 - eulers_zxy/(-M_PI_4));
      }

    } else {
      pitch_ratio = 1.0;
    }
  } else {
    pitch_ratio = 0.0;
  }
  Bound(pitch_ratio,0.0,1.0);

  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {

    // Roll
    g1g2[0][i] = g_hover[0][i] * (1.0 - ratio) + g_forward[0][i] * ratio;
    //Pitch, scaled with v^2
    g1g2[1][i] = g_hover[1][i] * (1.0 - pitch_ratio) + g_forward[1][i] * pitch_ratio * airspeed * airspeed / (16.0*16.0);
    //Yaw
    g1g2[2][i] = g_hover[2][i] * (1.0 - ratio) + g_forward[2][i] * ratio;
  }

  // Thrust effectiveness
  float ratio_spec_force = 0.0;
  Bound(airspeed, 8.0, 20.0);
  ratio_spec_force = (airspeed-8.0) / 12.0;

  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    // Thrust
    g1g2[3][i] = g_hover[3][i] * (1.0 - ratio_spec_force) + g_forward[3][i] * ratio_spec_force;
  }

  bool low_airspeed = (stateGetAirspeed_f() < INDI_SCHEDULING_LOW_AIRSPEED) && (autopilot_get_mode() != AP_MODE_NAV);

  // Tip prop ratio
  float pitch_deg = eulers_zxy.theta / M_PI * 180.f;
  float pitch_range_deg = sched_tip_prop_upper_pitch_limit_deg - sched_tip_prop_lower_pitch_limit_deg;
  if (sched_tip_props_always_on || low_airspeed || radio_control.values[RADIO_AUX2] > 0) {
    sched_ratio_tip_props = 1.0;
  } else if (pitch_deg > sched_tip_prop_upper_pitch_limit_deg) {
    sched_ratio_tip_props = 1.0;
  } else if (pitch_deg < sched_tip_prop_lower_pitch_limit_deg) {
    sched_ratio_tip_props = 0.0;
  } else {
    float pitch_offset = pitch_deg - sched_tip_prop_lower_pitch_limit_deg;
    sched_ratio_tip_props = pitch_offset / pitch_range_deg;
  }
  Bound(sched_ratio_tip_props, 0.0, 1.0);
}
