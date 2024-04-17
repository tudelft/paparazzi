/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger.fr>
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

/** @file "modules/ctrl/eff_scheduling_falcon.c"
 * Interpolation of control effectivenss matrix of the Falcon hybrid plane
 */

#include "modules/ctrl/eff_scheduling_falcon.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "state.h"

#include "modules/datalink/downlink.h"

// Airspeed at which only with motor
#ifndef EFF_SCHEDULING_FALCON_LOW_AIRSPEED
#define EFF_SCHEDULING_FALCON_LOW_AIRSPEED 8.0f
#endif

#ifdef STABILIZATION_INDI_G1
static float g1g2_hover[INDI_OUTPUTS][INDI_NUM_ACT] = STABILIZATION_INDI_G1;
#else
static float g1g2_hover[INDI_OUTPUTS][INDI_NUM_ACT] = {
  STABILIZATION_INDI_G1_ROLL,
  STABILIZATION_INDI_G1_PITCH,
  STABILIZATION_INDI_G1_YAW,
  STABILIZATION_INDI_G1_THRUST
};
#endif

void eff_scheduling_falcon_init(void)
{
  for (int8_t i = 0; i < INDI_OUTPUTS; i++) {
    for (int8_t j = 0; j < INDI_NUM_ACT; j++) {
        g1g2[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
    }
  }
}

void eff_scheduling_falcon_periodic(void)
{
  // calculate squared airspeed
  float airspeed = stateGetAirspeed_f();

  if (airspeed > EFF_SCHEDULING_FALCON_LOW_AIRSPEED) {
    airspeed -= EFF_SCHEDULING_FALCON_LOW_AIRSPEED; //offset for start eff at zero!
    struct FloatEulers eulers_zxy;
    float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

    float pitch_ratio = 0.0f;
    if (eulers_zxy.theta > -M_PI_4) {
      pitch_ratio = 0.0f;
    }
    else {
      pitch_ratio = fabsf(1.0f - eulers_zxy.theta/(float)(-M_PI_4));
    }
    Bound(pitch_ratio, 0.0f, 1.0f);


    Bound(airspeed, 0.0f, 30.0f);
    float airspeed2 = airspeed*airspeed;

    // not effect of elevon on body roll axis
    g1g2[0][4] = 0;
    g1g2[0][5] = 0;

    float pitch_eff = pitch_ratio * EFF_PITCH_A * airspeed2;
    g1g2[1][4] =  pitch_eff / 1000; // elevon_right
    g1g2[1][5] = -pitch_eff / 1000; // elevon_left

    float yaw_eff = pitch_ratio * EFF_YAW_A * airspeed2;
    g1g2[2][4] = -yaw_eff / 1000; // elevon_right
    g1g2[2][5] = -yaw_eff / 1000; // elevon_left

    // No thrust generated by elevon, maybe take drag in accout for the future ?
    g1g2[3][4] = 0;
    g1g2[3][5] = 0;
  }
  else {
    //Come back to motor control
    g1g2[0][4] = 0; // elevon_left
    g1g2[0][5] = 0; // elevon_right

    g1g2[1][4] = 0; // elevon_left
    g1g2[1][5] = 0; // elevon_right

    g1g2[2][4] = 0; // elevon_left
    g1g2[2][5] = 0; // elevon_right

    g1g2[3][4] = 0; // elevon_left
    g1g2[3][5] = 0; // elevon_right
  }
}

extern void eff_scheduling_falcon_report(void)
{
  float f[6] = {
    g1g2[1][4], g1g2[1][5],
    g1g2[2][4], g1g2[2][5],
    g1g2[1][0], g1g2[2][0]
  };
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 6, f);
}
