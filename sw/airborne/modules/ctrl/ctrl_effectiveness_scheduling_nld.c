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

/** @file modules/ctrl/ctrl_effectiveness_scheduling.c
 * Module that interpolates gainsets in flight based on the transition percentage
 */

#include "modules/ctrl/ctrl_effectiveness_scheduling_nld.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/radio_control/radio_control.h"

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#error "You need to use WLS control allocation for this module"
#endif

#ifndef INDI_FUNCTIONS_RC_CHANNEL
#error "You need to define an RC channel to switch between simple and advanced scheduling"
#endif

//static float g1g2_forward[INDI_OUTPUTS][INDI_NUM_ACT] = {FWD_G1_ROLL,
//                                                  FWD_G1_PITCH, FWD_G1_YAW, FWD_G1_THRUST
//                                                 };
//
//static float g1g2_hover[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
//                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
//                                               };
//
//static float g2_both[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING

int32_t use_scheduling = 1;

float ce_pitch_a = CE_PITCH_A;
float ce_pitch_b = CE_PITCH_B;
float ce_roll_a = CE_ROLL_A;
float ce_roll_b = CE_ROLL_B;
float ce_yaw_a = CE_YAW_A;
float ce_yaw_b = CE_YAW_B;

//Get the specified gains in the gainlibrary
void ctrl_eff_scheduling_init(void)
{
//  //sum of G1 and G2
//  int8_t i;
//  int8_t j;
//  for (i = 0; i < INDI_OUTPUTS; i++) {
//    for (j = 0; j < INDI_NUM_ACT; j++) {
//      if (i != 2) {
//        g1g2_hover[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
//        g1g2_forward[i][j] = g1g2_forward[i][j] / INDI_G_SCALING;
//      } else {
//        g1g2_forward[i][j] = (g1g2_forward[i][j] + g2_both[j]) / INDI_G_SCALING;
//        g1g2_hover[i][j] = (g1g2_hover[i][j] + g2_both[j]) / INDI_G_SCALING;
//      }
//    }
//  }
}

void ctrl_eff_scheduling_periodic(void)
{
    if (use_scheduling == 1) {
        ctrl_eff_scheduling_periodic_b();
    }

//#ifdef INDI_THRUST_ON_PITCH_EFF
//  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
//  if(radio_control.values[INDI_FUNCTIONS_RC_CHANNEL] > 0 && (actuator_state_filt_vect[0] < -7000) && (actuator_state_filt_vect[1] > 7000)) {
//    Bwls[1][2] = INDI_THRUST_ON_PITCH_EFF/INDI_G_SCALING;
//    Bwls[1][3] = INDI_THRUST_ON_PITCH_EFF/INDI_G_SCALING;
//  } else if(radio_control.values[INDI_FUNCTIONS_RC_CHANNEL] > 0 && (actuator_state_filt_vect[0] > 7000) && (actuator_state_filt_vect[1] < -7000)) {
//    Bwls[1][2] = -INDI_THRUST_ON_PITCH_EFF/INDI_G_SCALING;
//    Bwls[1][3] = -INDI_THRUST_ON_PITCH_EFF/INDI_G_SCALING;
//  } else {
//    Bwls[1][2] = 0.0;
//    Bwls[1][3] = 0.0;
//  }
//#endif

}

void ctrl_eff_scheduling_periodic_b(void)
{
  float airspeed = stateGetAirspeed_f();
//  struct FloatEulers eulers_zxy;

//    // use fixed value @ 6 m/s
//      float airspeed2 = 36.0;
////      float airspeed2 = airspeed*airspeed;
//
//      float pitch_eff = ce_pitch_a + ce_pitch_b*airspeed2;
////    g1g2[1][0] = -pitch_eff/1000;
//      g1g2[1][2] =  pitch_eff/1000;
//
//      float yaw_eff = ce_yaw_a + ce_yaw_b*airspeed2;
//      g1g2[2][0] = -yaw_eff/1000;
//      g1g2[2][1] = yaw_eff/1000;
//
//      /// Roll effectiveness depending on the airspeed
//      float roll_eff = ce_roll_a + ce_roll_b*airspeed2;
////    g1g2[0][2] = -roll_eff/1000;
//      g1g2[0][3] = roll_eff/1000;
//  } else {

#ifdef GUIDANCE_INDI_SOARING_USE_90_PITCH_OFFSET
#warning "schedule_use_90_pitch_offset"
    /// 90deg pitch offset;
    // Yaw is roll, Roll is yaw

    // calculate squared airspeed
    Bound(airspeed, 7.0, 23.0);
    float airspeed2 = airspeed*airspeed;

    float pitch_eff = ce_pitch_a + ce_pitch_b*airspeed2;
//    g1g2[1][0] = -pitch_eff/1000;
    g1g2[1][2] =  pitch_eff/1000;

    float yaw_eff = ce_yaw_a + ce_yaw_b*airspeed2;
    g1g2[2][0] = -yaw_eff/1000;
    g1g2[2][1] = yaw_eff/1000;

    /// Roll effectiveness depending on the airspeed
    // TODO: check the sign
    float roll_eff = ce_roll_a + ce_roll_b*airspeed2;
//    g1g2[0][2] = -roll_eff/1000;
    g1g2[0][3] = roll_eff/1000;
//  }
#else
/// NO 90 degrees pitch offset;
// Yaw is yaw, Roll is roll

    // calculate squared airspeed
    Bound(airspeed, 7.0, 23.0);
    float airspeed2 = airspeed*airspeed;

    float pitch_eff = ce_pitch_a + ce_pitch_b*airspeed2;
//    g1g2[1][0] = -pitch_eff/1000;
    g1g2[1][2] =  pitch_eff/1000;

    float roll_eff = ce_roll_a + ce_roll_b*airspeed2;
    g1g2[0][0] = roll_eff/1000;
    g1g2[0][1] = -roll_eff/1000;

    float yaw_eff = ce_yaw_a + ce_yaw_b*airspeed2;
//    g1g2[0][2] = -roll_eff/1000;
    g1g2[2][3] = yaw_eff/1000;
#endif

}

