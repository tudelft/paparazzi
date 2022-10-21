/*
 * Copyright (C) 2022 D.C. van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/ctrl/ctrl_eff_sched_rot_wing_v3.c"
 * @author D.C. van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Crtl effectiveness scheduler for thr rotating wing drone V3
 */

#include "modules/ctrl/ctrl_eff_sched_rot_wing_v3.h"

#include "modules/rot_wing_drone/wing_rotation_controller.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

#include "state.h"
#include "filters/low_pass_filter.h"

// Define initialization values
float g2_startup[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1_startup[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                                };
float rot_wing_side_motors_g1_p_0[2];
float rot_wing_side_motors_g1_q_90[2] = ROT_WING_SCHED_G1_Q_90; 

// Define polynomial constants for effectiveness values for aerodynamic surfaces
#ifndef ROT_WING_SCHED_G1_AERO_CONST_P
#error "ROT_WING_SCHED_AERO_CONST_P should be defined"
#endif

#ifndef ROT_WING_SCHED_G1_AERO_CONST_Q
#error "ROT_WING_SCHED_AERO_CONST_Q should be defined"
#endif

#ifndef ROT_WING_SCHED_G1_AERO_CONST_R
#error "ROT_WING_SCHED_AERO_CONST_R should be defined"
#endif

float rot_wing_aerodynamic_eff_const_g1_p[1] = ROT_WING_SCHED_G1_AERO_CONST_P;
float rot_wing_aerodynamic_eff_const_g1_q[1] = ROT_WING_SCHED_G1_AERO_CONST_Q;
float rot_wing_aerodynamic_eff_const_g1_r[1] = ROT_WING_SCHED_G1_AERO_CONST_R;

// Define settings to multiply initial control eff scheduling values
float g1_p_multiplier = 1.;
float g1_q_multiplier = 1.;
float g1_r_multiplier = 1.;
float g1_t_multiplier = 1.;

bool wing_rotation_sched_activated = false;

// Define filters
#ifndef ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF
#define ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF 1.5
#endif

Butterworth2LowPass airspeed_lowpass_filter;

void init_eff_scheduling(void)
{
  // Copy initial effectiveness on roll for side motors
  rot_wing_side_motors_g1_p_0[0] = g1_startup[0][1];
  rot_wing_side_motors_g1_p_0[1] = g1_startup[0][3];

  // Init filters
  float tau = 1.0 / (2.0 * M_PI * ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  init_butterworth_2_low_pass(&airspeed_lowpass_filter, tau, sample_time, 0.0);
}

void event_eff_scheduling(void)
{
  // Calculate triogeometric variables of wing rotation
  float cosr;
  float sinr;
  if (wing_rotation_sched_activated) {
    cosr = cosf(wing_rotation.wing_angle_rad);
    sinr = sinf(wing_rotation.wing_angle_rad);
  } else {
    cosr = 1;
    sinr = 0;
  }
  
  float g1_p_side_motors[2];
  float g1_q_side_motors[2];

  // Calculate roll and pitch effectiveness of the two roll side motors
  g1_p_side_motors[0] = rot_wing_side_motors_g1_p_0[0] * cosr;
  g1_p_side_motors[1] = rot_wing_side_motors_g1_p_0[1] * cosr;

  g1_q_side_motors[0] = rot_wing_side_motors_g1_q_90[0] * sinr;
  g1_q_side_motors[1] = rot_wing_side_motors_g1_q_90[1] * sinr;

  // Update inner loop effectiveness matrix for motors
  g1g2[0][0] = g1_startup[0][0] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][0] = g1_startup[1][0] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][0] = (g1_startup[2][0] * g1_r_multiplier + g2_startup[0]) / INDI_G_SCALING;
  g1g2[3][0] = g1_startup[3][0] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][1] = g1_p_side_motors[0] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][1] = g1_q_side_motors[0] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][1] = (g1_startup[2][1] * g1_r_multiplier + g2_startup[1]) / INDI_G_SCALING;
  g1g2[3][1] = g1_startup[3][1] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][2] = g1_startup[0][2] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][2] = g1_startup[1][2] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][2] = (g1_startup[2][2] * g1_r_multiplier + g2_startup[2]) / INDI_G_SCALING;
  g1g2[3][2] = g1_startup[3][2] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][3] = g1_p_side_motors[1] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][3] = g1_q_side_motors[1] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][3] = (g1_startup[2][3] * g1_r_multiplier + g2_startup[3]) / INDI_G_SCALING;
  g1g2[3][3] = g1_startup[3][3] * g1_t_multiplier / INDI_G_SCALING;

  // Update inner loop effectiveness matrix for aerodynamic surfaces
  float airspeed = stateGetAirspeed_f();
  update_butterworth_2_low_pass(&airspeed_lowpass_filter, airspeed);
  float airspeed2 = airspeed_lowpass_filter.o[0] * airspeed_lowpass_filter.o[0];

  g1g2[0][4] = rot_wing_aerodynamic_eff_const_g1_p[0] * airspeed2 / INDI_G_SCALING;

  g1g2[1][4] = rot_wing_aerodynamic_eff_const_g1_q[0] * airspeed2 / INDI_G_SCALING;

  g1g2[2][4] = rot_wing_aerodynamic_eff_const_g1_r[0] * airspeed2 / INDI_G_SCALING;

}