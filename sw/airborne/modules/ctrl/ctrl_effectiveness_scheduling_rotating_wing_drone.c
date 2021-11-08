/*
 * Copyright (C) 2021 Dennis van Wijngaarden <D.C.vanWIjngaarden@tudelft.nl>
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

/** @file modules/ctrl/ctrl_effectiveness_scheduling_rotating_wing_drone.c
 * Module that interpolates gainsets in flight based on the transition percentage
 */

#include "modules/ctrl/ctrl_effectiveness_scheduling_rotating_wing_drone.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

// Define wing angle as setting
uint8_t rot_wing_angle_deg = 0; // degrees as setting

// Define arm length
#ifndef ROT_WING_SCHED_Y_ARM_LENGTH // Length of rotating arm from CG to side motor [m]
#error "ROT_WING_SCHED_Y_ARM_LENGTH should be defined"
#endif

float rot_wing_y_length = ROT_WING_SCHED_Y_ARM_LENGTH;

#ifndef ROT_WING_SCHED_G1_Q_90  // g1_q values for motor 2 (right motor) and motor 4 (left motor) in case the wing has rotated 90 deg
#error "ROT_WING_SCHED_G1_Q_90 should be defined"
#endif

// Define g1 p and q values for roll and pitch for 0 and 90 degrees rotation angles for the right and left motors
float rot_wing_g1_p_0[2];
float rot_wing_g1_q_90[2] = ROT_WING_SCHED_G1_Q_90; 

void ctrl_eff_scheduling_rotating_wing_drone_init(void)
{
  // init scheduling

  // Copy initial effectiveness on roll for side motors
  rot_wing_g1_p_0[0] = g1g2[0][1];
  rot_wing_g1_p_0[1] = g1g2[1][3];
  
}

void ctrl_eff_scheduling_rotating_wing_drone_periodic(void)
{
  // periodic rotating ctrl_eff code

  // Pitch and Roll effectiveness of side motors (motor 2 right(idx = 1), motor 4 left (idx = 3)) should be updated based on the angle of the rotating wing
  float rot_wing_angle_rad = (float)rot_wing_angle_deg / 180. * M_PI;
  float c_rot_wing_angle = cos(rot_wing_angle_rad);
  float s_rot_wing_angle = sin(rot_wing_angle_rad);

  g1g2[0][1] = c_rot_wing_angle * rot_wing_g1_p_0[0] / INDI_G_SCALING; // Roll eff right motor
  g1g2[0][3] = c_rot_wing_angle * rot_wing_g1_p_0[1] / INDI_G_SCALING; // Roll eff left motor

  g1g2[1][1] = s_rot_wing_angle * rot_wing_g1_q_90[0] / INDI_G_SCALING; // Pitch eff right motor
  g1g2[1][3] = s_rot_wing_angle * rot_wing_g1_q_90[1] / INDI_G_SCALING; // Pitch eff left motor
}








