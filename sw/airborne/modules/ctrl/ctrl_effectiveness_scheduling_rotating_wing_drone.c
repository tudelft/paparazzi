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
#include "modules/rot_wing_drone/wing_rotation_controller.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

// Define arm length
#ifndef ROT_WING_SCHED_Y_ARM_LENGTH // Length of rotating arm from CG to side motor [m]
#error "ROT_WING_SCHED_Y_ARM_LENGTH should be defined"
#endif

float rot_wing_y_length = ROT_WING_SCHED_Y_ARM_LENGTH;

#ifndef ROT_WING_SCHED_G1_Q_90  // g1_q values for motor 2 (right motor) and motor 4 (left motor) in case the wing has rotated 90 deg
#error "ROT_WING_SCHED_G1_Q_90 should be defined"
#endif

// Define g1 p and q values for roll and pitch for 0 and 90 degrees rotation angles for the right and left motors
float rot_wing_initial_g1_p[INDI_NUM_ACT] = STABILIZATION_INDI_G1_ROLL;

float rot_wing_side_motors_g1_p_0[2];
float rot_wing_side_motors_g1_q_90[2] = ROT_WING_SCHED_G1_Q_90; 

// Define control effectiveness variables in roll and pitch for side motors
float rot_wing_g1_p_side_motors[2];
float rot_wing_g1_q_side_motors[2];

// Define initialization values
float g2_startup[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1_startup[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                                };

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_rot_wing_eff(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROT_WING_EFF(trans, dev, AC_ID,
                          2, rot_wing_g1_p_side_motors,
                          2, rot_wing_g1_q_side_motors);
}
#endif

void ctrl_eff_scheduling_rotating_wing_drone_init(void)
{
  // init scheduling

  // Copy initial effectiveness on roll for side motors
  rot_wing_side_motors_g1_p_0[0] = rot_wing_initial_g1_p[1];
  rot_wing_side_motors_g1_p_0[1] = rot_wing_initial_g1_p[3];

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROT_WING_EFF, send_rot_wing_eff);
  #endif
  
}

void ctrl_eff_scheduling_rotating_wing_drone_periodic(void)
{
  // periodic rotating ctrl_eff code

  // Pitch and Roll effectiveness of side motors (motor 2 right(idx = 1), motor 4 left (idx = 3)) should be updated based on the angle of the rotating wing
  float rot_wing_angle_rad = wing_rotation.wing_angle_rad;
  float c_rot_wing_angle = cosf(rot_wing_angle_rad);
  float s_rot_wing_angle = sinf(rot_wing_angle_rad);

  rot_wing_g1_p_side_motors[0] = c_rot_wing_angle * rot_wing_side_motors_g1_p_0[0] / INDI_G_SCALING;
  rot_wing_g1_p_side_motors[1] = c_rot_wing_angle * rot_wing_side_motors_g1_p_0[1] / INDI_G_SCALING;

  rot_wing_g1_q_side_motors[0] = s_rot_wing_angle * rot_wing_side_motors_g1_q_90[0] / INDI_G_SCALING;
  rot_wing_g1_q_side_motors[1] = s_rot_wing_angle * rot_wing_side_motors_g1_q_90[1] / INDI_G_SCALING;

  // Update init matrix if adaptive switched on to adapt to rotating wing effectiveness
  #ifdef STABILIZATION_INDI_USE_ADAPTIVE
    if (autopilot_in_flight())
    {
      // add delta g1 to g1_est to compensate for wing rotation
      g1_est[0][1] = (g1_est[0][1] + (rot_wing_g1_p_side_motors[0] * INDI_G_SCALING - g1_init[0][1]));
      g1_est[0][3] = (g1_est[0][3] + (rot_wing_g1_p_side_motors[1] * INDI_G_SCALING - g1_init[0][3]));
      g1_est[1][1] = (g1_est[1][1] + (rot_wing_g1_q_side_motors[0] * INDI_G_SCALING - g1_init[1][1]));
      g1_est[1][3] = (g1_est[1][3] + (rot_wing_g1_q_side_motors[1] * INDI_G_SCALING - g1_init[1][3]));

      // Update g1 init matrices to current values
      g1_init[0][1] = rot_wing_g1_p_side_motors[0] * INDI_G_SCALING; // Roll eff right motor
      g1_init[0][3] = rot_wing_g1_p_side_motors[1] * INDI_G_SCALING; // Roll eff left motor

      g1_init[1][1] = rot_wing_g1_q_side_motors[0] * INDI_G_SCALING; // Pitch eff right motor
      g1_init[1][3] = rot_wing_g1_q_side_motors[1] * INDI_G_SCALING; // Pitch eff left motor
    } else { 
      // Update g1 init matrices to current values
      g1_init[0][1] = rot_wing_g1_p_side_motors[0] * INDI_G_SCALING; // Roll eff right motor
      g1_init[0][3] = rot_wing_g1_p_side_motors[1] * INDI_G_SCALING; // Roll eff left motor

      g1_init[1][1] = rot_wing_g1_q_side_motors[0] * INDI_G_SCALING; // Pitch eff right motor
      g1_init[1][3] = rot_wing_g1_q_side_motors[1] * INDI_G_SCALING; // Pitch eff left motor

      // Direclty update g1g2 if not in flight ad adaptive INDI not activated if !in_flight
      //sum of G1 and G2
      for (int8_t i = 0; i < INDI_OUTPUTS; i++) {
        for (int8_t j = 0; j < INDI_NUM_ACT; j++) {
          if (i != 2) {
            g1g2[i][j] = g1_startup[i][j] / INDI_G_SCALING;
          } else {
            g1g2[i][j] = (g1_startup[i][j] + g2_startup[j]) / INDI_G_SCALING;
          }
        }
      }

      float_vect_copy(g1_est[0], g1_startup[0], INDI_OUTPUTS * INDI_NUM_ACT);
      float_vect_copy(g2_est, g2_startup, INDI_NUM_ACT);

      g1g2[0][1] = rot_wing_g1_p_side_motors[0]; // Roll eff right motor
      g1g2[0][3] = rot_wing_g1_p_side_motors[1]; // Roll eff left motor

      g1g2[1][1] = rot_wing_g1_q_side_motors[0]; // Pitch eff right motor
      g1g2[1][3] = rot_wing_g1_q_side_motors[1]; // Pitch eff left motor

      g1_est[0][1] = rot_wing_g1_p_side_motors[0] * INDI_G_SCALING; // Roll eff right motor
      g1_est[0][3] = rot_wing_g1_p_side_motors[1] * INDI_G_SCALING; // Roll eff left motor

      g1_est[1][1] = rot_wing_g1_q_side_motors[0] * INDI_G_SCALING; // Pitch eff right motor
      g1_est[1][3] = rot_wing_g1_q_side_motors[1] * INDI_G_SCALING; // Pitch eff left motor
    }

    
  #else // Direclty update g1g2 matrix without adaptive
    g1g2[0][1] = rot_wing_g1_p_side_motors[0]; // Roll eff right motor
    g1g2[0][3] = rot_wing_g1_p_side_motors[1]; // Roll eff left motor

    g1g2[1][1] = rot_wing_g1_q_side_motors[0]; // Pitch eff right motor
    g1g2[1][3] = rot_wing_g1_q_side_motors[1]; // Pitch eff left motor
  #endif // STABILIZATION_INDI_USE_ADAPTIVE
}