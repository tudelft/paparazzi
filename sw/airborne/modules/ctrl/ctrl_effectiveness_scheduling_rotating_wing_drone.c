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
#include "state.h"
#include "filters/low_pass_filter.h"

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

float rot_wing_aerodynamic_eff_const_g1_p[4] = ROT_WING_SCHED_G1_AERO_CONST_P;
float rot_wing_aerodynamic_eff_const_g1_q[4] = ROT_WING_SCHED_G1_AERO_CONST_Q;
float rot_wing_aerodynamic_eff_const_g1_r[4] = ROT_WING_SCHED_G1_AERO_CONST_R;

// Define control effectiveness variables in roll, pitch and yaw for aerodynamic surfaces {AIL_LEFT, AIL_RIGHT, VTAIL_LEFT, VTAIL_RIGHT}
float rot_wing_g1_p_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};
float rot_wing_g1_q_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};
float rot_wing_g1_r_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};

// Define initialization values
float g2_startup[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1_startup[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                                };

// Define filters
#ifndef ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF
#define ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF 1.5
#endif

Butterworth2LowPass airspeed_lowpass_filter;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_rot_wing_eff(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROT_WING_EFF(trans, dev, AC_ID,
                          INDI_NUM_ACT, g1_init[0],
                          INDI_NUM_ACT, g1_init[1],
                          INDI_NUM_ACT, g1_init[2]);
}
#endif

void ctrl_eff_scheduling_rotating_wing_drone_init(void)
{
  // init scheduling

  // Copy initial effectiveness on roll for side motors
  rot_wing_side_motors_g1_p_0[0] = rot_wing_initial_g1_p[1];
  rot_wing_side_motors_g1_p_0[1] = rot_wing_initial_g1_p[3];

  // Init filters
  float tau = 1.0 / (2.0 * M_PI * ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  init_butterworth_2_low_pass(&airspeed_lowpass_filter, tau, sample_time, 0.0);

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

  rot_wing_g1_p_side_motors[0] = c_rot_wing_angle * rot_wing_side_motors_g1_p_0[0];
  rot_wing_g1_p_side_motors[1] = c_rot_wing_angle * rot_wing_side_motors_g1_p_0[1];

  rot_wing_g1_q_side_motors[0] = s_rot_wing_angle * rot_wing_side_motors_g1_q_90[0];
  rot_wing_g1_q_side_motors[1] = s_rot_wing_angle * rot_wing_side_motors_g1_q_90[1];

  // Effectiveness values of aerodynamic surfaces based on airspeed
  float airspeed = stateGetAirspeed_f();
  update_butterworth_2_low_pass(&airspeed_lowpass_filter, airspeed);
  float airspeed2 = airspeed_lowpass_filter.o[0] * airspeed_lowpass_filter.o[0];
  // Perform analysis on rotating ailerons
  for (int i = 0; i < 2; i++) {
    rot_wing_g1_p_aerodynamic_surf[i] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[i] * s_rot_wing_angle; // roll effectieness scales with rotation
    rot_wing_g1_q_aerodynamic_surf[i] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[i];
    rot_wing_g1_r_aerodynamic_surf[i] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_r[i];
  }
  // Perform analysis on fixed tail
  for (int i = 2; i < 4; i++) {
    rot_wing_g1_p_aerodynamic_surf[i] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[i];
    rot_wing_g1_q_aerodynamic_surf[i] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[i];
    rot_wing_g1_r_aerodynamic_surf[i] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_r[i];
  }

  // Update init matrix if adaptive switched on to adapt to rotating wing effectiveness
  #ifdef STABILIZATION_INDI_USE_ADAPTIVE
    if (autopilot_in_flight())
    {
      // add delta g1 to g1_est to compensate for wing rotation
      g1_est[0][1] = (g1_est[0][1] + (rot_wing_g1_p_side_motors[0] - g1_init[0][1]));
      g1_est[0][3] = (g1_est[0][3] + (rot_wing_g1_p_side_motors[1] - g1_init[0][3]));
      g1_est[1][1] = (g1_est[1][1] + (rot_wing_g1_q_side_motors[0] - g1_init[1][1]));
      g1_est[1][3] = (g1_est[1][3] + (rot_wing_g1_q_side_motors[1] - g1_init[1][3]));

      // Update g1 init matrices to current values
      g1_init[0][1] = rot_wing_g1_p_side_motors[0]; // Roll eff right motor
      g1_init[0][3] = rot_wing_g1_p_side_motors[1]; // Roll eff left motor

      g1_init[1][1] = rot_wing_g1_q_side_motors[0]; // Pitch eff right motor
      g1_init[1][3] = rot_wing_g1_q_side_motors[1]; // Pitch eff left motor

      // Update values for aerodynamic surfaces
      for (int i = 4; i < 8; i++) {
        uint8_t j = i-4; // Index of actuator
        g1_est[0][i] = (g1_est[0][i] + (rot_wing_g1_p_aerodynamic_surf[j] - g1_init[0][i]));
        g1_est[1][i] = (g1_est[1][i] + (rot_wing_g1_q_aerodynamic_surf[j] - g1_init[1][i]));
        g1_est[2][i] = (g1_est[2][i] + (rot_wing_g1_r_aerodynamic_surf[j] - g1_init[2][i]));

        g1_init[0][i] = rot_wing_g1_p_aerodynamic_surf[j];
        g1_init[1][i] = rot_wing_g1_q_aerodynamic_surf[j];
        g1_init[2][i] = rot_wing_g1_r_aerodynamic_surf[j];
      }

    } else { 
      // Update g1 init matrices to current values
      g1_init[0][1] = rot_wing_g1_p_side_motors[0]; // Roll eff right motor
      g1_init[0][3] = rot_wing_g1_p_side_motors[1]; // Roll eff left motor

      g1_init[1][1] = rot_wing_g1_q_side_motors[0]; // Pitch eff right motor
      g1_init[1][3] = rot_wing_g1_q_side_motors[1]; // Pitch eff left motor

      for (int i = 4; i < 8; i++) {
        uint8_t j = i-4; // Index of actuator
        g1_init[0][i] = rot_wing_g1_p_aerodynamic_surf[j];
        g1_init[1][i] = rot_wing_g1_q_aerodynamic_surf[j];
        g1_init[2][i] = rot_wing_g1_r_aerodynamic_surf[j];
      }

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

      g1g2[0][1] = rot_wing_g1_p_side_motors[0] / INDI_G_SCALING; // Roll eff right motor
      g1g2[0][3] = rot_wing_g1_p_side_motors[1] / INDI_G_SCALING; // Roll eff left motor

      g1g2[1][1] = rot_wing_g1_q_side_motors[0] / INDI_G_SCALING; // Pitch eff right motor
      g1g2[1][3] = rot_wing_g1_q_side_motors[1] / INDI_G_SCALING; // Pitch eff left motor

      g1_est[0][1] = rot_wing_g1_p_side_motors[0]; // Roll eff right motor
      g1_est[0][3] = rot_wing_g1_p_side_motors[1]; // Roll eff left motor

      g1_est[1][1] = rot_wing_g1_q_side_motors[0]; // Pitch eff right motor
      g1_est[1][3] = rot_wing_g1_q_side_motors[1]; // Pitch eff left motor

      // Update values for aerodynamic surfaces
      for (int i = 4; i < 8; i++) {
        uint8_t j = i-4; // Index of actuator
        g1g2[0][i] = rot_wing_g1_p_aerodynamic_surf[j] / INDI_G_SCALING;
        g1g2[1][i] = rot_wing_g1_q_aerodynamic_surf[j] / INDI_G_SCALING;
        g1g2[2][i] = rot_wing_g1_r_aerodynamic_surf[j] / INDI_G_SCALING;

        g1_est[0][i] = rot_wing_g1_p_aerodynamic_surf[j];
        g1_est[1][i] = rot_wing_g1_q_aerodynamic_surf[j];
        g1_est[2][i] = rot_wing_g1_r_aerodynamic_surf[j];
      }
    }

    
  #else // Direclty update g1g2 matrix without adaptive
    g1g2[0][1] = rot_wing_g1_p_side_motors[0] / INDI_G_SCALING; // Roll eff right motor
    g1g2[0][3] = rot_wing_g1_p_side_motors[1] / INDI_G_SCALING; // Roll eff left motor

    g1g2[1][1] = rot_wing_g1_q_side_motors[0] / INDI_G_SCALING; // Pitch eff right motor
    g1g2[1][3] = rot_wing_g1_q_side_motors[1] / INDI_G_SCALING; // Pitch eff left motor

    // Update values for aerodynamic surfaces
    for (int i = 4; i < 8; i++) {
      uint8_t j = i-4; // Index of actuator
      g1g2[0][i] = rot_wing_g1_p_aerodynamic_surf[j] / INDI_G_SCALING;
      g1g2[1][i] = rot_wing_g1_q_aerodynamic_surf[j] / INDI_G_SCALING;
      g1g2[2][i] = rot_wing_g1_r_aerodynamic_surf[j] / INDI_G_SCALING;
    }
  #endif // STABILIZATION_INDI_USE_ADAPTIVE
}