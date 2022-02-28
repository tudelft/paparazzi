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
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
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

float rot_wing_g1_p[8];
float rot_wing_g1_q[8];
float rot_wing_g1_r[8];

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

// For quick testing and tuning
float rot_wing_aero_eff_const_p; 
float rot_wing_aero_eff_const_q; 
float rot_wing_aero_eff_const_r; 

float rot_wing_min_lift_pitch_eff = 0.;//GUIDANCE_INDI_PITCH_LIFT_EFF;
float rot_wing_max_lift_pitch_eff = 0.035;//GUIDANCE_INDI_PITCH_LIFT_EFF;

// Define control effectiveness variables in roll, pitch and yaw for aerodynamic surfaces {AIL_LEFT, AIL_RIGHT, VTAIL_LEFT, VTAIL_RIGHT}
float rot_wing_g1_p_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};
float rot_wing_g1_q_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};
float rot_wing_g1_r_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};

// Define initialization values
float g2_startup[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1_startup[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                                };

float rot_wing_aileron_limit_deg = 45; // Aileron is effective from this angle value onwards
float rot_wing_roll_prop_limit_deg = 70; // Roll props are not effective anymore from thus value onwards
float rot_wing_pitch_prop_limit_deg = 70; // Pitch props are not effective anymore from thus value onwards
float rot_wing_yaw_prop_limit_deg = 70; // // Yaw props are not effective anymore from thus value onwards
float rot_wing_limit_deadzone_deg = 2; // The deadzone that is put on the wing angle sensor 

bool rot_wing_ailerons_activated; // will be set during initialization
bool rot_wing_roll_props_activated; // Will be set during initialization
bool rot_wing_pitch_props_activated; // Will be set during initialization
bool rot_wing_yaw_props_activated; // Will be set during initialization

// Define filters
#ifndef ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF
#define ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF 1.5
#endif

Butterworth2LowPass airspeed_lowpass_filter;

// Scheduler function to activate ailerons / roll motors bases
inline void init_active_actuators(void);
inline void evaluate_actuator_active(void);
inline void schedule_motor_effectiveness(float c_rot_wing_angle, float s_rot_wing_angle);
inline void schedule_aero_effectiveness(float c_rot_wing_angle, float s_rot_wing_angle, float airspeed2);
inline void update_g1g2_matrix(void);
inline void schedule_lift_pitch_eff(float rot_wing_angle_rad);

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

  // Init testing settings
  rot_wing_aero_eff_const_p = rot_wing_aerodynamic_eff_const_g1_p[0];
  rot_wing_aero_eff_const_q = -rot_wing_aerodynamic_eff_const_g1_q[2];
  rot_wing_aero_eff_const_r = rot_wing_aerodynamic_eff_const_g1_r[3];

  // Init filters
  float tau = 1.0 / (2.0 * M_PI * ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  init_butterworth_2_low_pass(&airspeed_lowpass_filter, tau, sample_time, 0.0);

  // Initialize if roll props and ailerons are activated
  init_active_actuators();

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

  // Effectiveness values of aerodynamic surfaces based on airspeed
  float airspeed = stateGetAirspeed_f();
  update_butterworth_2_low_pass(&airspeed_lowpass_filter, airspeed);
  float airspeed2 = airspeed_lowpass_filter.o[0] * airspeed_lowpass_filter.o[0];

  #ifdef ROT_WING_SCHED_AERO_EFF_TUNING
    rot_wing_aerodynamic_eff_const_g1_p[0] = rot_wing_aero_eff_const_p;
    rot_wing_aerodynamic_eff_const_g1_p[1] = rot_wing_aero_eff_const_p;
    rot_wing_aerodynamic_eff_const_g1_q[2] = -rot_wing_aero_eff_const_q;
    rot_wing_aerodynamic_eff_const_g1_r[3] = rot_wing_aero_eff_const_r;
  #endif

  // perform function that schedules switching on and off actuators based on roll angle
  evaluate_actuator_active();

  // Update motor effectiveness
  schedule_motor_effectiveness(c_rot_wing_angle, s_rot_wing_angle);

  // Update aerodynamic effectiveness
  schedule_aero_effectiveness(c_rot_wing_angle, s_rot_wing_angle, airspeed2);

  // Update pitch lift effectiveness
  schedule_lift_pitch_eff(rot_wing_angle_rad);

  // Finally update g1g2 matrix
  update_g1g2_matrix();
}

void init_active_actuators(void)
{
  float rot_wing_angle_deg = wing_rotation.wing_angle_deg;

  if (rot_wing_angle_deg > rot_wing_aileron_limit_deg) {
    rot_wing_ailerons_activated = true;
  } else {
    rot_wing_ailerons_activated = false;
  }

  if (rot_wing_angle_deg < rot_wing_roll_prop_limit_deg) {
    rot_wing_roll_props_activated = true;
  } else {
    rot_wing_roll_props_activated = false;
  }

  if (rot_wing_angle_deg < rot_wing_pitch_prop_limit_deg) {
    rot_wing_pitch_props_activated = true;
  } else {
    rot_wing_pitch_props_activated = false;
  }

  if (rot_wing_angle_deg < rot_wing_yaw_prop_limit_deg) {
    rot_wing_yaw_props_activated = true;
  } else {
    rot_wing_yaw_props_activated = false;
  }
}

void evaluate_actuator_active(void)
{
  // This function checks which actuators may be used for roll

  float rot_wing_angle_deg = wing_rotation.wing_angle_deg;

  // Evaluate Ailerons
  if (rot_wing_ailerons_activated) {
    // Check if needs to be deactivated
    if (rot_wing_angle_deg < (rot_wing_aileron_limit_deg - rot_wing_limit_deadzone_deg)) {
      rot_wing_ailerons_activated = false;
    }
  } else {
    // Check if needs to be activated
    if (rot_wing_angle_deg > (rot_wing_aileron_limit_deg + rot_wing_limit_deadzone_deg)) {
      rot_wing_ailerons_activated = true;
    }
  }

  // Evaluate roll props
  if (rot_wing_roll_props_activated) {
    // Check if needs to be deactivated
    if (rot_wing_angle_deg > (rot_wing_roll_prop_limit_deg + rot_wing_limit_deadzone_deg)) {
      rot_wing_roll_props_activated = false;
    } 
  } else {
    // Check if needs to be activated
    if (rot_wing_angle_deg < (rot_wing_roll_prop_limit_deg - rot_wing_limit_deadzone_deg)) {
      rot_wing_roll_props_activated = true;
    }
  }

  // Evaluate pitch props
  if (rot_wing_pitch_props_activated) {
    // Check if needs to be deactivated
    if (rot_wing_angle_deg > (rot_wing_pitch_prop_limit_deg + rot_wing_limit_deadzone_deg)) {
      rot_wing_pitch_props_activated = false;
    } 
  } else {
    // Check if needs to be activated
    if (rot_wing_angle_deg < (rot_wing_pitch_prop_limit_deg - rot_wing_limit_deadzone_deg)) {
      rot_wing_pitch_props_activated = true;
    }
  }

  // Evaluate yaw props
  if (rot_wing_yaw_props_activated) {
    // Check if needs to be deactivated
    if (rot_wing_angle_deg > (rot_wing_yaw_prop_limit_deg + rot_wing_limit_deadzone_deg)) {
      rot_wing_yaw_props_activated = false;
    } 
  } else {
    // Check if needs to be activated
    if (rot_wing_angle_deg < (rot_wing_yaw_prop_limit_deg - rot_wing_limit_deadzone_deg)) {
      rot_wing_yaw_props_activated = true;
    }
  }
}

void schedule_motor_effectiveness(float c_rot_wing_angle, float s_rot_wing_angle)
{
  // Calculate roll, pitch, yaw effectiveness values
  // roll
  rot_wing_g1_p[0] = g1_startup[0][0] * rot_wing_roll_props_activated;
  rot_wing_g1_p[1] = c_rot_wing_angle * g1_startup[0][1] * rot_wing_roll_props_activated;
  rot_wing_g1_p[2] = g1_startup[0][2] * rot_wing_roll_props_activated;
  rot_wing_g1_p[3] = c_rot_wing_angle * g1_startup[0][3] * rot_wing_roll_props_activated;

  // pitch
  rot_wing_g1_q[0] = g1_startup[1][0] * rot_wing_pitch_props_activated;
  rot_wing_g1_q[1] = s_rot_wing_angle * rot_wing_side_motors_g1_q_90[0] * rot_wing_pitch_props_activated;
  rot_wing_g1_q[2] = g1_startup[1][2] * rot_wing_pitch_props_activated;
  rot_wing_g1_q[3] = s_rot_wing_angle * rot_wing_side_motors_g1_q_90[1] * rot_wing_pitch_props_activated;

  // yaw
  rot_wing_g1_r[0] = g1_startup[2][0] * rot_wing_yaw_props_activated;
  rot_wing_g1_r[1] = g1_startup[2][1] * rot_wing_yaw_props_activated;
  rot_wing_g1_r[2] = g1_startup[2][2] * rot_wing_yaw_props_activated;
  rot_wing_g1_r[3] = g1_startup[2][3] * rot_wing_yaw_props_activated;

  // add delta g1 to g1_est to compensate for wing rotation
  g1_est[0][0] = (g1_est[0][0] + (rot_wing_g1_p[0] - g1_init[0][0])) * rot_wing_roll_props_activated;
  g1_est[0][1] = (g1_est[0][1] + (rot_wing_g1_p[1] - g1_init[0][1])) * rot_wing_roll_props_activated;
  g1_est[0][2] = (g1_est[0][2] + (rot_wing_g1_p[2] - g1_init[0][2])) * rot_wing_roll_props_activated;
  g1_est[0][3] = (g1_est[0][3] + (rot_wing_g1_p[3] - g1_init[0][3])) * rot_wing_roll_props_activated;

  g1_est[1][0] = (g1_est[1][0] + (rot_wing_g1_q[0] - g1_init[1][0])) * rot_wing_pitch_props_activated;
  g1_est[1][1] = (g1_est[1][1] + (rot_wing_g1_q[1] - g1_init[1][1])) * rot_wing_pitch_props_activated;
  g1_est[1][2] = (g1_est[1][2] + (rot_wing_g1_q[2] - g1_init[1][2])) * rot_wing_pitch_props_activated;
  g1_est[1][3] = (g1_est[1][3] + (rot_wing_g1_q[3] - g1_init[1][3])) * rot_wing_pitch_props_activated;

  g1_est[2][0] = (g1_est[2][0] + (rot_wing_g1_r[0] - g1_init[2][0])) * rot_wing_yaw_props_activated;
  g1_est[2][1] = (g1_est[2][1] + (rot_wing_g1_r[1] - g1_init[2][1])) * rot_wing_yaw_props_activated;
  g1_est[2][2] = (g1_est[2][2] + (rot_wing_g1_r[2] - g1_init[2][2])) * rot_wing_yaw_props_activated;
  g1_est[2][3] = (g1_est[2][3] + (rot_wing_g1_r[3] - g1_init[2][3])) * rot_wing_yaw_props_activated;

  // Update g1 init matrices to current values
  g1_init[0][0] = rot_wing_g1_p[0];
  g1_init[0][1] = rot_wing_g1_p[1];
  g1_init[0][2] = rot_wing_g1_p[2];
  g1_init[0][3] = rot_wing_g1_p[3];

  g1_init[1][0] = rot_wing_g1_q[0];
  g1_init[1][1] = rot_wing_g1_q[1];
  g1_init[1][2] = rot_wing_g1_q[2];
  g1_init[1][3] = rot_wing_g1_q[3];

  g1_init[2][0] = rot_wing_g1_r[0];
  g1_init[2][1] = rot_wing_g1_r[1];
  g1_init[2][2] = rot_wing_g1_r[2];
  g1_init[2][3] = rot_wing_g1_r[3];
}

void schedule_aero_effectiveness(float c_rot_wing_angle, float s_rot_wing_angle, float airspeed2)
{
  // Calculate roll, pitch, yaw effectiveness values
  // roll
  rot_wing_g1_p[4] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[0] * s_rot_wing_angle * rot_wing_ailerons_activated;
  rot_wing_g1_p[5] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[1] * s_rot_wing_angle * rot_wing_ailerons_activated;
  rot_wing_g1_p[6] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[2];
  rot_wing_g1_p[7] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[3];
  
  // pitch
  rot_wing_g1_q[4] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[0] * c_rot_wing_angle * rot_wing_ailerons_activated;
  rot_wing_g1_q[5] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[1] * c_rot_wing_angle * rot_wing_ailerons_activated;
  rot_wing_g1_q[6] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[2];
  rot_wing_g1_q[7] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[3];

  // yaw
  rot_wing_g1_r[4] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_r[0] * s_rot_wing_angle * rot_wing_ailerons_activated;
  rot_wing_g1_r[5] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_r[1] * s_rot_wing_angle * rot_wing_ailerons_activated;
  rot_wing_g1_r[6] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_r[2];
  rot_wing_g1_r[7] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_r[3];

  // add delta g1 to g1_est to compensate for wing rotation
  g1_est[0][4] = (g1_est[0][4] + (rot_wing_g1_p[4] - g1_init[0][4])) * rot_wing_ailerons_activated;
  g1_est[0][5] = (g1_est[0][5] + (rot_wing_g1_p[5] - g1_init[0][5])) * rot_wing_ailerons_activated;
  g1_est[0][6] = (g1_est[0][6] + (rot_wing_g1_p[6] - g1_init[0][6]));
  g1_est[0][7] = (g1_est[0][7] + (rot_wing_g1_p[7] - g1_init[0][7]));

  g1_est[1][4] = (g1_est[1][4] + (rot_wing_g1_q[4] - g1_init[1][4])) * rot_wing_ailerons_activated;
  g1_est[1][5] = (g1_est[1][5] + (rot_wing_g1_q[5] - g1_init[1][5])) * rot_wing_ailerons_activated;
  g1_est[1][6] = (g1_est[1][6] + (rot_wing_g1_q[6] - g1_init[1][6]));
  g1_est[1][7] = (g1_est[1][7] + (rot_wing_g1_q[7] - g1_init[1][7]));

  g1_est[2][4] = (g1_est[2][4] + (rot_wing_g1_r[4] - g1_init[2][4])) * rot_wing_ailerons_activated;
  g1_est[2][5] = (g1_est[2][5] + (rot_wing_g1_r[5] - g1_init[2][5])) * rot_wing_ailerons_activated;
  g1_est[2][6] = (g1_est[2][6] + (rot_wing_g1_r[6] - g1_init[2][6]));
  g1_est[2][7] = (g1_est[2][7] + (rot_wing_g1_r[7] - g1_init[2][7]));

  // Update g1 init matrices to current values
  g1_init[0][4] = rot_wing_g1_p[4];
  g1_init[0][5] = rot_wing_g1_p[5];
  g1_init[0][6] = rot_wing_g1_p[6];
  g1_init[0][7] = rot_wing_g1_p[7];

  g1_init[1][4] = rot_wing_g1_q[4];
  g1_init[1][5] = rot_wing_g1_q[5];
  g1_init[1][6] = rot_wing_g1_q[6];
  g1_init[1][7] = rot_wing_g1_q[7];

  g1_init[2][4] = rot_wing_g1_r[4];
  g1_init[2][5] = rot_wing_g1_r[5];
  g1_init[2][6] = rot_wing_g1_r[6];
  g1_init[2][7] = rot_wing_g1_r[7];
}

void update_g1g2_matrix(void)
{
  #ifdef STABILIZATION_INDI_USE_ADAPTIVE
  if (!autopilot_in_flight()) {
    for (int i = 0; i < 8; i++) {
        g1g2[0][i] = rot_wing_g1_p[i] / INDI_G_SCALING;
        g1g2[1][i] = rot_wing_g1_q[i] / INDI_G_SCALING;
        g1g2[2][i] = (rot_wing_g1_r[i] + g2_startup[i]) / INDI_G_SCALING;
    }
  }
  #else
  // Copy effectiveness direcly inyo g1g2
  for (int i = 0; i < 8; i++) {
        g1g2[0][i] = rot_wing_g1_p[i] / INDI_G_SCALING;
        g1g2[1][i] = rot_wing_g1_q[i] / INDI_G_SCALING;
        g1g2[2][i] = (rot_wing_g1_r[i] + g2_startup[i]) / INDI_G_SCALING;
    }
  #endif // STABILIZATION_INDI_USE_ADAPTIVE
} 

void schedule_lift_pitch_eff(float rot_wing_angle_rad)
{
  float lift_pitch_eff_diff = rot_wing_max_lift_pitch_eff - rot_wing_min_lift_pitch_eff;
  float pitch_lift_angle_rad = (rot_wing_angle_rad - M_PI / 6.) * 1.5;
  Bound(pitch_lift_angle_rad, 0, M_PI/2);
  float s_pitch_lift = sinf(pitch_lift_angle_rad);
  lift_pitch_eff = rot_wing_min_lift_pitch_eff + s_pitch_lift * lift_pitch_eff_diff;
}