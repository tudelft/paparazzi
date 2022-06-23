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
#include "subsystems/actuators.h"
#include "state.h"
#include "filters/low_pass_filter.h"
#include "modules/guidance/ctrl_interface.h"

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
float rot_wing_g1_t[8];

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
float rot_wing_max_lift_pitch_eff =0.09;//GUIDANCE_INDI_PITCH_LIFT_EFF;0.035

// Define control effectiveness variables in roll, pitch and yaw for aerodynamic surfaces {AIL_LEFT, AIL_RIGHT, VTAIL_LEFT, VTAIL_RIGHT}
float rot_wing_g1_p_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};
float rot_wing_g1_q_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};
float rot_wing_g1_r_aerodynamic_surf[4] = {0.0, 0.0, 0.0, 0.0};

// Define initialization values
float g2_startup[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1_startup[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                                };

float rot_wing_aileron_limit_deg = 25; // Aileron is effective from this angle value onwards
float rot_wing_roll_prop_limit_deg = 70; // Roll props are not effective anymore from thus value onwards
float rot_wing_pitch_prop_limit_deg = 100; // Pitch props are not effective anymore from thus value onwards
float rot_wing_yaw_prop_limit_deg = 100; // // Yaw props are not effective anymore from thus value onwards
float rot_wing_limit_deadzone_deg = 2; // The deadzone that is put on the wing angle sensor 
int16_t rot_wing_thrust_z_limit = 2; // PPRZ cmd
int16_t rot_wing_thrust_z_deadzone = 0; // PPRZ cmd

bool rot_wing_ailerons_activated; // will be set during initialization
bool rot_wing_roll_props_activated; // Will be set during initialization
bool rot_wing_pitch_props_activated; // Will be set during initialization
bool rot_wing_yaw_props_activated; // Will be set during initialization
bool rot_wing_thrust_z_activated; // Will be set during initialization

float rot_wing_speedz_gain_tuning_constant = GUIDANCE_INDI_SPEED_GAINZ;
float rot_wing_speedz_gain_tuning_gradient = 0.114;

float rot_wing_speed_gain_tuning_constant = GUIDANCE_INDI_SPEED_GAIN;
float rot_wing_speed_gain_tuning_gradient = 0.026;

float rot_wing_pitch_pref_fwd_deg = 10;//;
float rot_wing_pitch_pref_hover_deg = -2;//-2.;

float rot_wing_thrust_z_effectiveness_scale_factor = 2.5;

float lift_estimate= 0.0;

// Define filters
#ifndef ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF
#define ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF 1.5
#endif

Butterworth2LowPass airspeed_lowpass_filter;

// Scheduler function to activate ailerons / roll motors bases
inline void init_active_actuators(void);
inline void evaluate_actuator_active(float airspeed);
inline void schedule_motor_effectiveness(float c_rot_wing_angle, float s_rot_wing_angle);
inline void schedule_aero_effectiveness(float c_rot_wing_angle, float s_rot_wing_angle, float airspeed2);
inline void update_g1g2_matrix(void);
inline void schedule_lift_pitch_eff(float rot_wing_angle_rad, float airspeed2);
inline void schedule_guidance_zgains(float airspeed);
inline void schedule_guidance_hgains(float airspeed);
inline void schedule_pref_pitch_angle(float s_rot_wing_angle);

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
    if((manual_eff)&&(!better_eff)){
          rot_wing_aerodynamic_eff_const_g1_p[0] = roll_ail_L_eff;
          rot_wing_aerodynamic_eff_const_g1_p[1] = roll_ail_R_eff;
          rot_wing_aerodynamic_eff_const_g1_q[0] = pitch_ail_L_eff;
          rot_wing_aerodynamic_eff_const_g1_q[1] = pitch_ail_R_eff;
          rot_wing_aerodynamic_eff_const_g1_q[2] = pitch_elev_eff;
          rot_wing_aerodynamic_eff_const_g1_r[3] = yaw_rud_eff;
    }
    else if (better_eff){
          rot_wing_aerodynamic_eff_const_g1_p[0] = 0; //  PUT ESTIMATED VALUES
          rot_wing_aerodynamic_eff_const_g1_p[1] = 0; //  PUT ESTIMATED VALUES
          rot_wing_aerodynamic_eff_const_g1_q[0] = 0; //  PUT ESTIMATED VALUES
          rot_wing_aerodynamic_eff_const_g1_q[1] = 0; //  PUT ESTIMATED VALUES
          rot_wing_aerodynamic_eff_const_g1_q[2] = 0; //  PUT ESTIMATED VALUES
          rot_wing_aerodynamic_eff_const_g1_r[3] = 0; //  PUT ESTIMATED VALUES
    }
    else{
          rot_wing_aerodynamic_eff_const_g1_p[0] = rot_wing_aero_eff_const_p;
          rot_wing_aerodynamic_eff_const_g1_p[1] = rot_wing_aero_eff_const_p;
          rot_wing_aerodynamic_eff_const_g1_q[2] = -rot_wing_aero_eff_const_q;
          rot_wing_aerodynamic_eff_const_g1_r[3] = rot_wing_aero_eff_const_r;
    }
  #endif

  // perform function that schedules switching on and off actuators based on roll angle
  evaluate_actuator_active(airspeed_lowpass_filter.o[0]);

  // Update motor effectiveness
  schedule_motor_effectiveness(c_rot_wing_angle, s_rot_wing_angle);

  // Update aerodynamic effectiveness
  schedule_aero_effectiveness(c_rot_wing_angle, s_rot_wing_angle, airspeed2);

  // Update pitch lift effectiveness
  schedule_lift_pitch_eff(rot_wing_angle_rad, airspeed2);

  // Schedile pitch pref
  schedule_pref_pitch_angle(s_rot_wing_angle);

  // Update gains
  schedule_guidance_zgains(airspeed_lowpass_filter.o[0]);

  // Update gains
  schedule_guidance_hgains(airspeed_lowpass_filter.o[0]);

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

  rot_wing_thrust_z_activated = true;
}

void evaluate_actuator_active(float airspeed)
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

  // Evaluate z thrust
  if (rot_wing_thrust_z_activated)
  {
    // only deactivate when airspeed is bigger than 5 m/s
    if (airspeed > 5)
    {
      // Loop over actuator 0, 1, 2, 3, if all actuators ppprz_cmds are below the threshold, switch off
      uint8_t counter_motors = 0;
      for (uint8_t i = 0; i < 4; i++)
      {
        if (actuators_pprz[i] < (rot_wing_thrust_z_limit - rot_wing_thrust_z_deadzone))
        {
          counter_motors += 1;
        }
      }
      if (counter_motors == 4)
      {
        rot_wing_thrust_z_activated = false;
      }
      if (airspeed > u_motor_free && motor_free && rot_wing_angle_deg > 80.0){rot_wing_thrust_z_activated = false;}
    }
  } else {
    // activate when below 5 m/s
    if (airspeed < 5)
    {
      rot_wing_thrust_z_activated = true;
    } else if (airspeed > u_motor_free && motor_free && rot_wing_angle_deg > 80.0){rot_wing_thrust_z_activated = false;
    } else {
      // Loop over actuator 0, 1, 2, 3
      for (uint8_t i = 0; i < 3; i++)
      {
        if (actuators_pprz[i] > (rot_wing_thrust_z_limit + rot_wing_thrust_z_deadzone))
        {
          rot_wing_thrust_z_activated = true;
          break;
        }
      }
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

  // thrust
  rot_wing_g1_t[0] = g1_startup[3][0] * rot_wing_thrust_z_effectiveness_scale_factor;
  rot_wing_g1_t[1] = g1_startup[3][1] * rot_wing_thrust_z_effectiveness_scale_factor;
  rot_wing_g1_t[2] = g1_startup[3][2] * rot_wing_thrust_z_effectiveness_scale_factor;
  rot_wing_g1_t[3] = g1_startup[3][3] * rot_wing_thrust_z_effectiveness_scale_factor;

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

  g1_est[3][0] = (g1_est[3][0] + (rot_wing_g1_t[0] - g1_init[3][0]));
  g1_est[3][1] = (g1_est[3][1] + (rot_wing_g1_t[1] - g1_init[3][1]));
  g1_est[3][2] = (g1_est[3][2] + (rot_wing_g1_t[2] - g1_init[3][2]));
  g1_est[3][3] = (g1_est[3][3] + (rot_wing_g1_t[3] - g1_init[3][3]));

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

  g1_init[3][0] = rot_wing_g1_t[0];
  g1_init[3][1] = rot_wing_g1_t[1];
  g1_init[3][2] = rot_wing_g1_t[2];
  g1_init[3][3] = rot_wing_g1_t[3];
}

void schedule_aero_effectiveness(float c_rot_wing_angle, float s_rot_wing_angle, float airspeed2)
{
  // Define Trigonometric functions
  float s3  = s_rot_wing_angle*s_rot_wing_angle*s_rot_wing_angle;
  float cc3 = c_rot_wing_angle - c_rot_wing_angle*c_rot_wing_angle*c_rot_wing_angle;

  // Calculate roll, pitch, yaw effectiveness values
  if (better_ail){
    // Scheduling based on new Skew model
    // roll
    rot_wing_g1_p[4] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[0] * s3 * rot_wing_ailerons_activated;
    rot_wing_g1_p[5] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[1] * s3 * rot_wing_ailerons_activated;
    rot_wing_g1_p[6] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[2];
    rot_wing_g1_p[7] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_p[3];
    
    // pitch
    rot_wing_g1_q[4] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[0] * cc3 * rot_wing_ailerons_activated;
    rot_wing_g1_q[5] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[1] * cc3 * rot_wing_ailerons_activated;
    rot_wing_g1_q[6] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[2];
    rot_wing_g1_q[7] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_q[3];
  } else {
    // Old scheduling  
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
    }

  // yaw
  rot_wing_g1_r[4] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_r[0] * s_rot_wing_angle * rot_wing_ailerons_activated; // eff is 0 so it does not matter
  rot_wing_g1_r[5] = airspeed2 * rot_wing_aerodynamic_eff_const_g1_r[1] * s_rot_wing_angle * rot_wing_ailerons_activated; // eff is 0 so it does not matter
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
        g1g2[3][i] = rot_wing_g1_t[i] / INDI_G_SCALING;
    }
  #endif // STABILIZATION_INDI_USE_ADAPTIVE
} 

void schedule_lift_pitch_eff(float rot_wing_angle_rad, float airspeed2)
{
  if (better_lpe){
  // NEW CODE
  // Lift = 1/2 rho S v^2 [m1*sin(Lambda)^2+k1][m2*theta+k2]
  // Lift = (1/2 rho S v^2)[w1,w2,w3,w4][theta sin(Lambda)^2;sin(Lambda)^2;theta;1]
  // gamma = (1/2 rho S v^2)
  // dL/dtheta = gamma * [w1 sin(Lambda)^2 + w3]
  float gamma = 0.5*1.225*(1.56*0.235)*airspeed2;
  Bound(rot_wing_angle_rad, 0, M_PI/2);
  float s2 = sinf(rot_wing_angle_rad)*sinf(rot_wing_angle_rad);
  lift_pitch_eff = gamma*(-1.8847*s2-1.5037)/3.5;
  
  // Calculate lift if needed for guidance. Lift is NEGATIVE.
  /** state eulers in zxy order */
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  lift_estimate = gamma*(-1.8847*eulers_zxy.theta*s2-0.2780*s2-1.5037*eulers_zxy.theta-0.0043); // [N]
  } else {  
  // OLD CODE
  float lift_pitch_eff_diff = rot_wing_max_lift_pitch_eff - rot_wing_min_lift_pitch_eff;
  float pitch_lift_angle_rad = (rot_wing_angle_rad - M_PI / 6.) * 1.5;
  Bound(pitch_lift_angle_rad, 0, M_PI/2);
  float s_pitch_lift = sinf(pitch_lift_angle_rad);
  lift_pitch_eff = rot_wing_min_lift_pitch_eff + s_pitch_lift * lift_pitch_eff_diff;
  }

}

void schedule_guidance_zgains(float airspeed)
{
  if (airspeed < 4)
  {
    gih_params.pos_gainz = 1.5;
    gih_params.speed_gainz= 1.3;
  } else {
    gih_params.pos_gainz = 1.;
    gih_params.speed_gainz= 2.4;
  }

  //float speed_gainz = rot_wing_speedz_gain_tuning_constant - rot_wing_speedz_gain_tuning_gradient * airspeed;
  // Check if speed_gain_z is not negative or too small, than enter a value of 0.2
  //if (speed_gainz < 0.2)
  //{
    //speed_gainz = 0.2;
  //}
  //gih_params.speed_gainz = speed_gainz;
}

void schedule_guidance_hgains(float airspeed)
{
  if (airspeed < 4)
  {
    gih_params.pos_gain = 0.8;
    gih_params.speed_gain= 0.6;
  } else {
    gih_params.pos_gain = 1.5;
    gih_params.speed_gain= 1.4;
  }
  //float speed_gain = rot_wing_speed_gain_tuning_constant - rot_wing_speed_gain_tuning_gradient * airspeed;
  // Check if speed_gain_z is not negative or too small, than enter a value of 0.2
  //if (speed_gain < 0.2)
  //{
    //speed_gain = 0.2;
  //}
  //gih_params.speed_gain = speed_gain;
}

void schedule_pref_pitch_angle(float s_rot_wing_angle)
{
  float pitch_pref_range_deg = rot_wing_pitch_pref_fwd_deg - rot_wing_pitch_pref_hover_deg;

  // Bound rot_wing_angle
  if (s_rot_wing_angle < 0)
  {
    s_rot_wing_angle = 0;
  }

  // Schedule prefered pitch angle
  float pitch_diff_deg = pitch_pref_range_deg * s_rot_wing_angle;
  pitch_pref_deg = rot_wing_pitch_pref_hover_deg + pitch_diff_deg;
}