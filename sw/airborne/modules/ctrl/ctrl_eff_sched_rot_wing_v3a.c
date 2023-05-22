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

/** @file "modules/ctrl/ctrl_eff_sched_rot_wing_v3a.c"
 * @author D.C. van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Crtl effectiveness scheduler for thr rotating wing drone V3
 */

#include "modules/ctrl/ctrl_eff_sched_rot_wing_v3a.h"

#include "modules/rot_wing_drone/wing_rotation_controller_v3a.h"

#include "modules/actuators/actuators.h"

#include "modules/core/abi.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_rot_wing.h"

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

bool wing_rotation_sched_activated = true;
bool pusher_sched_activated = true;

float sched_pitch_hover_deg = 0.;
float sched_pitch_forward_deg = 10.;
float sched_lower_hover_speed = 7.;
float sched_upper_hover_speed = 14.;

// Define filters
#ifndef ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF
#define ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF 0.1
#endif

Butterworth2LowPass airspeed_lowpass_filter;

// Define scheduling constants
const float k_elevator[3] = {0.4603,  -4.81466, -28.8464};
const float k_rudder[3] = {-26.1434, -0.336403, -1.16702 };
const float k_pusher[2] = {0.007777, -0.67521};

float I_xx = 0.18707079;
float I_yy = 1.04;
float I_zz = 1.14;
const float weight_sched = 6.5;

inline void update_inertia(float *cosr2, float *sinr2);
inline void update_hover_motor_effectiveness(float *sk, float *cosr, float *sinr, float *airspeed_f);
inline void update_elevator_effectiveness(int16_t *elev_pprz, float *airspeed, float *airspeed2, float *pp_scaled);
inline void update_rudder_effectiveness(float *airspeed2, float *pp_scaled, float *T_mean_scaled, float *cosr);
inline void update_left_aileron_effectiveness(float *airspeed2, float *sinr);
inline void update_right_aileron_effectiveness(float *airspeed2, float *sinr);
inline void update_pusher_effectiveness(float *airspeed_f, float pusher_cmd_filt);
// inline void schedule_pref_pitch_angle_deg(float *airspeed_f);
// inline void schedule_pref_pitch_angle_deg(float *sinr);
inline void schedule_pref_pitch_angle_deg(float wing_rot_deg);
inline void schedule_liftd(float *airspeed2, float *sinr);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_thrust_bx_eff(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_THRUST_BX_EFF(trans, dev, AC_ID,
                              &thrust_bx_eff);
}
#endif

void init_eff_scheduling(void)
{
  // Copy initial effectiveness on roll for side motors
  rot_wing_side_motors_g1_p_0[0] = g1_startup[0][1];
  rot_wing_side_motors_g1_p_0[1] = g1_startup[0][3];

  // Init filters
  float tau = 1.0 / (2.0 * M_PI * ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  init_butterworth_2_low_pass(&airspeed_lowpass_filter, tau, sample_time, 0.0);

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_THRUST_BX_EFF, send_thrust_bx_eff);
  #endif
}

void event_eff_scheduling(void)
{
  // Update relevant states
  // Update airspeed
  float airspeed = stateGetAirspeed_f();
  update_butterworth_2_low_pass(&airspeed_lowpass_filter, airspeed);
  float airspeed2 = airspeed * airspeed;//airspeed_lowpass_filter.o[0] * airspeed_lowpass_filter.o[0];

  // Update skew angle and triogeometric variables of wing rotation
  float cosr;
  float sinr;
  float cosr2;
  float sinr2;
  float wing_rotation_deg = wing_rotation.wing_angle_deg;
  if (wing_rotation_sched_activated) {
    cosr = cosf(wing_rotation.wing_angle_rad);
    sinr = sinf(wing_rotation.wing_angle_rad);
    cosr2 = cosr * cosr;
    sinr2 = sinr * sinr;
  } else {
    cosr = 1;
    cosr2 = 1;
    sinr = 0;
    sinr2 = 0;
  }

  // Update actuator states
  int16_t *elev_pprz = &actuators_pprz[5];

  float pp_scaled = actuators_pprz[6] / MAX_PPRZ * 8191. / 1000.;
  float T_mean_scaled = (actuators_pprz[0] + actuators_pprz[1] + actuators_pprz[2] + actuators_pprz[3]) / 4. / MAX_PPRZ * 8191. / 1000.;

  // Calculate deflection of elevator
  
  update_inertia(&cosr2, &sinr2);
  update_hover_motor_effectiveness(&wing_rotation.wing_angle_rad, &cosr, &sinr, &airspeed);
  update_rudder_effectiveness(&airspeed2, &pp_scaled, &T_mean_scaled, &cosr);
  update_elevator_effectiveness(elev_pprz, &airspeed, &airspeed2, &pp_scaled);
  update_left_aileron_effectiveness(&airspeed2, &sinr);
  update_right_aileron_effectiveness(&airspeed2, &sinr);
  update_pusher_effectiveness(&airspeed, thrust_bx_state_filt);
  //schedule_pref_pitch_angle_deg(&sinr);
  schedule_pref_pitch_angle_deg(wing_rotation_deg);
  schedule_liftd(&airspeed2, &sinr2);

  // float g1_p_side_motors[2];
  // float g1_q_side_motors[2];

  // // Calculate roll and pitch effectiveness of the two roll side motors
  // g1_p_side_motors[0] = rot_wing_side_motors_g1_p_0[0] * cosr;
  // g1_p_side_motors[1] = rot_wing_side_motors_g1_p_0[1] * cosr;

  // g1_q_side_motors[0] = rot_wing_side_motors_g1_q_90[0] * sinr;
  // g1_q_side_motors[1] = rot_wing_side_motors_g1_q_90[1] * sinr;

  // // Update inner loop effectiveness matrix for motors
  // g1g2[0][0] = g1_startup[0][0] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][0] = g1_startup[1][0] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][0] = (g1_startup[2][0] * g1_r_multiplier + g2_startup[0]) / INDI_G_SCALING;
  // g1g2[3][0] = g1_startup[3][0] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][1] = g1_p_side_motors[0] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][1] = g1_q_side_motors[0] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][1] = (g1_startup[2][1] * g1_r_multiplier + g2_startup[1]) / INDI_G_SCALING;
  // g1g2[3][1] = g1_startup[3][1] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][2] = g1_startup[0][2] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][2] = g1_startup[1][2] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][2] = (g1_startup[2][2] * g1_r_multiplier + g2_startup[2]) / INDI_G_SCALING;
  // g1g2[3][2] = g1_startup[3][2] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][3] = g1_p_side_motors[1] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][3] = g1_q_side_motors[1] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][3] = (g1_startup[2][3] * g1_r_multiplier + g2_startup[3]) / INDI_G_SCALING;
  // g1g2[3][3] = g1_startup[3][3] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][4] = rot_wing_aerodynamic_eff_const_g1_p[0] * airspeed2 / INDI_G_SCALING;

  // g1g2[1][4] = rot_wing_aerodynamic_eff_const_g1_q[0] * airspeed2 / INDI_G_SCALING;

  // g1g2[2][4] = rot_wing_aerodynamic_eff_const_g1_r[0] * airspeed2 / INDI_G_SCALING;

}

void update_inertia(float *cosr2, float *sinr2)
{
  // Inertia with wing
  I_xx = 0.102529209767747 * *sinr2 + 0.0422707902322535 * *cosr2 + 0.1448;
  I_yy = 0.303551738402111 * *sinr2 + 0.366381594931223 * *cosr2 + 0.669933333333333;
  Bound(I_xx, 0.0001, 100);
  Bound(I_yy, 0.0001, 100);
}

void update_hover_motor_effectiveness(float *sk, float *cosr, float *sinr, float *airspeed_f)
{
  float g1_p_side_motors[2];
  float g1_q_side_motors[2];

  float bounded_airspeed = *airspeed_f;
  Bound(bounded_airspeed, 0, 20);

  // Calculate roll and pitch effectiveness of the two roll side motors
  g1_p_side_motors[0] = rot_wing_side_motors_g1_p_0[0] * *cosr;
  g1_p_side_motors[1] = rot_wing_side_motors_g1_p_0[1] * *cosr;

  g1_q_side_motors[0] = rot_wing_side_motors_g1_q_90[0] * *sinr;
  g1_q_side_motors[1] = rot_wing_side_motors_g1_q_90[1] * *sinr;

  // Update inner loop effectiveness matrix for motors
  g1g2[0][0] = g1_startup[0][0] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][0] = g1_startup[1][0] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][0] = (g1_startup[2][0] * g1_r_multiplier + g2_startup[0]) / INDI_G_SCALING;
  g1g2[3][0] = g1_startup[3][0] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][1] = g1_p_side_motors[0] * g1_p_multiplier / INDI_G_SCALING;
  //g1g2[0][1] = ((-0.00335545 - 0.00214628 * *cosr + 0.00369705 * *sinr + -0.00302647 * *cosr * *cosr + -0.00032898 * *sinr * *sinr)) * g1_p_multiplier;
  // For pprz_cmd = 5500
  g1g2[0][1] = ((0.000072804595016592 * *sk * *sk * bounded_airspeed * *cosr - 0.00129490506761079 * *cosr) / I_xx) * g1_p_multiplier;
  Bound(g1g2[0][1], -1, -0.00001);
  //g1g2[1][1] = g1_q_side_motors[0] * g1_q_multiplier / INDI_G_SCALING;
  // For pprz_cmd = 5500
  g1g2[1][1] = ((-0.000563317785007957 * *sk * *sk * *sinr + 0.00123052200974951 * *sk) / I_yy) * g1_q_multiplier;
  Bound(g1g2[1][1], 0, 1);
  g1g2[2][1] = (g1_startup[2][1] * g1_r_multiplier + g2_startup[1]) / INDI_G_SCALING;
  g1g2[3][1] = g1_startup[3][1] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][2] = g1_startup[0][2] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][2] = g1_startup[1][2] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][2] = (g1_startup[2][2] * g1_r_multiplier + g2_startup[2]) / INDI_G_SCALING;
  g1g2[3][2] = g1_startup[3][2] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][3] = (g1_p_side_motors[1] * g1_p_multiplier - 0.283333 * bounded_airspeed * *cosr) / INDI_G_SCALING;
  //g1g2[0][3] = ((0.0040856 + 0.00123478 * *cosr + -0.00428635 * *sinr + 0.00390033 * *cosr * *cosr + 0.00018527 * *sinr * *sinr)) * g1_p_multiplier;
  // For pprz_cmd = 5500
  g1g2[0][3] = -((0.000072804595016592 * *sk * *sk * bounded_airspeed * *cosr - 0.00129490506761079 * *cosr + 0.000053954 * bounded_airspeed * *cosr) / I_xx) * g1_p_multiplier;
  Bound(g1g2[0][3], 0.00001, 1);
  //g1g2[1][3] = g1_q_side_motors[1] * g1_q_multiplier / INDI_G_SCALING;
  // For pprz_cmd = 5500
  g1g2[1][3] = -((-0.000563317785007957 * *sk * *sk * *sinr + 0.00123052200974951 * *sk) / I_yy) * g1_q_multiplier;
  Bound(g1g2[1][3], -1, 0);
  g1g2[2][3] = (g1_startup[2][3] * g1_r_multiplier + g2_startup[3]) / INDI_G_SCALING;
  g1g2[3][3] = g1_startup[3][3] * g1_t_multiplier / INDI_G_SCALING;
}

void update_elevator_effectiveness(int16_t *elev_pprz, float *airspeed, float *airspeed2, float *pp_scaled)
{
  // Calculate deflection angle in [deg]
  float de = -0.004885417 * *elev_pprz + 36.6;
  float bounded_airspeed = *airspeed;
  float bounded_airspeed2 = *airspeed2; 
  Bound(bounded_airspeed, 0. ,20.);
  Bound(bounded_airspeed2, 0., 400.);

  float dMyde = (k_elevator[0] * de * bounded_airspeed2 +
                k_elevator[1] * *pp_scaled * *pp_scaled * bounded_airspeed + 
                k_elevator[2] * bounded_airspeed2) / 10000.;

  float dMydpprz = dMyde * -0.004885417;
  
  // Convert moment to effectiveness
  float eff_y_elev = dMydpprz / I_yy;

  Bound(eff_y_elev, 0.00001, 0.1);

  g1g2[0][5] = 0;
  g1g2[1][5] = eff_y_elev;
  g1g2[2][5] = 0;
  g1g2[3][5] = 0;
  
}

void update_rudder_effectiveness(float *airspeed2, float *pp_scaled, float *T_mean_scaled, float *cosr)
{
  float bounded_airspeed2 = *airspeed2;
  Bound(bounded_airspeed2, 0., 400.);
  float dMzdr = (k_rudder[0] * *pp_scaled * *T_mean_scaled + 
                k_rudder[1] * *T_mean_scaled * bounded_airspeed2 * *cosr + 
                k_rudder[2] * bounded_airspeed2) / 10000.;

  // Convert moment to effectiveness

  float dMzdpprz = dMzdr * -0.0018;

  // Convert moment to effectiveness
  float eff_z_rudder = dMzdpprz / I_zz;

  Bound(eff_z_rudder, 0.00001, 0.1);

  g1g2[0][4] = 0 / INDI_G_SCALING;
  g1g2[1][4] = 0 / INDI_G_SCALING;
  g1g2[2][4] = eff_z_rudder;
  g1g2[3][4] = 0 / INDI_G_SCALING;
}

void update_left_aileron_effectiveness(float *airspeed2, float *sinr)
{
  float bounded_airspeed2 = *airspeed2;
  Bound(bounded_airspeed2, 0., 400.);
  float dMxdpprz = 3.46740494791667e-6 * bounded_airspeed2 * *sinr * *sinr * *sinr;
  float eff_x_left_aileron = dMxdpprz / I_xx;
  Bound(eff_x_left_aileron, 0, 0.005);
  g1g2[0][6] = eff_x_left_aileron;
}

void update_right_aileron_effectiveness(float *airspeed2, float *sinr)
{
  float bounded_airspeed2 = *airspeed2;
  Bound(bounded_airspeed2, 0., 400.);
  float dMxdpprz = 3.33667052083333e-6 * bounded_airspeed2 * *sinr * *sinr * *sinr;
  float eff_x_right_aileron = dMxdpprz / I_xx;
  Bound(eff_x_right_aileron, 0, 0.005)
  g1g2[0][7] = eff_x_right_aileron;
}

void update_pusher_effectiveness(float *airspeed_f, float pusher_cmd_filt)
{
  if (pusher_sched_activated)
  {
    float bounded_airspeed = *airspeed_f;
    Bound(bounded_airspeed, 0., 20.);

    float rpmP = -2.91178067445214e-5*pusher_cmd_filt*pusher_cmd_filt + 1.32098226269777*pusher_cmd_filt - 131.497033952591;
    float dFxdrpmP = k_pusher[0]*rpmP + k_pusher[1]* bounded_airspeed;
    float drpmPdpprz = 1.32098226269777 - 5.82356134890428e-5*pusher_cmd_filt;

    float eff_pusher = (dFxdrpmP * drpmPdpprz / weight_sched) / 10000.;

  Bound(eff_pusher, 0.00030, 0.0015);
  thrust_bx_eff = eff_pusher;
  } else {
    thrust_bx_eff = STABILIZATION_INDI_PUSHER_PROP_EFFECTIVENESS;
  }
}

// void schedule_pref_pitch_angle_deg(float *airspeed_f)
// {
//   if (*airspeed_f < sched_lower_hover_speed) {
//     pitch_pref_deg = sched_pitch_hover_deg;
//   } else if (*airspeed_f > sched_upper_hover_speed) {
//     pitch_pref_deg = sched_pitch_forward_deg;
//   } else {
//     float airspeed_sched_range = sched_upper_hover_speed - sched_lower_hover_speed;
//     float pitch_sched_range = sched_pitch_forward_deg - sched_pitch_hover_deg;
//     Bound(airspeed_sched_range,0.01, 25.);
//     pitch_pref_deg = (1. - (*airspeed_f - sched_lower_hover_speed) / airspeed_sched_range) * pitch_sched_range + sched_pitch_hover_deg;
//     Bound(pitch_pref_deg, -10, 10);
//   }
// }

// void schedule_pref_pitch_angle_deg(float *sinr)
// {
//   float pitch_pref_range_deg = sched_pitch_forward_deg - sched_pitch_hover_deg;

//   // Schedule prefered pitch angle
//   float pitch_diff_deg = pitch_pref_range_deg * *sinr;
//   pitch_pref_deg = sched_pitch_hover_deg + pitch_diff_deg;
// }

void schedule_pref_pitch_angle_deg(float wing_rot_deg)
{
  float scheduled_pitch_angle = 0;
  if (wing_rot_deg < 55) {
    scheduled_pitch_angle = 0;
  } else {
    float pitch_range = 7.;
    float pitch_progression = (wing_rot_deg - 55) / 35.;
    scheduled_pitch_angle = pitch_range * pitch_progression;
  }
  Bound(scheduled_pitch_angle, 0., 7.);
  pitch_pref_deg = scheduled_pitch_angle;
}

void schedule_liftd(float *airspeed2, float *sinr2)
{
  float bounded_airspeed2 = *airspeed2;
  Bound(bounded_airspeed2, 0., 400.);
  float lift_d_wing = (-0.74529194103945 * bounded_airspeed2 * *sinr2 - 0.4065513216373 * bounded_airspeed2) / weight_sched;
  float lift_d_fuselage = -0.072362752875 * bounded_airspeed2 / weight_sched;
  float lift_d_tail = -0.1452739306305 * bounded_airspeed2 / weight_sched;

  float lift_d = lift_d_wing + lift_d_fuselage + lift_d_tail;
  Bound(lift_d, -130., 0.);
  AbiSendMsgLIFT_D(LIFT_D_SCHED_ID, lift_d);
}