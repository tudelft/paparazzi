/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rotwing/rotwing_state.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module keeps track of the current state of a rotating wing drone and desired state set by the RC or flightplan. Paramters are being scheduled in each change of a current state and desired state. Functions are defined in this module to call the actual state and desired state and set a desired state.
 */

#include "modules/rot_wing_drone/rotwing_state.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"

/* Minimum RPM to consider the hover motor running */
#ifndef ROTWING_HOV_MOT_RUN_RPM_TH
#define ROTWING_HOV_MOT_RUN_RPM_TH 1000
#endif

/* Minimum RPM to consider the pusher motor running */
#ifndef ROTWING_PUSH_MOT_RUN_RPM_TH
#define ROTWING_PUSH_MOT_RUN_RPM_TH 1000
#endif


/** ABI binding feedback data.
 */
#ifndef ROTWING_STATE_ACT_FEEDBACK_ID
#define ROTWING_STATE_ACT_FEEDBACK_ID ABI_BROADCAST
#endif
abi_event rotwing_state_feedback_ev;
static void rotwing_state_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback_msg, uint8_t num_act);
struct rotwing_state_t rotwing_state;


inline void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);
static void rotwing_state_quad_motors(void);
static void rotwing_state_skew(void);
static void rotwing_state_airspeed(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_rotating_wing_state(struct transport_tx *trans, struct link_device *dev)
{
  /*uint16_t adc_dummy = 0;
  pprz_msg_send_ROTATING_WING_STATE(trans, dev, AC_ID,
                                    &rotwing_state.current_state,
                                    &rotwing_state.desired_state,
                                    &rotwing_state_skewing.wing_angle_deg,
                                    &rotwing_state_skewing.wing_angle_deg_sp,
                                    &adc_dummy,
                                    &rotwing_state_skewing.servo_pprz_cmd);*/
}
#endif // PERIODIC_TELEMETRY


void rotwing_state_init(void)
{
  // Initialize rotwing state
  rotwing_state.meas_wing_angle_deg = 0;
  for (int i = 0; i < 5; i++) {
    rotwing_state.meas_rpm[i] = 0;
  }

  // Bind ABI messages
  AbiBindMsgACT_FEEDBACK(ROTWING_STATE_ACT_FEEDBACK_ID, &rotwing_state_feedback_ev, rotwing_state_feedback_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTATING_WING_STATE, send_rotating_wing_state);
#endif
}

void rotwing_state_periodic(void)
{



}


void rotwing_state_skew_actuator_periodic(void)
{
//   // calc rotation percentage of setpoint (0 deg = -1, 45 deg = 0, 90 deg = 1)
//   float wing_rotation_percentage = (rotwing_state_skewing.wing_angle_deg_sp - 45.) / 45.;
//   Bound(wing_rotation_percentage, -1., 1.);

//   float servo_pprz_cmd = MAX_PPRZ * wing_rotation_percentage;
//   // Calulcate rotation_cmd
//   Bound(servo_pprz_cmd, -MAX_PPRZ, MAX_PPRZ);

// #if ROTWING_STATE_USE_ROTATION_REF_MODEL
//   // Rotate with second order filter
//   static float rotwing_state_skew_p_cmd = -MAX_PPRZ;
//   static float rotwing_state_skew_d_cmd = 0;
//   float speed_sp  = 0.001 * (servo_pprz_cmd - rotwing_state_skew_p_cmd);
//   rotwing_state_skew_d_cmd += 0.003 * (speed_sp - rotwing_state_skew_d_cmd);
//   rotwing_state_skew_p_cmd += rotwing_state_skew_d_cmd;
//   Bound(rotwing_state_skew_p_cmd, -MAX_PPRZ, MAX_PPRZ);
//   rotwing_state_skewing.servo_pprz_cmd = rotwing_state_skew_p_cmd;
// #else
//   // Directly controlling the wing rotation
//   rotwing_state_skewing.servo_pprz_cmd = servo_pprz_cmd;
// #endif

// #if USE_NPS
//   // Export to the index of the SKEW in the NPS_ACTUATOR_NAMES array
//   actuators_pprz[INDI_NUM_ACT] = (rotwing_state_skewing.servo_pprz_cmd + MAX_PPRZ) / 2.; // Scale to simulation command

//   // Simulate wing angle from command
//   rotwing_state_skewing.wing_angle_deg = (float) rotwing_state_skewing.servo_pprz_cmd / MAX_PPRZ * 45. + 45.;

//   // SEND ABI Message to ctr_eff_sched and other modules that want Actuator position feedback
//   struct act_feedback_t feedback;
//   feedback.idx =  SERVO_ROTATION_MECH_IDX;
//   feedback.position = 0.5 * M_PI - RadOfDeg(rotwing_state_skewing.wing_angle_deg);
//   feedback.set.position = true;

//   // Send ABI message
//   AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
// #endif
}

static void rotwing_state_feedback_cb(uint8_t __attribute__((unused)) sender_id,
                                      struct act_feedback_t UNUSED *feedback_msg, uint8_t UNUSED num_act_message)
{
  for (int i = 0; i < num_act_message; i++) {

    for (int i = 0; i < num_act_message; i++) {
      // Check for wing rotation feedback
      if ((feedback_msg[i].set.position) && (feedback_msg[i].idx == SERVO_ROTATION_MECH_IDX)) {
        // Get wing rotation angle from sensor
        float wing_angle_rad = 0.5 * M_PI - feedback_msg[i].position;
        //rotwing_state_skewing.wing_angle_deg = DegOfRad(wing_angle_rad);

        // Bound wing rotation angle
        //Bound(rotwing_state_skewing.wing_angle_deg, 0, 90.);
      }
    }

    // Sanity check that index is valid
    int idx = feedback_msg[i].idx;
    if (feedback_msg[i].set.rpm) {
      if ((idx == SERVO_MOTOR_FRONT_IDX) || (idx == SERVO_BMOTOR_FRONT_IDX)) {
        rotwing_state.meas_rpm[0] = feedback_msg->rpm;
      } else if ((idx == SERVO_MOTOR_RIGHT_IDX) || (idx == SERVO_BMOTOR_RIGHT_IDX)) {
        rotwing_state.meas_rpm[1] = feedback_msg->rpm;
      } else if ((idx == SERVO_MOTOR_BACK_IDX) || (idx == SERVO_BMOTOR_BACK_IDX)) {
        rotwing_state.meas_rpm[2] = feedback_msg->rpm;
      } else if ((idx == SERVO_MOTOR_LEFT_IDX) || (idx == SERVO_BMOTOR_LEFT_IDX)) {
        rotwing_state.meas_rpm[3] = feedback_msg->rpm;
      } else if ((idx == SERVO_MOTOR_PUSH_IDX) || (idx == SERVO_BMOTOR_PUSH_IDX)) {
        rotwing_state.meas_rpm[4] = feedback_msg->rpm;
      }
    }
  }
}

bool rotwing_state_hover_motors_running(void) {
  // Check if hover motors are running
  if (rotwing_state.meas_rpm[0] > ROTWING_HOV_MOT_RUN_RPM_TH
      && rotwing_state.meas_rpm[1] > ROTWING_HOV_MOT_RUN_RPM_TH
      && rotwing_state.meas_rpm[2] > ROTWING_HOV_MOT_RUN_RPM_TH
      && rotwing_state.meas_rpm[3] > ROTWING_HOV_MOT_RUN_RPM_TH) {
    return true;
  } else {
    return false;
  }
}

bool rotwing_state_pusher_motor_running(void) {
  // Check if pusher motor is running
  if (rotwing_state.meas_rpm[4] > ROTWING_PUSH_MOT_RUN_RPM_TH) {
    return true;
  } else {
    return false;
  }
}

void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle)
{
//   // adjust weights
//   float fixed_wing_percentage = !hover_motors_active; // TODO: when hover props go below 40%, ...
//   Bound(fixed_wing_percentage, 0, 1);
// #define AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT 16

//   float Wv_original[GUIDANCE_INDI_HYBRID_V] = GUIDANCE_INDI_WLS_PRIORITIES;

//   // Increase importance of forward acceleration in forward flight
//   Wv_gih[0] = Wv_original[0] * (1.0f + fixed_wing_percentage *
//                                          AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT); // stall n low hover motor_off (weight 16x more important than vertical weight)

//   struct FloatEulers eulers_zxy;
//   float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

//   // Evaluate motors_on boolean
//   if (!hover_motors_active) {
//     if (stateGetAirspeed_f() < 15.) {
//       hover_motors_active = true;
//       bool_disable_hover_motors = false;
//     } else if (eulers_zxy.theta > RadOfDeg(15.0)) {
//       hover_motors_active = true;
//       bool_disable_hover_motors = false;
//     }
//   } else {
//     bool_disable_hover_motors = false;
//   }

//   float du_min_thrust_z = ((MAX_PPRZ - actuator_state_filt_vect[0]) * g1g2[3][0] + (MAX_PPRZ -
//                            actuator_state_filt_vect[1]) * g1g2[3][1] + (MAX_PPRZ - actuator_state_filt_vect[2]) * g1g2[3][2] +
//                            (MAX_PPRZ - actuator_state_filt_vect[3]) * g1g2[3][3]) * hover_motors_active;
//   Bound(du_min_thrust_z, -50., 0.);
//   float du_max_thrust_z = -(actuator_state_filt_vect[0] * g1g2[3][0] + actuator_state_filt_vect[1] * g1g2[3][1] +
//                             actuator_state_filt_vect[2] * g1g2[3][2] + actuator_state_filt_vect[3] * g1g2[3][3]);
//   Bound(du_max_thrust_z, 0., 50.);

//   float roll_limit_rad = guidance_indi_max_bank;
//   float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
//   float min_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MIN_PITCH);

//   float fwd_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
//   float quad_pitch_limit_rad = RadOfDeg(5.0);

//   float airspeed = stateGetAirspeed_f();

//   float scheduled_pitch_angle = 0.f;
//   float pitch_angle_range = 3.;
//   if (rotwing_state_skewing.wing_angle_deg < 55) {
//     scheduled_pitch_angle = guidance_indi_pitch_pref_deg_quad;
//     Wu_gih[1] = Wu_gih_original[1];
//     max_pitch_limit_rad = quad_pitch_limit_rad;
//   } else {
//     float pitch_progression = (airspeed - ROTWING_STATE_FW_SKEW_SPEED) / 2.f;
//     Bound(pitch_progression, 0.f, 1.f);
//     scheduled_pitch_angle = pitch_angle_range * pitch_progression + guidance_indi_pitch_pref_deg_quad*(1.f-pitch_progression);
//     Wu_gih[1] = Wu_gih_original[1] * (1.f - pitch_progression*0.99);
//     max_pitch_limit_rad = quad_pitch_limit_rad + (fwd_pitch_limit_rad - quad_pitch_limit_rad) * pitch_progression;
//   }
//   if (!hover_motors_active) {
//     scheduled_pitch_angle = 8.;
//     max_pitch_limit_rad = fwd_pitch_limit_rad;
//   }
//   Bound(scheduled_pitch_angle, -5., 8.);
//   guidance_indi_pitch_pref_deg = scheduled_pitch_angle;

//   float pitch_pref_rad = RadOfDeg(guidance_indi_pitch_pref_deg);

//   // Set lower limits
//   du_min_gih[0] = -roll_limit_rad - roll_angle; //roll
//   du_min_gih[1] = min_pitch_limit_rad - pitch_angle; // pitch
//   du_min_gih[2] = du_min_thrust_z;
//   du_min_gih[3] = (-actuator_state_filt_vect[8] * g1g2[4][8]);

//   // Set upper limits limits
//   du_max_gih[0] = roll_limit_rad - roll_angle; //roll
//   du_max_gih[1] = max_pitch_limit_rad - pitch_angle; // pitch
//   du_max_gih[2] = du_max_thrust_z;
//   du_max_gih[3] = 9.0; // Hacky value to prevent drone from pitching down in transition

//   // Set prefered states
//   du_pref_gih[0] = 0; // prefered delta roll angle
//   du_pref_gih[1] = -pitch_angle + pitch_pref_rad;// prefered delta pitch angle
//   du_pref_gih[2] = du_max_gih[2]; // Low thrust better for efficiency
//   du_pref_gih[3] = body_v[0]; // solve the body acceleration
}

void rotwing_state_request_hover(void) {

}

void rotwing_state_request_free(void) {

}

bool rotwing_state_hover_motors_disable(void) {
  return false;
}