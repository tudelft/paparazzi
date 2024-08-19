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

/* Minimum measured RPM to consider the hover motors running (RPM) */
#ifndef ROTWING_QUAD_MIN_RPM
#define ROTWING_QUAD_MIN_RPM 800
#endif

/* Minimum measured RPM to consider the pusher motor running (RPM) */
#ifndef ROTWING_PUSH_MIN_RPM
#define ROTWING_PUSH_MIN_RPM 500
#endif

/* Timeout for the RPM feedback (seconds) */
#ifndef ROTWING_RPM_TIMEOUT
#define ROTWING_RPM_TIMEOUT 0.2
#endif

/* Minimum thrust to consider the hover motors idling (PPRZ) */
#ifndef ROTWING_QUAD_IDLE_MIN_THRUST
#define ROTWING_QUAD_IDLE_MIN_THRUST 100
#endif

/* Minimum time for the hover motors to be considered idling (seconds) */
#ifndef ROTWING_QUAD_IDLE_TIMEOUT
#define ROTWING_QUAD_IDLE_TIMEOUT 2.0
#endif

/* Stall speed of the fixed wing (m/s) */
//#ifndef ROTWING_STALL_SPEED
//#define ROTWING_STALL_SPEED 15.0
//#endif

/* Margin for the stall speed (percentile) */
#ifndef ROTWING_STALL_MARGIN
#define ROTWING_STALL_MARGIN 0.2
#endif

/* Minimum measured skew angle to consider the rotating wing in fixedwing (deg) */
#ifndef ROTWING_FW_SKEW_ANGLE
#define ROTWING_FW_SKEW_ANGLE 85.0
#endif

/* Maximum airspeed which we can fly in quad mode, 0 skew angle (m/s) */
// #ifndef ROTWING_QUAD_MAX_AIRSPEED
// #define ROTWING_QUAD_MAX_AIRSPEED 15.0
// #endif

/* Maximum airspeed which we can fly in fixedwing mode, 90 skew angle (m/s) */
// #ifndef ROTWING_FW_MAX_AIRSPEED
// #define ROTWING_FW_MAX_AIRSPEED 25.0
// #endif

/* Maximum airspeed which we can fly in quad without pusher, 0 skew angle (m/s) */
// #ifndef ROTWING_QUAD_NOPUSH_AIRSPEED
// #define ROTWING_QUAD_NOPUSH_AIRSPEED 8.0
// #endif

/* Sanity checks */
#if ROTWING_SKEW_START_AIRSPEED > ROTWING_QUAD_MAX_AIRSPEED
#error "ROTWING_SKEW_START_AIRSPEED cannot be higher than ROTWING_QUAD_MAX_AIRSPEED"
#endif

/* Sanity checks */
#if ROTWING_SKEW_FW_AIRSPEED < ROTWING_SKEW_START_AIRSPEED
#error "ROTWING_SKEW_FW_AIRSPEED cannot be lower than ROTWING_SKEW_START_AIRSPEED"
#endif

/** ABI binding feedback data */
#ifndef ROTWING_STATE_ACT_FEEDBACK_ID
#define ROTWING_STATE_ACT_FEEDBACK_ID ABI_BROADCAST
#endif
abi_event rotwing_state_feedback_ev;
static void rotwing_state_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback_msg, uint8_t num_act);

static bool rotwing_state_hover_motors_idling(void);
struct rotwing_state_t rotwing_state;

inline void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);

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
  rotwing_state.request_hover = true;
  rotwing_state.meas_wing_angle_deg = 0;
  rotwing_state.meas_wing_angle_time = 0;
  for (int i = 0; i < 5; i++) {
    rotwing_state.meas_rpm[i] = 0;
    rotwing_state.meas_rpm_time[i] = 0;
  }

  // Bind ABI messages
  AbiBindMsgACT_FEEDBACK(ROTWING_STATE_ACT_FEEDBACK_ID, &rotwing_state_feedback_ev, rotwing_state_feedback_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTATING_WING_STATE, send_rotating_wing_state);
#endif
}

/**
 * @brief Check if hover motors are idling (COMMAND_THRUST < ROTWING_QUAD_IDLE_MIN_THRUST) for ROTWING_QUAD_IDLE_TIMEOUT time
 * @return true if hover motors are idling, false otherwise
 */
static bool rotwing_state_hover_motors_idling(void) {
  static float last_idle_time = 0;
  // Check if hover motors are idling and reset timer
  if(stabilization.cmd[COMMAND_THRUST] > ROTWING_QUAD_IDLE_MIN_THRUST) {
    last_idle_time = get_sys_time_float();
  }

  // Check if we exceeded the idle timeout
  if(get_sys_time_float() - last_idle_time > ROTWING_QUAD_IDLE_TIMEOUT) {
    return true;
  }
  return false;
}

void rotwing_state_periodic(void)
{
  float meas_airspeed = stateGetAirspeed_f();
  const float min_fwd_airspeed = ROTWING_STALL_SPEED * (1 + ROTWING_STALL_MARGIN);
  

  /* Handle the quad motors on/off state */
  if(rotwing_state.request_hover) {
    rotwing_state.quad_motors_enabled = true;
  }
  
  //Vair > (Vstall + margin) && Qcmd <= (idle + margin) && wing_angle >= (fw_angle -margin) && Vnav >= (Vstall + margin)
  else if(meas_airspeed > min_fwd_airspeed && rotwing_state_hover_motors_idling() && rotwing_state.meas_wing_angle_deg >= ROTWING_FW_SKEW_ANGLE) { //  && nav_airspeed >= min_fwd_airspeed
    rotwing_state.quad_motors_enabled = false;
  }
  else {
    rotwing_state.quad_motors_enabled = true;
  }

  /* Handle the skeq angle setpoint */
  if(!rotwing_state_hover_motors_running()) {
    // TODO: this does not work with takeoff!!
    rotwing_state.sp_wing_angle_deg = 90;
  }
  else if(!rotwing_state_pusher_motor_running()) {
    rotwing_state.sp_wing_angle_deg = 0;
  }
  else if(rotwing_state.request_hover && meas_airspeed < ROTWING_QUAD_MAX_AIRSPEED) {
    rotwing_state.sp_wing_angle_deg = 0;
  }
  else {
    // SKEWING function based on Vair and maybe Vnav
    float wing_angle_scheduled_sp_deg = 0;
    float airspeed = stateGetAirspeed_f();
    
    // Airspeed scheduled skewing logic, 0 degrees if airspeed < ROTWING_STATE_QUAD_MAX_SPEED
    // 55 degrees if ROTWING_STATE_QUAD_MAX_SPEED < airspeed < ROTWING_SKEW_START_AIRSPEED
    // linear scaling between 55 and 90 degrees when airspeed > ROTWING_SKEW_START_AIRSPEED
    if (airspeed < ROTWING_SKEW_START_AIRSPEED) {
      wing_angle_scheduled_sp_deg = 0;

    } else if (airspeed < ROTWING_QUAD_MAX_AIRSPEED) {
      wing_angle_scheduled_sp_deg = 55;

    } else if (airspeed > ROTWING_QUAD_MAX_AIRSPEED) {
      wing_angle_scheduled_sp_deg = ((airspeed - ROTWING_QUAD_MAX_AIRSPEED)) / (ROTWING_SKEW_FW_AIRSPEED - ROTWING_QUAD_MAX_AIRSPEED) * 35. + 55.;
    
    } else {
      wing_angle_scheduled_sp_deg = 0;
    }

    Bound(wing_angle_scheduled_sp_deg, 0., 90.)
    rotwing_state.sp_wing_angle_deg = wing_angle_scheduled_sp_deg;
  }

  /* Handle the airspeed bounding */
  if(!rotwing_state_hover_motors_running()) {
    rotwing_state.min_airspeed = min_fwd_airspeed; // Vstall + margin
    rotwing_state.max_airspeed = ROTWING_FW_MAX_AIRSPEED; // Max airspeed FW
  }
  else if(!rotwing_state_pusher_motor_running()) {
    rotwing_state.min_airspeed = 0;
    rotwing_state.max_airspeed = ROTWING_QUAD_NOPUSH_AIRSPEED; // pusher_fail_v
  }
  else{
    // AIRSPEED function based on wing_angle and Qmrpm
    float sinr2 = sinf(RadOfDeg(rotwing_state.meas_wing_angle_deg)) * sinf(RadOfDeg(rotwing_state.meas_wing_angle_deg));

    rotwing_state.min_airspeed = (Vstall + margin) * (rotwing_state.meas_wing_angle_deg - 55.f) / (75.f - 55.f); // Gets bounded later
    rotwing_state.max_airspeed = Vmax_quad + (Vmax_fw - Vmax_quad) * sinr2; // Start at Vmax quad and increase to max airspeed in FW

  }

  Bound(scheduled_min_airspeed, 0, Vstall + margin);
  Bound(scheduled_max_airspeed, Vmax_quad, Vmax_fw);

  // Override failing skewing while fwd
  /*if(rotwing_state.meas_wing_angle_deg > 70 && rotwing_state_.skewing_failing) {
    rotwing_state.min_airspeed = 0; // Vstall + margin
    rotwing_state.max_airspeed = 0; // Max airspeed FW
  }*/
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
    int idx = feedback_msg[i].idx;

    // Check for wing rotation feedback
    if ((feedback_msg[i].set.position) && (idx == SERVO_ROTATION_MECH_IDX)) {
      // Get wing rotation angle from sensor
      float wing_angle_rad = 0.5 * M_PI - feedback_msg[i].position;
      rotwing_state.meas_wing_angle_deg = DegOfRad(wing_angle_rad);
      rotwing_state.meas_wing_angle_time = get_sys_time_float();

      // Bound wing rotation angle
      Bound(rotwing_state.meas_wing_angle_deg, 0, 90.);
    }

    // Get the RPM feedbacks of the motors
    if (feedback_msg[i].set.rpm) {
      if ((idx == SERVO_MOTOR_FRONT_IDX) || (idx == SERVO_BMOTOR_FRONT_IDX)) {
        rotwing_state.meas_rpm[0] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[0] = get_sys_time_float();
      } else if ((idx == SERVO_MOTOR_RIGHT_IDX) || (idx == SERVO_BMOTOR_RIGHT_IDX)) {
        rotwing_state.meas_rpm[1] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[1] = get_sys_time_float();
      } else if ((idx == SERVO_MOTOR_BACK_IDX) || (idx == SERVO_BMOTOR_BACK_IDX)) {
        rotwing_state.meas_rpm[2] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[2] = get_sys_time_float();
      } else if ((idx == SERVO_MOTOR_LEFT_IDX) || (idx == SERVO_BMOTOR_LEFT_IDX)) {
        rotwing_state.meas_rpm[3] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[3] = get_sys_time_float();
      } else if ((idx == SERVO_MOTOR_PUSH_IDX) || (idx == SERVO_BMOTOR_PUSH_IDX)) {
        rotwing_state.meas_rpm[4] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[4] = get_sys_time_float();
      }
    }
  }
}

bool rotwing_state_hover_motors_running(void) {
  float current_time = get_sys_time_float();
  // Check if hover motors are running
  if (rotwing_state.meas_rpm[0] > ROTWING_QUAD_MIN_RPM
      && rotwing_state.meas_rpm[1] > ROTWING_QUAD_MIN_RPM
      && rotwing_state.meas_rpm[2] > ROTWING_QUAD_MIN_RPM
      && rotwing_state.meas_rpm[3] > ROTWING_QUAD_MIN_RPM
      && (current_time - rotwing_state.meas_rpm_time[0]) < ROTWING_RPM_TIMEOUT
      && (current_time - rotwing_state.meas_rpm_time[1]) < ROTWING_RPM_TIMEOUT
      && (current_time - rotwing_state.meas_rpm_time[2]) < ROTWING_RPM_TIMEOUT
      && (current_time - rotwing_state.meas_rpm_time[3]) < ROTWING_RPM_TIMEOUT) {
    return true;
  } else {
    return false;
  }
}

bool rotwing_state_pusher_motor_running(void) {
  // Check if pusher motor is running
  if (rotwing_state.meas_rpm[4] > ROTWING_PUSH_MIN_RPM && (get_sys_time_float() - rotwing_state.meas_rpm_time[4]) < ROTWING_RPM_TIMEOUT) {
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
  rotwing_state.request_hover = true;
}

void rotwing_state_request_free(void) {
  rotwing_state.request_hover = false;
}