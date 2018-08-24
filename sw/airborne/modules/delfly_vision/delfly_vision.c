/*
 * Copyright (C) Matej Karasek
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
/**
 * @file "modules/delfly_vision/delfly_vision.c"
 * @author Matej Karasek
 * Vision module for (tail less) DelFlies
 * Include delfly_vision.xml to your airframe file.
 * Define parameters STEREO_PORT, STEREO_BAUD
 */

#include "modules/delfly_vision/delfly_vision.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/telemetry.h"
#include "pprzlink/messages.h"
#include "pprzlink/intermcu_msg.h"

#include "mcu_periph/sys_time.h"
//#include "subsystems/abi.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
//#include "guidance/guidance_v.c"

#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

#include "math/pprz_algebra_int.h"

// general stereocam definitions
#if !defined(STEREO_BODY_TO_STEREO_PHI) || !defined(STEREO_BODY_TO_STEREO_THETA) || !defined(STEREO_BODY_TO_STEREO_PSI)
#warning "STEREO_BODY_TO_STEREO_XXX not defined. Using default Euler rotation angles (0,0,0)"
#endif

#ifndef STEREO_BODY_TO_STEREO_PHI
#define STEREO_BODY_TO_STEREO_PHI 0
#endif

#ifndef STEREO_BODY_TO_STEREO_THETA
#define STEREO_BODY_TO_STEREO_THETA 0
#endif

#ifndef STEREO_BODY_TO_STEREO_PSI
#define STEREO_BODY_TO_STEREO_PSI 0
#endif


#ifndef DELFLY_VISION_PHI_GAINS_P
#define DELFLY_VISION_PHI_GAINS_P 0
#endif

#ifndef DELFLY_VISION_PHI_GAINS_I
#define DELFLY_VISION_PHI_GAINS_I 0
#endif

#ifndef DELFLY_VISION_THETA_GAINS_P
#define DELFLY_VISION_THETA_GAINS_P 0.5
#endif

#ifndef DELFLY_VISION_THETA_GAINS_I
#define DELFLY_VISION_THETA_GAINS_I 0
#endif

#ifndef DELFLY_VISION_THRUST_GAINS_P
#define DELFLY_VISION_THRUST_GAINS_P 0
#endif

#ifndef DELFLY_VISION_THRUST_GAINS_I
#define DELFLY_VISION_THRUST_GAINS_I 0
#endif

struct stereocam_t stereocam = {
  .device = (&((UART_LINK).device)),
  .msg_available = false
};
static uint8_t stereocam_msg_buf[256]  __attribute__((aligned));   ///< The message buffer for the stereocamera

static struct Int32Eulers stab_cmd;   ///< The commands that are send to the satbilization loop
static struct Int32Eulers rc_sp;   ///< Euler setpoints given by the rc

struct gate_t gate;
struct Int32Eulers att_sp = {.phi=0, .theta=0, .psi=0};
float thrust_sp;
struct FloatRMat RM_body_to_cam;
struct FloatRMat RM_body_to_gate;
struct FloatRMat RM_cam_to_gate;
struct FloatRMat RM_earth_to_gate;
struct FloatRMat RM_earth_to_body;
struct FloatEulers angle_to_gate = {.phi=0, .theta=0, .psi=0};

bool filt_on=false;
float filt_tc=3;  // gate filter time constant, in seconds
int gate_target_size=0.4; // target gate size for distance keeping, in rad

struct pid_t phi_gains = {DELFLY_VISION_PHI_GAINS_P, DELFLY_VISION_PHI_GAINS_I, 0.f};
struct pid_t theta_gains = {DELFLY_VISION_THETA_GAINS_P, DELFLY_VISION_THETA_GAINS_I, 0.f};
struct pid_t thrust_gains = {DELFLY_VISION_THRUST_GAINS_P, DELFLY_VISION_THRUST_GAINS_I, 0.f};

// todo implement
float max_thurst = 1.f;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_delfly_vision_msg(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DELFLY_VISION(trans, dev, AC_ID,
                                  &gate.quality, &gate.width, &gate.height,
                                  &gate.phi, &gate.theta, &gate.depth,
                                  &angle_to_gate.theta, &angle_to_gate.psi,
                                  &att_sp.phi, &att_sp.theta, &att_sp.psi,
                                  &thrust_sp);
}
#endif


void delfly_vision_init(void)
{
  // Initialize transport protocol
  pprz_transport_init(&stereocam.transport);

  gate.quality = 0;
  gate.width = 0.f;
  gate.height = 0.f;
  gate.phi = 0.f;
  gate.theta = 0.f;
  gate.depth = 0.f;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DELFLY_VISION, send_delfly_vision_msg);
#endif

  struct FloatEulers euler = {STEREO_BODY_TO_STEREO_PHI, STEREO_BODY_TO_STEREO_THETA, STEREO_BODY_TO_STEREO_PSI};
  float_rmat_of_eulers(&RM_body_to_cam, &euler);
}

static float alignment_error_sum = 0.f, dist_error_sum = 0.f, alt_error_sum = 0.f, lat_error_sum = 0.f;
static float last_time = 0.f;
/*
 * Compute attitude set-point given gate position
 */
static void att_sp_align_3d(void)
{
  static float gate_w, gate_h, gate_theta, gate_psi;

  // rotate angles from camera to body reference frame
  // we apply a convenience rotation here to align the camera x and y with the y and -z body frame
  struct FloatEulers cam_angles = {.phi=0., .theta=gate.phi, .psi=gate.theta}, body_angles, attitude;

  // for debugging
//  cam_angles.phi = 0.;
//  cam_angles.theta = 0.5;
//  cam_angles.psi = 0.3;

  float_rmat_of_eulers_321(&RM_cam_to_gate, &cam_angles); // rotation matrix cam to gate
  float_rmat_comp(&RM_body_to_gate, &RM_body_to_cam, &RM_cam_to_gate); // rotation matrix body to gate
  float_eulers_of_rmat(&body_angles, &RM_body_to_gate); // Eulers of gate in body frame

  // the following three lines might be wrapped in a function float_rmat_transp_mult(&angle_to_gate, stateGetNedToBodyRMat_f(), &body_angles); its current implementation is incorrect
  float_rmat_of_eulers_321(&RM_body_to_gate, &body_angles); // rotation matrix body to gate

  /* true earth axes */
  //float_rmat_comp(&RM_earth_to_gate, stateGetNedToBodyRMat_f(), &RM_body_to_gate); // rotation matrix earth to gate

  /* earth axes, but x always aligned with current heading */
  attitude.psi = 0.;
  attitude.phi = stateGetNedToBodyEulers_f()->phi;
  attitude.theta = stateGetNedToBodyEulers_f()->theta;
  float_rmat_of_eulers_321(&RM_earth_to_body, &attitude); // rotation matrix earth to body
  float_rmat_comp(&RM_earth_to_gate, &RM_earth_to_body, &RM_body_to_gate); // rotation matrix earth to gate

  float_eulers_of_rmat(&angle_to_gate, &RM_earth_to_gate); // Eulers of gate in earth frame

  // update filters
  float dt = get_sys_time_float() - last_time;
  if (dt <= 0) return;
  last_time = get_sys_time_float();
  if (dt < 1.f && filt_on)
  {
    // propagate low-pass filter
    float scaler = 1.f / (filt_tc + dt);
    gate_w = (gate.width*dt + gate_w*filt_tc) * scaler;
    gate_h = (gate.height*dt + gate_h*filt_tc) * scaler;
    gate_theta = (angle_to_gate.theta*dt + gate_theta*filt_tc) * scaler;
    gate_psi = (angle_to_gate.psi*dt + gate_psi*filt_tc) * scaler;
  }
  else
  {
    // reset filter if last update too long ago
    gate_w = gate.width;
    gate_h = gate.height;
    gate_theta = angle_to_gate.theta;
    gate_psi = angle_to_gate.psi;
  }

  // compute errors
  float alignment_error = gate_h / gate_w - 1.f; // [-]
  BoundLower(alignment_error, 0.f);
  // sign of the alignment error is ambiguous so set it based on the long term derivative of the error
  // alignment_gain_sign = -(derivative of the error)
  float alignment_gain_sign = 1.f; // TODO implement
  alignment_error *= alignment_gain_sign;

  float dist_error = (gate_target_size - gate_h);  // rad
  float alt_error = -gate_theta; // rad
  float lat_error = gate_psi; // rad

  // Increment integrated errors
  alignment_error_sum += alignment_error*dt;
  dist_error_sum += dist_error*dt;
  alt_error_sum += alt_error*dt;
  lat_error_sum += lat_error*dt;

  // apply pid gains for yaw, pitch and thrust
  struct FloatEulers sp;
  sp.phi = phi_gains.p*alignment_error + phi_gains.i*alignment_error_sum;
  sp.theta = -theta_gains.p*dist_error + theta_gains.i*dist_error_sum;
  thrust_sp = thrust_gains.p*alt_error + thrust_gains.i*alt_error_sum;

  // simply set angle for yaw
  sp.psi = gate_psi + stateGetNedToBodyEulers_f()->psi;
  //sp.psi = 0;

  // bound result to max values
  BoundAbs(sp.phi, STABILIZATION_ATTITUDE_SP_MAX_PHI);
  BoundAbs(sp.theta, STABILIZATION_ATTITUDE_SP_MAX_THETA);
  Bound(thrust_sp, 0.f, max_thurst); // TODO add nominal thrust somewhere

  // scale to integers
  att_sp.phi = ANGLE_BFP_OF_REAL(sp.phi);
  att_sp.theta = ANGLE_BFP_OF_REAL(sp.theta);
  att_sp.psi = ANGLE_BFP_OF_REAL(sp.psi);
  //guidance_v_zd_sp = thrust_sp; // todo implement
}

/*
 * Evaluate state machine to decide what to do
 */
static void evaluate_state_machine(void)
{
  att_sp_align_3d();
}

/* Parse the InterMCU message */
static void delfly_vision_parse_msg(void)
{
  /* Parse the stereocam message */
  uint8_t msg_id = pprzlink_get_msg_id(stereocam_msg_buf);
  switch (msg_id) {

    case DL_STEREOCAM_GATE: {
      gate.quality = DL_STEREOCAM_GATE_quality(stereocam_msg_buf);
      gate.width   = DL_STEREOCAM_GATE_width(stereocam_msg_buf);
      gate.height  = DL_STEREOCAM_GATE_height(stereocam_msg_buf);
      gate.phi     = DL_STEREOCAM_GATE_phi(stereocam_msg_buf);
      gate.theta   = DL_STEREOCAM_GATE_theta(stereocam_msg_buf);
      gate.depth   = DL_STEREOCAM_GATE_depth(stereocam_msg_buf);

      evaluate_state_machine();

      break;
    }

    default:
      break;
  }
}

void delfly_vision_periodic(void)
{
  // for debugging
  //  evaluate_state_machine();
}

void delfly_vision_event(void)
{
   // Check if we got some message from the stereocam
   pprz_check_and_parse(stereocam.device, &stereocam.transport, stereocam_msg_buf, &stereocam.msg_available);

   // If we have a message we should parse it
   if (stereocam.msg_available) {
     delfly_vision_parse_msg();
     stereocam.msg_available = false;
   }
}


/**
 * Initialization of horizontal & vertical guidance modules
 */
void guidance_h_module_init(void) {}
void guidance_v_module_init(void) {}

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
//  /* Set roll/pitch to 0 degrees and psi to current heading */
//  stab_cmd.phi = 0;
//  stab_cmd.theta = 0;
//  stab_cmd.psi = stateGetNedToBodyEulers_i()->psi;
//
  // reset integrator of rc_yaw_setpoint
  stabilization_attitude_read_rc_setpoint_eulers(&rc_sp, false, false, false);

  stab_cmd.phi = rc_sp.phi;
  stab_cmd.theta = rc_sp.theta;
  stab_cmd.psi = rc_sp.psi;

}

void guidance_v_module_enter(void) {}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
  // TODO: change the desired vx/vy
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_h_module_run(bool in_flight)
{
  stabilization_attitude_read_rc_setpoint_eulers(&rc_sp, in_flight, false, false);

  static uint8_t prev = 0;

  if (radio_control.values[RADIO_FLAP] > 5000) // Vision switch ON
  {

    stab_cmd.phi = rc_sp.phi + att_sp.phi;
    stab_cmd.theta = rc_sp.theta + att_sp.theta;
    stab_cmd.psi = att_sp.psi;

    prev = 0;
  }
  else // Vision switch OFF
  {
    if (prev == 0)
    {
      stabilization_attitude_read_rc_setpoint_eulers(&rc_sp, false, false, false);
      prev = 1;
    }

    stab_cmd.phi = rc_sp.phi;
    stab_cmd.theta = rc_sp.theta;
    stab_cmd.psi = rc_sp.psi;

    // reset vision code integrators
    alignment_error_sum = 0.f;
    dist_error_sum = 0.f;
    alt_error_sum = 0.f;
    lat_error_sum = 0.f;
  }

  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&stab_cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}

void guidance_v_module_run(UNUSED bool in_flight)
{
  stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];
}
