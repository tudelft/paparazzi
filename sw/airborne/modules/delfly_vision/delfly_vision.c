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
#include "subsystems/abi.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "math/pprz_isa.h"

#include "subsystems/datalink/downlink.h"

#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

#include "math/pprz_algebra_int.h"

// general stereocam definitions
#if !defined(STEREO_BODY_TO_STEREO_PHI) || !defined(STEREO_BODY_TO_STEREO_THETA) || !defined(STEREO_BODY_TO_STEREO_PSI)
#warning "STEREO_BODY_TO_STEREO_XXX not defined. Using default Euler rotation angles (0,0,0)"
#endif

#define VISION_ON FALSE

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
#define DELFLY_VISION_THETA_GAINS_P 0
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

#ifndef DELFLY_VISION_THRUST_GAINS_D
#define DELFLY_VISION_THRUST_GAINS_D 0
#endif

// Guidance defines

#ifndef GUIDANCE_V_MIN_ERR_Z
#define GUIDANCE_V_MIN_ERR_Z POS_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_Z
#define GUIDANCE_V_MAX_ERR_Z POS_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MIN_ERR_ZD
#define GUIDANCE_V_MIN_ERR_ZD SPEED_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_ZD
#define GUIDANCE_V_MAX_ERR_ZD SPEED_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MAX_SUM_ERR
#define GUIDANCE_V_MAX_SUM_ERR 2000000
#endif

#define FF_CMD_FRAC 18

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

float laser_altitude;
abi_event laser_event;
abi_event baro_event_vision;
float baro_pressure;
float baro_pressure_ref;
float baro_altitude;
float baro_altitude_ref;
bool baro_initialized;
float fused_altitude;
float fused_altitude_ref;
float fused_altitude_prev;
bool previous_laser_based;
float last_altitude_time;
float last_laser_time;
float laser_altitude_prev;
float laser_rate;
float climb_rate;
float altitude_setp;

bool filt_on=true;
float filt_tc=0.25;  // gate filter time constant, in seconds
float gate_target_size=0.35; // target gate size for distance keeping, in rad

struct pid_t phi_gains = {DELFLY_VISION_PHI_GAINS_P, DELFLY_VISION_PHI_GAINS_I, 0.f};
struct pid_t theta_gains = {DELFLY_VISION_THETA_GAINS_P, DELFLY_VISION_THETA_GAINS_I, 0.f};
struct pid_t thrust_gains = {DELFLY_VISION_THRUST_GAINS_P, DELFLY_VISION_THRUST_GAINS_I, DELFLY_VISION_THRUST_GAINS_D};

float obstacle_psi;
float line_psi;
float safe_angle=0.1;

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
                                  &thrust_sp, &laser_altitude, &baro_altitude,
                                  &fused_altitude, &climb_rate, &laser_rate,
                                  &line_psi, &obstacle_psi);
}
#endif

//void laser_data_cb(uint8_t sender_id, float distance, float elevation, float heading)
static void laser_data_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{
  laser_altitude = distance;
  //gate.quality = distance*100;

  last_laser_time = get_sys_time_float();
}

static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{
  if (!baro_initialized && pressure > 1e-7)
  {
    // wait for a first positive value
    baro_pressure_ref = pressure;
    baro_initialized = true;
  }

  if (baro_initialized)
  {
    baro_altitude = pprz_isa_height_of_pressure(pressure, baro_pressure_ref);
  }
}

static void estimate_altitude(void)
{
  static float p_laser = 0.8;
  static float p_baro = 0.95;
  if (laser_altitude > 4.0 || (get_sys_time_float() - last_laser_time) > 0.2) // laser measurement out of range, or no recent laser readings
  {
    if (previous_laser_based)
    {
      baro_altitude_ref = baro_altitude;
      fused_altitude_ref = fused_altitude;
      previous_laser_based = 0;
    }

    // complementary filter - baro based measurements
    fused_altitude = fused_altitude*p_baro + (fused_altitude_ref + baro_altitude - baro_altitude_ref)*(1.0-p_baro);
  }
  else // laser measurement correct, trust laser
  {
    // complementary filter - laser based measurements
    fused_altitude = fused_altitude*p_laser + laser_altitude*(1.0-p_laser);
    previous_laser_based = 1;
  }

  float deltat = get_sys_time_float() - last_altitude_time;
  if (deltat > 0)
    {
    climb_rate = (fused_altitude - fused_altitude_prev)/deltat;
    laser_rate = (laser_altitude - laser_altitude_prev)/deltat;
    }
  else
  {
    climb_rate = 0;
    laser_rate = 0;
  }

  last_altitude_time = get_sys_time_float();
  fused_altitude_prev = fused_altitude;
  laser_altitude_prev = laser_altitude;

  // reusing the GATE messages, TODO create new messages!!!
//  gate.phi = climb_rate;
//  gate.theta = laser_rate;

}

void delfly_vision_init(void)
{
  // Subscribe to ABI messages
//  AbiBindMsgOBSTACLE_DETECTION(ABI_BROADCAST, &laser_event, laser_data_cb);
  AbiBindMsgAGL(ABI_BROADCAST, &laser_event, laser_data_cb);
  // Bind to BARO_ABS message
  AbiBindMsgBARO_ABS(ABI_BARO_DIFF_ID, &baro_event_vision, baro_cb);
  baro_initialized = false;

  // Initialize transport protocol
  pprz_transport_init(&stereocam.transport);

  gate.quality = 0;
  gate.width = 0.f;
  gate.height = 0.f;
  gate.phi = 0.f;
  gate.theta = 0.f;
  gate.depth = 0.f;

  laser_altitude = 0.f;
  fused_altitude = 0.f;
  fused_altitude_prev = 0.f;
  previous_laser_based = 1;
  last_altitude_time = 0.f;
  last_laser_time = 0.f;
  laser_altitude_prev = 0.f;
  climb_rate = 0.f;
  laser_rate = 0.f;

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
//  sp.phi = phi_gains.p*lat_error + phi_gains.i*lat_error_sum;
  sp.phi = phi_gains.p*alignment_error + phi_gains.i*alignment_error_sum;
  sp.theta = -theta_gains.p*dist_error - theta_gains.i*dist_error_sum;
//  thrust_sp = thrust_gains.p*alt_error + thrust_gains.i*alt_error_sum;


  obstacle_psi=-gate.depth; // TODO make a new message for obstacles
  line_psi=gate_psi;
    // simply set angle for yaw
  if (obstacle_psi==-1 || abs(line_psi - obstacle_psi) > safe_angle) // no obstacle detected or obstacle safely out of our flight path
    {
      sp.psi = line_psi + stateGetNedToBodyEulers_f()->psi;
    }
  else // we see an obstacle and it is on our flight path
    {
      if (line_psi > obstacle_psi) {
        sp.psi = obstacle_psi + safe_angle + stateGetNedToBodyEulers_f()->psi;
      }
      else {
        sp.psi = obstacle_psi - safe_angle + stateGetNedToBodyEulers_f()->psi;
      }
    }
//  sp.psi = stateGetNedToBodyEulers_f()->psi;

  // bound result to max values
  BoundAbs(sp.phi, STABILIZATION_ATTITUDE_SP_MAX_PHI/3);
  BoundAbs(sp.theta, STABILIZATION_ATTITUDE_SP_MAX_THETA/3);
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

    default: {}
      break;
  }

}



void delfly_vision_periodic(void)
{
  // for debugging
  //  evaluate_state_machine();
  estimate_altitude();
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
void guidance_v_module_init(void)
{
  altitude_setp = 0.f;
}

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
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

  if (radio_control.values[RADIO_FLAP] > 5000 && (get_sys_time_float() - last_time) < 0.2 && VISION_ON) // Vision switch ON, and we had a recent update from the camera
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
      stabilization_attitude_read_rc_setpoint_eulers(&rc_sp, false, false, false); // reset rc yaw setpoint
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

void guidance_v_module_run(bool in_flight)
{

  static bool altitude_hold_on = 0;

  if (radio_control.values[RADIO_FLAP] > 5000) // Vision switch ON
    {
      if (altitude_hold_on == 0) // entering altitude hold
      {
        altitude_hold_on = 1;
        altitude_setp = fused_altitude;
        guidance_v_z_sum_err = 0; // reset integration error
      }

      // in the following code, z is considered positive upwards
      int32_t err_z  = POS_BFP_OF_REAL(altitude_setp) - POS_BFP_OF_REAL(fused_altitude);
      Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
      int32_t err_zd = 0 - POS_BFP_OF_REAL(climb_rate); // desired rate is 0
      Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);

      if (in_flight) {
        guidance_v_z_sum_err += err_z;
        Bound(guidance_v_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);
      } else {
        guidance_v_z_sum_err = 0;
      }

      /* our nominal command : (g + zdd)*m   */
      int32_t inv_m;
      inv_m = BFP_OF_REAL(9.81 / (guidance_v_nominal_throttle * MAX_PPRZ), FF_CMD_FRAC);

      const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(9.81, FF_CMD_FRAC) -
                              (guidance_v_zdd_ref << (FF_CMD_FRAC - INT32_ACCEL_FRAC));

      guidance_v_ff_cmd = g_m_zdd / inv_m;
      /* feed forward command */
      guidance_v_ff_cmd = (guidance_v_ff_cmd << INT32_TRIG_FRAC) / guidance_v_thrust_coeff;

      /* bound the nominal command to MAX_PPRZ */
      Bound(guidance_v_ff_cmd, 0, MAX_PPRZ);

      /* our error feed back command                   */
      guidance_v_fb_cmd = ((guidance_v_kp * 8 * err_z)  >> 7) + // the P gain is 8 times stronger than in the standard guidance_v
                          ((guidance_v_kd * err_zd) >> 16) +
                          ((guidance_v_ki * guidance_v_z_sum_err) >> 16);

      guidance_v_delta_t = guidance_v_ff_cmd + guidance_v_fb_cmd;

      /* bound the result */
      Bound(guidance_v_delta_t, 0, MAX_PPRZ);

    stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
    thrust_sp = guidance_v_delta_t;
    }
  else // Vision switch OFF
    {
      stabilization_cmd[COMMAND_THRUST] = radio_control.values[RADIO_THROTTLE];
      altitude_hold_on = 0;

    }
}
