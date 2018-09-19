/*
 * Copyright (C) 2014 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 *               2018 Freek van Tienen <freek.v.tienen@gmail.com>
 * This is code for guidance of the hybrid Delftacopter.
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

/** @file firmwares/rotorcraft/guidance/guidance_delftacopter.c
 *  Guidance controllers (horizontal and vertical) for the Hybrid DelftaCopter UAV.
 */

#include "firmwares/rotorcraft/guidance/guidance_delftacopter.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

/* for guidance_v_thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

/* Hover throttle curve */
#ifndef DC_HOVER_THROTTLE_CURVE
#define DC_HOVER_THROTTLE_CURVE 1
#endif

/* Forward throttle curve */
#ifndef DC_FORWARD_THROTTLE_CURVE
#define DC_FORWARD_THROTTLE_CURVE 2
#endif

/* Transition throttle curve */
#ifndef DC_TRANSITION_THROTTLE_CURVE
#define DC_TRANSITION_THROTTLE_CURVE DC_FORWARD_THROTTLE_CURVE
#endif

/* Extra transition time in special curve and no turning */
#ifndef DC_TRANSITION_TIME
#define DC_TRANSITION_TIME 3
#endif


// Variables used for settings
enum hybrid_mode_t dc_hybrid_mode = HB_HOVER;
int32_t guidance_hybrid_norm_ref_airspeed;

// Exported global variables
int32_t guidance_feed_forward_yaw_which_is_delftacopter_roll = 0;


// Vertical gains
float vertical_pgain = DC_FORWARD_VERTICAL_PGAIN;
float vertical_dgain = DC_FORWARD_VERTICAL_DGAIN;
float low_airspeed_pitch_gain = DC_FORWARD_VERTICAL_LOW_AIRSPEED_PITCH_GAIN;

// Horizontal gains
int16_t transition_throttle_to_forward = DC_TRANSITION_THROTTLE_TO_FORWARD;
int16_t transition_throttle_to_hover = DC_TRANSITION_THROTTLE_TO_HOVER;
float max_airspeed = DC_FORWARD_MAX_AIRSPEED;
float max_turn_bank = DC_FORWARD_MAX_TURN_BANK;
float turn_bank_gain = DC_FORWARD_TURN_BANK_GAIN;
float vertical_pitch_of_roll = DC_FORWARD_PITCH_OF_ROLL;
int32_t nominal_forward_thrust = DC_FORWARD_NOMINAL_THRUST;
float throttle_from_pitch_up = DC_FORWARD_THROTTLE_FROM_PITCH_UP;
float forward_max_psi = DC_FORWARD_MAX_PSI;
float acc_y_filter_cutoff_hz = DC_FORWARD_ACC_Y_FILTER_CUTOFF_HZ;
float e_psi_deg_from_acc_y = DC_FORWARD_E_PSI_DEG_FROM_ACC_Y;

// Forward stabilization loop
bool delftacopter_fwd_controller_enabled = false;
float delftacopter_fwd_roll = 0;
float delftacopter_fwd_pitch = 0;
float delftacopter_fwd_yaw = 0;
float delftacopter_fwd_roll_pgain = DC_FORWARD_ROLL_PGAIN;
float delftacopter_fwd_roll_dgain = DC_FORWARD_ROLL_DGAIN;
float delftacopter_fwd_pitch_pgain = DC_FORWARD_PITCH_PGAIN;
float delftacopter_fwd_pitch_dgain = DC_FORWARD_PITCH_DGAIN;
float delftacopter_fwd_pitch_igain = DC_FORWARD_PITCH_IGAIN;
float feedforward_yaw_of_turn_rate = DC_FORWARD_K_FF_TO_YAW;
float delftacopter_fwd_advance_angle_p = DC_FORWARD_ADVANCE_P;
float delftacopter_fwd_advance_angle_q = DC_FORWARD_ADVANCE_Q;
float delftacopter_fwd_pitch_swp = DC_FORWARD_PITCH_SWP;

/* Private functions */
static void guidance_hybrid_attitude_delftacopter(struct Int32Eulers *ypr_sp);
static void guidance_hybrid_transition_forward(void);
static void guidance_hybrid_transition_hover(void);
static void guidance_hybrid_forward(void);
static void guidance_hybrid_hover(bool in_flight);
static void guidance_hybrid_set_nav_throttle_curve(void);

// Private variables
static int32_t last_hover_heading;
static int32_t last_forward_heading;
static int32_t transition_time = 0;
static float filtered_acc_y = 0;

struct Int32Eulers guidance_hybrid_ypr_sp;
static struct Int32Vect2 guidance_hybrid_airspeed_sp;
static struct Int32Vect2 guidance_h_pos_err;
static struct Int32Vect2 wind_estimate;
static struct Int32Vect2 guidance_hybrid_ref_airspeed;
static Butterworth2LowPass acc_y_filter;

static int32_t norm_sp_airspeed_disp;
static int32_t heading_diff_disp;
static float high_res_psi;
int32_t v_control_pitch = 0;
struct NedCoor_i ned_gps_vel;
float dperpendicular = 0.0;
float perpendicular_prev = 0.0;
float perpen_dgain = 0.0;
float perpendicular = 0.0;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_hybrid_guidance(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  struct NedCoor_i *speed = stateGetSpeedNed_i();
  pprz_msg_send_HYBRID_GUIDANCE(trans, dev, AC_ID,
                                &(pos->x), &(pos->y),
                                &(speed->x), &(speed->y),
                                &wind_estimate.x, &wind_estimate.y,
                                &guidance_h_pos_err.x,
                                &guidance_h_pos_err.y,
                                &guidance_hybrid_airspeed_sp.x,
                                &guidance_hybrid_airspeed_sp.y,
                                &guidance_hybrid_norm_ref_airspeed,
                                &heading_diff_disp,
                                &guidance_hybrid_ypr_sp.phi,
                                &guidance_hybrid_ypr_sp.theta,
                                &guidance_hybrid_ypr_sp.psi,
                                &filtered_acc_y);
}

#endif

/**
 * Initialization of the variables and registering of telemetry
 */
void guidance_hybrid_init(void)
{
  INT_EULERS_ZERO(guidance_hybrid_ypr_sp);
  INT_VECT2_ZERO(guidance_hybrid_airspeed_sp);
  INT_VECT2_ZERO(guidance_hybrid_ref_airspeed);
  INT_VECT2_ZERO(wind_estimate);

  high_res_psi = 0;
  guidance_hybrid_norm_ref_airspeed = 0;
  dperpendicular = 0.0;
  perpendicular_prev = 0.0;

  init_butterworth_2_low_pass(&acc_y_filter, 1/(acc_y_filter_cutoff_hz * 2 * M_PI), 1./PERIODIC_FREQUENCY, 0);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HYBRID_GUIDANCE, send_hybrid_guidance);
#endif
}

void guidance_delftacopter_set_acc_y_cutoff_hz(float cutoff_hz)
{
  acc_y_filter_cutoff_hz = cutoff_hz;
  float tau_acc_y_filter = 1/(cutoff_hz * 2 * M_PI); // acc_y filter cutoff frequency in Hertz

  init_butterworth_2_low_pass(&acc_y_filter, tau_acc_y_filter, 1./PERIODIC_FREQUENCY, get_butterworth_2_low_pass(&acc_y_filter));
}

/**
 * Guidance running in NAV mode only
 */
void guidance_hybrid_run(bool in_flight)
{
  // Set the correct throttle curve
  guidance_hybrid_set_nav_throttle_curve();

  // Calculate the sideslip
  guidance_hybrid_update_sideslip_estimate();

  /* Verify in which flight mode the delftacopter is flying */
  // Transition to forward flight
  if((dc_hybrid_mode == HB_FORWARD) && (transition_percentage < (100 << INT32_PERCENTAGE_FRAC) || transition_time < (DC_TRANSITION_TIME*PERIODIC_FREQUENCY))) {
    guidance_hybrid_transition_forward();
  }
  // Transition to hover
  else if((dc_hybrid_mode == HB_HOVER) && (transition_percentage > 0)) {
    guidance_hybrid_transition_hover();
  }
  // Forward flight
  else if(dc_hybrid_mode == HB_FORWARD) {
    arrived_at_waypoint = ARRIVED_AT_WAYPOINT;
    guidance_hybrid_forward();
  }
  // Hover flight
  else {
    arrived_at_waypoint = ARRIVED_AT_WAYPOINT_HOVER;
    guidance_hybrid_hover(in_flight);
  }
}

/**
 * Transition to forward flight
 * Keeps the roll and extra pitch at zero
 */
static void guidance_hybrid_transition_forward(void) {
  // Transition to forward
  guidance_h_transition_run(true);

  // Set the corresponding attitude
  struct Int32Eulers transition_att_sp;
  transition_att_sp.phi = 0;
  transition_att_sp.theta = transition_theta_offset;
  transition_att_sp.psi = last_hover_heading;
  stabilization_attitude_set_rpy_setpoint_i(&transition_att_sp);

  if(transition_percentage < (100 << INT32_PERCENTAGE_FRAC)) {
    // Reset just transitioned timer
    transition_time = 0;
  } else {
    transition_time++;
  }
}

/**
 * Transition to hover flight
 * Keeps the roll and extra pitch at zero
 */
static void guidance_hybrid_transition_hover(void) {
  // Transition to hover
  guidance_h_transition_run(false);

  // Set the corresponding attitude
  struct Int32Eulers transition_att_sp;
  transition_att_sp.phi = 0;
  transition_att_sp.theta = transition_theta_offset;
  transition_att_sp.psi = last_forward_heading;
  stabilization_attitude_set_rpy_setpoint_i(&transition_att_sp);

  // Reset the reference, as it is used in hover mode
  guidance_h_reset_reference_from_current_position();

  // Set nav_heading to current heading
  nav_heading = stabilization_attitude_get_heading_i();
  INT32_ANGLE_NORMALIZE(nav_heading);

  // Reset roll (fwd yaw) integrator
  stabilization_att_sum_err_quat.qx = 0;
}

/**
 * Get the current body y acceleration and low-pass filter it. Usable for side-slip control.
 */
void guidance_hybrid_update_sideslip_estimate(void) {
  struct Int32Vect3 *acceleration = stateGetAccelBody_i();
  float acceleration_y_f = ACCEL_FLOAT_OF_BFP(acceleration->y);

  filtered_acc_y = update_butterworth_2_low_pass(&acc_y_filter, acceleration_y_f);
}

/**
 * Forward flight guidance loop
 */
static void guidance_hybrid_forward(void) {
  INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_target);
  guidance_hybrid_attitude_delftacopter(&guidance_hybrid_ypr_sp);
  guidance_hybrid_set_cmd_i(&guidance_hybrid_ypr_sp);

  last_forward_heading = guidance_hybrid_ypr_sp.psi;
}

/**
 * Hover flight guidance loop
 */
static void guidance_hybrid_hover(bool in_flight) {
  INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_carrot);
  guidance_h_update_reference();

  // Set psi command
  guidance_h.sp.heading = ANGLE_FLOAT_OF_BFP(nav_heading);
  FLOAT_ANGLE_NORMALIZE(guidance_h.sp.heading);

  // Make sure the heading is right before leaving horizontal_mode attitude
  struct Int32Eulers sp_cmd_i;
  sp_cmd_i.psi = stabilization_attitude_get_heading_i();
  guidance_hybrid_reset_heading(&sp_cmd_i);
  last_hover_heading = sp_cmd_i.psi;

#if GUIDANCE_INDI
  (void)in_flight;
  guidance_indi_run(guidance_h.sp.heading);
#else
  /* compute x,y earth commands */
  guidance_h_traj_run(in_flight);
  /* set final attitude setpoint */
  int32_t heading_sp_i = ANGLE_BFP_OF_REAL(guidance_h.sp.heading);
  stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth, heading_sp_i);
#endif
}

/**
 * Set the throttle curve based on the specific flight mode and transition percentage
 */
static void guidance_hybrid_set_nav_throttle_curve(void) {
  // When hovering
  if(dc_hybrid_mode == HB_HOVER) {
    // Check the transition percentage
    if(transition_percentage < (100 << INT32_PERCENTAGE_FRAC)) {
      nav_throttle_curve_set(DC_HOVER_THROTTLE_CURVE);
    } else {
      nav_throttle_curve_set(DC_TRANSITION_THROTTLE_CURVE);
    }
  } 
  // Forward flight
  else {
    // Check the transition percentage
    if(transition_percentage < (100 << INT32_PERCENTAGE_FRAC)) {
      nav_throttle_curve_set(DC_HOVER_THROTTLE_CURVE);
    } else if (transition_time < (DC_TRANSITION_TIME*PERIODIC_FREQUENCY)) {
      nav_throttle_curve_set(DC_TRANSITION_THROTTLE_CURVE);
    } else {
      nav_throttle_curve_set(DC_FORWARD_THROTTLE_CURVE);
    }
  }
}

/**
 * Reset the heading to the given heading
 * @param sp_cmd The heading (psi) to change to
 */
void guidance_hybrid_reset_heading(struct Int32Eulers *sp_cmd)
{
  guidance_hybrid_ypr_sp.psi = sp_cmd->psi;
  high_res_psi = ANGLE_FLOAT_OF_BFP(sp_cmd->psi);
}


/// Convert a required airspeed to a certain attitude for the Quadshot
static void guidance_hybrid_attitude_delftacopter(struct Int32Eulers *ypr_sp)
{
  // Get speed NED in m/s
  ned_of_ecef_vect_i(&ned_gps_vel, &state.ned_origin_i, &gps.ecef_vel);
  float north = ((float)ned_gps_vel.x) / 100.0f;
  float east =  ((float)ned_gps_vel.y) / 100.0f;

  // e.g. NN-E  n=4, e=3
  nav_set_heading_towards_target();
  float cosh = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
  float sinh = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));

  // e.g. good cos = 4/5 sin = 3/5
  float to_wp         =   north * cosh + east * sinh;
  perpendicular = - north * sinh + east * cosh;

  dperpendicular = (perpendicular - perpendicular_prev)*PERIODIC_FREQUENCY;
  perpendicular_prev = perpendicular;
  // dperpendicular should be bounded to get rid of spikes when switching waypoints
  float heading_diff_d = dperpendicular * perpen_dgain;
  BoundAbs(heading_diff_d, 20.0/180.0*M_PI);

  // towp = 4*4/5 + 3 * 3/5 = 16+9 / 5 = 5
  // perpendic = -4 * 3/5 + 3 * 4/5 = 0/5 = 0

  // e.g. fly north=5, east=0 en we moeten nog steeds 4/3 NNE

  // towp = 5*4/5 + 0 * 3/5 = 4
  // perpendicular = -5*3/5 + 0*4/5 = -3 : we vliegen teveel naar links

  // linearize atan (angle less than 45 degree
  float heading_diff = - (perpendicular / 20.0f + heading_diff_d);  // m/s

/*
  Even proberen vervangen: als het niet werkt moet dit weer aan.
  float meas_course = ((float) gps.course)/1e7;
  FLOAT_ANGLE_NORMALIZE(meas_course);

  float heading_diff;
  if(gps.gspeed <500) {
    //The difference of the current heading with the required heading.
    heading_diff = ANGLE_FLOAT_OF_BFP(nav_heading) - stabilization_attitude_get_heading_f();
  } else {
    heading_diff = ANGLE_FLOAT_OF_BFP(nav_heading) - meas_course;
  }
  FLOAT_ANGLE_NORMALIZE(heading_diff);
*/
  // When error is so large that we fly away from the waypoint, turn maximum
  // Solves: when flying perfectly away from waypoint, then perpendicular = 0
  if (to_wp < 0)
  {
    heading_diff = ANGLE_FLOAT_OF_BFP(nav_heading) - stabilization_attitude_get_heading_f();
  }
  FLOAT_ANGLE_NORMALIZE(heading_diff);

  //only for debugging
  heading_diff_disp = (int32_t)(heading_diff / 3.14 * 180.0);

  //calculate the norm of the airspeed setpoint
  int32_t norm_sp_airspeed;
  norm_sp_airspeed = int32_vect2_norm(&guidance_hybrid_airspeed_sp);

  norm_sp_airspeed_disp = norm_sp_airspeed;

  int32_t psi = ypr_sp->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);

  guidance_hybrid_ref_airspeed.x = (guidance_hybrid_norm_ref_airspeed * c_psi) >> INT32_TRIG_FRAC;
  guidance_hybrid_ref_airspeed.y = (guidance_hybrid_norm_ref_airspeed * s_psi) >> INT32_TRIG_FRAC;

 
  delftacopter_fwd_roll = heading_diff * turn_bank_gain;
  BoundAbs(delftacopter_fwd_roll, max_turn_bank / 180.0 * M_PI);

  float pitch_up_from_roll = vertical_pitch_of_roll * fabs(delftacopter_fwd_roll); // radians
  delftacopter_fwd_pitch = pitch_up_from_roll + ANGLE_FLOAT_OF_BFP(v_control_pitch);

 
  // Feedforward estimate angular rotation omega = g*tan(phi)/v
  float omega = 9.81 / max_airspeed * tanf(ANGLE_FLOAT_OF_BFP(ypr_sp->phi));
  BoundAbs(omega, 0.7);

  // Add side-slip controller here
  omega += -e_psi_deg_from_acc_y * filtered_acc_y;

  // Set forward yaw
  delftacopter_fwd_yaw = omega * feedforward_yaw_of_turn_rate;
  delftacopter_fwd_controller_enabled = true;
}

void guidance_hybrid_set_cmd_i(struct Int32Eulers *sp_cmd)
{
  // create a quaternion with just the roll and pitch command
  struct FloatQuat q_p;
  struct FloatQuat q_r;
  struct FloatEulers pitch_cmd = {0.0, ANGLE_FLOAT_OF_BFP(sp_cmd->theta), 0.0};
  struct FloatEulers roll_cmd = {ANGLE_FLOAT_OF_BFP(sp_cmd->phi), 0.0, 0.0};
  float_quat_of_eulers(&q_p, &pitch_cmd);
  float_quat_of_eulers(&q_r, &roll_cmd);

  struct FloatQuat q_rp;
  float_quat_comp(&q_rp, &q_r, &q_p);

  struct Int32Quat q_rp_i;
  QUAT_BFP_OF_REAL(q_rp_i, q_rp);

  //   get the vertical vector to rotate the roll pitch setpoint around
  struct Int32Vect3 zaxis = {0, 0, 1};

  /* get current heading setpoint */
  struct Int32Quat q_yaw_sp;
  int32_quat_of_axis_angle(&q_yaw_sp, &zaxis, sp_cmd->psi);
  int32_quat_normalize(&q_yaw_sp); // int32_quat_of_axis_angle does not normalize the quaternion

  //   first apply the roll/pitch setpoint and then the yaw
  int32_quat_comp(&stab_att_sp_quat, &q_yaw_sp, &q_rp_i);

  int32_eulers_of_quat(&stab_att_sp_euler, &stab_att_sp_quat);
}

void guidance_hybrid_vertical(void)
{
  // Only run vertical loop in forward mode
  if(transition_percentage <= 0) {
    stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
    return;
  }
  // Transition to forward
  else if ((dc_hybrid_mode == HB_FORWARD) && transition_percentage < (100 << INT32_PERCENTAGE_FRAC)) {
    stabilization_cmd[COMMAND_THRUST] = transition_throttle_to_forward;
    return;
  }
  // Transitions to hover
  else if ((dc_hybrid_mode == HB_HOVER) && transition_percentage < (100 << INT32_PERCENTAGE_FRAC)) {
    stabilization_cmd[COMMAND_THRUST] = transition_throttle_to_hover;
    return;
  }
  
  int32_t vertical_err = -(guidance_v_z_ref - stateGetPositionNed_i()->z);
  v_control_pitch = ANGLE_BFP_OF_REAL( POS_FLOAT_OF_BFP(vertical_err) * vertical_pgain + stateGetSpeedNed_f()->z * vertical_dgain);

  Bound(v_control_pitch, ANGLE_BFP_OF_REAL(RadOfDeg(DC_FORWARD_MIN_PITCH)), ANGLE_BFP_OF_REAL(RadOfDeg(DC_FORWARD_MAX_PITCH)));

  float airspeed = stateGetAirspeed_f();
  if(airspeed < DC_FORWARD_VERTICAL_LOW_AIRSPEED) {
    v_control_pitch -= ANGLE_BFP_OF_REAL((DC_FORWARD_VERTICAL_LOW_AIRSPEED - airspeed)*RadOfDeg(low_airspeed_pitch_gain));
  }

  Bound(v_control_pitch, ANGLE_BFP_OF_REAL(RadOfDeg(DC_FORWARD_MIN_PITCH)), ANGLE_BFP_OF_REAL(RadOfDeg(DC_FORWARD_MAX_PITCH)));

  stabilization_cmd[COMMAND_THRUST] = nominal_forward_thrust;
  // Extra thrust when pitching up
  if(v_control_pitch > 0) {
    stabilization_cmd[COMMAND_THRUST] += ANGLE_FLOAT_OF_BFP(v_control_pitch)/M_PI*180.0*throttle_from_pitch_up;
  }
  Bound(stabilization_cmd[COMMAND_THRUST], 0, 9600);
}
