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

/* High res frac for integration of angles */
#define INT32_ANGLE_HIGH_RES_FRAC 18

// Variables used for settings
enum hybrid_mode_t dc_hybrid_mode = HB_HOVER;
int32_t guidance_hybrid_norm_ref_airspeed;

// Vertical gains
float vertical_pgain = DC_FORWARD_VERTICAL_PGAIN;
float vertical_dgain = DC_FORWARD_VERTICAL_DGAIN;
float low_airspeed_pitch_gain = DC_FORWARD_VERTICAL_LOW_AIRSPEED_PITCH_GAIN;

// Horizontal gains
int32_t max_airspeed = DC_FORWARD_MAX_AIRSPEED;
float max_turn_bank = DC_FORWARD_MAX_TURN_BANK;
float turn_bank_gain = DC_FORWARD_TURN_BANK_GAIN;
float vertical_pitch_of_roll = DC_FORWARD_PITCH_OF_ROLL;
int32_t nominal_forward_thrust = DC_FORWARD_NOMINAL_THRUST;
float throttle_from_pitch_up = DC_FORWARD_THROTTLE_FROM_PITCH_UP;

/* Private functions */
static void guidance_hybrid_attitude_delftacopter(struct Int32Eulers *ypr_sp);
static void guidance_hybrid_transition_forward(void);
static void guidance_hybrid_transition_hover(void);
static void guidance_hybrid_forward(void);
static void guidance_hybrid_hover(void);
static void guidance_hybrid_set_nav_throttle_curve(void);
static void change_heading_in_wind(void);

// Private variables
static int32_t last_hover_heading;
static int32_t last_forward_heading;
static int32_t transition_time = 0;
bool has_transitioned = false;
float wind_heading_deg = 0.0;
static bool reset_wind_heading = false;

struct Int32Eulers guidance_hybrid_ypr_sp;
static struct Int32Vect2 guidance_hybrid_airspeed_sp;
static struct Int32Vect2 guidance_h_pos_err;
static struct Int32Vect2 wind_estimate;
static struct Int32Vect2 guidance_hybrid_ref_airspeed;

static int32_t norm_sp_airspeed_disp;
static int32_t heading_diff_disp;
static int32_t omega_disp;
static int32_t high_res_psi;
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
                                &guidance_hybrid_ypr_sp.psi);
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

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HYBRID_GUIDANCE, send_hybrid_guidance);
#endif
}

#define INT32_ANGLE_HIGH_RES_NORMALIZE(_a) {             \
    while ((_a) > (INT32_ANGLE_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC)))  (_a) -= (INT32_ANGLE_2_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC));    \
    while ((_a) < (-(INT32_ANGLE_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC)))) (_a) += (INT32_ANGLE_2_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC));    \
  }

/**
 * Guidance running in NAV mode only
 */
void guidance_hybrid_run(void)
{
  // Set the correct throttle curve
  guidance_hybrid_set_nav_throttle_curve();

  /* Verify in which flight mode the delftacopter is flying */
  // Transition to forward flight
  if((dc_hybrid_mode == HB_FORWARD) && (transition_percentage < (100 << INT32_PERCENTAGE_FRAC))) {
    guidance_hybrid_transition_forward();
  }
  // Transition to hover
  else if((dc_hybrid_mode == HB_HOVER) && (transition_percentage > 0)) {
    guidance_hybrid_transition_hover();
  }
  // Forward flight
  else if(dc_hybrid_mode == HB_FORWARD) {
    guidance_hybrid_forward();
  }
  // Hover flight
  else {
    guidance_hybrid_hover();
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

  // Reset just transitioned timer
  transition_time = 0;
  has_transitioned = false;
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
 * Forward flight guidance loop
 */
static void guidance_hybrid_forward(void) {
  INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_target);
  guidance_hybrid_attitude_delftacopter(&guidance_hybrid_ypr_sp);
  guidance_hybrid_set_cmd_i(&guidance_hybrid_ypr_sp);

  last_forward_heading = guidance_hybrid_ypr_sp.psi;

  // Need 3 seconds in forward flight to get speed
  if(transition_time < 3*PERIODIC_FREQUENCY) {
    transition_time += 1;
  } else {
    has_transitioned = true;
  }
}

/**
 * Hover flight guidance loop
 */
static void guidance_hybrid_hover(void) {
  // Reset just transitioned timer
  transition_time = 0;
  has_transitioned = false;

  INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_carrot);
  guidance_h_update_reference();
  change_heading_in_wind();

  // Set psi command
  guidance_h.sp.heading = ANGLE_FLOAT_OF_BFP(nav_heading);
  FLOAT_ANGLE_NORMALIZE(guidance_h.sp.heading);

  // Make sure the heading is right before leaving horizontal_mode attitude
  struct Int32Eulers sp_cmd_i;
  sp_cmd_i.psi = stabilization_attitude_get_heading_i();
  guidance_hybrid_reset_heading(&sp_cmd_i);
  last_hover_heading = sp_cmd_i.psi;

  // Run INDI guidance for hover
  guidance_indi_run(guidance_h.sp.heading);
}

/**
 * Set the throttle curve based on the specific flight mode and transition percentage
 */
static void guidance_hybrid_set_nav_throttle_curve(void) {
  // When hovering
  if(dc_hybrid_mode == HB_HOVER) {
    // Check the transition percentage
    if(transition_percentage < (50 << INT32_PERCENTAGE_FRAC)) {
      nav_throttle_curve_set(DC_HOVER_THROTTLE_CURVE);
    } else {
      nav_throttle_curve_set(DC_FORWARD_THROTTLE_CURVE);
    }
  } 
  // Forward flight
  else {
    // Check the transition percentage
    if(transition_percentage < (90 << INT32_PERCENTAGE_FRAC)) {
      nav_throttle_curve_set(DC_HOVER_THROTTLE_CURVE);
    } else {
      nav_throttle_curve_set(DC_FORWARD_THROTTLE_CURVE);
    }
  }
}

static void change_heading_in_wind(void) {
  /*
  //find the angle of the integrator
  struct FloatVect2 pos_integrator = {guidance_h_trim_att_integrator.x, guidance_h_trim_att_integrator.y};
  wind_heading = atan2f(pos_integrator.y, pos_integrator.x);
  FLOAT_ANGLE_NORMALIZE(wind_heading);
  */

  // The wind heading can be set manually
  // TODO: use the function find_wind_heading instead
  if(reset_wind_heading) {
    set_wind_heading_to_current90();
    reset_wind_heading = false;
  }
  float wind_heading = RadOfDeg(wind_heading_deg);
  //struct FloatQuat *current_quat = stateGetNedToBodyQuat_f();
  //find_wind_heading(current_quat);
  FLOAT_ANGLE_NORMALIZE(wind_heading);

  // There are two possible ways to fly sideways into the wind
  int32_t desired_heading1 = ANGLE_BFP_OF_REAL(wind_heading) + ANGLE_BFP_OF_REAL(M_PI_2);
  int32_t desired_heading2 = ANGLE_BFP_OF_REAL(wind_heading) - ANGLE_BFP_OF_REAL(M_PI_2);
  INT32_ANGLE_NORMALIZE(desired_heading1);
  INT32_ANGLE_NORMALIZE(desired_heading2);

  // Find the one closest to the current heading
  int32_t heading_error = desired_heading1 - nav_heading;
  INT32_ANGLE_NORMALIZE(heading_error);
  if(ANGLE_FLOAT_OF_BFP(abs(heading_error)) > M_PI_2) {
    heading_error = desired_heading2 - nav_heading;
    INT32_ANGLE_NORMALIZE(heading_error);
  }

  // slowly change the heading in that direction
  if(heading_error>0){
    nav_heading += 1;
  } else if(heading_error<0){
    nav_heading -= 1;
  }
}

/** Set wind heading to current heading */
void set_wind_heading_to_current90(void) {
  float wind_heading = stateGetNedToBodyEulers_f()->psi + M_PI_2;
  FLOAT_ANGLE_NORMALIZE(wind_heading);
  wind_heading_deg = DegOfRad(wind_heading);
}

/**
 * Reset the heading to the given heading
 * @param sp_cmd The heading (psi) to change to
 */
void guidance_hybrid_reset_heading(struct Int32Eulers *sp_cmd)
{
  guidance_hybrid_ypr_sp.psi = sp_cmd->psi;
  high_res_psi = sp_cmd->psi << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
}

/// Convert a required airspeed to a certain attitude for the Quadshot
static void guidance_hybrid_attitude_delftacopter(struct Int32Eulers *ypr_sp)
{
  // Get speed NED in m/s
  ned_of_ecef_vect_i(&ned_gps_vel, &state.ned_origin_i, &gps.ecef_vel);
  float north = ((float)ned_gps_vel.x) / 100.0f;
  float east =  ((float)ned_gps_vel.y) / 100.0f;

  // e.g. NN-E  n=4, e=3
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

  ypr_sp->theta = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);

  ypr_sp->phi = ANGLE_BFP_OF_REAL(heading_diff * turn_bank_gain);
  if (ypr_sp->phi > ANGLE_BFP_OF_REAL(max_turn_bank / 180.0 * M_PI)) { ypr_sp->phi = ANGLE_BFP_OF_REAL(max_turn_bank / 180.0 * M_PI); }
  if (ypr_sp->phi < ANGLE_BFP_OF_REAL(-max_turn_bank / 180.0 * M_PI)) { ypr_sp->phi = ANGLE_BFP_OF_REAL(-max_turn_bank / 180.0 * M_PI); }

  int32_t omega = 0;
  //feedforward estimate angular rotation omega = g*tan(phi)/v
  omega = ANGLE_BFP_OF_REAL(9.81 / max_airspeed * tanf(ANGLE_FLOAT_OF_BFP(
                              ypr_sp->phi)));

  if (omega > ANGLE_BFP_OF_REAL(0.7)) { omega = ANGLE_BFP_OF_REAL(0.7); }
  if (omega < ANGLE_BFP_OF_REAL(-0.7)) { omega = ANGLE_BFP_OF_REAL(-0.7); }

  //only for debugging purposes
  omega_disp = omega;

  //go to higher resolution because else the increment is too small to be added
  high_res_psi += (omega << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC)) / 512;

  INT32_ANGLE_HIGH_RES_NORMALIZE(high_res_psi);

  // go back to angle_frac
  ypr_sp->psi = high_res_psi >> (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);

  float pitch_up_from_roll = vertical_pitch_of_roll * fabs(ANGLE_FLOAT_OF_BFP(ypr_sp->phi)); // radians
  ypr_sp->theta = ypr_sp->theta + ANGLE_BFP_OF_REAL(pitch_up_from_roll);
  ypr_sp->theta = ypr_sp->theta + v_control_pitch;
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

  //   first apply the roll/pitch setpoint and then the yaw
  int32_quat_comp(&stab_att_sp_quat, &q_yaw_sp, &q_rp_i);

  int32_eulers_of_quat(&stab_att_sp_euler, &stab_att_sp_quat);
}

void guidance_hybrid_vertical(void)
{
  // Only run vertical loop in forward mode
  if(transition_percentage <= 0) {
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