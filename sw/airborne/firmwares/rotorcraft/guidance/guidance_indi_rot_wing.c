/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Come to IROS2016 to learn more!
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_rot_wing.h"
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "stdio.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "wls/wls_alloc.h"


// The acceleration reference is calculated with these gains. If you use GPS,
// they are probably limited by the update rate of your GPS. The default
// values are tuned for 4 Hz GPS updates. If you have high speed position updates, the
// gains can be higher, depending on the speed of the inner loop.
#ifndef GUIDANCE_INDI_SPEED_GAIN
#define GUIDANCE_INDI_SPEED_GAIN 1.8
#define GUIDANCE_INDI_SPEED_GAINZ 1.8
#endif

#ifndef GUIDANCE_INDI_POS_GAIN
#define GUIDANCE_INDI_POS_GAIN 0.5
#define GUIDANCE_INDI_POS_GAINZ 0.5
#endif

#ifndef GUIDANCE_INDI_MIN_PITCH
#define GUIDANCE_INDI_MIN_PITCH -20
#define GUIDANCE_INDI_MAX_PITCH 20
#endif

struct guidance_indi_hybrid_params gih_params = {
  .pos_gain = GUIDANCE_INDI_POS_GAIN,
  .pos_gainz = GUIDANCE_INDI_POS_GAINZ,

  .speed_gain = GUIDANCE_INDI_SPEED_GAIN,
  .speed_gainz = GUIDANCE_INDI_SPEED_GAINZ,

  .heading_bank_gain = GUIDANCE_INDI_HEADING_BANK_GAIN,
};

#ifndef GUIDANCE_INDI_MAX_AIRSPEED
#error "You must have an airspeed sensor to use this guidance"
#endif
float guidance_indi_max_airspeed = GUIDANCE_INDI_MAX_AIRSPEED;

// Tell the guidance that the airspeed needs to be zeroed.
// Recomended to also put GUIDANCE_INDI_NAV_SPEED_MARGIN low in this case.
#ifndef GUIDANCE_INDI_ZERO_AIRSPEED
#define GUIDANCE_INDI_ZERO_AIRSPEED FALSE
#endif

#ifndef STABILIZATION_INDI_PUSHER_PROP_EFFECTIVENESS
#error "You need to define a STABILIZATUIN_INDI_PUSHER_PROP_EFFECTIVENESS to use guidance_indi_rot_wing"
#endif

/*Airspeed threshold where making a turn is "worth it"*/
#ifndef TURN_AIRSPEED_TH
#define TURN_AIRSPEED_TH 10.0
#endif

/*Boolean to force the heading to a static value (only use for specific experiments)*/
bool take_heading_control = false;

bool force_forward = false;

struct FloatVect3 sp_accel = {0.0,0.0,0.0};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float guidance_indi_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void guidance_indi_filter_thrust(void);

#ifndef GUIDANCE_INDI_THRUST_DYNAMICS
#ifndef STABILIZATION_INDI_ACT_DYN_P
#error "You need to define GUIDANCE_INDI_THRUST_DYNAMICS to be able to use indi vertical control"
#else // assume that the same actuators are used for thrust as for roll (e.g. quadrotor)
#define GUIDANCE_INDI_THRUST_DYNAMICS STABILIZATION_INDI_ACT_DYN_P
#endif
#endif //GUIDANCE_INDI_THRUST_DYNAMICS

#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

#ifndef GUIDANCE_INDI_SP_FILTER_CUTOFF
#define GUIDANCE_INDI_SP_FILTER_CUTOFF 1.0
#endif

float inv_eff[4];

float lift_pitch_eff = GUIDANCE_INDI_PITCH_LIFT_EFF;

// Max bank angle in radians
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

/** state eulers in zxy order */
struct FloatEulers eulers_zxy;

float thrust_act = 0;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass filt_accel_ned_sp[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;
Butterworth2LowPass accely_filt;

struct FloatVect2 desired_airspeed;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 euler_cmd;

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;
float sp_filter_cutoff = GUIDANCE_INDI_SP_FILTER_CUTOFF;

float guidance_indi_hybrid_heading_sp = 0.f;
struct FloatEulers guidance_euler_cmd;
float thrust_in;

struct FloatVect3 gi_speed_sp = {0.0, 0.0, 0.0};

// ABI handlers

#ifndef GUIDANCE_INDI_VEL_SP_ID
#define GUIDANCE_INDI_VEL_SP_ID ABI_BROADCAST
#endif
abi_event vel_sp_ev;
static void vel_sp_cb(uint8_t sender_id, struct FloatVect3 *vel_sp);
struct FloatVect3 indi_vel_sp = {0.0, 0.0, 0.0};
float time_of_vel_sp = 0.0;

#ifndef GUIDANCE_INDI_LIFT_D_ID
#define GUIDANCE_INDI_LIFT_D_ID ABI_BROADCAST
#endif
abi_event lift_d_ev;
static void lift_d_cb(uint8_t sender_id, float lift_d);
float indi_lift_d_abi = 0;
float time_of_lift_d = 0.0;

// WLS implementation for rot_wing
float Gmat_rot_wing[3][4]; // Outer loop effectiveness matrix of the rotating wing
float du_min_rot_wing[4];
float du_max_rot_wing[4];
float du_pref_rot_wing[4];
float indi_v_rot_wing[3];
float *Bwls_rot_wing[3];
int num_iter_rot_wing = 0;
float rot_wing_du[4];
float rot_wing_v[3];
float Wv_rot_wing[3] = {10., 10., 10.};
float pitch_priority_factor = 11.; //18;
float roll_priority_factor = 10.;
float thrust_priority_factor = 7.;
float pusher_priority_factor = 30.;
float horizontal_accel_weight = 10.;
float vertical_accel_weight = 10.;
float Wu_rot_wing[4] = {10., 10., 100., 1.};
float pitch_pref_deg = 0;
float pitch_pref_rad = 0;
float push_first_order_constant = 1.0;
float hover_motor_slowdown = 1.0;
float a_diff_limit = 3.0;
float a_diff_limit_z = 1.0;

float rot_wing_roll_limit = 0.785; // 45 deg
float rot_wing_pitch_limit = 0.785; // 20 deg

float rot_wing_max_pitch_limit_deg = 8.;
float rot_wing_min_pitch_limit_deg = -20.;

float airspeed_turn_lower_bound = 10.;

void guidance_indi_propagate_filters(void);
static void guidance_indi_calcg_rot_wing(struct FloatVect3 a_diff);
static float guidance_indi_get_liftd(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_guidance_indi_hybrid(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_INDI_HYBRID(trans, dev, AC_ID,
                              &sp_accel.x,
                              &sp_accel.y,
                              &sp_accel.z,
                              &guidance_euler_cmd.phi,
                              &guidance_euler_cmd.theta,
                              &euler_cmd.z,
                              &filt_accel_ned[0].o[0],
                              &filt_accel_ned[1].o[0],
                              &filt_accel_ned[2].o[0],
                              &gi_speed_sp.x,
                              &gi_speed_sp.y,
                              &gi_speed_sp.z);
}
#endif

/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  /*AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);*/
  AbiBindMsgVEL_SP(GUIDANCE_INDI_VEL_SP_ID, &vel_sp_ev, vel_sp_cb);
  AbiBindMsgLIFT_D(GUIDANCE_INDI_LIFT_D_ID, &lift_d_ev, lift_d_cb);

  float tau = 1.0/(2.0*M_PI*filter_cutoff);
  float tau_sp_filter = 1.0/(2.0*M_PI*sp_filter_cutoff);
  float sample_time = 1.0/PERIODIC_FREQUENCY;
  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_ned_sp[i], tau_sp_filter, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_guidance_indi_hybrid);
#endif

// Initialize the array of pointers to the rows of g1g2
  for (int8_t i = 0; i < 3; i++) {
    Bwls_rot_wing[i] = Gmat_rot_wing[i];
  }
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void) {
  thrust_in = stabilization_cmd[COMMAND_THRUST];
  thrust_act = thrust_in;
  guidance_indi_hybrid_heading_sp = stateGetNedToBodyEulers_f()->psi;

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float tau_sp_filter = 1.0/(2.0*M_PI*sp_filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_ned_sp[i], tau_sp_filter, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);
}

#include "firmwares/rotorcraft/navigation.h"
/**
 * @param accel_sp accel setpoint in NED frame [m/s^2]
 * @param heading_sp the desired heading [rad]
 * @return stabilization setpoint structure
 *
 * main indi guidance function
 */
struct StabilizationSetpoint guidance_indi_run(struct FloatVect3 *accel_sp, float heading_sp) 
{
  // set global accel sp variable FIXME clean this
  sp_accel = *accel_sp;

  /*Obtain eulers with zxy rotation order*/
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  /*Calculate the transition percentage so that the ctrl_effecitveness scheduling works*/
  transition_percentage = BFP_OF_REAL((eulers_zxy.theta/RadOfDeg(-75.0f))*100,INT32_PERCENTAGE_FRAC);
  Bound(transition_percentage,0,BFP_OF_REAL(100.0f,INT32_PERCENTAGE_FRAC));
  const int32_t max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
  transition_theta_offset = INT_MULT_RSHIFT((transition_percentage <<
        (INT32_ANGLE_FRAC - INT32_PERCENTAGE_FRAC)) / 100, max_offset, INT32_ANGLE_FRAC);

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters();

#if GUIDANCE_INDI_RC_DEBUG
#warning "GUIDANCE_INDI_RC_DEBUG lets you control the accelerations via RC, but disables autonomous flight!"
  // for rc control horizontal, rotate from body axes to NED
  float psi = eulers_zxy.psi;
  float rc_x = -(radio_control.values[RADIO_PITCH]/9600.0)*8.0;
  float rc_y = (radio_control.values[RADIO_ROLL]/9600.0)*8.0;
  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

  // for rc vertical control
  sp_accel.z = -(radio_control.values[RADIO_THROTTLE]-4500)*8.0/9600.0;
#endif

  // Filter sp_accel
  update_butterworth_2_low_pass(&filt_accel_ned_sp[0], sp_accel.x);
  update_butterworth_2_low_pass(&filt_accel_ned_sp[1], sp_accel.y);
  update_butterworth_2_low_pass(&filt_accel_ned_sp[2], sp_accel.z);

  struct FloatVect3 accel_filt;
  accel_filt.x = filt_accel_ned[0].o[0];
  accel_filt.y = filt_accel_ned[1].o[0];
  accel_filt.z = filt_accel_ned[2].o[0];

  struct FloatVect3 a_diff;
  a_diff.x = sp_accel.x - accel_filt.x;
  a_diff.y = sp_accel.y - accel_filt.y;
  a_diff.z = sp_accel.z - accel_filt.z;

  // a_diff.x = filt_accel_ned_sp[0].o[0] - accel_filt.x;
  // a_diff.y = filt_accel_ned_sp[1].o[0] - accel_filt.y;
  // a_diff.z = filt_accel_ned_sp[2].o[0] - accel_filt.z;

  //Bound the acceleration error so that the linearization still holds
  Bound(a_diff.x, -a_diff_limit, a_diff_limit);
  Bound(a_diff.y, -a_diff_limit, a_diff_limit);
  Bound(a_diff.z, -a_diff_limit_z, a_diff_limit_z);

  //If the thrust to specific force ratio has been defined, include vertical control
  //else ignore the vertical acceleration error
#ifndef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifndef STABILIZATION_ATTITUDE_INDI_FULL
  a_diff.z = 0.0;
#endif
#endif

  // Perform WLS
  guidance_indi_calcg_rot_wing(a_diff);

  //Calculate roll,pitch and thrust command
  //MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);

  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, rot_wing_du[2]);
  AbiSendMsgTHRUSTBX(THRUST_BX_INCREMENT_ID, rot_wing_du[3]);

  //Coordinated turn
  //feedforward estimate angular rotation omega = g*tan(phi)/v
  float omega;
  const float max_phi = RadOfDeg(60.0);
#if GUIDANCE_INDI_ZERO_AIRSPEED
  float airspeed_turn = 0.f;
#else
  float airspeed_turn = stateGetAirspeed_f();
#endif
  // We are dividing by the airspeed, so a lower bound is important
  Bound(airspeed_turn, 10.0f, 30.0f);

  guidance_euler_cmd.phi = roll_filt.o[0] + rot_wing_du[0];
  guidance_euler_cmd.theta = pitch_filt.o[0] + rot_wing_du[1];

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, -RadOfDeg(60.0), RadOfDeg(60.0));

  // Use the current roll angle to determine the corresponding heading rate of change.
  float coordinated_turn_roll = eulers_zxy.phi;

  if( (guidance_euler_cmd.theta > 0.0) && ( fabs(guidance_euler_cmd.phi) < guidance_euler_cmd.theta)) {
    coordinated_turn_roll = ((guidance_euler_cmd.phi > 0.0) - (guidance_euler_cmd.phi < 0.0))*guidance_euler_cmd.theta;
  }

  if (fabs(coordinated_turn_roll) < max_phi) {
    omega = 9.81 / airspeed_turn * tanf(coordinated_turn_roll);
  } else { //max 60 degrees roll
    omega = 9.81 / airspeed_turn * 1.72305 * ((coordinated_turn_roll > 0.0) - (coordinated_turn_roll < 0.0));
  }

#ifdef FWD_SIDESLIP_GAIN
  // Add sideslip correction
  omega -= accely_filt.o[0]*FWD_SIDESLIP_GAIN;
#endif

  // For a hybrid it is important to reduce the sideslip, which is done by changing the heading.
  // For experiments, it is possible to fix the heading to a different value.
  if (take_heading_control) {
    // heading is fixed by nav
    guidance_euler_cmd.psi = heading_sp;
  }
  else {
    // heading is free and controlled by guidance
    guidance_indi_hybrid_heading_sp += omega / PERIODIC_FREQUENCY;
    FLOAT_ANGLE_NORMALIZE(guidance_indi_hybrid_heading_sp);
    // limit heading setpoint to be within bounds of current heading
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    float delta_limit = STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    float heading = stabilization_attitude_get_heading_f();
    float delta_psi = guidance_indi_hybrid_heading_sp - heading;
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > delta_limit) {
      guidance_indi_hybrid_heading_sp = heading + delta_limit;
    } else if (delta_psi < -delta_limit) {
      guidance_indi_hybrid_heading_sp = heading - delta_limit;
    }
    FLOAT_ANGLE_NORMALIZE(guidance_indi_hybrid_heading_sp);
#endif
    guidance_euler_cmd.psi = guidance_indi_hybrid_heading_sp;
  }

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
  guidance_indi_filter_thrust();

  //Add the increment in specific force * specific_force_to_thrust_gain to the filtered thrust
  thrust_in = thrust_filt.o[0] + euler_cmd.z*guidance_indi_specific_force_gain;
  Bound(thrust_in, GUIDANCE_INDI_MIN_THROTTLE, 9600);

#if GUIDANCE_INDI_RC_DEBUG
  if(radio_control.values[RADIO_THROTTLE]<300) {
    thrust_in = 0;
  }
#endif

  //Overwrite the thrust command from guidance_v
  stabilization_cmd[COMMAND_THRUST] = thrust_in;
#endif

  // Set the quaternion setpoint from eulers_zxy
  struct FloatQuat sp_quat;
  float_quat_of_eulers_zxy(&sp_quat, &guidance_euler_cmd);
  float_quat_normalize(&sp_quat);

  return stab_sp_from_quat_f(&sp_quat);
}

// compute accel setpoint from speed setpoint (use global variables ! FIXME)
static struct FloatVect3 compute_accel_from_speed_sp(void)
{
  struct FloatVect3 accel_sp = { 0.f, 0.f, 0.f };

  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  //for rc control horizontal, rotate from body axes to NED
  float psi = eulers_zxy.psi;
  float cpsi = cosf(psi);
  float spsi = sinf(psi);
  float speed_sp_b_x =  cpsi * gi_speed_sp.x + spsi * gi_speed_sp.y;
  float speed_sp_b_y = -spsi * gi_speed_sp.x + cpsi * gi_speed_sp.y;

  // Get airspeed or zero it
#if GUIDANCE_INDI_ZERO_AIRSPEED
  float airspeed = 0.f;
#else
  float airspeed = stateGetAirspeed_f();
#endif
  struct NedCoor_f *groundspeed = stateGetSpeedNed_f();
  struct FloatVect2 airspeed_v = { cpsi * airspeed, spsi * airspeed };
  struct FloatVect2 windspeed;
  VECT2_DIFF(windspeed, *groundspeed, airspeed_v);

  VECT2_DIFF(desired_airspeed, gi_speed_sp, windspeed); // Use 2d part of gi_speed_sp
  float norm_des_as = FLOAT_VECT2_NORM(desired_airspeed);

  // Make turn instead of straight line
  if ((airspeed > TURN_AIRSPEED_TH) && (norm_des_as > (TURN_AIRSPEED_TH+2.0f))) {

    // Give the wind cancellation priority.
    if (norm_des_as > guidance_indi_max_airspeed) {
      float groundspeed_factor = 0.0f;

      // if the wind is faster than we can fly, just fly in the wind direction
      if (FLOAT_VECT2_NORM(windspeed) < guidance_indi_max_airspeed) {
        float av = gi_speed_sp.x * gi_speed_sp.x + gi_speed_sp.y * gi_speed_sp.y;
        float bv = -2.f * (windspeed.x * gi_speed_sp.x + windspeed.y * gi_speed_sp.y);
        float cv = windspeed.x * windspeed.x + windspeed.y * windspeed.y - guidance_indi_max_airspeed * guidance_indi_max_airspeed;

        float dv = bv * bv - 4.0f * av * cv;

        // dv can only be positive, but just in case
        if (dv < 0.0f) {
          dv = fabsf(dv);
        }
        float d_sqrt = sqrtf(dv);

        groundspeed_factor = (-bv + d_sqrt)  / (2.0f * av);
      }

      desired_airspeed.x = groundspeed_factor * gi_speed_sp.x - windspeed.x;
      desired_airspeed.y = groundspeed_factor * gi_speed_sp.y - windspeed.y;

      speed_sp_b_x = guidance_indi_max_airspeed;
    }

    // desired airspeed can not be larger than max airspeed
    speed_sp_b_x = Min(norm_des_as, guidance_indi_max_airspeed);

    if (force_forward) {
      speed_sp_b_x = guidance_indi_max_airspeed;
    }

    // Calculate accel sp in body axes, because we need to regulate airspeed
    struct FloatVect2 sp_accel_b;
    // In turn acceleration proportional to heading diff
    sp_accel_b.y = atan2f(desired_airspeed.y, desired_airspeed.x) - psi;
    FLOAT_ANGLE_NORMALIZE(sp_accel_b.y);
    sp_accel_b.y *= gih_params.heading_bank_gain;

    // Control the airspeed if error small enough
    sp_accel_b.x = (speed_sp_b_x - airspeed) * gih_params.speed_gain;

    accel_sp.x = cpsi * sp_accel_b.x - spsi * sp_accel_b.y;
    accel_sp.y = spsi * sp_accel_b.x + cpsi * sp_accel_b.y;
    accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
  }
  else { // Go somewhere in the shortest way

    if (airspeed > TURN_AIRSPEED_TH) {
      // Groundspeed vector in body frame
      float groundspeed_x = cpsi * stateGetSpeedNed_f()->x + spsi * stateGetSpeedNed_f()->y;
      float speed_increment = speed_sp_b_x - groundspeed_x;

      // limit groundspeed setpoint to max_airspeed + (diff gs and airspeed)
      if ((speed_increment + airspeed) > guidance_indi_max_airspeed) {
        speed_sp_b_x = guidance_indi_max_airspeed + groundspeed_x - airspeed;
      }
    }

    gi_speed_sp.x = cpsi * speed_sp_b_x - spsi * speed_sp_b_y;
    gi_speed_sp.y = spsi * speed_sp_b_x + cpsi * speed_sp_b_y;

    accel_sp.x = (gi_speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;
    accel_sp.y = (gi_speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain;
    accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
  }

  // Bound the acceleration setpoint
  float accelbound = 3.0f + airspeed / guidance_indi_max_airspeed * 5.0f; // FIXME remove hard coded values
  float_vect3_bound_in_2d(&accel_sp, accelbound);
  /*BoundAbs(sp_accel.x, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
  /*BoundAbs(sp_accel.y, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
  BoundAbs(accel_sp.z, 3.0);

  //printf("accel_sp %f %f %f\n", accel_sp.x, accel_sp.y, accel_sp.z);
  return accel_sp;
}

static float bound_vz_sp(float vz_sp)
{
  // Bound vertical speed setpoint
  if (stateGetAirspeed_f() > 13.f) {
    Bound(vz_sp, -nav.climb_vspeed, -nav.descend_vspeed); // FIXME no harcoded values
  } else {
    Bound(vz_sp, -nav.climb_vspeed, -nav.descend_vspeed); // FIXME don't use nav settings
  }
  return vz_sp;
}

struct StabilizationSetpoint guidance_indi_run_mode(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndiHybrid_HMode h_mode, enum GuidanceIndiHybrid_VMode v_mode)
{
  struct FloatVect3 pos_err = { 0 };
  struct FloatVect3 accel_sp = { 0 };

  // First check for velocity setpoint from module // FIXME should be called like this
  float dt = get_sys_time_float() - time_of_vel_sp;
  // If the input command is not updated after a timeout, switch back to flight plan control
  if (dt < 0.5) {
    gi_speed_sp.x = indi_vel_sp.x;
    gi_speed_sp.y = indi_vel_sp.y;
    gi_speed_sp.z = indi_vel_sp.z;
    accel_sp = compute_accel_from_speed_sp(); // compute accel sp
    return guidance_indi_run(&accel_sp, gh->sp.heading);
  }

  if (h_mode == GUIDANCE_INDI_HYBRID_H_POS) {
    //Linear controller to find the acceleration setpoint from position and velocity
    pos_err.x = POS_FLOAT_OF_BFP(gh->ref.pos.x) - stateGetPositionNed_f()->x;
    pos_err.y = POS_FLOAT_OF_BFP(gh->ref.pos.y) - stateGetPositionNed_f()->y;
    gi_speed_sp.x = pos_err.x * gih_params.pos_gain;// + SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
    gi_speed_sp.y = pos_err.y * gih_params.pos_gain;// + SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    if (v_mode == GUIDANCE_INDI_HYBRID_V_POS) {
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      gi_speed_sp.z = bound_vz_sp(pos_err.z * gih_params.pos_gainz);// + SPEED_FLOAT_OF_BFP(gv->zd_ref));
    } else if (v_mode == GUIDANCE_INDI_HYBRID_V_SPEED) {
      gi_speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    } else {
      gi_speed_sp.z = 0.f;
    }
    accel_sp = compute_accel_from_speed_sp(); // compute accel sp
    if (v_mode == GUIDANCE_INDI_HYBRID_V_ACCEL) {
      accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;// + ACCEL_FLOAT_OF_BFP(gv->zdd_ref); // overwrite accel
    }
    return guidance_indi_run(&accel_sp, gh->sp.heading);
  }
  else if (h_mode == GUIDANCE_INDI_HYBRID_H_SPEED) {
    gi_speed_sp.x = SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
    gi_speed_sp.y = SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    if (v_mode == GUIDANCE_INDI_HYBRID_V_POS) {
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      gi_speed_sp.z = bound_vz_sp(pos_err.z * gih_params.pos_gainz);// + SPEED_FLOAT_OF_BFP(gv->zd_ref));
    } else if (v_mode == GUIDANCE_INDI_HYBRID_V_SPEED) {
      gi_speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    } else {
      gi_speed_sp.z = 0.f;
    }
    accel_sp = compute_accel_from_speed_sp(); // compute accel sp
    if (v_mode == GUIDANCE_INDI_HYBRID_V_ACCEL) {
      accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;// + ACCEL_FLOAT_OF_BFP(gv->zdd_ref); // overwrite accel
    }
    return guidance_indi_run(&accel_sp, gh->sp.heading);
  }
  else { // H_ACCEL
    gi_speed_sp.x = 0.f;
    gi_speed_sp.y = 0.f;
    if (v_mode == GUIDANCE_INDI_HYBRID_V_POS) {
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      gi_speed_sp.z = bound_vz_sp(pos_err.z * gih_params.pos_gainz);// + SPEED_FLOAT_OF_BFP(gv->zd_ref));
    } else if (v_mode == GUIDANCE_INDI_HYBRID_V_SPEED) {
      gi_speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    } else {
      gi_speed_sp.z = 0.f;
    }
    accel_sp = compute_accel_from_speed_sp(); // compute accel sp in case z control is required
    // overwrite accel X and Y
    accel_sp.x = (gi_speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;// + ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
    accel_sp.y = (gi_speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain;// + ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
    if (v_mode == GUIDANCE_INDI_HYBRID_V_ACCEL) {
      accel_sp.z = (gi_speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;// + ACCEL_FLOAT_OF_BFP(gv->zdd_ref); // overwrite accel
    }
    return guidance_indi_run(&accel_sp, gh->sp.heading);
  }
}

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + GUIDANCE_INDI_THRUST_DYNAMICS * (thrust_in - thrust_act);

  // same filter as for the acceleration
  update_butterworth_2_low_pass(&thrust_filt, thrust_act);
}
#endif

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 * Called as a periodic function with PERIODIC_FREQ
 */
void guidance_indi_propagate_filters(void) {
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, eulers_zxy.phi);
  update_butterworth_2_low_pass(&pitch_filt, eulers_zxy.theta);

  // Propagate filter for sideslip correction
  float accely = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  update_butterworth_2_low_pass(&accely_filt, accely);
}

/**
 * Perform WLS
 *
 * @param Gmat array to write the matrix to [3x4]
 */
void guidance_indi_calcg_rot_wing(struct FloatVect3 a_diff) {

  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_zxy.phi);
  float cphi = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi = sinf(eulers_zxy.psi);
  float cpsi = cosf(eulers_zxy.psi);

  pitch_pref_rad = pitch_pref_deg / 180. * M_PI;
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  /*Amount of lift produced by the wing*/
  float lift_thrust_bz = -9.81; // Sum of lift and thrust in boxy z axis (level flight) 
  float pitch_lift = eulers_zxy.theta;
  Bound(pitch_lift,-M_PI_2,0);

  // get the derivative of the lift wrt to theta
  float liftd = guidance_indi_get_liftd();

  // Calc assumed body acceleration setpoint and error
  float accel_bx_sp = cpsi * sp_accel.x + spsi * sp_accel.y;
  float accel_bx = cpsi * filt_accel_ned[0].o[0] + spsi * filt_accel_ned[1].o[0];
  float accel_bx_err = accel_bx_sp - accel_bx;
  // float accel_bx_err = cpsi * a_diff.x + spsi * a_diff.y;

  // cpsi = 1  spsi = 0   ->   psi=0

  Gmat_rot_wing[0][1] =  ctheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING;
  Gmat_rot_wing[1][1] =  sphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*liftd;
  Gmat_rot_wing[2][1] = -cphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;

  Gmat_rot_wing[0][0] =  0
  Gmat_rot_wing[1][0] = -cphi*lift_thrust_bz;
  Gmat_rot_wing[2][0] = -sphi*lift_thrust_bz;

  Gmat_rot_wing[0][2] =  stheta;        // psi=90 -> ax->sphi*ctheta
  Gmat_rot_wing[1][2] = -sphi*ctheta;   // psi=90 -> ay->stheta
  Gmat_rot_wing[2][2] =  cphi*ctheta;

  Gmat_rot_wing[0][3] =  ctheta;
  Gmat_rot_wing[1][3] =  sphi*stheta;
  Gmat_rot_wing[2][3] = -cphi*stheta;

  // Perform WLS
  // WLS Control Allocator

  rot_wing_v[0] =  cpsi * a_diff.x + spsi * a_diff.y;
  rot_wing_v[1] = -spsi * a_diff.x + cpsi * a_diff.y;
  rot_wing_v[2] =  a_diff.z;

  // Set lower limits
  du_min_rot_wing[0] = -rot_wing_roll_limit - roll_filt.o[0]; //roll
  du_min_rot_wing[1] = rot_wing_min_pitch_limit_deg/180.*M_PI - pitch_filt.o[0]; // pitch
  du_min_rot_wing[2] = (MAX_PPRZ - actuator_state_filt_vect[0]) * g1g2[3][0] + (MAX_PPRZ - actuator_state_filt_vect[1]) * g1g2[3][1] + (MAX_PPRZ - actuator_state_filt_vect[2]) * g1g2[3][2] + (MAX_PPRZ - actuator_state_filt_vect[3]) * g1g2[3][3];
  du_min_rot_wing[3] = (-thrust_bx_state_filt*STABILIZATION_INDI_PUSHER_PROP_EFFECTIVENESS) * push_first_order_constant;

  // Set upper limits limits
  du_max_rot_wing[0] = rot_wing_roll_limit - roll_filt.o[0]; //roll
  du_max_rot_wing[1] = rot_wing_max_pitch_limit_deg/180.*M_PI - pitch_filt.o[0]; // pitch
  du_max_rot_wing[2] = -(actuator_state_filt_vect[0]*g1g2[3][0] + actuator_state_filt_vect[1]*g1g2[3][1] + actuator_state_filt_vect[2]*g1g2[3][2] + actuator_state_filt_vect[3]*g1g2[3][3]);
  du_max_rot_wing[3] = a_diff_limit * 3;
  
  // Set prefered states
  du_pref_rot_wing[0] = 0; // prefered delta roll angle
  du_pref_rot_wing[1] = -pitch_filt.o[0] + pitch_pref_rad;// prefered delta pitch angle
  du_pref_rot_wing[2] = du_max_rot_wing[2];
  du_pref_rot_wing[3] = accel_bx_err - 9.81 * sinf(pitch_filt.o[0]);

  float transition_percentage = 0.0f; // TODO: when hover props go below 40%, ...
  #define AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT 5

  // Set weights
  Wu_rot_wing[0] = roll_priority_factor * 10.414;
  Wu_rot_wing[1] = pitch_priority_factor * 27.53 * (1 - transition_percentage * 0.75);
  Wu_rot_wing[2] = thrust_priority_factor * 0.626;
  Wu_rot_wing[3] = pusher_priority_factor * 1.0;

  Wv_rot_wing[0] = horizontal_accel_weight * (1.0f + transition_percentage * AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT);
  Wv_rot_wing[1] = horizontal_accel_weight;
  Wv_rot_wing[2] = vertical_accel_weight;

  num_iter_rot_wing =
    wls_alloc_guidance(rot_wing_du, rot_wing_v, du_min_rot_wing, du_max_rot_wing, Bwls_rot_wing, 0, 0, Wv_rot_wing, Wu_rot_wing, du_pref_rot_wing, 100000, 10);
}



/**
 * @brief Get the derivative of lift w.r.t. pitch.
 *
 * @return The derivative of lift w.r.t. pitch
 */
float guidance_indi_get_liftd(void) {
  float liftd = 0.0;
  float dt = get_sys_time_float() - time_of_lift_d;
  if (dt < 0.5) {
    liftd = indi_lift_d_abi;
  }
  return liftd;
}


/**
 * ABI callback that obtains the velocity setpoint from a module
  */
static void vel_sp_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *vel_sp)
{
  indi_vel_sp.x = vel_sp->x;
  indi_vel_sp.y = vel_sp->y;
  indi_vel_sp.z = vel_sp->z;
  time_of_vel_sp = get_sys_time_float();
}

/**
 * ABI callback that obtains the liftd from a module
  */
static void lift_d_cb(uint8_t sender_id __attribute__((unused)), float lift_d)
{
  indi_lift_d_abi = lift_d;
  time_of_lift_d = get_sys_time_float();
}

#if GUIDANCE_INDI_ROT_WING_USE_AS_DEFAULT
// guidance indi control function is implementing the default functions of guidance

void guidance_h_run_enter(void)
{
  guidance_indi_enter();
}

void guidance_v_run_enter(void)
{
  // nothing to do
}

static struct VerticalGuidance *_gv = &guidance_v;
static enum GuidanceIndiHybrid_VMode _v_mode = GUIDANCE_INDI_HYBRID_V_POS;

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_HYBRID_H_POS, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_HYBRID_H_SPEED, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_HYBRID_H_ACCEL, _v_mode);
}

int32_t guidance_v_run_pos(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_HYBRID_V_POS;
  return (int32_t)thrust_in; // nothing to do
}

int32_t guidance_v_run_speed(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_HYBRID_V_SPEED;
  return (int32_t)thrust_in; // nothing to do
}

int32_t guidance_v_run_accel(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_HYBRID_V_ACCEL;
  return (int32_t)thrust_in; // nothing to do
}

#endif