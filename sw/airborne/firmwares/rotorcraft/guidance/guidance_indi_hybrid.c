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
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/radio_control.h"
#include "state.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "stdio.h"
#include "filters/low_pass_filter.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "stabilization/wls/wls_alloc.h"
#include "modules/system_identification/sys_id_chirp.h"

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

// Max ground speed that will be commanded
#define NAV_MAX_SPEED (GUIDANCE_INDI_MAX_AIRSPEED + 10.0)
float nav_max_speed = NAV_MAX_SPEED;

#ifndef MAX_DECELERATION
#define MAX_DECELERATION 1.
#endif

/*Boolean to force the heading to a static value (only use for specific experiments)*/
bool take_heading_control = true;

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

//#else
//#define GUIDANCE_INDI_SPECIFIC_FORCE_GAIN -943.0
//#define GUIDANCE_INDI_THRUST_DYNAMICS 0.1
//float guidance_indi_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

#ifdef GUIDANCE_INDI_LINE_GAIN
float guidance_indi_line_gain = GUIDANCE_INDI_LINE_GAIN;
#else
float guidance_indi_line_gain = 1.0;
#endif

float inv_eff[4];

float lift_pitch_eff = GUIDANCE_INDI_PITCH_LIFT_EFF;

// Max bank angle in radians
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

/** state eulers in zxy order */
struct FloatEulers eulers_zxy;

float thrust_act = 0;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;
Butterworth2LowPass accely_filt;

struct FloatVect2 desired_airspeed;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 euler_cmd;
float acc_T_bx;
bool T_bx_effective = false;

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;

struct FloatEulers guidance_euler_cmd;
float thrust_in;

struct FloatVect3 speed_sp = {0.0, 0.0, 0.0};

// Rot wing specific matrices
float Gmat_rot_wing[3][4];
float Gmat_rot_wing_trans_mult[3][3];
float Gmat_rot_winginv[4][4];
float Gmat_rot_wing_pseudo_inv[4][3];

// WLS implementation for Rot wing
float du_min_hybrid[4];
float du_max_hybrid[4];
float du_pref_hybrid[4];
float indi_v_hybrid[3];
float *Bwls_hybrid[3];
int num_iter_hybrid = 0;
float hybrid_du[4];
float hybrid_v[3];
float Wv_hybrid[3] = {10., 10., 1.};
float pitch_priority_factor = 10.;
float roll_priority_factor = 0.;
float thrust_priority_factor = 10.;
float pusher_priority_factor = 1.;
float Wu_hybrid[4] = {10.,10.,100.,1.};//{105.,230.,250.,1.}{230.,105.,25.,1.}{24.,11.,25.,5.};
float pitch_pref_deg = 0;
float pitch_pref_rad = 0;

float hybrid_roll_limit = 0.785; // 45 deg
float hybrid_pitch_limit = 0.244;//14 deg//0.349; // 20 deg

bool chirp_init_check = FALSE ;
float chirp_val_init = 0;
int chirp_number = 0;
float phi_ref_c;
float theta_ref_c;
float psi_ref_c;
struct NedCoor_f pos_ref_c;
struct FloatVect3 speed_ref_c; 
struct FloatVect3 acc_body_ref_c;
struct FloatVect3 acc_body_c;
float fwd_sideslip_g = 0;
#ifdef FWD_SIDESLIP_GAIN
fwd_sideslip_g = FWD_SIDESLIP_GAIN;
#endif

void guidance_indi_propagate_filters(void);
static void guidance_indi_calcg_wing(struct FloatMat33 *Gmat);
static void guidance_indi_calcg_rot_wing(void);
static void guidance_indi_calcg_rot_wing_wls(struct FloatVect3 a_diff);
static float guidance_indi_get_liftd(float pitch, float theta);
struct FloatVect3 nav_get_speed_sp_from_go(struct EnuCoor_i target, float pos_gain);
struct FloatVect3 nav_get_speed_sp_from_line(struct FloatVect2 line_v_enu, struct FloatVect2 to_end_v_enu, struct EnuCoor_i target, float pos_gain);
struct FloatVect3 nav_get_speed_setpoint(float pos_gain);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_guidance_indi_hybrid(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_INDI_HYBRID(trans, dev, AC_ID,
                              &sp_accel.x,
                              &sp_accel.y,
                              &sp_accel.z,
                              &euler_cmd.x,
                              &euler_cmd.y,
                              &euler_cmd.z,
                              &acc_T_bx,
                              &filt_accel_ned[0].o[0],
                              &filt_accel_ned[1].o[0],
                              &filt_accel_ned[2].o[0],
                              &speed_sp.x,
                              &speed_sp.y,
                              &speed_sp.z);
}
#endif

/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  /*AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);*/

  float tau = 1.0/(2.0*M_PI*filter_cutoff);
  float sample_time = 1.0/PERIODIC_FREQUENCY;
  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);

  // Initialize the array of pointers to the rows of g1g2
  uint8_t i;
  for (i = 0; i < 3; i++) {
    Bwls_hybrid[i] = Gmat_rot_wing[i];
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_guidance_indi_hybrid);
#endif
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void) {
  thrust_in = stabilization_cmd[COMMAND_THRUST];
  thrust_act = thrust_in;

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
  init_butterworth_2_low_pass(&accely_filt, tau, sample_time, 0.0);
}

#include "firmwares/rotorcraft/navigation.h"
/**
 * @param heading_sp the desired heading [rad]
 *
 * main indi guidance function
 */
void guidance_indi_run(float *heading_sp) {

  /*Obtain eulers with zxy rotation order*/
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  /*Calculate the transition percentage so that the ctrl_effecitveness scheduling works*/
  transition_percentage = BFP_OF_REAL((eulers_zxy.theta/RadOfDeg(-75.0))*100,INT32_PERCENTAGE_FRAC);
  Bound(transition_percentage,0,BFP_OF_REAL(100.0,INT32_PERCENTAGE_FRAC));
  const int32_t max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
  transition_theta_offset = INT_MULT_RSHIFT((transition_percentage <<
        (INT32_ANGLE_FRAC - INT32_PERCENTAGE_FRAC)) / 100, max_offset, INT32_ANGLE_FRAC);

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters();

  //Linear controller to find the acceleration setpoint from position and velocity
  float pos_x_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x) - stateGetPositionNed_f()->x;
  float pos_y_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y) - stateGetPositionNed_f()->y;
  float pos_z_err = POS_FLOAT_OF_BFP(guidance_v_z_ref - stateGetPositionNed_i()->z);

  if(autopilot.mode == AP_MODE_NAV) {
    speed_sp = nav_get_speed_setpoint(gih_params.pos_gain);
  } else{
    speed_sp.x = pos_x_err * gih_params.pos_gain;
    speed_sp.y = pos_y_err * gih_params.pos_gain;
    speed_sp.z = pos_z_err * gih_params.pos_gainz;
  }
  VECT3_ASSIGN(pos_ref_c, POS_FLOAT_OF_BFP(navigation_target.y), POS_FLOAT_OF_BFP(navigation_target.x), -POS_FLOAT_OF_BFP(navigation_target.z));
  //for rc control horizontal, rotate from body axes to NED
  float psi = eulers_zxy.psi;
  /*NAV mode*/
  float speed_sp_b_x = cosf(psi) * speed_sp.x + sinf(psi) * speed_sp.y;
  float speed_sp_b_y =-sinf(psi) * speed_sp.x + cosf(psi) * speed_sp.y;

  float airspeed = stateGetAirspeed_f();

  struct NedCoor_f *groundspeed = stateGetSpeedNed_f();
  struct FloatVect2 airspeed_v = {cos(psi)*airspeed, sin(psi)*airspeed};
  struct FloatVect2 windspeed;
  VECT2_DIFF(windspeed, *groundspeed, airspeed_v);

  VECT2_DIFF(desired_airspeed, speed_sp, windspeed); // Use 2d part of speed_sp
  float norm_des_as = FLOAT_VECT2_NORM(desired_airspeed);

  // Make turn instead of straight line
  if((airspeed > 10.0) && (norm_des_as > 12.0)) {

  // Give the wind cancellation priority.
    if (norm_des_as > guidance_indi_max_airspeed) {
      float groundspeed_factor = 0.0;

      // if the wind is faster than we can fly, just fly in the wind direction
      if(FLOAT_VECT2_NORM(windspeed) < guidance_indi_max_airspeed) {
        float av = speed_sp.x * speed_sp.x + speed_sp.y * speed_sp.y;
        float bv = -2 * (windspeed.x * speed_sp.x + windspeed.y * speed_sp.y);
        float cv = windspeed.x * windspeed.x + windspeed.y * windspeed.y - guidance_indi_max_airspeed * guidance_indi_max_airspeed;

        float dv = bv * bv - 4.0 * av * cv;

        // dv can only be positive, but just in case
        if(dv < 0.0) {
          dv = fabs(dv);
        }
        float d_sqrt = sqrtf(dv);

        groundspeed_factor = (-bv + d_sqrt)  / (2.0 * av);
      }

      desired_airspeed.x = groundspeed_factor * speed_sp.x - windspeed.x;
      desired_airspeed.y = groundspeed_factor * speed_sp.y - windspeed.y;

      speed_sp_b_x = guidance_indi_max_airspeed;
    }

    // desired airspeed can not be larger than max airspeed
    speed_sp_b_x = Min(norm_des_as,guidance_indi_max_airspeed);

    if(force_forward) {
      speed_sp_b_x = guidance_indi_max_airspeed;
    }

    // Calculate accel sp in body axes, because we need to regulate airspeed
    struct FloatVect2 sp_accel_b;
    // In turn acceleration proportional to heading diff
    sp_accel_b.y = atan2(desired_airspeed.y, desired_airspeed.x) - psi;
    FLOAT_ANGLE_NORMALIZE(sp_accel_b.y);
    sp_accel_b.y *= gih_params.heading_bank_gain;

    // Control the airspeed
    sp_accel_b.x = (speed_sp_b_x - airspeed) * gih_params.speed_gain;

    sp_accel.x = cosf(psi) * sp_accel_b.x - sinf(psi) * sp_accel_b.y;
    sp_accel.y = sinf(psi) * sp_accel_b.x + cosf(psi) * sp_accel_b.y;

    sp_accel.z = (speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
  } else { // Go somewhere in the shortest way

    if(airspeed > 10.0) {
      // Groundspeed vector in body frame
      float groundspeed_x = cosf(psi) * stateGetSpeedNed_f()->x + sinf(psi) * stateGetSpeedNed_f()->y;
      float speed_increment = speed_sp_b_x - groundspeed_x;

      // limit groundspeed setpoint to max_airspeed + (diff gs and airspeed)
      if((speed_increment + airspeed) > guidance_indi_max_airspeed) {
        speed_sp_b_x = guidance_indi_max_airspeed + groundspeed_x - airspeed;
      }
    } else {
      // Groundspeed vector in body frame
      float groundspeed_x = cosf(psi) * stateGetSpeedNed_f()->x + sinf(psi) * stateGetSpeedNed_f()->y;
      speed_sp_b_x = Min(guidance_indi_max_airspeed + groundspeed_x - airspeed, speed_sp_b_x);
    }

    speed_sp.x = cosf(psi) * speed_sp_b_x - sinf(psi) * speed_sp_b_y;
    speed_sp.y = sinf(psi) * speed_sp_b_x + cosf(psi) * speed_sp_b_y;
    
    sp_accel.x = (speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;
    sp_accel.y = (speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain;
    sp_accel.z = (speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
  }
  
  // Bound the acceleration setpoint
  float accelbound = 3.0 + airspeed/guidance_indi_max_airspeed*5.0;
  vect_bound_in_2d(&sp_accel, accelbound);
  /*BoundAbs(sp_accel.x, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
  /*BoundAbs(sp_accel.y, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
  BoundAbs(sp_accel.z, 3.0);
  acc_body_ref_c.x = cosf(psi) * sp_accel.x + sinf(psi) * sp_accel.y;
  acc_body_ref_c.y = -sinf(psi) * sp_accel.x + cosf(psi) * sp_accel.y;
  acc_body_ref_c.z = sp_accel.z;

#if GUIDANCE_INDI_RC_DEBUG
#warning "GUIDANCE_INDI_RC_DEBUG lets you control the accelerations via RC, but disables autonomous flight!"
  //for rc control horizontal, rotate from body axes to NED
  float psi = eulers_zxy.psi;
  float rc_x = -(radio_control.values[RADIO_PITCH]/9600.0)*8.0;
  float rc_y = (radio_control.values[RADIO_ROLL]/9600.0)*8.0;
  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

  //for rc vertical control
  sp_accel.z = -(radio_control.values[RADIO_THROTTLE]-4500)*8.0/9600.0;
#endif
#ifdef GUIDANCE_INDI_HYBRID_ROT_WING
  //Calculate matrix of partial derivatives and invert matrix
  //guidance_indi_calcg_rot_wing();
#else

  //Calculate matrix of partial derivatives
  guidance_indi_calcg_wing(&Ga);
  //Invert this matrix
  MAT33_INV(Ga_inv, Ga);
#endif
  VECT3_ASSIGN(speed_ref_c,speed_sp.x,speed_sp.y,speed_sp.z)
  struct FloatVect3 accel_filt;
  accel_filt.x = filt_accel_ned[0].o[0];
  accel_filt.y = filt_accel_ned[1].o[0];
  accel_filt.z = filt_accel_ned[2].o[0];
  VECT3_ASSIGN(acc_body_c,accel_filt.x,accel_filt.y,accel_filt.z)
  struct FloatVect3 a_diff;
  a_diff.x = sp_accel.x - accel_filt.x;
  a_diff.y = sp_accel.y - accel_filt.y;
  a_diff.z = sp_accel.z - accel_filt.z;

  //Bound the acceleration error so that the linearization still holds
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);

  //If the thrust to specific force ratio has been defined, include vertical control
  //else ignore the vertical acceleration error
#ifndef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifndef STABILIZATION_ATTITUDE_INDI_FULL
  a_diff.z = 0.0;
#endif
#endif

  //Calculate roll,pitch and thrust command
#ifdef GUIDANCE_INDI_HYBRID_ROT_WING
  guidance_indi_calcg_rot_wing_wls(a_diff);
  euler_cmd.x = hybrid_du[0];
  euler_cmd.y = hybrid_du[1];
  euler_cmd.z = -1.0*hybrid_du[2];
  acc_T_bx = hybrid_du[3];
  //printf("Commanded Delta T = %f\n", euler_cmd.z);
#else
  MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);
#endif
 // add chirp values to euler cmd  AP_MODE_HOVER_Z_HOLD
  if (autopilot.mode == AP_MODE_HOVER_Z_HOLD){
    //printf("AUTOPILOT check PASSED \n");
    //printf("autorpilot_mode= %s \n",autopilot.mode);
      for (int8_t i = 0; i < 2; i++) {  
        if (chirp_active){
          if(i==chirp_axis){
            if(!chirp_init_check){
              if(i==0){chirp_val_init = roll_filt.o[0];}
              if(i==1){chirp_val_init = pitch_filt.o[0];}
              chirp_init_check = TRUE;
              chirp_number += 1;
            }
            if(i==0){euler_cmd.x = chirp_val_init+current_chirp_values[0]-roll_filt.o[0];}
            if(i==1){euler_cmd.y = chirp_val_init+pitch_pref_rad+current_chirp_values[1]-pitch_filt.o[0];
            acc_T_bx=0.0;}
          }   
        } else {
          chirp_init_check = FALSE;
          chirp_val_init = 0;
        }
      }
    }
  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, euler_cmd.z);
  AbiSendMsgTHRUST(THRUST_BX_INCREMENT_ID, acc_T_bx);

  //Coordinated turn
  //feedforward estimate angular rotation omega = g*tan(phi)/v
  float omega;
  const float max_phi = RadOfDeg(60.0);
  float airspeed_turn = airspeed;
  //We are dividing by the airspeed, so a lower bound is important
  Bound(airspeed_turn,10.0,30.0);

  guidance_euler_cmd.phi = roll_filt.o[0] + euler_cmd.x;
  guidance_euler_cmd.theta = pitch_filt.o[0] + euler_cmd.y;

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, RadOfDeg(-25.0), RadOfDeg(25.0));

  // Use the current roll angle to determine the corresponding heading rate of change.
  float coordinated_turn_roll = eulers_zxy.phi;

  if( (guidance_euler_cmd.theta > 0.0) && ( fabs(guidance_euler_cmd.phi) < guidance_euler_cmd.theta)) {
    coordinated_turn_roll = ((guidance_euler_cmd.phi > 0.0) - (guidance_euler_cmd.phi < 0.0))*guidance_euler_cmd.theta;
  }

  if (fabs(coordinated_turn_roll) < max_phi) {
    omega = 9.81 / airspeed_turn * tanf(coordinated_turn_roll);
    printf("omega = %f \n", omega);
  } else { //max 60 degrees roll
    omega = 9.81 / airspeed_turn * 1.72305 * ((coordinated_turn_roll > 0.0) - (coordinated_turn_roll < 0.0));
  }

#ifdef FWD_SIDESLIP_GAIN
  // Add sideslip correction
  omega -= accely_filt.o[0]*fwd_sideslip_g;
#endif

// For a hybrid it is important to reduce the sideslip, which is done by changing the heading.
// For experiments, it is possible to fix the heading to a different value.
#ifndef KNIFE_EDGE_TEST
  if (guidance_h.mode == GUIDANCE_H_MODE_HOVER) {
    // Dont change heading setpoint
  } else if(take_heading_control) {
    *heading_sp = ANGLE_FLOAT_OF_BFP(nav_heading) + current_chirp_values[2];
    FLOAT_ANGLE_NORMALIZE(*heading_sp);
  } else {
    *heading_sp += omega / PERIODIC_FREQUENCY;
    FLOAT_ANGLE_NORMALIZE(*heading_sp);
  }
#endif

  guidance_euler_cmd.psi = *heading_sp;
  phi_ref_c = guidance_euler_cmd.phi;
  theta_ref_c = guidance_euler_cmd.theta;
  psi_ref_c = guidance_euler_cmd.psi; 
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
  QUAT_BFP_OF_REAL(stab_att_sp_quat,sp_quat);
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
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * w.r.t. the NED accelerations, taking into account the lift of a wing that is
 * horizontal at -90 degrees pitch
 *
 * @param Gmat array to write the matrix to [3x3]
 */
void guidance_indi_calcg_wing(struct FloatMat33 *Gmat) {

  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_zxy.phi);
  float cphi = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi = sinf(eulers_zxy.psi);
  float cpsi = cosf(eulers_zxy.psi);
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  /*Amount of lift produced by the wing*/
  float pitch_lift = eulers_zxy.theta;
  Bound(pitch_lift,-M_PI_2,0);
  float lift = sinf(pitch_lift)*9.81;
  float T = cosf(pitch_lift)*-9.81;

  // get the derivative of the lift wrt to theta
  float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);

  RMAT_ELMT(*Gmat, 0, 0) =  cphi*ctheta*spsi*T + cphi*spsi*lift;
  RMAT_ELMT(*Gmat, 1, 0) = -cphi*ctheta*cpsi*T - cphi*cpsi*lift;
  RMAT_ELMT(*Gmat, 2, 0) = -sphi*ctheta*T -sphi*lift;
  RMAT_ELMT(*Gmat, 0, 1) = (ctheta*cpsi - sphi*stheta*spsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING + sphi*spsi*liftd;
  RMAT_ELMT(*Gmat, 1, 1) = (ctheta*spsi + sphi*stheta*cpsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*cpsi*liftd;
  RMAT_ELMT(*Gmat, 2, 1) = -cphi*stheta*T*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;
  RMAT_ELMT(*Gmat, 0, 2) = stheta*cpsi + sphi*ctheta*spsi;
  RMAT_ELMT(*Gmat, 1, 2) = stheta*spsi - sphi*ctheta*cpsi;
  RMAT_ELMT(*Gmat, 2, 2) = cphi*ctheta;
}

/**
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * w.r.t. the NED accelerations, taking into account the lift of a wing that is
 * horizontal at 0 degrees pitch
 *
 */
void guidance_indi_calcg_rot_wing(void) {
  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_zxy.phi);
  float cphi = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi = sinf(eulers_zxy.psi);
  float cpsi = cosf(eulers_zxy.psi);
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  float lift_thrust_bz = -9.81; // Sum of lift and thrust in boxy z axis (level flight)
  float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta - M_PI_2); // Convert to correct pitch angle

  // Calc assumed body acceleration setpoint and error
  float accel_bx_sp = cpsi * sp_accel.x + spsi * sp_accel.y;
  float accel_bx = cpsi * filt_accel_ned[0].o[0] + spsi * filt_accel_ned[1].o[0];
  float accel_bx_err = accel_bx_sp - accel_bx;

  if (true) {
  //if (accel_bx_err < 0 && actuator_thrust_bx_pprz <= 0) {
    RMAT_ELMT(Ga, 0, 0) = cphi*spsi*lift_thrust_bz;
    RMAT_ELMT(Ga, 1, 0) = -cphi*cpsi*lift_thrust_bz;
    RMAT_ELMT(Ga, 2, 0) = -sphi*lift_thrust_bz;
    RMAT_ELMT(Ga, 0, 1) = (ctheta*cpsi - sphi*stheta*spsi)*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + sphi*spsi*liftd;
    RMAT_ELMT(Ga, 1, 1) = (ctheta*spsi + sphi*stheta*cpsi)*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*cpsi*liftd;;
    RMAT_ELMT(Ga, 2, 1) = -cphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;
    RMAT_ELMT(Ga, 0, 2) = stheta*cpsi + sphi*ctheta*spsi;
    RMAT_ELMT(Ga, 1, 2) = stheta*spsi - sphi*ctheta*cpsi;
    RMAT_ELMT(Ga, 2, 2) = cphi*ctheta;

    //Calculate matrix of partial derivatives
    //Invert this matrix
    MAT33_INV(Ga_inv, Ga);

    T_bx_effective = false;

  } else {
    Gmat_rot_wing[0][0] = cphi*spsi*lift_thrust_bz;
    Gmat_rot_wing[1][0] = -cphi*cpsi*lift_thrust_bz;
    Gmat_rot_wing[2][0] = -sphi*lift_thrust_bz;

    Gmat_rot_wing[0][1] = (ctheta*cpsi - sphi*stheta*spsi)*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + sphi*spsi*liftd;
    Gmat_rot_wing[1][1] = (ctheta*spsi + sphi*stheta*cpsi)*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*cpsi*liftd;
    Gmat_rot_wing[2][1] = -cphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;

    Gmat_rot_wing[0][2] = stheta*cpsi + sphi*ctheta*spsi;
    Gmat_rot_wing[1][2] = stheta*spsi - sphi*ctheta*cpsi;
    Gmat_rot_wing[2][2] = cphi*ctheta;

    Gmat_rot_wing[0][3] = ctheta*cpsi - sphi*stheta*spsi;
    Gmat_rot_wing[1][3] = ctheta*spsi + sphi*stheta*cpsi;
    Gmat_rot_wing[2][3] = -cphi*stheta;

    // Don't pitch down if acc bx_err > 0 and pitch is greater than 0
    if (accel_bx_err > 0 && eulers_zxy.theta < 0) {
      Gmat_rot_wing[0][1] = 0;
      Gmat_rot_wing[1][1] = 0;
    }

    if (accel_bx_err < 0 && eulers_zxy.theta > 0 && actuator_thrust_bx_pprz > 0) {
      Gmat_rot_wing[0][1] = Gmat_rot_wing[0][1];
      Gmat_rot_wing[1][1] = Gmat_rot_wing[1][1];
      Gmat_rot_wing[0][3] = Gmat_rot_wing[0][3];
      Gmat_rot_wing[1][3] = Gmat_rot_wing[0][3];
    }

    if (accel_bx_err < 0 && actuator_thrust_bx_pprz <= 0) {
      Gmat_rot_wing[0][3] = 0;
      Gmat_rot_wing[1][3] = 0;
    }

    T_bx_effective = true;

    // Invert Gmat_rot_wing

    //Gmat_rot_wing * transpose()
    //calculate matrix multiplication of its transpose num_act*HYBRID_INDI_OUTPUTS x HYBRID_INDI_OUTPUTS*num_act
    float element = 0;
    int8_t row;
    int8_t col;
    int8_t i;
    for (row = 0; row < 3; row++) {
      for (col = 0; col < 3; col++) {
        element = 0;
        for (i = 0; i < 4; i++) {
          element = element + Gmat_rot_wing[row][i] * Gmat_rot_wing[col][i];
        }
        Gmat_rot_wing_trans_mult[row][col] = element;
      }
    }

    for (row = 0; row < 3; row++) {
      for (col = 0; col < 3; col++) {
         RMAT_ELMT(Ga, row, col) = Gmat_rot_wing_trans_mult[row][col];
      }
    }

    MAT33_INV(Ga_inv, Ga);

    //Gmat' * Ga_inv
    //calculate matrix multiplication INDI_NUM_ACTxINDI_NUM_ACT x HYBRID_INDI_
    for (row = 0; row < 4; row++) {
      for (col = 0; col < 3; col++) {
        element = 0;
        for (i = 0; i < 3; i++) {
          element = element + Ga_inv.m[i*3+col] * Gmat_rot_wing[i][row];
        }
        Gmat_rot_wing_pseudo_inv[row][col] = element;
      }
    }
  }
}

/**
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * w.r.t. the NED accelerations, taking into account the lift of a wing that is
 * horizontal at 0 degrees pitch
 *
 */
void guidance_indi_calcg_rot_wing_wls(struct FloatVect3 a_diff) {
  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_zxy.phi);
  float cphi = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi = sinf(eulers_zxy.psi);
  float cpsi = cosf(eulers_zxy.psi);
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  float lift_thrust_bz = stateGetAccelNed_f()->z-9.81; // Sum of lift and thrust in boxy z axis (level flight) 
  //Bound(lift_thrust_bz,0.0,10.0)
  float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta); //IS THIS RIGHT?// Convert to correct pitch angle
  float thrust_bz = (actuators_pprz[0] + actuators_pprz[1] + actuators_pprz[2] + actuators_pprz[3])*(-0.00051);//(g1g2[3][0]);
  float lift_approx = lift_thrust_bz-thrust_bz*cphi*ctheta;
  float thrust_bx = actuator_thrust_bx_pprz*THRUST_BX_EFF;
  // Calc assumed body acceleration setpoint and error
  float accel_bx_sp = cpsi * sp_accel.x + spsi * sp_accel.y;
  float accel_bx = cpsi * filt_accel_ned[0].o[0] + spsi * filt_accel_ned[1].o[0];
  float accel_bx_err = accel_bx_sp - accel_bx;

  Gmat_rot_wing[0][0] = (cphi*spsi-cpsi*sphi*stheta)*lift_thrust_bz;// cphi*spsi*lift_thrust_bz;
  Gmat_rot_wing[1][0] = (-cphi*cpsi-sphi*spsi*stheta)*lift_thrust_bz;// -cphi*cpsi*lift_thrust_bz;
  Gmat_rot_wing[2][0] = -sphi*ctheta*lift_thrust_bz;;//-sphi*ctheta*lift_thrust_bz;// -sphi*lift_thrust_bz;

  Gmat_rot_wing[0][1] = cpsi*ctheta*cphi*thrust_bz;//cpsi*ctheta*lift_thrust_bz-cpsi*stheta*thrust_bx+sphi*spsi*liftd;//cpsi*cphi*ctheta*thrust_bz-cpsi*stheta*thrust_bx+sphi*spsi*liftd;// (ctheta*cpsi - sphi*stheta*spsi)*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + sphi*spsi*liftd;
  Gmat_rot_wing[1][1] = spsi*cphi*ctheta*thrust_bz;//spsi*cphi*ctheta*thrust_bz-spsi*stheta*thrust_bx-cpsi*sphi*liftd;// (ctheta*spsi + sphi*stheta*cpsi)*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*cpsi*liftd;
  Gmat_rot_wing[2][1] = cphi*liftd;// -cphi*stheta*thrust_bz-ctheta*thrust_bx+cphi*liftd;// -cphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;

  Gmat_rot_wing[0][2] = (sphi*spsi+cphi*cpsi*stheta)*-1.0;// (stheta*cpsi + sphi*ctheta*spsi)*-1.0;
  Gmat_rot_wing[1][2] = (cphi*spsi*stheta-cpsi*sphi)*-1.0;// (stheta*spsi - sphi*ctheta*cpsi)*-1.0;
  Gmat_rot_wing[2][2] = (cphi*ctheta)*-1.0;// (cphi*ctheta)*-1.0;

  Gmat_rot_wing[0][3] = cpsi*ctheta;// ctheta*cpsi - sphi*stheta*spsi;
  Gmat_rot_wing[1][3] = ctheta*spsi;// ctheta*spsi + sphi*stheta*cpsi;
  Gmat_rot_wing[2][3] = -stheta;// -cphi*stheta;

  // Perform WLS
  // WLS Control Allocator

  hybrid_v[0] = a_diff.x;
  hybrid_v[1] = a_diff.y;
  hybrid_v[2] = a_diff.z;

  // Set lower limits
  du_min_hybrid[0] = -hybrid_roll_limit - roll_filt.o[0]; //roll
  du_min_hybrid[1] = -hybrid_pitch_limit - pitch_filt.o[0]; // pitch
  du_min_hybrid[2] = (actuators_pprz[0] + actuators_pprz[1] + actuators_pprz[2] + actuators_pprz[3])*(g1g2[3][0]);// + g1g2[3][1] + g1g2[3][2] + g1g2[3][3]);
  du_min_hybrid[3] = -actuator_thrust_bx_pprz*THRUST_BX_EFF;
  // Set upper limits limits
  du_max_hybrid[0] = hybrid_roll_limit - roll_filt.o[0]; //roll
  du_max_hybrid[1] = hybrid_pitch_limit - pitch_filt.o[0]; // pitch
  du_max_hybrid[2] = -(4*MAX_PPRZ - actuators_pprz[0] - actuators_pprz[1] - actuators_pprz[2] - actuators_pprz[3])*(g1g2[3][0]);// + g1g2[3][1] + g1g2[3][2] + g1g2[3][3]);
  du_max_hybrid[3] = (MAX_PPRZ - actuator_thrust_bx_pprz) * THRUST_BX_EFF;
  // printf("du_min_hibrid[2] %f\n", du_min_hybrid[2]);
  // printf("Gmat_rot_wing[2][1] %f\n", Gmat_rot_wing[2][1]);
  printf("liftd %f\n", liftd);
  // printf("Gmat_rot_wing[2][2] %f\n", Gmat_rot_wing[2][2]);
  // printf("Lift estimated %f\n", lift_approx);
  
  // Set prefered states
  pitch_pref_rad = pitch_pref_deg / 180. * M_PI;

  du_pref_hybrid[0] = -roll_filt.o[0];
  printf("pref change in roll %f\n", du_pref_hybrid[0]);
  du_pref_hybrid[1] = -pitch_filt.o[0] + pitch_pref_rad;
  du_pref_hybrid[2] = du_min_hybrid[2];//du_max_hybrid[2];//0;
  du_pref_hybrid[3] = du_min_hybrid[3];//accel_bx_err * cosf(pitch_pref_rad) - 9.81 * sinf(pitch_filt.o[0] - pitch_pref_rad);
  Bound(du_pref_hybrid[3], du_min_hybrid[3], du_max_hybrid[3]);

  Wu_hybrid[0] = roll_priority_factor * 10.414;
  Wu_hybrid[1] = pitch_priority_factor * 23.424;
  Wu_hybrid[2] = thrust_priority_factor * 0.626;
  Wu_hybrid[3] = pusher_priority_factor * 1.0;

  num_iter_hybrid =
    wls_alloc_hybrid(hybrid_du, hybrid_v, du_min_hybrid, du_max_hybrid, Bwls_hybrid, 0, 0, Wv_hybrid, Wu_hybrid, du_pref_hybrid, 1000000, 20);
}

/**
 * @brief Get the derivative of lift w.r.t. pitch.
 *
 * @param airspeed The airspeed says most about the flight condition
 *
 * @return The derivative of lift w.r.t. pitch
 */
float guidance_indi_get_liftd(float airspeed, float theta) {
  float liftd;// = 0.0;
  if(airspeed < 5) {
    //float pitch_interp = DegOfRad(theta);
    //Bound(pitch_interp, -80.0, -40.0);
    //float ratio = (pitch_interp + 40.0)/(-40.);
    // liftd = -24.0*ratio*lift_pitch_eff/0.12;
    liftd = 0.0;
  } else {
    if(DegOfRad(theta)<14){
    liftd = -airspeed*airspeed*lift_pitch_eff/M_PI*180.0;
    if (liftd > 0) {
      liftd = 0.0;
    }} else{
      printf("STALL\n");
      liftd = airspeed*airspeed*lift_pitch_eff/M_PI*180.0;
    }
  }
  //TODO: bound liftd
  return liftd;
}

/**
 * @brief function that returns a speed setpoint based on flight plan.
 *
 * The routines are meant for a hybrid UAV and assume measurement of airspeed.
 * Makes the vehicle track a vector field with a sink at a waypoint.
 * Use force_forward to maintain airspeed and fly 'through' waypoints.
 *
 * @return desired speed setpoint FloatVect3
 */
struct FloatVect3 nav_get_speed_setpoint(float pos_gain) {
  struct FloatVect3 speed_sp;
  if(horizontal_mode == HORIZONTAL_MODE_ROUTE) {
    speed_sp = nav_get_speed_sp_from_line(line_vect, to_end_vect, navigation_target, pos_gain);
  } else {
    speed_sp = nav_get_speed_sp_from_go(navigation_target, pos_gain);
  }
  return speed_sp;
}

/**
 * @brief follow a line.
 *
 * @param line_v_enu 2d vector from beginning (0) line to end in enu
 * @param to_end_v_enu 2d vector from current position to end in enu
 * @param target end waypoint in enu
 *
 * @return desired speed setpoint FloatVect3
 */
struct FloatVect3 nav_get_speed_sp_from_line(struct FloatVect2 line_v_enu, struct FloatVect2 to_end_v_enu, struct EnuCoor_i target, float pos_gain) {

  // enu -> ned
  struct FloatVect2 line_v = {line_v_enu.y, line_v_enu.x};
  struct FloatVect2 to_end_v = {to_end_v_enu.y, to_end_v_enu.x};

  struct NedCoor_f ned_target;
  // Target in NED instead of ENU
  VECT3_ASSIGN(ned_target, POS_FLOAT_OF_BFP(target.y), POS_FLOAT_OF_BFP(target.x), -POS_FLOAT_OF_BFP(target.z));

  // Calculate magnitude of the desired speed vector based on distance to waypoint
  float dist_to_target = float_vect2_norm(&to_end_v);
  float desired_speed;
  if(force_forward) {
    desired_speed = nav_max_speed;
  } else {
    desired_speed = dist_to_target * pos_gain;
    Bound(desired_speed, 0.0, nav_max_speed);
  }

  // Calculate length of line segment
  float length_line = float_vect2_norm(&line_v);
  if(length_line < 0.01) {
    length_line = 0.01;
  }

  //Normal vector to the line, with length of the line
  struct FloatVect2 normalv;
  VECT2_ASSIGN(normalv, -line_v.y, line_v.x);
  // Length of normal vector is the same as of the line segment
  float length_normalv = length_line;
  if(length_normalv < 0.01) {
    length_normalv = 0.01;
  }

  // Distance along the normal vector
  float dist_to_line = (to_end_v.x*normalv.x + to_end_v.y*normalv.y)/length_normalv;

  // Normal vector scaled to be the distance to the line
  struct FloatVect2 v_to_line, v_along_line;
  v_to_line.x = dist_to_line*normalv.x/length_normalv*guidance_indi_line_gain;
  v_to_line.y = dist_to_line*normalv.y/length_normalv*guidance_indi_line_gain;

  // Depending on the normal vector, the distance could be negative
  float dist_to_line_abs = fabs(dist_to_line);

  // The distance that needs to be traveled along the line
  /*float dist_along_line = (line_v.x*to_end_v.x + line_v.y*to_end_v.y)/length_line;*/
  v_along_line.x = line_v.x/length_line*50.0;
  v_along_line.y = line_v.y/length_line*50.0;

  // Calculate the desired direction to converge to the line
  struct FloatVect2 direction;
  VECT2_SMUL(direction, v_along_line, (1.0/(1+dist_to_line_abs*0.05)));
  VECT2_ADD(direction, v_to_line);
  float length_direction = float_vect2_norm(&direction);
  if(length_direction < 0.01) {
    length_direction = 0.01;
  }

  // Scale to have the desired speed
  struct FloatVect2 final_vector;
  VECT2_SMUL(final_vector, direction, desired_speed/length_direction);

  struct FloatVect3 speed_sp_return = {final_vector.x, final_vector.y, gih_params.pos_gainz*(ned_target.z - stateGetPositionNed_f()->z)};
  if((guidance_v_mode == GUIDANCE_V_MODE_NAV) && (vertical_mode == VERTICAL_MODE_CLIMB)) {
    speed_sp_return.z = SPEED_FLOAT_OF_BFP(guidance_v_zd_sp);
  }

  // Bound vertical speed setpoint
  if(stateGetAirspeed_f() > 13.0) {
    Bound(speed_sp_return.z, -4.0, 5.0);
  } else {
    Bound(speed_sp_return.z, -nav_climb_vspeed, -nav_descend_vspeed);
  }

  return speed_sp_return;
}

/**
 * @brief Go to a waypoint in the shortest way
 *
 * @param target the target waypoint
 *
 * @return desired speed FloatVect3
 */
struct FloatVect3 nav_get_speed_sp_from_go(struct EnuCoor_i target, float pos_gain) {
  // The speed sp that will be returned
  struct FloatVect3 speed_sp_return;
  struct NedCoor_f ned_target;
  // Target in NED instead of ENU
  VECT3_ASSIGN(ned_target, POS_FLOAT_OF_BFP(target.y), POS_FLOAT_OF_BFP(target.x), -POS_FLOAT_OF_BFP(target.z));
  // Calculate position error
  struct FloatVect3 pos_error;
  struct NedCoor_f *pos = stateGetPositionNed_f();
  VECT3_DIFF(pos_error, ned_target, *pos);

  VECT3_SMUL(speed_sp_return, pos_error, pos_gain);
  speed_sp_return.z = gih_params.pos_gainz*pos_error.z;

  if((guidance_v_mode == GUIDANCE_V_MODE_NAV) && (vertical_mode == VERTICAL_MODE_CLIMB)) {
    speed_sp_return.z = SPEED_FLOAT_OF_BFP(guidance_v_zd_sp);
  }

  if(force_forward) {
    vect_scale(&speed_sp_return, nav_max_speed);
  } else {
    // Calculate distance to waypoint
    float dist_to_wp = FLOAT_VECT2_NORM(pos_error);

    // Calculate max speed to decelerate from

    // dist_to_wp can only be positive, but just in case
    float max_speed_decel2 = 2*dist_to_wp*MAX_DECELERATION;
    if(max_speed_decel2 < 0.0) {
      max_speed_decel2 = fabs(max_speed_decel2);
    }
    float max_speed_decel = sqrtf(max_speed_decel2);

    // Bound the setpoint velocity vector
    float max_h_speed = Min(nav_max_speed, max_speed_decel);
    vect_bound_in_2d(&speed_sp_return, max_h_speed);
  }

  // Bound vertical speed setpoint
  if(stateGetAirspeed_f() > 13.0) {
    Bound(speed_sp_return.z, -4.0, 5.0);
  } else {
    Bound(speed_sp_return.z, -nav_climb_vspeed, -nav_descend_vspeed);
  }

  return speed_sp_return;
}
