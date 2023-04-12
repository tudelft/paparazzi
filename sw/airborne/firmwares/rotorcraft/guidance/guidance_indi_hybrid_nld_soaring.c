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
 * Modified by Sunyou for normal fixed wing vehicles (NLD)
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid_nld_soaring.h"
#include "modules/ins/ins_int.h"
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "modules/imu/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "stdio.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "wls/wls_alloc_gd.h"
#include "generated/flight_plan.h" // for waypoint reference pointers
//#include "subsystems/navigation/common_nav.h"
#include "modules/nav/waypoints.h"
#include <time.h>


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
//        .liftd_asq = GUIDANCE_INDI_LIFTD_ASQ, // coefficient of airspeed squared
//        .liftd_p80 = GUIDANCE_INDI_LIFTD_P80,
//        .liftd_p50 = GUIDANCE_INDI_LIFTD_P50,
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
bool take_heading_control = false;

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

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;

struct FloatEulers guidance_euler_cmd;
float thrust_in;

struct FloatVect3 speed_sp = {0.0, 0.0, 0.0};
struct FloatVect3 guidance_wind_gradient = {0.0, 0.0, 0.0};

#ifndef GUIDANCE_INDI_SPEED_SP_FROM_GRADIENT
#define GUIDANCE_INDI_SPEED_SP_FROM_GRADIENT FALSE
#endif
#ifndef GUIDANCE_INDI_GRAD_GAIN_X
#define GUIDANCE_INDI_GRAD_GAIN_X 1.0
#endif
#ifndef GUIDANCE_INDI_GRAD_GAIN_Z
#define GUIDANCE_INDI_GRAD_GAIN_Z 1.0
#endif
#ifndef GUIDANCE_INDI_GRAD_GAIN_REC_X
#define GUIDANCE_INDI_GRAD_GAIN_REC_X 1.0
#endif
#ifndef GUIDANCE_INDI_GRAD_GAIN_REC_Z
#define GUIDANCE_INDI_GRAD_GAIN_REC_Z 1.0
#endif
#ifndef GUIDANCE_INDI_POS_CTRL
#define GUIDANCE_INDI_POS_CTRL FALSE
#endif
#ifndef GUIDANCE_INDI_MAX_THROTTLE
#define GUIDANCE_INDI_MAX_THROTTLE 9600
#endif
#ifndef GUIDANCE_INDI_PREF_THROTTLE
#define GUIDANCE_INDI_PREF_THROTTLE 0.0
#endif
#ifndef GUIDANCE_INDI_Y_POSITION_CTRL
#define GUIDANCE_INDI_Y_POSITION_CTRL FALSE
#endif

// Moving WP
#ifndef GUIDANCE_INDI_SOARING_WP_ID
#define GUIDANCE_INDI_SOARING_WP_ID 3
#endif
#ifndef GUIDANCE_INDI_SOARING_MOVE_WP
#define GUIDANCE_INDI_SOARING_MOVE_WP FALSE
#endif
#ifndef GUIDANCE_INDI_SOARING_WP_W_THROTTLE
#define GUIDANCE_INDI_SOARING_WP_W_THROTTLE 0.1
#endif
#ifndef GUIDANCE_INDI_SOARING_WP_W_SPD_X
#define GUIDANCE_INDI_SOARING_WP_W_SPD_X 1.0
#endif
#ifndef GUIDANCE_INDI_SOARING_WP_W_SPD_Z
#define GUIDANCE_INDI_SOARING_WP_W_SPD_Z 1.0
#endif
#ifndef GUIDANCE_INDI_SOARING_WP_W_PITCH
#define GUIDANCE_INDI_SOARING_WP_W_PITCH 10.0
#endif
#ifndef GUIDANCE_INDI_SOARING_WP_WAIT_SEC
#define GUIDANCE_INDI_SOARING_WP_WAIT_SEC 10
#endif
#ifndef GUIDANCE_INDI_SOARING_WP_COST_THRES
#define GUIDANCE_INDI_SOARING_WP_COST_THRES 50
#endif


// 2m/0.1 = 20 data points for one axis
#define MAP_MAX_NUM_POINTS 400

bool speed_sp_from_gradient = GUIDANCE_INDI_SPEED_SP_FROM_GRADIENT;
bool speed_sp_from_position = GUIDANCE_INDI_POS_CTRL;
bool y_position_ctrl = GUIDANCE_INDI_Y_POSITION_CTRL;
float guidance_grad_gain_x = GUIDANCE_INDI_GRAD_GAIN_X;
float guidance_grad_gain_z = GUIDANCE_INDI_GRAD_GAIN_Z;
float guidance_grad_gain_rec_x = GUIDANCE_INDI_GRAD_GAIN_REC_X;
float guidance_grad_gain_rec_z = GUIDANCE_INDI_GRAD_GAIN_REC_Z;
float guidance_soaring_max_throttle = GUIDANCE_INDI_MAX_THROTTLE;

// WLS alloc    TODO: Weights and gamma_sq to tune
float linear_acc_diff[3];
float indi_d_euler[3];
float indi_euler_cmd[3];
float euler_state_filt_vect[3];
float gd_gamma_sq = 1000.;
float gd_Wv[3] = {1.0, 1.0, 1.0};         // x y z
float gd_Wu[3] = {1.0, 100.0, 1.0};         // roll pitch thrust
float gd_du_min[3];
float gd_du_max[3];
float gd_du_pref[3];
float *Bwls_g[3];
float g_arr[3][3];
int gd_num_iter = 0;
float pos_x_err = 0.;
float pos_y_err = 0.;
float pos_z_err = 0.;

struct FloatVect2 heading_target = {50., 0.};
struct FloatVect3 soaring_spd_sp = {0., 0., 0.};

float gd_k_thr = GUIDANCE_INDI_SOARING_WP_W_THROTTLE;
float gd_k_spd_x = GUIDANCE_INDI_SOARING_WP_W_SPD_X;
float gd_k_spd_z = GUIDANCE_INDI_SOARING_WP_W_SPD_Z;
float gd_k_pitch = GUIDANCE_INDI_SOARING_WP_W_PITCH;
float soaring_move_wp_wait_sec = GUIDANCE_INDI_SOARING_WP_WAIT_SEC;
float soaring_move_wp_cost_threshold = GUIDANCE_INDI_SOARING_WP_COST_THRES;
bool soaring_explore_positions = GUIDANCE_INDI_SOARING_MOVE_WP;

struct SoaringPositionMap soaring_position_map[MAP_MAX_NUM_POINTS];
struct FloatVect3 preset_move_ned[4] = {{1.0, 0., 0.}, {0., 0., 1.0}, {0., 0., -1.0}, {-1.0, 0., 0.}};
struct FloatVect3 amount_to_move_ned = {0., 0., 0.};
int soar_map_idx = 0;
float prev_wp_sum_cost = -1;
int prev_move_idx = 0;
int move_wp_wait_count = 0;
float move_wp_sum_cost = 0.;
struct FloatVect3 wp_soaring_pos;
uint8_t soar_wp_id = GUIDANCE_INDI_SOARING_WP_ID;
bool soaring_move_wp_running = false;
bool soaring_use_fixed_step_size = false;
float soaring_fixed_step_size = 0.2;

float soaring_step_k_big = 3;
float soaring_step_k_mid = 2;
float soaring_step_k_small = 1.5;

float soaring_step_size_big = 0.3;
float soaring_step_size_mid = 0.2;
float soaring_step_size_small = 0.1;
float soaring_step_size_fine = 0.05;

int32_t wp_pos_x;
int32_t wp_pos_y;
int32_t wp_pos_z;

time_t rand_seed;

void guidance_indi_soaring_propagate_filters(void);
static void guidance_indi_calcg_wing(struct FloatMat33 *Gmat);
static float guidance_indi_get_liftd(float pitch, float theta);
struct FloatVect3 nav_get_speed_sp_from_go(struct EnuCoor_i target, float pos_gain);
struct FloatVect3 nav_get_speed_sp_from_line(struct FloatVect2 line_v_enu, struct FloatVect2 to_end_v_enu, struct EnuCoor_i target, float pos_gain);
struct FloatVect3 nav_get_speed_setpoint(float pos_gain);

void guidance_indi_soaring_move_wp(float cost_avg_val);
void write_map_position_cost_info(struct FloatVect3 soaring_position, float corres_sum_cost);
void guidance_indi_hybrid_soaring_stop(void);
void guidance_indi_hybrid_soaring_start(void);

#define DEG2RAD 0.017

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_guidance_indi_hybrid(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_INDI_HYBRID_SOARING(trans, dev, AC_ID,
                              &sp_accel.x,
                              &sp_accel.y,
                              &sp_accel.z,
                              &indi_d_euler[0],
                              &indi_d_euler[1],
                              &indi_d_euler[2],
                              &filt_accel_ned[0].o[0],
                              &filt_accel_ned[1].o[0],
                              &filt_accel_ned[2].o[0],
                              &speed_sp.x,
                              &speed_sp.y,
                              &speed_sp.z,
                              &guidance_euler_cmd.phi,
                              &guidance_euler_cmd.theta,
                              &guidance_euler_cmd.psi,
                              &pos_x_err,
                              &pos_y_err,
                              &pos_z_err,
                              &roll_filt.o[0],
                              &pitch_filt.o[0],
                              &eulers_zxy.psi,
                              &gd_num_iter
                              );
}
static void send_soaring_wp_map(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SOARING_WP_MAP(trans, dev, AC_ID,
                               &wp_pos_x,
                               &wp_pos_y,
                               &wp_pos_z,
                               &move_wp_sum_cost,
                               &prev_wp_sum_cost,
                               &prev_move_idx,
                               &amount_to_move_ned.x,
                               &amount_to_move_ned.z
                              );
}
#endif

/**
 * @brief Init function
 */
void guidance_indi_soaring_init(void)
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

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID_SOARING, send_guidance_indi_hybrid);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SOARING_WP_MAP, send_soaring_wp_map);
#endif
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_soaring_enter(void) {
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
void guidance_indi_soaring_move_wp(float cost_avg_val){
    int idx_to_move = prev_move_idx;

    if (prev_wp_sum_cost < 0) {
        // pick a random neighbour
        idx_to_move = abs((int)(filt_accel_ned[2].o[0]*100))%4;
        prev_move_idx = idx_to_move;
        prev_wp_sum_cost = cost_avg_val;
    } else if(cost_avg_val < prev_wp_sum_cost) {
        // keep going
//        amount_to_move_ned = preset_move_ned[prev_move_idx];
        prev_wp_sum_cost = cost_avg_val;
        idx_to_move = prev_move_idx;
    } else {
        // go back
//        amount_to_move_ned = preset_move_ned[3-prev_move_idx];
        idx_to_move = 3-prev_move_idx;
        prev_wp_sum_cost = -1.0;
    }

    // Set a step size
    if (soaring_use_fixed_step_size) {
        VECT3_SMUL(amount_to_move_ned, preset_move_ned[idx_to_move], soaring_fixed_step_size);
    } else if (cost_avg_val > (soaring_move_wp_cost_threshold*soaring_step_k_big)) {
        VECT3_SMUL(amount_to_move_ned, preset_move_ned[idx_to_move], soaring_step_size_big);
    } else if (cost_avg_val > (soaring_move_wp_cost_threshold*soaring_step_k_mid)) {
        VECT3_SMUL(amount_to_move_ned, preset_move_ned[idx_to_move], soaring_step_size_mid);
    } else if (cost_avg_val > (soaring_move_wp_cost_threshold*soaring_step_k_small)) {
        VECT3_SMUL(amount_to_move_ned, preset_move_ned[idx_to_move], soaring_step_size_small);
    } else {
        VECT3_SMUL(amount_to_move_ned, preset_move_ned[idx_to_move], soaring_step_size_fine);
    }

    // move waypoints
    waypoints[soar_wp_id].enu_i.x += POS_BFP_OF_REAL(amount_to_move_ned.y);
    waypoints[soar_wp_id].enu_i.y += POS_BFP_OF_REAL(amount_to_move_ned.x);
    waypoints[soar_wp_id].enu_i.z += POS_BFP_OF_REAL(-1.0*amount_to_move_ned.z);

// bound wp maybe??

    wp_pos_x = waypoints[soar_wp_id].enu_i.y;
    wp_pos_y = waypoints[soar_wp_id].enu_i.x;
    wp_pos_z = waypoints[soar_wp_id].enu_i.z;

    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &soar_wp_id,
                               &(waypoints[soar_wp_id].enu_i.x),
                               &(waypoints[soar_wp_id].enu_i.y),
                               &(waypoints[soar_wp_id].enu_i.z));
}
void write_map_position_cost_info(struct FloatVect3 soaring_position, float corres_sum_cost){
    // write soaring_position & cost
    if(soar_map_idx < 400) {        // just to prevent from overflow
        soaring_position_map[soar_map_idx].pos = soaring_position;
        soaring_position_map[soar_map_idx].cost = corres_sum_cost;
        soar_map_idx ++;
    }
}

void guidance_indi_hybrid_soaring_start(void) {
    soaring_move_wp_running = true;
}
void guidance_indi_hybrid_soaring_stop(void) {
    soaring_move_wp_running = false;
}
// reset soaring wp to stdby
void guidance_indi_hybrid_soaring_reset(void) {
    soaring_move_wp_running = false;

    uint8_t stdby_wp_id = 2;

    wp_pos_x = waypoints[stdby_wp_id].enu_i.y;
    wp_pos_y = waypoints[stdby_wp_id].enu_i.x;
    wp_pos_z = waypoints[stdby_wp_id].enu_i.z;

    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &soar_wp_id,
                               &(waypoints[stdby_wp_id].enu_i.x),
                               &(waypoints[stdby_wp_id].enu_i.y),
                               &(waypoints[stdby_wp_id].enu_i.z));
}

/**
 * @param heading_sp the desired heading [rad]
 *
 * main indi guidance function
 */
void guidance_indi_soaring_run(float *heading_sp) {
    struct NedCoor_f *groundspeed = stateGetSpeedNed_f();

  /*Obtain eulers with zxy rotation order*/
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  // set heading towards the wind tunnel
  float psi = eulers_zxy.psi;
  struct FloatVect2 heading_pos_diff;

  VECT2_DIFF(heading_pos_diff, heading_target, *stateGetPositionNed_f());
//    float heading_f = atan2f(heading_pos_diff.y, heading_pos_diff.x);

//    if (VECT2_NORM2(heading_pos_diff) > 0.25) {
//        float heading_f = atan2f(heading_pos_diff.x, heading_pos_diff.y);
  float soaring_heading_sp = atan2f(heading_pos_diff.y, heading_pos_diff.x);
//    } else {
//        heading_sp = eulers_zxy.psi;
//    }
//    printf("%f %f %f %f %f\n", heading_target.x, heading_target.y, heading_pos_diff.x, heading_pos_diff.y ,soaring_heading_sp);

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_soaring_propagate_filters();

  //Linear controller to find the acceleration setpoint from position and velocity
    pos_x_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x) - stateGetPositionNed_f()->x;
    pos_y_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y) - stateGetPositionNed_f()->y;
    pos_z_err = POS_FLOAT_OF_BFP(guidance_v_z_ref - stateGetPositionNed_i()->z);

  if(speed_sp_from_position){
      // position ctrl
      speed_sp.x = pos_x_err * gih_params.pos_gain;
      speed_sp.y = pos_y_err * gih_params.pos_gain;
      speed_sp.z = pos_z_err * gih_params.pos_gainz;
  } else {
      if (!speed_sp_from_gradient) {
          speed_sp.x = soaring_spd_sp.x;
          speed_sp.y = soaring_spd_sp.y;
          speed_sp.z = soaring_spd_sp.z;
      } else {
          speed_sp.x = guidance_grad_gain_rec_x/guidance_wind_gradient.x * guidance_grad_gain_x;
          speed_sp.z = guidance_grad_gain_rec_z/guidance_wind_gradient.z * guidance_grad_gain_z;
          speed_sp.y = 0; // FIXME: calc from nav
      }
  }

  // y position control to keep it in the wind section
    if(y_position_ctrl){
        speed_sp.y = pos_y_err * gih_params.pos_gain;
    }

    //for rc control horizontal, rotate from body axes to NED
//  float psi = eulers_zxy.psi;
  /*NAV mode*/
//  float speed_sp_b_x = cosf(psi) * speed_sp.x + sinf(psi) * speed_sp.y;
//  float speed_sp_b_y =-sinf(psi) * speed_sp.x + cosf(psi) * speed_sp.y;

  float airspeed = stateGetAirspeed_f();

  struct FloatVect2 airspeed_v = {cos(psi)*airspeed, sin(psi)*airspeed};
  struct FloatVect2 windspeed;
  VECT2_DIFF(windspeed, *groundspeed, airspeed_v);

  VECT2_DIFF(desired_airspeed, speed_sp, windspeed); // Use 2d part of speed_sp
  float norm_des_as = FLOAT_VECT2_NORM(desired_airspeed);

    sp_accel.x = (speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;
    sp_accel.y = (speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain;
    sp_accel.z = (speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;

    // Bound the acceleration setpoint
  float accelbound = 3.0 + airspeed/guidance_indi_max_airspeed*5.0;
  vect_bound_in_2d(&sp_accel, accelbound);
  /*BoundAbs(sp_accel.x, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
  /*BoundAbs(sp_accel.y, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
  BoundAbs(sp_accel.z, 3.0);

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

  //Calculate matrix of partial derivatives
  guidance_indi_calcg_wing(&Ga);
  // copy G mat i know it's ugly TODO
  for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
          g_arr[i][j] = MAT33_ELMT((Ga),i,j);
      }
      Bwls_g[i] = g_arr[i];
  }
  //Invert this matrix
  MAT33_INV(Ga_inv, Ga);

  struct FloatVect3 accel_filt;
  accel_filt.x = filt_accel_ned[0].o[0];
  accel_filt.y = filt_accel_ned[1].o[0];
  accel_filt.z = filt_accel_ned[2].o[0];

  struct FloatVect3 a_diff;
  a_diff.x = sp_accel.x - accel_filt.x;
  a_diff.y = sp_accel.y - accel_filt.y;
  a_diff.z = sp_accel.z - accel_filt.z;

  //Bound the acceleration error so that the linearization still holds
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);


  // TODO: WLS allocation
  // u, v, umin, umax, B, u_guess(initial), W_init,
  // Wv, Wu, up(preferred control vector), gamma_sq, imax(max iter)
  //
  // Calculates min & max increments
  // min&max phi, theta, thrust
  gd_du_min[0] = -guidance_indi_max_bank*DEG2RAD - roll_filt.o[0];
  gd_du_max[0] = guidance_indi_max_bank*DEG2RAD - roll_filt.o[0];
  gd_du_pref[0] = 0.0 - roll_filt.o[0];

  gd_du_min[1] = GUIDANCE_INDI_MIN_PITCH*DEG2RAD - pitch_filt.o[0];
  gd_du_max[1] = GUIDANCE_INDI_MAX_PITCH*DEG2RAD - pitch_filt.o[0];
  gd_du_pref[1] = 0.0 - pitch_filt.o[0];

  gd_du_min[2] = 0.0 - thrust_filt.o[0];
  gd_du_max[2] = guidance_soaring_max_throttle - thrust_filt.o[0];
  gd_du_pref[2] = GUIDANCE_INDI_PREF_THROTTLE - thrust_filt.o[0]; // TODO: preferred value

    // Control objective TODO: change name
    linear_acc_diff[0] = a_diff.x;
    linear_acc_diff[1] = a_diff.y;
    linear_acc_diff[2] = a_diff.z;

    gd_num_iter =
            wls_alloc_gd(indi_d_euler, linear_acc_diff, gd_du_min, gd_du_max,
                               Bwls_g, 0, 0, gd_Wv, gd_Wu, gd_du_pref, gd_gamma_sq, 10);

  //Calculate roll,pitch and thrust command
//  MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);
//  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, euler_cmd.z);
//    float_vect_sum(indi_euler_cmd, euler_state_filt_vect, indi_d_euler, 3);
//printf("%d, %f, %f, %f, %f, %f\n", gd_num_iter, indi_d_euler[0], indi_d_euler[1], indi_d_euler[2], roll_filt.o[0], pitch_filt.o[0]);

// keep the original vars for telemetry// x:roll, y:pitch, z:throttle
    euler_cmd.x = indi_d_euler[0];
    euler_cmd.y = indi_d_euler[1];
    euler_cmd.z = indi_d_euler[2];

    guidance_euler_cmd.phi = roll_filt.o[0] + indi_d_euler[0];
    guidance_euler_cmd.theta = pitch_filt.o[0] + indi_d_euler[1];
    AbiSendMsgTHRUST(THRUST_INCREMENT_ID, indi_d_euler[2]);

    //Bound euler angles to prevent flipping
    Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
    Bound(guidance_euler_cmd.theta, -RadOfDeg(120.0), RadOfDeg(25.0));

//    printf("%f %f %f\n", guidance_wind_gradient.x, guidance_wind_gradient.y, guidance_wind_gradient.z);

  guidance_euler_cmd.psi = soaring_heading_sp;

  // Set the quaternion setpoint from eulers_zxy
  struct FloatQuat sp_quat;
  float_quat_of_eulers_zxy(&sp_quat, &guidance_euler_cmd);
  float_quat_normalize(&sp_quat);
  QUAT_BFP_OF_REAL(stab_att_sp_quat,sp_quat);

    if (soaring_explore_positions && soaring_move_wp_running) {
        // Move WP if necessary
        move_wp_wait_count++;
        // using incremental average
        move_wp_sum_cost = move_wp_sum_cost +
                           (((gd_k_thr * thrust_filt.o[0]
                              + gd_k_spd_x * fabs(groundspeed->x)     // ABS val?
                              + gd_k_spd_z * fabs(groundspeed->z)
                              + gd_k_pitch * fabs(euler_cmd.y)
                              )-move_wp_sum_cost)/move_wp_wait_count);

        if (move_wp_wait_count > soaring_move_wp_wait_sec*500) {
            move_wp_wait_count = 0;

//            write_map_position_cost_info(wp_soaring_pos, move_wp_sum_cost /
//                                                         soaring_move_wp_wait_sec);   // save the position and corresponding cost val

            if (move_wp_sum_cost > soaring_move_wp_cost_threshold) {
                guidance_indi_soaring_move_wp(move_wp_sum_cost);    // explore or go back to the original position
            } else {
                prev_wp_sum_cost = -1;
            }
            move_wp_sum_cost = 0;   // reset cost
        }
    }
}

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 * Called as a periodic function with PERIODIC_FREQ
 */
void guidance_indi_soaring_propagate_filters(void) {
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, eulers_zxy.phi);
  update_butterworth_2_low_pass(&pitch_filt, eulers_zxy.theta);
  update_butterworth_2_low_pass(&thrust_filt, stabilization_cmd[COMMAND_THRUST]);

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
 * @brief Get the derivative of lift w.r.t. pitch.
 *
 * @param airspeed The airspeed says most about the flight condition
 *
 * @return The derivative of lift w.r.t. pitch
 */
float guidance_indi_get_liftd(float airspeed, float theta) {
  float liftd = 0.0;
  if(airspeed < 12) {
    float pitch_interp = DegOfRad(theta);
    Bound(pitch_interp, -80.0, -40.0);
    float ratio = (pitch_interp + 40.0)/(-40.);
    liftd = -24.0*ratio*lift_pitch_eff/0.12;
  } else {
    liftd = -(airspeed - 8.5)*lift_pitch_eff/M_PI*180.0;
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
