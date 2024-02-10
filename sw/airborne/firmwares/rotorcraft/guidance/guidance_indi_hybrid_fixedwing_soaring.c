/*
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
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid_fixedwing_soaring.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Modified by Sunyou for fixed wing vehicles (Never Landing Drone project)
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid_fixedwing_soaring.h"
#include "state.h"
#include "generated/modules.h"

#include <time.h>
#include "stdio.h"

#include "generated/airframe.h"
#include "modules/ins/ins_int.h"
#include "modules/radio_control/radio_control.h"
#include "modules/imu/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
//#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
//#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"
//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
//#include "wls/wls_alloc_gd.h"
#include "generated/flight_plan.h" // for waypoint reference pointers
//#include "subsystems/navigation/common_nav.h"
#include "modules/nav/waypoints.h"
#include "firmwares/rotorcraft/navigation.h"


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
#define GUIDANCE_INDI_MIN_PITCH -120
#define GUIDANCE_INDI_MAX_PITCH -60
#endif


//struct guidance_indi_hybrid_params gih_params = {
//  .pos_gain = GUIDANCE_INDI_POS_GAIN,
//  .pos_gainz = GUIDANCE_INDI_POS_GAINZ,
//
//  .speed_gain = GUIDANCE_INDI_SPEED_GAIN,
//  .speed_gainz = GUIDANCE_INDI_SPEED_GAINZ,
//
//  .heading_bank_gain = GUIDANCE_INDI_HEADING_BANK_GAIN,
////        .liftd_asq = GUIDANCE_INDI_LIFTD_ASQ, // coefficient of airspeed squared
////        .liftd_p80 = GUIDANCE_INDI_LIFTD_P80,
////        .liftd_p50 = GUIDANCE_INDI_LIFTD_P50,
//};

//#ifndef GUIDANCE_INDI_MAX_AIRSPEED
//#error "You must have an airspeed sensor to use this guidance"
//#endif
//float guidance_indi_max_airspeed = GUIDANCE_INDI_MAX_AIRSPEED;

//#ifndef GUIDANCE_INDI_FILTER_CUTOFF
//#ifdef STABILIZATION_INDI_FILT_CUTOFF
//#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
//#else
//#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
//#endif
//#endif

//float inv_eff[4];


// Max bank angle in radians
//float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

/** state eulers in zxy order */
//struct FloatEulers eulers_zxy;

//float thrust_act = 0;
//Butterworth2LowPass filt_accel_ned[3];
//Butterworth2LowPass roll_filt;
//Butterworth2LowPass pitch_filt;
//Butterworth2LowPass thrust_filt;
//Butterworth2LowPass accely_filt;

//struct FloatVect2 desired_airspeed;

//struct FloatMat33 Ga;
//struct FloatMat33 Ga_inv;
//struct FloatVect3 euler_cmd;

//float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;

//struct FloatEulers guidance_euler_cmd;
//float thrust_in;

#ifndef GUIDANCE_INDI_POS_CTRL
#define GUIDANCE_INDI_POS_CTRL TRUE
#endif
#ifndef GUIDANCE_INDI_MAX_THROTTLE
#define GUIDANCE_INDI_MAX_THROTTLE 9600
#endif
#ifndef GUIDANCE_INDI_PREF_THROTTLE
#define GUIDANCE_INDI_PREF_THROTTLE 0.0
#endif
#ifndef GUIDANCE_INDI_Y_POSITION_CTRL
#define GUIDANCE_INDI_Y_POSITION_CTRL TRUE
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
#ifndef GUIDANCE_INDI_SOARING_HEADING_WP_ID
#define GUIDANCE_INDI_SOARING_HEADING_WP_ID 4
#endif
#ifndef GUIDANCE_INDI_SOARING_STDBY_WP_ID
#define GUIDANCE_INDI_SOARING_STDBY_WP_ID 2
#endif
// Moving WP Steps
#ifndef GUIDANCE_INDI_SOARING_STEP_K_BIG
#define GUIDANCE_INDI_SOARING_STEP_K_BIG 5
#endif
#ifndef GUIDANCE_INDI_SOARING_STEP_K_MED
#define GUIDANCE_INDI_SOARING_STEP_K_MED 3
#endif
#ifndef GUIDANCE_INDI_SOARING_STEP_K_SMALL
#define GUIDANCE_INDI_SOARING_STEP_K_SMALL 1.5
#endif
#ifndef GUIDANCE_INDI_SOARING_STEP_BIG
#define GUIDANCE_INDI_SOARING_STEP_BIG 0.5
#endif
#ifndef GUIDANCE_INDI_SOARING_STEP_MED
#define GUIDANCE_INDI_SOARING_STEP_MED 0.3
#endif
#ifndef GUIDANCE_INDI_SOARING_STEP_SMALL
#define GUIDANCE_INDI_SOARING_STEP_SMALL 0.1
#endif
#ifndef GUIDANCE_INDI_SOARING_STEP_FINE
#define GUIDANCE_INDI_SOARING_STEP_FINE 0.05
#endif
#ifndef GUIDANCE_INDI_SOARING_USE_FIXED_STEP_SIZE
#define GUIDANCE_INDI_SOARING_USE_FIXED_STEP_SIZE FALSE
#endif
#ifndef GUIDANCE_INDI_SOARING_FIXED_STEP_SIZE
#define GUIDANCE_INDI_SOARING_FIXED_STEP_SIZE 0.2
#endif

// use variable weights for WLS (x, z)
#ifndef GUIDANCE_INDI_SOARING_USE_VARIABLE_WEIGHTS
#define GUIDANCE_INDI_SOARING_USE_VARIABLE_WEIGHTS FALSE
#endif
#ifndef GUIDANCE_INDI_SOARING_LOGISTIC_FN_FACTOR
#define GUIDANCE_INDI_SOARING_LOGISTIC_FN_FACTOR 3
#endif

#ifndef GUIDANCE_INDI_SOARING_WP_STAY_DURATION
#define GUIDANCE_INDI_SOARING_WP_STAY_DURATION 5
#endif
#ifndef GUIDANCE_INDI_SOARING_WP_ARRIVAL_DIST
#define GUIDANCE_INDI_SOARING_WP_ARRIVAL_DIST 2.0
#endif

#ifndef GUIDANCE_INDI_SOARING_MIN_ALT
#define GUIDANCE_INDI_SOARING_MIN_ALT 5.0
#endif

// bound min, max
#ifndef GUIDANCE_INDI_MAX_ACC_SP
#define GUIDANCE_INDI_MAX_ACC_SP 3.0
#endif
#ifndef GUIDANCE_INDI_MIN_ACC_SP
#define GUIDANCE_INDI_MIN_ACC_SP -3.0
#endif

#ifndef GUIDANCE_INDI_SOARING_USE_FIXED_HEADING_WP
#define GUIDANCE_INDI_SOARING_USE_FIXED_HEADING_WP FALSE
#endif

#ifndef GUIDANCE_INDI_SOARING_HEADING_GAIN
#define GUIDANCE_INDI_SOARING_HEADING_GAIN 0.001
#endif

#ifndef GUIDANCE_INDI_SOARING_MAX_EXPLORATION_STEPS
#define GUIDANCE_INDI_SOARING_MAX_EXPLORATION_STEPS 1000
#endif

// 90 deg pitch offset
//#ifndef GUIDANCE_INDI_SOARING_USE_90_OFFSET
//#define GUIDANCE_INDI_SOARING_USE_90_OFFSET FALSE
//#endif
#ifdef GUIDANCE_INDI_SOARING_USE_90_PITCH_OFFSET
#warning "You are using 90deg pitch offset setting! (0 deg pitch == nose to the sky)"
#endif

#define SOARING_RESET_STDBY_TIMEOUT 3     // seconds

// 2m/0.1 = 20 data points for one axis
#define MAP_MAX_NUM_POINTS 400

bool speed_sp_from_position = GUIDANCE_INDI_POS_CTRL;
bool y_position_ctrl = GUIDANCE_INDI_Y_POSITION_CTRL;
float guidance_soaring_max_throttle = GUIDANCE_INDI_MAX_THROTTLE;

float weight_logistic_fn_factor = GUIDANCE_INDI_SOARING_LOGISTIC_FN_FACTOR;
float stay_wp_duration = GUIDANCE_INDI_SOARING_WP_STAY_DURATION;
float arrival_check_dist_3d = GUIDANCE_INDI_SOARING_WP_ARRIVAL_DIST;

// WLS alloc    TODO: Weights and gamma_sq to tune
float linear_acc_diff[3];
float indi_d_euler[3];
float indi_euler_cmd[3];
float euler_state_filt_vect[3];
float gd_gamma_sq = 1000.;
float gd_Wv[3] = {1.0, 1.0, 6.5};         // x y z
float gd_Wu[3] = {1.0, 100.0, 1.0};         // roll pitch thrust
float gd_du_min[3];
float gd_du_max[3];
float gd_du_pref[3];
float *Bwls_g[3];
float g_arr[3][3];
int gd_num_iter = 0;

struct FloatVect3 pos_err = {0., 0., 0.};
struct FloatVect3 accel_sp = {0.f, 0.f, 0.f};
struct FloatVect3 speed_sp = {0.f, 0.f, 0.f};
struct FloatVect3 guidance_wind_gradient = {0.0, 0.0, 0.0};

float lift_pitch_eff = GUIDANCE_INDI_PITCH_LIFT_EFF;

float gd_k_thr = GUIDANCE_INDI_SOARING_WP_W_THROTTLE;
float gd_k_spd_x = GUIDANCE_INDI_SOARING_WP_W_SPD_X;
float gd_k_spd_z = GUIDANCE_INDI_SOARING_WP_W_SPD_Z;
float gd_k_pitch = GUIDANCE_INDI_SOARING_WP_W_PITCH;
float soaring_move_wp_wait_sec = GUIDANCE_INDI_SOARING_WP_WAIT_SEC;
float soaring_move_wp_cost_threshold = GUIDANCE_INDI_SOARING_WP_COST_THRES;
bool soaring_explore_positions = GUIDANCE_INDI_SOARING_MOVE_WP;

struct SoaringPositionMap soaring_position_map[MAP_MAX_NUM_POINTS];
struct FloatVect3 preset_move_body[8] = {{1.0, 0., 0.}, {-1.0, 0., -1.0}, {0., 0., -1.0}, {-1.0, 0., 0.}, {1.0, 0., -1.0}, {1.0, 0., 1.0}, {0., 0., 1.0}, {-1.0, 0., 1.0}};
struct FloatVect3 amount_to_move_ned = {0., 0., 0.};
struct FloatVect3 amount_to_move_body = {0., 0., 0.};
int soar_map_idx = 0;
float prev_wp_sum_cost = -1;
int16_t prev_move_idx = 0;
int move_wp_wait_count = 0;
float move_wp_sum_cost = 0.;
uint16_t move_wp_entry_time = 0;
uint16_t move_wp_wait_time = 0;
struct FloatVect3 wp_soaring_pos;
uint8_t soar_wp_id = GUIDANCE_INDI_SOARING_WP_ID;
uint8_t heading_wp_id = GUIDANCE_INDI_SOARING_HEADING_WP_ID;
uint8_t stdby_wp_id = GUIDANCE_INDI_SOARING_STDBY_WP_ID;
bool soaring_mode_running = false;
bool use_variable_weights = GUIDANCE_INDI_SOARING_USE_VARIABLE_WEIGHTS;
bool soaring_use_fixed_step_size = GUIDANCE_INDI_SOARING_USE_FIXED_STEP_SIZE;
float soaring_fixed_step_size = GUIDANCE_INDI_SOARING_FIXED_STEP_SIZE;

float soaring_step_k_big = GUIDANCE_INDI_SOARING_STEP_K_BIG;
float soaring_step_k_mid = GUIDANCE_INDI_SOARING_STEP_K_MED;
float soaring_step_k_small = GUIDANCE_INDI_SOARING_STEP_K_SMALL;

float soaring_step_size_big = GUIDANCE_INDI_SOARING_STEP_BIG;
float soaring_step_size_mid = GUIDANCE_INDI_SOARING_STEP_MED;
float soaring_step_size_small = GUIDANCE_INDI_SOARING_STEP_SMALL;
float soaring_step_size_fine = GUIDANCE_INDI_SOARING_STEP_FINE;

float max_acc_sp = GUIDANCE_INDI_MAX_ACC_SP;
float min_acc_sp = GUIDANCE_INDI_MIN_ACC_SP;
float soaring_min_alt = GUIDANCE_INDI_SOARING_MIN_ALT;
float soaring_heading_sp = 0;       // in rad
float soaring_heading_gain = GUIDANCE_INDI_SOARING_HEADING_GAIN;
bool use_fixed_heading_wp = GUIDANCE_INDI_SOARING_USE_FIXED_HEADING_WP;

uint16_t soaring_max_exploration_steps = GUIDANCE_INDI_SOARING_MAX_EXPLORATION_STEPS;
uint16_t soaring_search_cnt_steps = 0;
float soaring_min_cost = INFINITY;

float soaring_wp_move_forward = 0;
float soaring_wp_move_right = 0;
float soaring_wp_move_up = 0;
bool soaring_manual_search = false;

int32_t wp_pos_x;
int32_t wp_pos_y;
int32_t wp_pos_z;

int32_t min_cost_wp_e;
int32_t min_cost_wp_n;
int32_t min_cost_wp_u;

time_t rand_seed;
uint16_t stdby_entry_time = 0;
uint16_t reset_stdby_timeout = SOARING_RESET_STDBY_TIMEOUT;

void guidance_indi_soaring_move_wp(float cost_avg_val);
void write_map_position_cost_info(struct FloatVect3 soaring_position, float corres_sum_cost);
void guidance_indi_hybrid_soaring_stop(void);
void guidance_indi_hybrid_soaring_start(void);
void guidance_indi_hybrid_soaring_reset(void);

void guidance_indi_soaring_reset_wp(uint8_t wp_id);
void guidance_indi_soaring_reset_stby_wp(void);
void guidance_indi_soaring_reset_soaring_wp(void);

#define DEG2RAD 0.017

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_guidance_indi_hybrid(struct transport_tx *trans, struct link_device *dev)
{
    // publish only additional information (+indi_hybrid)
    // roll/theta/psi, thrust, pos err + spd sp
  pprz_msg_send_GUIDANCE_INDI_HYBRID_SOARING(trans, dev, AC_ID,
                              &pos_err.x,
                              &pos_err.y,
                              &pos_err.z,
                              &speed_sp.x,
                              &speed_sp.y,
                              &speed_sp.z,
                              &accel_sp.x,
                              &accel_sp.y,
                              &accel_sp.z,
                              &soaring_heading_sp
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
                               &amount_to_move_ned.z,
                               &gd_k_thr,
                               &gd_k_spd_x,
                               &gd_k_spd_z,
                               &gd_k_pitch,
                               &soaring_move_wp_cost_threshold,
                               &arrival_check_dist_3d,
                               &stay_wp_duration,
                               &soaring_move_wp_wait_sec,
                               &soaring_search_cnt_steps,
                               &soaring_max_exploration_steps,
                               &soaring_min_cost,
                               &min_cost_wp_n,
                               &min_cost_wp_e,
                               &min_cost_wp_u
                              );
}
#endif

/**
 * @brief Init function
 */
void guidance_indi_soaring_init(void)
{
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
    soaring_heading_sp = nav.heading;
    guidance_indi_soaring_reset_soaring_wp();
}

void guidance_indi_soaring_move_wp(float cost_avg_val){
    int idx_to_move = prev_move_idx;

    if (prev_wp_sum_cost < 0) {
        // pick a random neighbour
//        idx_to_move = abs((int)(filt_accel_ned[2].o[0]*100))%8;
        idx_to_move = abs((int)(stateGetAccelNed_f()->z*100))%8;
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
        idx_to_move = 7-prev_move_idx;
        prev_wp_sum_cost = -1.0;
        prev_move_idx = -1;
    }

    float step_size = soaring_fixed_step_size;

    // Set a step size
    if (soaring_use_fixed_step_size) {
        step_size = soaring_fixed_step_size;
    } else if (cost_avg_val > (soaring_move_wp_cost_threshold*soaring_step_k_big)) {
        step_size = soaring_step_size_big;
    } else if (cost_avg_val > (soaring_move_wp_cost_threshold*soaring_step_k_mid)) {
        step_size = soaring_step_size_mid;
    } else if (cost_avg_val > (soaring_move_wp_cost_threshold*soaring_step_k_small)) {
        step_size = soaring_step_size_small;
    } else {
        step_size = soaring_step_size_fine;
    }

    VECT3_SMUL(amount_to_move_body, preset_move_body[idx_to_move], step_size);

    if ((POS_FLOAT_OF_BFP(waypoints[soar_wp_id].enu_i.z) - amount_to_move_body.z) < soaring_min_alt) {
        // pick a random neighbour that doesn't go down; default is 5m
        idx_to_move = abs((int)(stateGetAccelNed_f()->z*100))%5;
//        idx_to_move = abs((int)(filt_accel_ned[2].o[0]*100))%5;
        prev_move_idx = idx_to_move;
        prev_wp_sum_cost = cost_avg_val;
        VECT3_SMUL(amount_to_move_body, preset_move_body[idx_to_move], step_size);
    }

    struct FloatEulers eulers_zxy;
    float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
    float psi = eulers_zxy.psi;

    // rotate wp body to ned
    amount_to_move_ned.x = cosf(psi)*amount_to_move_body.x - sinf(psi)*amount_to_move_body.y;
    amount_to_move_ned.y = sinf(psi)*amount_to_move_body.x + cosf(psi)*amount_to_move_body.y;
    amount_to_move_ned.z = amount_to_move_body.z;   // z doesn't change

    // move waypoints
    waypoints[soar_wp_id].enu_i.x += POS_BFP_OF_REAL(amount_to_move_ned.y);
    waypoints[soar_wp_id].enu_i.y += POS_BFP_OF_REAL(amount_to_move_ned.x);
    waypoints[soar_wp_id].enu_i.z += POS_BFP_OF_REAL(-1.0*amount_to_move_ned.z);

// bound wp maybe??

//    wp_pos_x = waypoints[soar_wp_id].enu_i.y;
//    wp_pos_y = waypoints[soar_wp_id].enu_i.x;
//    wp_pos_z = waypoints[soar_wp_id].enu_i.z;
// FIXME: flight_altitude or nav_altitude?
    flight_altitude = waypoints[soar_wp_id].enu_i.z; // FIXME: manually set z ref

    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &soar_wp_id,
                               &(waypoints[soar_wp_id].enu_i.x),
                               &(waypoints[soar_wp_id].enu_i.y),
                               &(waypoints[soar_wp_id].enu_i.z));

    soaring_search_cnt_steps ++;
    if (soaring_min_cost > cost_avg_val) {
        soaring_min_cost = cost_avg_val;
        min_cost_wp_e = waypoints[soar_wp_id].enu_i.x;
        min_cost_wp_n = waypoints[soar_wp_id].enu_i.y;
        min_cost_wp_u = waypoints[soar_wp_id].enu_i.z;
    }
    // update threshold
    if (soaring_search_cnt_steps > soaring_max_exploration_steps) {
        if (soaring_move_wp_cost_threshold < soaring_min_cost) {
            soaring_move_wp_cost_threshold = soaring_min_cost;
        }
    }
}

// reset wp to current position
void guidance_indi_soaring_reset_wp(uint8_t wp_id) {
    waypoints[wp_id].enu_i.x = POS_BFP_OF_REAL(stateGetPositionNed_f()->y);
    waypoints[wp_id].enu_i.y = POS_BFP_OF_REAL(stateGetPositionNed_f()->x);
    waypoints[wp_id].enu_i.z = -1.0*stateGetPositionNed_i()->z;

    // x<->y because enu<->ned
//    wp_pos_x = waypoints[wp_id].enu_i.y;
//    wp_pos_y = waypoints[wp_id].enu_i.x;
//    wp_pos_z = waypoints[wp_id].enu_i.z;
    flight_altitude = waypoints[wp_id].enu_i.z;

    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &(waypoints[wp_id].enu_i.x),
                               &(waypoints[wp_id].enu_i.y),
                               &(waypoints[wp_id].enu_i.z));
}

// reset soaring wp to current position
void guidance_indi_soaring_reset_soaring_wp(void) {
//    guidance_indi_soaring_reset_wp(&soar_wp_id);
    waypoint_set_here(soar_wp_id);
}

// reset stdby WP to current position
void guidance_indi_soaring_reset_stby_wp(void) {
//    guidance_indi_soaring_reset_wp(&stdby_wp_id);
    waypoint_set_here(stdby_wp_id);

    // save entry time
    stdby_entry_time = autopilot.flight_time;
}

void guidance_indi_hybrid_soaring_start(void) {
    if (soaring_mode_running) {
        guidance_indi_soaring_reset_soaring_wp();   // reset wp to current position
    }
    soaring_mode_running = true;
    soaring_manual_search = false;

    move_wp_entry_time = autopilot.flight_time;
}
void guidance_indi_hybrid_soaring_stop(void) {
    soaring_mode_running = false;
    soaring_search_cnt_steps = 0;
    soaring_manual_search = false;
}
// reset soaring wp to stdby
void guidance_indi_hybrid_soaring_reset(void) {
//    soaring_mode_running = false;
    flight_altitude = waypoints[stdby_wp_id].enu_i.z;

    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &soar_wp_id,
                               &(waypoints[stdby_wp_id].enu_i.x),
                               &(waypoints[stdby_wp_id].enu_i.y),
                               &(waypoints[stdby_wp_id].enu_i.z));

    prev_wp_sum_cost = -1;
    move_wp_sum_cost = 0;
    move_wp_wait_count = 0;
    soaring_search_cnt_steps = 0;
}

/*
 * move waypoints / soaring search
 * periodic function
 * TODO: verify variable types / function calls
 */
void run_soaring_search(void){
    struct NedCoor_f *groundspeed = stateGetSpeedNed_f();

    // if in soaring mode
    if (soaring_explore_positions && soaring_mode_running) {
        // count for timeout
        move_wp_wait_count++;

        // calc cost fn using incremental average
        move_wp_sum_cost = move_wp_sum_cost +
                           (((gd_k_thr * (int32_t)(autopilot.throttle)
                              + gd_k_spd_x * fabs(groundspeed->x)     // ABS val?
                              + gd_k_spd_z * fabs(groundspeed->z)
                              + gd_k_pitch * fabs(stateGetBodyRates_f()->q)
                              )-move_wp_sum_cost)/move_wp_wait_count);

        bool wp_arrived = nav_check_wp_time_3d(&waypoints[soar_wp_id].enu_f, stay_wp_duration, arrival_check_dist_3d);
        move_wp_wait_time = autopilot.flight_time - move_wp_entry_time;

        // if wp not arrived && timeout, reset wp to the current pos
        // if wp arrived, move wp depending on the cost fn val
        if (wp_arrived) {
//            write_map_position_cost_info(wp_soaring_pos, move_wp_sum_cost /
//                                                         soaring_move_wp_wait_sec);   // save the position and corresponding cost val
            // move wp
            if (move_wp_sum_cost > soaring_move_wp_cost_threshold) {
                guidance_indi_soaring_move_wp(move_wp_sum_cost);    // explore or go back to the original position
            } else {
                // stay
                prev_wp_sum_cost = -2;
            }
            // reset count & cost
            move_wp_entry_time = autopilot.flight_time;
            move_wp_sum_cost = 0;
            move_wp_wait_count = 0;
        } else if (move_wp_wait_time > soaring_move_wp_wait_sec) {
            // not arrived && timeout
            // reset wp to current pos (assuming the wp cannot be reached)
            guidance_indi_soaring_reset_soaring_wp();
            prev_wp_sum_cost = -1;
            move_wp_entry_time = autopilot.flight_time;
            move_wp_sum_cost = 0;
            move_wp_wait_count = 0;
        }
    }

    // in soaring mode && manual search
    if (soaring_mode_running && soaring_manual_search && (!soaring_explore_positions)) {
        if (fabs(soaring_wp_move_forward) > 0.05 || fabs(soaring_wp_move_right) > 0.05 || fabs(soaring_wp_move_up) > 0.05) {
            struct FloatEulers eulers_zxy;
            float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
            float psi = eulers_zxy.psi;

            amount_to_move_body.x = soaring_wp_move_forward;
            amount_to_move_body.y = soaring_wp_move_right;
            amount_to_move_body.z = -soaring_wp_move_up;

            // rotate wp body to ned
            amount_to_move_ned.x = cosf(psi)*amount_to_move_body.x - sinf(psi)*amount_to_move_body.y;
            amount_to_move_ned.y = sinf(psi)*amount_to_move_body.x + cosf(psi)*amount_to_move_body.y;
            amount_to_move_ned.z = amount_to_move_body.z;   // z doesn't change

            // move waypoints
            waypoints[soar_wp_id].enu_i.x += POS_BFP_OF_REAL(amount_to_move_ned.y);
            waypoints[soar_wp_id].enu_i.y += POS_BFP_OF_REAL(amount_to_move_ned.x);
            waypoints[soar_wp_id].enu_i.z += POS_BFP_OF_REAL(-1.0*amount_to_move_ned.z);

            flight_altitude = waypoints[soar_wp_id].enu_i.z; // FIXME: manually set z ref

            DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &soar_wp_id,
                                       &(waypoints[soar_wp_id].enu_i.x),
                                       &(waypoints[soar_wp_id].enu_i.y),
                                       &(waypoints[soar_wp_id].enu_i.z));

            soaring_wp_move_forward = 0;
            soaring_wp_move_right = 0;
            soaring_wp_move_up = 0;
        }
    }

    // to reset standby waypoint after turning on AUTO2
    // quick fix for position overshooting at STDBY
    if (stdby_entry_time > 0 && (stdby_entry_time+reset_stdby_timeout < autopilot.flight_time)) {
        waypoint_set_here(stdby_wp_id);
        guidance_indi_soaring_reset_soaring_wp();
        stdby_entry_time = 0;
    }
}

/*
 * returns heading setpoint (gh->sp.heading)
 * float heading            in rad????
 */

float compute_soaring_heading_sp(void) {
    struct FloatEulers eulers_zxy;
    float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
    float psi = eulers_zxy.psi;

    // Set heading to a fixed WP
    if (use_fixed_heading_wp) {
        nav_set_heading_towards_waypoint(heading_wp_id);
        soaring_heading_sp = nav.heading;
    } else {
        float heading_ref_to_add = ((soaring_heading_sp - psi) / PERIODIC_FREQUENCY * soaring_heading_gain);
        Bound(heading_ref_to_add, -10.0, 10.0);
        soaring_heading_sp -= heading_ref_to_add;
    }
    return soaring_heading_sp;
}

/*
 * returns acceleration sp (Struct FloatVect3)
 */

struct FloatVect3 compute_soaring_accel_sp(struct HorizontalGuidance *gh, struct VerticalGuidance *gv) {
    pos_err.x = POS_FLOAT_OF_BFP(gh->ref.pos.x) - stateGetPositionNed_f()->x;
    pos_err.y = POS_FLOAT_OF_BFP(gh->ref.pos.y) - stateGetPositionNed_f()->y;
    pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;

    speed_sp.x = pos_err.x * gih_params.pos_gain;
    speed_sp.y = pos_err.y * gih_params.pos_gain;
    speed_sp.z = pos_err.z * gih_params.pos_gainz;
    Bound(speed_sp.z, -nav.climb_vspeed, -nav.descend_vspeed);   // FIXME vspeeds
    // climb >0 descend <0 ???? wtf.. z up?

    if (stab_indi_kill_throttle) {
        speed_sp.z = 1.0f;
    }
    accel_sp.x = (speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;
    accel_sp.y = (speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain;
    accel_sp.z = (speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;

//    FIXME: failsafe
    if (stab_indi_kill_throttle) {
        accel_sp.z = (1.0f - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
//        Bound(accel_sp.z, -1.0, 3.0);
    }

    // FIXME
    float airspeed = stateGetAirspeed_f();
    float accelbound = 3.0f + airspeed/guidance_indi_max_airspeed*5.0f;
    float_vect3_bound_in_2d(&accel_sp, accelbound);
    Bound(accel_sp.z, min_acc_sp, max_acc_sp);

    return accel_sp;
}


/**
 * @param heading_sp the desired heading [rad]
 *
 * main indi guidance function
 */
//void guidance_indi_soaring_run(float *heading_sp) {
//    struct NedCoor_f *groundspeed = stateGetSpeedNed_f();
//
//    // save wp info for logging
//    wp_pos_x = waypoints[soar_wp_id].enu_i.y;
//    wp_pos_y = waypoints[soar_wp_id].enu_i.x;
//    wp_pos_z = waypoints[soar_wp_id].enu_i.z;
//
//  /*Obtain eulers with zxy rotation order*/
//  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
//
//    /*Calculate the transition percentage so that the ctrl_effecitveness scheduling works*/
////    transition_percentage = BFP_OF_REAL((eulers_zxy.theta/RadOfDeg(-75.0))*100,INT32_PERCENTAGE_FRAC);
//    transition_percentage = BFP_OF_REAL((eulers_zxy.theta/TRANSITION_MAX_OFFSET)*100,INT32_PERCENTAGE_FRAC);
//    Bound(transition_percentage,0,BFP_OF_REAL(100.0,INT32_PERCENTAGE_FRAC));
//    const int32_t max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
//
//    transition_theta_offset = INT_MULT_RSHIFT((transition_percentage <<
//(INT32_ANGLE_FRAC - INT32_PERCENTAGE_FRAC)) / 100, max_offset, INT32_ANGLE_FRAC);
//
//
//  //filter accel to get rid of noise and filter attitude to synchronize with accel
//  guidance_indi_soaring_propagate_filters();
//
//  //// From here...
//
//  //Linear controller to find the acceleration setpoint from position and velocity
//  pos_x_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x) - stateGetPositionNed_f()->x;
//  pos_y_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y) - stateGetPositionNed_f()->y;
//  pos_z_err = POS_FLOAT_OF_BFP(guidance_v_z_ref - stateGetPositionNed_i()->z);
//
//  // position ctrl TODO: regulate airspeed?
//  speed_sp.x = pos_x_err * gih_params.pos_gain;
//  speed_sp.y = pos_y_err * gih_params.pos_gain;
//  speed_sp.z = pos_z_err * gih_params.pos_gainz;
//
//    float norm_spd_sp = FLOAT_VECT2_NORM(speed_sp);
//    if (norm_spd_sp > nav_max_speed){
//        vect_scale(&speed_sp, nav_max_speed);
//    }
//    Bound(speed_sp.z, -nav_climb_vspeed, -nav_descend_vspeed);
//
//    // y position control to keep it in the wind section
//    if(!y_position_ctrl){
//        speed_sp.y = 0;
//    }
//
////    failsafe
//    if (stab_indi_kill_throttle) {
//        speed_sp.z = 1.0;
//    }
//
//    //for rc control horizontal, rotate from body axes to NED
//  float psi = eulers_zxy.psi;
//
//  float airspeed = stateGetAirspeed_f();
//
//  struct FloatVect2 airspeed_v = {cos(psi)*airspeed, sin(psi)*airspeed};
//  struct FloatVect2 windspeed;
//  VECT2_DIFF(windspeed, *groundspeed, airspeed_v);
//
//      sp_accel.x = (speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;
//      sp_accel.y = (speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gain;
//      sp_accel.z = (speed_sp.z - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
//
////    FIXME: failsafe
//    if (stab_indi_kill_throttle) {
//        sp_accel.z = (1.0 - stateGetSpeedNed_f()->z) * gih_params.speed_gainz;
////        Bound(sp_accel.z, -1.0, 3.0);
//    }
//
//    // Bound the acceleration setpoint
//  float accelbound = 3.0 + airspeed/guidance_indi_max_airspeed*5.0;
//  vect_bound_in_2d(&acee, accelbound);
//  /*BoundAbs(sp_accel.x, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
//  /*BoundAbs(sp_accel.y, 3.0 + airspeed/guidance_indi_max_airspeed*6.0);*/
////  BoundAbs(sp_accel.z, 3.0);
//  Bound(sp_accel.z, min_acc_sp, max_acc_sp);
//
//#if GUIDANCE_INDI_RC_DEBUG
//#warning "GUIDANCE_INDI_RC_DEBUG lets you control the accelerations via RC, but disables autonomous flight!"
//  //for rc control horizontal, rotate from body axes to NED
//  float psi = eulers_zxy.psi;
//  float rc_x = -(radio_control.values[RADIO_PITCH]/9600.0)*8.0;
//  float rc_y = (radio_control.values[RADIO_ROLL]/9600.0)*8.0;
//  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
//  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;
//
//  //for rc vertical control
//  sp_accel.z = -(radio_control.values[RADIO_THROTTLE]-4500)*8.0/9600.0;
//#endif
//
//  //Calculate matrix of partial derivatives
//  // TODO
//  guidance_indi_calcg_wing(&Ga);
//  // copy G mat i know it's ugly FIXME
//  for(int i=0; i<3; i++){
//      for(int j=0; j<3; j++){
//          g_arr[i][j] = MAT33_ELMT((Ga),i,j);
//      }
//      Bwls_g[i] = g_arr[i];
//  }
//  //Invert this matrix
//  MAT33_INV(Ga_inv, Ga);
//
//  struct FloatVect3 accel_filt;
//  accel_filt.x = filt_accel_ned[0].o[0];
//  accel_filt.y = filt_accel_ned[1].o[0];
//  accel_filt.z = filt_accel_ned[2].o[0];
//
//  struct FloatVect3 a_diff;
//  a_diff.x = sp_accel.x - accel_filt.x;
//  a_diff.y = sp_accel.y - accel_filt.y;
//  a_diff.z = sp_accel.z - accel_filt.z;
//
//  //Bound the acceleration error so that the linearization still holds
//  Bound(a_diff.x, -6.0, 6.0);
//  Bound(a_diff.y, -6.0, 6.0);
//  Bound(a_diff.z, -9.0, 9.0);
//
//
//  // TODO: WLS allocation
//  // u, v, umin, umax, B, u_guess(initial), W_init,
//  // Wv, Wu, up(preferred control vector), gamma_sq, imax(max iter)
//  //
//  // Calculates min & max increments
//  // min&max phi, theta, thrust
//  gd_du_min[0] = -guidance_indi_max_bank*DEG2RAD - roll_filt.o[0];
//  gd_du_max[0] = guidance_indi_max_bank*DEG2RAD - roll_filt.o[0];
//  gd_du_pref[0] = 0.0 - roll_filt.o[0];
//
//  gd_du_min[1] = GUIDANCE_INDI_MIN_PITCH*DEG2RAD - pitch_filt.o[0];
//  gd_du_max[1] = GUIDANCE_INDI_MAX_PITCH*DEG2RAD - pitch_filt.o[0];
//  gd_du_pref[1] = 0.0 - pitch_filt.o[0];
//
//  gd_du_min[2] = 0.0 - thrust_filt.o[0];
//  gd_du_max[2] = guidance_soaring_max_throttle - thrust_filt.o[0];
//  gd_du_pref[2] = GUIDANCE_INDI_PREF_THROTTLE - thrust_filt.o[0]; // TODO: preferred value
//
//    // Control objective TODO: change name
//    linear_acc_diff[0] = a_diff.x;
//    linear_acc_diff[1] = a_diff.y;
//    linear_acc_diff[2] = a_diff.z;
//
////    TODO: revise
//    // assign weights (hor<->ver position ctrl priority)
//    if (use_variable_weights) {
//        float h_ratio = fabs(pos_x_err)/(fabs(pos_z_err)+0.00001);
//        float v_ratio = fabs(pos_z_err)/(fabs(pos_x_err)+0.00001);
//
//        // normalize; 0 <= tanh (logistic fn) <= 1;
//        // ratio 5->logistic fn 0.8320; 10->0.9734
//        float v_tanh = 0.5+0.5*tanh((v_ratio-1)/5);
//        float h_tanh = 0.5+0.5*tanh((h_ratio-1)/5);
//        float w_v_factor = 1 - h_tanh;
//
//        if ((v_tanh>h_tanh) || (fabs(pos_z_err)>3)){
//            w_v_factor = v_tanh;
//        }
//
//        gd_Wv[0] = 0.5 + (1-w_v_factor)*weight_logistic_fn_factor;
//        gd_Wv[2] = 0.5 + w_v_factor*weight_logistic_fn_factor;
//    }
//
//    gd_num_iter =
//            wls_alloc_gd(indi_d_euler, linear_acc_diff, gd_du_min, gd_du_max,
//                               Bwls_g, 0, 0, gd_Wv, gd_Wu, gd_du_pref, gd_gamma_sq, 10);
//
//// keep the original vars for telemetry// x:roll, y:pitch, z:throttle
//    euler_cmd.x = indi_d_euler[0];
//    euler_cmd.y = indi_d_euler[1];
//    euler_cmd.z = indi_d_euler[2];
//
//    guidance_euler_cmd.phi = roll_filt.o[0] + indi_d_euler[0];
//    guidance_euler_cmd.theta = pitch_filt.o[0] + indi_d_euler[1];
////    TODO: filter??
//    AbiSendMsgTHRUST(THRUST_INCREMENT_ID, indi_d_euler[2]);
//
//  //Bound euler angles to prevent flipping
//  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
//  Bound(guidance_euler_cmd.theta, RadOfDeg(GUIDANCE_INDI_MIN_PITCH), RadOfDeg(GUIDANCE_INDI_MAX_PITCH));
//
//
//    // Set the quaternion setpoint from eulers_zxy
//    struct FloatQuat sp_quat;
//    float_quat_of_eulers_zxy(&sp_quat, &guidance_euler_cmd);
//    float_quat_normalize(&sp_quat);
//    QUAT_BFP_OF_REAL(stab_att_sp_quat,sp_quat);
//
//
//
//    //// To here..
//  //// can be replaced;
//
//  //// and below here..
//  //// my original code so it cannot be in the hybrid code
//
//  //// heading setpoint should go into ..
//  //// guidance_h   heading reference or something.
//
//  //// Heading control
//
//// Set heading to a fixed WP
//if (use_fixed_heading_wp) {
//    nav_set_heading_towards_waypoint(heading_wp_id);
//    soaring_heading_sp = ANGLE_FLOAT_OF_BFP(nav.heading);
////    guidance_euler_cmd.psi = ANGLE_FLOAT_OF_BFP(nav.heading);
//} else {
//    float heading_ref_to_add = ((soaring_heading_sp - psi) / PERIODIC_FREQUENCY * soaring_heading_gain);
//    Bound(heading_ref_to_add, -10.0, 10.0);
//    soaring_heading_sp -= heading_ref_to_add;
//}
//    guidance_euler_cmd.psi = soaring_heading_sp;
//
//////
////// Soaring
//
//    // if in soaring mode
//    if (soaring_explore_positions && soaring_mode_running) {
//        // count for timeout
//        move_wp_wait_count++;
//
//        // calc cost fn using incremental average
//        move_wp_sum_cost = move_wp_sum_cost +
//                           (((gd_k_thr * thrust_filt.o[0]
//                              + gd_k_spd_x * fabs(groundspeed->x)     // ABS val?
//                              + gd_k_spd_z * fabs(groundspeed->z)
//                              + gd_k_pitch * fabs(euler_cmd.y)
//                              )-move_wp_sum_cost)/move_wp_wait_count);
//
//        bool wp_arrived = nav_check_wp_time_3d(&waypoints[soar_wp_id].enu_i, stay_wp_duration, arrival_check_dist_3d);
//        move_wp_wait_time = autopilot.flight_time - move_wp_entry_time;
//
//        // if wp not arrived && timeout, reset wp to the current pos
//        // if wp arrived, move wp depending on the cost fn val
//        if (wp_arrived) {
////            write_map_position_cost_info(wp_soaring_pos, move_wp_sum_cost /
////                                                         soaring_move_wp_wait_sec);   // save the position and corresponding cost val
//            // move wp
//            if (move_wp_sum_cost > soaring_move_wp_cost_threshold) {
//                guidance_indi_soaring_move_wp(move_wp_sum_cost);    // explore or go back to the original position
//            } else {
//                // stay
//                prev_wp_sum_cost = -2;
//            }
//                // reset count & cost
//            move_wp_entry_time = autopilot.flight_time;
//            move_wp_sum_cost = 0;
//            move_wp_wait_count = 0;
//        } else if (move_wp_wait_time > soaring_move_wp_wait_sec) {
//            // not arrived && timeout
//            // reset wp to current pos (assuming the wp cannot be reached)
//            guidance_indi_soaring_reset_soaring_wp();
//            prev_wp_sum_cost = -1;
//            move_wp_entry_time = autopilot.flight_time;
//            move_wp_sum_cost = 0;
//            move_wp_wait_count = 0;
//        }
//    }
//
//    // in soaring mode && manual search
//    if (soaring_mode_running && soaring_manual_search && (!soaring_explore_positions)) {
//        if (fabs(soaring_wp_move_forward) > 0.05 || fabs(soaring_wp_move_right) > 0.05 || fabs(soaring_wp_move_up) > 0.05) {
//            amount_to_move_body.x = soaring_wp_move_forward;
//            amount_to_move_body.y = soaring_wp_move_right;
//            amount_to_move_body.z = -soaring_wp_move_up;
//
//            // rotate wp body to ned
//            amount_to_move_ned.x = cosf(psi)*amount_to_move_body.x - sinf(psi)*amount_to_move_body.y;
//            amount_to_move_ned.y = sinf(psi)*amount_to_move_body.x + cosf(psi)*amount_to_move_body.y;
//            amount_to_move_ned.z = amount_to_move_body.z;   // z doesn't change
//
//            // move waypoints
//            waypoints[soar_wp_id].enu_i.x += POS_BFP_OF_REAL(amount_to_move_ned.y);
//            waypoints[soar_wp_id].enu_i.y += POS_BFP_OF_REAL(amount_to_move_ned.x);
//            waypoints[soar_wp_id].enu_i.z += POS_BFP_OF_REAL(-1.0*amount_to_move_ned.z);
//
//            nav_flight_altitude = waypoints[soar_wp_id].enu_i.z; // FIXME: manually set z ref
//
//            DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &soar_wp_id,
//                                       &(waypoints[soar_wp_id].enu_i.x),
//                                       &(waypoints[soar_wp_id].enu_i.y),
//                                       &(waypoints[soar_wp_id].enu_i.z));
//
//            soaring_wp_move_forward = 0;
//            soaring_wp_move_right = 0;
//            soaring_wp_move_up = 0;
//        }
//    }
//
//    if (stdby_entry_time > 0 && (stdby_entry_time+reset_stdby_timeout < autopilot.flight_time)) {
//        waypoint_set_here(stdby_wp_id);
//        guidance_indi_soaring_reset_soaring_wp();
//        stdby_entry_time = 0;
//    }
//}

///**
// * Calculate the matrix of partial derivatives of the roll, pitch and thrust
// * w.r.t. the NED accelerations, taking into account the lift of a wing that is
// * horizontal at -90 degrees pitch
// *
// * @param Gmat array to write the matrix to [3x3]
// */
//void guidance_indi_calcg_wing(struct FloatMat33 *Gmat) {
//
//  /*Pre-calculate sines and cosines*/
//  float sphi = sinf(eulers_zxy.phi);
//  float cphi = cosf(eulers_zxy.phi);
//  float stheta = sinf(eulers_zxy.theta);
//  float ctheta = cosf(eulers_zxy.theta);
//  float spsi = sinf(eulers_zxy.psi);
//  float cpsi = cosf(eulers_zxy.psi);
//  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better
//
//#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
//#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
//#endif
//
//  /*Amount of lift produced by the wing*/
//  float pitch_lift = eulers_zxy.theta;
//  Bound(pitch_lift,-M_PI_2,0);
//  float lift = sinf(pitch_lift)*9.81;
//  float T = cosf(pitch_lift)*-9.81;
//
//  // get the derivative of the lift wrt to theta
//  float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);
//
//  RMAT_ELMT(*Gmat, 0, 0) =  cphi*ctheta*spsi*T + cphi*spsi*lift;
//  RMAT_ELMT(*Gmat, 1, 0) = -cphi*ctheta*cpsi*T - cphi*cpsi*lift;
//  RMAT_ELMT(*Gmat, 2, 0) = -sphi*ctheta*T -sphi*lift;
//  RMAT_ELMT(*Gmat, 0, 1) = (ctheta*cpsi - sphi*stheta*spsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING + sphi*spsi*liftd;
//  RMAT_ELMT(*Gmat, 1, 1) = (ctheta*spsi + sphi*stheta*cpsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*cpsi*liftd;
//  RMAT_ELMT(*Gmat, 2, 1) = -cphi*stheta*T*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;
//  RMAT_ELMT(*Gmat, 0, 2) = stheta*cpsi + sphi*ctheta*spsi;
//  RMAT_ELMT(*Gmat, 1, 2) = stheta*spsi - sphi*ctheta*cpsi;
//  RMAT_ELMT(*Gmat, 2, 2) = cphi*ctheta;
//}
//
///**
// * @brief Get the derivative of lift w.r.t. pitch.
// *
// * @param airspeed The airspeed says most about the flight condition
// *
// * @return The derivative of lift w.r.t. pitch
// */
//float guidance_indi_get_liftd(float airspeed, float theta) {
//  float liftd = 0.0;
//  if(airspeed < 12) {
//    float pitch_interp = DegOfRad(theta);
//    Bound(pitch_interp, -80.0, -40.0);
//    float ratio = (pitch_interp + 40.0)/(-40.);
//    liftd = -24.0*ratio*lift_pitch_eff/0.12;
//  } else {
//    liftd = -(airspeed - 8.5)*lift_pitch_eff/M_PI*180.0;
//  }
//  //TODO: bound liftd
//  return liftd;
//}


/////// New code below

// TODO: change this...
//float WEAK guidance_indi_get_liftd(float airspeed, float theta) {
//    float liftd = 0.0f;
//
//    if (airspeed < 12.f) {
//        /* Assume the airspeed is too low to be measured accurately
//          * Use scheduling based on pitch angle instead.
//          * You can define two interpolation segments
//          */
//        float pitch_interp = DegOfRad(theta);
//        const float min_pitch = -80.0f;
//        const float middle_pitch = -50.0f;
//        const float max_pitch = -20.0f;
//
//        Bound(pitch_interp, min_pitch, max_pitch);
//        if (pitch_interp > middle_pitch) {
//            float ratio = (pitch_interp - max_pitch)/(middle_pitch - max_pitch);
//            liftd = -gih_params.liftd_p50*ratio;
//        } else {
//            float ratio = (pitch_interp - middle_pitch)/(min_pitch - middle_pitch);
//            liftd = -(gih_params.liftd_p80-gih_params.liftd_p50)*ratio - gih_params.liftd_p50;
//        }
//    } else {
//        liftd = -gih_params.liftd_asq*airspeed*airspeed;
//    }
//
//    //TODO: bound liftd
//    return liftd;
//}

/**
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * w.r.t. the NED accelerations, taking into account the lift of a wing that is
 * horizontal at -90 degrees pitch
 *
 * @param Gmat Dynamics matrix
 * @param a_diff acceleration errors in earth frame
 * @param body_v 3D vector to write the control objective v
 */

#ifdef GUIDANCE_INDI_SOARING_USE_90_PITCH_OFFSET
void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float v_gih[GUIDANCE_INDI_HYBRID_V]) {
    // Get attitude
    struct FloatEulers eulers_zxy;
    float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());


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
//    float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);
    float liftd = -24.0;    // FIXME

    Gmat[0][0] =  cphi*ctheta*spsi*T + cphi*spsi*lift;
    Gmat[1][0] = -cphi*ctheta*cpsi*T - cphi*cpsi*lift;
    Gmat[2][0] = -sphi*ctheta*T -sphi*lift;
    Gmat[0][1] = (ctheta*cpsi - sphi*stheta*spsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING + sphi*spsi*liftd;
    Gmat[1][1] = (ctheta*spsi + sphi*stheta*cpsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*cpsi*liftd;
    Gmat[2][1] = -cphi*stheta*T*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;
    Gmat[0][2] = stheta*cpsi + sphi*ctheta*spsi;
    Gmat[1][2] = stheta*spsi - sphi*ctheta*cpsi;
    Gmat[2][2] = cphi*ctheta;

    v_gih[0] = a_diff.x;
    v_gih[1] = a_diff.y;
    v_gih[2] = a_diff.z;
}
#else
/// without 90 deg rotation
void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float v_gih[GUIDANCE_INDI_HYBRID_V]) {
    // Get attitude
    struct FloatEulers eulers_zxy;
    float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

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
//    Bound(pitch_lift,-M_PI_2,0);
    float lift = -cosf(pitch_lift)*9.81;
    float T = sinf(pitch_lift)*9.81;

    // get the derivative of the lift wrt to theta
//    float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);
    float liftd = -24.0;    // FIXME


    // Calculate Cl, Cd, liftd and dragd
//    clalpha = getClAlpha(alpha);
//    cdalpha = getCdAlpha(alpha);
//    Cld = getCld(alpha);
//    Cdd = getCdd(alpha);
//
//    Ld = -qS*((ctheta-stheta)*clalpha + (ctheta+stheta)*Cld);
//    Dd = -qS*((stheta-ctheta)*cdalpha + (ctheta+stheta)*Cdd);

    // Gt + Gl
    Gmat[0][0] = -spsi*cphi*stheta*T + spsi*cphi*lift;
    Gmat[1][0] = cpsi*cphi*stheta*T - cpsi*cphi*lift;
    Gmat[2][0] = sphi*stheta*T - sphi*lift;
    Gmat[0][1] = (-stheta*cpsi-spsi*sphi*ctheta)*T*GUIDANCE_INDI_PITCH_EFF_SCALING + spsi*sphi*liftd;
    Gmat[1][1] = (-spsi*stheta+cpsi*sphi*ctheta)*T*GUIDANCE_INDI_PITCH_EFF_SCALING - cpsi*sphi*liftd;
    Gmat[2][1] = -cphi*ctheta*T*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;
    Gmat[0][2] = cpsi*ctheta - spsi*sphi*stheta;
    Gmat[1][2] = spsi*ctheta + cpsi*sphi*stheta;
    Gmat[2][2] = -cphi*stheta;

    // Gd, just in case
//    [0][1] = -cpsi*Dd;
//    [1][1] = -spsi*Dd;
//    0 for others

    v_gih[0] = a_diff.x;
    v_gih[1] = a_diff.y;
    v_gih[2] = a_diff.z;
}
#endif

#if GUIDANCE_INDI_HYBRID_USE_WLS
void guidance_indi_hybrid_set_wls_settings(float body_v[3] UNUSED, float roll_angle, float pitch_angle)
{
  // Set lower limits,
  // Set upper limits,
  // Set prefered states

  // roll
  du_min_gih[0] = -guidance_indi_max_bank - roll_angle; // roll
  du_max_gih[0] = guidance_indi_max_bank - roll_angle; //roll
  du_pref_gih[0] = 0.0 - roll_angle; // prefered delta roll angle

  // pitch
  du_min_gih[1] = RadOfDeg(GUIDANCE_INDI_MIN_PITCH) - pitch_angle; // pitch
  du_max_gih[1] = RadOfDeg(GUIDANCE_INDI_MAX_PITCH) - pitch_angle; // pitch
  du_pref_gih[1] = 0.0 - pitch_angle; // prefered delta pitch angle

// TODO:check thrust min/max
  // thrust
  du_min_gih[2] = 0.0 - actuator_state_filt_vect[4];
  du_max_gih[2] = MAX_PPRZ - actuator_state_filt_vect[4];
  du_pref_gih[2] = du_min_gih[2];
}
#endif
