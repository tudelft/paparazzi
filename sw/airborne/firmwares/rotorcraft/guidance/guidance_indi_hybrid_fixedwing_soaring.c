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

#ifndef GUIDANCE_INDI_SOARING_BODY_IS_NED
#define GUIDANCE_INDI_SOARING_BODY_IS_NED FALSE
#endif

// 90 deg pitch offset
//#ifndef GUIDANCE_INDI_SOARING_USE_90_OFFSET
//#define GUIDANCE_INDI_SOARING_USE_90_OFFSET FALSE
//#endif
#ifdef GUIDANCE_INDI_SOARING_USE_90_PITCH_OFFSET
#warning "You are using 90deg pitch offset setting! (0 deg pitch == nose to the sky)"
#endif

// Default behavior is setting stdby WP to current position
// when the mode changes to AUTO2
// Set TRUE to reset stdby WP again after n sec
#ifndef GUIDANCE_INDI_SOARING_RESET_STDBY_AFTER_N_SEC
#define GUIDANCE_INDI_SOARING_RESET_STDBY_AFTER_N_SEC TRUE
#endif
#ifndef GUIDANCE_INDI_SOARING_RESET_STDBY_TIMEOUT
#define GUIDANCE_INDI_SOARING_RESET_STDBY_TIMEOUT 3     // seconds
#endif
#ifndef GUIDANCE_INDI_SOARING_RESET_UNREACHABLE_WP
#define GUIDANCE_INDI_SOARING_RESET_UNREACHABLE_WP TRUE
#endif

// Use AOA to calculate G1
#ifndef GUIDANCE_INDI_SOARING_USE_AOA
#define GUIDANCE_INDI_SOARING_USE_AOA FALSE
#endif
#ifndef GUIDANCE_INDI_SOARING_USE_DRAG
#define GUIDANCE_INDI_SOARING_USE_DRAG FALSE
#endif

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

// 0.0057 deg == 0.0001 rad
#ifndef GUIDANCE_INDI_SOARING_CRITICAL_AOA
#define GUIDANCE_INDI_SOARING_CRITICAL_AOA 0.209
#endif

#ifndef GUIDANCE_INDI_SOARING_MAX_AOA
#define GUIDANCE_INDI_SOARING_MAX_AOA 0.34
#endif
#ifndef GUIDANCE_INDI_SOARING_MIN_AOA
#define GUIDANCE_INDI_SOARING_MIN_AOA -0.51
#endif

#ifndef GUIDANCE_INDI_SOARING_SPLINE_AOA_OFFSET
#define GUIDANCE_INDI_SOARING_SPLINE_AOA_OFFSET 0
#endif
#ifndef GUIDANCE_INDI_SOARING_SPLINE_AOA_SCALE
#define GUIDANCE_INDI_SOARING_SPLINE_AOA_SCALE 1.0
#endif

#ifndef GUIDANCE_INDI_SOARING_SPLINE_CL_OFFSET
#define GUIDANCE_INDI_SOARING_SPLINE_CL_OFFSET 0
#endif
#ifndef GUIDANCE_INDI_SOARING_SPLINE_CL_SCALE
#define GUIDANCE_INDI_SOARING_SPLINE_CL_SCALE 1.0
#endif

#ifndef GUIDANCE_INDI_SOARING_SPLINE_CD_SCALE
#define GUIDANCE_INDI_SOARING_SPLINE_CD_SCALE 1.0
#endif

#ifndef GUIDANCE_INDI_SOARING_Q
#define GUIDANCE_INDI_SOARING_Q 0.1103
#endif

//#ifndef GUIDANCE_INDI_SOARING_USE_SPLINE
//#define GUIDANCE_INDI_SOARING_USE_SPLINE FALSE
//#endif

// 2m/0.1 = 20 data points for one axis
#define MAP_MAX_NUM_POINTS 400

//temp vars for logging
float L;
float Ld;
float Dd;

//bool guidance_indi_soaring_use_spline_clcd = GUIDANCE_INDI_SOARING_USE_SPLINE;
bool guidance_indi_soaring_use_drag = GUIDANCE_INDI_SOARING_USE_DRAG;
bool guidance_indi_soaring_use_aoa = GUIDANCE_INDI_SOARING_USE_AOA;

float guidance_indi_soaring_min_aoa = GUIDANCE_INDI_SOARING_MIN_AOA;
float guidance_indi_soaring_max_aoa = GUIDANCE_INDI_SOARING_MAX_AOA;

float spline_aoa_scale = GUIDANCE_INDI_SOARING_SPLINE_AOA_SCALE;
float spline_aoa_offset = GUIDANCE_INDI_SOARING_SPLINE_AOA_OFFSET;
float spline_cl_offset = GUIDANCE_INDI_SOARING_SPLINE_CL_OFFSET;
float spline_cl_scale = GUIDANCE_INDI_SOARING_SPLINE_CL_SCALE;
float spline_cd_scale = GUIDANCE_INDI_SOARING_SPLINE_CD_SCALE;

bool use_aoa_pitch_pref = false;
bool use_aoa_pitch_limit = false;
float soaring_critical_aoa = GUIDANCE_INDI_SOARING_CRITICAL_AOA;
float pitch_eff_scale = GUIDANCE_INDI_PITCH_EFF_SCALING;
bool reset_unreachable_wp = GUIDANCE_INDI_SOARING_RESET_UNREACHABLE_WP;
bool reset_stdby_after_timeout = GUIDANCE_INDI_SOARING_RESET_STDBY_AFTER_N_SEC;
bool move_wp_body_is_ned = GUIDANCE_INDI_SOARING_BODY_IS_NED;
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
struct FloatVect3 preset_move_body[8] = {{1.0, 0., 0.}, {-1.0, 0., -1.0}, {0., 0., -1.0}, {1.0, 0., -1.0}, {-1.0, 0., 1.0}, {0., 0., 1.0}, {1.0, 0., 1.0}, {-1.0, 0., 0.}};
struct FloatVect3 preset_move_down[1] = {{0., 0., 1}};
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

bool only_move_down = 0;
float only_move_down_step_size = 0.02;
float soaring_move_wp_cost_threshold_phase2 = GUIDANCE_INDI_SOARING_WP_COST_THRES * 0.5;

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
uint16_t reset_stdby_timeout = GUIDANCE_INDI_SOARING_RESET_STDBY_TIMEOUT;

uint8_t using_reduced_ctrl = 0;

struct Waypoint soar_wp;

void guidance_indi_soaring_move_wp(float cost_avg_val);
void write_map_position_cost_info(struct FloatVect3 soaring_position, float corres_sum_cost);
void guidance_indi_hybrid_soaring_stop(void);
void guidance_indi_hybrid_soaring_start(void);
void guidance_indi_hybrid_soaring_reset(void);

//void guidance_indi_soaring_reset_wp(uint8_t wp_id);
void guidance_indi_soaring_reset_stby_wp(void);
void guidance_indi_soaring_reset_soaring_wp(void);

float guidance_indi_soaring_get_lift(float aoa, float airspeed, float Q);
float guidance_indi_soaring_get_liftd(float aoa, float airspeed, float Q);
float guidance_indi_soaring_get_drag(float aoa, float airspeed, float Q);
float guidance_indi_soaring_get_dragd(float aoa, float airspeed, float Q);

float guidance_indi_soaring_get_lift_spline(float aoa, float airspeed, float Q, float offset_aoa, float scale_aoa, float offset_cl, float scale_cl);
float guidance_indi_soaring_get_liftd_spline(float aoa, float airspeed, float Q, float offset_aoa, float scale_aoa, float scale_cl);
float guidance_indi_soaring_get_dragd_spline(float aoa, float airspeed, float Q, float offset_aoa, float scale_aoa, float scale_cl);

#define DEG2RAD 0.017

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_guidance_indi_hybrid(struct transport_tx *trans, struct link_device *dev)
{
    float aoa = stateGetAngleOfAttack_f();
    uint8_t use_aoa = guidance_indi_soaring_use_aoa;
    uint8_t use_drag = guidance_indi_soaring_use_drag;
    uint8_t ctrl_switching_enabled = soaring_ctrl_switch;
//    uint8_t using_reduced_ctrl = used_reduced_ctrl;
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
                              &soaring_heading_sp,
                              &aoa,
                              &L, &Ld, &Dd,
                              &use_aoa,
                              &use_drag,
                              &ctrl_switching_enabled,
                              &using_reduced_ctrl,
                              &soaring_critical_aoa
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

    if (only_move_down || (cost_avg_val < soaring_move_wp_cost_threshold_phase2)) {
        VECT3_SMUL(amount_to_move_body, preset_move_down[0], only_move_down_step_size);
    } else {
        VECT3_SMUL(amount_to_move_body, preset_move_body[idx_to_move], step_size);
    }

    if ((POS_FLOAT_OF_BFP(waypoints[soar_wp_id].enu_i.z) + amount_to_move_body.z) < soaring_min_alt) {
        // pick a random neighbour that doesn't go down; default is 5m
//        idx_to_move = abs((int)(stateGetAccelNed_f()->z*100))%5;
//        idx_to_move = abs((int)(filt_accel_ned[2].o[0]*100))%5;
//        prev_move_idx = idx_to_move;
//        prev_wp_sum_cost = cost_avg_val;
//        VECT3_SMUL(amount_to_move_body, preset_move_body[idx_to_move], step_size);
//        amount_to_move_body.z = POS_FLOAT_OF_BFP(waypoints[soar_wp_id].enu_i.z) - soaring_min_alt;
    }

    struct FloatEulers eulers_zxy;
    float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
    float psi = eulers_zxy.psi;

    if (move_wp_body_is_ned) {
        // else move the wp in NED (for indoor test)
        amount_to_move_ned.x = amount_to_move_body.x;
        amount_to_move_ned.y = amount_to_move_body.y;
        amount_to_move_ned.z = amount_to_move_body.z;   // z doesn't change
    } else {
        // rotate wp body to ned
        amount_to_move_ned.x = cosf(psi) * amount_to_move_body.x - sinf(psi) * amount_to_move_body.y;
        amount_to_move_ned.y = sinf(psi) * amount_to_move_body.x + cosf(psi) * amount_to_move_body.y;
        amount_to_move_ned.z = amount_to_move_body.z;   // z doesn't change
    }

    // move waypoints
    soar_wp.enu_i.x = waypoints[soar_wp_id].enu_i.x + POS_BFP_OF_REAL(amount_to_move_ned.y);
    soar_wp.enu_i.y = waypoints[soar_wp_id].enu_i.y + POS_BFP_OF_REAL(amount_to_move_ned.x);
    soar_wp.enu_i.z = waypoints[soar_wp_id].enu_i.z + POS_BFP_OF_REAL(-1.0*amount_to_move_ned.z);
    waypoint_move_enu_i(soar_wp_id, &soar_wp.enu_i);

    soaring_search_cnt_steps ++;
    if (soaring_min_cost > cost_avg_val) {
        soaring_min_cost = cost_avg_val;
        min_cost_wp_e = soar_wp.enu_i.x;
        min_cost_wp_n = soar_wp.enu_i.y;
        min_cost_wp_u = soar_wp.enu_i.z;
    }
    // update threshold
    if (soaring_search_cnt_steps > soaring_max_exploration_steps) {
        if (soaring_move_wp_cost_threshold < soaring_min_cost) {
            soaring_move_wp_cost_threshold = soaring_min_cost;
        }
    }
}

// reset wp to current position
//void guidance_indi_soaring_reset_wp(uint8_t wp_id) {
//    struct Waypoint wp_to_reset;
//
//    wp_to_reset.enu_i.x = POS_BFP_OF_REAL(stateGetPositionNed_f()->y);
//    wp_to_reset.enu_i.y = POS_BFP_OF_REAL(stateGetPositionNed_f()->x);
//    wp_to_reset.enu_i.z = -1.0*stateGetPositionNed_i()->z;
//
//    waypoint_move_enu_i(wp_id, &soar_wp.enu_i);
//}

// reset soaring wp to current position
void guidance_indi_soaring_reset_soaring_wp(void) {
    waypoint_set_here(soar_wp_id);
}

// reset stdby WP to current position
void guidance_indi_soaring_reset_stby_wp(void) {
    waypoint_set_here(stdby_wp_id);

    // save entry time
    stdby_entry_time = autopilot.flight_time;
}

void guidance_indi_hybrid_soaring_start(void) {
    guidance_indi_hybrid_soaring_reset();
//    if (soaring_mode_running) {
//        guidance_indi_soaring_reset_soaring_wp();   // reset wp to current position
//    }
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

    soar_wp.enu_i.x = waypoints[stdby_wp_id].enu_i.x;
    soar_wp.enu_i.y = waypoints[stdby_wp_id].enu_i.y;
    soar_wp.enu_i.z = waypoints[stdby_wp_id].enu_i.z;
    waypoint_move_enu_i(soar_wp_id, &soar_wp.enu_i);

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

    // save wp info for logging
    wp_pos_x = waypoints[soar_wp_id].enu_i.y;
    wp_pos_y = waypoints[soar_wp_id].enu_i.x;
    wp_pos_z = waypoints[soar_wp_id].enu_i.z;

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
            if (reset_unreachable_wp) {
                guidance_indi_soaring_reset_soaring_wp();
                prev_wp_sum_cost = -1;
            } else {
                guidance_indi_soaring_move_wp(move_wp_sum_cost);    // explore or go back to the original position
            }
            move_wp_entry_time = autopilot.flight_time;
            move_wp_sum_cost = 0;
            move_wp_wait_count = 0;
        }
    }

    // in soaring mode && manual search
    if (soaring_mode_running && soaring_manual_search && (!soaring_explore_positions)) {
        if (fabs(soaring_wp_move_forward) > 0.01 || fabs(soaring_wp_move_right) > 0.01 || fabs(soaring_wp_move_up) > 0.01) {
            struct FloatEulers eulers_zxy;
            float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
            float psi = eulers_zxy.psi;

            amount_to_move_body.x = soaring_wp_move_forward;
            amount_to_move_body.y = soaring_wp_move_right;
            amount_to_move_body.z = -soaring_wp_move_up;

            if (move_wp_body_is_ned) {
                // body == ned; (for indoor test)
                amount_to_move_ned.x = amount_to_move_body.x;
                amount_to_move_ned.y = amount_to_move_body.y;
                amount_to_move_ned.z = amount_to_move_body.z;   // z doesn't change
            } else {
                // rotate wp body to ned
                amount_to_move_ned.x = cosf(psi) * amount_to_move_body.x - sinf(psi) * amount_to_move_body.y;
                amount_to_move_ned.y = sinf(psi) * amount_to_move_body.x + cosf(psi) * amount_to_move_body.y;
                amount_to_move_ned.z = amount_to_move_body.z;   // z doesn't change
            }

            // move soaring wp
            soar_wp.enu_i.x = waypoints[soar_wp_id].enu_i.x + POS_BFP_OF_REAL(amount_to_move_ned.y);
            soar_wp.enu_i.y = waypoints[soar_wp_id].enu_i.y + POS_BFP_OF_REAL(amount_to_move_ned.x);
            soar_wp.enu_i.z = waypoints[soar_wp_id].enu_i.z + POS_BFP_OF_REAL(-1.0*amount_to_move_ned.z);
            waypoint_move_enu_i(soar_wp_id, &soar_wp.enu_i);

            move_wp_entry_time = autopilot.flight_time;

            soaring_wp_move_forward = 0;
            soaring_wp_move_right = 0;
            soaring_wp_move_up = 0;
        }
    }

    // to reset standby waypoint after turning on AUTO2
    // quick fix for position overshooting at STDBY
    if (reset_stdby_after_timeout) {
        if (stdby_entry_time > 0 && (stdby_entry_time + reset_stdby_timeout < autopilot.flight_time)) {
            waypoint_set_here(stdby_wp_id);
            guidance_indi_soaring_reset_soaring_wp();
            stdby_entry_time = 0;
        }
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
//    pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
    pos_err.z = -POS_FLOAT_OF_BFP(waypoints[soar_wp_id].enu_i.z) - stateGetPositionNed_f()->z;
   // TODO: bugfix

    speed_sp.x = pos_err.x * gih_params.pos_gain;
    speed_sp.y = pos_err.y * gih_params.pos_gainy;
    speed_sp.z = pos_err.z * gih_params.pos_gainz;
    Bound(speed_sp.z, -nav.climb_vspeed, -nav.descend_vspeed);   // FIXME vspeeds
    // climb >0 descend <0 ???? wtf.. z up?

    if (stab_indi_kill_throttle) {
        speed_sp.z = 1.0f;
    }
    accel_sp.x = (speed_sp.x - stateGetSpeedNed_f()->x) * gih_params.speed_gain;
    accel_sp.y = (speed_sp.y - stateGetSpeedNed_f()->y) * gih_params.speed_gainy;
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

float guidance_indi_soaring_get_lift_spline(float aoa, float airspeed, float Q, float offset_aoa, float scale_aoa, float offset_cl, float scale_cl) {
    aoa = aoa/scale_aoa - offset_aoa;

    float cl = 0;

    if (aoa < -0.235619) {
        cl = -1;
    } else if ((-0.235619 <= aoa) && (aoa < -0.218166)){
        cl = -247.674812 * (aoa + 0.235619)*(aoa + 0.235619)*(aoa + 0.235619) + 101.948160 * (aoa + 0.235619)*(aoa + 0.235619) - 4.573169 * (aoa + 0.235619) - 0.996307;
    } else if ((-0.218166 <= aoa) && (aoa < -0.200713)){
        cl = 193.097364 * (aoa + 0.218166)*(aoa + 0.218166)*(aoa + 0.218166) + 88.979937 * (aoa + 0.218166)*(aoa + 0.218166) - 1.240845 * (aoa + 0.218166) - 1.046385;
    } else if ((-0.200713 <= aoa) && (aoa < -0.178024)){
        cl = -991.640866 * (aoa + 0.200713)*(aoa + 0.200713)*(aoa + 0.200713) + 99.090491 * (aoa + 0.200713)*(aoa + 0.200713) + 2.041603 * (aoa + 0.200713) - 1.039911;
    } else if ((-0.178024 <= aoa) && (aoa < -0.125664)){
        cl = -254.935748 * (aoa + 0.178024)*(aoa + 0.178024)*(aoa + 0.178024) + 31.591639 * (aoa + 0.178024)*(aoa + 0.178024) + 5.006686 * (aoa + 0.178024) - 0.954159;
    } else if ((-0.125664 <= aoa) && (aoa < -0.073304)){
        cl = 67.967026 * (aoa + 0.125664)*(aoa + 0.125664)*(aoa + 0.125664) - 8.453575 * (aoa + 0.125664)*(aoa + 0.125664) + 6.218193 * (aoa + 0.125664) - 0.641994;
    } else if ((-0.073304 <= aoa) && (aoa < -0.020944)){
        cl = -16.932355 * (aoa + 0.073304)*(aoa + 0.073304)*(aoa + 0.073304) + 2.222661 * (aoa + 0.073304)*(aoa + 0.073304) + 5.891943 * (aoa + 0.073304) - 0.329830;
    } else if ((-0.020944 <= aoa) && (aoa < 0.031416)){
        cl = -0.237607 * (aoa + 0.020944)*(aoa + 0.020944)*(aoa + 0.020944) - 0.437067 * (aoa + 0.020944)*(aoa + 0.020944) + 5.985436 * (aoa + 0.020944) - 0.017666;
    } else if ((0.031416 <= aoa) && (aoa < 0.083776)){
        cl = 17.882781 * (aoa - 0.031416)*(aoa - 0.031416)*(aoa - 0.031416) - 0.474391 * (aoa - 0.031416)*(aoa - 0.031416) + 5.937712 * (aoa - 0.031416) + 0.294499;
    } else if ((0.083776 <= aoa) && (aoa < 0.136136)){
        cl = -71.293517 * (aoa - 0.083776)*(aoa - 0.083776)*(aoa - 0.083776) + 2.334630 * (aoa - 0.083776)*(aoa - 0.083776) + 6.035114 * (aoa - 0.083776) + 0.606663;
    } else if ((0.136136 <= aoa) && (aoa < 0.191986)){
        cl = -57.652565 * (aoa - 0.136136)*(aoa - 0.136136)*(aoa - 0.136136) - 8.864129 * (aoa - 0.136136)*(aoa - 0.136136) + 5.693230 * (aoa - 0.136136) + 0.918827;
    } else if ((0.191986 <= aoa) && (aoa < 0.205076)){
        cl = -1344.196915 * (aoa - 0.191986)*(aoa - 0.191986)*(aoa - 0.191986) - 18.523909 * (aoa - 0.191986)*(aoa - 0.191986) + 4.163594 * (aoa - 0.191986) + 1.199104;
    } else if ((0.205076 <= aoa) && (aoa < 0.218166)){
        cl = 360.176649 * (aoa - 0.205076)*(aoa - 0.205076)*(aoa - 0.205076) - 71.310399 * (aoa - 0.205076)*(aoa - 0.205076) + 2.987665 * (aoa - 0.205076) + 1.247416;
    } else if ((0.218166 <= aoa) && (aoa < 0.231256)){
        cl = -96.509680 * (aoa - 0.218166)*(aoa - 0.218166)*(aoa - 0.218166) - 57.166295 * (aoa - 0.218166)*(aoa - 0.218166) + 1.305910 * (aoa - 0.218166) + 1.275114;
    } else if ((0.231256 <= aoa) && (aoa < 0.244346)){
        cl = 25.862071 * (aoa - 0.231256)*(aoa - 0.231256)*(aoa - 0.231256) - 60.956221 * (aoa - 0.231256)*(aoa - 0.231256) - 0.240311 * (aoa - 0.231256) + 1.282196;
    } else if ((0.244346 <= aoa) && (aoa < 0.257436)){
        cl = -6.938604 * (aoa - 0.244346)*(aoa - 0.244346)*(aoa - 0.244346) - 59.940620 * (aoa - 0.244346)*(aoa - 0.244346) - 1.822847 * (aoa - 0.244346) + 1.268664;
    } else if ((0.257436 <= aoa) && (aoa < 0.270526)){
        cl = 1.892347 * (aoa - 0.257436)*(aoa - 0.257436)*(aoa - 0.257436) - 60.213098 * (aoa - 0.257436)*(aoa - 0.257436) - 3.395655 * (aoa - 0.257436) + 1.234517;
    } else if ((0.270526 <= aoa) && (aoa < 0.283616)){
        cl = -0.630782 * (aoa - 0.270526)*(aoa - 0.270526)*(aoa - 0.270526) - 60.138786 * (aoa - 0.270526)*(aoa - 0.270526) - 4.971058 * (aoa - 0.270526) + 1.179754;
    } else {
        cl = -12 * (aoa) + 4.5078;
    }

    return -(cl*scale_cl+offset_cl)*Q*airspeed*airspeed;
}
float guidance_indi_soaring_get_liftd_spline(float aoa, float airspeed, float Q, float offset_aoa, float scale_aoa, float scale_cld) {
    aoa = aoa/scale_aoa - offset_aoa;

    float cld = 0;

    if (aoa < -0.235619){
        cld = -4.44724;
    } else if ((-0.235619 <= aoa) && (aoa < -0.218166)){
        cld = 3 * -247.674812 * (aoa + 0.235619)*(aoa + 0.235619) + 2 * 101.948160 * (aoa + 0.235619) - 4.573169;
    } else if ((-0.218166 <= aoa) && (aoa < -0.200713)){
        cld = 3 * 193.097364 * (aoa + 0.218166)*(aoa + 0.218166) + 2 * 88.979937 * (aoa + 0.218166) - 1.240845;
    } else if ((-0.200713 <= aoa) && (aoa < -0.178024)){
        cld = 3 * -991.640866 * (aoa + 0.200713)*(aoa + 0.200713) + 2 * 99.090491 * (aoa + 0.200713) + 2.041603;
    } else if ((-0.178024 <= aoa) && (aoa < -0.125664)){
        cld = 3 * -254.935748 * (aoa + 0.178024)*(aoa + 0.178024) + 2 * 31.591639 * (aoa + 0.178024) + 5.006686;
    } else if ((-0.125664 <= aoa) && (aoa < -0.073304)){
        cld = 3 * 67.967026 * (aoa + 0.125664)*(aoa + 0.125664) + 2 * -8.453575 * (aoa + 0.125664) + 6.218193;
    } else if ((-0.073304 <= aoa) && (aoa < -0.020944)){
        cld = 3 * -16.932355 * (aoa + 0.073304)*(aoa + 0.073304) + 2 * 2.222661 * (aoa + 0.073304) + 5.891943;
    } else if ((-0.020944 <= aoa) && (aoa < 0.031416)){
        cld = 3 * -0.237607 * (aoa + 0.020944)*(aoa + 0.020944) + 2 * -0.437067 * (aoa + 0.020944) + 5.985436;
    } else if ((0.031416 <= aoa) && (aoa < 0.083776)){
        cld = 3 * 17.882781 * (aoa - 0.031416)*(aoa - 0.031416) + 2 * -0.474391 * (aoa - 0.031416) + 5.937712;
    } else if ((0.083776 <= aoa) && (aoa < 0.136136)){
        cld = 3 * -71.293517 * (aoa - 0.083776)*(aoa - 0.083776) + 2 * 2.334630 * (aoa - 0.083776) + 6.035114;
    } else if ((0.136136 <= aoa) && (aoa < 0.191986)){
        cld = 3 * -57.652565 * (aoa - 0.136136)*(aoa - 0.136136) + 2 * -8.864129 * (aoa - 0.136136) + 5.693230;
    } else if ((0.191986 <= aoa) && (aoa < 0.205076)){
        cld = 3 * -1344.196915 * (aoa - 0.191986)*(aoa - 0.191986) + 2 * -18.523909 * (aoa - 0.191986) + 4.163594;
    } else if ((0.205076 <= aoa) && (aoa < 0.218166)){
        cld = 3 * 360.176649 * (aoa - 0.205076)*(aoa - 0.205076) + 2 * -71.310399 * (aoa - 0.205076) + 2.987665;
    } else if ((0.218166 <= aoa) && (aoa < 0.231256)){
        cld = 3 * -96.509680 * (aoa - 0.218166)*(aoa - 0.218166) + 2 * -57.166295 * (aoa - 0.218166) + 1.305910;
    } else if ((0.231256 <= aoa) && (aoa < 0.244346)){
        cld = 3 * 25.862071 * (aoa - 0.231256)*(aoa - 0.231256) + 2 * -60.956221 * (aoa - 0.231256) - 0.240311;
    } else if ((0.244346 <= aoa) && (aoa < 0.257436)){
        cld = 3 * -6.938604 * (aoa - 0.244346)*(aoa - 0.244346) + 2 * -59.940620 * (aoa - 0.244346) - 1.822847;
    } else if ((0.257436 <= aoa) && (aoa < 0.270526)){
        cld = 3 * 1.892347 * (aoa - 0.257436)*(aoa - 0.257436) + 2 * -60.213098 * (aoa - 0.257436) - 3.395655;
    } else if ((0.270526 <= aoa) && (aoa < 0.283616)) {
        cld = 3 * -0.630782 * (aoa - 0.270526)*(aoa - 0.270526) + 2 * -60.138786 * (aoa - 0.270526) - 4.971058;
    } else {
        cld = -12;
    }
    return -(cld*scale_cld/scale_aoa)*Q*airspeed*airspeed;     // TODO: check sign
}

//float guidance_indi_soaring_get_drag_spline(float aoa, float airspeed, float Q,  float offset_aoa, float scale_aoa, float offset_cd, float scale_cd) {
//    aoa = (aoa/scale_aoa) - offset_aoa;
//
//    float cd = 0.9171*aoa*aoa - 0.0012*aoa + 0.0014;
////    0.9171   -0.0012    0.0014
//
//    return -(cd*scale_cd+offset_cd)*Q*airspeed*airspeed;
//}

float guidance_indi_soaring_get_dragd_spline(float aoa, float airspeed, float Q,  float offset_aoa, float scale_aoa, float scale_cd) {
    aoa = (aoa/scale_aoa) - offset_aoa;

    float cdd = 0.9171*2*aoa - 0.0012;

    return -(cdd*scale_cd/scale_aoa)*Q*airspeed*airspeed;
}

// for Seal plane
float guidance_indi_soaring_get_lift(float aoa, float airspeed, float Q) {
    float c_lift = 0.0;
    if (aoa < -0.5236) {            // -30 deg;
        c_lift = 0.85+5.0*aoa;
    } else if (aoa < soaring_critical_aoa) {      // 0.095 ..; //5.88 deg;   0 at -22 deg
        c_lift = 0.85+2.1*aoa;
    } else {
//        c_lift = 1.1451-1.006596*aoa;     // 0 at 20 deg
        c_lift = -7.0*aoa*aoa + 0.92*aoa + 1.0200;
    }

    //    -6.2337    2.2200    1.0600
    return -c_lift*Q*airspeed*airspeed;
}

float guidance_indi_soaring_get_liftd(float aoa, float airspeed, float Q) {
    float c_liftd = 2.1;
    if (aoa < -0.5236) {
        c_liftd = 5.0;
    } else if (aoa < soaring_critical_aoa) {      // 5.88 deg;
        c_liftd = 2.1;
    } else {
//        c_liftd = -1.006596;
        c_liftd = -8.0*2*aoa + 3.6;
    }
    return -c_liftd*Q*airspeed*airspeed;
}

float guidance_indi_soaring_get_drag(float aoa, float airspeed, float Q) {
    float c_drag = 0.0;
//    2.2077    0.9591    0.0076
//    if (aoa < -1.57) {
//    } else if (aoa > 1.57) {
//    } else {
        c_drag = 2.2077*aoa*aoa + 0.9591*aoa + 0.0076;
//    }
    return -c_drag*Q*airspeed*airspeed;
}

float guidance_indi_soaring_get_dragd(float aoa, float airspeed, float Q) {
    float c_dragd = 0.0;
    c_dragd = 2*2.2077*aoa + 0.9591;
    return -c_dragd*Q*airspeed*airspeed;
}

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

//#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
//#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
//#endif

    /*Amount of lift produced by the wing*/
    float pitch_lift = eulers_zxy.theta;
    Bound(pitch_lift,-M_PI_2,0);
    float lift = sinf(pitch_lift)*9.81;
    float T = cosf(pitch_lift)*-9.81;

    // get the derivative of the lift wrt to theta
//    float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);
//    float liftd = -24.0;    // FIXME
    float liftd = -gih_params.liftd_asq*airspeed*airspeed;

    Gmat[0][0] =  cphi*ctheta*spsi*T + cphi*spsi*lift;
    Gmat[1][0] = -cphi*ctheta*cpsi*T - cphi*cpsi*lift;
    Gmat[2][0] = -sphi*ctheta*T -sphi*lift;
    Gmat[0][1] = (ctheta*cpsi - sphi*stheta*spsi)*T*pitch_eff_scale + sphi*spsi*liftd;
    Gmat[1][1] = (ctheta*spsi + sphi*stheta*cpsi)*T*pitch_eff_scale - sphi*cpsi*liftd;
    Gmat[2][1] = -cphi*stheta*T*pitch_eff_scale + cphi*liftd;
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

    /*Amount of lift produced by the wing*/
    float pitch_lift = eulers_zxy.theta;
//    Bound(pitch_lift,-M_PI_2,0);
    float lift = -cosf(pitch_lift)*9.81;
    float T = sinf(pitch_lift)*9.81;
//    float drag = -sinf(pitch_lift)*9.81;

    float airspeed = stateGetAirspeed_f();
//    float Q = 0.1574;       // 0.5 rho S        for Seal
    float Q = GUIDANCE_INDI_SOARING_Q;           // Eclipson C

    float aoa = stateGetAngleOfAttack_f();      // in rad
    if (aoa > guidance_indi_soaring_max_aoa) {
        aoa = guidance_indi_soaring_max_aoa;
    } else if (aoa < guidance_indi_soaring_min_aoa) {
        aoa = guidance_indi_soaring_min_aoa;
    }

    if (airspeed < 6.3) {
        // AOA sensor only works when airspeed > 7m/s
        // below that, use pitch
        aoa = pitch_lift;
        airspeed = 6.3;
    }

    // get the derivative of the lift wrt to theta
//    float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);
//    float liftd = -24.0;    // FIXME
    float liftd = -gih_params.liftd_asq*airspeed*airspeed;
//    float dragd = -2*2.2077*pitch_lift*Q*airspeed*airspeed;   // Seal
    float dragd = -2*0.9171*pitch_lift*Q*airspeed*airspeed;     // Eclipson

    // Calculate Cl, Cd, liftd and dragd
    if (guidance_indi_soaring_use_aoa) {
//        if (guidance_indi_soaring_use_spline_clcd) {
            lift = guidance_indi_soaring_get_lift_spline(aoa, airspeed, Q, spline_aoa_offset, spline_aoa_scale, spline_cl_offset, spline_cl_scale);
            liftd = guidance_indi_soaring_get_liftd_spline(aoa, airspeed, Q, spline_aoa_offset, spline_aoa_scale, spline_cl_scale);
            dragd = guidance_indi_soaring_get_dragd_spline(aoa, airspeed, Q, spline_aoa_offset, spline_aoa_scale, spline_cd_scale);
//        } else {
//            lift = guidance_indi_soaring_get_lift(aoa, airspeed, Q);
//            liftd = guidance_indi_soaring_get_liftd(aoa, airspeed, Q);
//
////        drag = guidance_indi_soaring_get_drag(aoa, airspeed, Q);
//            dragd = guidance_indi_soaring_get_dragd(aoa, airspeed, Q);
//        }
    }

    // update vars for logging
    L = lift;
    Ld = liftd;
    Dd = dragd;

    // Gt + Gl
    Gmat[0][0] = -spsi*cphi*stheta*T + spsi*cphi*lift;
    Gmat[1][0] = cpsi*cphi*stheta*T - cpsi*cphi*lift;
    Gmat[2][0] = sphi*stheta*T - sphi*lift;
    Gmat[0][1] = (-stheta*cpsi-spsi*sphi*ctheta)*T*pitch_eff_scale + spsi*sphi*liftd;
    Gmat[1][1] = (-spsi*stheta+cpsi*sphi*ctheta)*T*pitch_eff_scale - cpsi*sphi*liftd;
    Gmat[2][1] = -cphi*ctheta*T*pitch_eff_scale + cphi*liftd;
    Gmat[0][2] = cpsi*ctheta - spsi*sphi*stheta;
    Gmat[1][2] = spsi*ctheta + cpsi*sphi*stheta;
    Gmat[2][2] = -cphi*stheta;

    // Gd, just in case
    if (guidance_indi_soaring_use_drag) {
        Gmat[0][1] += cpsi*dragd;
        Gmat[1][1] += spsi*dragd;
//    [0][1] = -cpsi*Dd;
//    [1][1] = -spsi*Dd;
//    0 for others
    }

    v_gih[0] = a_diff.x;
    v_gih[1] = a_diff.y;
    v_gih[2] = a_diff.z;

    using_reduced_ctrl = 0;
}
void guidance_indi_calcg_wing_reduced(float Gmat[GUIDANCE_INDI_HYBRID_V_REDUCED][GUIDANCE_INDI_HYBRID_U_REDUCED], struct FloatVect3 a_diff, float v_gih[GUIDANCE_INDI_HYBRID_V_REDUCED], bool altitude_ctrl) {
    // Get attitude
    struct FloatEulers eulers_zxy;
    float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

    /*Pre-calculate sines and cosines*/
    float sphi = sinf(eulers_zxy.phi);
    float cphi = cosf(eulers_zxy.phi);
//    float stheta = sinf(eulers_zxy.theta);
//    float ctheta = cosf(eulers_zxy.theta);
    float spsi = sinf(eulers_zxy.psi);
    float cpsi = cosf(eulers_zxy.psi);
    //minus gravity is a guesstimate of the thrust force, thrust measurement would be better

    /*Amount of lift produced by the wing*/
    float pitch_lift = eulers_zxy.theta;
//    Bound(pitch_lift,-M_PI_2,0);
    float lift = -cosf(pitch_lift)*9.81;
//    float T = sinf(pitch_lift)*9.81;
//    float drag = -sinf(pitch_lift)*9.81;

    float airspeed = stateGetAirspeed_f();
//    float Q = 0.1574;       // 0.5 rho S        for Seal
    float Q = GUIDANCE_INDI_SOARING_Q;           // Eclipson C

    float aoa = stateGetAngleOfAttack_f();      // in rad
    if (aoa > guidance_indi_soaring_max_aoa) {
        aoa = guidance_indi_soaring_max_aoa;
    } else if (aoa < guidance_indi_soaring_min_aoa) {
        aoa = guidance_indi_soaring_min_aoa;
    }

    if (airspeed < 6.3) {
        // AOA sensor only works when airspeed > 7m/s
        // below that, use pitch
        aoa = pitch_lift;
        airspeed = 6.3;
    }

    // get the derivative of the lift wrt to theta
    float liftd = -gih_params.liftd_asq*airspeed*airspeed;
//    float dragd = -2*2.2077*pitch_lift*Q*airspeed*airspeed;   // Seal
    float dragd = -2*0.9171*pitch_lift*Q*airspeed*airspeed;     // Eclipson

    // Calculate Cl, Cd, liftd and dragd
    if (guidance_indi_soaring_use_aoa) {
//        if (guidance_indi_soaring_use_spline_clcd) {
        lift = guidance_indi_soaring_get_lift_spline(aoa, airspeed, Q, spline_aoa_offset, spline_aoa_scale, spline_cl_offset, spline_cl_scale);
        liftd = guidance_indi_soaring_get_liftd_spline(aoa, airspeed, Q, spline_aoa_offset, spline_aoa_scale, spline_cl_scale);
        dragd = guidance_indi_soaring_get_dragd_spline(aoa, airspeed, Q, spline_aoa_offset, spline_aoa_scale, spline_cd_scale);
//        } else {
//            lift = guidance_indi_soaring_get_lift(aoa, airspeed, Q);
//            liftd = guidance_indi_soaring_get_liftd(aoa, airspeed, Q);
//
////        drag = guidance_indi_soaring_get_drag(aoa, airspeed, Q);
//            dragd = guidance_indi_soaring_get_dragd(aoa, airspeed, Q);
//        }
    }

    // update vars for logging
    L = lift;
    Ld = liftd;
    Dd = dragd;

    if (altitude_ctrl) {
        // GMat = GMat(2:3, 1:2);
        // Gl
//        Gmat[0][0] = spsi*cphi*lift;
        Gmat[0][0] = -cpsi*cphi*lift;
        Gmat[1][0] = -sphi*lift;
//        Gmat[0][1] = spsi*sphi*liftd;
        Gmat[0][1] = -cpsi*sphi*liftd;
        Gmat[1][1] = cphi*liftd;

        // Gd, just in case; TODO: check sign
        if (guidance_indi_soaring_use_drag) {
//            Gmat[0][1] += cpsi*dragd;
            Gmat[0][1] += spsi*dragd;
        }

        v_gih[0] = a_diff.y;
        v_gih[1] = a_diff.z;

        using_reduced_ctrl = 1;
    } else {
        // Gl
        Gmat[0][0] = spsi*cphi*lift;
        Gmat[1][0] = -cpsi*cphi*lift;
//        Gmat[2][0] = -sphi*lift;
        Gmat[0][1] = spsi*sphi*liftd;
        Gmat[1][1] = -cpsi*sphi*liftd;
//        Gmat[2][1] = cphi*liftd;

        // Gd, just in case
        if (guidance_indi_soaring_use_drag) {
            Gmat[0][1] += cpsi*dragd;
            Gmat[1][1] += spsi*dragd;
        }
        v_gih[0] = a_diff.x;
        v_gih[1] = a_diff.y;

        using_reduced_ctrl = 2;
    }

}
#endif

#if GUIDANCE_INDI_HYBRID_USE_WLS
void guidance_indi_hybrid_set_wls_settings(float body_v[3] UNUSED, float roll_angle, float pitch_angle)
{
    float aoa = stateGetAngleOfAttack_f();      // in rad
    float airspeed = stateGetAirspeed_f();

  // Set lower limits,
  // Set upper limits,
  // Set prefered states

  // roll
  du_min_gih[0] = -guidance_indi_max_bank - roll_angle; // roll
  du_max_gih[0] = guidance_indi_max_bank - roll_angle; //roll
  du_pref_gih[0] = 0.0 - roll_angle; // prefered delta roll angle

  // pitch
  du_min_gih[1] = RadOfDeg(GUIDANCE_INDI_MIN_PITCH) - pitch_angle; // pitch
  if (use_aoa_pitch_limit && airspeed > 7.0) {
    du_max_gih[1] = soaring_critical_aoa - aoa; // pitch
  } else {
    du_max_gih[1] = RadOfDeg(GUIDANCE_INDI_MAX_PITCH) - pitch_angle; // pitch
    }
  if (use_aoa_pitch_pref && airspeed > 6.5) {
        du_pref_gih[1] = soaring_critical_aoa - aoa; // prefered delta pitch angle
  } else {
  du_pref_gih[1] = 0.0 - pitch_angle; // prefered delta pitch angle
  }

// TODO:check thrust min/max
  // thrust
  du_min_gih[2] = 0.0 - actuator_state_filt_vect[4];
  du_max_gih[2] = MAX_PPRZ - actuator_state_filt_vect[4];
  du_pref_gih[2] = du_min_gih[2];
}

void guidance_indi_hybrid_set_wls_settings_reduced(float body_v[2] UNUSED, float roll_angle, float pitch_angle)
{
    float aoa = stateGetAngleOfAttack_f();      // in rad
    float airspeed = stateGetAirspeed_f();

  // Set lower limits,
  // Set upper limits,
  // Set prefered states

  // roll
  du_min_gih_reduced[0] = -guidance_indi_max_bank - roll_angle; // roll
  du_max_gih_reduced[0] = guidance_indi_max_bank - roll_angle; //roll
  du_pref_gih_reduced[0] = 0.0 - roll_angle; // prefered delta roll angle

  // pitch
  du_min_gih_reduced[1] = RadOfDeg(GUIDANCE_INDI_MIN_PITCH) - pitch_angle; // pitch
  if (use_aoa_pitch_limit && airspeed > 7.0) {
    du_max_gih_reduced[1] = soaring_critical_aoa - aoa; // pitch
  } else {
    du_max_gih_reduced[1] = RadOfDeg(GUIDANCE_INDI_MAX_PITCH) - pitch_angle; // pitch
    }
  if (use_aoa_pitch_pref && airspeed > 6.5) {
        du_pref_gih_reduced[1] = soaring_critical_aoa - aoa; // prefered delta pitch angle
  } else {
  du_pref_gih_reduced[1] = 0.0 - pitch_angle; // prefered delta pitch angle
  }
}
#endif
