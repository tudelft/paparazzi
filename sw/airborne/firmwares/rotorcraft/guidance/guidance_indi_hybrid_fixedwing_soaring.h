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
 *
 */

#ifndef GUIDANCE_INDI_HYBRID_FIXEDWING_SOARING
#define GUIDANCE_INDI_HYBRID_FIXEDWING_SOARING


#ifndef GUIDANCE_INDI_MIN_PITCH
#define GUIDANCE_INDI_MIN_PITCH -40
#define GUIDANCE_INDI_MAX_PITCH 40
#endif


//// Old codes
//// Below here

#ifndef GUIDANCE_INDI_SOARING
#define GUIDANCE_INDI_SOARING TRUE
#endif

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "filters/high_pass_filter.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"

extern void guidance_indi_soaring_enter(void);
extern void guidance_indi_soaring_run(float *heading_sp);
//extern void stabilization_attitude_set_setpoint_rp_quat_f(struct FloatEulers* indi_rp_cmd, bool in_flight, int32_t heading);
extern void guidance_indi_soaring_init(void);
//extern void guidance_indi_soaring_propagate_filters(void);
extern void guidance_indi_hybrid_soaring_start(void);
extern void guidance_indi_hybrid_soaring_stop(void);
extern void guidance_indi_hybrid_soaring_reset(void);
extern void guidance_indi_soaring_reset_stby_wp(void);
extern void guidance_indi_soaring_reset_soaring_wp(void);

extern float compute_soaring_heading_sp(void);
extern struct FloatVect3 compute_soaring_accel_sp(struct HorizontalGuidance *gh, struct VerticalGuidance *gv);
extern void run_soaring_search(void);

extern void guidance_indi_hybrid_soaring_set_all_flags(void);
extern void guidance_indi_hybrid_soaring_off_all_flags(void);

//struct guidance_indi_hybrid_params {
//  float pos_gain;
//  float pos_gainz;
//  float speed_gain;
//  float speed_gainz;
//  float heading_bank_gain;
//};

struct SoaringPositionMap {
    struct FloatVect3 pos;
    float cost;
};

//extern struct guidance_indi_hybrid_params gih_params;
//extern float guidance_indi_specific_force_gain;
//extern float guidance_indi_max_airspeed;
//extern float nav_max_speed;
//extern bool take_heading_control;
//extern float guidance_indi_max_bank;
//extern float lift_pitch_eff;

//extern struct FloatVect3 guidance_wind_gradient;

extern bool move_wp_body_is_ned;
extern bool reset_stdby_after_timeout;
extern bool reset_unreachable_wp;
extern bool guidance_indi_soaring_use_aoa;
extern bool guidance_indi_soaring_use_drag;
extern float pitch_eff_scale;
extern float soaring_critical_aoa;
extern bool use_aoa_pitch_limit;
extern bool use_aoa_pitch_pref;

extern bool only_move_down;
extern float only_move_down_step_size;
extern float soaring_move_wp_cost_threshold_phase2;

//extern bool guidance_indi_soaring_use_spline_clcd;
extern float guidance_indi_soaring_max_aoa;
extern float guidance_indi_soaring_min_aoa;

extern float spline_aoa_scale;
extern float spline_aoa_offset;
extern float spline_cl_offset;
extern float spline_cl_scale;
extern float spline_cd_scale;

extern float guidance_soaring_max_throttle;
extern bool speed_sp_from_position;
extern bool y_position_ctrl;
extern bool soaring_explore_positions;
extern bool soaring_mode_running;
//extern float gd_gamma_sq;
//extern float gd_Wv[3];
//extern float gd_Wu[3];
//extern float gd_du_pref[3];
extern bool use_variable_weights;

extern struct FloatVect2 heading_target;
extern struct FloatVect3 soaring_spd_sp;

extern float soaring_move_wp_wait_sec;
extern float soaring_move_wp_cost_threshold;
extern float gd_k_thr;
extern float gd_k_spd_x;
extern float gd_k_spd_z;
extern float gd_k_pitch;

extern float soaring_fixed_step_size;
extern bool soaring_use_fixed_step_size;
extern float soaring_step_k_big;
extern float soaring_step_k_mid;
extern float soaring_step_k_small;

extern float soaring_step_size_big;
extern float soaring_step_size_mid;
extern float soaring_step_size_small;
extern float soaring_step_size_fine;

extern float arrival_check_dist_3d;
extern float stay_wp_duration;
extern float weight_logistic_fn_factor;

extern float min_acc_sp;
extern float max_acc_sp;

extern float soaring_min_alt;

extern bool use_fixed_heading_wp;
extern float soaring_heading_gain;
extern uint16_t soaring_max_exploration_steps;

extern float soaring_wp_move_forward;
extern float soaring_wp_move_right;
extern float soaring_wp_move_up;
extern bool soaring_manual_search;

extern uint16_t reset_stdby_timeout;

#endif // GUIDANCE_INDI_HYBRID_FIXEDWING_SOARING
