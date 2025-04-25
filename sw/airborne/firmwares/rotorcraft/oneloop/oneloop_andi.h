/*
 * Copyright (C) 2023 Tomaso De Ponti <tmldeponti@tudelft.nl>
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

/** @file "firmwares/rotorcraft/oneloop/oneloop_andi.h"
 * @author Tomaso De Ponti <tmldeponti@tudelft.nl>
 * One loop (Guidance + Stabilization) ANDI controller for the rotating wing drone RW3C
 */

#ifndef ONELOOP_ANDI_H
#define ONELOOP_ANDI_H

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "generated/airframe.h"
#include "filters/low_pass_filter.h"
#include "filters/notch_filter_float.h"
//#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

#ifndef ANDI_NUM_ACT
#define ANDI_NUM_ACT COMMANDS_NB_REAL
#endif

#ifndef ANDI_NUM_VIRTUAL_ACT
#define ANDI_NUM_VIRTUAL_ACT COMMANDS_NB_VIRTUAL
#endif

// Number of virtual actuators (e.g. Phi, Theta). For now 2 and only 2 are supported but in the future this can be further developed. 
#if ANDI_NUM_VIRTUAL_ACT < 2
#error "You must specify the number of virtual actuators to be at least 2"
#define ANDI_NUM_VIRTUAL_ACT 2
#endif

#define ANDI_NUM_ACT_TOT (ANDI_NUM_ACT + ANDI_NUM_VIRTUAL_ACT)

#ifndef ANDI_OUTPUTS
#error "You must specify the number of controlled axis (outputs)"
#define ANDI_OUTPUTS 6
#endif
#define ANDI_G_SCALING 1000.0f

/** Control types.*/
#define  CTRL_ANDI 0
#define  CTRL_INDI 1

extern bool ctrl_off;
extern float act_state_filt_vect_1l[ANDI_NUM_ACT];
extern float actuator_state_1l[ANDI_NUM_ACT_TOT];
extern float nu[6];
extern float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
extern float andi_u[ANDI_NUM_ACT_TOT];
extern float andi_du[ANDI_NUM_ACT_TOT];
extern float psi_des_deg;
extern bool  heading_manual;
extern bool  yaw_stick_in_auto;
extern float fwd_sideslip_gain;
extern struct FloatEulers eulers_zxy_des;
extern float psi_des_rad;
extern float k_as;
extern float max_as;
extern float gi_unbounded_airspeed_sp;
extern float temp_k;
/*Chirp test Variables*/
extern bool  chirp_on;
extern float f0_chirp;
extern float f1_chirp;
extern float t_chirp;
extern float A_chirp;
extern int8_t chirp_axis;

// Delete once hybrid nav is fixed //////////////////////////////////////////////////////////////////////////////////
struct guidance_indi_hybrid_params {
  float pos_gain;
  float pos_gainz;
  float speed_gain;
  float speed_gainz;
  float heading_bank_gain;
  float liftd_asq;
  float liftd_p80;
  float liftd_p50;
};
extern struct guidance_indi_hybrid_params gih_params;
//extern bool force_forward; 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct OneloopGuidanceRef {
  float pos[3];     
  float vel[3]; 
  float acc[3];
  float jer[3];
};

struct OneloopGuidanceState {
  float pos[3];     
  float vel[3]; 
  float acc[3];
};

struct OneloopStabilizationRef {
  float att[3];     
  float att_d[3]; 
  float att_2d[3];
  float att_3d[3];
};

struct OneloopStabilizationState {
  float att[3];     
  float att_d[3]; 
  float att_2d[3];
};
struct OneloopGeneral {
  bool   half_loop;
  int    ctrl_type;
  struct OneloopGuidanceRef         gui_ref;     // Guidance References
  struct OneloopGuidanceState       gui_state;   // Guidance State
  struct OneloopStabilizationRef    sta_ref;     // Stabilization References
  struct OneloopStabilizationState  sta_state;   // Stabilization State
};

extern struct OneloopGeneral oneloop_andi;

struct PolePlacement{
  float omega_n;
  float zeta;
  float p3;
};
struct Gains3rdOrder{
  float k1[3];
  float k2[3];
  float k3[3];
};
struct Gains2ndOrder{
  float k2;
  float k3;
};

/* Possible types of Filters */
enum FilterType {
  LOWPASS_1,
  BUTTERWORTH_2,
  BUTTERWORTH_4,
  NOTCH
};
struct LP_t {
  float  tau;
  float  freq;
  float  freq_set;
  float  bandwidth;
  float  meas;
  float  meas_prev;
  float  out;
  enum FilterType filter_type; // Type of filter used for the structural modes
  union {
    struct FirstOrderLowPass lp1;
    Butterworth2LowPass bw2;
    Butterworth4LowPass bw4;
    struct SecondOrderNotchFilter notch;
  } meas_filt;
};
struct Oneloop_LP_t {
  struct LP_t p;
  struct LP_t q;
  struct LP_t r;
  struct LP_t p_dot;
  struct LP_t q_dot;
  struct LP_t r_dot;
  struct LP_t ax;
  struct LP_t ay;
  struct LP_t az;
};
extern struct Oneloop_LP_t LP;
struct notch_axis_t{
  struct SecondOrderNotchFilter filter;
  float freq;
  float bandwidth;
};

struct CustomFilter {
  float fc;
  float dt;
  float rho;
  float yk;
  float yk1;
  float uk;
  float uk1;
};
// struct Oneloop_CustomFilter_t {
//   struct CustomFilter p_dot;
//   struct CustomFilter q_dot;
//   struct CustomFilter r_dot;
//   struct CustomFilter ax;
//   struct CustomFilter ay;
//   struct CustomFilter az;
// };
struct Oneloop_CustomFilter_t {
  struct CustomFilter p_dot;
  struct CustomFilter q_dot;
  struct CustomFilter r_dot;
  struct CustomFilter ax;
  struct CustomFilter ay;
  struct CustomFilter az;
};
/** Init Custum filter.
 *
 * Laplace transform in continious time:
 *         (1-rho)*s + fc*rho
 * H(s) = --------------------
 *           rho*s + fc*rho
 *
 * @param filter      Custom filter structure
 * @param fc          Filtering frequency [Hz]
 * @param sample_time Sampling period of the signal
 * @param value       Initial value of the filter
 */
static inline void init_custom_filter(struct CustomFilter *filter, float fc, float sample_time, float rho, float value)
{
  filter->fc  = fc * 2.0f * M_PI;
  filter->dt  = sample_time;
  filter->rho = rho;
  filter->yk  = value;
  filter->yk1 = value;
  filter->uk  = value;
  filter->uk1 = value;
  //printf("fc: %f, dt: %f, rho: %f, yk: %f, yk1: %f, uk: %f, uk1: %f\n", filter->fc, filter->dt, filter->rho, filter->yk, filter->yk1, filter->uk, filter->uk1);
}
/** Update Custom filter state with a new value.
 *
 * @param filter Custom filter structure
 * @param value  New input value of the filter
 * @return       New filtered value
 */
static inline void update_custom_filter(struct CustomFilter *filter, float value)
{
  filter->uk  = filter->uk1;
  filter->uk1 = value;
  filter->yk  = filter->yk1;
  float D = (filter->dt * filter->fc + 2);
  float A = -(filter->dt * filter->fc - 2) / D;
  float B = (filter->dt * filter->fc - 2 * filter->rho + 2) / D;
  float C = (2 * filter->rho + filter->dt * filter->fc - 2) / D;
  float out = A * filter->yk + B * filter->uk1 + C * filter->uk;
  filter->yk1 = out;
  //return out;
}

struct Oneloop_notch_t{
  struct notch_axis_t roll;
  struct notch_axis_t pitch;
  struct notch_axis_t yaw;
};


/* Structural Modes Filtering*/
struct Oneloop_StructuralModes_t {
  float freq;
  float bandwidth;
  enum FilterType filter_type; // Type of filter used for the structural modes
  union {
    Butterworth2LowPass bw2;
    Butterworth4LowPass bw4;
    struct SecondOrderNotchFilter notch;
  } feedback_filter;
    union {
    Butterworth2LowPass bw2;
    Butterworth4LowPass bw4;
    struct SecondOrderNotchFilter notch;
  } model_filter;
};

extern bool drop_yaw;
extern int16_t temp_pitch;
/*Declaration of Reference Model and Error Controller Gains*/
extern struct PolePlacement p_att_e;
extern struct PolePlacement p_att_rm;
/*Position Loop*/
extern struct PolePlacement p_pos_e;
extern struct PolePlacement p_pos_rm;
/*Altitude Loop*/
extern struct PolePlacement p_alt_e;
extern struct PolePlacement p_alt_rm;
/*Heading Loop*/
extern struct PolePlacement p_head_e;
extern struct PolePlacement p_head_rm;
/*Gains of EC and RM*/
extern struct Gains3rdOrder k_att_e;
extern struct Gains3rdOrder k_att_rm;
extern struct Gains2ndOrder k_head_e;
extern struct Gains2ndOrder k_head_rm;
extern struct Gains3rdOrder k_pos_e;
extern struct Gains3rdOrder k_pos_rm;
extern void oneloop_andi_init(void);
extern void oneloop_andi_enter(bool half_loop_sp, int ctrl_type);
extern void oneloop_andi_set_failsafe_setpoint(void);
extern void oneloop_andi_run(bool in_flight, bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v);
extern void oneloop_andi_RM(bool half_loop, struct FloatVect3 PSA_des, int rm_order_h, int rm_order_v, bool in_flight_oneloop);
extern void oneloop_andi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);
extern void oneloop_from_nav(bool in_flight);
extern void guidance_set_min_max_airspeed(float min_airspeed, float max_airspeed);
#endif  // ONELOOP_ANDI_H
