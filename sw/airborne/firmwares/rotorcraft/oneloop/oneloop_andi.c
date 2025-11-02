/*
 * Copyright (C) 2025 Justin Dubois <j.p.g.dubois@student.tudelft.nl>
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

/* Include necessary header files */
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "math/wls/wls_alloc.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/rotwing_drone/rotwing_state.h"
#include "modules/core/commands.h"
#include "firmwares/rotorcraft/oneloop/cyclone_fu.h"

#include <stdio.h>
#if INS_EXT_POSE
#include "modules/ins/ins_ext_pose.h"
#endif

/*Define general struct of the Oneloop ANDI controller*/
struct OneloopGeneral oneloop_andi;

// Use compile-time macros to set default filter type enum value
// #if defined(FILTER_USE_LP1)
// enum FilterType filter_type = LOWPASS_1;
// #elif defined(FILTER_USE_BW2)
// enum FilterType filter_type = BUTTERWORTH_2;
// #elif defined(FILTER_USE_YAW_LP4)
// enum FilterType filter_type = BUTTERWORTH_4;
// #else
// #error "Filter type not defined. Define one of: USE_LP1, USE_BW2, USE_YAW_LP4."
// #endif

#ifdef ONELOOP_ANDI_ACT_IS_SERVO
const bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_IS_SERVO;
#else
const bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = {0};
#endif

#ifdef ONELOOP_ANDI_ACT_DYN
float  actuator_dynamics[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_DYN;
#else
#error "You must specify the actuator dynamics"
#endif

#if defined(ONELOOP_ANDI_ACT_MAX) && defined(ONELOOP_ANDI_ACT_MIN)
const float act_max[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX;
const float act_min[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN;
#else
#error "You must specify the actuator limits: ONELOOP_ANDI_ACT_MAX and ONELOOP_ANDI_ACT_MIN"
#endif

#if defined(ONELOOP_ANDI_ACT_RATE_MAX) && defined(ONELOOP_ANDI_ACT_RATE_MIN)
const float act_rate_max[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_RATE_MAX;
const float act_rate_min[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_RATE_MIN;
#else
#error "You must specify the actuator limits: ONELOOP_ANDI_ACT_RATE_MAX and ONELOOP_ANDI_ACT_RATE_MIN"
#endif

// #ifdef ONELOOP_ANDI_U_PREF
// static float u_pref[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_U_PREF;
// #else
// static float u_pref[ANDI_NUM_ACT_TOT] = {0.0};
// #endif

#if ANDI_NUM_ACT_TOT != WLS_N_U_MAX
#error Matrix-WLS_N_U_MAX is not equal to the number of actuators: define WLS_N_U_MAX == ANDI_NUM_ACT_TOT in airframe file
#endif
#if ANDI_OUTPUTS != WLS_N_V_MAX
#error Matrix-WLS_N_V_MAX is not equal to the number of controlled axis: define WLS_N_V_MAX == ANDI_OUTPUTS in airframe file
#endif

/*  Define Section of the functions used in this module*/
static float positive_non_zero(float input);
static void scaled_error_nd(uint_fast8_t n, float err[restrict n], const float a[static n], const float b[static n], const float k[static n]);
static void integrate_nd(uint_fast8_t n, float a[static n], const float a_dot[static n], float dt);

static void reference_model_rate(float dt, float rate_des[3], const struct Gains2ndOrder3 *k_rate_rm, const struct OneloopAttRef *bounds, struct OneloopAttRef *rate_ref);
static void reference_model_attitude(float dt, float att_des[3], const struct Gains3rdOrder3 *k_att_rm, const struct OneloopAttRef *bounds, struct OneloopAttRef *att_ref);
static void reference_model_position(float dt, float pos_des[2], const struct Gains3rdOrder2 *k_pos_rm, const struct OneloopPosRef *bounds, struct OneloopPosRef *pos_ref);
static void reference_model_altitude(float dt, float alt_des, const struct Gains3rdOrder1 *k_alt_rm, const struct OneloopAltRef *bounds, struct OneloopAltRef *alt_ref);
static void reference_model_heading(float dt, float head_des, const struct Gains2ndOrder1 *k_head_rm, const struct OneloopHeadRef *bounds, struct OneloopHeadRef *head_ref);

static void error_controller_rate(const struct OneloopAttRef *rate_ref, const struct OneloopAttState *rate_state, const struct Gains2ndOrder3 *k_rate_e, float *nu_rate);
static void error_controller_attitude(const struct OneloopAttRef *att_ref, const struct OneloopAttState *att_state, const struct Gains3rdOrder3 *k_att_e, float *nu_att);
static void error_controller_position(const struct OneloopPosRef *pos_ref, const struct OneloopPosState *pos_state, const struct Gains3rdOrder2 *k_pos_e, float *nu_pos);
static void error_controller_altitude(const struct OneloopAltRef *alt_ref, const struct OneloopAltState *alt_state, const struct Gains3rdOrder1 *k_alt_e, float *nu_alt);
static void error_controller_heading(const struct OneloopHeadRef *head_ref, const struct OneloopHeadState *head_state, const struct Gains2ndOrder1 *k_head_e, float *nu_head);


static void compute_gains_2nd_order_single(float* k1, float* k2, float omega_n, float zeta);
static void compute_gains_3rd_order_single(float* k1, float* k2, float* k3, float omega_n, float zeta, float p1);
static void compute_gains_2nd_order_3(struct Gains2ndOrder3* gains, const struct Poles2ndOrder3* poles);
static void compute_gains_2nd_order_2(struct Gains2ndOrder2* gains, const struct Poles2ndOrder2* poles);
static void compute_gains_2nd_order_1(struct Gains2ndOrder1* gains, const struct Poles2ndOrder1* poles);
static void compute_gains_3rd_order_3(struct Gains3rdOrder3* gains, const struct Poles3rdOrder3* poles);
static void compute_gains_3rd_order_2(struct Gains3rdOrder2* gains, const struct Poles3rdOrder2* poles);
static void compute_gains_3rd_order_1(struct Gains3rdOrder1* gains, const struct Poles3rdOrder1* poles);

static void  init_filter(struct Filter *filer, float fc, enum FilterType type);
static void  init_filter_on_type(struct Filter *filter, float x0);
static void  update_filter_on_type(struct Filter *filter, float input);
static void  oneloop_andi_propagate_filters(void);

static void get_desired_rates_radio_command(float rate_des[3], const float rate_bounds[3]);
static void get_desired_thrust_radio_command(float *thrust_des, float thrust_bound_min, float thrust_bound_max);
static void get_desired_attitude_radio_command(float att_des[3], const float att_bounds[2], float heading_rate_bound, float heading_ref, float dt);
static void get_desired_position_radio_command(float pos_des[2], const float pos_rate_bound[2], const float pos_ref[2], float dt);
static void get_desired_altitude_radio_command(float* alt_des, float alt_rate_bound, float alt_ref, float dt);
static void get_desired_heading_radio_command(float* heading_des, float heading_rate_bound, float heading_ref, float dt);

static void get_act_state_oneloop(void);

static void compute_wls_scaling_factors(float wls_scaler_u[ANDI_NUM_ACT_TOT], const float act_rate_max[ANDI_NUM_ACT_TOT], const float act_rate_min[ANDI_NUM_ACT_TOT]);
static void compute_wls_upper_bounds(float u_d_max[ANDI_NUM_ACT_TOT], const float act_state[ANDI_NUM_ACT_TOT], const float act_max[ANDI_NUM_ACT_TOT], const float act_rate_max[ANDI_NUM_ACT_TOT], float dt);
static void compute_wls_lower_bounds(float u_d_min[ANDI_NUM_ACT_TOT], const float act_state[ANDI_NUM_ACT_TOT], const float act_min[ANDI_NUM_ACT_TOT], const float act_rate_min[ANDI_NUM_ACT_TOT], float dt);

static void evaluate_effectiveness_matrix_oneloop(float eff_mat[ANDI_OUTPUTS * ANDI_NUM_ACT_TOT]);

static abi_event actuators_t4_in_event;
static void actuators_t4_in_callback(uint8_t sender_id, struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr);

/**
 * @brief Controller poles
 *
 * These structures define pole placement parameters for different control modes.
 * They are used for real-time tuning of stabilization, position, altitude, 
 * and heading control loops.
 */
// FIXME: Make poles dynamically set from airframe file
struct Poles2ndOrder3 p_rate_e = {.omega_n={15.0, 7.5, 7.5}, .zeta={1.0, 1.0, 1.0}};
struct Poles2ndOrder3 p_rate_rm = {.omega_n={12.0, 6.0, 6.0}, .zeta={1.0, 1.0, 1.0}};
struct Poles3rdOrder3 p_att_e = {.omega_n={15.0, 7.5, 7.5}, .zeta={1.0, 1.0, 1.0}, .p1={15.0, 7.5, 5.5}};
struct Poles3rdOrder3 p_att_rm = {.omega_n={12.0, 6.0, 6.0}, .zeta={1.0, 1.0, 1.0}, .p1={12.0, 6.0, 6.0}};
struct Poles3rdOrder2 p_pos_e = {.omega_n={1.0, 1.0}, .zeta={1.0, 1.0}, .p1={1.0, 1.0}};
struct Poles3rdOrder2 p_pos_rm = {.omega_n={0.8, 0.8}, .zeta={1.0, 1.0}, .p1={0.8, 0.8}};
struct Poles3rdOrder1 p_alt_e = {.omega_n=1.0, .zeta=1.0, .p1=1.0};
struct Poles3rdOrder1 p_alt_rm = {.omega_n=0.8, .zeta=1.0, .p1=0.8};
struct Poles2ndOrder1 p_head_e = {.omega_n=0.5, .zeta=1.0};
struct Poles2ndOrder1 p_head_rm = {.omega_n=0.5, .zeta=1.0};

/** @brief Reference bounds
 *
 * These structures define the maximum allowable reference values for
 * stabilization, position, altitude, and heading control loops.
 * They are used to limit the reference commands within safe operational limits.
 */
struct OneloopAttRef att_bounds = {.att={2, 2, 2}, .att_d={10.0, 10.0, 10.0}, .att_2d={1000.0, 1000.0, 1000.0}, .att_3d={1000.0, 1000.0, 1000.0}};
struct OneloopPosRef pos_bounds = {.pos={0, 0}, .vel={1000.0, 1000.0}, .acc={1000.0, 1000.0}, .jer={1000.0, 1000.0}};
struct OneloopAltRef alt_bounds = {.pos=0, .vel=1000.0, .acc=1000.0, .jer=1000.0};
struct OneloopHeadRef head_bounds = {.head=0, .head_d=1000.0, .head_2d=1000.0};

struct OneloopAttRef attitude_ref;
struct OneloopAttState attitude_state;
struct OneloopThrustRef thrust_ref;


/**
 * @brief Model coefficients 
 */
union CycloneCoefficients obm_coefficients = {
  .fx_motor_squared       = 0.00000735f,
  .fx_speed_forward       = -0.03f,

  .fy_speed_lateral       = -0.008f,

  .fz_motor_squared       = 0.0f,
  .fz_speed_forward       = 0.0f,
  .fz_speed_vertical      = -0.144f,
  .fz_elevator_speed      = 0.0f,
  .fz_elevator_motor      = 0.0f,

  .mx_motor_diff          = 0.0f,
  .mx_elevator_motor_diff = 0.0000283f,
  .mx_elevator_speed_diff = 0.344f,
  .mx_angular_coupling    = -2.18f,

  .my_speed_forward       = 0.0f,
  .my_speed_vertical      = -0.0888f,
  .my_constant_zero       = -1.032f,
  .my_motor_sum           = 0.0f,
  .my_elevator_motor_sum  = -0.0000424f,
  .my_elevator_speed_sum  = -0.2525f,
  .my_angular_sum         = 1.262f,

  .mz_speed_lateral       = -0.00371f,
  .mz_motor_diff          = 0.000039f,
  .mz_speed_roll          = -0.0129f,
  .mz_angular_coupling    = -0.4827f
};

/** 
 * @brief Controller gains
 *
 * These global variables store the controller gains derived from the
 * previously defined pole placement parameters.
 *
 * The values are computed from the designed poles (p_att_*, p_pos_*, etc.)
 * and directly used by the flight control laws in the real-time control loop.
 * Updating the poles during tuning automatically affects these gain values.
 */
struct Gains2ndOrder3 k_rate_e;
struct Gains2ndOrder3 k_rate_rm;
struct Gains3rdOrder3 k_att_e;
struct Gains3rdOrder3 k_att_rm;
struct Gains3rdOrder2 k_pos_e;
struct Gains3rdOrder2 k_pos_rm;
struct Gains3rdOrder1 k_alt_e;
struct Gains3rdOrder1 k_alt_rm;
struct Gains2ndOrder1 k_head_e;
struct Gains2ndOrder1 k_head_rm;

static struct Filter filt_p;
static struct Filter filt_q;
static struct Filter filt_r;
static struct Filter filt_p_dot;
static struct Filter filt_q_dot;
static struct Filter filt_r_dot;
static struct Filter filt_an;
static struct Filter filt_ae;
static struct Filter filt_ad;
static struct Filter filt_vn;
static struct Filter filt_ve;
static struct Filter filt_vd;

static struct Filter filt_ay;
static struct Filter filt_airspeed;
static struct Filter filt_u[ANDI_NUM_ACT];  // Low pass filter for real actuators        

/* Oneloop Misc variables*/
static float dt_1l = 1. / PERIODIC_FREQUENCY;
// static float g   = 9.81; // [m/s^2] Gravitational Acceleration

/* Oneloop Control Variables*/
float andi_u[ANDI_NUM_ACT_TOT];
float andi_du[ANDI_NUM_ACT_TOT];
float actuator_state_1l[ANDI_NUM_ACT_TOT];
float actuator_obs[ANDI_NUM_ACT];  // observed actuator states, updated by abi callback

float wls_wv_rate[ANDI_OUTPUTS]     = {0.0f, 0.0f, 4.0f, 0.0f, 8.0f, 8.0f, 8.00f};
float wls_wv_attitude[ANDI_OUTPUTS] = {0.0f, 0.0f, 4.0f, 0.0f, 8.0f, 8.0f, 8.00f};
float wls_wv_guidance[ANDI_OUTPUTS] = {4.0f, 4.0f, 4.0f, 1.0f, 8.0f, 8.0f, 8.00f};

float wls_scaler_u[ANDI_NUM_ACT_TOT];

/*WLS Settings*/
struct WLS_t wls_one_p = {
  .nu        = ANDI_NUM_ACT_TOT,
  .nv        = ANDI_OUTPUTS,
  .gamma_sq  = 1000.0,
  .v         = {0.0},
#ifdef ONELOOP_ANDI_WV // {ax_dot,ay_dot,az_dot,psi_dot,p_ddot,q_ddot,r_ddot}  
  .Wv        = ONELOOP_ANDI_WV,
#else
  .Wv        = {[0 ... ANDI_OUTPUTS-1] = 1.0},
#endif
#ifdef ONELOOP_ANDI_WU // {eL,eR,mL,mR,p,q,r}  
  .Wu        = ONELOOP_ANDI_WU,
#else
  .Wu        = {[0 ... ANDI_NUM_ACT_TOT-1] = 1.0},
#endif
  .u_pref    = {0.0},
  .u_min     = ONELOOP_ANDI_ACT_RATE_MIN,
  .u_max     = ONELOOP_ANDI_ACT_RATE_MAX,
  .PC        = 0.0,
  .SC        = 0.0,
  .iter      = 0
};

/* Effectiveness Matrix definition */
float *bwls_1l[ANDI_OUTPUTS];
float eff_mat[ANDI_OUTPUTS * ANDI_NUM_ACT_TOT];
// float eff_mat_stab[ANDI_OUTPUTS * ANDI_NUM_ACT];
float n_array[ANDI_OUTPUTS];
float m_array[ANDI_NUM_ACT_TOT];
float coupling_factor[ANDI_OUTPUTS];

/* Define messages of the module*/
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_wls_v_oneloop(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_v("one", &wls_one_p, trans, dev); 
}
static void send_wls_u_oneloop(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_u("one", &wls_one_p, trans, dev); 
}

static void send_eff_mat_one_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_EFF_MAT_ONE(trans, dev, AC_ID,
                            ANDI_NUM_ACT_TOT, &eff_mat[0 * ANDI_NUM_ACT_TOT],
                            ANDI_NUM_ACT_TOT, &eff_mat[1 * ANDI_NUM_ACT_TOT],
                            ANDI_NUM_ACT_TOT, &eff_mat[2 * ANDI_NUM_ACT_TOT],
                            ANDI_NUM_ACT_TOT, &eff_mat[3 * ANDI_NUM_ACT_TOT],
                            ANDI_NUM_ACT_TOT, &eff_mat[4 * ANDI_NUM_ACT_TOT],
                            ANDI_NUM_ACT_TOT, &eff_mat[5 * ANDI_NUM_ACT_TOT],
                            ANDI_NUM_ACT_TOT, &eff_mat[6 * ANDI_NUM_ACT_TOT]);
}

static void send_eff_mat_stab_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0f;
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID, 
                ANDI_NUM_ACT_TOT, &eff_mat[4 * ANDI_NUM_ACT_TOT],
                ANDI_NUM_ACT_TOT, &eff_mat[5 * ANDI_NUM_ACT_TOT],
                ANDI_NUM_ACT_TOT, &eff_mat[6 * ANDI_NUM_ACT_TOT], 
                           1,      &zero,
                           1,      &zero);
}

static void send_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                                        3, oneloop_andi.att_des,
                                        3, oneloop_andi.att_state.att,
                                        3, oneloop_andi.att_ref.att,
                                        3, oneloop_andi.att_state.att_d,
                                        3, oneloop_andi.att_ref.att_d,                                       
                                        3, oneloop_andi.att_state.att_2d,
                                        3, oneloop_andi.att_ref.att_2d,
                                        3, oneloop_andi.att_ref.att_3d,                                       
                                        ANDI_NUM_ACT, actuator_state_1l);                                      
}

static void send_guidance_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE(trans, dev, AC_ID,
                                        &oneloop_andi.pos_ref.pos[0],
                                        &oneloop_andi.pos_ref.pos[1],
                                        &oneloop_andi.alt_ref.pos,
                                        &oneloop_andi.pos_state.pos[0],
                                        &oneloop_andi.pos_state.pos[1],
                                        &oneloop_andi.alt_state.pos,
                                        &oneloop_andi.pos_ref.vel[0],
                                        &oneloop_andi.pos_ref.vel[1],
                                        &oneloop_andi.alt_ref.vel,
                                        &oneloop_andi.pos_state.vel[0],
                                        &oneloop_andi.pos_state.vel[1],
                                        &oneloop_andi.alt_state.vel,
                                        &oneloop_andi.pos_ref.acc[0],
                                        &oneloop_andi.pos_ref.acc[1],
                                        &oneloop_andi.alt_ref.acc,
                                        &oneloop_andi.pos_state.acc[0],
                                        &oneloop_andi.pos_state.acc[1],
                                        &oneloop_andi.alt_state.acc,
                                        &oneloop_andi.pos_ref.jer[0],
                                        &oneloop_andi.pos_ref.jer[1],
                                        &oneloop_andi.alt_ref.jer);
}

// static void debug_vect(struct transport_tx *trans, struct link_device *dev, char *name, float *data, int datasize)
// {
//   pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID,
//                            strlen(name), name,
//                            datasize, data);
// }

// static void send_oneloop_debug(struct transport_tx *trans, struct link_device *dev)
// {
//   float temp_debug_vect[14];
//   temp_debug_vect[0] = (float) chirp_n_counter;
//   temp_debug_vect[1] = oneloop_andi_model[1];
//   temp_debug_vect[2] = oneloop_andi_model[2];
//   temp_debug_vect[3] = oneloop_andi_model[3];
//   temp_debug_vect[4] = oneloop_andi_model[4];
//   temp_debug_vect[5] = oneloop_andi_model[5];
//   temp_debug_vect[6] = oneloop_andi_sigma;
//   temp_debug_vect[7] = chirp_on ? 1.0 : 0.0;
//   temp_debug_vect[8] = filt_ad.meas;
//   temp_debug_vect[9] = temp_checks_2[0];
//   temp_debug_vect[10] = temp_checks_2[1];
//   temp_debug_vect[11] = temp_ref_att[0];
//   temp_debug_vect[12] = temp_ref_att[1];
//   temp_debug_vect[13] = temp_ref_att[2];
//   debug_vect(trans, dev, "APF", temp_debug_vect, 14);
// }

#endif //PERIODIC_TELEMETRY



/** @brief Function to make sure that inputs are positive non zero vaues
 * 
 * If the input is zero or negative, it returns a small positive value.
 * @param input The float input value to check.
 * @return The original input if positive and non-zero, otherwise a small positive value.
 */
static float positive_non_zero(float input)
{
  if (input < FLT_EPSILON) {
    input = 0.00001; // FIXME: Remove magic number
  }
  return input;
}

/**
 * @brief Compute scaled error between two N-dimensional vectors.
 *
 * This function calculates the elementwise difference between two input arrays
 * and scales each component by the corresponding gain factor.
 *
 * @param n   Dimension of the arrays
 * @param err Output array for the computed scaled error [n]
 * @param a   First input array (reference or desired values) [n]
 * @param b   Second input array (measured or actual values) [n]
 * @param k   Scaling gains applied elementwise to the error [n]
 */
static void scaled_error_nd(uint_fast8_t n, float err[restrict n], const float a[static n], const float b[static n], const float k[static n])
{
  for (uint_fast8_t i = 0; i < (uint_fast8_t)n; i++) {
    err[i] = k[i] * (a[i] - b[i]);
  }
}

/**
 * @brief Integrate an N-dimensional vector forward in time.
 *
 * This function performs a simple discrete-time integration step for an input vector,
 * updating each element based on its time derivative and the time step size.
 *
 * The integration assumes a forward Euler method:
 *     a[i](t + dt) = a[i](t) + dt * a_dot[i]
 *
 * @param dt      Integration time step [s]
 * @param a       Input/output array containing the integrated state values [n]
 * @param a_dot   Input array containing the time derivatives of each state [n]
 * @param n       Dimension of the arrays
 */
static void integrate_nd(uint_fast8_t n, float a[static n], const float a_dot[static n], float dt)
{
  for (uint_fast8_t i = 0; i < n; i++) {
    a[i] = a[i] + dt * a_dot[i];
  }
}


/**
 * @brief Generates a bounded second-order reference signal for attitude rate control.
 *
 * This function computes smooth angular rate, acceleration, and jerk references based on the desired rates,
 * applying bounded limits to ensure stability and safe dynamic behavior. It updates the reference states 
 * in place within the provided `att_ref` structure, which is a quaternion-based reference with associated rate states.
 *
 * @param[in] dt         The sampling interval in seconds.
 * @param[in] rate_des   Desired angular rates (p, q, r) as a `FloatRates` structure.
 * @param[in] k_rate_rm  Gain parameters as `GainsOrder2Vect3`, for the proportional and derivative terms.
 * @param[in] bounds     Limits for the desired rates, rates derivatives, and accelerations, in `OneloopAttRefEulers`.
 * @param[in,out] att_ref  The reference model states (attitude quaternion, rates, and derivatives),
 *                         updated in place to produce the reference command.
 */
static void generate_reference_rate(
  float dt,
  struct FloatRates rate_des,
  const struct GainsOrder2Vect3 *k_rate_rm,
  const struct OneloopAttRefQuat *bounds,
  struct OneloopAttRefQuat *att_ref)
{
    float p_des = rate_des.p;
    float q_des = rate_des.q;
    float r_des = rate_des.r;

    BoundAbs(p_des, bounds->att_d.p);
    BoundAbs(q_des, bounds->att_d.q);
    BoundAbs(r_des, bounds->att_d.r);

    float p_d_des = k_rate_rm->k1.x * (p_des - att_ref->att_d.p);
    float q_d_des = k_rate_rm->k1.y * (q_des - att_ref->att_d.q);
    float r_d_des = k_rate_rm->k1.z * (r_des - att_ref->att_d.r);

    BoundAbs(p_d_des, bounds->att_2d.x);
    BoundAbs(q_d_des, bounds->att_2d.y);
    BoundAbs(r_d_des, bounds->att_2d.z);

    float p_2d_des = k_rate_rm->k2.x * (p_d_des - att_ref->att_2d.x);
    float q_2d_des = k_rate_rm->k2.y * (q_d_des - att_ref->att_2d.y);
    float r_2d_des = k_rate_rm->k2.z * (r_d_des - att_ref->att_2d.z);

    BoundAbs(p_2d_des, bounds->att_3d.x);
    BoundAbs(q_2d_des, bounds->att_3d.y);
    BoundAbs(r_2d_des, bounds->att_3d.z);

    att_ref->att_3d.x = p_2d_des;
    att_ref->att_3d.y = q_2d_des;
    att_ref->att_3d.z = r_2d_des;

    att_ref->att_2d.x += p_2d_des * dt;
    att_ref->att_2d.y += q_2d_des * dt;
    att_ref->att_2d.z += r_2d_des * dt;

    att_ref->att_d.p += att_ref->att_2d.x * dt;
    att_ref->att_d.q += att_ref->att_2d.y * dt;
    att_ref->att_d.r += att_ref->att_2d.z * dt;

    float_quat_identity(&att_ref->att); // zero the quaternion attitude reference
}

/**
 * @brief Generates a bounded second-order reference signal for quaternion-based attitude control.
 *
 * Computes smooth angular rate, acceleration, and jerk references using quaternion error feedback.
 * Bounds are applied to limit reference values and maintain system stability. The function updates
 * the reference states in place within the provided `att_ref` structure, which includes quaternion attitude
 * and associated rate states.
 *
 * @param[in] dt         Sampling interval in seconds.
 * @param[in] att_des    Desired attitude as a unit quaternion.
 * @param[in] k_att_rm   Gain parameters (proportional and derivative gains for rate control).
 * @param[in] bounds     Limits on rate references and derivatives (angular velocity and higher).
 * @param[in,out] att_ref Reference model state containing attitude quaternion and rate state, updated in place.
 */
static void generate_reference_attitude(
  float dt,
  struct FloatQuat att_des,
  const struct GainsOrder3Vect3 *k_att_rm,
  const struct OneloopAttRefQuat *bounds,
  struct OneloopAttRefQuat *att_ref)
{
    struct FloatQuat att_err;
    float_quat_inv_comp_norm_shortest(&att_err, &att_ref->att, &att_des);

    float p_des = k_att_rm->k1.x * att_err.qx;
    float q_des = k_att_rm->k1.y * att_err.qy;
    float r_des = k_att_rm->k1.z * att_err.qz;

    BoundAbs(p_des, bounds->att_d.p);
    BoundAbs(q_des, bounds->att_d.q);
    BoundAbs(r_des, bounds->att_d.r);

    float p_d_des = k_att_rm->k2.x * (p_des - att_ref->att_d.p);
    float q_d_des = k_att_rm->k2.y * (q_des - att_ref->att_d.q);
    float r_d_des = k_att_rm->k2.z * (r_des - att_ref->att_d.r);

    BoundAbs(p_d_des, bounds->att_2d.x);
    BoundAbs(q_d_des, bounds->att_2d.y);
    BoundAbs(r_d_des, bounds->att_2d.z);

    float p_2d_des = k_att_rm->k3.x * (p_d_des - att_ref->att_2d.x);
    float q_2d_des = k_att_rm->k3.y * (q_d_des - att_ref->att_2d.y);
    float r_2d_des = k_att_rm->k3.z * (r_d_des - att_ref->att_2d.z);

    BoundAbs(p_2d_des, bounds->att_3d.x);
    BoundAbs(q_2d_des, bounds->att_3d.y);
    BoundAbs(r_2d_des, bounds->att_3d.z);

    att_ref->att_3d.x = p_2d_des;
    att_ref->att_3d.y = q_2d_des;
    att_ref->att_3d.z = r_2d_des;

    att_ref->att_2d.x += p_2d_des * dt;
    att_ref->att_2d.y += q_2d_des * dt;
    att_ref->att_2d.z += r_2d_des * dt;

    att_ref->att_d.p += att_ref->att_2d.x * dt;
    att_ref->att_d.q += att_ref->att_2d.y * dt;
    att_ref->att_d.r += att_ref->att_2d.z * dt;

    float_quat_integrate(&att_ref->att, &att_ref->att_d, dt);
}

/**
 * @brief Generates a bounded second-order reference signal for thrust control.
 *
 * Applies bounded limits to a desired thrust input and computes smoothed
 * thrust rate and integrated thrust references. The reference states are
 * updated in place inside the provided \p thrust_ref structure.
 *
 * @param[in] dt           Sampling interval in seconds.
 * @param[in] thrust_des   Desired thrust input value.
 * @param[in] k_thrust_rm  Gain parameter for thrust rate control (proportional gain).
 * @param[in] bounds       Maximum allowable thrust magnitude.
 * @param[in,out] att_ref  Pointer to OneloopThrustRef struct holding thrust reference states.
 */
static void generate_reference_thrust(
  float dt,
  float thrust_des,
  const float k_thrust_rm,
  const float bounds,
  struct OneloopThrustRef *thrust_ref)
{
  BoundAbs(thrust_des, bounds);
  thrust_ref->thrust_d = k_thrust_rm * (thrust_des - thrust_ref->thrust);
  thrust_ref->thrust += thrust_ref->thrust_d * dt;
}



/**
 * @brief Computes the control error command for attitude rate regulation.
 *
 * This function calculates the virtual control input vector \c nu required to reduce the error
 * between the reference and current angular rate states. It uses proportional and derivative gains
 * on the differences of rate and rate derivative components, respectively, and adds feedforward jerk.
 *
 * @param[in] att_ref   Pointer to the reference attitude and rate state structure.
 * @param[in] att_state Pointer to the current attitude and rate state structure.
 * @param[in] k_rate_e  Pointer to gain parameters structure, containing proportional and derivative gains.
 *
 * @return A FloatVect3 structure representing the computed virtual control input vector.
 */
static struct FloatVect3 control_error_rate(
  const struct OneloopAttRefQuat *att_ref,
  const struct OneloopAttStateQuat *att_state,
  const struct GainsOrder2Vect3 *k_rate_e)
{
  struct FloatVect3 nu = att_ref->att_3d;

  nu.x += k_rate_e->k2.x * (att_ref->att_2d.x - att_state->att_2d.x);
  nu.y += k_rate_e->k2.y * (att_ref->att_2d.y - att_state->att_2d.y);
  nu.z += k_rate_e->k2.z * (att_ref->att_2d.z - att_state->att_2d.z);

  nu.x += k_rate_e->k1.x * (att_ref->att_d.p - att_state->att_d.p);
  nu.y += k_rate_e->k1.y * (att_ref->att_d.q - att_state->att_d.q);
  nu.z += k_rate_e->k1.z * (att_ref->att_d.r - att_state->att_d.r);

  return nu;
}

/**
 * @brief Computes the attitude rate error control command.
 *
 * This function calculates the control input vector \c nu to correct the attitude error and
 * the associated angular rate errors using proportional-derivative gains. The quaternion error
 * between the reference and current attitudes is also factored in the control law.
 *
 * @param[in] att_ref   Pointer to the reference attitude and rate states (quaternion-based).
 * @param[in] att_state Pointer to the current attitude and rate states.
 * @param[in] k_att_e   Pointer to gain parameters struct containing proportional, derivative, and jerk gains.
 *
 * @return A \c FloatVect3 struct representing the computed attitude error control command vector.
 */
static struct FloatVect3 control_error_attitude(
  const struct OneloopAttRefQuat *att_ref,
  const struct OneloopAttStateQuat *att_state,
  const struct GainsOrder3Vect3 *k_att_e)
{
  struct FloatVect3 nu = att_ref->att_3d;

  nu.x += k_att_e->k3.x * (att_ref->att_2d.x - att_state->att_2d.x);
  nu.y += k_att_e->k3.y * (att_ref->att_2d.y - att_state->att_2d.y);
  nu.z += k_att_e->k3.z * (att_ref->att_2d.z - att_state->att_2d.z);

  nu.x += k_att_e->k2.x * (att_ref->att_d.p - att_state->att_d.p);
  nu.y += k_att_e->k2.y * (att_ref->att_d.q - att_state->att_d.q);
  nu.z += k_att_e->k2.z * (att_ref->att_d.r - att_state->att_d.r);

  struct FloatQuat att_err;
  float_quat_inv_comp_norm_shortest(&att_err, &att_ref->att, &att_state->att);
  nu.x += k_att_e->k1.x * att_err.qx;
  nu.y += k_att_e->k1.y * att_err.qy;
  nu.z += k_att_e->k1.z * att_err.qz;

  return nu;
}

/**
 * @brief Computes the thrust control command based on desired and current thrust.
 *
 * This function calculates a corrected thrust command using a simple proportional
 * feedback law. The correction term is scaled by the thrust error gain k_thrust_e
 * to reduce the difference between the desired thrust (thrust_ref->thrust)
 * and the current thrust (thrust_state). The resulting command is based on
 * the desired thrust feedforward input thrust_ref->thrust_d.
 *
 * @param thrust_ref Pointer to a structure containing the desired thrust values.
 * @param thrust_state Current measured thrust value.
 * @param k_thrust_e Proportional gain applied to the thrust error correction.
 *
 * @return The computed thrust control command.
 */
static float control_error_thrust(
  const struct OneloopThrustRef *thrust_ref,
  const float thrust_state,
  const float k_thrust_e)
{
  float nu = thrust_ref->thrust_d;

  nu += k_thrust_e * (thrust_ref->thrust - thrust_state);

  return nu;
}














/**
 * @brief 2nd-order reference model for rate control.
 *
 * Generates smooth angular rate, acceleration, and jerk references with bounded dynamics.
 * The attitude field is unused in rate control.
 *
 * @param[in] dt          Sample time [s]
 * @param[in] rate_des    Desired angular rates [rad/s]
 * @param[in] k_stb_rm    Pointer to reference model gain parameters
 * @param[in] bounds      Pointer to reference model limits
 * @param[in,out] att_ref Pointer to struct containing reference model states (in/out)
 */
static void reference_model_rate(
  float dt, 
  float rate_des[3], 
  const struct Gains2ndOrder3 *k_rate_rm,
  const struct OneloopAttRef *bounds,
  struct OneloopAttRef *att_ref)
{
  float rate_d_des[3];
  float rate_2d_des[3];

  for (uint_fast8_t i = 0; i < 3; i++)
    BoundAbs(rate_des[i], bounds->att_d[i]);

  // Angular rate error 
  scaled_error_nd(3, rate_d_des, rate_des, att_ref->att_d, k_rate_rm->k1);
  for (uint_fast8_t i = 0; i < 3; i++)
    BoundAbs(rate_d_des[i], bounds->att_2d[i]);

  // Angular acceleration error
  scaled_error_nd(3, rate_2d_des, rate_d_des, att_ref->att_2d, k_rate_rm->k2);
  for (uint_fast8_t i = 0; i < 3; i++)
    BoundAbs(rate_2d_des[i], bounds->att_3d[i]);

  // Reference propagation 
  float_vect_copy(att_ref->att_3d, rate_2d_des, 3);
  integrate_nd(3, att_ref->att_2d, rate_2d_des, dt);
  integrate_nd(3, att_ref->att_d, att_ref->att_2d, dt);
  float_vect_zero(att_ref->att, 3);
}

/**
 * @brief 3rd-order reference model for attitude control.
 *
 * Generates smooth desired attitude, angular rate, acceleration, and jerk references with
 * bouded dynamics.
 *
 * @param[in] dt          Sample time [s]
 * @param[in] att_des     Desired attitude command [rad]
 * @param[in] k_stb_rm    Pointer to reference model gain parameters
 * @param[in] bounds      Pointer to reference model limits
 * @param[in,out] att_ref Pointer to struct containing reference model states (in/out)
 */
static void reference_model_attitude(
  float dt, 
  float att_des[3],
  const struct Gains3rdOrder3 *k_att_rm,
  const struct OneloopAttRef *bounds,
  struct OneloopAttRef *att_ref)
{
  float att_d_des[3];
  float att_2d_des[3];
  float att_3d_des[3];

  // Limit input attitude commands
  for (uint_fast8_t i = 0; i < 3; i++)
    BoundAbs(att_des[i], bounds->att[i]);

  // Attitude tracking error → desired angular rate
  scaled_error_nd(3, att_d_des, att_des, att_ref->att, k_att_rm->k1);
  for (uint_fast8_t i = 0; i < 3; i++)
    BoundAbs(att_d_des[i], bounds->att_d[i]);

  // Angular rate tracking error → desired angular acceleration
  scaled_error_nd(3, att_2d_des, att_d_des, att_ref->att_d, k_att_rm->k2);
  for (uint_fast8_t i = 0; i < 3; i++)
    BoundAbs(att_2d_des[i], bounds->att_2d[i]);

  scaled_error_nd(3, att_3d_des, att_2d_des, att_ref->att_2d, k_att_rm->k3);
  for (uint_fast8_t i = 0; i < 3; i++)
    BoundAbs(att_3d_des[i], bounds->att_3d[i]);

  // Propagate references
  float_vect_copy(att_ref->att_3d, att_3d_des, 3);
  integrate_nd(3, att_ref->att_2d, att_3d_des, dt);
  integrate_nd(3, att_ref->att_d, att_ref->att_2d, dt);
  integrate_nd(3, att_ref->att, att_ref->att_d, dt);
}

/**
 * @brief Reference Model for 3rd-order position control loop.
 *
 * Generates smooth position, velocity, acceleration, and jerk references
 * for North and East axes using 3rd order gains.
 * Velocity, acceleration, and jerk are bounded; position bounds are not applied.
 *
 * @param[in] dt        Sample time [s]
 * @param[in] pos_des   Desired 2D position [m] (North and East)
 * @param[in] k_pos_rm  Pointer to reference model gain parameters for 2D position
 * @param[in] bounds    Pointer to reference model limits for velocity, acceleration, jerk
 * @param[in,out] pos_ref   Pointer to struct with reference states (pos, vel, accel)
 */
static void reference_model_position(
  float dt,
  float pos_des[2],
  const struct Gains3rdOrder2 *k_pos_rm,
  const struct OneloopPosRef *bounds,
  struct OneloopPosRef *pos_ref)
{
  float vel_des[2];
  float acc_des[2];
  float jer_des[2];

  // Position error
  scaled_error_nd(2, vel_des, pos_des, pos_ref->pos, k_pos_rm->k1);
  for (uint_fast8_t i = 0; i < 2; i++) {
    BoundAbs(vel_des[i], bounds->vel[i]);
  }

  // Velocity error 
  scaled_error_nd(2, acc_des, vel_des, pos_ref->vel, k_pos_rm->k2);
  for (uint_fast8_t i = 0; i < 2; i++) {
    BoundAbs(acc_des[i], bounds->acc[i]);
  }

  // Acceleration error
  scaled_error_nd(2, jer_des, acc_des, pos_ref->acc, k_pos_rm->k3);
  for (uint_fast8_t i = 0; i < 2; i++) {
    BoundAbs(jer_des[i], bounds->jer[i]);
  }

  // Reference propagation 
  float_vect_copy(pos_ref->jer, jer_des, 2);
  integrate_nd(2, pos_ref->acc, jer_des, dt);
  integrate_nd(2, pos_ref->vel, pos_ref->acc, dt);
  integrate_nd(2, pos_ref->pos, pos_ref->vel, dt);
}

/**
 * @brief Reference Model for 3rd-order altitude control loop.
 *
 * Generates smooth altitude, vertical velocity, acceleration, and jerk references
 * using 3rd order gains. Velocity, acceleration, and jerk are bounded; altitude bounds are not applied.
 *
 * @param[in] dt        Sample time [s]
 * @param[in] alt_des   Desired altitude [m]
 * @param[in] k_alt_rm  Pointer to reference model gains for altitude
 * @param[in] bounds    Pointer to reference model limits for vertical velocity, acceleration, jerk
 * @param[in,out] alt_ref   Pointer to struct with altitude reference states (alt, vel, accel, jerk)
 */
static void reference_model_altitude(
  float dt,
  float alt_des,
  const struct Gains3rdOrder1 *k_alt_rm,
  const struct OneloopAltRef *bounds,
  struct OneloopAltRef *alt_ref)
{
  float vel_des;
  float acc_des;
  float jer_des;

  // Altitude error scaled by k1
  vel_des = k_alt_rm->k1 * (alt_des - alt_ref->pos);
  BoundAbs(vel_des, bounds->vel);

  // Velocity error scaled by k2, bounded by velocity limits
  acc_des = k_alt_rm->k2 * (vel_des - alt_ref->vel);
  BoundAbs(acc_des, bounds->acc);

  // Acceleration error scaled by k3, bounded by accel limits
  jer_des = k_alt_rm->k3 * (acc_des - alt_ref->acc);
  BoundAbs(jer_des, bounds->jer);

  // Reference propagation
  alt_ref->jer = jer_des;
  integrate_nd(1, &alt_ref->acc, &jer_des, dt);
  integrate_nd(1, &alt_ref->vel, &alt_ref->acc, dt);
  integrate_nd(1, &alt_ref->pos, &alt_ref->vel, dt);
}

/**
 * @brief Reference Model for 2nd-order heading control loop.
 *
 * Generates smooth heading angle and angular rate references
 * using 2nd order gains. Angular rate and acceleration are bounded;
 * heading angle bounds are not applied, instead heading is wrapped to [-pi, pi].
 *
 * @param[in] dt        Sample time [s]
 * @param[in] head_des  Desired heading angle [rad]
 * @param[in] k_head_rm Pointer to reference model gains for heading
 * @param[in] bounds    Pointer to reference model limits for heading rate and yaw acceleration
 * @param[in,out] head_ref  Pointer to struct with heading reference states (angle, rate, accel)
 */
static void reference_model_heading(
  float dt,
  float head_des,
  const struct Gains2ndOrder1 *k_head_rm,
  const struct OneloopHeadRef *bounds,
  struct OneloopHeadRef *head_ref)
{
  float rate_des;
  float accel_des;

  // Heading error
  float head_err = head_des - head_ref->head;

  NormRadAngle(head_err);
  rate_des = k_head_rm->k1 * head_err;
  BoundAbs(rate_des, bounds->head_d);

  // Rate error
  accel_des = k_head_rm->k2 * (rate_des - head_ref->head_d);
  BoundAbs(accel_des, bounds->head_2d);

  // Reference propagation
  head_ref->head_2d = accel_des;
  integrate_nd(1, &head_ref->head_d, &accel_des, dt);
  integrate_nd(1, &head_ref->head, &head_ref->head_d, dt);

  // Normalize angle to [-pi, pi]
  NormRadAngle(head_ref->head);
}

/**
 * @brief Compute rate control error and generate virtual control input.
 *
 * Calculates scaled error vectors for angular rate and angular acceleration
 * between reference and measured states, using structured 2nd-order gains.
 * These errors are combined with the jerk reference to form a virtual control
 * command for rate stabilization. The attitude reference is not used for rate control.
 *
 * @param[in]  att_ref    Pointer to rate reference states (angular rate, acceleration, jerk)
 * @param[in]  att_state  Pointer to current rate states (angular rate, acceleration)
 * @param[in]  k_rate_e   Pointer to structured 2nd-order gains (k1, k2 per axis)
 * @param[out] nu_rate    Pointer to output virtual control command [3]
 */
static void error_controller_rate(
  const struct OneloopAttRef *att_ref,
  const struct OneloopAttState *att_state,
  const struct Gains2ndOrder3 *k_rate_e,
  float *nu_rate)
{
  float omega_err[3];
  float omega_d_err[3];

  scaled_error_nd(3, omega_err, att_ref->att_d, att_state->att_d, k_rate_e->k1);
  scaled_error_nd(3, omega_d_err, att_ref->att_2d, att_state->att_2d, k_rate_e->k2);

  // Compute virtual command
  for (uint_fast8_t i = 0; i < 3; i++) {
    nu_rate[i] = omega_err[i] + omega_d_err[i] + att_ref->att_3d[i];
  }
}

/**
 * @brief Compute attitude control error and generate virtual control input.
 *
 * FIXME: Change to quaternion representation.
 *
 * Calculates scaled error vectors for attitude, angular rate, and angular acceleration
 * between reference and measured states, using structured 2nd-order gains.
 * These errors are summed with the angular jerk reference to form a virtual control
 * command for attitude stabilization.
 *
 * @param[in]  att_ref    Pointer to attitude reference states (attitude, angular rate, acceleration, jerk)
 * @param[in]  att_state  Pointer to current attitude states (attitude, angular rate, acceleration)
 * @param[in]  k_att_e    Pointer to structured gains for error scaling (k1, k2, k3 per axis)
 * @param[out] nu_att     Pointer to output virtual control command vector [3]
 */
static void error_controller_attitude(
  const struct OneloopAttRef *att_ref,
  const struct OneloopAttState *att_state,
  const struct Gains3rdOrder3 *k_att_e,
  float *nu_att)
{
  float att_err[3];
  float att_d_err[3];
  float att_2d_err[3];

  scaled_error_nd(3, att_err, att_ref->att, att_state->att, k_att_e->k1);
  scaled_error_nd(3, att_d_err, att_ref->att_d, att_state->att_d, k_att_e->k2);
  scaled_error_nd(3, att_2d_err, att_ref->att_2d, att_state->att_2d, k_att_e->k3);

  // Compute virtual command
  for (uint_fast8_t i = 0; i < 3; i++) {
    nu_att[i] = att_err[i] + att_d_err[i] + att_2d_err[i] + att_ref->att_3d[i];
  }
}


/**
 * @brief Compute the position (North, East) control error and generate virtual command.
 *
 * This function calculates the error vectors for position, velocity, and acceleration
 * between the reference states and current states, scales each error component by the
 * corresponding 3rd-order structured gains, then sums these scaled errors together with
 * the position jerk reference to form the virtual command for lateral stabilization.
 *
 * @param[in]  pos_ref   Pointer to position reference states (pos, vel, accel, jerk)
 * @param[in]  pos_state Pointer to current position states (pos, vel, accel)
 * @param[in]  k_pos_e   Pointer to structured 3rd-order gains for error scaling (k1, k2, k3 per axis)
 * @param[out] nu_pos    Pointer to output virtual command vector [2]
 */
static void error_controller_position(
  const struct OneloopPosRef *pos_ref,
  const struct OneloopPosState *pos_state,
  const struct Gains3rdOrder2 *k_pos_e,
  float *nu_pos)
{
  float pos_err[2];
  float vel_err[2];
  float acc_err[2];

  scaled_error_nd(2, pos_err, pos_ref->pos, pos_state->pos, k_pos_e->k1);
  scaled_error_nd(2, vel_err, pos_ref->vel, pos_state->vel, k_pos_e->k2);
  scaled_error_nd(2, acc_err, pos_ref->acc, pos_state->acc, k_pos_e->k3);

  // Compute virtual command
  for (uint_fast8_t i = 0; i < 2; i++) {
    nu_pos[i] = pos_err[i] + vel_err[i] + acc_err[i] + pos_ref->jer[i];
  }
}

/**
 * @brief Compute the altitude control error and generate virtual control input.
 *
 * This function calculates the error scalars for altitude, vertical velocity,
 * and acceleration between the reference states and current states. Each error is scaled
 * by its corresponding gain factor. These scaled errors are summed together with the
 * jerk reference to form the virtual command for altitude stabilization.
 *
 * @param[in]  alt_ref   Pointer to altitude reference states (position, velocity, acceleration, jerk)
 * @param[in]  alt_state Pointer to current altitude states (position, velocity, acceleration)
 * @param[in]  k_alt_e   Pointer to scalar 3rd-order gains for error scaling (k1, k2, k3)
 * @param[out] nu_alt    Pointer to output virtual control input scalar
 */
static void error_controller_altitude(
  const struct OneloopAltRef *alt_ref,
  const struct OneloopAltState *alt_state,
  const struct Gains3rdOrder1 *k_alt_e,
  float *nu_alt)
{
  float alt_err;
  float vel_err;
  float acc_err;

  scaled_error_nd(1, &alt_err, &alt_ref->pos, &alt_state->pos, &k_alt_e->k1);
  scaled_error_nd(1, &vel_err, &alt_ref->vel, &alt_state->vel, &k_alt_e->k2);
  scaled_error_nd(1, &acc_err, &alt_ref->acc, &alt_state->acc, &k_alt_e->k3);

  // Compute virtual command
  *nu_alt = alt_err + vel_err + acc_err + alt_ref->jer;
}

/**
 * @brief Compute heading control error and generate virtual control input.
 *
 * This function calculates the scalar error for heading angle and angular rate 
 * between the reference states and current states. Each error is scaled by 
 * its corresponding 3rd-order gain. The scaled errors are summed with the 
 * angular acceleration (head_2d) feedforward reference to produce the virtual command output.
 *
 * @param[in]  head_ref   Pointer to heading reference states (head, head_d, head_2d).
 * @param[in]  head_state Pointer to current heading states (head, head_d).
 * @param[in]  k_head_e   Pointer to 3rd-order gains (k1 and k2 gains).
 * @param[out] nu_head    Pointer to output virtual control command (scalar).
 */
static void error_controller_heading(
  const struct OneloopHeadRef *head_ref,
  const struct OneloopHeadState *head_state,
  const struct Gains2ndOrder1 *k_head_e,
  float *nu_head)
{
  float head_err;
  float head_d_err;

  scaled_error_nd(1, &head_err, &head_ref->head, &head_state->head, &k_head_e->k1);
  scaled_error_nd(1, &head_d_err, &head_ref->head_d, &head_state->head_d, &k_head_e->k2);

  // Compute virtual command
  *nu_head = head_err + head_d_err + head_ref->head_2d;
}


/**
 * @brief Compute gain coefficients for one 2nd-order system
 * 
 * @param[out] k1     Computed k2 gain
 * @param[out] k2     Computed k3 gain
 * @param[in]  omega_n  Natural frequency
 * @param[in]  zeta     Damping ratio
 */
static void compute_gains_2nd_order_single(float* k1, float* k2, float omega_n, float zeta)
{
    *k1 = omega_n / (2.0f * zeta);
    *k2 = 2.0f * zeta * omega_n;
}

/**
 * @brief Compute gain coefficients for one 3rd-order system
 * 
 * @param[out] k1     Computed k1 gain
 * @param[out] k2     Computed k2 gain
 * @param[out] k3     Computed k3 gain
 * @param[in]  omega_n  Natural frequency
 * @param[in]  zeta     Damping ratio
 * @param[in]  p1       Additional pole parameter
 */
static void compute_gains_3rd_order_single(float* k1, float* k2, float* k3, float omega_n, float zeta, float p1)
{ 
    *k1 = (omega_n * omega_n * p1) / (omega_n * omega_n + 2.0f * zeta * omega_n * p1);
    *k2 = (omega_n * omega_n + 2.0f * zeta * omega_n * p1) / (2.0f * zeta * omega_n + p1);
    *k3 = 2.0f * zeta * omega_n + p1;
}

/**
 * @brief Compute 2nd order gains with 3 outputs from poles
 * 
 * @param[out] gains Output structure for computed gains
 * @param[in] poles Input pole structure (omega_n, zeta)
 */
static void compute_gains_2nd_order_3(struct Gains2ndOrder3* gains, const struct Poles2ndOrder3* poles)
{
    for (uint_fast8_t i = 0; i < 3; ++i) {
        compute_gains_2nd_order_single(&gains->k1[i], &gains->k2[i],
                                       poles->omega_n[i], poles->zeta[i]);
    }
}

/**
 * @brief Compute 2nd order gains with 2 outputs from poles
 * 
 * @param[out] gains Output structure for computed gains
 * @param[in] poles Input pole structure (omega_n, zeta)
 */
static void compute_gains_2nd_order_2(struct Gains2ndOrder2* gains, const struct Poles2ndOrder2* poles)
{
    for (uint_fast8_t i = 0; i < 2; ++i) {
        compute_gains_2nd_order_single(&gains->k1[i], &gains->k2[i],
                                       poles->omega_n[i], poles->zeta[i]);
    }
}

/**
 * @brief Compute 2nd order gains with 1 output from poles
 *  
 * @param[out] gains Output structure for computed gains
 * @param[in] poles Input pole structure (omega_n, zeta)
 */
static void compute_gains_2nd_order_1(struct Gains2ndOrder1* gains, const struct Poles2ndOrder1* poles)
{
    compute_gains_2nd_order_single(&gains->k1, &gains->k2,
                                   poles->omega_n, poles->zeta);
}

/**
 * @brief Compute 3rd order gains with 3 outputs from poles
 * 
 * @param[out] gains Output structure for computed gains
 * @param[in] poles Input pole structure (omega_n, zeta, p1)
 */
static void compute_gains_3rd_order_3(struct Gains3rdOrder3* gains, const struct Poles3rdOrder3* poles)
{
    for (uint_fast8_t i = 0; i < 3; ++i) {
        compute_gains_3rd_order_single(&gains->k1[i], &gains->k2[i], &gains->k3[i],
                                      poles->omega_n[i], poles->zeta[i], poles->p1[i]);
    }
}

/**
 * @brief Compute 3rd order gains with 2 outputs from poles
 * 
 * @param[out] gains Output structure for computed gains
 * @param[in] poles Input pole structure (omega_n, zeta, p1)
 */
static void compute_gains_3rd_order_2(struct Gains3rdOrder2* gains, const struct Poles3rdOrder2* poles)
{
    for (uint_fast8_t i = 0; i < 2; ++i) {
        compute_gains_3rd_order_single(&gains->k1[i], &gains->k2[i], &gains->k3[i],
                                      poles->omega_n[i], poles->zeta[i], poles->p1[i]);
    }
}

/**
 * @brief Compute 3rd order gains with 1 output from poles
 * 
 * @param[out] gains Output structure for computed gains
 * @param[in] poles Input pole structure (omega_n, zeta, p1)
 */
static void compute_gains_3rd_order_1(struct Gains3rdOrder1* gains, const struct Poles3rdOrder1* poles)
{
    compute_gains_3rd_order_single(&gains->k1, &gains->k2, &gains->k3,
                                  poles->omega_n, poles->zeta, poles->p1);
}

void print_Gains3rdOrder1(const char *name, const struct Gains3rdOrder1 *g) {
    printf("%s:\n", name);
    printf("  k1: %f\n  k2: %f\n  k3: %f\n\n", g->k1, g->k2, g->k3);
}

void print_Gains3rdOrder2(const char *name, const struct Gains3rdOrder2 *g) {
    printf("%s:\n", name);
    printf("  k1: [%f, %f]\n", g->k1[0], g->k1[1]);
    printf("  k2: [%f, %f]\n", g->k2[0], g->k2[1]);
    printf("  k3: [%f, %f]\n\n", g->k3[0], g->k3[1]);
}

void print_Gains3rdOrder3(const char *name, const struct Gains3rdOrder3 *g) {
    printf("%s:\n", name);
    printf("  k1: [%f, %f, %f]\n", g->k1[0], g->k1[1], g->k1[2]);
    printf("  k2: [%f, %f, %f]\n", g->k2[0], g->k2[1], g->k2[2]);
    printf("  k3: [%f, %f, %f]\n\n", g->k3[0], g->k3[1], g->k3[2]);
}

void print_Gains2ndOrder1(const char *name, const struct Gains2ndOrder1 *g) {
    printf("%s:\n", name);
    printf("  k1: %f\n  k2: %f\n\n", g->k1, g->k2);
}

void print_Gains2ndOrder2(const char *name, const struct Gains2ndOrder2 *g) {
    printf("%s:\n", name);
    printf("  k1: [%f, %f]\n", g->k1[0], g->k1[1]);
    printf("  k2: [%f, %f]\n\n", g->k2[0], g->k2[1]);
}

void print_Gains2ndOrder3(const char *name, const struct Gains2ndOrder3 *g) {
    printf("%s:\n", name);
    printf("  k1: [%f, %f, %f]\n", g->k1[0], g->k1[1], g->k1[2]);
    printf("  k2: [%f, %f, %f]\n\n", g->k2[0], g->k2[1], g->k2[2]);
}


/** @brief Initialize a filter based on its type
 *
 *  Initializes the filter state of the specified type with initial value x0.
 *  Supported types: LOWPASS_1, BUTTERWORTH_2, BUTTERWORTH_4, NOTCH.
 *  If the filter_type is invalid, no initialization occurs.
 * 
 * @param[in,out] filter Pointer to the Filter filter struct
 * @param[in] x0 Initial value to set the filter state
 */
static void init_filter_on_type(struct Filter *filter, float x0) {
  float tau = 1.0f / (2.0f * M_PI * filter->freq);
  switch(filter->filter_type) {
    case LOWPASS_1:
      init_first_order_low_pass(&filter->lp1, tau, 1.0f / PERIODIC_FREQUENCY, x0);
      break;
    case BUTTERWORTH_2:
      init_butterworth_2_low_pass(&filter->bw2, tau, 1.0f / PERIODIC_FREQUENCY, x0);
      break;
    case BUTTERWORTH_4:
      init_butterworth_4_low_pass(&filter->bw4, tau, 1.0f / PERIODIC_FREQUENCY, x0);
      break;
    default:
      return;
  }
}

/** @brief Initialize the Low Pass Filter struct with specified cutoff frequency
 *
 *  Sets parameters and filter type based on compile-time macros,
 *  then initializes the filter state with 0 output.
 * @param[in,out] filter Pointer to the Filter filter struct
 * @param[in] fc Cutoff frequency for the low pass filter [Hz]
 * @param[in] type Type of filter to use (e.g., LOWPASS_1, BUTTERWORTH_2, BUTTERWORTH_4)
 */
static void init_filter(struct Filter *filter, float fc, enum FilterType type){
  filter->filter_type = type;
  filter->freq        = fc;
  init_filter_on_type(filter, 0.0);
  filter->meas        = 0.0;
  filter->out         = 0.0;
}

/** @brief Update a filter based on its type
 *
 *  Feeds input to the filter specified by filter_type and updates the output.
 *  Protects against invalid filter_type by setting output to 0.0f.
 * @param[in,out] filter Pointer to the Filter filter struct
 * @param[in] input New input value to feed into the filter
 */
static void update_filter_on_type(struct Filter *filter, float input) {
  switch(filter->filter_type) {
    case LOWPASS_1:
      update_first_order_low_pass(&filter->lp1, input);
      filter->out = filter->lp1.last_out;
      break;
    case BUTTERWORTH_2:
      update_butterworth_2_low_pass(&filter->bw2, input);
      filter->out = filter->bw2.o[0];
      break;
    case BUTTERWORTH_4:
      update_butterworth_4_low_pass(&filter->bw4, input);
      filter->out = filter->bw4.lp2.o[0];
      break;
    default:
      filter->out = 0.0f;
      break;
  }
}

/**
 * @brief Updates actuator state observations from input actuator message.
 *
 * Converts servo angles from 0.01 degrees to radians and copies ESC RPM measurements
 * into the global actuator observation array.
 *
 * @param[in] sender_id          ID of the message sender (unused).
 * @param[in] actuators_t4_in_ptr Pointer to actuators input struct containing servo angles and ESC RPMs.
 * @param[in,out] actuators_t4_extra_data_in_ptr Pointer to extra data (unused).
 * FIXME: Get the correct actuator indices dynamically
 */
static void actuators_t4_in_callback(uint8_t sender_id __attribute__((unused)), struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr __attribute__((unused)))
{
    actuator_obs[0] = (float)actuators_t4_in_ptr->servo_1_angle * M_PI / 18000;  // rad
    actuator_obs[1] = (float)actuators_t4_in_ptr->servo_5_angle * M_PI / 18000;  // rad
    actuator_obs[2] = (float)actuators_t4_in_ptr->esc_1_rpm;
    actuator_obs[3] = (float)actuators_t4_in_ptr->esc_2_rpm;
}


// static void actuator_feedback_callback(uint8_t sender_id __attribute__((unused)), struct act_feedback_t *feedback, uint8_t num_act)
// {
//   for (uint_fast8_t i = 0; i < num_act; i++) {
//     int8_t idx = feedback[i].idx;

//     if (feedback[i].set.rpm) {
//       actuator_obs[idx] = (feedback[i].rpm - get_servo_min_T4(idx) / (float)(get_servo_max_T4(idx) - get_servo_min_T4(idx)));
//     } else if (feedback[i].set.position) {
//       actuator_obs[idx] = feedback[i].position * M_PI / 180;
//     }
//   }
// }

/**
 * @brief Propagates sensor and actuator filters with the latest feedback data.
 *
 * This function retrieves the current accelerations, velocities, and body rates
 * from the state estimator and updates all corresponding first-order filters.
 * It computes angular rate derivatives using finite differences, applies each
 * filter update through update_filter_on_type(), and processes actuator inputs.
 */
static void oneloop_andi_propagate_filters(void)
{
  // Fetch feedback
  struct  NedCoor_f *accel = stateGetAccelNed_f();
  struct  NedCoor_f *veloc = stateGetSpeedNed_f();
  struct  FloatRates *body_rates = stateGetBodyRates_f();

  float filt_p_prev = filt_p.meas;
  float filt_q_prev = filt_q.meas;
  float filt_r_prev = filt_r.meas;

  // Store feedback in filters
  filt_ay.meas = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);

  filt_an.meas = accel->x;
  filt_ae.meas = accel->y;
  filt_ad.meas = accel->z;
  filt_vn.meas = veloc->x;
  filt_ve.meas = veloc->y;
  filt_vd.meas = veloc->z;

  filt_p.meas = body_rates->p;
  filt_q.meas = body_rates->q;
  filt_r.meas = body_rates->r;

  // Finite differences
  filt_p_dot.meas = (filt_p.meas - filt_p_prev) * PERIODIC_FREQUENCY;
  filt_q_dot.meas = (filt_q.meas - filt_q_prev) * PERIODIC_FREQUENCY;
  filt_r_dot.meas = (filt_r.meas - filt_r_prev) * PERIODIC_FREQUENCY;

  // Update feedback filters
  update_filter_on_type(&filt_an, filt_an.meas);
  update_filter_on_type(&filt_ae, filt_ae.meas);
  update_filter_on_type(&filt_ad, filt_ad.meas);
  update_filter_on_type(&filt_vn, filt_vn.meas);
  update_filter_on_type(&filt_ve, filt_ve.meas);
  update_filter_on_type(&filt_vd, filt_vd.meas);

  update_filter_on_type(&filt_p, filt_p.meas);
  update_filter_on_type(&filt_q, filt_q.meas);
  update_filter_on_type(&filt_r, filt_r.meas);
  update_filter_on_type(&filt_p_dot, filt_p_dot.meas);
  update_filter_on_type(&filt_q_dot, filt_q_dot.meas);
  update_filter_on_type(&filt_r_dot, filt_r_dot.meas);

   
  update_filter_on_type(&filt_ay,    filt_ay.meas);
  // update_filter_on_type(&filt_airspeed, 0.0f);

  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
      update_filter_on_type(&filt_u[i], actuator_obs[i]);
  }
}

static float apply_deadband(float input, float deadband)
{
  if (fabsf(input) < deadband)
    return 0.0f;
  else if (input > 0)
    return (input - deadband) / (1.0f - deadband);
  else
    return (input + deadband) / (1.0f - deadband);
}

/**
 * @brief Computes desired roll, pitch, and yaw rates from RC input.
 *
 * Converts radio control stick inputs to desired angular rates by scaling
 * each channel value to be bounded by the corresponding rate bound.
 *
 * @param[out] rate_des    Output array [roll, pitch, yaw] containing desired rates.
 * @param[in]  rate_bounds Input array [roll, pitch, yaw] of maximum rate limits.
 */
static void get_desired_rates_radio_command(float rate_des[3], const float rate_bounds[3])
{
  rate_des[0] = ((float)radio_control_get(RADIO_ROLL) / MAX_PPRZ) * rate_bounds[1];
  rate_des[1] = ((float)radio_control_get(RADIO_PITCH) / MAX_PPRZ) * rate_bounds[2];
  rate_des[2] = ((float)radio_control_get(RADIO_YAW) / MAX_PPRZ) * rate_bounds[3];
}

/**
 * Calculates the desired specific thrust command based on radio input and thrust bounds.
 *
 * The thrust command is computed as a scaled version of the radio throttle input,
 * mapped linearly to the specified thrust bounds.
 *
 * @param[out] thrust_des Pointer to the variable where the computed thrust command will be stored.
 * @param[in] thrust_bound_max Maximum thrust bound.
 * @param[in] thrust_bound_min Minimum thrust bound.
 */
static void get_desired_thrust_radio_command(float *thrust_des, float thrust_bound_min, float thrust_bound_max)
{
  *thrust_des = -(float)radio_control_get(RADIO_THROTTLE) / MAX_PPRZ * (thrust_bound_max - thrust_bound_min) + thrust_bound_min;
}

/**
 * @brief Computes desired roll, pitch, and yaw attitudes from RC input.
 *
 * Converts radio control stick inputs to desired attitude angles by scaling
 * the roll and pitch sticks with corresponding attitude bounds, and
 * computes the desired heading using a heading rate bound and time step.
 *
 * @param[in,out] att_des        Pointer to desired attitudes (radians).
 * @param[in] att_bounds         Input array [roll, pitch] of maximum attitude limits (radians).
 * @param[in] heading_rate_bound Maximum yaw rate bound (radians/sec).
 * @param[in] dt                 Time step for integration (seconds).
 */
static void get_desired_attitude_radio_command(float att_des[3], const float att_bounds[2], float heading_rate_bound, float heading_ref, float dt)
{
  att_des[0] = ((float)radio_control_get(RADIO_ROLL) / MAX_PPRZ) * att_bounds[0];
  att_des[1] = ((float)radio_control_get(RADIO_PITCH) / MAX_PPRZ) * att_bounds[1];

  get_desired_heading_radio_command(&att_des[2], heading_rate_bound, heading_ref, dt);
}

/**
 * @brief Computes desired horizontal position (North-East) based on RC input and integration.
 *
 * Converts the pitch and roll commands from radio control to horizontal velocity setpoints,
 * integrates them over the time step 'dt', and updates the desired position.
 * 
 * FIXME: Currently does not support different position bounds or scaling factors per axis.
 *
 * @param[in,out] pos_des Pointer to current desired position array [North, East] (meters).
 * @param[in] pos_rate_bound Maximum allowed horizontal rate (meters/second).
 * @param[in] dt Time step for integration (seconds).
 */
static void get_desired_position_radio_command(float pos_des[2], const float pos_rate_bound[2], const float pos_ref[2], float dt)
{
  // Convert RC inputs to desired velocity commands
  float north_rate_des = (apply_deadband((float)radio_control_get(RADIO_PITCH), 150.0f) / MAX_PPRZ) * pos_rate_bound[0];
  float east_rate_des  = (apply_deadband((float)radio_control_get(RADIO_ROLL), 150.0f)  / MAX_PPRZ) * pos_rate_bound[1];

  // Integrate velocity to update desired position
  pos_des[0] += north_rate_des * dt;
  pos_des[1] += east_rate_des  * dt;

  Bound(pos_des[0], pos_ref[0] - 0.1f, pos_ref[0] + 0.1f);
  Bound(pos_des[1], pos_ref[1] - 0.1f, pos_ref[1] + 0.1f);
}


/**
 * @brief Computes desired altitude based on thrust RC input and integration.
 *
 * Converts the thrust command from radio control to an altitude rate,
 * integrates it over the time step 'dt', and updates the desired altitude.
 * 
 * FIXME: Currently does not support different upper and lower altitude bounds.
 *
 * @param[in,out] alt_des Pointer to the current desired altitude (meters).
 * @param[in] alt_rate_bound Maximum allowed altitude rate (meters/second).
 * @param[in] dt Time step for integration (seconds).
 */
static void get_desired_altitude_radio_command(float* alt_des, float alt_rate_bound, float alt_ref, float dt)
{
  float rate_des = (apply_deadband((float)radio_control_get(RADIO_THROTTLE), 300.0f) / MAX_PPRZ) * alt_rate_bound;
  *alt_des += rate_des * dt;
  Bound(*alt_des, alt_ref - 0.1f, alt_ref + 0.1f);
}

/**
 * @brief Updates desired heading based on yaw RC input and integration.
 *
 * Integrates yaw rate command scaled by `heading_rate_bound` over time `dt`
 * and normalizes the resulting heading angle.
 *
 * @param[in,out] heading_des Pointer to current desired heading (radians).
 * @param[in] heading_rate_bound Maximum yaw rate bound (radians/sec).
 * @param[in] dt Time step for integration (seconds).
 */
static void get_desired_heading_radio_command(float* heading_des, float heading_rate_bound, float heading_ref, float dt)
{
  float rate_des = (apply_deadband((float)radio_control_get(RADIO_YAW), 150.0f) / MAX_PPRZ) * heading_rate_bound;
  float heading_des_new = *heading_des + rate_des * dt;
  float heading_diff = heading_des_new - heading_ref;
  NormRadAngle(heading_diff);
  BoundAbs(heading_diff, 0.1f);
  *heading_des += heading_ref + heading_diff;
  NormRadAngle(*heading_des);
}


/**
 * @brief Computes scaling factors to normalize actuator input ranges for Weighted Least Squares control.
 *
 * This function calculates a scaling vector based on the ratio of the actuator's maximum and minimum
 * values in SI units to their normalized ranges. The scaling improves the condition number of the matrix
 * used in Weighted Least Squares control allocation for better accuracy.
 *
 * @param[out] wls_scaler Array to store computed scaling factors.
 * @param[in] act_max Array of actuator maximum values in SI units.
 * @param[in] act_min Array of actuator minimum values in SI units.
 */
static void compute_wls_scaling_factors(float wls_scaler_u[ANDI_NUM_ACT_TOT], const float act_max[ANDI_NUM_ACT_TOT], const float act_min[ANDI_NUM_ACT_TOT]) {
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT_TOT; i++){
    wls_scaler_u[i] = positive_non_zero(act_max[i] - act_min[i]);
  }
}

static void compute_wls_upper_bounds(float u_d_max[ANDI_NUM_ACT_TOT], const float act_state[ANDI_NUM_ACT_TOT], const float act_max[ANDI_NUM_ACT_TOT], const float act_rate_max[ANDI_NUM_ACT_TOT], float dt)
{
    for (uint_fast8_t i = 0; i < ANDI_NUM_ACT_TOT; i++)
    {
        // Calculate max rate allowed to avoid exceeding actuator max position in one timestep
        float rate_limit_pos = (act_max[i] - act_state[i]) / dt;

        // The rate limit considering actuator rate constraints (negative min rate since min rate might be negative)
        float rate_limit_rate = act_rate_max[i];

        // u_d_max is the minimum of the two constraints to avoid surpassing either constraint in the next step
        u_d_max[i] = (rate_limit_pos < rate_limit_rate) ? rate_limit_pos : rate_limit_rate;
    }
}

static void compute_wls_lower_bounds(float u_d_min[ANDI_NUM_ACT_TOT], const float act_state[ANDI_NUM_ACT_TOT], const float act_min[ANDI_NUM_ACT_TOT], const float act_rate_min[ANDI_NUM_ACT_TOT], float dt)
{
    for (uint_fast8_t i = 0; i < ANDI_NUM_ACT_TOT; i++)
    {
        // Calculate min rate allowed to avoid going below actuator min position in one timestep
        float rate_limit_pos = (act_min[i] - act_state[i]) / dt;

        // The rate limit considering actuator rate constraints (positive min rate)
        float rate_limit_rate = act_rate_min[i];

        // u_d_min is the maximum of the two constraints to avoid surpassing either constraint in the next step
        u_d_min[i] = (rate_limit_pos > rate_limit_rate) ? rate_limit_pos : rate_limit_rate;

        // Additionally ensure u_d_min is not greater than zero if actuator cannot reverse direction
        if (u_d_min[i] > 0.0f)
            u_d_min[i] = 0.0f;
    }
}


/**
 * @brief Evaluate the control effectiveness
 * 
 * @param[out] eff_mat Control effectiveness matrix.
 * @param[in] scaler Scale effectiveness matrix for WLS.
 * FIXME: Add option for 'half loop' where CE is adjusted to not
 * control certain virtual actuators.
 */
static void evaluate_effectiveness_matrix_oneloop(float eff_mat[ANDI_OUTPUTS * ANDI_NUM_ACT_TOT])
{
  // float v_e[3];
  // v_e[0] = filt_vn.out;
  // v_e[1] = filt_ve.out;
  // v_e[2] = filt_vd.out;

  // float w_e[3];
  // w_e[0] = filt_p.out;
  // w_e[1] = filt_q.out;
  // w_e[2] = filt_r.out;

  // Temporary contant values instead of sensor measurements  
  float v_e[3] = {0.0f, 0.0f, 0.0f};
  float w_e[3] = {0.0f, 0.0f, 0.0f};
  float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float wind_e[3] = {0.0f, 0.0f, 0.0f};
  float u[4] = {0.0f, 0.0f, 800.0f, 800.0f};
  cyclone_fu(v_e, w_e,
             quat, u,
             wind_e, obm_coefficients.data,
             eff_mat);
}

/** @brief Init function of Oneloop ANDI controller  */
void oneloop_andi_init(void)
{ 
  printf("INIT ANDI controller: start \n");
  oneloop_andi.control_type = CONTROL_TYPE_ANDI;
  oneloop_andi.control_mode = CONTROL_MODE_RATE;

  // Compute gains from poles
  compute_gains_2nd_order_3(&k_rate_e, &p_rate_e);
  compute_gains_2nd_order_3(&k_rate_rm, &p_rate_rm);
  compute_gains_3rd_order_3(&k_att_e, &p_att_e);
  compute_gains_3rd_order_3(&k_att_rm, &p_att_rm);
  compute_gains_3rd_order_2(&k_pos_e, &p_pos_e);
  compute_gains_3rd_order_2(&k_pos_rm, &p_pos_rm);
  compute_gains_3rd_order_1(&k_alt_e, &p_alt_e);
  compute_gains_3rd_order_1(&k_alt_rm, &p_alt_rm);
  compute_gains_2nd_order_1(&k_head_e, &p_head_e);
  compute_gains_2nd_order_1(&k_head_rm, &p_head_rm);

  print_Gains2ndOrder3("RATE GAINS ERR:", &k_rate_e);
  print_Gains2ndOrder3("RATE GAINS REF:", &k_rate_rm);
  print_Gains3rdOrder3("ATT GAINS ERR:", &k_att_e);
  print_Gains3rdOrder3("ATT GAINS REF:", &k_att_rm);
  print_Gains3rdOrder2("POS GAINS ERR:", &k_pos_e);
  print_Gains3rdOrder2("POS GAINS REF:", &k_pos_rm);
  print_Gains3rdOrder1("ALT GAINS ERR:", &k_alt_e);
  print_Gains3rdOrder1("ALT GAINS REF:", &k_alt_rm);
  print_Gains2ndOrder1("HEAD GAINS ERR:", &k_head_e);
  print_Gains2ndOrder1("HEAD GAINS REF:", &k_head_rm);

  // FIXME: Putting a small non zero value is dangerous.
  // Make sure that the dynamics are positive and non-zero
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    actuator_dynamics[i] = positive_non_zero(actuator_dynamics[i]);
  }

  // Initialize filter
  init_filter(&filt_an,     2.0, BUTTERWORTH_2);
  init_filter(&filt_ae,     2.0, BUTTERWORTH_2);
  init_filter(&filt_ad,     2.0, BUTTERWORTH_2);
  init_filter(&filt_vn,     2.0, BUTTERWORTH_2);
  init_filter(&filt_ve,     2.0, BUTTERWORTH_2);
  init_filter(&filt_vd,     2.0, BUTTERWORTH_2);
  init_filter(&filt_p_dot,  2.0, BUTTERWORTH_2);
  init_filter(&filt_q_dot,  2.0, BUTTERWORTH_2);
  init_filter(&filt_r_dot,  2.0, BUTTERWORTH_2);
  init_filter(&filt_p,     20.0, BUTTERWORTH_2);
  init_filter(&filt_q,     20.0, BUTTERWORTH_2);
  init_filter(&filt_r,     20.0, BUTTERWORTH_2);

  init_filter(&filt_ay,       2.0, BUTTERWORTH_2);
  init_filter(&filt_airspeed, 2.0, BUTTERWORTH_2);

  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
      init_filter(&filt_u[i], 2.0, BUTTERWORTH_2);
  }

  // Initialize references to zero
  float_vect_zero(oneloop_andi.att_des, 3);
  float_vect_zero(oneloop_andi.att_ref.att, 3);
  float_vect_zero(oneloop_andi.att_ref.att_d, 3);
  float_vect_zero(oneloop_andi.att_ref.att_2d, 3);
  float_vect_zero(oneloop_andi.att_ref.att_3d, 3);
  float_vect_zero(oneloop_andi.pos_des, 2);
  float_vect_zero(oneloop_andi.pos_ref.pos, 2);
  float_vect_zero(oneloop_andi.pos_ref.vel, 2);
  float_vect_zero(oneloop_andi.pos_ref.acc, 2);
  float_vect_zero(oneloop_andi.pos_ref.jer, 2);
  oneloop_andi.alt_des = 0.0f;
  oneloop_andi.alt_ref.pos = 0.0f;
  oneloop_andi.alt_ref.vel = 0.0f;
  oneloop_andi.alt_ref.acc = 0.0f;
  oneloop_andi.alt_ref.jer = 0.0f;
  oneloop_andi.head_des = 0.0f;
  oneloop_andi.head_ref.head = 0.0f;
  oneloop_andi.head_ref.head_d = 0.0f;

  // Initialize controller variables
  float_vect_zero(andi_u, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);
  float_vect_zero(actuator_state_1l, ANDI_NUM_ACT_TOT);

  // Bind actuator feedback
  AbiBindMsgACTUATORS_T4_IN(ABI_BROADCAST, &actuators_t4_in_event, actuators_t4_in_callback);

  // Compute wls scaling constants
  compute_wls_scaling_factors(wls_scaler_u, act_max, act_min);
  // Start telemetry
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_ONE, send_eff_mat_one_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE, send_guidance_oneloop_andi);
    // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATOR_STATE, send_oneloop_actuator_state);
    // register_pferiodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_oneloop_debug);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_oneloop);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_oneloop);
  #endif
  printf("INIT ANDI controller: succes \n");
}

/**
 * @brief Function that resets important values upon engaging Oneloop ANDI.
 * FIXME: Ideally we should distinguish between the "stabilization" and "guidance" needs because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 */
void oneloop_andi_enter(enum ControlMode control_mode, enum ControlType control_type)
{
  printf("ENTER ANDI controller: start \n");
  oneloop_andi.control_mode   = control_mode;
  oneloop_andi.control_type   = control_type;

  // initialize wls weights
  switch (control_mode) {
    case CONTROL_MODE_RATE:
      float_vect_copy(wls_one_p.Wv, wls_wv_rate, ANDI_OUTPUTS);
      actuator_dynamics[2] = 1;
      actuator_dynamics[3] = 1;
      break;
    case CONTROL_MODE_ATTITUDE:
      float_vect_copy(wls_one_p.Wv, wls_wv_attitude, ANDI_OUTPUTS);
      actuator_dynamics[2] = 1;
      actuator_dynamics[3] = 1;
      break;
    case CONTROL_MODE_GUIDANCE:
      float_vect_copy(wls_one_p.Wv, wls_wv_guidance, ANDI_OUTPUTS);
      break;
  }
  printf("ENTER ANDI controller: succes \n");
}

// void stabilization_andi_rate_run(bool in_flight, struct StabilizationSetpoint att_sp, struct ThrustSetpoint, int32_t *cmd)
// {
//   // Fetch filtered aircraft states (SI units)
//   oneloop_andi.att_state.att_d[0]  = filt_p.out;
//   oneloop_andi.att_state.att_d[1]  = filt_q.out;
//   oneloop_andi.att_state.att_d[2]  = filt_r.out;
//   oneloop_andi.att_state.att_2d[0] = filt_p_dot.out;
//   oneloop_andi.att_state.att_2d[1] = filt_q_dot.out;
//   oneloop_andi.att_state.att_2d[2] = filt_r_dot.out;  
//   oneloop_andi.pos_state.vel[0] = filt_vn.out;      
//   oneloop_andi.pos_state.vel[1] = filt_ve.out;
//   oneloop_andi.alt_state.vel = filt_vd.out;

//   // Fetch filtered actuator states (SI units)
//   float act_state[ANDI_NUM_ACT]
//   for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
//     act_state[i] = filt_u[i];
//   }

//   float eff_mat_stab[4*4]
//   evaluate_effectiveness_matrix_stab(eff_mat_stab)
// }


// void oneloop_andi_rate_run(bool in_flight, struct StabilizationSetpoint *att_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
// {
//   // Update filters
//   oneloop_andi_propagate_filters();
//   // Register the state of the drone in the variables used in RM and EC
//   struct FloatEulers attitude_euler;
//   float_eulers_of_quat_zxy(&attitude_euler, stateGetNedToBodyQuat_f());
//   oneloop_andi.att_state.att[0] = attitude_euler.phi;
//   oneloop_andi.att_state.att[1] = attitude_euler.theta;
//   oneloop_andi.att_state.att[2] = attitude_euler.psi;
//   oneloop_andi.att_state.att_d[0]  = filt_p.out;
//   oneloop_andi.att_state.att_d[1]  = filt_q.out;
//   oneloop_andi.att_state.att_d[2]  = filt_r.out;
//   oneloop_andi.att_state.att_2d[0] = filt_p_dot.out;
//   oneloop_andi.att_state.att_2d[1] = filt_q_dot.out;
//   oneloop_andi.att_state.att_2d[2] = filt_r_dot.out;

//   oneloop_andi.pos_state.pos[0] = stateGetPositionNed_f()->x;   
//   oneloop_andi.pos_state.pos[1] = stateGetPositionNed_f()->y;   
//   oneloop_andi.pos_state.vel[0] = filt_vn.out;      
//   oneloop_andi.pos_state.vel[1] = filt_ve.out; 
//   oneloop_andi.pos_state.acc[0] = filt_an.out;
//   oneloop_andi.pos_state.acc[1] = filt_ae.out;

//   oneloop_andi.alt_state.pos = stateGetPositionNed_f()->z;      
//   oneloop_andi.alt_state.vel = filt_vd.out;      
//   oneloop_andi.alt_state.acc = filt_ad.out;

//   // Get desired rate and thurst from setpoints
//   // struct FloatRates rate_des = stab_sp_to_rates_f(att_sp);
//   // float thrust_des = th_sp_to_thrust_f(thrust_state, THRUST_AXIS_Z);

//   // Generate smooth references from inputs
//   generate_reference_rate(dt_1l, rate_des, k_rate_rm, att_bounds, att_ref)
// }

/**
 * @brief Main function that runs the controller and performs control allocation
 */
void oneloop_andi_run(enum ControlMode control_mode)
{
  // At beginnig of the loop: (1) Register Attitude, (2) Initialize gains of RM and EC, (3) Calculate Normalization of Actuators Signals, (4) Propagate Actuator Model, (5) Update effectiveness matrix
  oneloop_andi_propagate_filters();

  // Register the state of the drone in the variables used in RM and EC

  // Attitude
  struct FloatEulers attitude_euler;
  float_eulers_of_quat_zxy(&attitude_euler, stateGetNedToBodyQuat_f());
  oneloop_andi.att_state.att[0] = attitude_euler.phi;
  oneloop_andi.att_state.att[1] = attitude_euler.theta;
  oneloop_andi.att_state.att[2] = attitude_euler.psi;
  oneloop_andi.att_state.att_d[0]  = filt_p.out;
  oneloop_andi.att_state.att_d[1]  = filt_q.out;
  oneloop_andi.att_state.att_d[2]  = filt_r.out;
  oneloop_andi.att_state.att_2d[0] = filt_p_dot.out;
  oneloop_andi.att_state.att_2d[1] = filt_q_dot.out;
  oneloop_andi.att_state.att_2d[2] = filt_r_dot.out;

  // Position
  oneloop_andi.pos_state.pos[0] = stateGetPositionNed_f()->x;   
  oneloop_andi.pos_state.pos[1] = stateGetPositionNed_f()->y;   
  oneloop_andi.pos_state.vel[0] = filt_vn.out;      
  oneloop_andi.pos_state.vel[1] = filt_ve.out; 
  oneloop_andi.pos_state.acc[0] = filt_an.out;
  oneloop_andi.pos_state.acc[1] = filt_ae.out;

  // Altitude
  oneloop_andi.alt_state.pos = stateGetPositionNed_f()->z;      
  oneloop_andi.alt_state.vel = filt_vd.out;      
  oneloop_andi.alt_state.acc = filt_ad.out;
  // FIXME: Todo, add Heading state fetching (which is part of attitude, might be redundant)


  // Guidance Pseudo Control Vector (nu) based on reference model and error controller

  // Assemble pseudo control vector
  // FIXME: Add heading control.
  // FIXME: Add guidance control

  evaluate_effectiveness_matrix_oneloop(eff_mat);

  float_vect_zero(wls_one_p.v, ANDI_OUTPUTS); // An Ae Ad psi p q r
  switch (control_mode) {
    case CONTROL_MODE_RATE:
      {
      get_desired_rates_radio_command(oneloop_andi.att_des, att_bounds.att_d);
      reference_model_rate(dt_1l, oneloop_andi.att_des, &k_rate_rm, &att_bounds, &oneloop_andi.att_ref);
      error_controller_rate(&oneloop_andi.att_ref, &oneloop_andi.att_state, &k_rate_e, &wls_one_p.v[4]);

      // Direct thrust control
      get_desired_thrust_radio_command(&wls_one_p.v[2], 0.0f, 3.0f);
      break;
      }
    case CONTROL_MODE_ATTITUDE:
      {
      get_desired_attitude_radio_command(oneloop_andi.att_des, att_bounds.att, att_bounds.att_d[2], oneloop_andi.head_ref.head, dt_1l);
      reference_model_attitude(dt_1l, oneloop_andi.att_des, &k_att_rm, &att_bounds, &oneloop_andi.att_ref);
      error_controller_attitude(&oneloop_andi.att_ref, &oneloop_andi.att_state, &k_att_e, &wls_one_p.v[4]);

      // Direct thrust control
      get_desired_thrust_radio_command(&wls_one_p.v[2], 0.0f, 3.0f);
      break;
      }
    case CONTROL_MODE_GUIDANCE:
      {
      // Position
      get_desired_position_radio_command(oneloop_andi.pos_des, pos_bounds.vel, oneloop_andi.pos_ref.pos, dt_1l);
      reference_model_position(dt_1l, oneloop_andi.pos_des, &k_pos_rm, &pos_bounds, &oneloop_andi.pos_ref);
      error_controller_position(&oneloop_andi.pos_ref, &oneloop_andi.pos_state, &k_pos_e, &wls_one_p.v[0]);

      // Altitude
      get_desired_altitude_radio_command(&oneloop_andi.alt_des, alt_bounds.vel, oneloop_andi.alt_ref.pos, dt_1l);
      reference_model_altitude(dt_1l, oneloop_andi.alt_des, &k_alt_rm, &alt_bounds, &oneloop_andi.alt_ref);
      error_controller_altitude(&oneloop_andi.alt_ref, &oneloop_andi.alt_state, &k_alt_e, &wls_one_p.v[2]);

      // Heading (todo)

      // Rate (desired rate is taked from oneloop virtual actuator)
      reference_model_rate(dt_1l, &andi_du[4], &k_rate_rm, &att_bounds, &oneloop_andi.att_ref);
      error_controller_rate(&oneloop_andi.att_ref, &oneloop_andi.att_state, &k_rate_e, &wls_one_p.v[4]);
      break;
      }
    default:
      break;
      // handle invalid control_mode (now do nothing)
  }

  // Control Allocation
  // FIXME: Dynamically compute WLS bounds based on current actuator position.
  // FIXME: Put this part in its own function?
  float eff_mat_scaled[ANDI_OUTPUTS * ANDI_NUM_ACT_TOT];
  for (uint_fast8_t i = 0; i < ANDI_OUTPUTS; i++) {
    for (uint_fast8_t j = 0; j < ANDI_NUM_ACT_TOT; j++) {
      eff_mat_scaled[i * ANDI_NUM_ACT_TOT + j] = eff_mat[i * ANDI_NUM_ACT_TOT + j] * wls_scaler_u[j];
    }
  }

  // compute_wls_upper_bounds(wls_one_p.u_max, actuator_state_1l);

  for (uint_fast8_t i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = &eff_mat_scaled[i * ANDI_NUM_ACT_TOT];
  }
  // WLS Control Allocator
  wls_alloc(&wls_one_p, bwls_1l, 0, 0, 10);
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    andi_du[i] = wls_scaler_u[i] * wls_one_p.u[i];
  }

  // Commit real actuator commands
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    andi_u[i] = andi_du[i] / actuator_dynamics[i] + filt_u[i].out; // SI units
    commands[i] = (int16_t)andi_u[i] * 100;
    commands[0] = 5000;
  }
}
