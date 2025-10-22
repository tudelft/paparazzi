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
#if defined(FILTER_USE_LP1)
enum FilterType filter_type = LOWPASS_1;
#elif defined(FILTER_USE_BW2)
enum FilterType filter_type = BUTTERWORTH_2;
#elif defined(FILTER_USE_YAW_LP4)
enum FilterType filter_type = BUTTERWORTH_4;
#else
#error "Filter type not defined. Define one of: USE_LP1, USE_BW2, USE_YAW_LP4."
#endif

#ifdef ONELOOP_ANDI_ACT_IS_SERVO
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_IS_SERVO;
#else
bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = {0};
#endif

#ifdef ONELOOP_ANDI_ACT_DYN
float  act_dynamics[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_DYN;
float  act_dyn_ctrl[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_DYN;
#else
#error "You must specify the actuator dynamics"
#endif

#if defined(ONELOOP_ANDI_ACT_MAX) && defined(ONELOOP_ANDI_ACT_MIN)
float act_max[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX;
float act_min[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN;
#else
#error "You must specify the actuator limits: ONELOOP_ANDI_ACT_MAX and ONELOOP_ANDI_ACT_MIN"
#endif

#ifdef ONELOOP_ANDI_ACT_MAX_NORM
float  act_max_norm[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MAX_NORM;
#else
float  act_max_norm[ANDI_NUM_ACT_TOT] = {[0 ... ANDI_NUM_ACT_TOT-1] = 1.0};
#endif

#ifdef ONELOOP_ANDI_ACT_MIN_NORM
float  act_min_norm[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_ACT_MIN_NORM;
#else
float  act_min_norm[ANDI_NUM_ACT_TOT] = {[0 ... ANDI_NUM_ACT_TOT-1] = -1.0};
#endif

#ifdef ONELOOP_ANDI_U_PREF
static float u_pref[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_U_PREF;
#else
static float u_pref[ANDI_NUM_ACT_TOT] = {0.0};
#endif

#if ANDI_NUM_ACT_TOT != WLS_N_U_MAX
#error Matrix-WLS_N_U_MAX is not equal to the number of actuators: define WLS_N_U_MAX == ANDI_NUM_ACT_TOT in airframe file
#endif
#if ANDI_OUTPUTS != WLS_N_V_MAX
#error Matrix-WLS_N_V_MAX is not equal to the number of controlled axis: define WLS_N_V_MAX == ANDI_OUTPUTS in airframe file
#endif

/*  Define Section of the functions used in this module*/
static float positive_non_zero(float input);
static void scaled_error_nd(int n, float err[restrict n], float a[static n], float b[static n], float k[static n]);
static void integrate_nd(int n, float a[static n], float a_dot[static n], float dt);

static void reference_model_rate(float dt, float rate_des[3], const struct Gains2ndOrder3 *k_rate_rm, const struct OneloopAttRef *bounds, struct OneloopAttRef *rate_ref);
static void reference_model_attitude(float dt, float att_des[3], const struct Gains3rdOrder3 *k_att_rm, const struct OneloopAttRef *bounds, struct OneloopAttRef *att_ref);
static void reference_model_position(float dt, float pos_des[2], const struct Gains3ndOrder2 *k_pos_rm, const struct OneloopPosRef *bounds, struct OneloopPosRef *pos_ref);
static void reference_model_altitude(float dt, float alt_des, const struct Gains3ndOrder1 *k_alt_rm, const struct OneloopAltRef *bounds, struct OneloopAltRef *alt_ref);
static void reference_model_heading(float dt, float head_des, const struct Gains2ndOrder1 *k_head_rm, const struct OneloopHeadRef *bounds, struct OneloopHeadRef *head_ref);

static void error_controller_rate(float dt, const struct OneloopAttRef *rate_ref, const struct OneloopAttState *rate_state, const struct Gains2ndOrder3 *k_rate_e, float *nu_rate);
static void error_controller_attitude(float dt, const struct OneloopAttRef *att_ref, const struct OneloopAttState *att_state, const struct Gains3rdOrder3 *k_att_e, float *nu_att);
static void error_controller_position(float dt, const struct OneloopPosRef *pos_ref, const struct OneloopPosState *pos_state, const struct Gains2ndOrder3 *k_pos_e, float *nu_pos);
static void error_controller_altitude(float dt, const struct OneloopAltRef *alt_ref, const struct OneloopAltState *alt_state, const struct Gains2ndOrder3 *k_alt_e, float *nu_alt);
static void error_controller_heading(float dt, const struct OneloopHeadRef *head_ref, const struct OneloopHeadState *head_state, const struct Gains2ndOrder3 *k_head_e, float *nu_head);

static void compute_gains_2nd_order_single(float* k1, float* k2, float omega_n, float zeta);
static void compute_gains_3rd_order_single(float* k1, float* k2, float* k3, float omega_n, float zeta, float p1);
static void compute_gains_2nd_order_3d(struct Gains2ndOrder3* gains, const struct Poles2ndOrder3* poles);
static void compute_gains_2nd_order_2d(struct Gains2ndOrder2* gains, const struct Poles2ndOrder2* poles);
static void compute_gains_2nd_order_1d(struct Gains2ndOrder1* gains, const struct Poles2ndOrder1* poles);
static void compute_gains_3rd_order_3d(struct Gains3rdOrder3* gains, const struct Poles3rdOrder3* poles);
static void compute_gains_3rd_order_2d(struct Gains3rdOrder2* gains, const struct Poles3rdOrder2* poles);
static void compute_gains_3rd_order_1d(struct Gains3rdOrder1* gains, const struct Poles3rdOrder1* poles);

static void  init_filter(struct Filter *filer, float fc, enum FilterType type);
static void  init_filter_on_type(struct Filter *filter, float x0);
static void  update_filter_on_type(struct Filter *filter, float input);
static void  oneloop_andi_propagate_filters(void);

static void get_desired_rates_radio_command(float* rate_des, const float* rate_bounds);
static void get_desired_attitude_radio_command(float* att_des, const float* att_bounds, float heading_rate_bound, float dt);
static void get_desired_heading_radio_command(float* heading_des, float heading_rate_bound, float dt);

static void get_act_state_oneloop(void);
static void discretize_act_dynamics(float dt, float* act_dynamics_d, const float* act_dynamics);
static void compute_wls_scaling_factor(float* wls_scaler_u, const float* act_max, const float* act_min, const float* act_max_norm, const float* act_min_norm);
static void evaluate_effectiveness_matrix(float* eff_mat);

/**
 * @brief Controller poles
 *
 * These structures define pole placement parameters for different control modes.
 * They are used for real-time tuning of stabilization, position, altitude, 
 * and heading control loops.
 */
// FIXME: Make poles dynamically set from airframe file
struct Poles2ndOrder3 p_rate_e;
struct Poles2ndOrder3 p_rate_rm;
struct Poles3rdOrder3 p_att_e;
struct Poles3rdOrder3 p_att_rm;
struct Poles3rdOrder2 p_pos_e;
struct Poles3rdOrder2 p_pos_rm;
struct Poles3rdOrder1 p_alt_e;
struct Poles3rdOrder1 p_alt_rm;
struct Poles2ndOrder1 p_head_e;
struct Poles2ndOrder1 p_head_rm;

/** @brief Reference bounds
 *
 * These structures define the maximum allowable reference values for
 * stabilization, position, altitude, and heading control loops.
 * They are used to limit the reference commands within safe operational limits.
 */
struct OneloopStabRef att_bounds;
struct OneloopPosRef pos_bounds;
struct OneloopAltRef alt_bounds;
struct OneloopHeadRef head_bounds;

/**
 * @brief Model coefficients 
 */
union CycloneCoefficients obm_coefficients;

/** 
 * @brief Controller gains
 *
 * These global variables store the controller gains derived from the
 * previously defined pole placement parameters.
 *
 * The values are computed from the designed poles (p_stab_*, p_pos_*, etc.)
 * and directly used by the flight control laws in the real-time control loop.
 * Updating the poles during tuning automatically affects these gain values.
 */
struct Gains2ndOrder3 k_rate_e;
struct Gains2ndOrder3 k_rate_rm
struct Gains3rdOrder3 k_att_e;
struct Gains3rdOrder3 k_att_rm;
struct Gains3rdOrder2 k_pos_e;
struct Gains3rdOrder2 k_pos_rm;
struct Gains3rdOrder1 k_alt_e;
struct Gains3rdOrder1 k_alt_rm;
struct Gains2ndOrder1 k_head_e;
struct Gains2ndOrder1 k_head_rm;

static Filter filt_p;
static Filter filt_q;
static Filter filt_r;
static Filter filt_p_dot;
static Filter filt_q_dot;
static Filter filt_r_dot;
static Filter filt_an;
static Filter filt_ae;
static Filter filt_ad;
static Filter filt_vn;
static Filter filt_ve;
static Filter filt_vd;

static Filter filt_ay;
static Filter filt_airspeed;
static Filter filt_u[ANDI_NUM_ACT_TOT];  // Low pass filter for actuators          

/* Oneloop Misc variables*/
static float dt_1l = 1. / PERIODIC_FREQUENCY;
static float g   = 9.81; // [m/s^2] Gravitational Acceleration

/* Oneloop Control Variables*/
float andi_u[ANDI_NUM_ACT_TOT];
float andi_du[ANDI_NUM_ACT_TOT];
float nu[ANDI_OUTPUTS];
static float act_dynamics_d[ANDI_NUM_ACT_TOT];
float actuator_state_1l[ANDI_NUM_ACT_TOT];

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
  .u_min     = {0.0},
  .u_max     = {0.0},
  .PC        = 0.0,
  .SC        = 0.0,
  .iter      = 0
};

#ifdef ONELOOP_ANDI_WV // {ax_dot,ay_dot,az_dot,p_ddot,q_ddot,r_ddot}
static float Wv_backup[ANDI_OUTPUTS] = ONELOOP_ANDI_WV;
#else
static float Wv_backup[ANDI_OUTPUTS] = {[0 ... ANDI_OUTPUTS-1] = 1.0};
#endif

#ifdef ONELOOP_ANDI_WU // {mF,mR,mB,mL,mP,de,dr,da,df,phi,theta}
static float Wu_backup[ANDI_NUM_ACT_TOT] = ONELOOP_ANDI_WU;
#else
static float Wu_backup[ANDI_NUM_ACT_TOT] = {[0 ... ANDI_NUM_ACT_TOT-1] = 1.0};
#endif

/* Effectiveness Matrix definition */
float *bwls_1l[ANDI_OUTPUTS];
float eff_mat[ANDI_OUTPUTS][ANDI_NUM_ACT_TOT];
float n_array[ANDI_OUTPUTS];
float m_array[ANDI_NUM_ACT_TOT];
float coupling_factor[ANDI_OUTPUTS];
float wls_scaler_u[ANDI_NUM_ACT_TOT];

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
static void send_eff_mat_stab_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0;
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID, 
                ANDI_NUM_ACT, EFF_MAT_RW[3],
                ANDI_NUM_ACT, EFF_MAT_RW[4],
                ANDI_NUM_ACT, EFF_MAT_RW[5], 
                                   1, &zero,
                ANDI_NUM_ACT, G2_RW);
}

static void send_eff_mat_guid_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_EFF_MAT_GUID(trans, dev, AC_ID, 
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[0],
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[1],
                ANDI_NUM_ACT_TOT, EFF_MAT_RW[2]);
}
static void send_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  float temp_eulers_zxy_des[3] = {eulers_zxy_des.phi, eulers_zxy_des.theta, eulers_zxy_des.psi};
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                                        3, temp_eulers_zxy_des,
                                        3, oneloop_andi.sta_state.att,
                                        3, oneloop_andi.sta_ref.att,
                                        3, oneloop_andi.sta_state.att_d,
                                        3, oneloop_andi.sta_ref.att_d,                                       
                                        3, oneloop_andi.sta_state.att_2d,
                                        3, oneloop_andi.sta_ref.att_2d,
                                        3, oneloop_andi.sta_ref.att_3d,                                       
                                        ANDI_NUM_ACT, actuator_state_1l);                                      
}
// static void send_oneloop_actuator_state(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_ACTUATOR_STATE(trans, dev, AC_ID, 
//                                         ANDI_NUM_ACT, actuator_state_1l,
//                                         ANDI_NUM_ACT_TOT, andi_u);
// }
static void send_guidance_oneloop_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE(trans, dev, AC_ID,
                                        &oneloop_andi.gui_ref.pos[0],
                                        &oneloop_andi.gui_ref.pos[1],
                                        &oneloop_andi.gui_ref.pos[2],
                                        &oneloop_andi.gui_state.pos[0],
                                        &oneloop_andi.gui_state.pos[1],
                                        &oneloop_andi.gui_state.pos[2],
                                        &oneloop_andi.gui_ref.vel[0],
                                        &oneloop_andi.gui_ref.vel[1],
                                        &oneloop_andi.gui_ref.vel[2],
                                        &oneloop_andi.gui_state.vel[0],
                                        &oneloop_andi.gui_state.vel[1],
                                        &oneloop_andi.gui_state.vel[2],
                                        &oneloop_andi.gui_ref.acc[0],
                                        &oneloop_andi.gui_ref.acc[1],
                                        &oneloop_andi.gui_ref.acc[2],
                                        &oneloop_andi.gui_state.acc[0],
                                        &oneloop_andi.gui_state.acc[1],
                                        &oneloop_andi.gui_state.acc[2],
                                        &oneloop_andi.gui_ref.jer[0],
                                        &oneloop_andi.gui_ref.jer[1],
                                        &oneloop_andi.gui_ref.jer[2]);
}

static void debug_vect(struct transport_tx *trans, struct link_device *dev, char *name, float *data, int datasize)
{
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID,
                           strlen(name), name,
                           datasize, data);
}

static void send_oneloop_debug(struct transport_tx *trans, struct link_device *dev)
{
  float temp_debug_vect[14];
  temp_debug_vect[0] = (float) chirp_n_counter;
  temp_debug_vect[1] = oneloop_andi_model[1];
  temp_debug_vect[2] = oneloop_andi_model[2];
  temp_debug_vect[3] = oneloop_andi_model[3];
  temp_debug_vect[4] = oneloop_andi_model[4];
  temp_debug_vect[5] = oneloop_andi_model[5];
  temp_debug_vect[6] = oneloop_andi_sigma;
  temp_debug_vect[7] = chirp_on ? 1.0 : 0.0;
  temp_debug_vect[8] = filt_ad.meas;
  temp_debug_vect[9] = temp_checks_2[0];
  temp_debug_vect[10] = temp_checks_2[1];
  temp_debug_vect[11] = temp_ref_att[0];
  temp_debug_vect[12] = temp_ref_att[1];
  temp_debug_vect[13] = temp_ref_att[2];
  debug_vect(trans, dev, "APF", temp_debug_vect, 14);
}

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
 * @param err Output array for the computed scaled error [n]
 * @param a   First input array (reference or desired values) [n]
 * @param b   Second input array (measured or actual values) [n]
 * @param k   Scaling gains applied elementwise to the error [n]
 * @param n   Dimension of the arrays
 */
void scaled_error_nd(int n, float err[restrict n], float a[static n], float b[static n], float k[static n])
{
  for (uint8_t i = 0; i < n; i++) {
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
void integrate_nd(int n, float a[static n], float a_dot[static n], float dt)
{
  for (uint8_t i = 0; i < n; i++) {
    a[i] = a[i] + dt * a_dot[i];
  }
}

/**
 * @brief 2nd-order reference model for rate control.
 *
 * Generates smooth angular rate, acceleration, and jerk references with bounded dynamics.
 * The attitude field is unused in rate control.
 *
 * @param[in] dt          Sample time [s]
 * @param[in] rate_des   Desired angular rates [rad/s]
 * @param[in] k_stb_rm    Pointer to reference model gain parameters
 * @param[in] bounds      Pointer to reference model limits
 * @param[in,out] att_ref Pointer to struct containing reference model states (in/out)
 */

void reference_model_rate(
  float dt, 
  float rate_des[3], 
  const struct Gains2ndOrder3 *k_rate_rm,
  const struct OneloopAttRef *bounds,
  struct OneloopAttRef *att_ref)
{
  float rate_d_des[3];
  float rate_2d_des[3];

  for (uint8_t i = 0; i < 3; i++)
    BoundAbs(rate_des[i], bounds->att_d[i]);

  // Angular rate error 
  scaled_error_nd(3, rate_d_des, rate_des, att_ref->att_2d, k_rate_rm->k1);
  for (uint8_t i = 0; i < 3; i++)
    BoundAbs(rate_d_des[i], bounds->att_2d[i]);

  // Angular acceleration error
  scaled_error_nd(3, rate_2d_des, rate_d_des, att_ref->att_3d, k_rate_rm->k2);
  for (uint8_t i = 0; i < 3; i++)
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
void reference_model_attitude(
  float dt, 
  float att_des[3],
  const struct Gains3rOrder3 *k_att_rm,
  const struct OneloopAttRef *bounds,
  struct OneloopAttRef *att_ref)
{
  float att_d_des[3];
  float att_2d_des[3];
  float att_3d_des[3];

  // Limit input attitude commands
  for (uint8_t i = 0; i < 3; i++)
    BoundAbs(att_des[i], bounds->att[i]);

  // Attitude tracking error → desired angular rate
  scaled_error_nd(3, att_d_des, att_des, att_ref->att_d, k_att_rm->k1);
  for (uint8_t i = 0; i < 3; i++)
    BoundAbs(att_d_des[i], bounds->att_d[i]);

  // Angular rate tracking error → desired angular acceleration
  scaled_error_nd(3, att_2d_des, att_d_des, att_ref->att_2d, k_att_rm->k2);
  for (uint8_t i = 0; i < 3; i++)
    BoundAbs(att_2d_des[i], bounds->att_2d[i]);

  scaled_error_nd(3, att_3d_des, att_d_des, att_ref->att_3d, k_att_rm->k3);
  for (uint8_t i = 0; i < 3; i++)
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
void reference_model_position(
  float dt,
  float pos_des,
  const struct Gains3rdOrder2 *k_pos_rm,
  const struct OneloopPosRef *bounds,
  struct OneloopPosRef *pos_ref)
{
  float vel_des[2];
  float acc_des[2];
  float jer_des[2];

  // Position error
  scaled_error_nd(2, vel_des, pos_des, pos_ref->pos, k_pos_rm->k1);
  for (uint8_t i = 0; i < 2; i++) {
    BoundAbs(vel_des[i], bounds->vel[i]);
  }

  // Velocity error 
  scaled_error_nd(2, acc_des, vel_des, pos_ref->vel, k_pos_rm->k2);
  for (uint8_t i = 0; i < 2; i++) {
    BoundAbs(acc_des[i], bounds->acc[i]);
  }

  // Acceleration error
  scaled_error_nd(2, jer_des, acc_des, pos_ref->accel, k_pos_rm->k3);
  for (uint8_t i = 0; i < 2; i++) {
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
void reference_model_altitude(
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
  BoundsAbs(pos_err, bounds->vel);

  // Velocity error scaled by k2, bounded by velocity limits
  acc_des = k_alt_rm->k2 * (vel_des - alt_ref->vel);
  BoundAbs(acc_des, bounds->acc);

  // Acceleration error scaled by k3, bounded by accel limits
  jer_des = k_alt_rm->k3 * (acc_des - alt_ref->accel);
  BoundAbs(jer_des, bounds->jer);

  // Reference propagation
  alt_ref->jer = jer_des;
  integrate_nd(1, alt_ref->acc, &jer_des, dt);
  integrate_nd(1, alt_ref->vel, alt_ref->acc, dt);
  integrate_nd(1, alt_ref->pos, alt_ref->vel, dt);
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
void reference_model_heading(
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
  integrate_nd(1, head_ref->head_d, accel_des, dt);
  integrate_nd(1, head_ref->head, head_ref->head_d, dt);

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
 * @param[in]  dt         Time step [s]
 * @param[in]  att_ref    Pointer to rate reference states (angular rate, acceleration, jerk)
 * @param[in]  att_state  Pointer to current rate states (angular rate, acceleration)
 * @param[in]  k_rate_e   Pointer to structured 2nd-order gains (k1, k2 per axis)
 * @param[out] nu_rate    Pointer to output virtual control command [3]
 */
void error_controller_rate(
  float dt,
  const struct OneloopAttRef *att_ref,
  const struct OneloopAttState *att_state,
  const struct Gains2ndOrder3 *k_rate_e,
  float *nu_rate)
{
  float omega_err[3];
  float omega_d_err[3];

  scaled_error_nd(3, omega_err, att_ref->att_d, att_state->att_d, k_rate_e->k1);
  scaled_error_nd(3, omega_d_err, att_ref->att_2d att_state->att_2d, k_rate_e->k2);

  // Compute virtual command
  for (uint8_t i = 0; i < 3; i++) {
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
 * @param[in]  dt         Time step [s]
 * @param[in]  att_ref    Pointer to attitude reference states (attitude, angular rate, acceleration, jerk)
 * @param[in]  att_state  Pointer to current attitude states (attitude, angular rate, acceleration)
 * @param[in]  k_att_e    Pointer to structured gains for error scaling (k1, k2, k3 per axis)
 * @param[out] nu_att     Pointer to output virtual control command vector [3]
 */
void error_controller_attitude(
  float dt,
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
  scaled_error_nd(3, att_2d_err, att_ref->att_2d att_state->att_2d, k_rate_e->k3);

  // Compute virtual command
  for (uint8_t i = 0; i < 3; i++) {
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
 * @param[in]  dt        Time step [s]
 * @param[in]  pos_ref   Pointer to position reference states (pos, vel, accel, jerk)
 * @param[in]  pos_state Pointer to current position states (pos, vel, accel)
 * @param[in]  k_pos_e   Pointer to structured 3rd-order gains for error scaling (k1, k2, k3 per axis)
 * @param[out] nu_pos    Pointer to output virtual command vector [2]
 */
void error_controller_position(
  float dt,
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
  for (uint8_t i = 0; i < 2; i++) {
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
 * @param[in]  dt        Time step [s]. It is not modified, but not required to be const.
 * @param[in]  alt_ref   Pointer to altitude reference states (position, velocity, acceleration, jerk)
 * @param[in]  alt_state Pointer to current altitude states (position, velocity, acceleration)
 * @param[in]  k_alt_e   Pointer to scalar 3rd-order gains for error scaling (k1, k2, k3)
 * @param[out] nu_alt    Pointer to output virtual control input scalar
 */
void error_controller_altitude(
  float dt,
  const struct OneloopAltRef *alt_ref,
  const struct OneloopAltState *alt_state,
  const struct Gains3rdOrder1 *k_alt_e,
  float *nu_alt)
{
  float pos_err;
  float vel_err;
  float acc_err;

  scaled_error_nd(1, pos_err, pos_ref->pos, pos_state->pos, k_alt_e->k1);
  scaled_error_nd(1, vel_err, alt_ref->vel, alt_state->vel, k_alt_e->k2);
  scaled_error_nd(1, acc_err, alt_ref->acc, alt_state->acc, k_alt_e->k3);

  // Compute virtual command
  *nu_alt = pos_err + vel_err + acc_err + alt_ref->jer;
}

/**
 * @brief Compute heading control error and generate virtual control input.
 *
 * This function calculates the scalar error for heading angle and angular rate 
 * between the reference states and current states. Each error is scaled by 
 * its corresponding 3rd-order gain. The scaled errors are summed with the 
 * angular acceleration (head_2d) feedforward reference to produce the virtual command output.
 *
 * @param[in]  dt         Time step [s].
 * @param[in]  head_ref   Pointer to heading reference states (head, head_d, head_2d).
 * @param[in]  head_state Pointer to current heading states (head, head_d).
 * @param[in]  k_head_e   Pointer to 3rd-order gains (k1 and k2 gains).
 * @param[out] nu_head    Pointer to output virtual control command (scalar).
 */
void error_controller_heading(
  float dt,
  const struct OneloopAltRef *head_ref,
  const struct OneloopAltState *head_state,
  const struct Gains3rdOrder1 *k_head_e,
  float *nu_head)
{
  float head_err;
  float head_d_err;

  // Compute scaled position error
  scaled_error_nd(1, head_err, head_ref->head, head_state->head, k_head_e->k1);

  // Compute scaled velocity error
  scaled_error_nd(1, head_d_err, head_ref->head_d, head_state->head_d, k_head_e->k2);

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
void compute_gains_2nd_order_3(struct Gains2ndOrder3* gains, struct Poles2ndOrder3* poles)
{
    for (uint8_t i = 0; i < 3; ++i) {
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
void compute_gains_2nd_order_2(struct Gains2ndOrder2* gains, struct Poles2ndOrder2* poles)
{
    for (uint8_t i = 0; i < 2; ++i) {
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
void compute_gains_2nd_order_1(struct Gains2ndOrder1* gains, struct Poles2ndOrdert1* poles)
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
void compute_gains_3rd_order_3(struct Gains3rdOrder3* gains, struct Poles3rdOrder3* poles)
{
    for (uint8_t i = 0; i < 3; ++i) {
        compute_gains_3rd_order_single(&gains->k1[i], &gains->k2[i], &gains->k3[i],
                                      poles->omega_n[i], poles->zeta[i], poles->p3[i]);
    }
}

/**
 * @brief Compute 3rd order gains with 2 outputs from poles
 * 
 * @param[out] gains Output structure for computed gains
 * @param[in] poles Input pole structure (omega_n, zeta, p1)
 */
void compute_gains_3rd_order_2(struct Gains3rdOrder2* gains, struct Poles3rdOrder2* poles)
{
    for (uint8_t i = 0; i < 2; ++i) {
        compute_gains_3rd_order_single(&gains->k1[i], &gains->k2[i], &gains->k3[i],
                                      poles->omega_n[i], poles->zeta[i], poles->p3[i]);
    }
}

/**
 * @brief Compute 3rd order gains with 1 output from poles
 * 
 * @param[out] gains Output structure for computed gains
 * @param[in] poles Input pole structure (omega_n, zeta, p1)
 */
void compute_gains_3rd_order_1(struct Gains3rdOrder1* gains, struct Poles3rdOrdert1* poles)
{
    compute_gains_3rd_order_single(&gains->k1, &gains->k2, &gains->k3,
                                  poles->omega_n, poles->zeta, poles->p3);
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
      init_first_order_low_pass(&filter->state.lp1, tau, 1.0f / PERIODIC_FREQUENCY, x0);
      break;
    case BUTTERWORTH_2:
      init_butterworth_2_low_pass(&filter->state.bw2, tau, 1.0f / PERIODIC_FREQUENCY, x0);
      break;
    case BUTTERWORTH_4:
      init_butterworth_4_low_pass(&filter->state.bw4, tau, 1.0f / PERIODIC_FREQUENCY, x0);
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
  filter->type     = type;
  filter->freq     = fc;
  init_filter_on_type(filter, 0.0);
  filter->meas      = 0.0;
  filter->meas_prev = 0.0;
  filter->out       = 0.0;
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
      update_first_order_low_pass(&filter->state.lp1, input);
      filter->out = filter->meas_filt.lp1.last_out;
      break;
    case BUTTERWORTH_2:
      update_butterworth_2_low_pass(&filter->state.bw2, input);
      filter->out = filter->meas_filt.bw2.o[0];
      break;
    case BUTTERWORTH_4:
      update_butterworth_4_low_pass(&filter->state.bw4, input);
      filter->out = filter->meas_filt.bw4.lp2.o[2];
      break;
    default:
      filter->out = 0.0f;
      break;
  }
}

/** @brief  Propagate the filters */
void oneloop_andi_propagate_filters(void) {
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

  // FIXME: Fetch actuator feedback.
  for (int i = 0; i < ANDI_NUM_ACT_TOTAL; i++) {
      update_filter_on_type(&filt_u[i], 0.0f);
  }
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
static void get_desired_rates_radio_command(float* rate_des, const float* rate_bounds)
{
  rate_des[0] = ((float)radio_control_get(RADIO_ROLL) / MAX_PPRZ) * rate_bounds[1];
  rate_des[1] = ((float)radio_control_get(RADIO_PITCH) / MAX_PPRZ) * rate_bounds[2];
  rate_des[2] = ((float)radio_control_get(cRADIO_YAW) / MAX_PPRZ) * rate_bounds[3];
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
static void get_desired_attitude_radio_command(float* att_des, const float* att_bounds, float heading_rate_bound, float dt)
{
  att_des[0] = ((float)radio_control_get(RADIO_ROLL) / MAX_PPRZ) * att_bounds[0];
  att_des[1] = ((float)radio_control_get(RADIO_PITCH) / MAX_PPRZ) * att_bounds[1];

  get_desired_heading_radio_command(&att_des[2], heading_rate_bound, dt);
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
static void get_desired_heading_radio_command(float* heading_des, float heading_rate_bound, float dt)
{
  float rate_des = ((float)radio_control_get(RADIO_YAW) / MAX_PPRZ) * heading_rate_bound;
  *heading_des += rate_des * dt
  NormRadAngle(heading_des);
}


/** @brief  Function to reconstruct actuator state using first order dynamics */
void get_act_state_oneloop(void)
{
  int8_t i;
  float prev_actuator_state_1l;
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    if(i < ANDI_NUM_ACT){
      prev_actuator_state_1l = actuator_state_1l[i];
      actuator_state_1l[i] = prev_actuator_state_1l + act_dynamics_d[i] * (andi_u[i] - prev_actuator_state_1l);
      if(!autopilot_get_motors_on()){
        actuator_state_1l[i] = 0.0;
      }
      Bound(actuator_state_1l[i],act_min[i], act_max[i]);
    } else {
      actuator_state_1l[i] = oneloop_andi.sta_state.att[i-ANDI_NUM_ACT];
    }
  }
}

/**
 * @brief Computes discrete actuator dynamics from continuous inputs.
 *
 * @param[out] act_dynamics_d Array to store discrete dynamics results.
 * @param[in] act_dynamics Array of continuous actuator dynamics inputs.
 */
static void discretize_act_dynamics(float dt, float* act_dynamics_d, const float* act_dynamics)
{
  for (uint8_t i = 0; i < ANDI_NUM_ACT_TOT; i++){
    act_dynamics_d[i] = 1.0f - exp(-act_dynamics[i] * dt);
    Bound(act_dynamics_d[i], 0.00001f, 1.0f); //Fixme: Are these bounds always valid? Magic numbers
  }
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
 * @param[in] act_max_norm Array of actuator maximum values in normalized units.
 * @param[in] act_min_norm Array of actuator minimum values in normalized units.
 */
static void compute_wls_scaling_factor(float* wls_scaler_u, const float* act_max, const float* act_min, const float* act_max_norm, const float* act_min_norm) {
  for (uint8_t i = 0; i < ANDI_NUM_ACT_TOT; i++){
    float nominator = positive_non_zero(act_max[i] - act_min[i])
    float denominator = positive_non_zero(act_max_norm[i] - act_min_norm[i]);
    wls_scaler_u[i] = nominator / denominator;
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
void evaluate_effectiveness_matrix(float* eff_mat)
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
  float u[4] = {0.0f, 0.0f, 800f, 800f};
  cyclone_fu(v_e, w_e,
             quat, u,
             wind_e, obm_coefficients.data,
             eff_mat);
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
static void get_desired_altitude_radio_command(float* alt_des, float alt_rate_bound, float dt)
{
  float rate_des = ((float)radio_control_get(RADIO_THRUST) / MAX_PPRZ) * alt_rate_bound;
  *alt_des += rate_des * dt;
}

/** @brief Init function of Oneloop ANDI controller  */
void oneloop_andi_init(void)
{ 
  printf("INIT ANDI controller: start \n");
  oneloop_andi.ctrl_type = CONTROL_TYPE_ANDI;
  oneloop_andi.control_mode = CONTROL_MODE_RATE;

  // Initialize poles.
  // FIXME: Make poles dynamically set from airframe file
  p_rate_e  = {.omega_n={15.0, 7.5, 7.5}, .zeta={1.0, 1.0, 1.0}};
  p_rate_rm = {.omega_n={12.0, 6.0, 6.0}, .zeta={1.0, 1.0, 1.0}};
  p_att_e  = {.omega_n={15.0, 7.5, 7.5}, .zeta={1.0, 1.0, 1.0}, .p1={15.0, 7.5, 5.5}};
  p_att_rm = {.omega_n={12.0, 6.0, 6.0}, .zeta={1.0, 1.0, 1.0}, .p1={12.0, 6.0, 6.0}};
  p_pos_e   = {.omega_n={1.0, 1.0}, .zeta={1.0, 1.0}, .p1={1.0, 1.0}};
  p_pos_rm  = {.omega_n={0.8, 0.8}, .zeta={1.0, 1.0}, .p1={0.8, 0.8}};
  p_alt_e   = {.omega_n=1.0, .zeta=1.0, .p1=1.0};
  p_alt_rm  = {.omega_n=0.8, .zeta=1.0, .p1=0.8};
  p_head_e  = {.omega_n=0.5, .zeta=1.0};
  p_head_rm = {.omega_n=0.5, .zeta=1.0};

  // Compute gains from on poles
  compute_gains_2nd_order_3(&k_rate_e, &p_rate_e);
  compute_gains_2nd_order_3(&k_rate_rm, &p_rate_rm);
  compute_gains_3nd_order_3(&k_att_e, &p_att_e);
  compute_gains_3nd_order_3(&k_att_rm, &p_att_rm);
  compute_gains_3nd_order_2(&k_pos_e, &p_pos_e);
  compute_gains_3nd_order_2(&k_pos_rm, &p_pos_rm);
  compute_gains_3nd_order_1(&k_alt_e, &p_alt_e);
  compute_gains_3nd_order_1(&k_alt_rm, &p_alt_rm);
  compute_gains_2nd_order_1(&k_head_e, &p_head_e);
  compute_gains_2nd_order_1(&k_head_rm, &p_head_rm);
  
  // Initialize bounds
  // FIXME: Make bounds dynamically set from airframe file
  att_bounds = {.att={10, 10, 10}.att_d={1000.0, 1000.0, 1000.0}, .att_2d={1000.0, 1000.0, 1000.0}, .att_3d={1000.0, 1000.0, 1000.0}};
  pos_bounds  = {.pos={0, 0}, .vel={1000.0, 1000.0}, .acc={1000.0, 1000.0}, .jerk={1000.0, 1000.0}};
  alt_bounds  = {.alt=0, .vel=1000.0, .acc=1000.0, .jerk=1000.0};
  head_bounds = {.head=0, .head_rate=1000.0, .head_acc=1000.0};

  // Initialize obm coefficients
  // FIXME: Make coefficients set from airframe file
  obm_coefficients = {
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

  // FIXME: Putting a small non zero value is dangerous.
  // Make sure that the dynamics are positive and non-zero
  for (uint8_t i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    act_dynamics[i] = positive_non_zero(act_dynamics[i]);
  }

  discretize_act_dynamics(dt_1l, act_dynamics_d, act_dynamics);

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

  init_filter(&filt_ay,      2.0, BUTTERWORTH_2);
  init_filter(&filt_aispeed, 2.0, BUTTERWORTH_2);

  for (int i = 0; i < ANDI_NUM_ACT_TOT; i++) {
      init_filter(&filt_u[i], 2.0, BUTTERWORTH_2);
  }

  // Initialize references to zero
  float_vect_zero(oneloop_andi.att_ref.att, 3);
  float_vect_zero(oneloop_andi.att_ref.att_d, 3);
  float_vect_zero(oneloop_andi.att_ref.att_2d, 3);
  float_vect_zero(oneloop_andi.att_ref.att_3d, 3);
  float_vect_zero(oneloop_andi.pos_ref.pos, 2);
  float_vect_zero(oneloop_andi.pos_ref.vel, 2);
  float_vect_zero(oneloop_andi.pos_ref.acc, 2);
  float_vect_zero(oneloop_andi.pos_ref.jer, 2);
  oneloop_andi.alt_ref.pos = 0.0f;
  oneloop_andi.alt_ref.vel = 0.0f;
  oneloop_andi.alt_ref.acc = 0.0f;
  oneloop_andi.alt_ref.jer = 0.0f;
  oneloop_andi.head.head = 0.0f;
  oneloop_andi.head.head_d = 0.0f;

  // Initialize controller variables
  float_vect_zero(andi_u, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_du, ANDI_NUM_ACT_TOT);
  float_vect_zero(andi_u_n, ANDI_NUM_ACT_TOT);
  float_vect_zero(actuator_state_1l, ANDI_NUM_ACT_TOT);

  float_vect_zero(nu, ANDI_OUTPUTS);
  float_vect_zero(nu_n, ANDI_OUTPUTS);
  float_vect_zero(nav_target, 3);
  float_vect_zero(nav_target_new, 3);

  // Start telemetry
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_STAB, send_eff_mat_stab_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_GUID, send_eff_mat_guid_oneloop_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE, send_guidance_oneloop_andi);
    // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATOR_STATE, send_oneloop_actuator_state);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_oneloop_debug);
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
  // counter_andi++;
  oneloop_andi.control_mode   = control_mode;
  oneloop_andi.control_type   = control_type;
  printf("ENTER ANDI controller: succes \n");

}


/**
 * @brief Main function that runs the controller and performs control allocation
 * @param in_flight  The drone is in flight
 */
void oneloop_andi_run(enum ControlMode control_mode)
{
  // At beginnig of the loop: (1) Register Attitude, (2) Initialize gains of RM and EC, (3) Calculate Normalization of Actuators Signals, (4) Propagate Actuator Model, (5) Update effectiveness matrix

  // Step 1: Fetch all sensor measurmements.
  get_act_state_oneloop();

  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    act_dyn_ctrl[i] = act_dynamics[i];
  }

  // Register the state of the drone in the variables used in RM and EC
  float attitude_wxyz[4] = stateGetNedToBodyQuat_f();

  // Attitude
  float_eulers_of_quat_zxy(&oneloop_andi.att_state.att, attitude_wxyz);  //FIXME: Convert to quaternion representation
  oneloop_andi_propagate_filters();   //needs to be after update of attitude vector (WHY?)
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
  oneloop_andi.alt_state.pos[2] = stateGetPositionNed_f()->z;      
  oneloop_andi.alt_state.vel[2] = filt_vd.out;      
  oneloop_andi.alt_state.acc[2] = filt_ad.out;
  // FIXME: Todo, add Heading state fetching (which is part of attitude, might be redundant)


  // Guidance Pseudo Control Vector (nu) based on reference model and error controller

  // Assemble pseudo control vector
  // FIXME: Add heading control.
  // FIXME: Add guidance control
  float nu[ANDI_OUTPUTS] = {0.0f}; // An Ae Ad psi p q r
  switch (control_mode) {
    case CONTROL_MODE_RATE:
      {
      float rate_des[3];
      float alt_des;

      get_desired_rates_radio_command(rate_des, att_bounds.att_d);
      reference_model_rate(dt_1l, rate_des, k_rate_rm, &att_bounds, &oneloop_andi.att_ref);
      error_controller_rate(dt_1l, &oneloop_andi.att_ref, &oneloop_andi.att_state, &k_rate_e, &nu[4]);

      get_desired_altitude_radio_command(&alt_des, alt_bounds.vel, dt_1l);
      reference_model_altitude(dt_1l, alt_des, &k_alt_rm, &alt_bounds, &oneloop_andi.alt_ref)
      error_controller_altitude(dt_1l, &oneloop_andi.alt_ref, &oneloop_andi.alt_state, &k_alt_e, &nu[2]);
      break;
      }
    case CONTROL_MODE_ATTITUDE:
      {
      float att_des[3];
      float alt_des;

      get_desired_attitude_radio_command(att_des, att_bounds.att, att_bounds.att_d[2], dt_1l);
      reference_model_attitude(dt_1l, att_des, k_att_rm, &att_bounds, &oneloop_andi.att_ref);
      error_controller_attitude(dt_1l, &oneloop_andi.att_ref, &oneloop_andi.att_state, &k_att_e, &nu[4]);

      get_desired_altitude_radio_command(&alt_des, alt_bounds.vel, dt_1l);
      reference_model_altitude(dt_1l, alt_des, &k_alt_rm, &alt_bounds, &oneloop_andi.alt_ref)
      error_controller_altitude(dt_1l, &oneloop_andi.alt_ref, &oneloop_andi.alt_state, &k_alt_e, &nu[2]);
      break;
      }
    // case CONTROL_MODE_GUIDANCE:
    //   float pos_des[2];
    //   float alt_des;
    //   float rate_des; // fetch from virtual actuators.
    //   reference_model_position(dt_1l, pos_des, &k_pos_rm, &pos_bounds, &oneloop_andi.pos_ref)
    //   error_controller_rate(dt_1l, &oneloop_andi.pos_ref, &oneloop_andi.pos_state, &k_pos_e, &nu[0]);
    //   reference_model_altitude(dt_1l, alt_des, &k_alt_rm, &alt_bounds, &oneloop_andi.alt_ref)
    //   error_controller_altitude(dt_1l, &oneloop_andi.alt_ref, &oneloop_andi.alt_state, &k_alt_e, &nu[2]);
    //   reference_model_rate(dt_1l, rate_des, k_rate_rm, &att_bounds, &oneloop_andi.att_ref);
    //   error_controller_rate(dt_1l, &oneloop_andi.att_ref, &oneloop_andi.att_state, &k_rate_e, &nu[4]);
    //   break;
    default:
      break;
      // handle invalid control_mode (now do nothing)
  }

  // Control Allocation
  // FIXME: Dynamically compute WLS bounds based on current actuator position.
  // FIXME: Put this part in its own function?
  evaluate_effectiveness_matrix(eff_mat);

  float wls_scaler_u[ANDI_ACT_NUM_TOTAL];
  compute_wls_scaling_factors(wls_scaler_u, act_max, act_min, act_max_norm, act_mix_norm);
  for (uint8_t i = 0; i < ANDI_OUTPUTS; i++) {
    for (uint8_t j = 0; j < ANDI_NUM_ACT_TOT; j++) {
      eff_mat[i * ANDI_NUM_ACT_TOT + j] *= scaler[j];
    }
  }
  for (uint8_t i = 0; i < ANDI_OUTPUTS; i++) {
    bwls_1l[i] = eff_mat[i];
  }
  // WLS Control Allocator
  wls_alloc(&wls_one_p, bwls_1l, 0, 0, 10);
  for (i = 0; i < ANDI_NUM_ACT_TOTAL; i++) {
    andi_du[i] = scalar[i] * wls_one_p.u[i];
  }
 
  //FIXME: Convert du to u here.

  // Bound the inputs to the actuators
  for (i = 0; i < ANDI_NUM_ACT_TOT; i++) {
    Bound(andi_du[i], act_min[i], act_max[i]);
  }

  // Commit the actuator command
  for (i = 0; i < ANDI_NUM_ACT; i++) {
    commands[i] = (int16_t) andi_du[i];
  }
}
