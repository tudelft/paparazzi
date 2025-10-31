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

#ifdef STABILIZATION_ANDI_ACT_IS_SERVO
const bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = STABILIZATION_ANDI_ACT_IS_SERVO;
#else
const bool   actuator_is_servo[ANDI_NUM_ACT_TOT] = {0};
#endif

#ifdef STABILIZATION_ANDI_ACT_DYN
float  actuator_dynamics[ANDI_NUM_ACT_TOT] = STABILIZATION_ANDI_ACT_DYN;
#else
#error "You must specify the actuator dynamics"
#endif

#if defined(STABILIZATION_ANDI_ACT_MAX) && defined(STABILIZATION_ANDI_ACT_MIN)
const float act_max[ANDI_NUM_ACT_TOT] = STABILIZATION_ANDI_ACT_MAX;
const float act_min[ANDI_NUM_ACT_TOT] = STABILIZATION_ANDI_ACT_MIN;
#else
#error "You must specify the actuator limits: STABILIZATION_ANDI_ACT_MAX and STABILIZATION_ANDI_ACT_MIN"
#endif

#if defined(STABILIZATION_ANDI_ACT_RATE_MAX) && defined(STABILIZATION_ANDI_ACT_RATE_MIN)
const float act_rate_max[ANDI_NUM_ACT_TOT] = STABILIZATION_ANDI_ACT_RATE_MAX;
const float act_rate_min[ANDI_NUM_ACT_TOT] = STABILIZATION_ANDI_ACT_RATE_MIN;
#else
#error "You must specify the actuator limits: STABILIZATION_ANDI_ACT_RATE_MAX and STABILIZATION_ANDI_ACT_RATE_MIN"
#endif

#if ANDI_NUM_ACT_TOT != WLS_N_U_MAX
#error Matrix-WLS_N_U_MAX is not equal to the number of actuators: define WLS_N_U_MAX == ANDI_NUM_ACT_TOT in airframe file
#endif
#if ANDI_OUTPUTS != WLS_N_V_MAX
#error Matrix-WLS_N_V_MAX is not equal to the number of controlled axis: define WLS_N_V_MAX == ANDI_OUTPUTS in airframe file
#endif

// Declaration of Reference Model and Error Controller Poles
struct PolesOrder2Vect3 andi_p_rate_e = {.omega_n={.x=15.0, .y=7.5, .z=7.5}, .zeta={.x=1.0, .y=1.0, .z=1.0}};
struct PolesOrder2Vect3 andi_p_rate_rm = {.omega_n={.x=12.0, .y=6.0, .z=6.0}, .zeta={.x=1.0, .y=1.0, .z=1.0}};
struct PolesOrder3Vect3 andi_p_att_e = {.omega_n={.x=15.0, .y=7.5, .z=7.5}, .zeta={.x=1.0, .y=1.0, .z=1.0}, .p1={.x=15.0, .y=7.5, .z=5.5}};
struct PolesOrder3Vect3 andi_p_att_rm = {.omega_n={.x=12.0, .y=6.0, .z=6.0}, .zeta={.x=1.0, .y=1.0, .z=1.0}, .p1={.x=12.0, .y=6.0, .z=6.0}};
float andi_p_thrust_e = 1.0f; // omega_n
float andi_p_thrust_rm = 1.0f; // omega_n

struct GainsOrder2Vect3 andi_k_rate_e;
struct GainsOrder2Vect3 andi_k_rate_rm;
struct GainsOrder3Vect3 andi_k_att_e;
struct GainsOrder3Vect3 andi_k_att_rm;
float andi_k_thrust_e;
float andi_k_thrust_rm;

// Filter variables
Butterworth2LowPass measurement_filter_lp[3];
Butterworth2LowPass rates_filter_lp[3]
Butterworth2LowPass actuator_filter[ANDI_NUM_ACT];


// State variables
struct AttStateQuat attitude_state;
float ThrustState thrust_state; // specific thrust in m/s^2
float actuator_state[ANDI_NUM_ACT]; // actuator state in SI units

// Reference model variables
struct AttRefQuat attitude_ref;
struct ThrustRef thrust_ref;

// Controller variables
float ce_mat[ANDI_NUM_ACT * ANDI_OUTPUTS];

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
 * @param[in] bounds     Limits for the desired rates, rates derivatives, and accelerations, in `AttRefEulers`.
 * @param[in,out] att_ref  The reference model states (attitude quaternion, rates, and derivatives),
 *                         updated in place to produce the reference command.
 */
static void generate_reference_rate(
  float dt,
  struct FloatRates rate_des,
  const struct GainsOrder2Vect3 *k_rate_rm,
  const struct AttRefQuat *bounds,
  struct AttRefQuat *att_ref)
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
  const struct AttRefQuat *bounds,
  struct AttRefQuat *att_ref)
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
 * @param[in,out] att_ref  Pointer to ThrustRef struct holding thrust reference states.
 */
static void generate_reference_thrust(
  float dt,
  float thrust_des,
  const float k_thrust_rm,
  const float bounds,
  struct ThrustRef *thrust_ref)
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
  const struct AttRefQuat *att_ref,
  const struct AttStateQuat *att_state,
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
  const struct AttRefQuat *att_ref,
  const struct AttStateQuat *att_state,
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
  const struct ThrustRef *thrust_ref,
  const float thrust_state,
  const float k_thrust_e)
{
  float nu = thrust_ref->thrust_d;
  nu += k_thrust_e * (thrust_ref->thrust - thrust_state);
  return nu;
}


void stabilization_andi_init(void)
{
  printf("INIT ANDI controller: start \n");

  // Compute gains
  andi_k_rate_e = compute_gains_order_2_vect_3(&andi_p_rate_e);
  andi_k_rate_rm = compute_gains_order_2_vect_3(&andi_p_rate_rm);
  andi_k_att_e = compute_gains_order_3_vect_3(&andi_p_att_e);
  andi_k_att_rm = compute_gains_order_3_vect_3(&andi_p_att_rm);
  andi_k_thrust_e = andi_p_thrust_e;
  andi_k_thrust_rm = andi_p_thrust_rm;

  // Initialize filters
  init_butterworth_2_low_pass(&measurement_filter_lp[0], tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&measurement_filter_lp[1], tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&measurement_filter_lp[2], tau, sample_time, 0.0);

  init_butterworth_2_low_pass(&actuator_filter_lp[2], tau, sample_time, 0.0);

  tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_P);
  init_butterworth_2_low_pass(&rates_filt_so[0], tau, sample_time, 0.0);
  tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_Q);
  init_butterworth_2_low_pass(&rates_filt_so[1], tau, sample_time, 0.0);
  tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_R);
  init_butterworth_2_low_pass(&rates_filt_so[2], tau, sample_time, 0.0);

}

void stabilization_andi_rate_run(bool in_flight, struct StabilizationSetpoint *rate_setpoint, struct TrustSetpoint *thrust_setpoint, int32_t *cmd)
{
  // Update filters
  
  // Fetch current filtered state
  float_quat_identity(&attitude_state.att);
  
  // Evaluate control effectiveness matrix
  struct FloatVect3 body_vel = {.x=0.0f, .y=0.0f, .z=0.0f};
  evaluate_obm_f_stb_u(fu_mat, &attitude_state.att_d, &body_vel, actuator_state);
}

void stabilization_andi_attitude_run(bool in_flight, struct StabilizationSetpoint *attitude_setpoint, struct TrustSetpoint *thrust_setpoint, int32_t *cmd)
{
  // Update filters
  
  // Fetch current filtered state
  attitude_state.att = stateGetNedToBodyQuat_f();

  // Evaluate control effectiveness matrix
  struct FloatVect3 body_vel = {.x=0.0f, .y=0.0f, .z=0.0f};
  evaluate_obm_f_stb_u(fu_mat, &attitude_state.att_d, &body_vel, actuator_state);
}

