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
#include "firmwares/rotorcraft/stabilization/stabilization_andi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
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

#include <stdio.h>
#if INS_EXT_POSE
#include "modules/ins/ins_ext_pose.h"
#endif

#ifdef STABILIZATION_ANDI_ACT_IS_SERVO
const bool   ACTUATOR_IS_SERVO[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_IS_SERVO;
#else
const bool   ACTUATOR_IS_SERVO[ANDI_NUM_ACT] = {0};
#endif

#ifdef STABILIZATION_ANDI_ACT_DYNAMICS
const float  ACTUATOR_DYNAMICS[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_DYNAMICS;
#else
#error "You must specify the actuator dynamics"
#endif

#if defined(STABILIZATION_ANDI_ACT_MAX) && defined(STABILIZATION_ANDI_ACT_MIN)
const float ACTUATOR_MAX[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_MAX;
const float ACTUATOR_MIN[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_MIN;
#else
#error "You must specify the actuator limits: STABILIZATION_ANDI_ACT_MAX and STABILIZATION_ANDI_ACT_MIN"
#endif

#if defined(STABILIZATION_ANDI_ACT_RATE_MAX) && defined(STABILIZATION_ANDI_ACT_RATE_MIN)
const float ACTUATOR_D_MAX[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_RATE_MAX;
const float ACTUATOR_D_MIN[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_RATE_MIN;
#else
#error "You must specify the actuator limits: STABILIZATION_ANDI_ACT_RATE_MAX and STABILIZATION_ANDI_ACT_RATE_MIN"
#endif

#if defined STABILIZATION_ANDI_ACT_PREF
const float ACTUATOR_PREF[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_PREF;
#else
const float ACTUATOR_PREF[ANDI_NUM_ACT] = {0.0f};
#endif

#if defined STABILIZATION_ANDI_RC_RATE_MAX
const float RC_RATE_MAX[ANDI_OUTPUTS] = STABILIZATION_ANDI_RC_RATE_MAX;
#else
const float RC_RATE_MAX[ANDI_OUTPUTS] = {[0 ... ANDI_OUTPUTS - 1] = 5.0f};
#endif

#ifdef PERIODIC_FREQUENCY
const float SAMPLE_TIME = 1.0f / PERIODIC_FREQUENCY;
#else
#error "Periodic frequency is not defined."
#endif

#if ANDI_NUM_ACT != WLS_N_U_MAX
#error "Matrix-WLS_N_U_MAX is not equal to the number of actuators: define WLS_N_U_MAX == ANDI_NUM_ACT in airframe file"
#endif
#if ANDI_OUTPUTS != WLS_N_V_MAX
#error "Matrix-WLS_N_V_MAX is not equal to the number of controlled axis: define WLS_N_V_MAX == ANDI_OUTPUTS in airframe file"
#endif
struct WLS_t wls_stab_p = {
  .nu       = ANDI_NUM_ACT,
  .nv       = ANDI_OUTPUTS,
  .gamma_sq = 1000.0,
#ifdef STABILIZATION_ANDI_WLS_WV
  .Wv       = STABILIZATION_ANDI_WLS_WV,
#else
  .Wv       = {1000.0f, 1000.0f, 1.0f, 100.0f},
#endif
#ifdef STABILIZATION_ANDI_WLS_WU
  .Wu       = STABILIZATION_ANDI_WLS_WU,
#else
  .Wu       = {[0 ... ANDI_NUM_ACT - 1] = 1.0f},
#endif
  .u_pref   = {0.0f},
  .u_min    = {0.0f},
  .u_max    = {0.0f},
  .PC       = 0.0f,
  .SC       = 0.0f,
  .iter     = 0
};

// Declaration of Reference Model and Error Controller Poles
// struct PolesOrder2Vect3 andi_p_rate_ec = {.omega_n={.x=15.0, .y=15.0, .z=15.0}, .zeta={.x=1.0, .y=1.0, .z=1.0}};
// struct PolesOrder2Vect3 andi_p_rate_rm = {.omega_n={.x=12.0, .y=12.0, .z=12.0}, .zeta={.x=1.0, .y=1.0, .z=1.0}};
// struct PolesOrder3Vect3 andi_p_att_ec = {.omega_n={.x=7.5, .y=7.5, .z=7.5}, .zeta={.x=1.0, .y=1.0, .z=1.0}, .p1={.x=7.5, .y=7.5, .z=7.5}};
// struct PolesOrder3Vect3 andi_p_att_rm = {.omega_n={.x=6.0, .y=6.0, .z=6.0}, .zeta={.x=1.0, .y=1.0, .z=1.0}, .p1={.x=6.0, .y=6.0, .z=6.0}};
// float andi_p_thrust_ec = 40.0f; // omega_n
// float andi_p_thrust_rm = 32.0f; // omega_n
struct PolesOrder2Vect3 andi_p_rate_ec = {
  .omega_n={
    .x=STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_N_X, 
    .y=STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_N_Y, 
    .z=STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_N_Z}, 
  .zeta={
    .x=STABILIZATION_ANDI_POLE_RATE_EC_ZETA_X, 
    .y=STABILIZATION_ANDI_POLE_RATE_EC_ZETA_Y, 
    .z=STABILIZATION_ANDI_POLE_RATE_EC_ZETA_Z}
};
struct PolesOrder2Vect3 andi_p_rate_rm = {
  .omega_n={
    .x=STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_N_X, 
    .y=STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_N_Y, 
    .z=STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_N_Z}, 
  .zeta={
    .x=STABILIZATION_ANDI_POLE_RATE_RM_ZETA_X, 
    .y=STABILIZATION_ANDI_POLE_RATE_RM_ZETA_Y, 
    .z=STABILIZATION_ANDI_POLE_RATE_RM_ZETA_Z}
};
struct PolesOrder3Vect3 andi_p_att_ec = {
    .omega_n={
    .x=STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_X, 
    .y=STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_Y, 
    .z=STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_Z}, 
  .zeta={
    .x=STABILIZATION_ANDI_POLE_ATT_EC_ZETA_X, 
    .y=STABILIZATION_ANDI_POLE_ATT_EC_ZETA_Y, 
    .z=STABILIZATION_ANDI_POLE_ATT_EC_ZETA_Z},
  .p1={
    .x=STABILIZATION_ANDI_POLE_ATT_EC_P1_X,
    .y=STABILIZATION_ANDI_POLE_ATT_EC_P1_Y,
    .z=STABILIZATION_ANDI_POLE_ATT_EC_P1_Z}
};
struct PolesOrder3Vect3 andi_p_att_rm = {
    .omega_n={
    .x=STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_X, 
    .y=STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_Y, 
    .z=STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_Z}, 
  .zeta={
    .x=STABILIZATION_ANDI_POLE_ATT_RM_ZETA_X, 
    .y=STABILIZATION_ANDI_POLE_ATT_RM_ZETA_Y, 
    .z=STABILIZATION_ANDI_POLE_ATT_RM_ZETA_Z},
  .p1={
    .x=STABILIZATION_ANDI_POLE_ATT_RM_P1_X,
    .y=STABILIZATION_ANDI_POLE_ATT_RM_P1_Y,
    .z=STABILIZATION_ANDI_POLE_ATT_RM_P1_Z}
};
float andi_p_thrust_ec = STABILIZATION_ANDI_POLE_THRUST_EC;
float andi_p_thrust_rm = STABILIZATION_ANDI_POLE_THRUST_RM;

struct GainsOrder2Vect3 andi_k_rate_ec;
struct GainsOrder2Vect3 andi_k_rate_rm;
struct GainsOrder3Vect3 andi_k_att_ec;
struct GainsOrder3Vect3 andi_k_att_rm;
float andi_k_thrust_ec;
float andi_k_thrust_rm;

float actuator_meas[ANDI_NUM_ACT];

// Filter variables
struct AttFilter attitude_filter_meas;
struct AttFilter attitude_filter_sync;
Butterworth2LowPass thrust_filter_meas;
Butterworth2LowPass thrust_filter_sync;
Butterworth2LowPass actuator_filters[ANDI_NUM_ACT];

// Raw state measurement variables
struct FloatRates rates_prev;

// Filtered state variables
struct AttQuat attitude_state;
float thrust_state;
float actuator_state[ANDI_NUM_ACT];
float andi_u[ANDI_NUM_ACT];

// Reference model variables
struct AttQuat attitude_ref;
struct ThrustRef thrust_ref;

struct FloatRates rates_des;
struct FloatQuat attitude_des;
float thrust_des;

// Bounds
struct AttQuat attitude_bounds;
struct ThrustRef thrust_bounds;

// Controller variables
float ce_mat[ANDI_OUTPUTS * ANDI_NUM_ACT];

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_wls_v_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_v("one", &wls_stab_p, trans, dev); 
}
static void send_wls_u_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_u("one", &wls_stab_p, trans, dev); 
}
static void send_eff_mat_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0f;
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID, 
                                  ANDI_NUM_ACT, &ce_mat[0 * ANDI_NUM_ACT],
                                  ANDI_NUM_ACT, &ce_mat[1 * ANDI_NUM_ACT],
                                  ANDI_NUM_ACT, &ce_mat[2 * ANDI_NUM_ACT], 
                                  ANDI_NUM_ACT, &ce_mat[3 * ANDI_NUM_ACT],
                                             1, &zero);
}
static void send_stab_attitude_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                                              4, (float*)&attitude_des,
                                              4, (float*)&attitude_state.att,
                                              4, (float*)&attitude_ref.att,
                                              3, (float*)&attitude_state.att_d,
                                              3, (float*)&attitude_ref.att_d,
                                              3, (float*)&attitude_state.att_2d,
                                              3, (float*)&attitude_ref.att_2d,
                                              3, (float*)&attitude_ref.att_3d,
                                   ANDI_NUM_ACT, andi_u);                                      
}

static void send_stab_thrust_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_THRUST(trans, dev, AC_ID,
                                              &thrust_des,
                                              &thrust_ref.thrust,
                                              &thrust_state,
                                              &thrust_ref.thrust_d);                                      
}
#endif //PERIODIC_TELEMETRY


// #ifdef USE_ACTUATOR_FEEDBACK
// T4 Actuator feedback handling
struct ActuatorsT4In actuators_t4_obs;
abi_event actuators_t4_in_event;
static void actuators_t4_in_callback(uint8_t sender_id, struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr);

/**
 * @brief Callback function to handle incoming actuator telemetry data.
 *
 * This function copies the data from the input struct pointer `actuators_t4_in_ptr`
 * into a local static instance `actuator_obs` for further processing or monitoring.
 *
 * @param sender_id Identifier of the sender of the telemetry data.
 * @param actuators_t4_in_ptr Pointer to the incoming ActuatorsT4In struct containing
 *                            ESC telemetry, servo angles, loads, and other actuator info.
 * @param actuators_t4_extra_data_in_ptr Pointer to additional extra actuator data (unused).
 */
static void actuators_t4_in_callback(uint8_t sender_id __attribute__((unused)), struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr __attribute__((unused)))
{
  // Copy the entire ActuatorsT4In struct from the pointer to actuator_obs
  memcpy(&actuators_t4_obs, actuators_t4_in_ptr, sizeof(struct ActuatorsT4In));
}

/**
 * @brief Extract actuator states from ActuatorsT4In struct.
 *
 * This function converts selected actuator telemetry data from the 
 * input struct into floating-point values representing angles in radians
 * and rotational speeds in radians per second, storing them in the 
 * provided actuator_state array.
 *
 * FIXME: Consider adding a mutex to avoid data races if this function 
 *        is called concurrently with data updates.
 * FIXME: Avoid hardcoding indices and conversion factors; ask Erik for possible solution.
 *
 * @param[out] actuator_state Array of floats with size ANDI_NUM_ACT where the
 *                           converted actuator states will be stored. The caller
 *                           must allocate this.
 * @param[in] actuators_t4_in_ptr Pointer to the input struct containing actuator telemetry,
 *                                including servo angles (in 1e-2 degrees) and ESC RPM.
 * FIXME: The units of the 'rpm' feedback are rad/s, very confusing since rpm is a unit of its own (1/s)
 */
static void fetch_actuators_t4(float actuator_meas[ANDI_NUM_ACT], const struct ActuatorsT4In *actuators_t4_in_ptr)
{
  actuator_meas[0] = (float)actuators_t4_in_ptr->servo_1_angle / 18000 * M_PI; // Convert 1e-2 deg to rad
  actuator_meas[1] = (float)actuators_t4_in_ptr->servo_2_angle / 18000 * M_PI;
  actuator_meas[2] = (float)actuators_t4_in_ptr->esc_1_rpm * 2 * M_PI / 60;
  actuator_meas[2] *= actuator_meas[2]; // square motor rpm
  actuator_meas[3] = (float)actuators_t4_in_ptr->esc_2_rpm * 2 * M_PI / 60;
  actuator_meas[3] *= actuator_meas[3]; // square motor rpm
}

float act_dynamics_discrete[ANDI_NUM_ACT];

/**
 * @brief Retrieve actuator measurement from first order model.
 *
 * This function updates the passed array `actuator_meas` by applying a discrete
 * actuator dynamics filter, combining previous measurements and control inputs.
 *
 * @param[in,out] actuator_meas Array of floats with size ANDI_NUM_ACT to store
 *                              the actuator measurement results in radians (angles)
 *                              and radians per second (rotational speeds).
 * @param[in] andi_u Array of floats with size ANDI_NUM_ACT with previous actuator
 *                   commands.
 */
static void apply_actuator_dynamics_filter(float actuator_meas[ANDI_NUM_ACT], float andi_u[ANDI_NUM_ACT])
{
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    actuator_meas[i] = actuator_meas[i] * (1 - act_dynamics_discrete[i]) + andi_u[i] * act_dynamics_discrete[i];
    Bound(actuator_meas[i], ACTUATOR_MIN[i], ACTUATOR_MAX[i]);
  }
}
// #endif //USE_ACTUATOR_FEEDBACK

/**
 * @brief Retrieve the current actuator state measurements.
 *
 * This function updates the passed array `actuator_meas` with the latest
 * actuator data. The behavior depends on the compilation flag `USE_ACTUATOR_FEEDBACK`:
 *
 * - If `USE_ACTUATOR_FEEDBACK` is defined, it calls `fetch_actuators_t4` using the global
 *   variable `actuators_t4_obs` which holds the current actuator telemetry observation.
 *
 * - Otherwise, it calls `apply_actuator_dynamics_filter` which updates `actuator_meas`
 *   using the global control input array `andi_u` and the global discrete actuator
 *   dynamics array `act_dynamics_discrete`.
 *
 * @param[in,out] actuator_meas Array of floats with size ANDI_NUM_ACT to store
 *                              the actuator measurement results in radians (angles)
 *                              and radians per second (rotational speeds).
 *                              This array also functions as input and should contain the 
 *                              previous actuator state in case a model is used.
 * 
 * FIXME: Thrust control does not work with USE_ACTUATOR_FEEDBACK, needs to be fixed.
 */
static void get_actuator_measurement(float actuator_meas[ANDI_NUM_ACT])
{
  float actuator_meas_tmp_1[ANDI_NUM_ACT];
  float actuator_meas_tmp_2[ANDI_NUM_ACT];
  float_vect_copy(actuator_meas_tmp_2, actuator_meas, ANDI_NUM_ACT);

// #ifdef USE_ACTUATOR_FEEDBACK
  // FIXME: Using actuator feedback does not work at the moment
  fetch_actuators_t4(actuator_meas_tmp_1, &actuators_t4_obs);
// #else
  apply_actuator_dynamics_filter(actuator_meas_tmp_2, andi_u);
// #endif
  actuator_meas[0] =  actuator_meas_tmp_1[0];
  actuator_meas[1] = -actuator_meas_tmp_1[1];
  actuator_meas[2] =  actuator_meas_tmp_2[2];
  actuator_meas[3] =  actuator_meas_tmp_2[3];
}


struct FloatQuat stabilization_attitude_from_rc(void)
{
  const float RC_ANGLE_MAX[3] = {0.5f, 0.5f, 0.5f};
  struct FloatEulers attitude_sp;
  FLOAT_EULERS_ZERO(attitude_sp);
  attitude_sp.phi = radio_control.values[RADIO_ROLL] * RC_ANGLE_MAX[0] / MAX_PPRZ;
  attitude_sp.theta = radio_control.values[RADIO_PITCH] * RC_ANGLE_MAX[1] / MAX_PPRZ;
  attitude_sp.psi = radio_control.values[RADIO_YAW] * RC_ANGLE_MAX[2] / MAX_PPRZ;
  struct FloatQuat att_sp_quat;
  float_quat_of_eulers(&att_sp_quat, &attitude_sp);
  return att_sp_quat;
}


struct GainsOrder3Vect3 compute_reference_gains_order_3_vect_3(const struct PolesOrder3Vect3* poles)
{
  float rm_k1_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n * p1) / (omega_n * omega_n + 2.0f * zeta * omega_n * p1); }
  float rm_k2_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n + 2.0f * zeta * omega_n * p1) / (2.0f * zeta * omega_n + p1); }
  float rm_k3_order3_f(const float omega_n, const float zeta, const float p1) { return 2.0f * zeta * omega_n + p1; }

  struct GainsOrder3Vect3 gains;
  gains.k1.x = rm_k1_order3_f(poles->omega_n.x, poles->zeta.x, poles->p1.x);
  gains.k1.y = rm_k1_order3_f(poles->omega_n.y, poles->zeta.y, poles->p1.y);
  gains.k1.z = rm_k1_order3_f(poles->omega_n.z, poles->zeta.z, poles->p1.z);

  gains.k2.x = rm_k2_order3_f(poles->omega_n.x, poles->zeta.x, poles->p1.x);
  gains.k2.y = rm_k2_order3_f(poles->omega_n.y, poles->zeta.y, poles->p1.y);
  gains.k2.z = rm_k2_order3_f(poles->omega_n.z, poles->zeta.z, poles->p1.z);

  gains.k3.x = rm_k3_order3_f(poles->omega_n.x, poles->zeta.x, poles->p1.x);
  gains.k3.y = rm_k3_order3_f(poles->omega_n.y, poles->zeta.y, poles->p1.y);
  gains.k3.z = rm_k3_order3_f(poles->omega_n.z, poles->zeta.z, poles->p1.z);
  return gains;
}

struct GainsOrder2Vect3 compute_reference_gains_order_2_vect_3(const struct PolesOrder2Vect3* poles)
{
  float rm_k1_order2_f(const float omega_n, const float zeta) { return omega_n / (2.0f * zeta); }
  float rm_k2_order2_f(const float omega_n, const float zeta) { return 2.0f * zeta * omega_n; }

  struct GainsOrder2Vect3 gains;
  gains.k1.x = rm_k1_order2_f(poles->omega_n.x, poles->zeta.x);
  gains.k1.y = rm_k1_order2_f(poles->omega_n.y, poles->zeta.y);
  gains.k1.z = rm_k1_order2_f(poles->omega_n.z, poles->zeta.z);

  gains.k2.x = rm_k2_order2_f(poles->omega_n.x, poles->zeta.x);
  gains.k2.y = rm_k2_order2_f(poles->omega_n.y, poles->zeta.y);
  gains.k2.z = rm_k2_order2_f(poles->omega_n.z, poles->zeta.z);
  return gains;
}

struct GainsOrder3Vect3 compute_error_gains_order_3_vect_3(const struct PolesOrder3Vect3* poles)
{
  float ec_k1_order3_f(const float omega_n, const float zeta __attribute__((unused)), const float p1) { return (omega_n * omega_n * p1); }
  float ec_k2_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n + 2.0f * zeta * omega_n * p1); }
  float ec_k3_order3_f(const float omega_n, const float zeta, const float p1) { return 2.0f * zeta * omega_n + p1; }

  struct GainsOrder3Vect3 gains;
  gains.k1.x = ec_k1_order3_f(poles->omega_n.x, poles->zeta.x, poles->p1.x);
  gains.k1.y = ec_k1_order3_f(poles->omega_n.y, poles->zeta.y, poles->p1.y);
  gains.k1.z = ec_k1_order3_f(poles->omega_n.z, poles->zeta.z, poles->p1.z);

  gains.k2.x = ec_k2_order3_f(poles->omega_n.x, poles->zeta.x, poles->p1.x);
  gains.k2.y = ec_k2_order3_f(poles->omega_n.y, poles->zeta.y, poles->p1.y);
  gains.k2.z = ec_k2_order3_f(poles->omega_n.z, poles->zeta.z, poles->p1.z);

  gains.k3.x = ec_k3_order3_f(poles->omega_n.x, poles->zeta.x, poles->p1.x);
  gains.k3.y = ec_k3_order3_f(poles->omega_n.y, poles->zeta.y, poles->p1.y);
  gains.k3.z = ec_k3_order3_f(poles->omega_n.z, poles->zeta.z, poles->p1.z);
  return gains;
}

struct GainsOrder2Vect3 compute_error_gains_order_2_vect_3(const struct PolesOrder2Vect3* poles)
{
  float ec_k1_order2_f(const float omega_n, const float zeta __attribute__((unused))) { return omega_n * omega_n; }
  float ec_k2_order2_f(const float omega_n, const float zeta) { return 2.0f * zeta * omega_n; }

  struct GainsOrder2Vect3 gains;
  gains.k1.x = ec_k1_order2_f(poles->omega_n.x, poles->zeta.x);
  gains.k1.y = ec_k1_order2_f(poles->omega_n.y, poles->zeta.y);
  gains.k1.z = ec_k1_order2_f(poles->omega_n.z, poles->zeta.z);

  gains.k2.x = ec_k2_order2_f(poles->omega_n.x, poles->zeta.x);
  gains.k2.y = ec_k2_order2_f(poles->omega_n.y, poles->zeta.y);
  gains.k2.z = ec_k2_order2_f(poles->omega_n.z, poles->zeta.z);
  return gains;
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
 * @param[in] bounds     Limits for the desired rates, rates derivatives, and accelerations, in `AttRefEulers`.
 * @param[in,out] att_ref  The reference model states (attitude quaternion, rates, and derivatives),
 *                         updated in place to produce the reference command.
 */
static void generate_reference_rate(
  float dt,
  const struct FloatRates *rate_des,
  const struct GainsOrder2Vect3 *k_rate_rm,
  const struct AttQuat *bounds,
  struct AttQuat *att_ref)
{
    float p_des = rate_des->p;
    float q_des = rate_des->q;
    float r_des = rate_des->r;

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
  const struct FloatQuat *att_des,
  const struct GainsOrder3Vect3 *k_att_rm,
  const struct AttQuat *bounds,
  struct AttQuat *att_ref)
{
    struct FloatQuat att_err; // rotation needed from ref to des
    float_quat_inv_comp_norm_shortest(&att_err, (struct FloatQuat *)&att_ref->att, (struct FloatQuat *)att_des); // FIXME: quaternion library is not const correct.
    
    // factor 2 needed to convert quaternion difference to angular rate (assuming small angles)
    float p_des = k_att_rm->k1.x * att_err.qx * 2;
    float q_des = k_att_rm->k1.y * att_err.qy * 2;
    float r_des = k_att_rm->k1.z * att_err.qz * 2;

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
static void generate_reference_attitude_eulers(
  float dt,
  const struct FloatEulers *att_des,
  const struct GainsOrder3Vect3 *k_att_rm,
  const struct AttEulers *bounds,
  struct AttEulers *att_ref)
{    
    float p_des = k_att_rm->k1.x * (att_des->phi - att_ref->att.phi);
    float q_des = k_att_rm->k1.y * (att_des->theta - att_ref->att.theta);
    float r_des = k_att_rm->k1.z * (att_des->psi - att_ref->att.psi);

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

    att_ref->att.phi += att_ref->att_d.p * dt;    
    att_ref->att.theta += att_ref->att_d.q * dt;    
    att_ref->att.psi += att_ref->att_d.r * dt;    
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
  const struct ThrustRef *bounds,
  struct ThrustRef *thrust_ref)
{
  Bound(thrust_des, 0, bounds->thrust);
  thrust_ref->thrust_d = k_thrust_rm * (thrust_des - thrust_ref->thrust);

  BoundAbs(thrust_des, bounds->thrust_d);
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
 * @param[in] k_rate_ec  Pointer to gain parameters structure, containing proportional and derivative gains.
 *
 * @return A \c FloatVect3 structure representing the computed virtual control input vector.
 */
static struct FloatVect3 control_error_rate(
  const struct AttQuat *att_ref,
  const struct AttQuat *att_state,
  const struct GainsOrder2Vect3 *k_rate_ec)
{
  struct FloatVect3 nu = att_ref->att_3d;

  nu.x += k_rate_ec->k2.x * (att_ref->att_2d.x - att_state->att_2d.x);
  nu.y += k_rate_ec->k2.y * (att_ref->att_2d.y - att_state->att_2d.y);
  nu.z += k_rate_ec->k2.z * (att_ref->att_2d.z - att_state->att_2d.z);

  nu.x += k_rate_ec->k1.x * (att_ref->att_d.p - att_state->att_d.p);
  nu.y += k_rate_ec->k1.y * (att_ref->att_d.q - att_state->att_d.q);
  nu.z += k_rate_ec->k1.z * (att_ref->att_d.r - att_state->att_d.r);

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
 * @param[in] k_att_ec   Pointer to gain parameters struct containing proportional, derivative, and jerk gains.
 *
 * @return A \c FloatVect3 structure representing the computed virtual control input vector.
 */
static struct FloatVect3 control_error_attitude(
  const struct AttQuat *att_ref,
  const struct AttQuat *att_state,
  const struct GainsOrder3Vect3 *k_att_ec)
{
  struct FloatVect3 nu = att_ref->att_3d;

  nu.x += k_att_ec->k3.x * (att_ref->att_2d.x - att_state->att_2d.x);
  nu.y += k_att_ec->k3.y * (att_ref->att_2d.y - att_state->att_2d.y);
  nu.z += k_att_ec->k3.z * (att_ref->att_2d.z - att_state->att_2d.z);

  nu.x += k_att_ec->k2.x * (att_ref->att_d.p - att_state->att_d.p);
  nu.y += k_att_ec->k2.y * (att_ref->att_d.q - att_state->att_d.q);
  nu.z += k_att_ec->k2.z * (att_ref->att_d.r - att_state->att_d.r);

  struct FloatQuat att_err;
  float_quat_inv_comp_norm_shortest(&att_err, (struct FloatQuat *)&att_state->att, (struct FloatQuat *)&att_ref->att); // FIXME: quaternion library is not const correct
  // Multiplication by 2 needed to convert quaternion difference to angular rate (assuming small angles)
  nu.x += k_att_ec->k1.x * att_err.qx * 2;
  nu.y += k_att_ec->k1.y * att_err.qy * 2;
  nu.z += k_att_ec->k1.z * att_err.qz * 2;

  return nu;
}

/**
 * @brief Computes the thrust control command based on desired and current thrust.
 *
 * This function calculates a corrected thrust command using a simple proportional
 * feedback law. The correction term is scaled by the thrust error gain k_thrust_ec
 * to reduce the difference between the desired thrust (thrust_ref->thrust)
 * and the current thrust (thrust_state). The resulting command is based on
 * the desired thrust feedforward input thrust_ref->thrust_d.
 *
 * @param thrust_ref Pointer to a structure containing the desired thrust values.
 * @param thrust_state Current measured thrust value.
 * @param k_thrust_ec Proportional gain applied to the thrust error correction.
 *
 * @return A \c float representing the computed virtual control thrust input.
 */
static float control_error_thrust(
  const struct ThrustRef *thrust_ref,
  const float thrust_state,
  const float k_thrust_ec)
{
  float nu = thrust_ref->thrust_d;
  nu += k_thrust_ec * (thrust_ref->thrust - thrust_state);
  return nu;
}

/**
 * @brief Compute upper bounds for actuator rate commands based on actuator state and constraints.
 *
 * This function calculates the maximum allowable actuator rate commands (`u_d_max`) for each actuator to ensure that:
 * 1. The actuator position does not exceed its specified maximum (`act_max`) in the next timestep.
 * 2. The actuator rate does not exceed its maximum allowed rate (`act_rate_max`).
 * The computed upper bound is the minimum of the position-based rate limit and the maximum rate limit.
 *
 * @param[out] u_d_max Array to store the computed upper bounds for actuator rates.
 * @param[in] act_state Current states (positions) of the actuators.
 * @param[in] act_max Maximum allowable positions for the actuators.
 * @param[in] act_rate_max Maximum allowable rates for the actuators.
 * @param[in] dt Timestep duration over which the rate limits are applied.
 */
static void compute_wls_upper_bounds(float u_d_max[ANDI_NUM_ACT], const float act_state[ANDI_NUM_ACT], const float act_max[ANDI_NUM_ACT], const float act_rate_max[ANDI_NUM_ACT], float dt)
{
    for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++)
    {
        // Calculate max rate allowed to avoid exceeding actuator max position in one timestep
        float rate_limit_pos = (act_max[i] - act_state[i]) / dt;

        // The rate limit considering actuator rate constraints (negative min rate since min rate might be negative)
        float rate_limit_rate = act_rate_max[i];

        // u_d_max is the minimum of the two constraints to avoid surpassing either constraint in the next step
        u_d_max[i] = (rate_limit_pos < rate_limit_rate) ? rate_limit_pos : rate_limit_rate;
    }
}

/**
 * @brief Compute lower bounds for actuator rate commands based on actuator state and constraints.
 *
 * This function calculates the minimum allowable actuator rate commands (`u_d_min`) for each actuator to ensure that:
 * 1. The actuator position does not go below its specified minimum (`act_min`) in the next timestep.
 * 2. The actuator rate does not go below its minimum allowed rate (`act_rate_min`).
 * The computed lower bound is the maximum of the position-based rate limit and the minimum rate limit.
 * Additionally, if the computed lower bound is greater than zero (i.e., actuator cannot reverse direction), it is clamped to zero.
 *
 * @param[out] u_d_min Array to store the computed lower bounds for actuator rates.
 * @param[in] act_state Current states (positions) of the actuators.
 * @param[in] act_min Minimum allowable positions for the actuators.
 * @param[in] act_rate_min Minimum allowable rates for the actuators.
 * @param[in] dt Timestep duration over which the rate limits are applied.
 */
static void compute_wls_lower_bounds(float u_d_min[ANDI_NUM_ACT], const float act_state[ANDI_NUM_ACT], const float act_min[ANDI_NUM_ACT], const float act_rate_min[ANDI_NUM_ACT], float dt)
{
    for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++)
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

// TODO: Compute prefered wls output based on current actuator state.


/**
 * @brief Initialize attitude rate and rate derivative Butterworth low-pass filters.
 * 
 * @param att_filter Struct containing Butterworth filters for attitude rates.
 * @param freq_rates Cutoff frequency for the rate filters (rad/s).
 * @param freq_rates_d Cutoff frequency for the rate derivative filters (rad/s).
 * @param dt Sampling time interval (seconds).
 */
static void init_attitude_filters(struct AttFilter *att_filter_ptr, float freq_rates, float freq_rates_d, float dt)
{
  init_butterworth_2_low_pass(&att_filter_ptr->att_d_filter_p, 1.0f / freq_rates, dt, 0.0);
  init_butterworth_2_low_pass(&att_filter_ptr->att_d_filter_q, 1.0f / freq_rates, dt, 0.0);
  init_butterworth_2_low_pass(&att_filter_ptr->att_d_filter_r, 1.0f / freq_rates, dt, 0.0);
  init_butterworth_2_low_pass(&att_filter_ptr->att_2d_filter_x, 1.0f / freq_rates_d, dt, 0.0);
  init_butterworth_2_low_pass(&att_filter_ptr->att_2d_filter_y, 1.0f / freq_rates_d, dt, 0.0);
  init_butterworth_2_low_pass(&att_filter_ptr->att_2d_filter_z, 1.0f / freq_rates_d, dt, 0.0);
}

/**
 * @brief Initialize a Butterworth low-pass filter for thrust measurement.
 * 
 * @param thrust_filter Butterworth2LowPass filter instance for thrust.
 * @param freq Cutoff frequency of the filter (rad/s).
 * @param dt Sampling time interval (seconds).
 */
static void init_thrust_filter(Butterworth2LowPass *thrust_filter_ptr, float freq, float dt)
{
  init_butterworth_2_low_pass(thrust_filter_ptr, 1.0f / freq, dt, 0.0f);
}

/**
 * @brief Initialize an array of actuator Butterworth low-pass filters.
 * 
 * @param actuator_filters Array of Butterworth2LowPass filter instances for actuators.
 * @param freq Cutoff frequency for all actuator filters (rad/s).
 * @param dt Sampling time interval (seconds).
 */
static void init_actuator_filters(Butterworth2LowPass actuator_filters[ANDI_NUM_ACT], float freq, float dt)
{
  float tau = 1.0f / freq;
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&actuator_filters[i], tau, dt, 0.0);
  }
}

/**
 * @brief Update attitude rate and rate derivative filters with new attitude input data.
 * 
 * @param att_filter Struct holding the Butterworth filters to be updated.
 * @param att_input Pointer to an AttQuat struct containing attitude derivatives.
 */
static void update_attitude_filters(struct AttFilter *att_filter, const struct AttQuat *att_input)
{
  update_butterworth_2_low_pass(&att_filter->att_d_filter_p, att_input->att_d.p);
  update_butterworth_2_low_pass(&att_filter->att_d_filter_q, att_input->att_d.q);
  update_butterworth_2_low_pass(&att_filter->att_d_filter_r, att_input->att_d.r);
  update_butterworth_2_low_pass(&att_filter->att_2d_filter_x, att_input->att_2d.x);
  update_butterworth_2_low_pass(&att_filter->att_2d_filter_y, att_input->att_2d.y);
  update_butterworth_2_low_pass(&att_filter->att_2d_filter_z, att_input->att_2d.z);
}

/**
 * @brief Update array of actuator Butterworth filters with new actuator input values.
 * 
 * @param actuator_filters Array of Butterworth2LowPass filters to update.
 * @param actuator_input Array containing latest actuator input values.
 */
static void update_actuator_filters(Butterworth2LowPass actuator_filters[ANDI_NUM_ACT], const float actuator_input[ANDI_NUM_ACT])
{
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    update_butterworth_2_low_pass(&actuator_filters[i], actuator_input[i]);
  }
}

void stabilization_andi_init(void)
{
  printf("INIT ANDI controller: START \n");

  // Compute gains
  andi_k_rate_ec = compute_error_gains_order_2_vect_3(&andi_p_rate_ec);
  andi_k_rate_rm = compute_reference_gains_order_2_vect_3(&andi_p_rate_rm);
  andi_k_att_ec = compute_error_gains_order_3_vect_3(&andi_p_att_ec);
  andi_k_att_rm = compute_reference_gains_order_3_vect_3(&andi_p_att_rm);
  andi_k_thrust_ec = andi_p_thrust_ec;
  andi_k_thrust_rm = andi_p_thrust_rm;

  print_GainsOrder2Vect3("RATE EC gains:", &andi_k_rate_ec);
  print_GainsOrder3Vect3("ATT EC gains:", &andi_k_att_ec);
  printf("%s: %.3f\n", "THRUST_EC gain", andi_k_thrust_ec);

  print_GainsOrder2Vect3("RATE RM gains:", &andi_k_rate_rm);
  print_GainsOrder3Vect3("ATT RM gains:", &andi_k_att_rm);
  printf("%s: %.3f\n", "THRUST_RM gain", andi_k_thrust_rm);

  // Initialize state variables
  rates_prev.p = 0.0f;
  rates_prev.q = 0.0f;
  rates_prev.r = 0.0f;
  // This is not required to be initialized before entering andi_run
  float_quat_identity(&attitude_state.att);
  attitude_state.att_d.p = 0.0f;
  attitude_state.att_d.q = 0.0f;
  attitude_state.att_d.r = 0.0f;
  attitude_state.att_2d.x = 0.0f;
  attitude_state.att_2d.y = 0.0f;
  attitude_state.att_2d.z = 0.0f;
  // FIXME: Make a new struct for state attitude without the 3d field
  attitude_state.att_3d.x = 0.0f; // redundant, should never be used
  attitude_state.att_3d.y = 0.0f; // redundant, should never be used
  attitude_state.att_3d.z = 0.0f; // redundant, should never be used

  thrust_state = 0.0f;
  float_vect_zero(actuator_state, ANDI_NUM_ACT);
  float_vect_zero(actuator_meas, ANDI_NUM_ACT);

  // Initialize reference variables
  float_quat_identity(&attitude_ref.att);
  attitude_ref.att_d.p = 0.0f;
  attitude_ref.att_d.q = 0.0f;
  attitude_ref.att_d.r = 0.0f;
  attitude_ref.att_2d.x = 0.0f;
  attitude_ref.att_2d.y = 0.0f;
  attitude_ref.att_2d.z = 0.0f;
  attitude_ref.att_3d.x = 0.0f;
  attitude_ref.att_3d.y = 0.0f;
  attitude_ref.att_3d.z = 0.0f;

  thrust_ref.thrust = 0.0f;
  thrust_ref.thrust_d = 0.0f;

  // Initialize bounds
  float_quat_identity(&attitude_bounds.att);
  attitude_bounds.att_d.p = 100.0f;
  attitude_bounds.att_d.q = 100.0f;
  attitude_bounds.att_d.r = 100.0f;
  attitude_bounds.att_2d.x = 100.0f;
  attitude_bounds.att_2d.y = 100.0f;
  attitude_bounds.att_2d.z = 100.0f;
  attitude_bounds.att_3d.x = 100.0f;
  attitude_bounds.att_3d.y = 100.0f;
  attitude_bounds.att_3d.z = 100.0f;

  thrust_bounds.thrust = 52.0f;
  thrust_bounds.thrust_d = 10.0f;

  // Initial control effectiveness matrix
  struct FloatVect3 body_vel = {.x=0.0f, .y=0.0f, .z=0.0f};
  evaluate_obm_f_stb_u(ce_mat, &attitude_state.att_d, &body_vel, ACTUATOR_PREF);

  // Initialize filters
  init_attitude_filters(&attitude_filter_meas, 10.0f, 10.0f, SAMPLE_TIME);
  init_attitude_filters(&attitude_filter_sync, 10.0f ,10.0f, SAMPLE_TIME);
  init_thrust_filter(&thrust_filter_meas, 10.0f, SAMPLE_TIME);
  init_thrust_filter(&thrust_filter_sync, 10.0f, SAMPLE_TIME);
  init_actuator_filters(actuator_filters, 10.0f, SAMPLE_TIME);

// #ifdef USE_ACTUATOR_FEEDBACK
  // Bind T4 actuator feedback abi message
  AbiBindMsgACTUATORS_T4_IN(ABI_BROADCAST, &actuators_t4_in_event, actuators_t4_in_callback);
// #else
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    act_dynamics_discrete[i] = 1 - exp(-ACTUATOR_DYNAMICS[i] / PERIODIC_FREQUENCY);
  }
// #endif

  // Start telemetry
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_stab_attitude_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_stab_thrust_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_eff_mat_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_stabilization_andi);

  #endif
  printf("INIT ANDI controller: SUCCES \n");
}

void stabilization_andi_enter(void)
{
  // Do nothing for now
  rates_prev.p = 0.0f;
  rates_prev.q = 0.0f;
  rates_prev.r = 0.0f;
  // Reset attitude reference to current attitude
  attitude_ref.att = *stateGetNedToBodyQuat_f();
  attitude_ref.att_d.p = 0.0f;
  attitude_ref.att_d.q = 0.0f;
  attitude_ref.att_d.r = 0.0f;
  attitude_ref.att_2d.x = 0.0f;
  attitude_ref.att_2d.y = 0.0f;
  attitude_ref.att_2d.z = 0.0f;
  attitude_ref.att_3d.x = 0.0f;
  attitude_ref.att_3d.y = 0.0f;
  attitude_ref.att_3d.z = 0.0f;

  // FIXME: Reset thrust reference (not possible without actuator feedback)
  // consider setting thrust feedback to hover thrust, for now do not reset thrust reference


  // Reset actuator state (this is redundant when using actuator feedback)
  float_vect_zero(actuator_state, ANDI_NUM_ACT);
  float_vect_zero(actuator_meas, ANDI_NUM_ACT);

  init_attitude_filters(&attitude_filter_meas, 30.0f, 30.0f, SAMPLE_TIME);
  init_attitude_filters(&attitude_filter_sync, 30.0f ,30.0f, SAMPLE_TIME);

  init_actuator_filters(actuator_filters, 30.0f, SAMPLE_TIME);

  // Initial control effectiveness matrix
  struct FloatVect3 body_vel = {.x=0.0f, .y=0.0f, .z=0.0f};
  evaluate_obm_f_stb_u(ce_mat, &attitude_state.att_d, &body_vel, ACTUATOR_PREF);
}

void stabilization_andi_run(bool use_rate_control, bool in_flight, struct StabilizationSetpoint *stab_setpoint, struct ThrustSetpoint *thrust_setpoint, int32_t *cmd)
{

  // Recompute gains
  // FIXME: Can't this only be done when parameters changed? Maybe using handler?
  andi_k_rate_ec = compute_error_gains_order_2_vect_3(&andi_p_rate_ec);
  andi_k_rate_rm = compute_reference_gains_order_2_vect_3(&andi_p_rate_rm);
  andi_k_att_ec = compute_error_gains_order_3_vect_3(&andi_p_att_ec);
  andi_k_att_rm = compute_reference_gains_order_3_vect_3(&andi_p_att_rm);
  andi_k_thrust_ec = andi_p_thrust_ec;
  andi_k_thrust_rm = andi_p_thrust_rm;
  

  // Fetch current raw sensor measurements
  struct AttQuat attitude_meas;
  attitude_meas.att = *stateGetNedToBodyQuat_f();
  attitude_meas.att_d = *stateGetBodyRates_f();

  // FIXME: First derivative then filtering, could cause rounding errors.
  attitude_meas.att_2d.x = (attitude_meas.att_d.p - rates_prev.p) * PERIODIC_FREQUENCY;
  attitude_meas.att_2d.y = (attitude_meas.att_d.q - rates_prev.q) * PERIODIC_FREQUENCY;
  attitude_meas.att_2d.z = (attitude_meas.att_d.r - rates_prev.r) * PERIODIC_FREQUENCY;
  rates_prev = attitude_meas.att_d;

  get_actuator_measurement(actuator_meas);

  // Motor rpm feedback should be in (rad/s)^2
  // Specific thrust thrust_meas is in m/s^2
  // Dot product with last row of control effectiveness matrix. 
  float thrust_meas = 0;
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
      thrust_meas += ce_mat[3 * ANDI_NUM_ACT + i] * actuator_meas[i] * !ACTUATOR_IS_SERVO[i]; 
      // 3*4 + i -> accessing the last row entries in row-major order
  }
  // Get filtered states
  update_attitude_filters(&attitude_filter_meas, &attitude_meas);
  update_actuator_filters(actuator_filters, actuator_meas);
  // update_butterworth_2_low_pass(&thrust_filter_meas, thrust_meas);

  attitude_state.att = attitude_meas.att;
  attitude_state.att_d.p = get_butterworth_2_low_pass(&attitude_filter_meas.att_d_filter_p);
  attitude_state.att_d.q = get_butterworth_2_low_pass(&attitude_filter_meas.att_d_filter_q);
  attitude_state.att_d.r = get_butterworth_2_low_pass(&attitude_filter_meas.att_d_filter_r);
  attitude_state.att_2d.x = get_butterworth_2_low_pass(&attitude_filter_meas.att_2d_filter_x);
  attitude_state.att_2d.y = get_butterworth_2_low_pass(&attitude_filter_meas.att_2d_filter_y);
  attitude_state.att_2d.z = get_butterworth_2_low_pass(&attitude_filter_meas.att_2d_filter_z);

  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    actuator_state[i] = get_butterworth_2_low_pass(&actuator_filters[i]);
  };

  // thrust_state = get_butterworth_2_low_pass(&thrust_filter_meas);
  thrust_state = thrust_meas;

  // Get setpoints
  if (use_rate_control) {
    rates_des = stab_sp_to_rates_f(stab_setpoint);
    if (in_flight) generate_reference_rate(SAMPLE_TIME, &rates_des, &andi_k_rate_rm, &attitude_bounds, &attitude_ref);
  } else {
    attitude_des = stab_sp_to_quat_f(stab_setpoint);
    // attitude_des = stabilization_attitude_from_rc();
    if (in_flight) generate_reference_attitude(SAMPLE_TIME, &attitude_des, &andi_k_att_rm, &attitude_bounds, &attitude_ref);
  }

  // FIXME: Thrust setpoint can not be of type THRUST_INCR_SP
  // FIXME: Do not hardcode thrust des scaler
  thrust_des = th_sp_to_thrust_f(thrust_setpoint, 0, THRUST_AXIS_Z) * 52;
  generate_reference_thrust(SAMPLE_TIME, thrust_des, andi_k_thrust_rm, &thrust_bounds, &thrust_ref);

  // Time sync references
  update_attitude_filters(&attitude_filter_sync, &attitude_ref);
  // update_butterworth_2_low_pass(&thrust_filter_sync, thrust_ref.thrust);

  struct AttQuat attitude_ref_synced;
  attitude_ref_synced.att = attitude_ref.att;
  attitude_ref_synced.att_d.p = get_butterworth_2_low_pass(&attitude_filter_sync.att_d_filter_p);
  attitude_ref_synced.att_d.q = get_butterworth_2_low_pass(&attitude_filter_sync.att_d_filter_q);
  attitude_ref_synced.att_d.r = get_butterworth_2_low_pass(&attitude_filter_sync.att_d_filter_r);
  attitude_ref_synced.att_2d.x = get_butterworth_2_low_pass(&attitude_filter_sync.att_2d_filter_x);
  attitude_ref_synced.att_2d.y = get_butterworth_2_low_pass(&attitude_filter_sync.att_2d_filter_y);
  attitude_ref_synced.att_2d.z = get_butterworth_2_low_pass(&attitude_filter_sync.att_2d_filter_z);
  attitude_ref_synced.att_3d = attitude_ref.att_3d;

  struct ThrustRef thrust_ref_synced;
  // thrust_ref_synced.thrust = get_butterworth_2_low_pass(&thrust_filter_sync);
  thrust_ref_synced.thrust = thrust_ref.thrust;
  thrust_ref_synced.thrust_d = thrust_ref.thrust_d;

  // Construct pseudo control
  struct FloatVect3 nu_attitude;
  if (use_rate_control) {
    nu_attitude = control_error_rate(&attitude_ref_synced, &attitude_state, &andi_k_rate_ec);
  } else {
    nu_attitude = control_error_attitude(&attitude_ref_synced, &attitude_state, &andi_k_att_ec);
  }
  float nu_thrust = control_error_thrust(&thrust_ref_synced, thrust_state, andi_k_thrust_ec);

  // FIXME: Add state feedback here!
  if (in_flight) {
    wls_stab_p.v[0] = nu_attitude.x;
    wls_stab_p.v[1] = nu_attitude.y;
    wls_stab_p.v[2] = nu_attitude.z;
  } else {
    wls_stab_p.v[0] = 0.0f;
    wls_stab_p.v[1] = 0.0f;
    wls_stab_p.v[2] = 0.0f;
  }
  wls_stab_p.v[3] = nu_thrust;

  // Compute control effectiveness matrix based on current states
  // FIXME: control effectiveness matrix is not shcedules for now
  struct FloatVect3 body_vel = {.x=0.0f, .y=0.0f, .z=0.0f};
  struct FloatRates body_rates = {.p=0.0f, .q=0.0f, .r=0.0f};
  evaluate_obm_f_stb_u(ce_mat, &body_rates, &body_vel, ACTUATOR_PREF);

  float *bwls[ANDI_OUTPUTS];
  for (uint_fast8_t i = 0; i < ANDI_OUTPUTS; i++) {
    bwls[i] = &ce_mat[i * ANDI_NUM_ACT];
  }
  // Calcualte the min and max limits on actuator rate
  compute_wls_lower_bounds(wls_stab_p.u_min, actuator_state, ACTUATOR_MIN, ACTUATOR_D_MIN, SAMPLE_TIME);
  compute_wls_upper_bounds(wls_stab_p.u_max, actuator_state, ACTUATOR_MAX, ACTUATOR_D_MAX, SAMPLE_TIME);
  wls_alloc(&wls_stab_p, bwls, 0, 0, 10);

  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    andi_u[i] = wls_stab_p.u[i] / ACTUATOR_DYNAMICS[i] + actuator_state[i]; // FIXME: Remove relaxation
    // Bound(andi_u[i], ACTUATOR_MIN[i], ACTUATOR_MAX[i]); // This line is extra ensurance, should not be needed is actuator dynamics are correct.
  }


  // Commit actuator commands
  // Resulting commands are in rad for servo and rad/s for motor
  // Paparazzi expects the commands in pprz units (-MAX_PPRZ to MAX_PPRZ for servo, 0 to MAX_PPRZ for motor).
  // FIXME: Do not hardcode actuator layout
  // FIXME: Do not hardcode motor command to rpm factor (and use a better model for this mapping)
  commands[0] = (pprz_t)(andi_u[0] / ACTUATOR_MAX[0] * MAX_PPRZ);
  commands[1] = (pprz_t)(andi_u[1] / ACTUATOR_MAX[1] * MAX_PPRZ);
  commands[2] = (pprz_t)(sqrt(andi_u[2] / ACTUATOR_MAX[2]) * MAX_PPRZ); // FIXME: get this to work, also for non zero neutral servo position.
  commands[3] = (pprz_t)(sqrt(andi_u[3] / ACTUATOR_MAX[3]) * MAX_PPRZ);

  // Update thrust command such that the current is correctly estimated
  // FIXME: DO not hardcode actuator layout
  cmd[COMMAND_THRUST] = 0;
  cmd[COMMAND_THRUST] += (pprz_t)(sqrt(andi_u[2] / ACTUATOR_MAX[2]) * MAX_PPRZ);
  cmd[COMMAND_THRUST] += (pprz_t)(sqrt(andi_u[2] / ACTUATOR_MAX[2]) * MAX_PPRZ);
  cmd[COMMAND_THRUST] /= 2;
}

void stabilization_rate_enter(void)
{
  stabilization_andi_enter();
}

void stabilization_rate_run(bool in_flight, struct StabilizationSetpoint *rate_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stabilization_andi_run(true, in_flight, rate_sp, thrust, cmd);
}

void stabilization_attitude_enter(void)
{
  stabilization_andi_enter();
}

void stabilization_attitude_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stabilization_andi_run(false, in_flight, sp, thrust, cmd);
}

struct StabilizationSetpoint stabilization_rate_read_rc(struct RadioControl *rc)
{
  struct FloatRates rate_sp;
  FLOAT_RATES_ZERO(rate_sp);
  if (ROLL_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.p = rc->values[RC_RATE_P] * RC_RATE_MAX[0] / MAX_PPRZ;
  }
  if (PITCH_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.q = rc->values[RC_RATE_Q] * RC_RATE_MAX[1] / MAX_PPRZ;
  }
  if (YAW_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.r = rc->values[RC_RATE_R] * RC_RATE_MAX[2] / MAX_PPRZ;
  }
  return stab_sp_from_rates_f(&rate_sp);
}