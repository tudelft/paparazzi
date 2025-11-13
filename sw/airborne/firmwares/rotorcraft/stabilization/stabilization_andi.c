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

/*
 * Assumptions:
 * - Airframe is a tiltbody with 2 elevons and 2 motors in a tractor configuration 
 * - Elevons are at index 0 and 1 in the actuator array
 * - Motors are at index 2 and 3 in the actuator array
 * - ESC neutral value corresponds to zero thrust.
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

static inline float rm_k1_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n * p1) / (omega_n * omega_n + 2.0f * zeta * omega_n * p1); }
static inline float rm_k2_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n + 2.0f * zeta * omega_n * p1) / (2.0f * zeta * omega_n + p1); }
static inline float rm_k3_order3_f(const float omega_n, const float zeta, const float p1) { return 2.0f * zeta * omega_n + p1; }
static inline float rm_k1_order2_f(const float omega_n, const float zeta) { return omega_n / (2.0f * zeta); }
static inline float rm_k2_order2_f(const float omega_n, const float zeta) { return 2.0f * zeta * omega_n; }

static inline float ec_k1_order3_f(const float omega_n, const float zeta __attribute__((unused)), const float p1) { return (omega_n * omega_n * p1); }
static inline float ec_k2_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n + 2.0f * zeta * omega_n * p1); }
static inline float ec_k3_order3_f(const float omega_n, const float zeta, const float p1) { return 2.0f * zeta * omega_n + p1; }
static inline float ec_k1_order2_f(const float omega_n, const float zeta __attribute__((unused))) { return omega_n * omega_n; }
static inline float ec_k2_order2_f(const float omega_n, const float zeta) { return 2.0f * zeta * omega_n; }

// External variables (can be dynamically changed)
// FIXME: These should be initialized in the init function through handlers instead of here 
// (might not even need to be initialized depending on how the xml dl_settings work).
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

// Filter variables
float andi_rate_freq_cutoff = STABILIZATION_ANDI_CUTOFF_FREQ_RATE;  // Hz
float andi_accel_freq_cutoff = STABILIZATION_ANDI_CUTOFF_FREQ_RATE;  // Hz
float andi_jerk_freq_cutoff = STABILIZATION_ANDI_CUTOFF_FREQ_RATE;  // Hz

// WLS allocation variables
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
  .u_pref   = {0.0f}, // Must be zero
  .u_min    = {0.0f},
  .u_max    = {0.0f},
  .PC       = 0.0f,
  .SC       = 0.0f,
  .iter     = 0
};

float wls_u_scaler[ANDI_NUM_ACT];
float wls_v_scaler[ANDI_OUTPUTS];

// Controller gains
struct GainsOrder2Vect3 andi_k_rate_ec;
struct GainsOrder2Vect3 andi_k_rate_rm;
struct GainsOrder3Vect3 andi_k_att_ec;
struct GainsOrder3Vect3 andi_k_att_rm;
float andi_k_thrust_ec;
float andi_k_thrust_rm;

// Filter instances
struct FilterVect3 angular_rates_filter_meas;
struct FilterVect3 angular_rates_filter_sync;
struct FilterVect3 angular_accel_filter_meas;
struct FilterVect3 angular_accel_filter_sync;
Butterworth2LowPass thrust_filter_meas;
Butterworth2LowPass thrust_filter_sync;
Butterworth2LowPass actuator_filters[ANDI_NUM_ACT];

// Raw state measurement variables
struct FloatRates rates_prev;
float actuator_meas[ANDI_NUM_ACT];

// State variables
struct AttStateQuat attitude_state;
float thrust_state;
float actuator_state[ANDI_NUM_ACT];

// Reference model variables
struct AttQuat attitude_ref;
struct ThrustRef thrust_ref;
struct ThrustRef thrust_ref_synced; // Fixme: remove, should not be global

// Setpoints
struct FloatRates rates_des;
struct FloatQuat attitude_des;
float thrust_des;

// Bounds
struct AttQuat attitude_bounds;
struct ThrustRef thrust_bounds;

// Controller variables
float ce_mat[ANDI_OUTPUTS * ANDI_NUM_ACT];
float andi_u[ANDI_NUM_ACT];

// Debug variables
float control_output[ANDI_OUTPUTS] = {0.0f};

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_wls_v_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_v("stab", &wls_stab_p, trans, dev); 
}
static void send_wls_u_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  send_wls_u("stab", &wls_stab_p, trans, dev); 
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
                                              &thrust_ref_synced.thrust,
                                              &thrust_state,
                                              &thrust_ref_synced.thrust_d);                                      
}
static void send_debug_vect_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  char *name = "control output";
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID,
                                              strlen(name), name,
                                              ANDI_OUTPUTS, (float*)&control_output);
}
#endif //PERIODIC_TELEMETRY


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
 * FIXME: Avoid hardcoding indices, conversion factors, and inversions; ask Erik for possible solution. => Leave it be for now
 *
 * @param[out] actuator_state Array of floats with size ANDI_NUM_ACT where the
 *                           converted actuator states will be stored. The caller
 *                           must allocate this.
 * @param[in] actuators_t4_in_ptr Pointer to the input struct containing actuator telemetry,
 *                                including servo angles (in 1e-2 degrees) and ESC RPM.
 */
static void fetch_actuators_t4(float actuator_meas[ANDI_NUM_ACT], const struct ActuatorsT4In *actuators_t4_in_ptr)
{
  actuator_meas[0] = (float)actuators_t4_in_ptr->servo_1_angle / 18000 * M_PI; // Convert 1e-2 deg to rad
  actuator_meas[1] = -(float)actuators_t4_in_ptr->servo_6_angle / 18000 * M_PI; 
  actuator_meas[2] = (float)actuators_t4_in_ptr->esc_1_rpm * 2 * M_PI / 60; // Convert rpm to rad/s
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
 * FIXME: Add transmission delay handling
 */
static void apply_actuator_dynamics_filter(float actuator_meas[ANDI_NUM_ACT], float andi_u[ANDI_NUM_ACT])
{
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    actuator_meas[i] = actuator_meas[i] * (1 - act_dynamics_discrete[i]) + andi_u[i] * act_dynamics_discrete[i];
    Bound(actuator_meas[i], ACTUATOR_MIN[i], ACTUATOR_MAX[i]);
  }
}

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
 *        (probably need a proper rmp controller for this)
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
  // choose actuator feedback or model
  actuator_meas[0] = actuator_meas_tmp_1[0]; // elevon left
  actuator_meas[1] = actuator_meas_tmp_1[1]; // elevon right
  actuator_meas[2] = actuator_meas_tmp_2[2]; // motor left
  actuator_meas[3] = actuator_meas_tmp_2[3]; // motor right
}

/** 
 * Compute reference-model gains for a 3rd-order 3D system. 
 * Each axis gain (x, y, z) is computed using rm_k*_order3_f() with omega_n, zeta, and p1 parameters.
 * @param[in] poles Pointer to PolesOrder3Vect3 containing omega_n, zeta, and p1 for each axis.
 * @return Struct containing k1, k2, k3 gains for x, y, z.
 */
struct GainsOrder3Vect3 compute_reference_gains_order_3_vect_3(const struct PolesOrder3Vect3* poles)
{
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

/** 
 * Compute reference-model gains for a 2nd-order 3D system. 
 * Each axis gain (x, y, z) is computed using rm_k*_order2_f() with omega_n and zeta parameters.
 * @param[in] poles Pointer to PolesOrder2Vect3 containing omega_n and zeta for each axis.
 * @return Struct containing k1, k2 gains for x, y, z.
 */
struct GainsOrder2Vect3 compute_reference_gains_order_2_vect_3(const struct PolesOrder2Vect3* poles)
{
  struct GainsOrder2Vect3 gains;
  gains.k1.x = rm_k1_order2_f(poles->omega_n.x, poles->zeta.x);
  gains.k1.y = rm_k1_order2_f(poles->omega_n.y, poles->zeta.y);
  gains.k1.z = rm_k1_order2_f(poles->omega_n.z, poles->zeta.z);

  gains.k2.x = rm_k2_order2_f(poles->omega_n.x, poles->zeta.x);
  gains.k2.y = rm_k2_order2_f(poles->omega_n.y, poles->zeta.y);
  gains.k2.z = rm_k2_order2_f(poles->omega_n.z, poles->zeta.z);
  return gains;
}

/** 
 * Compute error-compensation gains for a 3rd-order 3D system. 
 * Each axis gain (x, y, z) is computed using ec_k*_order3_f() with omega_n, zeta, and p1 parameters.
 * @param[in] poles Pointer to PolesOrder3Vect3 containing omega_n, zeta, and p1 for each axis.
 * @return Struct containing k1, k2, k3 gains for x, y, z.
 */
struct GainsOrder3Vect3 compute_error_gains_order_3_vect_3(const struct PolesOrder3Vect3* poles)
{
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

/** 
 * Compute error-compensation gains for a 2nd-order 3D system. 
 * Each axis gain (x, y, z) is computed using ec_k*_order2_f() with omega_n and zeta parameters.
 * @param[in] poles[ Pointer to PolesOrder2Vect3 containing omega_n and zeta for each axis.
 * @return Struct containing k1, k2 gains for x, y, z.
 */
struct GainsOrder2Vect3 compute_error_gains_order_2_vect_3(const struct PolesOrder2Vect3* poles)
{
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

  // BoundAbs(thrust_ref->thrust_d, bounds->thrust_d);
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
  const struct AttStateQuat *att_state,
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
  const struct AttStateQuat *att_state,
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
  float_quat_inv_comp_norm_shortest(&att_err, &att_state->att, &att_ref->att);
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
 * @param[in] thrust_ref Pointer to a structure containing the desired thrust values.
 * @param[in] thrust_state Current measured thrust value.
 * @param[in] k_thrust_ec Proportional gain applied to the thrust error correction.
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
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    // Calculate max rate allowed to avoid exceeding actuator max position in one timestep
    float rate_limit_pos = (act_max[i] - act_state[i]) / dt;
    u_d_max[i] = (rate_limit_pos < act_rate_max[i]) ? rate_limit_pos : act_rate_max[i];
    Bound(u_d_max[i], 0.0f, INFINITY);
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
    u_d_min[i] = (rate_limit_pos > act_rate_min[i]) ? rate_limit_pos : act_rate_min[i];
    Bound(u_d_min[i], -INFINITY, 0.0f);
  }
}

/**
 * @brief Compute input scaling factors for normalizing weighted least squares.
 * 
 * This function calculates the input scaling factors (`u_scaler`) for each actuator
 * based on the provided minimum (`act_min`) and maximum (`act_max`) actuator values.
 * The scaling factor is computed as the inverse of the range (max - min). If the
 * range is zero, the scaling factor is set to 1.0 to avoid division by zero.
 * 
 * u_norm = u_scaler * u
 * 
 * @param[out] u_scaler Array to store the computed input scaling factors.
 * @param[in] act_min Array of minimum actuator values.
 * @param[in] act_max Array of maximum actuator values.
 */
static void compute_wls_u_scaler(float u_scaler[ANDI_NUM_ACT], const float act_min[ANDI_NUM_ACT], const float act_max[ANDI_NUM_ACT])
{
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    float range = act_max[i] - act_min[i];
    if (range == 0.0f) {
      u_scaler[i] = 1.0f;
    } else {
      u_scaler[i] = 1.0f / range;
    }
  }
}

/**
 * @brief Compute output scaling factors for normalizing weighted least squares outputs.
 *
 * For each output i this sets v_scaler[i] = 1.0f / v[i] when v[i] is non-zero; otherwise v_scaler[i] is set
 * to 1.0f to avoid a division-by-zero. The normalized output used in WLS is then v_norm = v_scaler * v.
 *
 * @param[out] v_scaler Array of length ANDI_NUM_ACT to store the computed inverse scaling factors.
 * @param[in]  v        Array of length ANDI_NUM_ACT containing reference/scaling values used to compute the inverse.
 */
static void compute_wls_v_scaler(float v_scaler[ANDI_NUM_ACT], const float v[ANDI_NUM_ACT])
{
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    if (v[i] == 0.0f) {
      v_scaler[i] = 1.0f;
    } else {
      v_scaler[i] = 1.0f / v[i];
    }
  }
}

/**
 * @brief Initialize a set of Butterworth low-pass filters to zero for 3D vector data.
 * 
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] freq Cutoff frequency for the filters (Hz).
 * @param[in] dt Sampling time interval (seconds).
 * 
 * FIXME: Add support for different filter types.
 */
static void init_filter_vect3(struct FilterVect3 *filter, float freq, float dt)
{
  init_butterworth_2_low_pass(&filter->x, 1.0f / freq, dt, 0.0f);
  init_butterworth_2_low_pass(&filter->y, 1.0f / freq, dt, 0.0f);
  init_butterworth_2_low_pass(&filter->z, 1.0f / freq, dt, 0.0f);
}

/**
 * @brief Initialize a Butterworth low-pass filter to zero.
 * 
 * @param[out] filter Butterworth2LowPass filter instance.
 * @param[in] freq Cutoff frequency of the filter (Hz).
 * @param[in] dt Sampling time interval (seconds).
 * 
 * FIXME: Add support for different filter types.
 */
static void init_filter(Butterworth2LowPass *filter, float freq, float dt)
{
  init_butterworth_2_low_pass(filter, 1.0f / freq, dt, 0.0f);
}

/**
 * @brief Initialize an array of Butterworth low-pass filters to zero.
 * 
 * @param[in] n Number of filters to initialize.
 * @param[out] filter_array Array of Butterworth2LowPass filters to initialize.
 * @param[in] freq Cutoff frequency for the filters (Hz).
 * @param[in] dt Sampling time interval (seconds).
 * 
 * FIXME: Add support for different filter types.
 */
static void init_filter_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], float freq, float dt)
{
  float tau = 1.0f / freq;
  for (uint_fast8_t i = 0; i < n; i++) {
    init_butterworth_2_low_pass(&filter_array[i], tau, dt, 0.0f);
  }
}

/**
 * @brief Update a Butterworth low-pass filter with new input data.
 * 
 * @param[in,out] filter Butterworth2LowPass filter instance to update.
 * @param[in] input New input data to feed into the filter.
 */
static void update_filter(Butterworth2LowPass *filter, float input)
{
  update_butterworth_2_low_pass(filter, input);
}

/**
 * @brief Update 3D vector Butterworth filters with new input data.
 * 
 * @param[in,out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] input Pointer to FloatVect3 struct containing new input data.
 */
static void update_filter_vect3(struct FilterVect3 *filter, const struct FloatVect3 *input)
{
  update_butterworth_2_low_pass(&filter->x, input->x);
  update_butterworth_2_low_pass(&filter->y, input->y);
  update_butterworth_2_low_pass(&filter->z, input->z); 
}

/**
 * @brief Update 3D vector Butterworth filters with new rate input data.
 * 
 * @param[in,out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] input Pointer to FloatRates struct containing new input rate data.
 */
static void update_filter_rates(struct FilterVect3 *filter, const struct FloatRates *input)
{
  update_butterworth_2_low_pass(&filter->x, input->p);
  update_butterworth_2_low_pass(&filter->y, input->q);
  update_butterworth_2_low_pass(&filter->z, input->r); 
}

/**
 * @brief Update an array of Butterworth low-pass filters with new input data.
 * 
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth2LowPass filters to update.
 * @param[in] input_array Array containing new input data for each filter.
 */
static void update_filter_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], const float input_array[restrict n])
{
  for (uint_fast8_t i = 0; i < n; i++) {
    update_butterworth_2_low_pass(&filter_array[i], input_array[i]);
  }
}

/**
 * @brief Reset a Butterworth low-pass filter to a specific value.
 * 
 * @param[out] filter Butterworth2LowPass filter instance to reset.
 * @param[in] value Value to reset the filter to.
 */
static void reset_filter(Butterworth2LowPass *filter, float value)
{
  filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
}

/**
 * @brief Reset 3D vector Butterworth filters to a specific value.
 * 
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatVect3 struct containing the reset value.
 */
static void reset_filter_vect3(struct FilterVect3 *filter, const struct FloatVect3 *value)
{
  filter->x.i[0] = filter->x.i[1] = filter->x.o[0] = filter->x.o[1] = value->x;
  filter->y.i[0] = filter->y.i[1] = filter->y.o[0] = filter->y.o[1] = value->y;
  filter->z.i[0] = filter->z.i[1] = filter->z.o[0] = filter->z.o[1] = value->z;
}

/**
 * @brief Reset 3D vector Butterworth filters to specific rate values.
 * 
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatRates struct containing the reset values.
 */
static void reset_filter_rates(struct FilterVect3 *filter, const struct FloatRates *value)
{
  filter->x.i[0] = filter->x.i[1] = filter->x.o[0] = filter->x.o[1] = value->p;
  filter->y.i[0] = filter->y.i[1] = filter->y.o[0] = filter->y.o[1] = value->q;
  filter->z.i[0] = filter->z.i[1] = filter->z.o[0] = filter->z.o[1] = value->r;
}

/**
 * @brief Reset an array of Butterworth low-pass filters to specific values.
 * 
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth2LowPass filters to reset.
 * @param[in] value_array Array containing reset values for each filter.
 */
static void reset_filter_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], const float value_array[restrict n])
{
  for (uint_fast8_t i = 0; i < n; i++) {
    filter_array[i].i[0]  = filter_array[i].i[1] = filter_array[i].o[0] = filter_array[i].o[1] = value_array[i];
  }
}

/**
 * @brief Retrieve the filtered output from a Butterworth low-pass filter.
 * 
 * @param[in] filter Butterworth2LowPass filter instance.
 * @return Filtered output value.
 */
static float get_filter(const Butterworth2LowPass *filter)
{
  return get_butterworth_2_low_pass(filter);
}

/**
 * @brief Retrieve the filtered output from 3D vector Butterworth filters.
 * 
 * @param[in] filter Struct containing Butterworth filters for x, y, z components.
 * @return FloatVect3 struct containing the filtered output values.
 */
static struct FloatVect3 get_filter_vect3(const struct FilterVect3 *filter)
{
  struct FloatVect3 output;
  output.x = get_butterworth_2_low_pass(&filter->x);
  output.y = get_butterworth_2_low_pass(&filter->y);
  output.z = get_butterworth_2_low_pass(&filter->z);
  return output;
}

/**
 * @brief Retrieve the filtered output from a Butterworth low-pass filter.
 * 
 * @param[in] filter Butterworth2LowPass filter instance.
 * @return Filtered output value.
 */
static struct FloatRates get_filter_rates(const struct FilterVect3 *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_2_low_pass(&filter->x);
  output.q = get_butterworth_2_low_pass(&filter->y);
  output.r = get_butterworth_2_low_pass(&filter->z);
  return output;
}

/**
 * @brief Retrieve the filtered outputs from an array of Butterworth low-pass filters.
 * 
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth2LowPass filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static void get_filter_array(uint8_t n, const Butterworth2LowPass filter_array[restrict n], float output_array[restrict n])
{
  for (uint_fast8_t i = 0; i < n; i++) {
    output_array[i] = get_butterworth_2_low_pass(&filter_array[i]);
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

  // Limit thrust bounds
  thrust_bounds.thrust =52.0f;
  thrust_bounds.thrust_d = 1000.0f;

  // Initial control effectiveness matrix
  struct FloatVect3 body_vel = {.x=0.0f, .y=0.0f, .z=0.0f};
  evaluate_obm_f_stb_u(ce_mat, &attitude_state.att_d, &body_vel, ACTUATOR_PREF);

  // Initialize filters
  init_filter_vect3(&angular_rates_filter_meas, andi_rate_freq_cutoff, SAMPLE_TIME);
  init_filter_vect3(&angular_rates_filter_sync, andi_rate_freq_cutoff, SAMPLE_TIME);
  init_filter_vect3(&angular_accel_filter_meas, andi_accel_freq_cutoff, SAMPLE_TIME);
  init_filter_vect3(&angular_accel_filter_sync, andi_accel_freq_cutoff, SAMPLE_TIME);
  init_filter(&thrust_filter_meas, andi_accel_freq_cutoff, SAMPLE_TIME);
  init_filter(&thrust_filter_sync, andi_accel_freq_cutoff, SAMPLE_TIME);
  init_filter_array(ANDI_OUTPUTS, actuator_filters, andi_jerk_freq_cutoff, SAMPLE_TIME);

  // Bind T4 actuator feedback abi message
  AbiBindMsgACTUATORS_T4_IN(ABI_BROADCAST, &actuators_t4_in_event, actuators_t4_in_callback);

  float_vect_zero(andi_u, ANDI_NUM_ACT);
  // Precompute discrete-time actuator dynamics coefficients
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    act_dynamics_discrete[i] = 1 - exp(-ACTUATOR_DYNAMICS[i] / PERIODIC_FREQUENCY);
  }

  // Start telemetry
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_stab_attitude_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_stab_thrust_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_eff_mat_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_stabilization_andi);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_debug_vect_stabilization_andi);

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

  // Reset actuator model internal state (this is redundant when using actuator feedback)
  float_vect_zero(actuator_state, ANDI_NUM_ACT);
  float_vect_zero(actuator_meas, ANDI_NUM_ACT);

  // Reset filters to current measurements (just zero mainly)
  reset_filter_rates(&angular_rates_filter_meas, &attitude_state.att_d);
  reset_filter_rates(&angular_rates_filter_sync, &attitude_ref.att_d);
  reset_filter_vect3(&angular_accel_filter_meas, &attitude_state.att_2d);
  reset_filter_vect3(&angular_accel_filter_sync, &attitude_ref.att_2d);
  reset_filter(&thrust_filter_meas, 0.0f);
  reset_filter(&thrust_filter_sync, 0.0f);
  reset_filter_array(ANDI_NUM_ACT, actuator_filters, ACTUATOR_PREF);
}

void stabilization_andi_run(bool use_rate_control, bool in_flight, struct StabilizationSetpoint *stab_setpoint, struct ThrustSetpoint *thrust_setpoint, int32_t *cmd)
{

  // Recompute gains
  // FIXME: Can't this only be done when parameters have changed? Maybe using handler?
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

  // NOTE: First derivative then filtering, could cause rounding errors but allows for the use of different filters. Alternatively, deal with double filtered signals in time syncing.
  attitude_meas.att_2d.x = (attitude_meas.att_d.p - rates_prev.p) * PERIODIC_FREQUENCY;
  attitude_meas.att_2d.y = (attitude_meas.att_d.q - rates_prev.q) * PERIODIC_FREQUENCY;
  attitude_meas.att_2d.z = (attitude_meas.att_d.r - rates_prev.r) * PERIODIC_FREQUENCY;
  rates_prev = attitude_meas.att_d;

  get_actuator_measurement(actuator_meas);

  float thrust_meas = evaluate_obm_thrust(actuator_meas);

  // Get filtered states
  update_filter_rates(&angular_rates_filter_meas, &attitude_meas.att_d);
  update_filter_vect3(&angular_accel_filter_meas, &attitude_meas.att_2d);
  update_filter_array(ANDI_NUM_ACT, actuator_filters, actuator_meas);
  update_filter(&thrust_filter_meas, thrust_meas);

  attitude_state.att = attitude_meas.att; // No filtering on attitude
  attitude_state.att_d = get_filter_rates(&angular_rates_filter_meas);
  attitude_state.att_2d = get_filter_vect3(&angular_accel_filter_meas);
  get_filter_array(ANDI_NUM_ACT, actuator_filters, actuator_state);
  thrust_state = get_filter(&thrust_filter_meas);

  // Get setpoints
  if (use_rate_control) {
    rates_des = stab_sp_to_rates_f(stab_setpoint);
    if (in_flight) generate_reference_rate(SAMPLE_TIME, &rates_des, &andi_k_rate_rm, &attitude_bounds, &attitude_ref);
  } else {
    attitude_des = stab_sp_to_quat_f(stab_setpoint);
    if (in_flight) generate_reference_attitude(SAMPLE_TIME, &attitude_des, &andi_k_att_rm, &attitude_bounds, &attitude_ref);
  }

  // FIXME: Thrust setpoint can not be of type THRUST_INCR_SP, this is not enforced.
  // FIXME: Do not hardcode thrust_des scaler
  thrust_des = th_sp_to_thrust_f(thrust_setpoint, 0, THRUST_AXIS_Z) * 52;
  generate_reference_thrust(SAMPLE_TIME, thrust_des, andi_k_thrust_rm, &thrust_bounds, &thrust_ref);

  // Time sync references
  update_filter_rates(&angular_rates_filter_sync, &attitude_ref.att_d);
  update_filter_vect3(&angular_accel_filter_sync, &attitude_ref.att_2d);
  update_filter(&thrust_filter_sync, thrust_ref.thrust);

  struct AttQuat attitude_ref_synced;
  attitude_ref_synced.att = attitude_ref.att;
  attitude_ref_synced.att_d = get_filter_rates(&angular_rates_filter_sync);
  attitude_ref_synced.att_2d = get_filter_vect3(&angular_accel_filter_sync);
  attitude_ref_synced.att_3d = attitude_ref.att_3d;

  // struct ThrustRef thrust_ref_synced;
  thrust_ref_synced.thrust = get_filter(&thrust_filter_sync);
  thrust_ref_synced.thrust_d = thrust_ref.thrust_d;

  // Construct pseudo control
  struct FloatVect3 nu_attitude;
  if (use_rate_control) {
    nu_attitude = control_error_rate(&attitude_ref_synced, &attitude_state, &andi_k_rate_ec);
  } else {
    nu_attitude = control_error_attitude(&attitude_ref_synced, &attitude_state, &andi_k_att_ec);
  }
  float nu_thrust = control_error_thrust(&thrust_ref_synced, thrust_state, andi_k_thrust_ec);

  float nu[ANDI_OUTPUTS];

  // FIXME: Add state feedback here!

  if (in_flight) {
    nu[0] = nu_attitude.x;
    nu[1] = nu_attitude.y;
    nu[2] = nu_attitude.z;
  } else {
    nu[0] = 0.0f;
    nu[1] = 0.0f;
    nu[2] = 0.0f;
  }
  nu[3] = nu_thrust;

  // Compute control effectiveness matrix based on current states
  // FIXME: control effectiveness matrix is not scheduled for now
  // FIXME: body velocity is not measured or filtered for now
  struct FloatVect3 body_vel = {.x=0.0f, .y=0.0f, .z=0.0f};
  struct FloatRates body_rates = {.p=0.0f, .q=0.0f, .r=0.0f};
  evaluate_obm_f_stb_u(ce_mat, &body_rates, &body_vel, ACTUATOR_PREF);

  // Solve control allocation using weighted least squares
  float u_min[ANDI_NUM_ACT];
  float u_max[ANDI_NUM_ACT];
  compute_wls_lower_bounds(u_min, actuator_state, ACTUATOR_MIN, ACTUATOR_D_MIN, SAMPLE_TIME);
  compute_wls_upper_bounds(u_max, actuator_state, ACTUATOR_MAX, ACTUATOR_D_MAX, SAMPLE_TIME);
  compute_wls_u_scaler(wls_u_scaler, ACTUATOR_MIN, ACTUATOR_MAX);
  compute_wls_v_scaler(wls_v_scaler, nu);

  float ce_mat_scaled[ANDI_NUM_ACT][ANDI_OUTPUTS];
  float *bwls[ANDI_NUM_ACT];
  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) { // step through rows
    wls_stab_p.u_min[i] = u_min[i] * wls_u_scaler[i];
    wls_stab_p.u_max[i] = u_max[i] * wls_u_scaler[i];
    wls_stab_p.u_pref[i] = ACTUATOR_PREF[i] * wls_u_scaler[i];
    for (uint_fast8_t j = 0; j < ANDI_OUTPUTS; j++) { // step through columns
      ce_mat_scaled[i][j] = ce_mat[i * ANDI_OUTPUTS + j] * wls_v_scaler[i] / wls_u_scaler[j]; // u_scaler on each column, v_scaler on each row
    }
    bwls[i] = ce_mat_scaled[i];
  }

  for (uint_fast8_t i = 0; i < ANDI_OUTPUTS; i++) {
    wls_stab_p.v[i] = nu[i] * wls_v_scaler[i];
  }
  wls_alloc(&wls_stab_p, bwls, 0, 0, 10);

  for (uint_fast8_t i = 0; i < ANDI_NUM_ACT; i++) {
    andi_u[i] = (wls_stab_p.u[i] / wls_u_scaler[i]) / ACTUATOR_DYNAMICS[i] + actuator_meas[i];
  }
  // Compute control outputs for logging
  for (uint_fast8_t i = 0; i < ANDI_OUTPUTS; i++) {
      control_output[i] = 0.0f;
      for (uint_fast8_t j = 0; j < ANDI_NUM_ACT; j++) {
          control_output[i] += ce_mat[i * ANDI_NUM_ACT + j] * wls_stab_p.u[j];
      }
  }

  // Commit actuator commands
  // Resulting commands are in rad for servo and rad/s for motor
  // Paparazzi expects the commands in pprz units (-MAX_PPRZ to MAX_PPRZ for servo, 0 to MAX_PPRZ for motor).
  // FIXME: Do not hardcode actuator layout
  // FIXME: Do not hardcode motor command to rpm factor (and use a better model for this mapping)
  commands[0] = (pprz_t)(andi_u[0] / ACTUATOR_MAX[0] * MAX_PPRZ);
  commands[1] = (pprz_t)(andi_u[1] / ACTUATOR_MAX[1] * MAX_PPRZ);
  commands[2] = (pprz_t)(sqrt(andi_u[2] / ACTUATOR_MAX[2]) * MAX_PPRZ);
  commands[3] = (pprz_t)(sqrt(andi_u[3] / ACTUATOR_MAX[3]) * MAX_PPRZ);

  // Update thrust command such that the current is correctly estimated
  // FIXME: Do not hardcode actuator layout
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