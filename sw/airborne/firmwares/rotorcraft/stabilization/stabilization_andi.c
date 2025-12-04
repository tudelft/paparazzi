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
 * - ESC neutral value corresponds to zero thrust (usually equals 0).
 * - Motors are controlled in squared rpm (to linearize thrust curve).
 */

/**
 * FIXME: Normalization of WLS allocation is not ideal. The Wu and Wv costs are dependent on the
 * scaling of the control effectiveness matrix, which itself depends on the current state (e.g. airspeed).
 * A better approach would be to normalize the costs based on the maximum achievable control derivatives
 * for each actuator and output.
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
#include "math/wls/wls_alloc.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/rotwing_drone/rotwing_state.h"
#include "modules/core/commands.h"
#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "filters/low_pass_filter_types.h"
#include "filters/complementary_filter.h"
#include "filters/complementary_filter_types.h"
#include "filters/transport_delay.h"
#include "filters/transport_delay_types.h"

#include <stdio.h>
#if INS_EXT_POSE
#include "modules/ins/ins_ext_pose.h"
#endif

#ifdef STABILIZATION_ANDI_SCHEDULE_EFF
const bool SCHEDULE_EFF = STABILIZATION_ANDI_SCHEDULE_EFF;
#else
const bool SCHEDULE_EFF = false;
#endif

#ifdef STABILIZATION_ANDI_USE_STATE_DYNAMICS
const bool USE_STATE_DYNAMICS = STABILIZATION_ANDI_USE_STATE_DYNAMICS;
#else
const bool USE_STATE_DYNAMICS = false;
#endif

#ifdef STABILIZATION_ANDI_RELAX_OBM
const float ANDI_RELAX_OBM = STABILIZATION_ANDI_RELAX_OBM;
#else
const float ANDI_RELAX_OBM = 1.0f;
#endif

#ifdef STABILIZATION_ANDI_ACT_IS_SERVO
const bool ACTUATOR_IS_SERVO[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_IS_SERVO;
#else
const bool ACTUATOR_IS_SERVO[ANDI_NUM_ACT] = {0};
#endif

#ifdef STABILIZATION_ANDI_ACT_DYNAMICS
const float ACTUATOR_DYNAMICS[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_DYNAMICS;
#else
#error "You must specify the actuator dynamics"
#endif

#ifdef STABILIZATION_ANDI_ACT_DELAY
const uint8_t ACTUATOR_DELAY[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_DELAY;
#else
const uint8_t ACTUATOR_DELAY[ANDI_NUM_ACT] = {0};
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

#ifdef STABILIZATION_ANDI_THRUST_MIN
const float THRUST_MIN = STABILIZATION_ANDI_THRUST_MIN;
#else
const float THRUST_MIN = 0.0f;
#endif

#ifdef STABILIZATION_ANDI_THRUST_MAX
const float THRUST_MAX = STABILIZATION_ANDI_THRUST_MAX;
#else
#error "You must specify maximum specific thrust: STABILIZATION_ANDI_THRUST_MAX"
#endif

#ifdef PERIODIC_FREQUENCY
const float SAMPLE_TIME = 1.0f / PERIODIC_FREQUENCY;
#else
#error "Periodic frequency is not defined."
#endif

#if ANDI_NUM_ACT > WLS_N_U_MAX
#error Matrix-WLS_N_U_MAX too small or not defined: define WLS_N_U_MAX >= ANDI_NUM_ACT in airframe file
#endif
#if ANDI_OUTPUTS > WLS_N_V_MAX
#error Matrix-WLS_N_V_MAX too small or not defined: define WLS_N_V_MAX >= ANDI_OUTPUTS in airframe file
#endif

#ifdef STABILIZATION_ANDI_WLS_WV
const float WLS_WV[ANDI_OUTPUTS] = STABILIZATION_ANDI_WLS_WV;
#else
const float WLS_WV[ANDI_OUTPUTS] = {[0 ... ANDI_OUTPUTS - 1] = 1.0f};
#endif

/**
 * Normalized actuator cost for WLS allocation.
 * Each value corresponds to the relative cost of using each actuator, normalized over
 * the possible range of u_dot for that actuator.
 */
#ifdef STABILIZATION_ANDI_WLS_WU
float WLS_WU[ANDI_NUM_ACT] = STABILIZATION_ANDI_WLS_WU;
#else
float WLS_WU[ANDI_NUM_ACT] = {[0 ... ANDI_NUM_ACT - 1] = 1.0f};
#endif

// Function prototypes
static void actuators_t4_in_callback(uint8_t sender_id, struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr);
static void fetch_actuators_t4(float actuator_meas[ANDI_NUM_ACT], const struct ActuatorsT4In *actuators_t4_in_ptr);
static void apply_actuator_dynamics_filter(float actuator_meas[ANDI_NUM_ACT], float u_cmd[ANDI_NUM_ACT]);
static void get_actuator_measurement(float actuator_meas[ANDI_NUM_ACT]);
static struct GainsOrder3Vect3 compute_reference_gains_order_3_vect_3(const struct PolesOrder3Vect3 *poles);
static struct GainsOrder2Vect3 compute_reference_gains_order_2_vect_3(const struct PolesOrder2Vect3 *poles);
static struct GainsOrder3Vect3 compute_error_gains_order_3_vect_3(const struct PolesOrder3Vect3 *poles);
static struct GainsOrder2Vect3 compute_error_gains_order_2_vect_3(const struct PolesOrder2Vect3 *poles);
static void generate_reference_rate(float dt, const struct FloatRates *rate_des, const struct GainsOrder2Vect3 *k_rate_rm, const struct AttQuat *bounds, struct AttQuat *att_ref);
static void generate_reference_attitude(float dt, const struct FloatQuat *att_des, const struct GainsOrder3Vect3 *k_att_rm, const struct AttQuat *bounds, struct AttQuat *att_ref);
static void generate_reference_thrust(float dt, float thrust_des, const float k_thrust_rm, const struct ThrustRef *bounds_min, const struct ThrustRef *bounds_max, struct ThrustRef *thrust_ref);
static struct FloatVect3 control_error_rate(const struct AttQuat *att_ref, const struct AttStateQuat *att_state, const struct GainsOrder2Vect3 *k_rate_ec);
static struct FloatVect3 control_error_attitude(const struct AttQuat *att_ref, const struct AttStateQuat *att_state, const struct GainsOrder3Vect3 *k_att_ec);
static float control_error_thrust(const struct ThrustRef *thrust_ref, const float thrust_state, const float k_thrust_ec);
static void compute_wls_upper_bounds(float u_d_max[ANDI_NUM_ACT], const float act_state[ANDI_NUM_ACT], const float act_max[ANDI_NUM_ACT], const float act_rate_max[ANDI_NUM_ACT], float dt);
static void compute_wls_lower_bounds(float u_d_min[ANDI_NUM_ACT], const float act_state[ANDI_NUM_ACT], const float act_min[ANDI_NUM_ACT], const float act_rate_min[ANDI_NUM_ACT], float dt);
static void compute_wls_u_scaler(float u_scaler[ANDI_NUM_ACT], const float act_min[ANDI_NUM_ACT], const float act_max[ANDI_NUM_ACT]);
static void compute_wls_v_scaler(float v_scaler[ANDI_NUM_ACT], const float v[ANDI_NUM_ACT]);

static inline float ec_k1_order3_f(const float omega_n, const float zeta, const float omega_a) { return (omega_n * omega_n * (omega_a - 2 * zeta * omega_n)); }
static inline float ec_k2_order3_f(const float omega_n, const float zeta, const float omega_a) { return (omega_n * omega_n + 2.0f * zeta * omega_n * (omega_a - 2 * zeta * omega_n)); }
static inline float ec_k3_order3_f(const float omega_n UNUSED, const float zeta UNUSED, const float omega_a) { return omega_a; }
static inline float rm_k1_order3_f(const float omega_n, const float zeta, const float omega_a) { return ec_k1_order3_f(omega_n, zeta, omega_a) / ec_k2_order3_f(omega_n, zeta, omega_a); }
static inline float rm_k2_order3_f(const float omega_n, const float zeta, const float omega_a) { return ec_k2_order3_f(omega_n, zeta, omega_a) / ec_k3_order3_f(omega_n, zeta, omega_a); }
static inline float rm_k3_order3_f(const float omega_n, const float zeta, const float omega_a) { return ec_k3_order3_f(omega_n, zeta, omega_a); }

static inline float ec_k1_order2_f(const float omega_n, const float zeta UNUSED) { return omega_n * omega_n; }
static inline float ec_k2_order2_f(const float omega_n, const float zeta) { return 2.0f * zeta * omega_n; }
static inline float rm_k1_order2_f(const float omega_n, const float zeta) { return omega_n / (2.0f * zeta); }
static inline float rm_k2_order2_f(const float omega_n, const float zeta) { return 2.0f * zeta * omega_n; }

struct PolesOrder2Vect3 andi_p_rate_ec = {
  .omega_n = {
    .x = STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_N_X,
    .y = STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_N_Y,
    .z = STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_N_Z},
  .zeta = {
    .x = STABILIZATION_ANDI_POLE_RATE_EC_ZETA_X,
    .y = STABILIZATION_ANDI_POLE_RATE_EC_ZETA_Y,
    .z = STABILIZATION_ANDI_POLE_RATE_EC_ZETA_Z}
  };
struct PolesOrder2Vect3 andi_p_rate_rm = {
  .omega_n = {
    .x = STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_N_X,
    .y = STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_N_Y,
    .z = STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_N_Z},
  .zeta = {
    .x = STABILIZATION_ANDI_POLE_RATE_RM_ZETA_X,
    .y = STABILIZATION_ANDI_POLE_RATE_RM_ZETA_Y,
    .z = STABILIZATION_ANDI_POLE_RATE_RM_ZETA_Z}
  };
struct PolesOrder3Vect3 andi_p_att_ec = {
  .omega_n = {
    .x = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_X,
    .y = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_Z},
  .zeta = {
    .x = STABILIZATION_ANDI_POLE_ATT_EC_ZETA_X,
    .y = STABILIZATION_ANDI_POLE_ATT_EC_ZETA_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_EC_ZETA_Z},
  .omega_a = {
    .x = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_A_X,
    .y = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_A_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_A_Z}
  };
struct PolesOrder3Vect3 andi_p_att_rm = {
  .omega_n = {
    .x = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_X,
    .y = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_Z},
  .zeta = {
    .x = STABILIZATION_ANDI_POLE_ATT_RM_ZETA_X,
    .y = STABILIZATION_ANDI_POLE_ATT_RM_ZETA_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_RM_ZETA_Z},
  .omega_a = {
    .x = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_A_X,
    .y = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_A_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_A_Z}
  };
float andi_p_thrust_ec = STABILIZATION_ANDI_POLE_THRUST_EC;
float andi_p_thrust_rm = STABILIZATION_ANDI_POLE_THRUST_RM;

float andi_omega_freq_cutoff = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA;         // rad/s
float andi_omega_dot_freq_cutoff = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA_DOT; // rad/s
float andi_accel_freq_cutoff = STABILIZATION_ANDI_CUTOFF_FREQ_ACCEL;         // rad/s
float andi_vel_freq_cutoff = STABILIZATION_ANDI_CUTOFF_FREQ_VEL;             // rad/s
float andi_thrust_freq_cutoff = STABILIZATION_ANDI_CUTOFF_FREQ_THRUST;       // rad/s

// WLS allocation variables
struct WLS_t wls_stab_p = {
    .nu = ANDI_NUM_ACT,
    .nv = ANDI_OUTPUTS,
    .gamma_sq = 100000.0,
    .u_pref = {0.0f},
    .u_min = {0.0f},
    .u_max = {0.0f},
    .PC = 0.0f,
    .SC = 0.0f,
    .iter = 0};

// Controller gains
struct GainsOrder2Vect3 andi_k_rate_ec;
struct GainsOrder2Vect3 andi_k_rate_rm;
struct GainsOrder3Vect3 andi_k_att_ec;
struct GainsOrder3Vect3 andi_k_att_rm;
float andi_k_thrust_ec;
float andi_k_thrust_rm;

// Low pass filters for measured states and time synchronization
struct FirstOrderLowPassVect3 angular_rates_meas_lpf;
struct FirstOrderLowPassVect3 angular_rates_sync_lpf;
struct FirstOrderLowPassVect3 angular_accel_meas_lpf;
struct FirstOrderLowPassVect3 angular_accel_sync_lpf;
struct FirstOrderLowPass thrust_meas_lpf;
struct FirstOrderLowPass thrust_sync_lpf;

// Complementary filter instances for state dependent contribution
struct FirstOrderComplementaryVect3 angular_rates_cf;
struct FirstOrderComplementaryVect3 angular_accel_cf;
struct FloatRates angular_rates_obm;

struct FirstOrderComplementaryVect3 linear_vel_cf;
struct FirstOrderComplementaryVect3 linear_accel_cf;
struct FloatVect3 linear_velocity_obm;

// Actuator filtering and delay
struct TransportDelay actuator_delay[ANDI_NUM_ACT]; // transport delay for actuator model
float act_dynamics_discrete[ANDI_NUM_ACT]; // discrete-time actuator dynamics
float actuator_rt[ANDI_NUM_ACT]; // undelayed actuator measurement for model
float actuator_prev[ANDI_NUM_ACT]; // previous actuator measurement for du calculation
float actuator_meas[ANDI_NUM_ACT];
float actuator_state_dot[ANDI_NUM_ACT];

// T4 Actuator feedback handling
struct ActuatorsT4In actuators_t4_obs;
abi_event actuators_t4_in_event;

// Raw state measurement variables
struct FloatRates rates_prev;

// State variables
struct AttStateQuat attitude_state_cf;  // undelayed attitude state for feedforward
struct AttStateQuat attitude_state_lpf; // delayed attitude state for feedback
struct LinState linear_state_cf;        // undelayed linear state for feedforward
float thrust_state_lpf;                 // delayed thrust state for feedback
float actuator_state[ANDI_NUM_ACT];
float actuator_t4_state[ANDI_NUM_ACT];

// Reference model variables
struct AttQuat attitude_ref;
struct ThrustRef thrust_ref;
struct AttQuat attitude_ref_sync;
struct ThrustRef thrust_ref_sync;

// Setpoints
struct FloatRates rates_des;
struct FloatQuat attitude_des;
float thrust_des;

// Bounds
struct AttQuat attitude_bounds;
struct ThrustRef thrust_bounds_min;
struct ThrustRef thrust_bounds_max;

// Controller variables
float ce_mat[ANDI_OUTPUTS * ANDI_NUM_ACT];
float du_min[ANDI_NUM_ACT];
float du_max[ANDI_NUM_ACT];
float du_cmd[ANDI_NUM_ACT];
float u_cmd[ANDI_NUM_ACT];

// Pseudo command variables
float nu_obj[ANDI_OUTPUTS]; // Total pseudo command allocated to the actuators
float nu_ec[ANDI_OUTPUTS]; // Pseudo command from the error controller
float nu_obm[ANDI_OUTPUTS]; // Pseudo command from the on board model state dependent term
float nu_reconstructed[ANDI_OUTPUTS]; // Reconstructed angular acceleration from the actuator commands (for model verification)

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_wls_v_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  char *name = "andi";
  pprz_msg_send_WLS_V(trans, dev, AC_ID,
                      strlen(name), name,
                      &wls_stab_p.gamma_sq, // Does this need scaling as a function of scaling factor?
                      (uint8_t *)&wls_stab_p.iter,
                      ANDI_OUTPUTS, nu_obj,
                      ANDI_OUTPUTS, (float *)WLS_WV);
}
static void send_wls_u_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  char *name = "andi";
  float zero_array[ANDI_NUM_ACT] = {0.0f};
  pprz_msg_send_WLS_U(trans, dev, AC_ID,
                      strlen(name), name,
                      ANDI_NUM_ACT, (float *)WLS_WU,
                      ANDI_NUM_ACT, zero_array,
                      ANDI_NUM_ACT, du_min,
                      ANDI_NUM_ACT, du_max,
                      ANDI_NUM_ACT, du_cmd);
}

static void send_eff_mat_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0f;
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID,
                             ANDI_NUM_ACT, &ce_mat[0 * ANDI_NUM_ACT],
                             ANDI_NUM_ACT, &ce_mat[1 * ANDI_NUM_ACT],
                             ANDI_NUM_ACT, &ce_mat[2 * ANDI_NUM_ACT],
                             ANDI_NUM_ACT, &ce_mat[3 * ANDI_NUM_ACT],
                             1,  &zero);
}
static void send_stab_attitude_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                              4, (float *)&attitude_des,
                              4, (float *)&attitude_state_lpf.att,
                              4, (float *)&attitude_ref_sync.att,
                              3, (float *)&attitude_state_lpf.att_d,
                              3, (float *)&attitude_ref_sync.att_d,
                              3, (float *)&attitude_state_lpf.att_2d,
                              3, (float *)&attitude_ref_sync.att_2d,
                              3, (float *)&attitude_ref_sync.att_3d,
                              ANDI_OUTPUTS, actuator_state);
}

static void send_stab_thrust_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_THRUST(trans, dev, AC_ID,
                            &thrust_des,
                            &thrust_ref.thrust,
                            &thrust_state_lpf,
                            &thrust_ref.thrust_d);
}
static void send_stab_pseudo_command_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  /**
   * nu_obj: Total pseudo command sent to the actuators
   * nu_ec: Pseudo command from the error controller
   * nu_obm: Pseudo command from the on board model state dependent term
   * nu_reconstructed: Reconstructed angular acceleration from the actuator commands. 
   * This is used to verify the on board model. If the OBM is perfect, then nu_reconstructed should be
   * equal to the measured state plus any external disturbances.
   * 
   * nu_obj = nu_ec + nu_obm
   * nu_reconstructed = Ce * u_meas - nu_obm
   */
  pprz_msg_send_STAB_PSEUDO_COMMAND(trans, dev, AC_ID,
                                    ANDI_OUTPUTS, nu_obj, // Total pseudo command
                                    ANDI_OUTPUTS, nu_ec, // From error controller
                                    ANDI_OUTPUTS, nu_obm, // From on board model state dependent term
                                    ANDI_OUTPUTS, nu_reconstructed); // Reconstructed from actuator commands
}

#endif // PERIODIC_TELEMETRY

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
static void actuators_t4_in_callback(uint8_t sender_id UNUSED, struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr UNUSED)
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
 * FIXME: All ESC type actuators will be squared rpm for thrust linearization.
 *
 * @param[out] actuator_state Array of floats with size ANDI_NUM_ACT where the
 *                           converted actuator states will be stored. The caller
 *                           must allocate this.
 * @param[in] actuators_t4_in_ptr Pointer to the input struct containing actuator telemetry,
 *                                including servo angles (in 1e-2 degrees) and ESC RPM.
 */
static void fetch_actuators_t4(float actuator_meas[ANDI_NUM_ACT], const struct ActuatorsT4In *actuators_t4_in_ptr)
{
  actuator_meas[0] = RadOfCentiDeg((float)actuators_t4_in_ptr->servo_1_angle);
  actuator_meas[1] = -RadOfCentiDeg((float)actuators_t4_in_ptr->servo_6_angle);
  actuator_meas[2] = (float)actuators_t4_in_ptr->esc_1_rpm * 2 * M_PI / 60;      // Convert rpm to rad/s
  actuator_meas[2] *= actuator_meas[2];                                          // square motor rpm
  actuator_meas[3] = (float)actuators_t4_in_ptr->esc_2_rpm * 2 * M_PI / 60;
  actuator_meas[3] *= actuator_meas[3]; // square motor rpm
}

/**
 * @brief Retrieve actuator measurement from first order model.
 *
 * This function updates the passed array `actuator_meas` by applying a discrete
 * actuator dynamics filter, combining previous measurements and control inputs.
 *
 * @param[in,out] actuator_meas Array of floats with size ANDI_NUM_ACT to store
 *                              the actuator measurement results in radians (angles)
 *                              and radians per second (rotational speeds).
 * @param[in] u_cmd Array of floats with size ANDI_NUM_ACT with previous actuator
 *                   commands.
 * FIXME: Add transmission delay handling?
 */
static void apply_actuator_dynamics_filter(float actuator_meas[ANDI_NUM_ACT], float u_cmd[ANDI_NUM_ACT])
{
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    actuator_meas[i] = actuator_meas[i] * (1 - act_dynamics_discrete[i]) + u_cmd[i] * act_dynamics_discrete[i];
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
 *   using the global control input array `u_cmd` and the global discrete actuator
 *   dynamics array `act_dynamics_discrete`.
 *
 * @param[in,out] actuator_meas Array of floats with size ANDI_NUM_ACT to store
 *                              the actuator measurement results in radians (angles)
 *                              and radians per second (rotational speeds).
 *                              This array also functions as input and should contain the
 *                              previous actuator state in case a model is used.
 */
static void get_actuator_measurement(float actuator_meas[ANDI_NUM_ACT])
{
  float actuator_meas_tmp_1[ANDI_NUM_ACT];
  float actuator_meas_tmp_2[ANDI_NUM_ACT];
  float_vect_copy(actuator_meas_tmp_2, actuator_meas, ANDI_NUM_ACT);

  // FIXME: Using actuator feedback for ESC does not work at the moment
  fetch_actuators_t4(actuator_meas_tmp_1, &actuators_t4_obs);
  apply_actuator_dynamics_filter(actuator_meas_tmp_2, u_cmd);

  // choose actuator feedback or model
  actuator_meas[0] = actuator_meas_tmp_1[0]; // elevon left
  actuator_meas[1] = actuator_meas_tmp_1[1]; // elevon right
  actuator_meas[2] = actuator_meas_tmp_2[2]; // motor^2 left
  actuator_meas[3] = actuator_meas_tmp_2[3]; // motor^2 right
}

/**
 * Compute reference-model gains for a 3rd-order 3D system.
 * Each axis gain (x, y, z) is computed using rm_k*_order3_f() with omega_n, zeta, and p1 parameters.
 * @param[in] poles Pointer to PolesOrder3Vect3 containing omega_n, zeta, and p1 for each axis.
 * @return Struct containing k1, k2, k3 gains for x, y, z.
 */
static struct GainsOrder3Vect3 compute_reference_gains_order_3_vect_3(const struct PolesOrder3Vect3 *poles)
{
  struct GainsOrder3Vect3 gains;
  gains.k1.x = rm_k1_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k1.y = rm_k1_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k1.z = rm_k1_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);

  gains.k2.x = rm_k2_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k2.y = rm_k2_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k2.z = rm_k2_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);

  gains.k3.x = rm_k3_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k3.y = rm_k3_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k3.z = rm_k3_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);
  return gains;
}

/**
 * Compute reference-model gains for a 2nd-order 3D system.
 * Each axis gain (x, y, z) is computed using rm_k*_order2_f() with omega_n and zeta parameters.
 * @param[in] poles Pointer to PolesOrder2Vect3 containing omega_n and zeta for each axis.
 * @return Struct containing k1, k2 gains for x, y, z.
 */
static struct GainsOrder2Vect3 compute_reference_gains_order_2_vect_3(const struct PolesOrder2Vect3 *poles)
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
static struct GainsOrder3Vect3 compute_error_gains_order_3_vect_3(const struct PolesOrder3Vect3 *poles)
{
  struct GainsOrder3Vect3 gains;
  gains.k1.x = ec_k1_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k1.y = ec_k1_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k1.z = ec_k1_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);

  gains.k2.x = ec_k2_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k2.y = ec_k2_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k2.z = ec_k2_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);

  gains.k3.x = ec_k3_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k3.y = ec_k3_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k3.z = ec_k3_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);
  return gains;
}

/**
 * Compute error-compensation gains for a 2nd-order 3D system.
 * Each axis gain (x, y, z) is computed using ec_k*_order2_f() with omega_n and zeta parameters.
 * @param[in] poles Pointer to PolesOrder2Vect3 containing omega_n and zeta for each axis.
 * @return Struct containing k1, k2 gains for x, y, z.
 */
static struct GainsOrder2Vect3 compute_error_gains_order_2_vect_3(const struct PolesOrder2Vect3 *poles)
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
  struct FloatQuat att_err;  // rotation from ref to des
  float_quat_inv_comp_norm_shortest(&att_err, (struct FloatQuat *)&att_ref->att, (struct FloatQuat *)att_des);

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
  float_quat_normalize(&att_ref->att);
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
 * @param[in,out] thrust_ref  Pointer to ThrustRef struct holding thrust reference states.
 */
static void generate_reference_thrust(
    float dt,
    float thrust_des,
    const float k_thrust_rm,
    const struct ThrustRef *bounds_min,
    const struct ThrustRef *bounds_max,
    struct ThrustRef *thrust_ref)
{
  Bound(thrust_des, bounds_min->thrust, bounds_max->thrust);
  thrust_ref->thrust_d = k_thrust_rm * (thrust_des - thrust_ref->thrust);

  Bound(thrust_ref->thrust_d, bounds_min->thrust_d, bounds_max->thrust_d);
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
 * @param[in] k_att_ec  Pointer to gain parameters struct containing proportional, derivative, and jerk gains.
 *
 * @return The computed virtual control input vector.
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

  struct FloatQuat att_err; // rotation from state to reference
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
 * @return The computed virtual control thrust input.
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
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
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
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
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
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    float range = fabs(act_max[i] - act_min[i]);
    if (range <= FLT_EPSILON)
    {
      u_scaler[i] = 1.0f;
    }
    else
    {
      u_scaler[i] = 1.0f / range;
    }
  }
}

/**
 * @brief Compute output scaling factors for normalizing weighted least squares outputs.
 *
 * For each output i this sets v_scaler[i] = 1.0f / abs(v[i]) when v[i] is non-zero; otherwise v_scaler[i] is set
 * to 1.0f to avoid a division-by-zero. The normalized output used in WLS is then 
 * 
 * v_norm = v_scaler * v.
 *
 * @param[out] v_scaler Array of length ANDI_NUM_ACT to store the computed inverse scaling factors.
 * @param[in]  v        Array of length ANDI_NUM_ACT containing reference/scaling values used to compute the inverse.
 */
static void compute_wls_v_scaler(float v_scaler[ANDI_NUM_ACT], const float v[ANDI_NUM_ACT])
{
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    if (fabs(v[i]) <= FLT_EPSILON)
    {
      v_scaler[i] = 1.0f;
    }
    else
    {
      v_scaler[i] = 1.0f / fabs(v[i]);
    }
  }
}

void stabilization_andi_init(void)
{
  // Compute gains
  andi_k_rate_ec = compute_error_gains_order_2_vect_3(&andi_p_rate_ec);
  andi_k_rate_rm = compute_reference_gains_order_2_vect_3(&andi_p_rate_rm);
  andi_k_att_ec = compute_error_gains_order_3_vect_3(&andi_p_att_ec);
  andi_k_att_rm = compute_reference_gains_order_3_vect_3(&andi_p_att_rm);
  andi_k_thrust_ec = andi_p_thrust_ec;
  andi_k_thrust_rm = andi_p_thrust_rm;

  // Initialize state variables
  rates_prev.p = 0.0f;
  rates_prev.q = 0.0f;
  rates_prev.r = 0.0f;

  float_quat_identity(&attitude_state_lpf.att);
  attitude_state_lpf.att_d.p = 0.0f;
  attitude_state_lpf.att_d.q = 0.0f;
  attitude_state_lpf.att_d.r = 0.0f;
  attitude_state_lpf.att_2d.x = 0.0f;
  attitude_state_lpf.att_2d.y = 0.0f;
  attitude_state_lpf.att_2d.z = 0.0f;

  float_quat_identity(&attitude_state_cf.att);
  attitude_state_cf.att_d.p = 0.0f;
  attitude_state_cf.att_d.q = 0.0f;
  attitude_state_cf.att_d.r = 0.0f;
  attitude_state_cf.att_2d.x = 0.0f;
  attitude_state_cf.att_2d.y = 0.0f;
  attitude_state_cf.att_2d.z = 0.0f;

  linear_state_cf.vel.x = 0.0f;
  linear_state_cf.vel.y = 0.0f;
  linear_state_cf.vel.z = 0.0f;
  linear_state_cf.acc.x = 0.0f;
  linear_state_cf.acc.y = 0.0f;
  linear_state_cf.acc.z = 0.0f;

  thrust_state_lpf = 0.0f;
  float_vect_zero(actuator_state, ANDI_NUM_ACT);
  float_vect_zero(actuator_t4_state, ANDI_NUM_ACT);
  float_vect_zero(actuator_meas, ANDI_NUM_ACT);
  float_vect_zero(actuator_prev, ANDI_NUM_ACT);
  float_vect_zero(actuator_rt, ANDI_NUM_ACT);
  float_vect_zero(u_cmd, ANDI_NUM_ACT);
  float_vect_zero(du_cmd, ANDI_NUM_ACT);
  float_vect_zero(nu_obj, ANDI_NUM_ACT);
  float_vect_zero(nu_ec, ANDI_NUM_ACT);
  float_vect_zero(nu_obm, ANDI_NUM_ACT);
  float_vect_zero(nu_reconstructed, ANDI_NUM_ACT);

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

  float_quat_identity(&attitude_ref_sync.att);
  attitude_ref_sync.att_d.p = 0.0f;
  attitude_ref_sync.att_d.q = 0.0f;
  attitude_ref_sync.att_d.r = 0.0f;
  attitude_ref_sync.att_2d.x = 0.0f;
  attitude_ref_sync.att_2d.y = 0.0f;
  attitude_ref_sync.att_2d.z = 0.0f;
  attitude_ref_sync.att_3d.x = 0.0f;
  attitude_ref_sync.att_3d.y = 0.0f;
  attitude_ref_sync.att_3d.z = 0.0f;

  thrust_ref_sync.thrust = 0.0f;
  thrust_ref_sync.thrust_d = 0.0f;

  // FIXME: These bounds should be set via parameters
  // Initialize attitude bounds (symmetric bounds on abs values)
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

  // Limit thrust bounds (asymmetric bounds)
  thrust_bounds_min.thrust = THRUST_MIN;
  thrust_bounds_max.thrust = THRUST_MAX;
  thrust_bounds_min.thrust_d = -1000.0f;
  thrust_bounds_max.thrust_d = 1000.0f;

  // Initial control effectiveness matrix
  evaluate_obm_f_stb_u(ce_mat, &attitude_state_cf.att_d, &linear_state_cf.vel, ACTUATOR_PREF); // FIXME: Choose state estimate for scheduling.

  // Initialize filters
  init_first_order_low_pass_vect3(&angular_rates_meas_lpf, 1.0f / andi_omega_freq_cutoff, SAMPLE_TIME);
  init_first_order_low_pass_vect3(&angular_rates_sync_lpf, 1.0f / andi_omega_freq_cutoff, SAMPLE_TIME);
  init_first_order_low_pass_vect3(&angular_accel_meas_lpf, 1.0f / andi_omega_dot_freq_cutoff, SAMPLE_TIME);
  init_first_order_low_pass_vect3(&angular_accel_sync_lpf, 1.0f / andi_omega_dot_freq_cutoff, SAMPLE_TIME);
  init_first_order_low_pass(&thrust_meas_lpf, 1.0f / andi_thrust_freq_cutoff, SAMPLE_TIME, 0.0f);
  init_first_order_low_pass(&thrust_sync_lpf, 1.0f / andi_thrust_freq_cutoff, SAMPLE_TIME, 0.0f);

  init_first_order_complementary_vect3(&angular_rates_cf, 1.0f / andi_omega_freq_cutoff, SAMPLE_TIME);
  init_first_order_complementary_vect3(&angular_accel_cf, 1.0f / andi_omega_dot_freq_cutoff, SAMPLE_TIME);
  angular_rates_obm.p = 0.0f;
  angular_rates_obm.q = 0.0f;
  angular_rates_obm.r = 0.0f;

  init_first_order_complementary_vect3(&linear_vel_cf, 1.0f / andi_vel_freq_cutoff, SAMPLE_TIME);
  init_first_order_complementary_vect3(&linear_accel_cf, 1.0f / andi_accel_freq_cutoff, SAMPLE_TIME);
  linear_velocity_obm.x = 0.0f;
  linear_velocity_obm.y = 0.0f;
  linear_velocity_obm.z = 0.0f;

  // Bind T4 actuator feedback abi message
  AbiBindMsgACTUATORS_T4_IN(ABI_BROADCAST, &actuators_t4_in_event, actuators_t4_in_callback);

  // Precompute discrete-time actuator dynamics coefficients
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    act_dynamics_discrete[i] = 1 - exp(-ACTUATOR_DYNAMICS[i] / PERIODIC_FREQUENCY);
  }
  init_transport_delay_array(ANDI_NUM_ACT, actuator_delay, ACTUATOR_DELAY, actuator_state);

// Start telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_stab_attitude_stabilization_andi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_THRUST, send_stab_thrust_stabilization_andi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_eff_mat_stabilization_andi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_stabilization_andi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_stabilization_andi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_PSEUDO_COMMAND, send_stab_pseudo_command_stabilization_andi);
#endif
}

void stabilization_andi_enter(void)
{
  // Clear previous rates
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

  attitude_ref_sync.att = *stateGetNedToBodyQuat_f();
  attitude_ref_sync.att_d.p = 0.0f;
  attitude_ref_sync.att_d.q = 0.0f;
  attitude_ref_sync.att_d.r = 0.0f;
  attitude_ref_sync.att_2d.x = 0.0f;
  attitude_ref_sync.att_2d.y = 0.0f;
  attitude_ref_sync.att_2d.z = 0.0f;
  attitude_ref_sync.att_3d.x = 0.0f;
  attitude_ref_sync.att_3d.y = 0.0f;
  attitude_ref_sync.att_3d.z = 0.0f;

  // Fetch linear measurements
  struct LinState lin_meas;
  float_quat_vmult(&lin_meas.vel, stateGetNedToBodyQuat_f(), (struct FloatVect3 *)stateGetSpeedNed_f());
  linear_state_cf.acc.x = 0.0f;
  linear_state_cf.acc.y = 0.0f;
  linear_state_cf.acc.z = 0.0f;

  // Reset thrust reference to zero
  // thrust_ref.thrust = 0.0f; // Thrust at k = -1
  // thrust_ref.thrust_d = 0.0f; // Thrust_d at k = -1

  // Reset actuator model internal state
  // thrust_state = 0.0f; // Thrust at k = -1
  // float_vect_zero(actuator_state, ANDI_NUM_ACT);
  // float_vect_zero(actuator_meas, ANDI_NUM_ACT); // Actuator at k = -1
  // float_vect_zero(u_cmd, ANDI_NUM_ACT); // Actuator command at k = -1
  // float_vect_zero(du_cmd, ANDI_NUM_ACT); // Actuator command at k = -1
}

void stabilization_andi_run(bool use_rate_control, bool in_flight, struct StabilizationSetpoint *stab_setpoint, struct ThrustSetpoint *thrust_setpoint, int32_t *cmd)
{

  // Recompute gains
  // FIXME: Can't this only be done when parameters have changed? Maybe using a handler?
  andi_k_rate_ec = compute_error_gains_order_2_vect_3(&andi_p_rate_ec);
  andi_k_rate_rm = compute_reference_gains_order_2_vect_3(&andi_p_rate_rm);
  andi_k_att_ec = compute_error_gains_order_3_vect_3(&andi_p_att_ec);
  andi_k_att_rm = compute_reference_gains_order_3_vect_3(&andi_p_att_rm);
  andi_k_thrust_ec = andi_p_thrust_ec;
  andi_k_thrust_rm = andi_p_thrust_rm;

  // Fetch linear measurements
  struct LinState lin_meas;
  float_quat_vmult(&lin_meas.vel, stateGetNedToBodyQuat_f(), (struct FloatVect3 *)stateGetSpeedNed_f()); // From Kalman filter
  lin_meas.acc = *stateGetAccelBody_f();                                                                 // From accelerometer

  // Fetch attitude measurements
  struct AttStateQuat attitude_meas;
  attitude_meas.att = *stateGetNedToBodyQuat_f(); // From Kalman filter
  attitude_meas.att_d = *stateGetBodyRates_f();   // From gyroscope
  attitude_meas.att_2d.x = (attitude_meas.att_d.p - rates_prev.p) * PERIODIC_FREQUENCY;
  attitude_meas.att_2d.y = (attitude_meas.att_d.q - rates_prev.q) * PERIODIC_FREQUENCY;
  attitude_meas.att_2d.z = (attitude_meas.att_d.r - rates_prev.r) * PERIODIC_FREQUENCY;
  rates_prev = attitude_meas.att_d; // Store previous rates for next acceleration calculation

  // Fetch actuator measurements
  get_actuator_measurement(actuator_rt); // FIXME: This only includes servo feedback and models ESC Feedback.
  update_transport_delay_array(ANDI_NUM_ACT, actuator_delay, actuator_rt);
  get_transport_delay_array(ANDI_NUM_ACT, actuator_delay, actuator_meas);

  float actuator_t4_meas[ANDI_NUM_ACT];
  fetch_actuators_t4(actuator_t4_meas, &actuators_t4_obs); // FIXME: This does include ESC feedback

  // COMPLEMENTARY FILTERING
  // Evaluate On Board Model at previous state.
  struct FloatVect3 angular_accel_obm = evaluate_obm_moments(&attitude_state_cf.att_d, &linear_state_cf.vel, actuator_state, actuator_state_dot);
  struct FloatVect3 linear_accel_obm = evaluate_obm_forces(&attitude_state_cf.att_d, &linear_state_cf.vel, actuator_state, actuator_state_dot);

  // Cascaded complementary filter for linear velocity and acclererations
  update_first_order_complementary_vect3(&linear_accel_cf, &linear_accel_obm, &lin_meas.acc);
  linear_state_cf.acc = get_first_order_complementary_vect3(&linear_accel_cf);
  float_vect3_integrate_fi(&linear_velocity_obm, &linear_state_cf.acc, SAMPLE_TIME);
  update_first_order_complementary_vect3(&linear_vel_cf, &linear_velocity_obm, &lin_meas.vel);
  linear_state_cf.vel = get_first_order_complementary_vect3(&linear_vel_cf);

  // Cascaded complementary filter for angular rates and accelerations
  update_first_order_complementary_vect3(&angular_accel_cf, &angular_accel_obm, &attitude_meas.att_2d);
  attitude_state_cf.att_2d = get_first_order_complementary_vect3(&angular_accel_cf);
  float_rates_vect3_integrate_fi(&angular_rates_obm, &attitude_state_cf.att_2d, SAMPLE_TIME);
  update_first_order_complementary_rates(&angular_rates_cf, &angular_rates_obm, &attitude_meas.att_d);
  attitude_state_cf.att_d = get_first_order_complementary_rates(&angular_rates_cf);

  attitude_state_cf.att = attitude_meas.att; // No filtering on attitude

  update_first_order_low_pass_vect3(&angular_accel_meas_lpf, &attitude_meas.att_2d);
  attitude_state_lpf.att_2d = get_first_order_low_pass_vect3(&angular_accel_meas_lpf);
  update_first_order_low_pass_rates(&angular_rates_meas_lpf, &attitude_meas.att_d);
  attitude_state_lpf.att_d = get_first_order_low_pass_rates(&angular_rates_meas_lpf);

  attitude_state_lpf.att = attitude_meas.att; // No filtering on attitude

  float_vect_copy(actuator_state, actuator_meas, ANDI_NUM_ACT);
  float_vect_copy(actuator_t4_state, actuator_t4_meas, ANDI_NUM_ACT);

  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    actuator_state_dot[i] = (actuator_state[i] - actuator_prev[i]) * PERIODIC_FREQUENCY;
  }
  float_vect_copy(actuator_prev, actuator_state, ANDI_NUM_ACT);

  float thrust_meas = evaluate_obm_thrust_z(actuator_state); // Do not use actuator_t4_state here!
  update_first_order_low_pass(&thrust_meas_lpf, thrust_meas);
  thrust_state_lpf = get_first_order_low_pass(&thrust_meas_lpf);

  // Get setpoints
  if (use_rate_control)
  {
    rates_des = stab_sp_to_rates_f(stab_setpoint);
    if (in_flight)
      generate_reference_rate(SAMPLE_TIME, &rates_des, &andi_k_rate_rm, &attitude_bounds, &attitude_ref);
  }
  else
  {
    attitude_des = stab_sp_to_quat_f(stab_setpoint);
    if (in_flight)
      generate_reference_attitude(SAMPLE_TIME, &attitude_des, &andi_k_att_rm, &attitude_bounds, &attitude_ref);
  }

  // FIXME: Thrust setpoint can not be of type THRUST_INCR_SP, this is not enforced but will silently fail
  thrust_des = th_sp_to_thrust_f(thrust_setpoint, 0, THRUST_AXIS_Z) * THRUST_MAX;
  generate_reference_thrust(SAMPLE_TIME, thrust_des, andi_k_thrust_rm, &thrust_bounds_min, &thrust_bounds_max, &thrust_ref);

  // SYNC FILTERING OF REFERENCE INPUTS
  attitude_ref_sync.att_3d.x = attitude_ref.att_3d.x;
  attitude_ref_sync.att_3d.y = attitude_ref.att_3d.y;
  attitude_ref_sync.att_3d.z = attitude_ref.att_3d.z;
  update_first_order_low_pass_vect3(&angular_rates_sync_lpf, &attitude_ref.att_2d);
  attitude_ref_sync.att_2d = get_first_order_low_pass_vect3(&angular_rates_sync_lpf);
  update_first_order_low_pass_rates(&angular_rates_sync_lpf, &attitude_ref.att_d);
  attitude_ref_sync.att_d = get_first_order_low_pass_rates(&angular_rates_sync_lpf);
  attitude_ref_sync.att = attitude_ref.att; // No filtering on attitude

  thrust_ref_sync.thrust_d = thrust_ref.thrust;
  update_first_order_low_pass(&thrust_sync_lpf, thrust_ref.thrust);
  thrust_ref_sync.thrust = get_first_order_low_pass(&thrust_sync_lpf);

  // Construct pseudo control
  struct FloatVect3 nu_attitude;
  if (use_rate_control)
  {
    nu_attitude = control_error_rate(&attitude_ref_sync, &attitude_state_cf, &andi_k_rate_ec);
  }
  else
  {
    nu_attitude = control_error_attitude(&attitude_ref_sync, &attitude_state_cf, &andi_k_att_ec);
  }
  float nu_thrust = control_error_thrust(&thrust_ref_sync, thrust_state_lpf, andi_k_thrust_ec);

  nu_ec[0] = nu_attitude.x;
  nu_ec[1] = nu_attitude.y;
  nu_ec[2] = nu_attitude.z;
  nu_ec[3] = nu_thrust;

  // State feedback from on board model
  if (USE_STATE_DYNAMICS) evaluate_obm_f_stb_x(nu_obm, &attitude_state_cf.att_d, &linear_state_cf.vel, &attitude_meas.att_2d, &linear_state_cf.acc, actuator_t4_state);

  if (in_flight)
  {
    nu_obj[0] = nu_ec[0] - (nu_obm[0] * ANDI_RELAX_OBM);
    nu_obj[1] = nu_ec[1] - (nu_obm[1] * ANDI_RELAX_OBM);
    nu_obj[2] = nu_ec[2] - (nu_obm[2] * ANDI_RELAX_OBM);
  }
  else
  {
    nu_obj[0] = 0.0f;
    nu_obj[1] = 0.0f;
    nu_obj[2] = 0.0f;
  }
  nu_obj[3] = nu_ec[3] - (nu_obm[3] * ANDI_RELAX_OBM);

  // Compute control effectiveness matrix based on current states
  if (SCHEDULE_EFF) evaluate_obm_f_stb_u(ce_mat, &attitude_state_cf.att_d, &linear_state_cf.vel, actuator_t4_state);

  // Reconstruct nu based actuator state, this should be close to state measurements if OBM is accurate
  float_vect_copy(nu_reconstructed, nu_obm, ANDI_OUTPUTS);
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    for (uint8_t j = 0; j < ANDI_OUTPUTS; j++)
    {
      nu_reconstructed[j] += ce_mat[i * ANDI_OUTPUTS + j] * actuator_t4_state[i];
    }
  }

  // Solve control allocation using weighted least squares
  compute_wls_lower_bounds(du_min, actuator_state, ACTUATOR_MIN, ACTUATOR_D_MIN, SAMPLE_TIME);
  compute_wls_upper_bounds(du_max, actuator_state, ACTUATOR_MAX, ACTUATOR_D_MAX, SAMPLE_TIME);
  float wls_u_scaler[ANDI_NUM_ACT];
  float wls_v_scaler[ANDI_OUTPUTS] = {[0 ... ANDI_OUTPUTS - 1] = 1.0f}; // Disable v scaling
  compute_wls_u_scaler(wls_u_scaler, du_min, du_max);
  // compute_wls_v_scaler(wls_v_scaler, nu_obj);

  float ce_mat_scaled[ANDI_OUTPUTS][ANDI_NUM_ACT];
  float *bwls[ANDI_OUTPUTS];
  // Scale control effectiveness matrix
  for (uint8_t i = 0; i < ANDI_OUTPUTS; i++)
  {
    for (uint8_t j = 0; j < ANDI_NUM_ACT; j++)
    {
      ce_mat_scaled[i][j] = ce_mat[i * ANDI_NUM_ACT + j] * wls_v_scaler[i] / wls_u_scaler[j];
    }
    bwls[i] = ce_mat_scaled[i];
  }

  // Scale actuator bounds and weights
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    wls_stab_p.u_min[i] = du_min[i] * wls_u_scaler[i];
    wls_stab_p.u_max[i] = du_max[i] * wls_u_scaler[i]; // FIXME: Put u_pref as mean of u_min, u_max?
    wls_stab_p.Wu[i] = WLS_WU[i];
  }

  // Scale pseudo control and weights
  for (uint8_t i = 0; i < ANDI_OUTPUTS; i++)
  {
    wls_stab_p.v[i] = nu_obj[i] * wls_v_scaler[i];
    wls_stab_p.Wv[i] = WLS_WV[i];
  }

  wls_alloc(&wls_stab_p, bwls, 0, 0, 10);

  // Scale back actuator commands
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++)
  {
    du_cmd[i] = (wls_stab_p.u[i] / wls_u_scaler[i]);
    u_cmd[i] = du_cmd[i] / ACTUATOR_DYNAMICS[i] + actuator_state[i];
  }

  // Commit actuator commands
  // Resulting commands are in rad for servo and rad/s for motor
  // Paparazzi expects the commands in pprz units (-MAX_PPRZ to MAX_PPRZ for servo, 0 to MAX_PPRZ for motor).
  // FIXME: Do not hardcode actuator layout
  // FIXME: Do not hardcode motor command to rpm factor (and use a better model for this mapping)
  commands[0] = (pprz_t)(u_cmd[0] / ACTUATOR_MAX[0] * MAX_PPRZ);
  commands[1] = (pprz_t)(u_cmd[1] / ACTUATOR_MAX[1] * MAX_PPRZ);
  commands[2] = (pprz_t)(sqrt(u_cmd[2] / ACTUATOR_MAX[2]) * MAX_PPRZ);
  commands[3] = (pprz_t)(sqrt(u_cmd[3] / ACTUATOR_MAX[3]) * MAX_PPRZ);

  // Set Thrust command for compatibility with other modules
  // FIXME: Do not hardcode actuator layout
  cmd[COMMAND_THRUST] = 0;
  cmd[COMMAND_THRUST] += (pprz_t)(sqrt(u_cmd[2] / ACTUATOR_MAX[2]) * MAX_PPRZ);
  cmd[COMMAND_THRUST] += (pprz_t)(sqrt(u_cmd[3] / ACTUATOR_MAX[3]) * MAX_PPRZ);
  cmd[COMMAND_THRUST] /= 2;

}

/**
 * @brief Default weak function for evaluating state feedback from onboard model.
 */
void WEAK evaluate_obm_f_stb_x(float nu_obm[ANDI_OUTPUTS], const struct FloatRates *rates UNUSED, const struct FloatVect3 *vel_body UNUSED, const struct FloatVect3 *ang_accel UNUSED, const struct FloatVect3 *accel_body UNUSED, const float actuator_state[ANDI_NUM_ACT] UNUSED)
{
  // Default: No state feedback
  float_vect_zero(nu_obm, ANDI_OUTPUTS);
}

// FIXME: The following functions are to integrate the controller in the existing stabilization framework, find a better way to do this
void stabilization_attitude_enter(void)
{
  ;
}

void stabilization_rate_enter(void)
{
  ;
}

void stabilization_rate_run(bool in_flight, struct StabilizationSetpoint *rate_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stabilization_andi_run(true, in_flight, rate_sp, thrust, cmd);
}

void stabilization_attitude_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stabilization_andi_run(false, in_flight, sp, thrust, cmd);
}

// FIXME: This function is duplicated in stabilization_rate.c, find a way to share it
struct StabilizationSetpoint stabilization_rate_read_rc(struct RadioControl *rc)
{
  struct FloatRates rate_sp;
  FLOAT_RATES_ZERO(rate_sp);
  if (ROLL_RATE_DEADBAND_EXCEEDED(rc))
  {
    rate_sp.p = rc->values[RC_RATE_P] * RC_RATE_MAX[0] / MAX_PPRZ;
  }
  if (PITCH_RATE_DEADBAND_EXCEEDED(rc))
  {
    rate_sp.q = rc->values[RC_RATE_Q] * RC_RATE_MAX[1] / MAX_PPRZ;
  }
  if (YAW_RATE_DEADBAND_EXCEEDED(rc))
  {
    rate_sp.r = rc->values[RC_RATE_R] * RC_RATE_MAX[2] / MAX_PPRZ;
  }
  return stab_sp_from_rates_f(&rate_sp);
}

// FIXME: Maybe this helps prevent falling from sky
bool autopilot_in_flight_end_detection(bool motors_on UNUSED)
{
  return false;
}