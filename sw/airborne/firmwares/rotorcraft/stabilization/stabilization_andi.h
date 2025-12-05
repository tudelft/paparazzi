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
/** @file stabilization_andi.h
 *  @brief ANDI stabilization controller for rotorcraft
 */

#ifndef STABILIZATION_ANDI_H
#define STABILIZATION_ANDI_H

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "generated/airframe.h"
#include <stdio.h>
#include "filters/low_pass_filter.h"


#ifndef ANDI_NUM_ACT
#define ANDI_NUM_ACT COMMANDS_NB_REAL
#endif

#ifndef ANDI_OUTPUTS
#error "You must specify the number of controlled axis (outputs)"
#endif

struct AttQuat
{
  struct FloatQuat att;
  struct FloatRates att_d;
  struct FloatVect3 att_2d;
  struct FloatVect3 att_3d;
};

struct AttStateQuat
{
  struct FloatQuat att;
  struct FloatRates att_d;
  struct FloatVect3 att_2d;
};

struct LinState
{
  struct FloatVect3 vel;
  struct FloatVect3 acc;
};

struct ThrustRef
{
  float thrust;
  float thrust_d;
};

struct PolesOrder3Vect3
{
  struct FloatVect3 omega_n;
  struct FloatVect3 zeta;
  struct FloatVect3 omega_a;
};

struct PolesOrder2Vect3
{
  struct FloatVect3 omega_n;
  struct FloatVect3 zeta;
};

struct GainsOrder2Vect3
{
  struct FloatVect3 k1;
  struct FloatVect3 k2;
};

struct GainsOrder3Vect3
{
  struct FloatVect3 k1;
  struct FloatVect3 k2;
  struct FloatVect3 k3;
};

/*Declaration of Reference Model and Error Controller Poles*/
extern struct PolesOrder2Vect3 andi_p_rate_ec;
extern struct PolesOrder2Vect3 andi_p_rate_rm;
extern struct PolesOrder3Vect3 andi_p_att_ec;
extern struct PolesOrder3Vect3 andi_p_att_rm;
extern float andi_p_thrust_ec;
extern float andi_p_thrust_rm;

void stabilization_andi_init(void);
void stabilization_andi_enter(void);
void stabilization_andi_run(bool use_rate_control, bool in_flight, struct StabilizationSetpoint *stab_setpoint, struct ThrustSetpoint *thrust_setpoint, int32_t *cmd);


/**
 * @brief OBM coefficients instance for the cyclone airframe
 * 
 * This instance holds the coefficients used by the cyclone OBM
 * to compute forces and moments based on the current state and
 * actuator inputs.
 * 
 * @note The actual coefficients must be defined and initialized
 *       in the corresponding .c file (obm_cyclone.c).
 * @see obm_cyclone.c
 */
extern union CycloneCoefficients obm_coefficients;

/**
 * @brief Evaluate total force acting on the vehicle from the OBM
 *
 * Computes the full set of forces modeled by the on board model for
 * the given airframe as a function of the state and input.
 * 
 * This function is used for complementary filtering in ANDI.
 * 
 * @param[in] rates Current angular rates (p, q, r) used by the model (rad/s).
 * @param[in] vel_body Body-frame linear velocity vector (u, v, w) (m/s).
 * @param[in] actuator_state Current actuator commands/deflections array (length
 *                           ANDI_NUM_ACT). Units and normalization depend on the
 *                           airframe and actuator type (e.g. radians, throttle fraction).
 * @return Computed force vector produced by the actuators as per the OBM.
 *
 * @note Implementation is airframe-specific.
 * @see obm_cyclone.c
 */
struct FloatVect3 evaluate_obm_forces(const struct FloatRates *rates, const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT]);

/**
 * @brief Evaluate total moments acting on the vehicle from the OBM
 *
 * Computes the full set of moments modeled by the on board model for
 * the given airframe as a function of the state and input.
 * 
 * This function is used for complementary filtering in ANDI.
 * 
 * @param[in] rates Current angular rates (p, q, r) used by the model (rad/s).
 * @param[in] vel_body Body-frame linear velocity vector (u, v, w) (m/s).
 * @param[in] actuator_state Current actuator commands/deflections array (length
 *                           ANDI_NUM_ACT). Units and normalization depend on the
 *                           airframe and actuator type (e.g. radians, throttle fraction).
 * @return Computed moment vector produced by the actuators as per the OBM.
 *
 * @note Implementation is airframe-specific.
 * @see obm_cyclone.c
 */
struct FloatVect3 evaluate_obm_moments(const struct FloatRates *rates, const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT]);

/**
 * @brief Evaluate the state-dependent control effectiveness matrix F_u for stabilization.
 *
 * This function computes the mapping from actuator inputs to aerodynamic/motor outputs
 * for the stabilization model. The output is written into the provided, pre-allocated
 * array fu_mat of size ANDI_NUM_ACT * ANDI_OUTPUTS.
 *
 * The produced matrix represents the partial derivatives of the modeled outputs with
 * respect to actuator commands (∂f/∂u) evaluated at the current state and actuator
 * conditions. It is intended to be implemented per-airframe (see obm_cyclone for an
 * example).
 *
 * @param[out] fu_mat  Pre-allocated array (size ANDI_NUM_ACT * ANDI_OUTPUTS) receiving
 *                     the flattened control-effectiveness matrix. Convention: element
 *                     for output i and actuator j should be written to
 *                     fu_mat[i * ANDI_NUM_ACT + j].
 * @param[in]  rates   Current angular rates (typically p, q, r) used by the model (rad/s).
 * @param[in]  vel_body Body-frame linear velocity vector (u, v, w) (m/s).
 * @param[in]  actuator_state Current actuator commands/deflections array (length
 *                            ANDI_NUM_ACT). Units and normalization depend on the
 *                            airframe and actuator type (e.g. radians, throttle fraction).
 *
 * @note Implementations are airframe-specific and must account for the particular
 *       actuator layout and aerodynamic/motor characteristics of the platform.
 * @see obm_cyclone.c
 */
void evaluate_obm_f_stb_u(float fu_mat[ANDI_NUM_ACT * ANDI_OUTPUTS], const struct FloatRates *rates, const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT]);

/**
 * @brief Evaluate the state-dependent contribution F_x * x_dot for stabilization.
 *
 * Computes nu_obm = F_x * x_dot, the portion of the open-body-model outputs that
 * depends on the current state rates/accelerations (i.e. the state-dependent term).
 * This is the term that captures how changes in state drive the modeled outputs
 * independent of actuator inputs. 
 * 
 * @note This contribution is often neglected for INDI, but can be included for ANDI.
 *
 * @param[out] nu_obm  Pre-allocated output vector of length ANDI_OUTPUTS receiving
 *                     the computed F_x * x_dot values.
 * @param[in]  rates   Current angular rates (p, q, r) used by the model (rad/s).
 * @param[in]  vel_body Body-frame velocity vector (u, v, w) (m/s).
 * @param[in]  ang_accel Body angular accelerations (p_dot, q_dot, r_dot) (rad/s^2).
 * @param[in]  accel_body Body-frame linear accelerations (ax, ay, az) (m/s^2).
 * @param[in]  actuator_state Current actuator commands/deflections array (length
 *                            ANDI_NUM_ACT). Some terms of F_x may be actuator-state
 *                            dependent (e.g. rotor wake effects).
 *
 * @note The function computes only the state-dependent part of the model. The full
 *       modeled output is typically nu_obm + F_u * delta_u (where F_u is provided by
 *       evaluate_obm_f_stb_u). Implementations must be provided per airframe.
 * @see obm_cyclone.c
 */
void evaluate_obm_f_stb_x(float nu_obm[ANDI_OUTPUTS], const struct FloatRates *rates, const struct FloatVect3 *vel_body, const struct FloatVect3 *ang_accel, const struct FloatVect3 *accel_body, const float actuator_state[ANDI_NUM_ACT]);


/**
 * @brief Compute total thrust produced by the current actuator state.
 *
 * Returns the aggregate thrust generated by the set of actuators described by
 * actuator_state. This value is simplified typically used by the stabilization/obm code when
 * converting actuator commands to net force for direct thrust control.
 * 
 * @note This function does not intend to accurately compute the thrust force;
 *      it only returns an approximation of the total specific thrust in the body z-direction.
 *      The intended use is as feedback for direct throttle control in ANDI stabilization.
 *
 * @param[in] actuator_state Current actuator commands/deflections array (length
 *                           ANDI_NUM_ACT). Units and normalization depend on the
 *                           airframe (e.g. rotor collective, throttle fraction).
 * @return Total specific thrust produced by the actuators (SI units, m/s^2). If
 *         the airframe model uses a normalized thrust unit, document that convention
 *         in the airframe implementation.
 *
 * @note Implementation is airframe-specific.
 * @see obm_cyclone.c
 */
float evaluate_obm_thrust_z(const float actuator_state[ANDI_NUM_ACT]);
#endif // STABILIZATION_ANDI_H