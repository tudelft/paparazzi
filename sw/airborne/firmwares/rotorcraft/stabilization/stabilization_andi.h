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

#ifndef STABILIZATION_ANDI_H
#define STABILIZATION_ANDI_H

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "generated/airframe.h"
#include "filters/low_pass_filter.h"
#include "math/pole_placement/pole_placement.h"

#ifndef ANDI_NUM_ACT
#define ANDI_NUM_ACT COMMANDS_NB_REAL
#endif

#ifndef ANDI_OUTPUTS
#error "You must specify the number of controlled axis (outputs)"
#endif

// Reference model
struct AttRefQuat {
  struct FloatQuat att; 
  struct FloatRates att_d;
  struct FloatVect3 att_2d;
  struct FloatVect3 att_3d;
};

struct ThrustRef {
  float thrust;
  float thrust_d;
};

// State
struct AttStateQuat {
  struct FloatQuat att; 
  struct FloatRates att_d;
  struct FloatVect3 att_2d;
};

// Filters
struct AttFilter {
  Butterworth2LowPass 
}

extern union CycloneCoefficients obm_coefficients;

/*Declaration of Reference Model and Error Controller Poles*/
extern struct PolesOrder2Vect3 andi_p_rate_e;
extern struct PolesOrder2Vect3 andi_p_rate_rm;
extern struct PolesOrder3Vect3 andi_p_att_e;
extern struct PolesOrder3Vect3 andi_p_att_rm;
extern float andi_p_thrust_e;
extern float andi_p_thrust_rm;

void stabilization_andi_init(void);
void stabilization_andi_rate_run(bool in_flight, struct StabilizationSetpoint *rate_setpoint, struct TrustSetpoint *thrust_setpoint, int32_t *cmd);
void stabilization_andi_attitude_run(bool in_flight, struct StabilizationSetpoint *attitude_setpoint, struct TrustSetpoint *thrust_setpoint, int32_t *cmd);


/* On board model*/
void evaluate_obm_f_stb_u(float fu_mat[ANDI_NUM_ACT * ANDI_OUTPUTS], const FloatRates *rates, const struct FloatVect *vel_body, const float actuator_state[ANDI_NUM_ACT]);
void evaluate_obm_f_stb_x(float fx_mat[ANDI_NUM_ACT * ANDI_OUTPUTS], const FloatRates *rates, const struct FloatVect *vel_body, const float actuator_state[ANDI_NUM_ACT]);

#endif // STABILIZATION_ANDI_H