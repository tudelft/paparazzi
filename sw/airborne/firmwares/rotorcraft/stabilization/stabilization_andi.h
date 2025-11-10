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
/** @file stabilization_oneloop.c
 */




 /*
 * Copyright (C) 2011-2012 The Paparazzi Team
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

#ifndef STABILIZATION_ANDI_H
#define STABILIZATION_ANDI_H

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "generated/airframe.h"
#include "filters/low_pass_filter.h"
#include <stdio.h>

#ifndef ANDI_NUM_ACT
#define ANDI_NUM_ACT COMMANDS_NB_REAL
#endif

#ifndef ANDI_OUTPUTS
#error "You must specify the number of controlled axis (outputs)"
#endif

// Reference model
struct AttQuat {
  struct FloatQuat att; 
  struct FloatRates att_d;
  struct FloatVect3 att_2d;
  struct FloatVect3 att_3d;
};

struct AttStateQuat {
  struct FloatQuat att; 
  struct FloatRates att_d;
  struct FloatVect3 att_2d;
};

struct ThrustRef {
  float thrust;
  float thrust_d;
};

// Filters
struct AttFilter {
  Butterworth2LowPass att_d_filter_p;
  Butterworth2LowPass att_d_filter_q;
  Butterworth2LowPass att_d_filter_r;
  Butterworth2LowPass att_2d_filter_x;
  Butterworth2LowPass att_2d_filter_y;
  Butterworth2LowPass att_2d_filter_z;
};

struct FilterVect3 {
  Butterworth2LowPass x;
  Butterworth2LowPass y;
  Butterworth2LowPass z;
};

struct PolesOrder3Vect3 {
  struct FloatVect3 omega_n;
  struct FloatVect3 zeta;
  struct FloatVect3 p1;
};

struct PolesOrder2Vect3 {
  struct FloatVect3 omega_n;
  struct FloatVect3 zeta;
};

struct GainsOrder2Vect3 {
  struct FloatVect3 k1;
  struct FloatVect3 k2;
};

struct GainsOrder3Vect3 {
  struct FloatVect3 k1;
  struct FloatVect3 k2;
  struct FloatVect3 k3;
};

static inline void print_FloatVect3(const char *name, struct FloatVect3 v) {
    printf("%s: [%.3f, %.3f, %.3f]\n", name, v.x, v.y, v.z);
}

static inline void print_GainsOrder2Vect3(const char *gain_name, const struct GainsOrder2Vect3 *gains) {
    printf("%s (GainsOrder2Vect3):\n", gain_name);
    print_FloatVect3("  k1", gains->k1);
    print_FloatVect3("  k2", gains->k2);
}

static inline void print_GainsOrder3Vect3(const char *gain_name, const struct GainsOrder3Vect3 *gains) {
    printf("%s (GainsOrder3Vect3):\n", gain_name);
    print_FloatVect3("  k1", gains->k1);
    print_FloatVect3("  k2", gains->k2);
    print_FloatVect3("  k3", gains->k3);
}

// static inline float rm_k1_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n * p1) / (omega_n * omega_n + 2.0f * zeta * omega_n * p1); }
// static inline float rm_k2_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n + 2.0f * zeta * omega_n * p1) / (2.0f * zeta * omega_n + p1); }
// static inline float rm_k3_order3_f(const float omega_n, const float zeta, const float p1) { return 2.0f * zeta * omega_n + p1; }
// static inline float rm_k1_order2_f(const float omega_n, const float zeta) { return omega_n / (2.0f * zeta); }
// static inline float rm_k2_order2_f(const float omega_n, const float zeta) { return 2.0f * zeta * omega_n; }

// static inline float ec_k1_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n * p1); }
// static inline float ec_k2_order3_f(const float omega_n, const float zeta, const float p1) { return (omega_n * omega_n + 2.0f * zeta * omega_n * p1); }
// static inline float ec_k3_order3_f(const float omega_n, const float zeta, const float p1) { return 2.0f * zeta * omega_n + p1; }
// static inline float ec_k1_order2_f(const float omega_n, const float zeta) { return omega_n * omega_n; }
// static inline float ec_k2_order2_f(const float omega_n, const float zeta) { return 2.0f * zeta * omega_n; }



extern union CycloneCoefficients obm_coefficients;

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

/* On board model*/
void evaluate_obm_f_stb_u(float fu_mat[ANDI_NUM_ACT * ANDI_OUTPUTS], const struct FloatRates *rates, const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT]);
// void evaluate_obm_f_stb_x(float fx_mat[ANDI_NUM_ACT * ANDI_OUTPUTS], const struct FloatRates *rates, const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT]);
float evaluate_obm_thrust(const float actuator_state[ANDI_NUM_ACT]);
#endif // STABILIZATION_ANDI_H