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

#ifndef ANDI_NUM_ACT
#define ANDI_NUM_ACT COMMANDS_NB_REAL
#endif

#ifndef ANDI_NUM_VIRTUAL_ACT
#define ANDI_NUM_VIRTUAL_ACT COMMANDS_NB_VIRTUAL
#endif

// Number of virtual actuators (e.g. Phi, Theta). For now 2 and only 2 are supported but in the future this can be further developed. 
#if ANDI_NUM_VIRTUAL_ACT < 2
#error "You must specify the number of virtual actuators to be at least 2"
#endif

#define ANDI_NUM_ACT_TOT (ANDI_NUM_ACT + ANDI_NUM_VIRTUAL_ACT)

#ifndef ANDI_OUTPUTS
#error "You must specify the number of controlled axis (outputs)"
#endif

extern float actuator_state_1l[ANDI_NUM_ACT_TOT];
extern float andi_u[ANDI_NUM_ACT_TOT];
extern float andi_du[ANDI_NUM_ACT_TOT];


struct Poles3rdOrder1{
  float omega_n;
  float zeta;
  float p1;
};

struct Poles3rdOrder2{
  float omega_n[2];
  float zeta[2];
  float p1[2];
};

struct Poles3rdOrder3{
  float omega_n[3];
  float zeta[3];
  float p1[3];
};

struct Poles2ndOrder1{
  float omega_n;
  float zeta;
};

struct Poles2ndOrder2{
  float omega_n[2];
  float zeta[2];
};

struct Poles2ndOrder3{
  float omega_n[3];
  float zeta[3];
};

struct Gains3rdOrder1{
  float k1;
  float k2;
  float k3;
};

struct Gains3rdOrder2{
  float k1[2];
  float k2[2];
  float k3[2];
};

struct Gains3rdOrder3{
  float k1[3];
  float k2[3];
  float k3[3];
};

struct Gains2ndOrder1{
  float k1;
  float k2;
};

struct Gains2ndOrder2{
  float k1[2];
  float k2[2];
};

struct Gains2ndOrder3{
  float k1[3];
  float k2[3];
};

struct OneloopPosRef {
  float pos[2];     
  float vel[2]; 
  float acc[2];
  float jer[2];
};

struct OneloopPosState {
  float pos[2];     
  float vel[2]; 
  float acc[2];
};

struct OneloopAltRef {
  float pos;     
  float vel; 
  float acc;
  float jer;
};

struct OneloopAltState {
  float pos;     
  float vel; 
  float acc;
};

struct OneloopHeadRef {
  float head;     
  float head_d; 
  float head_2d;
};

struct OneloopHeadState {
  float head;   
  float head_d;  
};

struct OneloopAttRef {
  float att[3]; 
  float att_d[3];
  float att_2d[3];
  float att_3d[3];
};

struct OneloopAttState {
  float att[3]; 
  float att_d[3];
  float att_2d[3];
};

enum ControlMode {
  CONTROL_MODE_RATE,
  CONTROL_MODE_ATTITUDE,
  CONTROL_MODE_GUIDANCE
};

enum ControlType {
  CONTROL_TYPE_ANDI,
  CONTROL_TYPE_INDI
};

struct OneloopGeneral {
  enum ControlType control_type;
  enum ControlMode control_mode;
  float att_des[3];
  struct OneloopAttRef     att_ref;
  struct OneloopAttState   att_state;
  float pos_des[2];
  struct OneloopPosRef     pos_ref;       // Guidance References
  struct OneloopPosState   pos_state;     // Guidance State
  float alt_des;
  struct OneloopAltRef     alt_ref;       // Altitude References
  struct OneloopAltState   alt_state;     // Altitude State
  float head_des;
  struct OneloopHeadRef    head_ref;      // Heading References
  struct OneloopHeadState  head_state;    // Heading State
};

extern struct OneloopGeneral oneloop_andi;

enum FilterType {
  LOWPASS_1,
  BUTTERWORTH_2,
  BUTTERWORTH_4,
};

/**
 * @brief Structure representing filter parameters and state measurement filtering.
 * 
 * Contains frequency, and measurement data.
 * Supports multiple filter types (first-order low-pass, Butterworth 2nd and 4th order, and notch filters)
 * using a union to store the specific filter parameters.
 */
struct Filter {
  float  freq;
  float  meas;
  float  out;
  enum FilterType filter_type;
  union {
    struct FirstOrderLowPass lp1;
    Butterworth2LowPass bw2;
    Butterworth4LowPass bw4;
  };
};



struct GainsOrder2Vect3{
  struct FloatVect3 k1;
  struct FloatVect3 k2;
};

struct GainsOrder2Vect2{
  struct FloatVect2 k1;
  struct FloatVect2 k2;
};

struct GainsOrder2{
  float k1;
  float k2;
};

struct GainsOrder3Vect3{
  struct FloatVect3 k1;
  struct FloatVect3 k2;
  struct FloatVect3 k3;
};

struct GainsOrder3Vect2{
  struct FloatVect2 k1;
  struct FloatVect2 k2;
  struct FloatVect2 k3;
};

struct GainsOrder3{
  float k1;
  float k2;
  float k3;
};

struct OneloopAttRefQuat {
  struct FloatQuat att; 
  struct FloatRates att_d;
  struct FloatVect3 att_2d;
  struct FloatVect3 att_3d;
};

struct OneloopThrustRef {
  float thrust;
  float thrust_d;
};

struct OneloopAttStateQuat {
  struct FloatQuat att; 
  struct FloatRates att_d;
  struct FloatVect3 att_2d;
};

union CycloneCoefficients {
    struct {
        // X-axis force coefficients (f_ff_x)
        float fx_motor_squared;          // Motor thrust squared effect
        float fx_speed_forward;          // Forward speed effect

        // Y-axis force coefficients (f_ff_y) 
        float fy_speed_lateral;          // Lateral speed effect

        // Z-axis force coefficients (f_ff_z)
        float fz_motor_squared;          // Motor thrust squared effect
        float fz_speed_forward;          // Forward speed effect
        float fz_speed_vertical;         // Vertical speed effect
        float fz_elevator_speed;         // Elevator-speed coupling
        float fz_elevator_motor;         // Elevator-motor coupling

        // X-axis moment coefficients (m_ff_x)
        float mx_motor_diff;             // (motor_l^2 - motor_r^2)
        float mx_elevator_motor_diff;    // (ele_l * motor_l^2 - ele_r * motor_r^2)
        float mx_elevator_speed_diff;    // (ele_l - ele_r) * speed * v_ff(1)
        float mx_angular_coupling;       // w_ff(2) * w_ff(3)

        // Y-axis moment coefficients (m_ff_y)
        float my_speed_forward;          // speed * v_ff(1)
        float my_speed_vertical;         // speed * v_ff(3)
        float my_constant_zero;          // constant 0 term
        float my_motor_sum;              // motor_l^2 + motor_r^2
        float my_elevator_motor_sum;     // ele_l * motor_l^2 + ele_r * motor_r^2
        float my_elevator_speed_sum;     // (ele_l + ele_r) * speed * v_ff(1)
        float my_angular_sum;            // w_ff(1) + w_ff(3)

        // Z-axis moment coefficients (m_ff_z)
        float mz_speed_lateral;          // speed * v_ff(2)
        float mz_motor_diff;             // motor_l^2 - motor_r^2
        float mz_speed_roll;             // speed * w_ff(1)
        float mz_angular_coupling;       // w_ff(1) * w_ff(2)
    };
    float data[23];
}; 

extern union CycloneCoefficients obm_coefficients;



/*Declaration of Reference Model and Error Controller Gains*/
/*Rate Loop*/
extern struct Poles2ndOrder3 p_rate_e;
extern struct Poles2ndOrder3 p_rate_rm;
extern struct Poles3rdOrder3 p_att_e;
extern struct Poles3rdOrder3 p_att_rm;
extern struct Poles3rdOrder2 p_pos_e;
extern struct Poles3rdOrder2 p_pos_rm;
extern struct Poles3rdOrder1 p_alt_e;
extern struct Poles3rdOrder1 p_alt_rm;
extern struct Poles2ndOrder1 p_head_e;
extern struct Poles2ndOrder1 p_head_rm;

/*Gains of EC and RM*/
extern struct Gains2ndOrder3 k_rate_e;
extern struct Gains2ndOrder3 k_rate_rm;
extern struct Gains3rdOrder3 k_att_e;
extern struct Gains3rdOrder3 k_att_rm;
extern struct Gains3rdOrder2 k_pos_e;
extern struct Gains3rdOrder2 k_pos_rm;
extern struct Gains3rdOrder1 k_alt_e;
extern struct Gains3rdOrder1 k_alt_rm;
extern struct Gains2ndOrder1 k_head_e;
extern struct Gains2ndOrder1 k_head_rm;

void oneloop_andi_init(void);
void oneloop_andi_enter(enum ControlMode control_mode_sp, enum ControlType control_type);
void oneloop_andi_run(enum ControlMode control_mode_sp);
#endif  // ONELOOP_ANDI_H
