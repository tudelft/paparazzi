/*
 * Copyright (C) 2026
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

/** @file "modules/guidance/guidance_unified.c"
 * A unified guidance module.
 */

#include "modules/guidance/guidance_unified.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "modules/radio_control/radio_control.h"
#include "modules/radio_control/rc_datalink.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"

static abi_event rc_ev;
static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc);

struct ThrustSetpoint thr_sp;

float yaw_damping(void);

// Gains and limits
static const float THRUST_LIMIT = 7.0f;
static const float p_LIMIT = 3.5f;
static const float q_LIMIT = 3.5f;
static const float K_P = 3.0f;
static const float K_V = 2.2f;
static const float K_A = 0.8f;
static const float K_psi = 2.0f;
static const float ROLL_RATE_GAIN = 5.0f;
static const float PITCH_RATE_GAIN = 5.0f;

// Gains and limits SIM
// static const float THRUST_LIMIT = 3.5f;
// static const float p_LIMIT = 3.5f;
// static const float q_LIMIT = 3.5f;
// static const float K_P = 0.8f;
// static const float K_V = 1.5f;
// static const float K_A = 0.8f;
// static const float K_psi = 2.0f;
// static const float ROLL_RATE_GAIN = 5.0f;
// static const float PITCH_RATE_GAIN = 5.0f;

static const float g = 9.81f;

#ifndef MOL_DRONE_WEIGHT
#error "You have to define MOL_DRONE_WEIGHT for the Swing!"
#endif
float mass = MOL_DRONE_WEIGHT;
float freq = (float)PERIODIC_FREQUENCY;

// Globally defined parameters (able to access these with logging)
float pos_ref[3];
float vel_ref[3];
float vel_a_filt[3];
float accel_ref[3];
float accel_ref_with_gains[3];
float accel_a_filt[3];
float T;
float T_filt;
float roll_rate_calc;
float pitch_rate_calc;
float cmd_yaw_rate;
float T_cmd;
float dcmd[3];
struct FloatQuat quat;
struct FloatVect3 d_accel_ref_b_calc;
struct FloatVect3 d_accel_ref_v_calc;

// Initiate Butterworth filters
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Butterworth2LowPass accel_filter_0;
Butterworth2LowPass accel_filter_1;
Butterworth2LowPass accel_filter_2;
Butterworth2LowPass T_cmd_filter;  // Commanded thrust

#ifndef FILT_CUTOFF
#error "You have to define FILT_CUTOFF for the swing!"
#endif
float cutoff_freq = FILT_CUTOFF;


struct ctrl_guidance_unified {
    struct AttitudeRCInput rc_sp;
    struct FloatRates cmd;
} ctrl;

struct ThrustSetpoint get_thrust(void)
{
    return thr_sp;
}

static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc)
{
    int32_t rc_throttle = (int32_t)rc->values[RADIO_THROTTLE];

    THRUST_SP_SET_ZERO(thr_sp);
    thr_sp = th_sp_from_thrust_i(rc_throttle, THRUST_AXIS_Z);
}

void guidance_unified_init(void)
{
    AbiBindMsgRADIO_CONTROL(ABI_BROADCAST, &rc_ev, rc_cb);
    // stabilization_attitude_rc_setpoint_init(&ctrl.rc_sp);
    
    float tau = 1.0 / (2.0 * M_PI * cutoff_freq);
    float sample_time = 1.0 / freq;

    init_butterworth_2_low_pass(&accel_filter_0, tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&accel_filter_1, tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&accel_filter_2, tau, sample_time, 0.0);

    init_butterworth_2_low_pass(&T_cmd_filter, tau, sample_time, 0.0);
}

void guidance_unified_enter(void)
{
    // ctrl.cmd.r = stateGetBodyRates_f()->r;
    // stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot_in_flight(), false, false, &radio_control);
}

void guidance_unified_run(bool in_flight)
{
    // stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot_in_flight(), false, false, &radio_control);

    //////////////// Determining input for guidance function (d_accel_ref) ////////////////
    // Counter for desired trajectory
    static int counter = 0;
    counter += 1;
    float A = 1.5f;
    float b = 2.0f;

    float t = get_sys_time_float();

    // Desired position
    // pos_ref[0] = 0.0;
    // pos_ref[0] = A * cosf(b*counter/freq);
    pos_ref[0] = A * cosf(b*t);
    pos_ref[1] = -A * sinf(b*t);
    // pos_ref[1] = 0.0;
    pos_ref[2] = -1.5;

    // Analytical derivatives of pos_ref for the feedforward input
    // Not including frequency in the derivative, as time = counter / freq
    // vel_ref[0] = 0.0;
    // vel_ref[0] = -A * b * sinf(b*counter/freq);
    vel_ref[0] = -A * b * sinf(b*t);
    vel_ref[1] = -A * b * cosf(b*t);
    // vel_ref[1] = 0.0;
    vel_ref[2] = 0.0;

    // accel_ref[0] = 0.0;
    // accel_ref[0] = -A * b * b * cosf(b * counter/freq);
    accel_ref[0] = -A * b * b * cosf(b * t);
    accel_ref[1] = A * b * b * sinf(b * t);
    // accel_ref[1] = 0.0;
    accel_ref[2] = 0.0;

    // Current positions
    struct NedCoor_f *pos_actual = stateGetPositionNed_f();
    float pos_a[3];
    pos_a[0] = pos_actual->x;
    pos_a[1] = pos_actual->y;
    pos_a[2] = pos_actual->z;

    // Current velocities 
    struct NedCoor_f *vel_actual = stateGetSpeedNed_f(); // Plots give negative values, so I'm guessing that it the get function is for velocity and not speed
    float vel_a[3];
    vel_a[0] = vel_actual->x;
    vel_a[1] = vel_actual->y;
    vel_a[2] = vel_actual->z;

    // Current accelerations
    struct NedCoor_f *accel_actual = stateGetAccelNed_f();
    float accel_a[3];
    accel_a[0] = accel_actual->x;
    accel_a[1] = accel_actual->y;
    accel_a[2] = accel_actual->z;

    accel_a_filt[0] = update_butterworth_2_low_pass(&accel_filter_0, accel_a[0]);
    accel_a_filt[1] = update_butterworth_2_low_pass(&accel_filter_1, accel_a[1]);
    accel_a_filt[2] = update_butterworth_2_low_pass(&accel_filter_2, accel_a[2]);


    // Difference in accelerations: d_accel_ref
    static float d_accel_ref[3];
    for (int i = 0; i < 3; i++) {
        float pos_component = (pos_ref[i] - pos_a[i]) * K_P;
        float vel_component = (vel_ref[i] - vel_a[i]) * K_V;
        // Filter measured acceleration
        // accel_a_filt[0] = update_butterworth_2_low_pass(&accel_filter_0, accel_a[0]);
        // accel_a_filt[1] = update_butterworth_2_low_pass(&accel_filter_1, accel_a[1]);
        // accel_a_filt[2] = update_butterworth_2_low_pass(&accel_filter_2, accel_a[2]);
        float acc_component = (accel_ref[i] - accel_a_filt[i]) * K_A;
        
        accel_ref_with_gains[i] = pos_component + vel_component + acc_component;

        d_accel_ref[i] = accel_ref_with_gains[i]; 

    }
    // Add gravity to global z coordinate
    d_accel_ref[2] = d_accel_ref[2] - g*mass;
    // d_accel_ref[2] = d_accel_ref[2] - g*0.2;

    //////////////// Control law ////////////////
    T = -ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z)*mass; //Not used for now

    // Rotation matrix, replacing eul2rotm(eulerzyx,"ZYX"). This gets the desired acceleration in the body frame
    struct FloatRMat *rot = stateGetNedToBodyRMat_f(); 

    // Calculate d_accel_ref_b via "matrix" calculation: d_accel_ref_b = rot * d_accel_ref 
    struct FloatVect3 d_accel_ref_b;
    struct FloatVect3 d_accel_ref_v = {d_accel_ref[0], d_accel_ref[1], d_accel_ref[2]};

    float_rmat_vmult(&d_accel_ref_b, rot, &d_accel_ref_v);

    d_accel_ref_v_calc = d_accel_ref_v;
    d_accel_ref_b_calc = d_accel_ref_b;

    // Calculate dcmd via "matrix" calculation: dcmd = B_inverse * d_accel_ref_b * mass;
    // Inverse of the control effectiveness matrix = {{0, 1/T, 0}, {1/T, 0, 0}, {0, 0, 1}};
    // dcmd[3] is defined globally
    //   dcmd[0] = 1/T * d_accel_ref_b.y * mass;
    //   dcmd[1] = 1/T * d_accel_ref_b.x * mass;
    //   dcmd[2] = 1 * d_accel_ref_b.z * mass;
    dcmd[0] = 1/g * d_accel_ref_b.y;
    dcmd[1] = 1/g * d_accel_ref_b.x;
    dcmd[2] = 1 * d_accel_ref_b.z * mass;

    // Quaternion
    struct FloatEulers e;
    e.psi = 0.0;     
    e.theta = dcmd[1]; 
    e.phi = dcmd[0]; 
    float_quat_of_eulers(&quat, &e); //This function employs ZYX, as in MATLab


    // Make array to return
    static float rates_guidance[3];
    // rates_guidance[0] = -PITCH_RATE_GAIN*2*quat.qy;
    // rates_guidance[1] = ROLL_RATE_GAIN*2*quat.qx;
    rates_guidance[0] = ROLL_RATE_GAIN*2*quat.qx;
    rates_guidance[1] = -PITCH_RATE_GAIN*2*quat.qy;
    rates_guidance[2] = dcmd[2]/mass;

    // Clamp roll rate
    if (rates_guidance[0] > p_LIMIT) rates_guidance[0] = p_LIMIT;
    if (rates_guidance[0] < -p_LIMIT) rates_guidance[0] = -p_LIMIT;

    // Clamp pitch rate
    if (rates_guidance[1] > q_LIMIT) rates_guidance[1] = q_LIMIT;
    if (rates_guidance[1] < -q_LIMIT) rates_guidance[1] = -q_LIMIT;

    // Clamp thrust increment
    if (rates_guidance[2] > THRUST_LIMIT) rates_guidance[2] = THRUST_LIMIT;
    if (rates_guidance[2] < -THRUST_LIMIT) rates_guidance[2] = -THRUST_LIMIT;

    // Send control to the drone (angular rates)
    ctrl.cmd.p = rates_guidance[0];
    ctrl.cmd.q = rates_guidance[1];
    roll_rate_calc = ctrl.cmd.p;
    pitch_rate_calc = ctrl.cmd.q;

    struct FloatEulers *angles = stateGetNedToBodyEulers_f();
    float psi = angles->psi;

    cmd_yaw_rate = 0.0f;

    if (psi != 0.0) {
        cmd_yaw_rate = -K_psi * psi;
    }
    // if (psi > M_PI/2) {
    //     cmd_yaw_rate = -K_psi * psi;
    // } else if (psi < M_PI/2) {
    //     cmd_yaw_rate = K_psi * psi;
    // }

    ctrl.cmd.r = cmd_yaw_rate;
    cmd_yaw_rate = ctrl.cmd.r;

    T_cmd = rates_guidance[2];

    struct StabilizationSetpoint sp = stab_sp_from_rates_f(&(ctrl.cmd));
    struct ThrustSetpoint th = th_sp_from_incr_f(T_cmd, THRUST_AXIS_Z);
    
    // execute attitude stabilization:
    stabilization_indi_rate_run(in_flight, &sp, &th, stabilization.cmd);

    // pos_ref[0] = 0.0;
    // // pos_ref[0] = A * cosf(b*counter/freq);
    // pos_ref[1] = -A * sinf(b*(t-0.1));
    // // pos_ref[1] = 0.0;
    // pos_ref[2] = -1.5;

    // // Analytical derivatives of pos_ref for the feedforward input
    // // Not including frequency in the derivative, as time = counter / freq
    // vel_ref[0] = 0.0;
    // // vel_ref[0] = -A * b * sinf(b*counter/freq);
    // vel_ref[1] = -A * b * cosf(b*(t-0.1));
    // // vel_ref[1] = 0.0;
    // vel_ref[2] = 0.0;

    // accel_ref[0] = 0.0;
    // // accel_ref[0] = -A * b * b * cosf(b * counter/freq);
    // accel_ref[1] = A * b * b * sinf(b * (t-0.1));
    // // accel_ref[1] = 0.0;
    // accel_ref[2] = 0.0;
}