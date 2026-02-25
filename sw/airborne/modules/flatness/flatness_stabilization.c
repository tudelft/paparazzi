/*
 * Copyright (C) 2026 Evangelos Ntouros <e.ntouros@tudelft.nl>
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

/** @file "modules/flatness/flatness_stabilization.c"
 * @author Evangelos Ntouros <e.ntouros@tudelft.nl>
 * A differential flatness-based INDI stabilization controller.
 */

#include "modules/flatness/flatness_stabilization.h"
#include "modules/actuators/actuators.h"

#include "filters/low_pass_filter.h"

// #include "modules/datalink/telemetry.h"

typedef struct {
    float x;
    float y;
    float z;
} Gain_t;

// constants
static const float PHI_COEFF_DIV = 100000000;
static const float MU_X = 70  / PHI_COEFF_DIV;
static const float MU_Y = 150 / PHI_COEFF_DIV;
static const float MU_Z = 15  / PHI_COEFF_DIV;
static const float C_Z  = -5  / PHI_COEFF_DIV;
static const float ACT_CUTOFF_FREQ = 11.0f;
static const float FILT_CUTOFF = 5;
static const Gain_t Kq = {1.0f, 1.0f, 1.0f};
static const Gain_t Komega = {5.0f, 5.0f, 5.0f};

// global vars
dbg_t dbg;
struct Fl_stabilization fl_stabilization;
static Butterworth2LowPass act_filter[ACTUATORS_NB];
static Butterworth2LowPass rates_num_der_filter[3];
static float ACT_DYN_ALPHA;

// helper functions
float discrete_first_order_filter(float, float, float);
void expose_dbg_variables(float *, float *, float *);
void forw_rot_flatness(float *, float *);
void inv_rot_flatness(float, float *, float *);

void expose_dbg_variables(float *u_cmd, float *u, float *u_filt)
{
    // shallow copy, be carefull!
    dbg.u_cmd = u_cmd;
    dbg.u = u;
    dbg.u_filt = u_filt;
}

void flatness_stabilization_init(void)
{
    float tau = 1.0 / (2.0 * M_PI * FILT_CUTOFF);
    float sample_time = 1.0 / PERIODIC_FREQUENCY;

    for (int i = 0; i < ACTUATORS_NB; i++) {
        init_butterworth_2_low_pass(&act_filter[i], tau, sample_time, 0.0);
    }

    for (int i = 0; i < 3; i++) {
        init_butterworth_2_low_pass(&rates_num_der_filter[i], tau, sample_time, 0.0);
    }

    ACT_DYN_ALPHA = exp(-ACT_CUTOFF_FREQ/PERIODIC_FREQUENCY);
}

void flatness_stabilization_run(bool in_flight, struct StabilizationSetpoint *att_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
    // calculate quaternion error
    struct FloatQuat quat_err;
    struct FloatQuat quat_sp = stab_sp_to_quat_f(att_sp);
    struct FloatQuat *quat = stateGetNedToBodyQuat_f();
    float_quat_inv_comp_norm_shortest(&quat_err, quat, &quat_sp);

    // calculate rates setpoint
    struct FloatRates rates_sp;
    rates_sp.p = 2 * Kq.x * quat_err.qx;
    rates_sp.q = 2 * Kq.y * quat_err.qy;
    rates_sp.r = 2 * Kq.z * quat_err.qz;

    // add FF rate sp
    // struct FloatRates rate_ff = stab_sp_to_rates_f(att_sp);
    // RATES_ADD(rates_sp, rate_ff);

    // calculate angular acceleration setpoint
    struct FloatRates *rates = stateGetBodyRates_f();
    struct FloatRates ang_accel_sp = {0., 0., 0.};
    ang_accel_sp.p = Komega.x * (rates_sp.p - rates->p);
    ang_accel_sp.q = Komega.y * (rates_sp.q - rates->q);
    ang_accel_sp.r = Komega.z * (rates_sp.r - rates->r);

    // butterworth filter for actuators    
    float u[ACTUATORS_NB] = { 0., 0., 0., 0. };
    float u_cmd[ACTUATORS_NB] = { 0., 0., 0., 0. };
    float u_filt[ACTUATORS_NB] = { 0., 0., 0., 0. };
    for (int i = 0; i < ACTUATORS_NB; i++) {
        u[i] = discrete_first_order_filter(ACT_DYN_ALPHA, u_cmd[i], u[i]);
        update_butterworth_2_low_pass(&act_filter[i], u[i]);
        u_filt[i] = act_filter[i].o[0];
    }
    
    // numerical derivative + butterworth filter for angular acceleration
    float ang_accel_filt[3] = {0., 0., 0.};
    float rates_vector[3] = {rates->p, rates->q, rates->r};
    for (int i = 0; i < 3; i++) {
        update_butterworth_2_low_pass(&rates_num_der_filter[i], rates_vector[i]);
        ang_accel_filt[i] = (rates_num_der_filter[i].o[0] - rates_num_der_filter[i].o[1]) * PERIODIC_FREQUENCY;
    }

    // forward rotational flatness
    float m_filt[3];
    forw_rot_flatness(u_filt, m_filt);

    // incremental law
    float m_cmd[3];
    float ang_accel_sp_vector[3] = {ang_accel_sp.p, ang_accel_sp.q, ang_accel_sp.r};
    for (int i = 0; i < 3; i++) {
        m_cmd[i] = (ang_accel_sp_vector[i] - ang_accel_filt[i]) + m_filt[i];
    }

    // inverse rotational flatness
    float specific_thrust = -10; // m/s^2
    inv_rot_flatness(specific_thrust, m_cmd, u_cmd);

    if (in_flight) {
        // assign commands
        for (int i = 0; i < ACTUATORS_NB; i++) {
            // actuators_pprz[i] = (int16_t) thrust->sp.thrust_i[THRUST_AXIS_Z];
            actuators_pprz[i] = u_cmd[i];
        }
        cmd[COMMAND_THRUST] = (actuators_pprz[0] + actuators_pprz[1] + actuators_pprz[2] + actuators_pprz[3])/4;
    }
    stabilization.cmd[COMMAND_THRUST] = cmd[COMMAND_THRUST]; // for autopilot_check_in_flight()

    printf("%f\t%f\t%f\n", u_cmd[0], u[0], u_filt[0]);

    expose_dbg_variables(u_cmd, u, u_filt);
}

float discrete_first_order_filter(float alpha, float input, float prev_output)
{  
    float output = alpha*prev_output + (1 - alpha) * input;

    return output;
}

void forw_rot_flatness(float *u, float *m)
{
    m[0] = MU_X * (u[0]*u[0] - u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
    m[1] = MU_Y * (u[0]*u[0] + u[1]*u[1] - u[2]*u[2] - u[3]*u[3]);
    m[2] = MU_Z * (-u[0]*u[0] + u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
}

void inv_rot_flatness(float tau, float *m, float *u)
{
    // I could do that with matrix inverse
    u[0] = sqrt((tau/C_Z + m[0]/MU_X + m[1]/MU_Y - m[2]/MU_Z)/4);
    u[1] = sqrt((tau/C_Z - m[0]/MU_X + m[1]/MU_Y + m[2]/MU_Z)/4);
    u[2] = sqrt((tau/C_Z - m[0]/MU_X - m[1]/MU_Y - m[2]/MU_Z)/4);
    u[3] = sqrt((tau/C_Z + m[0]/MU_X - m[1]/MU_Y + m[2]/MU_Z)/4);
}