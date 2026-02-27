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

#define SAFE_SQRT(x) (sqrt((x) > 0 ? (x) : 0.0))

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
static const float ACT_CUTOFF_OMEGA = 11.0f;
static const float FILT_CUTOFF_FREQ = 5;
static const Gain_t Kq = {1.0f, 1.0f, 1.0f};
static const Gain_t Komega = {5.0f, 5.0f, 5.0f};

// global vars declared as extern in header file
dbg_t dbg;
struct Fl_stabilization fl_stabilization;

// global vars
static Butterworth2LowPass act_filter[ACTUATORS_NB];
static Butterworth2LowPass rates_num_der_filter[3];
static float ACT_DYN_ALPHA;
static Act_t act = {0};
static struct FloatQuat *quat, quat_sp;
static struct FloatRates rates_sp;
static struct FloatRates *rates;
static struct FloatRates ang_accel_sp = {0., 0., 0.};
static float ang_accel_filt[3] = {0., 0., 0.};

// helper functions
static float discrete_first_order_filter(float, float, float);
static void forw_rot_flatness(float *, float *);
static void inv_rot_flatness(float, float *, float *);
static void expose_dbg_variables(struct FloatQuat *, struct FloatQuat *, 
                          struct FloatRates *, struct FloatRates *,
                          struct FloatRates *, float *,
                          Act_t *);

// -------- CODE ---------- //

static void expose_dbg_variables(struct FloatQuat *quat, struct FloatQuat *quat_sp,
                          struct FloatRates *rates, struct FloatRates *rates_sp,
                          struct FloatRates *ang_accel_sp, float *ang_accel_filt,
                          Act_t *act)
{
    dbg.quat = quat;
    dbg.quat_sp = quat_sp;
    dbg.rates = rates;
    dbg.rates_sp = rates_sp;
    dbg.ang_accel_sp = ang_accel_sp;
    dbg.ang_accel_filt = ang_accel_filt;
    dbg.act = act;
}

void flatness_stabilization_init(void)
{
    float tau = 1.0 / (2.0 * M_PI * FILT_CUTOFF_FREQ);
    float sample_time = 1.0 / PERIODIC_FREQUENCY;

    for (int i = 0; i < ACTUATORS_NB; i++) {
        init_butterworth_2_low_pass(&act_filter[i], tau, sample_time, 0.0);
    }

    for (int i = 0; i < 3; i++) {
        init_butterworth_2_low_pass(&rates_num_der_filter[i], tau, sample_time, 0.0);
    }

    ACT_DYN_ALPHA = exp(-ACT_CUTOFF_OMEGA/PERIODIC_FREQUENCY);
}

void flatness_stabilization_run(bool UNUSED in_flight, struct StabilizationSetpoint *att_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{

    // calculate quaternion error
    struct FloatQuat quat_err;
    quat_sp = stab_sp_to_quat_f(att_sp);
    quat = stateGetNedToBodyQuat_f();
    float_quat_inv_comp_norm_shortest(&quat_err, quat, &quat_sp);

    // calculate rates setpoint
    rates_sp.p = 2 * Kq.x * quat_err.qx;
    rates_sp.q = 2 * Kq.y * quat_err.qy;
    rates_sp.r = 2 * Kq.z * quat_err.qz;

    // add FF rate sp
    // struct FloatRates rate_ff = stab_sp_to_rates_f(att_sp);
    // RATES_ADD(rates_sp, rate_ff);

    // calculate angular acceleration setpoint
    rates = stateGetBodyRates_f();
    ang_accel_sp.p = Komega.x * (rates_sp.p - rates->p);
    ang_accel_sp.q = Komega.y * (rates_sp.q - rates->q);
    ang_accel_sp.r = Komega.z * (rates_sp.r - rates->r);

    // actuator state estimation + butterworth filter    
    for (int i = 0; i < ACTUATORS_NB; i++) {
        act.state[i] = discrete_first_order_filter(ACT_DYN_ALPHA, act.cmd[i], act.state[i]);
        update_butterworth_2_low_pass(&act_filter[i], act.state[i]);
        act.state_filt[i] = act_filter[i].o[0];
    }
    
    // angular acceleration numerical calculation + butterworth filter
    float rates_vector[3] = {rates->p, rates->q, rates->r};
    for (int i = 0; i < 3; i++) {
        update_butterworth_2_low_pass(&rates_num_der_filter[i], rates_vector[i]);
        ang_accel_filt[i] = (rates_num_der_filter[i].o[0] - rates_num_der_filter[i].o[1]) * PERIODIC_FREQUENCY;
    }

    // forward rotational flatness
    float m_filt[3];
    forw_rot_flatness(act.state_filt, m_filt);

    // incremental law
    float m_cmd[3];
    float ang_accel_sp_vector[3] = {ang_accel_sp.p, ang_accel_sp.q, ang_accel_sp.r};
    for (int i = 0; i < 3; i++) {
        m_cmd[i] = (ang_accel_sp_vector[i] - ang_accel_filt[i]) + m_filt[i];
    }

    // This comes from the guidance code or by the rc controller (map 0-9600 -> 0-2g)
    float specific_thrust = -(float)(2*9.81/9600)*thrust->sp.thrust_i[THRUST_AXIS_Z];

    // inverse rotational flatness
    inv_rot_flatness(specific_thrust, m_cmd, act.cmd);

    // assign commands
    for (int i = 0; i < ACTUATORS_NB; i++) {
        actuators_pprz[i] = act.cmd[i];
    }
    
    cmd[COMMAND_THRUST] = thrust->sp.thrust_i[THRUST_AXIS_Z];
    stabilization.cmd[COMMAND_THRUST] = cmd[COMMAND_THRUST]; // for autopilot_check_in_flight()

    // printf("%d\t",in_flight);
    // printf("%.0f\t%.0f\t%.0f\t%.0f", act.cmd[0], act.cmd[1], act.cmd[2], act.cmd[3]);
    // printf("\n");

    expose_dbg_variables(quat, &quat_sp, rates, &rates_sp, &ang_accel_sp, ang_accel_filt, &act);
}

static float discrete_first_order_filter(float alpha, float input, float prev_output)
{  
    // y_{k} = a*y_{k-1} + (1 - a)*x_{k}
    float output = alpha*prev_output + (1 - alpha) * input;

    return output;
}

static void forw_rot_flatness(float *u, float *m)
{
    m[0] = MU_X * (u[0]*u[0] - u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
    m[1] = MU_Y * (u[0]*u[0] + u[1]*u[1] - u[2]*u[2] - u[3]*u[3]);
    m[2] = MU_Z * (-u[0]*u[0] + u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
}

static void inv_rot_flatness(float tau, float *m, float *u)
{
    // I could do that with matrix inverse
    u[0] = SAFE_SQRT((tau/C_Z + m[0]/MU_X + m[1]/MU_Y - m[2]/MU_Z)/ACTUATORS_NB);
    u[1] = SAFE_SQRT((tau/C_Z - m[0]/MU_X + m[1]/MU_Y + m[2]/MU_Z)/ACTUATORS_NB);
    u[2] = SAFE_SQRT((tau/C_Z - m[0]/MU_X - m[1]/MU_Y - m[2]/MU_Z)/ACTUATORS_NB);
    u[3] = SAFE_SQRT((tau/C_Z + m[0]/MU_X - m[1]/MU_Y + m[2]/MU_Z)/ACTUATORS_NB);
}