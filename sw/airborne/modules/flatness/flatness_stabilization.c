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
#include "mcu_periph/sys_time.h"
#include "modules/energy/electrical.h"
#include "math/wls/wls_alloc.h"
#include "modules/core/abi.h"

// #include "modules/datalink/telemetry.h"

#define SAFE_SQRT(x) (sqrtf((x) > 0 ? (x) : 0.0f))

// struct WLS_t wls_stab = {
//     .nu        = 4,
//     .nv        = 4,
//     .gamma_sq  = 10000.0f,
//     .v         = {0.0f},
//     .Wv        = {1000.0f, 1000.0f, 1.0f, 100.0f},
//     .Wu        = {1.0f, 1.0f, 1.0f, 1.0f},
//     .u_pref    = {0.0f, 0.0f, 0.0f, 0.0f},
//     .u_min     = {0.0f, 0.0f, 0.0f, 0.0f},
//     .u_max     = {0.9216f, 0.9216f, 0.9216f, 0.9216f},
//     .PC        = 0.0f,
//     .SC        = 0.0f,
//     .iter      = 0
// };

// constants
static const float C_X = -0.612;
static const float C_Z = -0.079;

static const float MU_X = 60.0f  / 100000000.0f;
static const float MU_Y = 120.0f / 100000000.0f;
static const float MU_Z = 12.0f  / 100000000.0f;
static const float C_T  = -3.5f  / 100000000.0f;

static const float MU_X_v = 4.5f  / 100000000.0f;
static const float MU_Y_v = 10.4f / 100000000.0f;
static const float MU_Z_v = 0.88f  / 100000000.0f;
static const float C_T_v  = -0.22f  / 100000000.0f;

// static const float G[4][4] = {
//     {  MU_X * 100000000.0f,  -MU_X * 100000000.0f,  -MU_X * 100000000.0f,   MU_X * 100000000.0f },
//     {  MU_Y * 100000000.0f,   MU_Y * 100000000.0f,  -MU_Y * 100000000.0f,  -MU_Y * 100000000.0f },
//     { -MU_Z * 100000000.0f,   MU_Z * 100000000.0f,  -MU_Z * 100000000.0f,   MU_Z * 100000000.0f },
//     {  C_T * 100000000.0f,    C_T * 100000000.0f,    C_T * 100000000.0f,    C_T * 100000000.0f  }
// };

static const float ACT_CUTOFF_OMEGA = 11.0f;
static const float FILT_CUTOFF_FREQ = 5.0f;
static const Gain_t Kq = {3.0f, 3.5f, 2.0f};
static const Gain_t Komega = {14.0f, 16.0f, 12.0f};

// global vars declared as extern in header file
dbg_t dbg;
struct Fl_stabilization fl_stabilization;

// global vars
static abi_event rc_ev;
struct ThrustSetpoint thr_sp;
static float timestamp;
static Butterworth2LowPass act_filter[4];
static Butterworth2LowPass rates_num_der_filter[3];
static float ACT_DYN_ALPHA;
static Act_t act = {
    .cmd        = {0.0f, 0.0f, 0.0f, 0.0f},
    .state      = {0.0f},
    .state_filt = {0.0f}
};
static struct FloatQuat *quat, quat_sp;
static struct FloatRates rates_sp;
static struct FloatRates *rates;
static struct FloatRates ang_accel_sp = {0.0f, 0.0f, 0.0f};
static float ang_accel_filt[3] = {0.0f, 0.0f, 0.0f};
static int32_t temp_throttle;

// helper functions
static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc);
static float discrete_first_order_filter(float, float, float);
static void forw_rot_flatness(float *, float *);
static void inv_rot_flatness(float, float *, float *);
static void expose_dbg_variables(void);

// -------- CODE ---------- //

static void expose_dbg_variables(void)
{
    //metadata
    dbg.Kq = Kq;
    dbg.Komega = Komega;
    dbg.MU_X = MU_X;
    dbg.MU_Y = MU_Y;
    dbg.MU_Z = MU_Z;
    dbg.C_T = C_T;

    //data
    dbg.timestamp = timestamp;
    dbg.voltage = electrical.vsupply;
    dbg.throttle = temp_throttle;
    dbg.quat = quat;
    dbg.quat_sp = &quat_sp;
    dbg.rates = rates;
    dbg.rates_sp = &rates_sp;
    dbg.ang_accel_sp = &ang_accel_sp;
    dbg.ang_accel_filt = ang_accel_filt;
    dbg.act = &act;
}

static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc)
{
    int32_t rc_throttle = (int32_t)rc->values[RADIO_THROTTLE];

    THRUST_SP_SET_ZERO(thr_sp);
    thr_sp = th_sp_from_thrust_i(rc_throttle, THRUST_AXIS_Z);

    stabilization_attitude_read_rc_setpoint(&fl_stabilization.rc_in, autopilot_in_flight(), FALSE, FALSE, rc);
    fl_stabilization.rc_sp = stab_sp_from_quat_f(&fl_stabilization.rc_in.rc_quat);
}


void flatness_stabilization_init(void)
{
    float tau = 1.0f / (2.0f * M_PI * FILT_CUTOFF_FREQ);
    float sample_time = 1.0f / PERIODIC_FREQUENCY;

    for (int i = 0; i < 4; i++) {
        init_butterworth_2_low_pass(&act_filter[i], tau, sample_time, 0.0f);
    }

    for (int i = 0; i < 3; i++) {
        init_butterworth_2_low_pass(&rates_num_der_filter[i], tau, sample_time, 0.0f);
    }

    // actuator dynamics
    ACT_DYN_ALPHA = exp(-ACT_CUTOFF_OMEGA/PERIODIC_FREQUENCY);

    AbiBindMsgRADIO_CONTROL(ABI_BROADCAST, &rc_ev, rc_cb);
}

void flatness_stabilization_run(bool UNUSED in_flight, struct StabilizationSetpoint *att_sp, int32_t *cmd)
{
    // get the timestamp
    timestamp = get_sys_time_float();

    // calculate quaternion error
    struct FloatQuat quat_err;
    quat_sp = stab_sp_to_quat_f(att_sp);
    quat = stateGetNedToBodyQuat_f();
    float_quat_inv_comp_norm_shortest(&quat_err, quat, &quat_sp);

    // calculate rates setpoint
    rates_sp.p = 2.0f * Kq.x * quat_err.qx;
    rates_sp.q = 2.0f * Kq.y * quat_err.qy;
    rates_sp.r = 2.0f * Kq.z * quat_err.qz;

    // add FF rate sp
    // struct FloatRates rate_ff = stab_sp_to_rates_f(att_sp);
    // RATES_ADD(rates_sp, rate_ff);

    // calculate angular acceleration setpoint
    rates = stateGetBodyRates_f();
    ang_accel_sp.p = Komega.x * (rates_sp.p - rates->p);
    ang_accel_sp.q = Komega.y * (rates_sp.q - rates->q);
    ang_accel_sp.r = Komega.z * (rates_sp.r - rates->r);

    // actuator state estimation + butterworth filter    
    for (int i = 0; i < 4; i++) {
        act.state[i] = discrete_first_order_filter(ACT_DYN_ALPHA, act.cmd[i], act.state[i]);
        update_butterworth_2_low_pass(&act_filter[i], act.state[i]);
        act.state_filt[i] = act_filter[i].o[0];
    }
    
    // angular acceleration: butterworth filter + numerical estimation
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

    // this mapping is wrong because throttle -> specific thrust is a quadratic map!
    // I approximate with linear here
    float specific_thrust = -(float)(1.5f*9.81f/9600.0f)*thr_sp.sp.thrust_i[THRUST_AXIS_Z];

    // 1. inverse rotational flatness
    inv_rot_flatness(specific_thrust, m_cmd, act.cmd);
    
    // 2. ... or do it with matrix inverse
    // float Ginv[4][4];
    // float *Ginv_rows[4] = { Ginv[0], Ginv[1], Ginv[2], Ginv[3] };
    // float u_squared[4];
    // float indi_v[4] = {m_cmd[0], m_cmd[1], m_cmd[2], specific_thrust};
    // float_mat_inv_4d(Ginv, G);
    // float_mat_vect_mul(u_squared, Ginv_rows, indi_v, 4, 4);
    // act.cmd[0] = SAFE_SQRT(u_squared[0] * 100000000.0f);
    // act.cmd[1] = SAFE_SQRT(u_squared[1] * 100000000.0f);
    // act.cmd[2] = SAFE_SQRT(u_squared[2] * 100000000.0f);
    // act.cmd[3] = SAFE_SQRT(u_squared[3] * 100000000.0f);
    
    // 3. ... or WLS
    // WLS Control Allocator
    // float *G_rows[4] = { &G[0][0], &G[1][0], &G[2][0], &G[3][0] };
    // float indi_v[4] = {m_cmd[0], m_cmd[1], m_cmd[2], specific_thrust};
    // float u_squared[4];
    // for (int i = 0; i < 4; i++) {
    //     wls_stab.v[i] = indi_v[i];
    // }
    // wls_alloc(&wls_stab, G_rows, 0, 0, 10);
    // for (int i = 0; i < 4; i++) {
    //     u_squared[i] = wls_stab.u[i];
    // }
    // act.cmd[0] = SAFE_SQRT(u_squared[0] * 100000000.0f);
    // act.cmd[1] = SAFE_SQRT(u_squared[1] * 100000000.0f);
    // act.cmd[2] = SAFE_SQRT(u_squared[2] * 100000000.0f);
    // act.cmd[3] = SAFE_SQRT(u_squared[3] * 100000000.0f);

    // assign commands
    // for (int i = 0; i < 4; i++) {
    //     if (thrust->sp.thrust_i[THRUST_AXIS_Z] < 4500) {
    //         act.cmd[0] = 200;
    //         act.cmd[1] = 200;
    //         act.cmd[2] = 200;
    //         act.cmd[3] = 200;      
    //     }
    //     else {
    //         act.cmd[0] = 9600;
    //         act.cmd[1] = 9600;
    //         act.cmd[2] = 9600;
    //         act.cmd[3] = 9600;               
    //     }
    //     actuators_pprz[i] = act.cmd[i];
    // }
    
    for (int i = 0; i < 4; i++) {
        actuators_pprz[i] = act.cmd[i];
    }

    cmd[COMMAND_THRUST] = thr_sp.sp.thrust_i[THRUST_AXIS_Z];
    temp_throttle = cmd[COMMAND_THRUST];
    stabilization.cmd[COMMAND_THRUST] = cmd[COMMAND_THRUST]; // for autopilot_check_in_flight()

    // printf("%d\t",in_flight);
    // printf("%.0f\t%.0f\t%.0f\t%.0f\t", act.cmd[0], act.cmd[1], act.cmd[2], act.cmd[3]);
    // printf("\n");

    expose_dbg_variables();
}

// static void flatness_guidance_run() {

//     // forw flatness
//     struct NedCoor_f vel_i = stateGetSpeedNed_f();
//     struct NedCoor_f vel_b;
//     struct FloatRMat R_i2b = stateGetNedToBodyRMat_f();
    
//     float_rmat_vmult(&vel_b, &R_i2b, &vel_i);
//     float vel_norm = sqrtf(vel_b.x*vel_b.x + vel_b.y*vel_b.y + vel_b.z*vel_b.z);

//     float fbfx = C_X*vel_norm*vel_b.x;
//     float fbfz = C_Z*vel_norm*vel_b.z + C_T*(uf(1)^2 + uf(2)^2 + uf(3)^2 + uf(4)^2);
// }

static float discrete_first_order_filter(float alpha, float input, float prev_output)
{  
    // y_{k} = a*y_{k-1} + (1 - a)*x_{k}
    float output = alpha*prev_output + (1 - alpha) * input;

    return output;
}

static void forw_rot_flatness(float *u, float *m)
{
    // m[0] = MU_X * (u[0]*u[0] - u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
    // m[1] = MU_Y * (u[0]*u[0] + u[1]*u[1] - u[2]*u[2] - u[3]*u[3]);
    // m[2] = MU_Z * (-u[0]*u[0] + u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);

    float v_squared = electrical.vsupply * electrical.vsupply;

    m[0] = MU_X_v * v_squared * (u[0]*u[0] - u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
    m[1] = MU_Y_v * v_squared * (u[0]*u[0] + u[1]*u[1] - u[2]*u[2] - u[3]*u[3]);
    m[2] = MU_Z_v * v_squared * (-u[0]*u[0] + u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
}

static void inv_rot_flatness(float tau, float *m, float *u)
{
    // u[0] = SAFE_SQRT((tau/C_T + m[0]/MU_X + m[1]/MU_Y - m[2]/MU_Z)/4.0f);
    // u[1] = SAFE_SQRT((tau/C_T - m[0]/MU_X + m[1]/MU_Y + m[2]/MU_Z)/4.0f);
    // u[2] = SAFE_SQRT((tau/C_T - m[0]/MU_X - m[1]/MU_Y - m[2]/MU_Z)/4.0f);
    // u[3] = SAFE_SQRT((tau/C_T + m[0]/MU_X - m[1]/MU_Y + m[2]/MU_Z)/4.0f);

    float v_squared = electrical.vsupply * electrical.vsupply;

    u[0] = SAFE_SQRT(( tau/(C_T_v   * v_squared) + 
                       m[0]/(MU_X_v * v_squared) + 
                       m[1]/(MU_Y_v * v_squared) - 
                       m[2]/(MU_Z_v * v_squared))/4.0f );
    u[1] = SAFE_SQRT(( tau/(C_T_v   * v_squared) - 
                       m[0]/(MU_X_v * v_squared) + 
                       m[1]/(MU_Y_v * v_squared) + 
                       m[2]/(MU_Z_v * v_squared))/4.0f );
    u[2] = SAFE_SQRT(( tau/(C_T_v   * v_squared) - 
                       m[0]/(MU_X_v * v_squared) - 
                       m[1]/(MU_Y_v * v_squared) - 
                       m[2]/(MU_Z_v * v_squared))/4.0f );
    u[3] = SAFE_SQRT(( tau/(C_T_v   * v_squared) + 
                       m[0]/(MU_X_v * v_squared) - 
                       m[1]/(MU_Y_v * v_squared) + 
                       m[2]/(MU_Z_v * v_squared))/4.0f );  
}

// static void inv_transl_flatness() {

//     // hardcode by axis

//     // find theta_e and specific thrust
    
// }