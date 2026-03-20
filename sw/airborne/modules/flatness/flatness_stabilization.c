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
#include "modules/radio_control/radio_control.h"
#include "modules/core/abi.h"
#include "autopilot.h"
#include "state.h"

// #include "modules/datalink/telemetry.h"

#define SAFE_SQRT(x) (sqrtf((x) > 0 ? (x) : 0.0f))

typedef enum {
    FSM_INIT = 0,
    FSM_STANDBY,
    FSM_TRAJECTORY
} fl_guid_fsm_t;

// constants
// static const float C_X = -0.612f;
// static const float C_Z = -0.079f;
static const float C_X = 0.0f;
static const float C_Z = 0.0f;

static const float MU_X_v = 4.5f  / 100000000.0f;
static const float MU_Y_v = 10.4f / 100000000.0f;
static const float MU_Z_v = 0.88f  / 100000000.0f;
static const float C_T_v  = -0.25f  / 100000000.0f;

static const float MIN_TAU = -0.981f;
static const float MAX_TAU = -2.0f*9.81f;
static const float ACCEL_BOUND = 9.81f/1.0f;

static const float ACT_CUTOFF_OMEGA = 11.0f;
static const float FILT_CUTOFF_FREQ = 5.0f;
static const Gain_t Kq = {3.0f, 3.0f, 3.0f};
static const Gain_t Komega = {14.0f, 15.0f, 14.0f};
static const Gain_t Kp = {1.0f, 1.0f, 1.0f};
static const Gain_t Kv = {2.0f, 2.0f, 2.0f};

static const struct FloatVect3 POS_END = {0.0f, 0.0f, -2.0f};
static const float P2P_DT = 3.0f;  

// global vars declared as extern in header file
dbg_t dbg;
struct Fl_stabilization fl_stabilization;

// global vars
static abi_event rc_ev;
struct ThrustSetpoint rc_thr_sp;
static float timestamp;
static Butterworth2LowPass act_filter[4];
static Butterworth2LowPass accel_filter[3];
static Butterworth2LowPass spec_force_filter[4];
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
static float temp_spec_thrust_sp;
struct FloatVect3 pos_ref = {0.0f, 0.0f, 0.0f};
struct FloatVect3 vel_ref = {0.0f, 0.0f, 0.0f};
struct FloatVect3 accel_ref = {0.0f, 0.0f, 0.0f};
static struct FloatVect3 vel_sp, accel_sp;
static float accel_vector[3], accel_filt[3];
float f_cmd[3];
fl_guid_fsm_t fsm_state = FSM_INIT;
float quintic_coeff_n[6], quintic_coeff_e[6], quintic_coeff_d[6];
float timestamp_p2p_start, timestamp_p2p;
bool flatness_guided;

// helper functions
static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc);
static float discrete_first_order_filter(float, float, float);
static void forw_rot_flatness(float *, float *);
static void inv_rot_flatness(float, float *, float *);
static void forw_transl_flatness(float, struct FloatVect3 *, float *, struct FloatVect3 *);
static void expose_dbg_variables(void);
float spec_thrust_from_throttle(float throttle, float min_spec_thrust, float max_spec_thrust);
float throttle_from_spec_thrust(float spec_thrust, float min_spec_thrust, float max_spec_thrust);
void flatness_guidance_run(bool in_flight, int32_t *cmd);
void compute_quintic_ref(float t, struct FloatVect3 *pos_start, const struct FloatVect3 *pos_end);
void compute_quintic_coefficients(struct FloatVect3 *pos_start, const struct FloatVect3 *pos_end, struct FloatVect3 *vel_start, struct FloatVect3 *accel_start);

// -------- CODE ---------- //

static void expose_dbg_variables(void)
{
    //metadata
    dbg.Kq = Kq;
    dbg.Komega = Komega;
    dbg.Kp = Kp;
    dbg.Kv = Kv;
    dbg.MU_X_v = MU_X_v;
    dbg.MU_Y_v = MU_Y_v;
    dbg.MU_Z_v = MU_Z_v;
    dbg.C_T_v = C_T_v;

    //data
    dbg.timestamp = timestamp;
    dbg.voltage = electrical.vsupply;
    dbg.throttle = temp_throttle;
    dbg.spec_thrust_sp = temp_spec_thrust_sp;

    dbg.pos_ref = pos_ref;
    dbg.vel_ref = vel_ref;
    dbg.accel_ref = accel_ref;
    dbg.vel_sp = vel_sp;
    dbg.accel_sp = accel_sp;
    dbg.f_cmd.x = f_cmd[0]; dbg.f_cmd.y = f_cmd[1]; dbg.f_cmd.z = f_cmd[2];
    dbg.accel_filt.x = accel_filt[0]; dbg.accel_filt.y = accel_filt[1]; dbg.accel_filt.z = accel_filt[2];

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

    THRUST_SP_SET_ZERO(rc_thr_sp);
    rc_thr_sp = th_sp_from_thrust_i(rc_throttle, THRUST_AXIS_Z);

    stabilization_attitude_read_rc_setpoint(&fl_stabilization.rc_in, autopilot_in_flight(), FALSE, FALSE, rc);
    fl_stabilization.rc_sp = stab_sp_from_quat_f(&fl_stabilization.rc_in.rc_quat);
}

float get_spec_thrust(void)
{
    float spec_thrust = spec_thrust_from_throttle((float)rc_thr_sp.sp.thrust_i[THRUST_AXIS_Z], MIN_TAU, MAX_TAU);

    return spec_thrust;
}

float spec_thrust_from_throttle(float throttle, float min_spec_thrust, float max_spec_thrust)
{
    // a linear map: [0 9600] --> [min max]
    return min_spec_thrust + (throttle - 0.0f)*(max_spec_thrust - min_spec_thrust)/(9600.0f - 0.0f);
}

float throttle_from_spec_thrust(float spec_thrust, float min_spec_thrust, float max_spec_thrust)
{
    // a linear map: [0 9600] <-- [min max]
    return 0.0f + (spec_thrust - min_spec_thrust)*(9600.0f - 0.0f)/(max_spec_thrust - min_spec_thrust);
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
        init_butterworth_2_low_pass(&accel_filter[i], tau, sample_time, 0.0f);
        init_butterworth_2_low_pass(&spec_force_filter[i], tau, sample_time, 0.0f);
    }

    // actuator dynamics
    ACT_DYN_ALPHA = exp(-ACT_CUTOFF_OMEGA/PERIODIC_FREQUENCY);

    AbiBindMsgRADIO_CONTROL(ABI_BROADCAST, &rc_ev, rc_cb);
}

void flatness_stabilization_run(bool UNUSED in_flight, struct StabilizationSetpoint *att_sp, float spec_thrust_sp, int32_t *cmd)
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

    if (flatness_guided == false) {
        // actuator state estimation + butterworth filter    
        for (int i = 0; i < 4; i++) {
            act.state[i] = discrete_first_order_filter(ACT_DYN_ALPHA, act.cmd[i], act.state[i]);
            update_butterworth_2_low_pass(&act_filter[i], act.state[i]);
            act.state_filt[i] = act_filter[i].o[0];
        }
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

    // 1. inverse rotational flatness
    inv_rot_flatness(spec_thrust_sp, m_cmd, act.cmd);
    
    for (int i = 0; i < 4; i++) {
        actuators_pprz[i] = act.cmd[i];
    }

    cmd[COMMAND_THRUST] = throttle_from_spec_thrust(spec_thrust_sp, MIN_TAU, MAX_TAU); // for GCS throttle display
    stabilization.cmd[COMMAND_THRUST] = cmd[COMMAND_THRUST]; // for autopilot_check_in_flight()
    temp_throttle = cmd[COMMAND_THRUST]; // for logging
    temp_spec_thrust_sp = spec_thrust_sp; // for logging

    // printf("%d\t",in_flight);
    // printf("%.2f\t", spec_thrust_sp);
    // printf("%.0f\t%.0f\t%.0f\t%.0f\t", act.cmd[0], act.cmd[1], act.cmd[2], act.cmd[3]);
    // printf("\n");

    expose_dbg_variables();
}

void flatness_guidance_run(bool UNUSED in_flight, int32_t *cmd) {

    // position P controller
    struct NedCoor_f *pos = stateGetPositionNed_f();
    struct FloatVect3 vel_sp0;
    vel_sp0.x = Kp.x * (pos_ref.x - pos->x);
    vel_sp0.y = Kp.y * (pos_ref.y - pos->y);
    vel_sp0.z = Kp.z * (pos_ref.z - pos->z);

    vel_sp.x = vel_sp0.x + vel_ref.x;
    vel_sp.y = vel_sp0.y + vel_ref.y;
    vel_sp.z = vel_sp0.z + vel_ref.z;

    // velocity P controller
    struct NedCoor_f *vel_i_nedcoor_f = stateGetSpeedNed_f();
    struct FloatVect3 *vel_i = (struct FloatVect3 *)vel_i_nedcoor_f;
    struct FloatVect3 accel_sp0;
    accel_sp0.x = Kv.x * (vel_sp.x - vel_i->x);
    accel_sp0.y = Kv.y * (vel_sp.y - vel_i->y);
    accel_sp0.z = Kv.z * (vel_sp.z - vel_i->z);

    accel_sp.x = accel_sp0.x + accel_ref.x;
    accel_sp.y = accel_sp0.y + accel_ref.y;
    accel_sp.z = accel_sp0.z + accel_ref.z;

    // bound the commanded acceleration
    if (accel_sp.x > ACCEL_BOUND)
        accel_sp.x = ACCEL_BOUND;
    else if (accel_sp.x < -ACCEL_BOUND)
        accel_sp.x = -ACCEL_BOUND;
    
    if (accel_sp.y > ACCEL_BOUND)
        accel_sp.y = ACCEL_BOUND;
    else if (accel_sp.y < -ACCEL_BOUND)
        accel_sp.y = -ACCEL_BOUND;

    if (accel_sp.z > ACCEL_BOUND)
        accel_sp.z = ACCEL_BOUND;
    else if (accel_sp.z < -ACCEL_BOUND)
        accel_sp.z = -ACCEL_BOUND;

    // actuator state estimation + butterworth filter    
    for (int i = 0; i < 4; i++) {
        act.state[i] = discrete_first_order_filter(ACT_DYN_ALPHA, act.cmd[i], act.state[i]);
        update_butterworth_2_low_pass(&act_filter[i], act.state[i]);
        act.state_filt[i] = act_filter[i].o[0];
    }

    // calculate body velocity and norm.
    struct FloatRMat *R_i2b = stateGetNedToBodyRMat_f();
    struct FloatVect3 vel_b;
    float_rmat_vmult(&vel_b, R_i2b, vel_i);
    float vel_norm = sqrtf(vel_b.x*vel_b.x + vel_b.y*vel_b.y + vel_b.z*vel_b.z);

    // forw flatness -> rotate -> filter
    struct FloatVect3 fb, fi;
    forw_transl_flatness(vel_norm, &vel_b, act.state, &fb);
    float_rmat_transp_vmult(&fi, R_i2b, &fb);
    float fi_vector[3] = {fi.x, fi.y, fi.z}; 
    float fi_vector_filt[3];
    for (int i = 0; i < 3; i++) {    
        update_butterworth_2_low_pass(&spec_force_filter[i], fi_vector[i]);
        fi_vector_filt[i] = spec_force_filter[i].o[0];
    }   

    // incremental law
    struct NedCoor_f *accel = stateGetAccelNed_f();
    accel_vector[0] = accel->x;
    accel_vector[1] = accel->y;
    accel_vector[2] = accel->z;
    for (int i = 0; i < 3; i++) {    
        update_butterworth_2_low_pass(&accel_filter[i], accel_vector[i]);
        accel_filt[i] = accel_filter[i].o[0];
    }
    f_cmd[0] = (accel_sp.x - accel_filt[0]) + fi_vector_filt[0];
    f_cmd[1] = (accel_sp.y - accel_filt[1]) + fi_vector_filt[1];
    f_cmd[2] = (accel_sp.z - accel_filt[2]) + fi_vector_filt[2];

    // inverse translational flatness (get attitude and spec. thrust sp)
    struct FloatEulers eulers_sp = {0.0f, 0.0f, 0.0f};
    float beta_x = -sinf(eulers_sp.psi)*f_cmd[0] + cosf(eulers_sp.psi)*f_cmd[1];
    float beta_z = f_cmd[2];
    eulers_sp.phi = atan2f(beta_x, -beta_z);

    struct FloatRMat R_i2e;
    float_rmat_of_eulers_312(&R_i2e, &eulers_sp);
    struct FloatVect3 ve, fe;
    struct FloatVect3 fi_cmd = {f_cmd[0], f_cmd[1], f_cmd[2]};
    float_rmat_vmult(&ve, &R_i2e, vel_i);
    float_rmat_vmult(&fe, &R_i2e, &fi_cmd);

    float sigma_x = fe.x - C_X*vel_norm*ve.x;
    float sigma_z = fe.z - C_X*vel_norm*ve.z;
    float theta_e = atan2f(-sigma_x, -sigma_z);

    float spec_thrust_sp = sinf(theta_e)*fe.x + 
                        cos(theta_e)*fe.z - 
                        C_Z*vel_norm*(sin(theta_e)*ve.x + cos(theta_e)*ve.z);

    eulers_sp.theta = theta_e;
    struct FloatQuat _quat_sp;
    float_quat_of_eulers_zxy(&_quat_sp, &eulers_sp);
    struct StabilizationSetpoint _att_sp = stab_sp_from_quat_f(&_quat_sp);

    flatness_stabilization_run(in_flight, &_att_sp, spec_thrust_sp, cmd);

    // printf("%.2f\t%.2f\t%.2f\t%.2f\t", accel_sp.z , accel_filt[2], fi_filt.z, f_cmd[2]);
    // printf("%.0f\t%.0f\t%.0f\t%.0f\t", act.cmd[0], act.cmd[1], act.cmd[2], act.cmd[3]);
    // printf("\n");
}

static void forw_transl_flatness(float vel_norm, struct FloatVect3 *vel_b, float *u, struct FloatVect3 *fb)
{
    float v_squared = electrical.vsupply * electrical.vsupply;

    fb->x = C_X*vel_norm*vel_b->x;
    fb->y = 0; // remove?
    fb->z = C_Z*vel_norm*vel_b->z + C_T_v * v_squared * (u[0]*u[0] + u[1]*u[1] + u[2]*u[2] + u[3]*u[3]);
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

static float discrete_first_order_filter(float alpha, float input, float prev_output)
{  
    // y_{k} = a*y_{k-1} + (1 - a)*x_{k}
    float output = alpha*prev_output + (1 - alpha) * input;

    return output;
}

void flatness_guidance_fsm(bool UNUSED in_flight, int32_t *cmd)
{
    switch (fsm_state)
    {
        case FSM_INIT:
            // printf("fsm_init\n");
            timestamp_p2p_start = get_sys_time_float();
            struct FloatVect3 *pos_start = (struct FloatVect3 *)stateGetPositionNed_f();
            struct FloatVect3 *vel_start = (struct FloatVect3 *)stateGetSpeedNed_f();
            struct FloatVect3 *accel_start = (struct FloatVect3 *)stateGetAccelNed_f();
            compute_quintic_coefficients(pos_start, &POS_END, vel_start, accel_start);
            fsm_state = FSM_STANDBY;
             /* fall through */ 

        case FSM_STANDBY:
            // printf("fsm_stdby\n");
            timestamp_p2p = get_sys_time_float() - timestamp_p2p_start;
            if (timestamp_p2p > P2P_DT) {
                timestamp_p2p = P2P_DT;
            } 
            compute_quintic_ref(timestamp_p2p, pos_start, &POS_END);
            flatness_guidance_run(in_flight, cmd);

            // if (/* condition to start trajectory */) {
            //     fsm_state = FSM_TRAJECTORY;
            // }
            break;

        case FSM_TRAJECTORY:
            // TODO: trajectory execution logic
            break;

        default:
            fsm_state = FSM_INIT;
            break;
    }
}

void compute_quintic_ref(float t, struct FloatVect3 *pos_start, const struct FloatVect3 *pos_end)
{
    float t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
    pos_ref.x = pos_start->x + (quintic_coeff_n[1]*t + quintic_coeff_n[2]*t2 + quintic_coeff_n[3]*t3 + quintic_coeff_n[4]*t4 + quintic_coeff_n[5]*t5)*(pos_end->x - pos_start->x);
    pos_ref.y = pos_start->y + (quintic_coeff_e[1]*t + quintic_coeff_e[2]*t2 + quintic_coeff_e[3]*t3 + quintic_coeff_e[4]*t4 + quintic_coeff_e[5]*t5)*(pos_end->y - pos_start->y);
    pos_ref.z = pos_start->z + (quintic_coeff_d[1]*t + quintic_coeff_d[2]*t2 + quintic_coeff_d[3]*t3 + quintic_coeff_d[4]*t4 + quintic_coeff_d[5]*t5)*(pos_end->z - pos_start->z);

    vel_ref.x = (quintic_coeff_n[1] + 2*quintic_coeff_n[2]*t + 3*quintic_coeff_n[3]*t2 + 4*quintic_coeff_n[4]*t3 + 5*quintic_coeff_n[5]*t4)*(pos_end->x - pos_start->x);
    vel_ref.y = (quintic_coeff_e[1] + 2*quintic_coeff_e[2]*t + 3*quintic_coeff_e[3]*t2 + 4*quintic_coeff_e[4]*t3 + 5*quintic_coeff_e[5]*t4)*(pos_end->y - pos_start->y);
    vel_ref.z = (quintic_coeff_d[1] + 2*quintic_coeff_d[2]*t + 3*quintic_coeff_d[3]*t2 + 4*quintic_coeff_d[4]*t3 + 5*quintic_coeff_d[5]*t4)*(pos_end->z - pos_start->z);

    accel_ref.x = (2*quintic_coeff_n[2] + 6*quintic_coeff_n[3]*t + 12*quintic_coeff_n[4]*t2 + 20*quintic_coeff_n[5]*t3)*(pos_end->x - pos_start->x);
    accel_ref.y = (2*quintic_coeff_e[2] + 6*quintic_coeff_e[3]*t + 12*quintic_coeff_e[4]*t2 + 20*quintic_coeff_e[5]*t3)*(pos_end->y - pos_start->y);
    accel_ref.z = (2*quintic_coeff_d[2] + 6*quintic_coeff_d[3]*t + 12*quintic_coeff_d[4]*t2 + 20*quintic_coeff_d[5]*t3)*(pos_end->z - pos_start->z);
}

void compute_quintic_coefficients(struct FloatVect3 *pos_start, const struct FloatVect3 *pos_end, struct FloatVect3 *vel_start, struct FloatVect3 *accel_start)
{
    quintic_coeff_n[0] = 0;
    quintic_coeff_n[1] = vel_start->x/(pos_end->x - pos_start->x);
    quintic_coeff_n[2] = accel_start->x/(2*(pos_end->x - pos_start->x));
    quintic_coeff_n[3] = -(3*quintic_coeff_n[2]*P2P_DT*P2P_DT + 6*quintic_coeff_n[1]*P2P_DT - 10)/(P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_n[4] = (3*quintic_coeff_n[2]*P2P_DT*P2P_DT + 8*quintic_coeff_n[1]*P2P_DT - 15)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_n[5] = -(quintic_coeff_n[2]*P2P_DT*P2P_DT + 3*quintic_coeff_n[1]*P2P_DT - 6)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT*P2P_DT);

    quintic_coeff_e[0] = 0;
    quintic_coeff_e[1] = vel_start->y/(pos_end->y - pos_start->y);
    quintic_coeff_e[2] = accel_start->y/(2*(pos_end->y - pos_start->y));
    quintic_coeff_e[3] = -(3*quintic_coeff_e[2]*P2P_DT*P2P_DT + 6*quintic_coeff_e[1]*P2P_DT - 10)/(P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_e[4] = (3*quintic_coeff_e[2]*P2P_DT*P2P_DT + 8*quintic_coeff_e[1]*P2P_DT - 15)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_e[5] = -(quintic_coeff_e[2]*P2P_DT*P2P_DT + 3*quintic_coeff_e[1]*P2P_DT - 6)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT*P2P_DT);

    quintic_coeff_d[0] = 0;
    quintic_coeff_d[1] = vel_start->z/(pos_end->z - pos_start->z);
    quintic_coeff_d[2] = accel_start->z/(2*(pos_end->z - pos_start->z));
    quintic_coeff_d[3] = -(3*quintic_coeff_d[2]*P2P_DT*P2P_DT + 6*quintic_coeff_d[1]*P2P_DT - 10)/(P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_d[4] = (3*quintic_coeff_d[2]*P2P_DT*P2P_DT + 8*quintic_coeff_d[1]*P2P_DT - 15)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_d[5] = -(quintic_coeff_d[2]*P2P_DT*P2P_DT + 3*quintic_coeff_d[1]*P2P_DT - 6)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT*P2P_DT);
}

void flatness_guidance_fsm_init(void)
{
    fsm_state = FSM_INIT;
}

void flatness_set_guided(bool value)
{
    flatness_guided = value;
}