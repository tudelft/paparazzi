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
#include <stdio.h>

// #include "modules/datalink/telemetry.h"

#define SAFE_SQRT(x) (sqrtf((x) > 0 ? (x) : 0.0f))

// ---- level circle ---- //
// static const float DOWN_OFFSET = -2.0f;
// static const float NORTH_OFFSET = 0.0f;
// static const float EAST_OFFSET = 0.0f;

// #define REF_TRAJ_FILENAME "circle_vs2_r2.csv"
// #define NB_CSV_ROWS 1998

// #define REF_TRAJ_FILENAME "circle_vs3_r2.csv"
// #define NB_CSV_ROWS 1698

// #define REF_TRAJ_FILENAME "circle_vs4_r2.csv"
// #define NB_CSV_ROWS 1398

// #define REF_TRAJ_FILENAME "circle_vs5_r2.csv"
// #define NB_CSV_ROWS 1198

// ---- snap loop ---- //
// static const float DOWN_OFFSET = -1.0f;
// static const float NORTH_OFFSET = -5.0f + 1.41f;
// static const float EAST_OFFSET = -5.0f + 1.41f;

// #define REF_TRAJ_FILENAME "snap_loop_vs2_r2.csv"
// #define NB_CSV_ROWS 1348

// ---- immelmann ---- //
static const float DOWN_OFFSET = -1.0f;
static const float NORTH_OFFSET = 0.0f;
static const float EAST_OFFSET = -5.0f + 2.0f;

// #define REF_TRAJ_FILENAME "snap_immelmann_vs2_r1.csv"
// #define NB_CSV_ROWS 865

#define REF_TRAJ_FILENAME "snap_immelmann_vs3_r1.csv"
#define NB_CSV_ROWS 577

// #define REF_TRAJ_FILENAME "snap_immelmann_vs3.5_r1.csv"
// #define NB_CSV_ROWS 495

#define NB_CSV_COLS 18

static struct FloatVect3 pos_end = {0.0f, 0.0f, -2.0f};
static const float P2P_DT = 3.0f;
static const float P2P_TO_TRAJ_DELAY = 5.0f;

typedef enum {
    FSM_INIT = 0,
    FSM_P2P,
    FSM_TRAJECTORY_INIT,
    FSM_TRAJECTORY,
    FSM_END
} fl_guid_fsm_t;

typedef struct {
    float px, py, pz;
    float vx, vy, vz;
    float ax, ay, az;
    float p, q, r;
    float psi;
} Traj_row_t;

// constants
static const float C_X = -0.172f;
static const float C_Z = -0.079f;
// static const float C_X = 0.0f;
// static const float C_Z = 0.0f;

// static const float MU_X_v = 4.5f  / 100000000.0f;
// static const float MU_Y_v = 10.4f / 100000000.0f;
// static const float MU_Z_v = 0.88f  / 100000000.0f;
// static const float C_T_v  = -0.25f  / 100000000.0f;

static const float MU_X_v = 4.05f  / 100000000.0f;
static const float MU_Y_v = 8.54f / 100000000.0f;
static const float MU_Z_v = 0.93f  / 100000000.0f;
static const float C_T_v  = -0.281f  / 100000000.0f;

static const float MIN_TAU = -0.981f;
static const float MAX_TAU = -2.0f*9.81f;
static const float ACCEL_BOUND = 1.6f*9.81f;
static const float RATES_BOUND = (float)M_PI/4.0f;

static const float ACT_CUTOFF_OMEGA = 19.0f;
static const float FILT_CUTOFF_FREQ = 5.0f;
static const Gain_t Kq = {2.5f, 2.5f, 2.5f};
static const Gain_t Komega = {10.0f, 10.0f, 10.0f};
static const Gain_t Kp = {1.0f, 1.0f, 1.0f};
static const Gain_t Kv = {3.5f, 3.5f, 3.5f};

static const float VEL_NORM_THRESHOLD = 1.0f;

// global vars declared as extern in header file
dbg_t dbg;
struct Fl_stabilization fl_stabilization;

// global vars
static abi_event rc_ev;
struct ThrustSetpoint rc_thr_sp;
static float timestamp, timestamp_prev, dt;
static Butterworth2LowPass act_filter[4];
static Butterworth2LowPass accel_filter[3];
static Butterworth2LowPass spec_force_filter[3];
static Butterworth2LowPass rates_num_der_filter[3];
static float ACT_DYN_ALPHA;
static Act_t act = {
    .cmd        = {0.0f, 0.0f, 0.0f, 0.0f},
    .state      = {0.0f},
    .state_filt = {0.0f}
};
static struct FloatQuat *quat, quat_sp;
static struct FloatRates rates_sp, rates_ref;
static struct FloatRates *rates;
static struct FloatRates ang_accel_sp = {0.0f, 0.0f, 0.0f};
static float ang_accel_filt[3] = {0.0f, 0.0f, 0.0f};
static int32_t temp_throttle;
static float temp_spec_thrust_sp;
struct FloatVect3 pos_ref = {0.0f, 0.0f, 0.0f};
struct FloatVect3 vel_ref = {0.0f, 0.0f, 0.0f};
struct FloatVect3 accel_ref = {0.0f, 0.0f, 0.0f};
float psi_ref;
static struct FloatVect3 vel_sp, accel_sp;
static float accel_vector[3], accel_filt[3];
float f_cmd[3];
fl_guid_fsm_t fsm_state = FSM_INIT;
float quintic_coeff_n[6], quintic_coeff_e[6], quintic_coeff_d[6];
float timestamp_p2p_start, timestamp_p2p, timestamp_traj, timestamp_traj_start;
bool flatness_guided;
static FILE *ref_traj_fd = NULL;
static Traj_row_t traj[NB_CSV_ROWS];
static struct FloatVect3 *pos_start;
static struct FloatVect3 *vel_start;
static struct FloatVect3 *accel_start;
float fi_vector[3] = {0.0f, 0.0f, 0.0f};
float fi_vector_filt[3] = {0.0f, 0.0f, 0.0f};
float sign_test;

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
void load_csv_traj(void);

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
    dbg.C_X = C_X;
    dbg.C_Z = C_Z;
    dbg.ACCEL_BOUND = ACCEL_BOUND;
    dbg.ACT_CUTOFF_OMEGA = ACT_CUTOFF_OMEGA;
    dbg.FILT_CUTOFF_FREQ = FILT_CUTOFF_FREQ;

    //data
    dbg.timestamp = timestamp;
    dbg.guided = flatness_guided;
    dbg.dt = dt;
    dbg.Ts = timestamp - timestamp_prev;
    dbg.voltage = electrical.vsupply;
    dbg.throttle = temp_throttle;
    dbg.spec_thrust_sp = temp_spec_thrust_sp;

    dbg.pos_ref = pos_ref;
    dbg.vel_ref = vel_ref;
    dbg.accel_ref = accel_ref;
    dbg.psi_ref = psi_ref;
    dbg.rates_ref = rates_ref;

    dbg.vel_sp = vel_sp;
    dbg.accel_sp = accel_sp;
    dbg.f_cmd.x = f_cmd[0]; dbg.f_cmd.y = f_cmd[1]; dbg.f_cmd.z = f_cmd[2];
    dbg.accel_filt.x = accel_filt[0]; dbg.accel_filt.y = accel_filt[1]; dbg.accel_filt.z = accel_filt[2];
    dbg.sign_test = sign_test;

    dbg.quat_sp = &quat_sp;
    dbg.quat = quat;
    dbg.rates_sp = &rates_sp;
    dbg.rates = rates;
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
    float tau = 1.0f / (2.0f * (float)M_PI * FILT_CUTOFF_FREQ);
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

    char filename[256];
    sprintf(filename, "%s/%s", STRINGIFY(REF_TRAJ_FILE_PATH), REF_TRAJ_FILENAME);
    ref_traj_fd = fopen(filename, "r");
    if(!ref_traj_fd) {
        return; // todo: add error handling here?
    }
    
    load_csv_traj();
}

void flatness_stabilization_run(bool UNUSED in_flight, struct StabilizationSetpoint *att_sp, float spec_thrust_sp, int32_t *cmd)
{
    if (flatness_guided == false) {
        timestamp_prev = timestamp;
        timestamp = get_sys_time_float();
    }

    // calculate quaternion error
    struct FloatQuat quat_err;
    quat_sp = stab_sp_to_quat_f(att_sp);
    quat = stateGetNedToBodyQuat_f();
    float_quat_inv_comp_norm_shortest(&quat_err, quat, &quat_sp);

    // calculate rates setpoint
    rates_sp.p = 2.0f * Kq.x * quat_err.qx;
    rates_sp.q = 2.0f * Kq.y * quat_err.qy;
    rates_sp.r = 2.0f * Kq.z * quat_err.qz;

    // add FF rates
    if (flatness_guided == false) {
        rates_ref.p = 0.0f;
        rates_ref.q = 0.0f;
        rates_ref.r = 0.0f;
    }
    RATES_ADD(rates_sp, rates_ref);
    if (fsm_state == FSM_P2P) {
        RATES_BOUND_CUBE(rates_sp, -RATES_BOUND, RATES_BOUND);
    }
    
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

    dt = get_sys_time_float() - timestamp;

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

    // VECT3_STRIM(accel_sp, -ACCEL_BOUND, ACCEL_BOUND);

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
    if (in_flight == true) {
        forw_transl_flatness(vel_norm, &vel_b, act.state, &fb);
        float_rmat_transp_vmult(&fi, R_i2b, &fb);
    } else {
        fi.x = 0;
        fi.y = 0;
        fi.z = 0;
    }
    fi_vector[0] = fi.x; 
    fi_vector[1] = fi.y; 
    fi_vector[2] = fi.z;
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

    // inverse rotational flatness
    struct FloatQuat _quat_sp;
    float spec_thrust_sp;
    if (vel_norm > VEL_NORM_THRESHOLD && fsm_state != FSM_P2P) {
        // coordinated
        struct FloatVect3 ey, ey_hat, ex, ex_hat, ez, ez_hat, tmp;
        struct FloatVect3 f_cmd_vect3 = {f_cmd[0], f_cmd[1], f_cmd[2]};
        VECT3_CROSS_PRODUCT(ey, *vel_i, f_cmd_vect3)
        VECT3_SDIV(ey_hat, ey, sqrtf(VECT3_NORM2(ey)))

        struct FloatVect3 ey_hat_cur = {R_i2b->m[3], R_i2b->m[4], R_i2b->m[5]};
        sign_test = VECT3_DOT_PRODUCT(ey_hat_cur, ey_hat);
        if (sign_test >= 0) {
            ;
        } else {
            // VECT3_SMUL(ey_hat, ey_hat, -1.0f);
            ey_hat.x = -ey_hat.x;
            ey_hat.y = -ey_hat.y;
            ey_hat.z = -ey_hat.z;
        }
        

        // todo guard against r // ey

        struct FloatVect3 arb_vect = {1.0f, 1.0f, 1.0f};
        VECT3_SMUL(tmp, ey_hat, (VECT3_DOT_PRODUCT(arb_vect, ey_hat) / VECT3_DOT_PRODUCT(ey_hat, ey_hat)))
        VECT3_SUB(arb_vect, tmp)
        VECT3_COPY(ex, arb_vect)
        VECT3_SDIV(ex_hat, ex, sqrtf(VECT3_NORM2(ex)))

        VECT3_CROSS_PRODUCT(ez, ex_hat, ey_hat)
        VECT3_SDIV(ez_hat, ez, sqrtf(VECT3_NORM2(ez))) // renormalize just to be sure

        struct FloatRMat R_i2e;
        R_i2e.m[0] = ex_hat.x; R_i2e.m[1] = ex_hat.y; R_i2e.m[2] = ex_hat.z;
        R_i2e.m[3] = ey_hat.x; R_i2e.m[4] = ey_hat.y; R_i2e.m[5] = ey_hat.z;
        R_i2e.m[6] = ez_hat.x; R_i2e.m[7] = ez_hat.y; R_i2e.m[8] = ez_hat.z;

        struct FloatVect3 ve, fe;
        struct FloatVect3 fi_cmd = {f_cmd[0], f_cmd[1], f_cmd[2]};
        float_rmat_vmult(&ve, &R_i2e, vel_i);
        float_rmat_vmult(&fe, &R_i2e, &fi_cmd);

        float sigma_x = fe.x - C_X*vel_norm*ve.x;
        float sigma_z = fe.z - C_X*vel_norm*ve.z;
        float theta_e = atan2f(-sigma_x, -sigma_z);

        spec_thrust_sp = sinf(theta_e)*fe.x + 
                         cosf(theta_e)*fe.z - 
                         C_Z*vel_norm*(sinf(theta_e)*ve.x + cosf(theta_e)*ve.z);

        struct FloatRMat R_e2b, R_i2b_sp;
        R_e2b.m[0] = cosf(theta_e);    R_e2b.m[1] = 0.0f;    R_e2b.m[2] = -sinf(theta_e);
        R_e2b.m[3] = 0.0f;             R_e2b.m[4] = 1.0f;    R_e2b.m[5] = 0.0f;
        R_e2b.m[6] = sinf(theta_e);    R_e2b.m[7] = 0.0f;    R_e2b.m[8] = cosf(theta_e);

        float_rmat_comp(&R_i2b_sp, &R_i2e, &R_e2b);
        float_quat_of_rmat(&_quat_sp, &R_i2b_sp);
    } else {
        // uncoordinated
        struct FloatEulers eulers_sp = {0.0f, 0.0f, 0.0f};
        eulers_sp.psi = psi_ref;
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

        spec_thrust_sp = sinf(theta_e)*fe.x + 
                            cosf(theta_e)*fe.z - 
                            C_Z*vel_norm*(sinf(theta_e)*ve.x + cosf(theta_e)*ve.z);

        eulers_sp.theta = theta_e;
        float_quat_of_eulers_zxy(&_quat_sp, &eulers_sp);
    }

    struct StabilizationSetpoint _att_sp = stab_sp_from_quat_f(&_quat_sp);

    flatness_stabilization_run(in_flight, &_att_sp, spec_thrust_sp, cmd);

    // printf("%.2f\t%.2f\t%.2f\t%.2f\t", accel_sp.z , accel_filt[2], fi_filt.z, f_cmd[2]);
    // printf("%.0f\t%.0f\t%.0f\t%.0f\t", act.cmd[0], act.cmd[1], act.cmd[2], act.cmd[3]);
    // printf("psi=%f", eulers_sp.psi);
    // printf("\n");
}

void load_csv_traj(void)
{
    char line[512];

    rewind(ref_traj_fd);

    int csv_traj_i = 0;

    while (fgets(line, sizeof(line), ref_traj_fd) != NULL && csv_traj_i < NB_CSV_ROWS) {
        float t;
        float px, py, pz;
        float vx, vy, vz;
        float ax, ay, az;
        float jx, jy, jz;
        float psi, psidot;
        float p, q, r;

        int n = sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                       &t, &px, &py, &pz, &vx, &vy, &vz, &ax, &ay, &az, &jx, &jy, &jz, &psi, &psidot, &p, &q, &r);

        if (n == NB_CSV_COLS) {
            traj[csv_traj_i].px = px;
            traj[csv_traj_i].py = py;
            traj[csv_traj_i].pz = pz;

            traj[csv_traj_i].vx = vx;
            traj[csv_traj_i].vy = vy;
            traj[csv_traj_i].vz = vz;

            traj[csv_traj_i].ax = ax;
            traj[csv_traj_i].ay = ay;
            traj[csv_traj_i].az = az;

            traj[csv_traj_i].p = p;
            traj[csv_traj_i].q = q;
            traj[csv_traj_i].r = r;

            traj[csv_traj_i].psi = psi;

            csv_traj_i++;
        }
    }
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
    float v_squared = electrical.vsupply * electrical.vsupply;

    m[0] = MU_X_v * v_squared * (u[0]*u[0] - u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
    m[1] = MU_Y_v * v_squared * (u[0]*u[0] + u[1]*u[1] - u[2]*u[2] - u[3]*u[3]);
    m[2] = MU_Z_v * v_squared * (-u[0]*u[0] + u[1]*u[1] - u[2]*u[2] + u[3]*u[3]);
}

static void inv_rot_flatness(float tau, float *m, float *u)
{
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

    timestamp_prev = timestamp;
    timestamp = get_sys_time_float();

    switch (fsm_state)
    {
        case FSM_INIT:
            pos_start = (struct FloatVect3 *)stateGetPositionNed_f();
            vel_start = (struct FloatVect3 *)stateGetSpeedNed_f();
            accel_start = (struct FloatVect3 *)stateGetAccelNed_f();
            
            vel_start->x = 0.0f; vel_start->y = 0.0f; vel_start->z = 0.0f;
            accel_start->x = 0.0f; accel_start->y = 0.0f; accel_start->z = 0.0f;

            pos_end.x = traj[0].px + NORTH_OFFSET;
            pos_end.y = traj[0].py + EAST_OFFSET;
            pos_end.z = traj[0].pz + DOWN_OFFSET;

            compute_quintic_coefficients(pos_start, &pos_end, vel_start, accel_start);

            timestamp_p2p_start = get_sys_time_float();
            fsm_state = FSM_P2P;
             /* fall through */ 

        case FSM_P2P:
            timestamp_p2p = get_sys_time_float() - timestamp_p2p_start;
            if (timestamp_p2p < P2P_DT) {
                compute_quintic_ref(timestamp_p2p, pos_start, &pos_end);
            } else {
                compute_quintic_ref(P2P_DT, pos_start, &pos_end);
            }

            // todo bound yaw rate
            psi_ref = traj[0].psi;

            rates_ref.p = 0.0f;
            rates_ref.q = 0.0f;
            rates_ref.r = 0.0f;

            flatness_guidance_run(in_flight, cmd);

            if (timestamp_p2p > P2P_DT + P2P_TO_TRAJ_DELAY) {
                fsm_state = FSM_TRAJECTORY_INIT; //use for complete trajectory 
                // ; // use for only p2p
            }
            break;

        case FSM_TRAJECTORY_INIT:
            timestamp_traj_start = get_sys_time_float();
            fsm_state = FSM_TRAJECTORY;
            /* fall through */

        case FSM_TRAJECTORY:
            timestamp_traj = get_sys_time_float() - timestamp_traj_start;
            int csv_i = (int)roundf(timestamp_traj/0.01f);
            
            if (csv_i < NB_CSV_ROWS) {
                pos_ref.x = traj[csv_i].px + NORTH_OFFSET;
                pos_ref.y = traj[csv_i].py + EAST_OFFSET;
                pos_ref.z = traj[csv_i].pz + DOWN_OFFSET;

                vel_ref.x = traj[csv_i].vx;
                vel_ref.y = traj[csv_i].vy;
                vel_ref.z = traj[csv_i].vz;

                accel_ref.x = traj[csv_i].ax;
                accel_ref.y = traj[csv_i].ay;
                accel_ref.z = traj[csv_i].az;

                rates_ref.p = traj[csv_i].p;
                rates_ref.q = traj[csv_i].q;
                rates_ref.r = traj[csv_i].r;

                psi_ref = traj[csv_i].psi;
            } else {
                vel_ref.x = 0.0f;
                vel_ref.y = 0.0f;
                vel_ref.z = 0.0f;

                accel_ref.x = 0.0f;
                accel_ref.y = 0.0f;
                accel_ref.z = 0.0f;

                rates_ref.p = 0.0f;
                rates_ref.q = 0.0f;
                rates_ref.r = 0.0f;

                fsm_state = FSM_END;
            }

            flatness_guidance_run(in_flight, cmd);
            break;

        case FSM_END:
            flatness_guidance_run(in_flight, cmd); // wait in the last pos
            break;

        default:
            fsm_state = FSM_INIT;
            break;
    }
}

void compute_quintic_ref(float t, struct FloatVect3 *ps, const struct FloatVect3 *pe)
{
    float t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
    pos_ref.x = ps->x + (quintic_coeff_n[1]*t + quintic_coeff_n[2]*t2 + quintic_coeff_n[3]*t3 + quintic_coeff_n[4]*t4 + quintic_coeff_n[5]*t5)*(pe->x - ps->x);
    pos_ref.y = ps->y + (quintic_coeff_e[1]*t + quintic_coeff_e[2]*t2 + quintic_coeff_e[3]*t3 + quintic_coeff_e[4]*t4 + quintic_coeff_e[5]*t5)*(pe->y - ps->y);
    pos_ref.z = ps->z + (quintic_coeff_d[1]*t + quintic_coeff_d[2]*t2 + quintic_coeff_d[3]*t3 + quintic_coeff_d[4]*t4 + quintic_coeff_d[5]*t5)*(pe->z - ps->z);

    vel_ref.x = (quintic_coeff_n[1] + 2*quintic_coeff_n[2]*t + 3*quintic_coeff_n[3]*t2 + 4*quintic_coeff_n[4]*t3 + 5*quintic_coeff_n[5]*t4)*(pe->x - ps->x);
    vel_ref.y = (quintic_coeff_e[1] + 2*quintic_coeff_e[2]*t + 3*quintic_coeff_e[3]*t2 + 4*quintic_coeff_e[4]*t3 + 5*quintic_coeff_e[5]*t4)*(pe->y - ps->y);
    vel_ref.z = (quintic_coeff_d[1] + 2*quintic_coeff_d[2]*t + 3*quintic_coeff_d[3]*t2 + 4*quintic_coeff_d[4]*t3 + 5*quintic_coeff_d[5]*t4)*(pe->z - ps->z);

    accel_ref.x = (2*quintic_coeff_n[2] + 6*quintic_coeff_n[3]*t + 12*quintic_coeff_n[4]*t2 + 20*quintic_coeff_n[5]*t3)*(pe->x - ps->x);
    accel_ref.y = (2*quintic_coeff_e[2] + 6*quintic_coeff_e[3]*t + 12*quintic_coeff_e[4]*t2 + 20*quintic_coeff_e[5]*t3)*(pe->y - ps->y);
    accel_ref.z = (2*quintic_coeff_d[2] + 6*quintic_coeff_d[3]*t + 12*quintic_coeff_d[4]*t2 + 20*quintic_coeff_d[5]*t3)*(pe->z - ps->z);
}

void compute_quintic_coefficients(struct FloatVect3 *ps, const struct FloatVect3 *pe, struct FloatVect3 *vs, struct FloatVect3 *as)
{
    quintic_coeff_n[0] = 0;
    quintic_coeff_n[1] = vs->x/(pe->x - ps->x);
    quintic_coeff_n[2] = as->x/(2*(pe->x - ps->x));
    quintic_coeff_n[3] = -(3*quintic_coeff_n[2]*P2P_DT*P2P_DT + 6*quintic_coeff_n[1]*P2P_DT - 10)/(P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_n[4] = (3*quintic_coeff_n[2]*P2P_DT*P2P_DT + 8*quintic_coeff_n[1]*P2P_DT - 15)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_n[5] = -(quintic_coeff_n[2]*P2P_DT*P2P_DT + 3*quintic_coeff_n[1]*P2P_DT - 6)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT*P2P_DT);

    quintic_coeff_e[0] = 0;
    quintic_coeff_e[1] = vs->y/(pe->y - ps->y);
    quintic_coeff_e[2] = as->y/(2*(pe->y - ps->y));
    quintic_coeff_e[3] = -(3*quintic_coeff_e[2]*P2P_DT*P2P_DT + 6*quintic_coeff_e[1]*P2P_DT - 10)/(P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_e[4] = (3*quintic_coeff_e[2]*P2P_DT*P2P_DT + 8*quintic_coeff_e[1]*P2P_DT - 15)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT);
    quintic_coeff_e[5] = -(quintic_coeff_e[2]*P2P_DT*P2P_DT + 3*quintic_coeff_e[1]*P2P_DT - 6)/(P2P_DT*P2P_DT*P2P_DT*P2P_DT*P2P_DT);

    quintic_coeff_d[0] = 0;
    quintic_coeff_d[1] = vs->z/(pe->z - ps->z);
    quintic_coeff_d[2] = as->z/(2*(pe->z - ps->z));
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