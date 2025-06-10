#include <stdio.h>
#include "state.h"
#include "autopilot.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"
#include "acc_pn/acc_pn.h"
#include "ctbr_nn/ppo_controller.h"
/** EKF */
#include "modules/ins/ins_ext_pose.h"

#include "ctbr_pn.h"

#define CTBR_OBS_DIM 31
#define CTBR_DT (1.0f / 512.0f)
#define CTBR_CUTOFF_FREQ 8.0f
#define CTBR_TAU (1.0f / (2.0f * M_PI * CTBR_CUTOFF_FREQ))

static bool ctbr_active = false;

/*---------------------------------------------------------------------------*/
/* Target tracking                                                           */
/*---------------------------------------------------------------------------*/

struct pnmessage target_message = {
  .device = (&((DOWNLINK_DEVICE).device)),
  .enabled = true,
  .msg_available = false,
};

uint8_t pn_msg_buf[256] __attribute__((aligned));

static struct FloatVect3 target_pos_enu = {0, 0.0, 2.0};
static struct FloatVect3 target_vel_enu = {0, 0, 0};
static bool first_target_received = false;

static struct LoggerData_PN ctbr_log;

/*---------------------------------------------------------------------------*/
/* Input filtering (inputs only)                                             */
/*---------------------------------------------------------------------------*/

static Butterworth2LowPass filt_self_vx, filt_self_vy, filt_self_vz;
static Butterworth2LowPass filt_target_vx, filt_target_vy, filt_target_vz;
static Butterworth2LowPass filt_abz;

void get_ctbr_action(const float *obs, float *ctbr_out); // extern NN inference

/*---------------------------------------------------------------------------*/
/* Observation builder                                                       */
/*---------------------------------------------------------------------------*/

static void normalize_and_magnitude(struct FloatVect3 *vec, float *dir_out, float *mag_out) {
  float mag = sqrtf(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
  if (mag > 1e-5f) {
    dir_out[0] = vec->x / mag;
    dir_out[1] = vec->y / mag;
    dir_out[2] = vec->z / mag;
  } else {
    dir_out[0] = dir_out[1] = dir_out[2] = 0.0f;
  }
  *mag_out = mag;
}

static void build_ctbr_obs(float *obs) {
  struct EnuCoor_f *pu_pos = stateGetPositionEnu_f();
  struct EnuCoor_f *pu_vel = stateGetSpeedEnu_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();
  float raw_abz = ekf_U[2] - ekf_X[11];

  // Filtered velocities
  float vx = update_butterworth_2_low_pass(&filt_self_vx, pu_vel->x);
  float vy = update_butterworth_2_low_pass(&filt_self_vy, pu_vel->y);
  float vz = update_butterworth_2_low_pass(&filt_self_vz, pu_vel->z);

  float tx = update_butterworth_2_low_pass(&filt_target_vx, target_vel_enu.x);
  float ty = update_butterworth_2_low_pass(&filt_target_vy, target_vel_enu.y);
  float tz = update_butterworth_2_low_pass(&filt_target_vz, target_vel_enu.z);

  float filtered_abz = update_butterworth_2_low_pass(&filt_abz, raw_abz);

  struct FloatVect3 pu_pos_v = {pu_pos->x, pu_pos->y, pu_pos->z};
  struct FloatVect3 ev_pos_v = target_pos_enu;

  struct FloatVect3 pu_vel_v = {vx, vy, vz};
  struct FloatVect3 ev_vel_v = {tx, ty, tz};

  struct FloatVect3 rel_pos = {
    .x = ev_pos_v.x - pu_pos_v.x,
    .y = ev_pos_v.y - pu_pos_v.y,
    .z = ev_pos_v.z - pu_pos_v.z
  };

  struct FloatVect3 rel_vel = {
    .x = ev_vel_v.x - pu_vel_v.x,
    .y = ev_vel_v.y - pu_vel_v.y,
    .z = ev_vel_v.z - pu_vel_v.z
  };

  // Index for filling obs array
  int i = 0;

  // Normalize and store pu_pos_v
  normalize_and_magnitude(&pu_pos_v, &obs[i], &obs[i+3]); i += 4;
  normalize_and_magnitude(&ev_pos_v, &obs[i], &obs[i+3]); i += 4;
  normalize_and_magnitude(&pu_vel_v, &obs[i], &obs[i+3]); i += 4;
  normalize_and_magnitude(&ev_vel_v, &obs[i], &obs[i+3]); i += 4;
  normalize_and_magnitude(&rel_pos,  &obs[i], &obs[i+3]); i += 4;
  normalize_and_magnitude(&rel_vel,  &obs[i], &obs[i+3]); i += 4;

  // Attitude (phi, theta, psi)
  obs[i++] = att->phi;
  obs[i++] = att->theta;
  obs[i++] = att->psi;

  // Angular rates
  obs[i++] = rates->p;
  obs[i++] = rates->q;
  obs[i++] = rates->r;

  // Filtered acceleration in body frame normalize with the normalized (mass-less) thrust comands 
  float T_min = 0.0;
  float T_max = 16.0;
  obs[i++] = (filtered_abz - T_min) / (T_max - T_min) * 2 - 1;

  // Sanity check
  if (i != CTBR_OBS_DIM) {
    printf("[build_ctbr_obs] WARNING: expected CTBR_OBS_DIM=%d but filled %d\n", CTBR_OBS_DIM, i);
  }

  // Logging
  ctbr_log.accel_command.x = 0.0f;
  ctbr_log.accel_command.y = 0.0f;
  ctbr_log.accel_command.z = 0.0f;

  ctbr_log.pos_target = target_pos_enu;
  ctbr_log.vel_target = target_vel_enu;

  ctbr_log.raw_pu_vel = pu_vel_v;
  ctbr_log.raw_ev_vel = ev_vel_v;

  ctbr_log.filt_pu_vel.x = vx;
  ctbr_log.filt_pu_vel.y = vy;
  ctbr_log.filt_pu_vel.z = vz;

  ctbr_log.filt_ev_vel.x = tx;
  ctbr_log.filt_ev_vel.y = ty;
  ctbr_log.filt_ev_vel.z = tz;

  ctbr_log.raw_r = rel_pos;
  ctbr_log.raw_r_dot = rel_vel;

  ctbr_log.filt_r_dot.x = tx - vx;
  ctbr_log.filt_r_dot.y = ty - vy;
  ctbr_log.filt_r_dot.z = tz - vz;
}


/*---------------------------------------------------------------------------*/
/* Main CTBR logic                                                           */
/*---------------------------------------------------------------------------*/
void ctbr_run(void) {
  if (!ctbr_active || guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    return;
  }

  float obs[CTBR_OBS_DIM];
  float action[4]; // NN outputs: [p, q, r, thrust], range [-1, 1]

  build_ctbr_obs(obs);
  get_ctbr_action(obs, action);

  // === SCALE FROM [-1, 1] TO PHYSICAL UNITS ===
  // Body rates [rad/s]
  const float p_min = -3.0f, p_max = 3.0f;
  const float q_min = -3.0f, q_max = 3.0f;
  const float r_min = -2.0f, r_max = 2.0f;

  // Thrust [m/s²] in body-z direction (used in ACCEL_SP)
  const float T_min = 0.0f, T_max = 16.0f;

  const float p_cmd = (action[0] + 1.0f) * 0.5f * (p_max - p_min) + p_min;
  const float q_cmd = (action[1] + 1.0f) * 0.5f * (q_max - q_min) + q_min;
  const float r_cmd = (action[2] + 1.0f) * 0.5f * (r_max - r_min) + r_min;
  const float T_cmd = (action[3] + 1.0f) * 0.5f * (T_max - T_min) + T_min;

  // 1. Send scaled thrust (z-body) as acceleration setpoint
  struct FloatVect3 acc_sp_body = {0., 0., -T_cmd};  // z-thrust
  AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, 1, &acc_sp_body);  // flag=1: use all 3 axes

  // 2. Store body rates and thrust for override (used in stabilization_indi_simple)
  control_nn[0] = p_cmd;
  control_nn[1] = q_cmd;
  control_nn[2] = r_cmd;
  control_nn[3] = T_cmd;

  // Optional: for debugging/logging
  // printf("[ctbr_run] raw=[%.2f %.2f %.2f %.2f] -> scaled=[%.2f %.2f %.2f %.2f]\n",
  //        action[0], action[1], action[2], action[3], p_cmd, q_cmd, r_cmd, T_cmd);
}


/*---------------------------------------------------------------------------*/
/* Target parser                                                             */
/*---------------------------------------------------------------------------*/

void pn_parse_TARGET_INFO(uint8_t *buf) {
  if (!first_target_received) {
    printf("[ctbr_pn] First TARGET_INFO received at t = %.2f s\n", get_sys_time_float());
    first_target_received = true;
  }

  target_pos_enu.x = DL_TARGET_INFO_enu_x(buf);
  target_pos_enu.y = DL_TARGET_INFO_enu_y(buf);
  target_pos_enu.z = DL_TARGET_INFO_enu_z(buf);

  target_vel_enu.x = DL_TARGET_INFO_enu_xd(buf);
  target_vel_enu.y = DL_TARGET_INFO_enu_yd(buf);
  target_vel_enu.z = DL_TARGET_INFO_enu_zd(buf);
}

/*---------------------------------------------------------------------------*/
/* Event parsing                                                             */
/*---------------------------------------------------------------------------*/

void pn_event(void) {
  if (target_message.enabled) {
    pprz_check_and_parse(target_message.device, &target_message.transport, pn_msg_buf, &target_message.msg_available);

    if (target_message.msg_available) {
      target_message.time_since_last_frame = 0;
      dl_parse_msg(target_message.device, &target_message.transport.trans_tx, pn_msg_buf);
    }

    target_message.msg_available = false;
  }
}

/*---------------------------------------------------------------------------*/
/* Public Interface                                                          */
/*---------------------------------------------------------------------------*/

void pn_init(void) {
  printf("[ctbr_pn] init\n");
  pprz_transport_init(&target_message.transport);

  float tau = CTBR_TAU;
  init_butterworth_2_low_pass(&filt_self_vx, tau, CTBR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_self_vy, tau, CTBR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_self_vz, tau, CTBR_DT, 0.0f);

  init_butterworth_2_low_pass(&filt_target_vx, tau, CTBR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_target_vy, tau, CTBR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_target_vz, tau, CTBR_DT, 0.0f);

  init_butterworth_2_low_pass(&filt_abz, CTBR_TAU, CTBR_DT, 0.0f);
}

void pn_start(void) {
  ctbr_active = true;
  printf("[ctbr_pn] start\n");
}

void pn_stop(void) {
  ctbr_active = false;
  printf("[ctbr_pn] stop\n");
}


void pn_run(void) {
  ctbr_run();
}

struct LoggerData_PN *pn_info_logger(void) {
  return &ctbr_log;
}

