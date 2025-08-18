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
#include "ctbr_nn/ppo_controller.h"
/** EKF */
#include "modules/ins/ins_ext_pose.h"

#include "ctbr_pn.h"

/*---------------------------------------------------------------------------*/
/* Constants                                                                 */
/*---------------------------------------------------------------------------*/
#define CTBR_OBS_DIM 18
#define CTBR_DT (1.0f / 100.0f) // 100 Hz
#define CTBR_CUTOFF_FREQ 8.0f   // Hz
#define CTBR_TAU (1.0f / (2.0f * M_PI * CTBR_CUTOFF_FREQ))

// Observation normalization safety
#define MAG_EPSILON 1e-5f

// Action scaling ranges
#define P_MIN -3.0f
#define P_MAX 3.0f
#define Q_MIN -3.0f
#define Q_MAX 3.0f
#define R_MIN -2.0f
#define R_MAX 2.0f
#define THRUST_MIN 1.41f
#define THRUST_MAX 20.4f

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

static struct FloatVect3 target_pos_ned = {1.0f, 0.0f, -2.5f};
static struct FloatVect3 target_vel_ned = {0.0f, 0.0f, 0.0f};

static bool first_target_received = false;

static struct LoggerData_PN ctbr_log;

/*---------------------------------------------------------------------------*/
/* Input filtering (inputs only)                                             */
/*---------------------------------------------------------------------------*/

static Butterworth2LowPass filt_self_vx, filt_self_vy, filt_self_vz;
static Butterworth2LowPass filt_abz;

void get_action(const float *obs, float *ctbr_out); // extern NN inference

/*---------------------------------------------------------------------------*/
/* Observation builder                                                       */
/*---------------------------------------------------------------------------*/

static void normalize_and_magnitude(struct FloatVect3 *vec, float *dir_out, float *mag_out)
{
  float mag = sqrtf(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
  if (mag > MAG_EPSILON)
  {
    dir_out[0] = vec->x / mag;
    dir_out[1] = vec->y / mag;
    dir_out[2] = vec->z / mag;
  }
  else
  {
    dir_out[0] = dir_out[1] = dir_out[2] = 0.0f;
  }
  *mag_out = mag;
}

static void build_ctbr_obs(float *obs)
{
  struct NedCoor_f *pu_pos = stateGetPositionNed_f();
  struct NedCoor_f *pu_vel = stateGetSpeedNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();
  float raw_abz = ekf_U[2] - ekf_X[11];

  // Filtered velocities
  float vx = update_butterworth_2_low_pass(&filt_self_vx, pu_vel->x);
  float vy = update_butterworth_2_low_pass(&filt_self_vy, pu_vel->y);
  float vz = update_butterworth_2_low_pass(&filt_self_vz, pu_vel->z);

  float filtered_abz = update_butterworth_2_low_pass(&filt_abz, raw_abz);

  // Build rel_pos and rel_vel in NED
  struct FloatVect3 rel_pos_ned = {
      .x = target_pos_ned.x - pu_pos->x,
      .y = target_pos_ned.y - pu_pos->y,
      .z = target_pos_ned.z - pu_pos->z,
  };

  struct FloatVect3 rel_vel_ned = {
      .x = target_vel_ned.x - pu_vel->x,
      .y = target_vel_ned.y - pu_vel->y,
      .z = target_vel_ned.z - pu_vel->z,
  };

  // Rotation: from NED to BODY
  float cphi = cosf(att->phi), sphi = sinf(att->phi);
  float ctheta = cosf(att->theta), stheta = sinf(att->theta);
  float cpsi = cosf(att->psi), spsi = sinf(att->psi);

  float R[3][3] = {
      {ctheta * cpsi, ctheta * spsi, -stheta},
      {sphi * stheta * cpsi - cphi * spsi, sphi * stheta * spsi + cphi * cpsi, sphi * ctheta},
      {cphi * stheta * cpsi + sphi * spsi, cphi * stheta * spsi - sphi * cpsi, cphi * ctheta}};

  // Rotate into body frame
  struct FloatVect3 rel_pos_body = {
      .x = R[0][0] * rel_pos_ned.x + R[0][1] * rel_pos_ned.y + R[0][2] * rel_pos_ned.z,
      .y = R[1][0] * rel_pos_ned.x + R[1][1] * rel_pos_ned.y + R[1][2] * rel_pos_ned.z,
      .z = R[2][0] * rel_pos_ned.x + R[2][1] * rel_pos_ned.y + R[2][2] * rel_pos_ned.z,
  };

  struct FloatVect3 rel_vel_body = {
      .x = R[0][0] * rel_vel_ned.x + R[0][1] * rel_vel_ned.y + R[0][2] * rel_vel_ned.z,
      .y = R[1][0] * rel_vel_ned.x + R[1][1] * rel_vel_ned.y + R[1][2] * rel_vel_ned.z,
      .z = R[2][0] * rel_vel_ned.x + R[2][1] * rel_vel_ned.y + R[2][2] * rel_vel_ned.z,
  };

  int i = 0;

  // Normalize and fill into obs
  normalize_and_magnitude(&rel_pos_body, &obs[i], &obs[i + 3]);
  i += 4;
  normalize_and_magnitude(&rel_vel_body, &obs[i], &obs[i + 3]);
  i += 4;

  // Rotation matrix cols 1 and 2 (get_rot_columns equivalent)
  // Col 1
  obs[i++] = ctheta * cpsi;
  obs[i++] = ctheta * spsi;
  obs[i++] = -stheta;

  // Col 2
  obs[i++] = sphi * stheta * cpsi - cphi * spsi;
  obs[i++] = sphi * stheta * spsi + cphi * cpsi;
  obs[i++] = sphi * ctheta;

  // Angular rates
  obs[i++] = rates->p;
  obs[i++] = rates->q; // Frame adjustment
  obs[i++] = rates->r;

  // T_force approximation: body thrust (z)
  obs[i++] = -filtered_abz;
  // Sanity check
  if (i != CTBR_OBS_DIM)
  {
    printf("[build_ctbr_obs] WARNING: expected CTBR_OBS_DIM=%d but filled %d\n", CTBR_OBS_DIM, i);
  }

  // Logging
  ctbr_log.accel_command.x = 0.0f;
  ctbr_log.accel_command.y = 0.0f;
  ctbr_log.accel_command.z = 0.0f;

  ctbr_log.pos_target = target_pos_ned;
  ctbr_log.vel_target = target_vel_ned;

  ctbr_log.filt_pu_vel.x = vx;
  ctbr_log.filt_pu_vel.y = vy;
  ctbr_log.filt_pu_vel.z = vz;
}
/*---------------------------------------------------------------------------*/
/* Main CTBR logic                                                           */
/*---------------------------------------------------------------------------*/
void ctbr_run(void)
{
  if (!ctbr_active || guidance_h.mode != GUIDANCE_H_MODE_GUIDED)
  {
    return;
  }

  float obs[CTBR_OBS_DIM];
  float action[4]; // NN outputs: [p, q, r, thrust], range [-1, 1]

  build_ctbr_obs(obs);
  get_ctbr_action(obs, action);

  // === SCALE FROM [-1, 1] TO PHYSICAL UNITS ===
  // Body rates [rad/s]
  const float p_min = P_MIN, p_max = P_MAX;
  const float q_min = Q_MIN, q_max = Q_MAX;
  const float r_min = R_MIN, r_max = R_MAX;

  // Thrust [m/s²] in body-z direction (used in ACCEL_SP)
  const float T_min = THRUST_MIN, T_max = THRUST_MAX;

  const float p_cmd = (action[0] + 1.0f) * 0.5f * (p_max - p_min) + p_min;
  const float q_cmd = (action[1] + 1.0f) * 0.5f * (q_max - q_min) + q_min;
  const float r_cmd = (action[2] + 1.0f) * 0.5f * (r_max - r_min) + r_min;
  const float T_cmd = (action[3] + 1.0f) * 0.5f * (T_max - T_min) + T_min;

  // 1. Send scaled thrust (z-body) as acceleration setpoint
  struct FloatVect3 acc_sp_body = {0., 0., -T_cmd};     // z-thrust
  AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, 1, &acc_sp_body); // flag=1: use all 3 axes

  // 2. Store body rates and thrust for override (used in stabilization_indi_simple)
  control_nn[0] = p_cmd;
  control_nn[1] = q_cmd; // AGAIN COORDINATE FRAME MISMATCH!!!!
  control_nn[2] = r_cmd;
  control_nn[3] = T_cmd;
}

/*---------------------------------------------------------------------------*/
/* Target parser                                                             */
/*---------------------------------------------------------------------------*/

void pn_parse_TARGET_INFO(uint8_t *buf)
{
  target_pos_ned.x = DL_TARGET_INFO_enu_x(buf);
  target_pos_ned.y = DL_TARGET_INFO_enu_y(buf);
  target_pos_ned.z = DL_TARGET_INFO_enu_z(buf);

  target_vel_ned.x = DL_TARGET_INFO_enu_xd(buf);
  target_vel_ned.y = DL_TARGET_INFO_enu_yd(buf);
  target_vel_ned.z = DL_TARGET_INFO_enu_zd(buf);
}

/*---------------------------------------------------------------------------*/
/* Event parsing                                                             */
/*---------------------------------------------------------------------------*/

void pn_event(void)
{
  if (target_message.enabled)
  {
    pprz_check_and_parse(target_message.device, &target_message.transport, pn_msg_buf, &target_message.msg_available);

    if (target_message.msg_available)
    {
      target_message.time_since_last_frame = 0;
      dl_parse_msg(target_message.device, &target_message.transport.trans_tx, pn_msg_buf);
    }

    target_message.msg_available = false;
  }
}

/*---------------------------------------------------------------------------*/
/* Public Interface                                                          */
/*---------------------------------------------------------------------------*/

void pn_init(void)
{
  printf("[ctbr_pn] init\n");
  pprz_transport_init(&target_message.transport);

  float tau = CTBR_TAU;
  init_butterworth_2_low_pass(&filt_self_vx, tau, CTBR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_self_vy, tau, CTBR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_self_vz, tau, CTBR_DT, 0.0f);

  init_butterworth_2_low_pass(&filt_abz, CTBR_TAU, CTBR_DT, 0.0f);
}

void pn_start(void)
{
  ctbr_active = true;
  printf("[ctbr_pn] start\n");
}

void pn_stop(void)
{
  ctbr_active = false;
  printf("[ctbr_pn] stop\n");
}

void pn_run(void)
{
  ctbr_run();
}

struct LoggerData_PN *pn_info_logger(void)
{
  return &ctbr_log;
}
