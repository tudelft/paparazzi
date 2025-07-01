#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "state.h"
#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"
#include "motor_nn/ppo_controller.h"
#include "modules/ins/ins_ext_pose.h"

#include "boards/bebop/actuators.h"
#include "modules/core/abi.h"

#include "motor_pn.h"

/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */
#define MOTOR_OBS_DIM 21
#define MOTOR_DT (1.0f / 100.0f) /* 100 Hz control loop    */
#define MOTOR_CUTOFF_FREQ 12.0f  /* Hz – Butterworth fc    */
#define MOTOR_TAU (1.0f / (2.0f * M_PI * MOTOR_CUTOFF_FREQ))

#define RPM_MIN 3000.0f /* for normalisation      */
#define RPM_MAX 12000.0f

static struct LoggerData_PN pn_log;

/* -------------------------------------------------------------------------- */
/* Globals                                                                    */
/* -------------------------------------------------------------------------- */
static bool motor_active = false;

/* Target tracking message */
struct pnmessage target_message = {
    .device = (&((DOWNLINK_DEVICE).device)),
    .enabled = true,
    .msg_available = false,
};
uint8_t pn_msg_buf[256] __attribute__((aligned));

static struct FloatVect3 target_pos_ned = {0.0f, 0.0f, -2.0f};
static struct FloatVect3 target_vel_ned = {0.0f, 0.0f, 0.0f};

/* Low-pass filters */
static Butterworth2LowPass
    filt_self_vx,
    filt_self_vy, filt_self_vz,
    filt_target_vx, filt_target_vy, filt_target_vz,
    filt_rpm1, filt_rpm2, filt_rpm3, filt_rpm4;

/* -------------------------------------------------------------------------- */
/* Helper functions                                                           */
/* -------------------------------------------------------------------------- */
static inline void normalize_and_magnitude(const struct FloatVect3 *v,
                                           float *dir_out, float *mag_out)
{
  const float mag = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
  if (mag > 1e-6f)
  {
    dir_out[0] = v->x / mag;
    dir_out[1] = v->y / mag;
    dir_out[2] = v->z / mag;
  }
  else
  {
    dir_out[0] = dir_out[1] = dir_out[2] = 0.0f;
  }
  *mag_out = mag;
}

static inline void rot_columns_w2b(const struct FloatEulers *att, float col[6])
{
  const float cr = cosf(att->phi);
  const float sr = sinf(att->phi);
  const float cp = cosf(att->theta);
  const float sp = sinf(att->theta);
  const float cy = cosf(att->psi);
  const float sy = sinf(att->psi);

  /* Column 1 */
  col[0] = cp * cy;
  col[1] = cp * sy;
  col[2] = -sp;
  /* Column 2 */
  col[3] = sr * sp * cy - cr * sy;
  col[4] = sr * sp * sy + cr * cy;
  col[5] = sr * cp;
}

static inline void world_to_body(const struct FloatEulers *att,
                                 const struct FloatVect3 *w,
                                 struct FloatVect3 *b)
{
  const float cr = cosf(att->phi);
  const float sr = sinf(att->phi);
  const float cp = cosf(att->theta);
  const float sp = sinf(att->theta);
  const float cy = cosf(att->psi);
  const float sy = sinf(att->psi);

  const float r11 = cp * cy;
  const float r12 = sr * sp * cy - cr * sy;
  const float r13 = sr * sy + cr * cy * sp;

  const float r21 = cp * sy;
  const float r22 = cr * cy + sr * sp * sy;
  const float r23 = cr * sp * sy - cy * sr;

  const float r31 = -sp;
  const float r32 = sr * cp;
  const float r33 = cr * cp;

  b->x = r11 * w->x + r12 * w->y + r13 * w->z;
  b->y = r21 * w->x + r22 * w->y + r23 * w->z;
  b->z = r31 * w->x + r32 * w->y + r33 * w->z;
}

static inline float norm_rpm(float rpm)
{
  float x = (rpm - RPM_MIN) / (RPM_MAX - RPM_MIN); /* 0-1 */
  x = x * 2.0f - 1.0f;                             /* -1..1 */
  if (x > 1.0f)
    x = 1.0f;
  if (x < -1.0f)
    x = -1.0f;
  return x;
}

/* -------------------------------------------------------------------------- */
/* Observation builder                                                        */
/* -------------------------------------------------------------------------- */
static void build_motor_obs(float *obs /* [MOTOR_OBS_DIM] */)
{
  struct NedCoor_f *pu_pos = stateGetPositionNed_f();
  struct NedCoor_f *pu_vel = stateGetSpeedNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  // Filtered velocities
  float vx = update_butterworth_2_low_pass(&filt_self_vx, pu_vel->x);
  float vy = update_butterworth_2_low_pass(&filt_self_vy, pu_vel->y);
  float vz = update_butterworth_2_low_pass(&filt_self_vz, pu_vel->z);

  pn_log.raw_pu_vel.x = pu_vel->x;
  pn_log.raw_pu_vel.y = pu_vel->y;
  pn_log.raw_pu_vel.z = pu_vel->z;

  pn_log.filt_pu_vel.x = vx;
  pn_log.filt_pu_vel.y = vy;
  pn_log.filt_pu_vel.z = vz;

  /* --- Filter RPMs ------------------------------------------------------ */
  const float w1 = update_butterworth_2_low_pass(&filt_rpm1, actuators_bebop.rpm_obs[0]);
  const float w2 = update_butterworth_2_low_pass(&filt_rpm2, actuators_bebop.rpm_obs[1]);
  const float w3 = update_butterworth_2_low_pass(&filt_rpm3, actuators_bebop.rpm_obs[2]);
  const float w4 = update_butterworth_2_low_pass(&filt_rpm4, actuators_bebop.rpm_obs[3]);

  struct FloatVect3 rel_pos = {
      .x = target_pos_ned.x - pu_pos->x,
      .y = target_pos_ned.y - pu_pos->y,
      .z = target_pos_ned.z - pu_pos->z,
  };

  struct FloatVect3 rel_vel = {
      .x = target_vel_ned.x - pu_vel->x,
      .y = target_vel_ned.y - pu_vel->y,
      .z = target_vel_ned.z - pu_vel->z,
  };

  pn_log.raw_r = rel_pos;
  pn_log.raw_r_dot = rel_vel;

  // Index for filling obs array
  int i = 0;

  normalize_and_magnitude(&rel_pos, &obs[i], &obs[i + 3]);
  i += 4;
  normalize_and_magnitude(&rel_vel, &obs[i], &obs[i + 3]);
  i += 4;

  // Rotation matrix cols 1 and 2 (get_rot_columns equivalent)
  float cphi = cosf(att->phi), sphi = sinf(att->phi);
  float ctheta = cosf(att->theta), stheta = sinf(att->theta);
  float cpsi = cosf(att->psi), spsi = sinf(att->psi);

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

  /* normalised RPMs */
  obs[i++] = norm_rpm(w1);
  obs[i++] = norm_rpm(w2);
  obs[i++] = norm_rpm(w3);
  obs[i++] = norm_rpm(w4);

  /* sanity check */
  if (i != MOTOR_OBS_DIM)
  {
    printf("[build_motor_obs] WARNING: filled %d / %d elements\n", i, MOTOR_OBS_DIM);
  }
} /* build_motor_obs */

/* -------------------------------------------------------------------------- */
/* Main MOTOR logic                                                            */
/* -------------------------------------------------------------------------- */
void motor_run(void)
{
  if (!motor_active || guidance_h.mode != GUIDANCE_H_MODE_GUIDED)
  {
    return;
  }

  float obs[MOTOR_OBS_DIM];
  float action[4];

  build_motor_obs(obs);
  get_action(obs, action);

  control_nn[0] = action[0];
  control_nn[1] = action[1];
  control_nn[2] = action[2];
  control_nn[3] = action[3];
}

/* -------------------------------------------------------------------------- */
/* Target message parser                                                      */
/* -------------------------------------------------------------------------- */
void pn_parse_TARGET_INFO(uint8_t *buf)
{
  target_pos_ned.x = DL_TARGET_INFO_enu_x(buf);
  target_pos_ned.y = DL_TARGET_INFO_enu_y(buf);
  target_pos_ned.z = DL_TARGET_INFO_enu_z(buf);

  target_vel_ned.x = DL_TARGET_INFO_enu_xd(buf);
  target_vel_ned.y = DL_TARGET_INFO_enu_yd(buf);
  target_vel_ned.z = DL_TARGET_INFO_enu_zd(buf);
}

/* -------------------------------------------------------------------------- */
/* Event processing                                                            */
/* -------------------------------------------------------------------------- */
void pn_event(void)
{
  if (target_message.enabled)
  {
    pprz_check_and_parse(target_message.device, &target_message.transport,
                         pn_msg_buf, &target_message.msg_available);

    if (target_message.msg_available)
    {
      target_message.time_since_last_frame = 0;
      dl_parse_msg(target_message.device, &target_message.transport.trans_tx, pn_msg_buf);
    }
    target_message.msg_available = false;
  }
}

/* -------------------------------------------------------------------------- */
/* Public interface                                                            */
/* -------------------------------------------------------------------------- */
void pn_init(void)
{
  printf("[motor_pn] init\n");
  pprz_transport_init(&target_message.transport);

  const float tau = MOTOR_TAU;
  init_butterworth_2_low_pass(&filt_self_vx, tau, MOTOR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_self_vy, tau, MOTOR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_self_vz, tau, MOTOR_DT, 0.0f);

  init_butterworth_2_low_pass(&filt_target_vx, tau, MOTOR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_target_vy, tau, MOTOR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_target_vz, tau, MOTOR_DT, 0.0f);

  init_butterworth_2_low_pass(&filt_rpm1, tau, MOTOR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_rpm2, tau, MOTOR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_rpm3, tau, MOTOR_DT, 0.0f);
  init_butterworth_2_low_pass(&filt_rpm4, tau, MOTOR_DT, 0.0f);
}

void pn_start(void)
{
  motor_active = true;
  printf("[motor_pn] start\n");
}

void pn_stop(void)
{
  motor_active = false;
  printf("[motor_pn] stop\n");
}

void pn_run(void)
{
  motor_run();
}

struct LoggerData_PN *pn_info_logger(void)
{
  return &pn_log;
}
