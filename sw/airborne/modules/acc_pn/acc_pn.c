// pn.c

#include <stdio.h>
#include <math.h>
#include "state.h"
#include "autopilot.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"
#include "acc_nn/ppo_controller.h"
#include "acc_pn.h"
/** EKF */
#include "modules/ins/ins_ext_pose.h"

/*---------------------------------------------------------------------------*/
/*                            External Messaging                             */
/*---------------------------------------------------------------------------*/

struct pnmessage target_message = {
    .device = (&((DOWNLINK_DEVICE).device)),
    .enabled = true,
    .msg_available = false,
};

uint8_t pn_msg_buf[256] __attribute__((aligned)); ///< The InterMCU message buffer

void pn_parse_REMOTE_GPS_LOCAL(uint8_t *buf);

static struct FloatVect3 target_pos_enu = {1.0, 0.0, 2.0}; // ENU
static struct FloatVect3 target_vel_enu = {0, 0, 0};       // ENU

// NED for traditional pursuit laws
static struct FloatVect3 target_pos_ned = {0.0, 1.0, -2.0};
static struct FloatVect3 target_vel_ned = {0, 0, 0};

static bool first_remote_gps_msg_received = false;

/*---------------------------------------------------------------------------*/
/*                            Configuration                                  */
/*---------------------------------------------------------------------------*/
static const float DT = 1.0f / 100.0f;
static const float LAMBDA = 50.0f;
static const float PP_WEIGHT = 0.03f;
static const float MAX_ACCEL = 18.0f;
static const float EPSILON = 1e-3f;
static const float K2 = 5.1f;
static const float V_R = -5.0f; // closing speed bias (GRTPN only)

/*---------------------------------------------------------------------------*/
/*                            Filter State                                   */
/*---------------------------------------------------------------------------*/
static Butterworth2LowPass filter_acc_x;
static Butterworth2LowPass filter_acc_y;
static Butterworth2LowPass filter_acc_z;

static const float ACC_CUTOFF_FREQ = 8.0f; // Hz
static const float TAU_ACC = 1.0f / (2.0f * M_PI * ACC_CUTOFF_FREQ);

static Butterworth2LowPass filter_pu_vel_x, filter_pu_vel_y, filter_pu_vel_z;
static Butterworth2LowPass filter_ev_vel_x, filter_ev_vel_y, filter_ev_vel_z;
static Butterworth2LowPass filter_rdot_x, filter_rdot_y, filter_rdot_z;
static Butterworth2LowPass filt_abz;

static const float OBS_SAMPLE_TIME = 1.0f / 100.0f;
static const float OBS_CUTOFF_FREQ = 8.0f; // Hz
static const float TAU_OBS = 1.0f / (2.0f * M_PI * OBS_CUTOFF_FREQ);

/*---------------------------------------------------------------------------*/
/*                            State & Mode                                  */
/*---------------------------------------------------------------------------*/
static float time_s = 0.0f;
static pn_mode_t cur_mode = PN_MODE_NEURAL;
static struct LoggerData_PN pn_log;

/*---------------------------------------------------------------------------*/
/*                         Internal Helpers                                 */
/*---------------------------------------------------------------------------*/
#define V3_NORM(v) float_vect3_norm(&(v))

/** In‐place clamp to max_val */
static void saturate3(struct FloatVect3 *v, float max_val)
{
  float n = float_vect3_norm(v);
  if (n > max_val)
  {
    float scale = max_val / n;
    /* v = v * scale */
    float_vect_smul(&v->x, &v->x, scale, 3);
  }
}

/** Record last outputs */
static void pn_info(const struct FloatVect3 *pt,
                    const struct FloatVect3 *vt,
                    const struct FloatVect3 *ac)
{
  pn_log.pos_target = *pt;
  pn_log.vel_target = *vt;
  pn_log.accel_command = *ac;
}

void get_action(const float *obs, float *action_out);

static void normalize_and_magnitude(const struct FloatVect3 *v, float *unit_out, float *mag_out)
{
  float norm = float_vect3_norm((struct FloatVect3 *)v);
  *mag_out = norm;
  float safe_norm = norm > 1e-6f ? norm : 1e-6f;
  unit_out[0] = v->x / safe_norm;
  unit_out[1] = v->y / safe_norm;
  unit_out[2] = v->z / safe_norm;
}

// static void build_observation(float *obs)
// {
//   struct NedCoor_f *pu_pos = stateGetPositionNed_f();
//   struct NedCoor_f *pu_vel = stateGetSpeedNed_f();
//   struct FloatEulers *att = stateGetNedToBodyEulers_f();
//   struct FloatRates *rates = stateGetBodyRates_f();
//   float raw_abz = ekf_U[2] - ekf_X[11];

//   float filtered_abz = update_butterworth_2_low_pass(&filt_abz, raw_abz);

//   // Build rel_pos and rel_vel in NED
//   struct FloatVect3 rel_pos_ned = {
//       .x = target_pos_ned.x - pu_pos->x,
//       .y = target_pos_ned.y - pu_pos->y,
//       .z = target_pos_ned.z - pu_pos->z,
//   };

//   struct FloatVect3 rel_vel_ned = {
//       .x = target_vel_ned.x - pu_vel->x,
//       .y = target_vel_ned.y - pu_vel->y,
//       .z = target_vel_ned.z - pu_vel->z,
//   };

//   // Rotation: from NED to BODY
//   float cphi = cosf(att->phi), sphi = sinf(att->phi);
//   float ctheta = cosf(att->theta), stheta = sinf(att->theta);
//   float cpsi = cosf(att->psi), spsi = sinf(att->psi);

//   float R[3][3] = {
//       {ctheta * cpsi, ctheta * spsi, -stheta},
//       {sphi * stheta * cpsi - cphi * spsi, sphi * stheta * spsi + cphi * cpsi, sphi * ctheta},
//       {cphi * stheta * cpsi + sphi * spsi, cphi * stheta * spsi - sphi * cpsi, cphi * ctheta}};

//   // Rotate into body frame
//   struct FloatVect3 rel_pos_body = {
//       .x = R[0][0] * rel_pos_ned.x + R[0][1] * rel_pos_ned.y + R[0][2] * rel_pos_ned.z,
//       .y = R[1][0] * rel_pos_ned.x + R[1][1] * rel_pos_ned.y + R[1][2] * rel_pos_ned.z,
//       .z = R[2][0] * rel_pos_ned.x + R[2][1] * rel_pos_ned.y + R[2][2] * rel_pos_ned.z,
//   };

//   struct FloatVect3 rel_vel_body = {
//       .x = R[0][0] * rel_vel_ned.x + R[0][1] * rel_vel_ned.y + R[0][2] * rel_vel_ned.z,
//       .y = R[1][0] * rel_vel_ned.x + R[1][1] * rel_vel_ned.y + R[1][2] * rel_vel_ned.z,
//       .z = R[2][0] * rel_vel_ned.x + R[2][1] * rel_vel_ned.y + R[2][2] * rel_vel_ned.z,
//   };

//   int i = 0;

//   // Normalize and fill into obs
//   normalize_and_magnitude(&rel_pos_body, &obs[i], &obs[i + 3]);
//   i += 4;
//   normalize_and_magnitude(&rel_vel_body, &obs[i], &obs[i + 3]);
//   i += 4;

//   // Rotation matrix cols 1 and 2 (get_rot_columns equivalent)
//   // Col 1
//   obs[i++] = ctheta * cpsi;
//   obs[i++] = ctheta * spsi;
//   obs[i++] = -stheta;

//   // Col 2
//   obs[i++] = sphi * stheta * cpsi - cphi * spsi;
//   obs[i++] = sphi * stheta * spsi + cphi * cpsi;
//   obs[i++] = sphi * ctheta;

//   // Angular rates
//   obs[i++] = rates->p;
//   obs[i++] = rates->q; // Frame adjustment
//   obs[i++] = rates->r;

//   // T_force approximation: body thrust (z)
//   obs[i++] = -filtered_abz;
// }

static void build_observation(float *obs)
{
  struct NedCoor_f *pu_pos = stateGetPositionNed_f();
  struct NedCoor_f *pu_vel = stateGetSpeedNed_f();
  // struct FloatEulers *att = stateGetNedToBodyEulers_f();
  // struct FloatRates *rates = stateGetBodyRates_f();
  // float raw_abz = ekf_U[2] - ekf_X[11];

  // float filtered_abz = update_butterworth_2_low_pass(&filt_abz, raw_abz);

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

  int i = 0;

  // Normalize and fill into obs
  normalize_and_magnitude(&rel_pos_ned, &obs[i], &obs[i + 3]);
  i += 4;
  normalize_and_magnitude(&rel_vel_ned, &obs[i], &obs[i + 3]);
  i += 4;
}

// Converts NED to ENU
static inline struct FloatVect3 ned_to_enu(struct FloatVect3 ned)
{
  struct FloatVect3 enu = {
      .x = ned.y,
      .y = ned.x,
      .z = -ned.z};
  return enu;
}

// Converts ENU to NED
static inline struct FloatVect3 enu_to_ned(struct FloatVect3 enu)
{
  struct FloatVect3 ned = {
      .x = enu.y,
      .y = enu.x,
      .z = -enu.z};
  return ned;
}

/*---------------------------------------------------------------------------*/
/*                      Individual Pursuit Laws                              */
/*---------------------------------------------------------------------------*/
static void run_frpn(void)
{
  struct FloatVect3 r, r_dot, tmp, term1, part, acc_ned, acc_enu;
  struct NedCoor_f *pos_n = stateGetPositionNed_f();
  struct NedCoor_f *vel_n = stateGetSpeedNed_f();
  struct FloatVect3 pos_t = target_pos_ned;
  struct FloatVect3 vel_t = target_vel_ned;

  /* --- relative vectors r = pos_t - pos_n, r_dot = vel_t - vel_n --- */
  VECT3_ASSIGN(r,
               pos_t.x - pos_n->x,
               pos_t.y - pos_n->y,
               pos_t.z - pos_n->z);
  VECT3_ASSIGN(r_dot,
               vel_t.x - vel_n->x,
               vel_t.y - vel_n->y,
               vel_t.z - vel_n->z);

  float R = V3_NORM(r);
  float Rdot = V3_NORM(r_dot);
  float t_go = R / (Rdot + EPSILON);

  /* FRPN term1 = (r + r_dot * t_go) / t_go^2 */
  float_vect_smul(&tmp.x, &r_dot.x, t_go, 3); // tmp = r_dot * t_go
  float_vect_add(&tmp.x, &r.x, 3);            // tmp += r
  float_vect_smul(&term1.x, &tmp.x, 1.0f / (t_go * t_go), 3);

  /* blend & scale */
  float_vect_smul(&part.x, &r.x, PP_WEIGHT, 3);
  float_vect_smul(&tmp.x, &term1.x, 1 - PP_WEIGHT, 3);
  float_vect_sum(&acc_ned.x, &tmp.x, &part.x, 3);
  float_vect_smul(&acc_ned.x, &acc_ned.x, LAMBDA, 3);

  saturate3(&acc_ned, MAX_ACCEL);

  /* smooth accel via low-pass filter */
  acc_ned.x = update_butterworth_2_low_pass(&filter_acc_x, acc_ned.x);
  acc_ned.y = update_butterworth_2_low_pass(&filter_acc_y, acc_ned.y);
  acc_ned.z = update_butterworth_2_low_pass(&filter_acc_z, acc_ned.z);

  acc_enu = ned_to_enu(acc_ned);

  pn_log.filt_accel_command = acc_ned;

  AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, 1, &acc_ned);
  pn_info(&target_pos_ned, &target_vel_ned, &acc_ned);
}

static void run_grtpn(void)
{
  struct FloatVect3 r, r_dot, Ir, cross, phi_dot, glob, acc_ned, acc_enu;
  struct NedCoor_f *pos_n = stateGetPositionNed_f();
  struct NedCoor_f *vel_n = stateGetSpeedNed_f();

  struct FloatVect3 pos_t = target_pos_ned;
  struct FloatVect3 vel_t = target_vel_ned;

  VECT3_ASSIGN(r,
               pos_t.x - pos_n->x,
               pos_t.y - pos_n->y,
               pos_t.z - pos_n->z);
  VECT3_ASSIGN(r_dot,
               vel_t.x - vel_n->x,
               vel_t.y - vel_n->y,
               vel_t.z - vel_n->z);

  float R = V3_NORM(r);
  float Vc = V3_NORM(r_dot);

  /* Ir = unit(r) */
  if (R > EPSILON)
  {
    Ir.x = r.x / R;
    Ir.y = r.y / R;
    Ir.z = r.z / R;
  }
  else
  {
    Ir.x = Ir.y = Ir.z = 0.0f;
  }

  /* φ̇ = (Ir × r_dot) / R² */
  cross.x = Ir.y * r_dot.z - Ir.z * r_dot.y;
  cross.y = Ir.z * r_dot.x - Ir.x * r_dot.z;
  cross.z = Ir.x * r_dot.y - Ir.y * r_dot.x;
  float_vect_smul(&phi_dot.x, &cross.x, 1.0f / (R * R), 3);

  /* α = k2*(Vc - v_r) + r_dot·φ̇ */
  float dot = float_vect_dot_product(&r_dot.x, &phi_dot.x, 3);
  float alpha = K2 * (Vc - V_R) + dot;

  /* global term = Ir * α */
  float_vect_smul(&glob.x, &Ir.x, alpha, 3);

  /* local = (φ̇ × Ir) * (λ * Vc) */
  cross.x = phi_dot.y * Ir.z - phi_dot.z * Ir.y;
  cross.y = phi_dot.z * Ir.x - phi_dot.x * Ir.z;
  cross.z = phi_dot.x * Ir.y - phi_dot.y * Ir.x;
  float_vect_smul(&cross.x, &cross.x, LAMBDA * Vc, 3);

  /* combine & send */
  float_vect_sum(&acc_ned.x, &glob.x, &cross.x, 3);

  saturate3(&acc_ned, MAX_ACCEL);

  /* smooth accel via low-pass filter */
  acc_ned.x = update_butterworth_2_low_pass(&filter_acc_x, acc_ned.x);
  acc_ned.y = update_butterworth_2_low_pass(&filter_acc_y, acc_ned.y);
  acc_ned.z = update_butterworth_2_low_pass(&filter_acc_z, acc_ned.z);

  acc_enu = ned_to_enu(acc_ned);

  pn_log.filt_accel_command = acc_enu;

  AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, 1, &acc_ned);
  pn_info(&target_pos_enu, &target_vel_enu, &acc_enu);
}

static void run_nn_policy(void)
{
  float obs[8];
  float accel_out[3];

  build_observation(obs);
  get_action(obs, accel_out); // Returns NED action

  struct FloatVect3 acc_ned = {
      .x = accel_out[0] * MAX_ACCEL,
      .y = accel_out[1] * MAX_ACCEL,
      .z = accel_out[2] * MAX_ACCEL};

  // acc_ned.x = update_butterworth_2_low_pass(&filter_acc_x, acc_ned.x);
  // acc_ned.y = update_butterworth_2_low_pass(&filter_acc_y, acc_ned.y);
  // acc_ned.z = update_butterworth_2_low_pass(&filter_acc_z, acc_ned.z);

  pn_log.filt_accel_command = acc_ned;

  // saturate3(&acc_ned, MAX_ACCEL);

  AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, 1, &acc_ned);
  pn_info(&target_pos_ned, &target_vel_ned, &acc_ned);
}

/*---------------------------------------------------------------------------*/
/*                            Public API                                     */
/*---------------------------------------------------------------------------*/
void pn_init(void)
{
  printf("[pn] init\n");
  pprz_transport_init(&target_message.transport);

  init_butterworth_2_low_pass(&filter_acc_x, TAU_ACC, DT, 0.0f);
  init_butterworth_2_low_pass(&filter_acc_y, TAU_ACC, DT, 0.0f);
  init_butterworth_2_low_pass(&filter_acc_z, TAU_ACC, DT, 0.0f);
  init_butterworth_2_low_pass(&filt_abz, TAU_ACC, DT, 0.0f);

  init_butterworth_2_low_pass(&filter_pu_vel_x, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);
  init_butterworth_2_low_pass(&filter_pu_vel_y, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);
  init_butterworth_2_low_pass(&filter_pu_vel_z, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);

  init_butterworth_2_low_pass(&filter_ev_vel_x, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);
  init_butterworth_2_low_pass(&filter_ev_vel_y, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);
  init_butterworth_2_low_pass(&filter_ev_vel_z, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);

  init_butterworth_2_low_pass(&filter_rdot_x, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);
  init_butterworth_2_low_pass(&filter_rdot_y, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);
  init_butterworth_2_low_pass(&filter_rdot_z, TAU_OBS, OBS_SAMPLE_TIME, 0.0f);
}

void pn_start(void)
{
  time_s = 0.0f;
  printf("[pn] start\n");
}

void pn_stop(void)
{
  printf("[pn] stop\n");
}

void pn_set_mode(pn_mode_t m)
{
  cur_mode = m;
}

void pn_event(void)
{
  /* Parse incoming bytes */
  if (target_message.enabled)
  {
    pprz_check_and_parse(target_message.device, &target_message.transport, pn_msg_buf, &target_message.msg_available);

    if (target_message.msg_available)
    {
      // uint8_t class_id = pprzlink_get_msg_class_id(pn_msg_buf);

      target_message.time_since_last_frame = 0;
      dl_parse_msg(target_message.device, &target_message.transport.trans_tx, pn_msg_buf);
    }
    target_message.msg_available = false;
  }
}

void pn_run(void)
{
  time_s += DT;
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED)
  {
    return;
  }

  switch (cur_mode)
  {
  case PN_MODE_FRPN:
    run_frpn();
    break;
  case PN_MODE_GRTPN:
    run_grtpn();
    break;
  case PN_MODE_NEURAL:
    run_nn_policy();
    break;
  }
}

void pn_parse_TARGET_INFO(uint8_t *buf)
{
  if (!first_remote_gps_msg_received)
  {
    float t_now = get_sys_time_float();
    printf("[pn] First TARGET_INFO message received at t = %.3f seconds\n", t_now);
    first_remote_gps_msg_received = true;
  }

  target_pos_ned.x = DL_TARGET_INFO_enu_x(buf);
  target_pos_ned.y = DL_TARGET_INFO_enu_y(buf);
  target_pos_ned.z = DL_TARGET_INFO_enu_z(buf);

  target_vel_ned.x = DL_TARGET_INFO_enu_xd(buf);
  target_vel_ned.y = DL_TARGET_INFO_enu_yd(buf);
  target_vel_ned.z = DL_TARGET_INFO_enu_zd(buf);

  // Also convert to ENU
  target_pos_enu = ned_to_enu(target_pos_ned);
  target_vel_enu = ned_to_enu(target_vel_ned);
}

struct LoggerData_PN *pn_info_logger(void)
{
  return &pn_log;
}
