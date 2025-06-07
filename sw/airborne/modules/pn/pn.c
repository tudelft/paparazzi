// pn.c

#include "pn.h"
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
#include "nn_controller/ppo_controller_weights.h"
#include "pn/nn_controller/ppo_controller.h"


/*---------------------------------------------------------------------------*/
/*                            External Messaging                             */
/*---------------------------------------------------------------------------*/

struct pnmessage target_message = {
  .device = (&((DOWNLINK_DEVICE).device)),
  .enabled = true,
  .msg_available = false,
};

uint8_t pn_msg_buf[256] __attribute__((aligned));  ///< The InterMCU message buffer

void pn_parse_REMOTE_GPS_LOCAL(uint8_t *buf);

static struct FloatVect3 target_pos_enu = {0, 0.0, 2.0};  // ENU
static struct FloatVect3 target_vel_enu = {0, 0, 0};     // ENU

// NED for traditional pursuit laws
static struct FloatVect3 target_pos_ned = {0.0, 0.0, -2.0};  
static struct FloatVect3 target_vel_ned = {0, 0, 0};

static bool first_remote_gps_msg_received = false;


/*---------------------------------------------------------------------------*/
/*                            Configuration                                  */
/*---------------------------------------------------------------------------*/
static const float DT        = 1.0f/100.0f;
static const float LAMBDA    = 50.0f;
static const float PP_WEIGHT = 0.03f;
static const float MAX_ACCEL = 20.0f;
static const float EPSILON   = 1e-3f;
static const float K2        = 5.1f;
static const float V_R       = -5.0f;  // closing speed bias (GRTPN only)

/* acceleration filter time constant (seconds) */
static const float ACC_FILT_TAU     = 0.00001f;

/*---------------------------------------------------------------------------*/
/*                            Filter State                                   */
/*---------------------------------------------------------------------------*/
static struct FirstOrderLowPass acc_filt_x;
static struct FirstOrderLowPass acc_filt_y;
static struct FirstOrderLowPass acc_filt_z;

/*---------------------------------------------------------------------------*/
/*                            State & Mode                                  */
/*---------------------------------------------------------------------------*/
static float time_s = 0.0f;
static pn_mode_t cur_mode = PN_MODE_NEURAL;
static struct Proportional_nav pn_log;

/*---------------------------------------------------------------------------*/
/*                         Internal Helpers                                 */
/*---------------------------------------------------------------------------*/
#define V3_NORM(v) float_vect3_norm(&(v))

/** In‐place clamp to max_val */
static void saturate3(struct FloatVect3 *v, float max_val) {
  float n = float_vect3_norm(v);
  if (n > max_val) {
    float scale = max_val / n;
    /* v = v * scale */
    float_vect_smul(&v->x, &v->x, scale, 3);
  }
}

/** Record last outputs */
static void pn_info(const struct FloatVect3 *pt,
                    const struct FloatVect3 *vt,
                    const struct FloatVect3 *ac) {
  pn_log.pos_target    = *pt;
  pn_log.vel_target    = *vt;
  pn_log.accel_command = *ac;
}

void get_action(const float *obs, float *action_out);

static void build_observation(float *obs) {
  // Self state in ENU
  struct EnuCoor_f *pu_pos = stateGetPositionEnu_f();
  struct EnuCoor_f *pu_vel = stateGetSpeedEnu_f();
  // struct FloatEulers *self_att = stateGetNedToBodyEulers_f();  // Optional, not needed anymore
  // float thrust = stabilization_cmd[COMMAND_THRUST];            // Optional, not needed anymore

  struct FloatVect3 ev_pos = target_pos_enu;
  struct FloatVect3 ev_vel = target_vel_enu;

  // (1) pursuer position
  obs[0] = pu_pos->x;
  obs[1] = pu_pos->y;
  obs[2] = pu_pos->z;

  // (2) evader position
  obs[3] = ev_pos.x;
  obs[4] = ev_pos.y;
  obs[5] = ev_pos.z;

  // (3) pursuer velocity
  obs[6] = pu_vel->x;
  obs[7] = pu_vel->y;
  obs[8] = pu_vel->z;

  // (4) evader velocity
  obs[9] = ev_vel.x;
  obs[10] = ev_vel.y;
  obs[11] = ev_vel.z;

  // (5) Line-of-sight vector: r = ev_pos - pu_pos
  struct FloatVect3 r = {
    .x = ev_pos.x - pu_pos->x,
    .y = ev_pos.y - pu_pos->y,
    .z = ev_pos.z - pu_pos->z
  };
  obs[12] = r.x;
  obs[13] = r.y;
  obs[14] = r.z;

  // (6) LOS distance
  float R = float_vect3_norm(&r);
  obs[15] = R;

  // (7) Relative velocity: r_dot = ev_vel - pu_vel
  struct FloatVect3 r_dot = {
    .x = ev_vel.x - pu_vel->x,
    .y = ev_vel.y - pu_vel->y,
    .z = ev_vel.z - pu_vel->z
  };
  obs[16] = r_dot.x;
  obs[17] = r_dot.y;
  obs[18] = r_dot.z;

  // (8) LOS rate: Vc = ||r_dot||
  float Vc = float_vect3_norm(&r_dot);
  obs[19] = Vc;

  // (9) phi_dot = (Ir × r_dot) / ||r||²
  float safe_R = (R > 1e-6f) ? R : 1e-6f;
  struct FloatVect3 Ir = {
    .x = r.x / safe_R,
    .y = r.y / safe_R,
    .z = r.z / safe_R
  };

  struct FloatVect3 cross = {
    .x = Ir.y * r_dot.z - Ir.z * r_dot.y,
    .y = Ir.z * r_dot.x - Ir.x * r_dot.z,
    .z = Ir.x * r_dot.y - Ir.y * r_dot.x
  };

  float denom = safe_R * safe_R;
  obs[20] = cross.x / denom;
  obs[21] = cross.y / denom;
  obs[22] = cross.z / denom;
}


// Converts NED to ENU
static inline struct FloatVect3 ned_to_enu(struct FloatVect3 ned) {
  struct FloatVect3 enu = {
    .x = ned.y,
    .y = ned.x,
    .z = -ned.z
  };
  return enu;
}

// Converts ENU to NED
static inline struct FloatVect3 enu_to_ned(struct FloatVect3 enu) {
  struct FloatVect3 ned = {
    .x = enu.y,
    .y = enu.x,
    .z = -enu.z
  };
  return ned;
}



/*---------------------------------------------------------------------------*/
/*                      Individual Pursuit Laws                              */
/*---------------------------------------------------------------------------*/
static void run_frpn(void) {
    struct FloatVect3 r, r_dot, tmp, term1, part, acc;
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

    float R    = V3_NORM(r);
    float Rdot = V3_NORM(r_dot);
    float t_go = R / (Rdot + EPSILON);

    /* FRPN term1 = (r + r_dot * t_go) / t_go^2 */
    float_vect_smul(&tmp.x, &r_dot.x, t_go, 3);      // tmp = r_dot * t_go
    float_vect_add(&tmp.x, &r.x, 3);                // tmp += r
    float_vect_smul(&term1.x, &tmp.x, 1.0f/(t_go*t_go), 3);

    /* blend & scale */
    float_vect_smul(&part.x,    &r.x,       PP_WEIGHT,    3);
    float_vect_smul(&tmp.x,     &term1.x, 1-PP_WEIGHT,   3);
    float_vect_sum(&acc.x, &tmp.x, &part.x, 3);
    float_vect_smul(&acc.x,     &acc.x,     LAMBDA,       3);

    saturate3(&acc, MAX_ACCEL);

    /* smooth accel via low-pass filter */
    acc.x = update_first_order_low_pass(&acc_filt_x, acc.x);   
    acc.y = update_first_order_low_pass(&acc_filt_y, acc.y);   
    acc.z = update_first_order_low_pass(&acc_filt_z, acc.z);  

    AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, 1, &acc);
    pn_info(&pos_t, &vel_t, &acc);
}

static void run_grtpn(void) {
    struct FloatVect3 r, r_dot, Ir, cross, phi_dot, glob, acc;
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

    float R  = V3_NORM(r);
    float Vc = V3_NORM(r_dot);

    /* Ir = unit(r) */
    if (R > EPSILON) {
        Ir.x = r.x / R;  Ir.y = r.y / R;  Ir.z = r.z / R;
    } else {
        Ir.x = Ir.y = Ir.z = 0.0f;
    }

    /* φ̇ = (Ir × r_dot) / R² */
    cross.x = Ir.y * r_dot.z - Ir.z * r_dot.y;
    cross.y = Ir.z * r_dot.x - Ir.x * r_dot.z;
    cross.z = Ir.x * r_dot.y - Ir.y * r_dot.x;
    float_vect_smul(&phi_dot.x, &cross.x, 1.0f/(R*R), 3);

    /* α = k2*(Vc - v_r) + r_dot·φ̇ */
    float dot = float_vect_dot_product(&r_dot.x, &phi_dot.x, 3);
    float alpha = K2 * (Vc - V_R) + dot;

    /* global term = Ir * α */
    float_vect_smul(&glob.x, &Ir.x, alpha, 3);

    /* local = (φ̇ × Ir) * (λ * Vc) */
    cross.x = phi_dot.y*Ir.z - phi_dot.z*Ir.y;
    cross.y = phi_dot.z*Ir.x - phi_dot.x*Ir.z;
    cross.z = phi_dot.x*Ir.y - phi_dot.y*Ir.x;
    float_vect_smul(&cross.x, &cross.x, LAMBDA*Vc, 3);

    /* combine & send */
    float_vect_sum(&acc.x, &glob.x, &cross.x, 3);

    saturate3(&acc, MAX_ACCEL);

    /* smooth accel via low-pass filter */
    acc.x = update_first_order_low_pass(&acc_filt_x, acc.x);   
    acc.y = update_first_order_low_pass(&acc_filt_y, acc.y);   
    acc.z = update_first_order_low_pass(&acc_filt_z, acc.z);   

    AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, 1, &acc);
    pn_info(&pos_t, &vel_t, &acc);
}

static void run_nn_policy(void) {
  float obs[23];
  float accel_out[3];

  build_observation(obs);
  get_action(obs, accel_out);  // Returns ENU action

  struct FloatVect3 acc_enu = {
    .x = accel_out[0] * MAX_ACCEL,
    .y = accel_out[1] * MAX_ACCEL,
    .z = accel_out[2] * MAX_ACCEL
  };

  // Convert to NED before sending to ABI
  struct FloatVect3 acc_ned = enu_to_ned(acc_enu);

  // saturate3(&acc_ned, MAX_ACCEL);

  // Optional: Filtering here
  // acc_ned.x = update_first_order_low_pass(&acc_filt_x, acc_ned.x);
  // acc_ned.y = update_first_order_low_pass(&acc_filt_y, acc_ned.y);
  // acc_ned.z = update_first_order_low_pass(&acc_filt_z, acc_ned.z);

  AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, 1, &acc_ned);
  pn_info(&target_pos_ned, &target_vel_ned, &acc_ned);
}



/*---------------------------------------------------------------------------*/
/*                            Public API                                     */
/*---------------------------------------------------------------------------*/
void pn_init(void) {
    printf("[pn] init\n");

    pprz_transport_init(&target_message.transport);

    /* initialize filters with zero initial value */
    init_first_order_low_pass(&acc_filt_x, ACC_FILT_TAU, DT, 0.0f);
    init_first_order_low_pass(&acc_filt_y, ACC_FILT_TAU, DT, 0.0f);
    init_first_order_low_pass(&acc_filt_z, ACC_FILT_TAU, DT, 0.0f);
  }

void pn_start(void) {
  time_s = 0.0f;
  printf("[pn] start\n");
}

void pn_stop(void) {
  printf("[pn] stop\n");
}

void pn_set_mode(pn_mode_t m) {
  cur_mode = m;
}


void pn_event(void)
{
  /* Parse incoming bytes */
  if (target_message.enabled) {
    pprz_check_and_parse(target_message.device, &target_message.transport, pn_msg_buf, &target_message.msg_available);

    if (target_message.msg_available) {
      // uint8_t class_id = pprzlink_get_msg_class_id(pn_msg_buf);
      
      target_message.time_since_last_frame = 0;
      dl_parse_msg(target_message.device, &target_message.transport.trans_tx, pn_msg_buf);
    }
    target_message.msg_available = false;
  }
}

void pn_run(void) {
  time_s += DT;
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    return;
  }

  switch (cur_mode) {
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

void pn_parse_TARGET_INFO(uint8_t *buf) {
  if (!first_remote_gps_msg_received) {
    float t_now = get_sys_time_float();
    printf("[pn] First TARGET_INFO message received at t = %.3f seconds\n", t_now);
    first_remote_gps_msg_received = true;
  }

  // Store in ENU directly
  target_pos_enu.x = DL_TARGET_INFO_enu_x(buf);
  target_pos_enu.y = DL_TARGET_INFO_enu_y(buf);
  target_pos_enu.z = DL_TARGET_INFO_enu_z(buf);

  target_vel_enu.x = DL_TARGET_INFO_enu_xd(buf);
  target_vel_enu.y = DL_TARGET_INFO_enu_yd(buf);
  target_vel_enu.z = DL_TARGET_INFO_enu_zd(buf);

  // Also convert to NED for traditional controllers
  target_pos_ned = enu_to_ned(target_pos_enu);
  target_vel_ned = enu_to_ned(target_vel_enu);
}



struct Proportional_nav *pn_info_logger(void) {
  return &pn_log;
}
