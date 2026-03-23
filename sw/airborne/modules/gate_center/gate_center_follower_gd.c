#include "modules/gate_center/gate_center_follower_gd.h"

#include "autopilot.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "mcu_periph/sys_time.h"
#include "modules/core/abi.h"
#include "modules/core/abi_sender_ids.h"
#include "modules/nav/waypoints.h"
#include "state.h"

#include <math.h>

#ifndef GATE_CENTER_RELATIVE_LOCALIZATION_ID
#define GATE_CENTER_RELATIVE_LOCALIZATION_ID DETECT_GATE_ABI_ID
#endif

#ifndef GCF_FORWARD_SPEED
#define GCF_FORWARD_SPEED 0.35f
#endif

#ifndef GCF_TARGET_ALT
#define GCF_TARGET_ALT 1.0f
#endif

#ifndef GCF_DETECTION_TIMEOUT_S
#define GCF_DETECTION_TIMEOUT_S 0.90f
#endif

#ifndef GCF_TURN_DEGREES
#define GCF_TURN_DEGREES 180.0f
#endif

#ifndef GCF_TURN_RATE_DEG_S
#define GCF_TURN_RATE_DEG_S 90.0f
#endif

#ifndef GCF_POST_LOSS_STRAIGHT_S
#define GCF_POST_LOSS_STRAIGHT_S 2.0f
#endif

float gcf_forward_speed = GCF_FORWARD_SPEED;
float gcf_detection_timeout_s = GCF_DETECTION_TIMEOUT_S;
float gcf_target_alt = GCF_TARGET_ALT;
float gcf_turn_degrees = GCF_TURN_DEGREES;
float gcf_turn_rate_deg_s = GCF_TURN_RATE_DEG_S;
float gcf_post_loss_straight_s = GCF_POST_LOSS_STRAIGHT_S;

static abi_event gcf_rel_loc_ev;
static float gcf_last_seen_s = -1000.f;
static float gcf_last_periodic_s = -1.f;
static float gcf_turn_remaining_deg = 0.f;
static float gcf_post_loss_deadline_s = -1.f;

enum gcf_state_t {
  GCF_WAIT_FOR_GATE = 0,
  GCF_GO_STRAIGHT,
  GCF_GO_STRAIGHT_AFTER_LOSS,
  GCF_TURNING_180
};

static enum gcf_state_t gcf_state = GCF_WAIT_FOR_GATE;

static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void normalize_heading(float *heading)
{
  while (*heading > M_PI) {
    *heading -= 2.0f * M_PI;
  }
  while (*heading < -M_PI) {
    *heading += 2.0f * M_PI;
  }
}

static void move_waypoint_forward_alt(uint8_t waypoint, float forward_m, float target_alt)
{
  struct EnuCoor_i new_coor;
  float heading = stateGetNedToBodyEulers_f()->psi;

  float dx = sinf(heading) * forward_m;
  float dy = cosf(heading) * forward_m;

  new_coor.x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(dx);
  new_coor.y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(dy);
  new_coor.z = POS_BFP_OF_REAL(target_alt);
  waypoint_move_enu_i(waypoint, &new_coor);
}

static void hold_position_at_alt(float target_alt)
{
  struct EnuCoor_i hold = *stateGetPositionEnu_i();
  hold.z = POS_BFP_OF_REAL(target_alt);
  waypoint_move_enu_i(WP_GOAL, &hold);
  waypoint_move_enu_i(WP_TRAJECTORY, &hold);
}

static void gcf_relative_localization_cb(uint8_t __attribute__((unused)) sender_id,
                                         int32_t __attribute__((unused)) id,
                                         float x, float y, float z,
                                         float __attribute__((unused)) vx,
                                         float __attribute__((unused)) vy,
                                         float __attribute__((unused)) vz)
{
  (void)x;
  (void)y;
  (void)z;
  gcf_last_seen_s = get_sys_time_float();
}

void gate_center_follower_gd_init(void)
{
  AbiBindMsgRELATIVE_LOCALIZATION(GATE_CENTER_RELATIVE_LOCALIZATION_ID,
                                  &gcf_rel_loc_ev,
                                  gcf_relative_localization_cb);
}

void gate_center_follower_gd_periodic(void)
{
  if (!autopilot_in_flight()) {
    gcf_state = GCF_WAIT_FOR_GATE;
    gcf_turn_remaining_deg = 0.f;
    gcf_post_loss_deadline_s = -1.f;
    gcf_last_periodic_s = -1.f;
    return;
  }

  float now_s = get_sys_time_float();
  float dt_s = 0.25f;
  if (gcf_last_periodic_s > 0.f) {
    dt_s = clampf(now_s - gcf_last_periodic_s, 0.01f, 0.50f);
  }
  gcf_last_periodic_s = now_s;

  bool gate_visible = ((now_s - gcf_last_seen_s) <= gcf_detection_timeout_s);
  float target_alt = gcf_target_alt;

  switch (gcf_state) {
    case GCF_WAIT_FOR_GATE:
      hold_position_at_alt(target_alt);
      if (gate_visible) {
        gcf_state = GCF_GO_STRAIGHT;
      }
      break;

    case GCF_GO_STRAIGHT:
      move_waypoint_forward_alt(WP_GOAL, gcf_forward_speed, target_alt);
      move_waypoint_forward_alt(WP_TRAJECTORY, 1.5f * gcf_forward_speed, target_alt);
      if (!gate_visible) {
        gcf_state = GCF_GO_STRAIGHT_AFTER_LOSS;
        gcf_post_loss_deadline_s = now_s + gcf_post_loss_straight_s;
      }
      break;

    case GCF_GO_STRAIGHT_AFTER_LOSS:
      move_waypoint_forward_alt(WP_GOAL, gcf_forward_speed, target_alt);
      move_waypoint_forward_alt(WP_TRAJECTORY, 1.5f * gcf_forward_speed, target_alt);
      if (gate_visible) {
        gcf_state = GCF_GO_STRAIGHT;
      } else if (now_s >= gcf_post_loss_deadline_s) {
        gcf_state = GCF_TURNING_180;
        gcf_turn_remaining_deg = gcf_turn_degrees;
      }
      break;

    case GCF_TURNING_180: {
      hold_position_at_alt(target_alt);
      if (gcf_turn_remaining_deg > 0.f) {
        float step_deg = fminf(gcf_turn_remaining_deg, gcf_turn_rate_deg_s * dt_s);
        nav.heading += RadOfDeg(step_deg);
        normalize_heading(&nav.heading);
        gcf_turn_remaining_deg -= step_deg;
      } else {
        gcf_state = GCF_WAIT_FOR_GATE;
        gcf_post_loss_deadline_s = -1.f;
      }
      break;
    }

    default:
      gcf_state = GCF_WAIT_FOR_GATE;
      break;
  }
}
