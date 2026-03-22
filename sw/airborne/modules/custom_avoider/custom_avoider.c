/*
 * Gate-only navigation module:
 * - subscribes ONLY to VISUAL_DETECTION
 * - searches for gate
 * - aligns to gate
 * - flies toward gate
 * - performs blind pass-through when close
 */

#include "modules/custom_avoider/custom_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "generated/flight_plan.h"

/* --------------------------------------------------------- */
/* ABI                                                       */
/* --------------------------------------------------------- */

#ifndef GATE_FUSION_VISUAL_DETECTION_ID
#define GATE_FUSION_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event gate_ev;

/* gate detector output */
static uint8_t gate_detected = 0;
static int16_t gate_px = 0;
static int16_t gate_pw = 0;
static int16_t gate_ph = 0;
static int32_t gate_quality = 0;

/* --------------------------------------------------------- */
/* Tunable settings                                          */
/* --------------------------------------------------------- */

#ifndef GATE_FUSION_IMAGE_CENTER_X
#define GATE_FUSION_IMAGE_CENTER_X 120
#endif

#ifndef GATE_FUSION_ALIGN_TOL_PX
#define GATE_FUSION_ALIGN_TOL_PX 20
#endif

#ifndef GATE_FUSION_MIN_QUALITY
#define GATE_FUSION_MIN_QUALITY 250
#endif

#ifndef GATE_FUSION_BLIND_QUALITY
#define GATE_FUSION_BLIND_QUALITY 500
#endif

#ifndef GATE_FUSION_BLIND_MIN_WIDTH
#define GATE_FUSION_BLIND_MIN_WIDTH 120
#endif

#ifndef GATE_FUSION_BLIND_MIN_HEIGHT
#define GATE_FUSION_BLIND_MIN_HEIGHT 120
#endif

#ifndef GATE_FUSION_BLIND_CYCLES
#define GATE_FUSION_BLIND_CYCLES 10
#endif

#ifndef GATE_FUSION_FORWARD_DIST
#define GATE_FUSION_FORWARD_DIST 1.0f
#endif

#ifndef GATE_FUSION_TRAJ_FORWARD_DIST
#define GATE_FUSION_TRAJ_FORWARD_DIST 1.5f
#endif

#ifndef GATE_FUSION_SEARCH_HEADING_INC_DEG
#define GATE_FUSION_SEARCH_HEADING_INC_DEG 5.0f
#endif

#ifndef GATE_FUSION_ALIGN_HEADING_INC_DEG
#define GATE_FUSION_ALIGN_HEADING_INC_DEG 4.0f
#endif

#ifndef GATE_FUSION_DEBUG
#define GATE_FUSION_DEBUG true
#endif

#ifndef GATE_FUSION_DEBUG_PERIOD
#define GATE_FUSION_DEBUG_PERIOD 20
#endif

/* --------------------------------------------------------- */
/* State machine                                             */
/* --------------------------------------------------------- */

enum navigation_state_t {
  SEARCH_FOR_GATE = 0,
  ALIGN_TO_GATE,
  FLY_TO_GATE,
  BLIND_THROUGH_GATE
};

static enum navigation_state_t navigation_state = SEARCH_FOR_GATE;
static float heading_increment = 5.f;
static uint8_t blind_cycles_remaining = 0;
static uint16_t debug_cycle_count = 0;

/* --------------------------------------------------------- */
/* Forward declarations                                      */
/* --------------------------------------------------------- */

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t gate_is_good(void);
static uint8_t gate_is_close(void);
static const char *navigation_state_name(enum navigation_state_t state);
static void set_navigation_state(enum navigation_state_t new_state, const char *reason);
static void debug_print_periodic_status(const char *phase, int16_t err_x);

#if GATE_FUSION_DEBUG
#define GATE_DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define GATE_DEBUG_PRINT(...) do { } while (0)
#endif

/* --------------------------------------------------------- */
/* ABI callback                                              */
/* --------------------------------------------------------- */

static void gate_detection_cb(uint8_t sender_id __attribute__((unused)),
                              int16_t pixel_x,
                              int16_t pixel_y __attribute__((unused)),
                              int16_t pixel_width,
                              int16_t pixel_height,
                              int32_t quality,
                              int16_t extra __attribute__((unused)))
{
  gate_px = pixel_x;
  gate_pw = pixel_width;
  gate_ph = pixel_height;
  gate_quality = quality;
  gate_detected = (quality > 0) ? 1 : 0;
}

/* --------------------------------------------------------- */
/* Init                                                      */
/* --------------------------------------------------------- */

void navigation_controller_init(void)
{
  navigation_state = SEARCH_FOR_GATE;
  blind_cycles_remaining = 0;
  debug_cycle_count = 0;

  AbiBindMsgVISUAL_DETECTION(GATE_FUSION_VISUAL_DETECTION_ID,
                             &gate_ev,
                             gate_detection_cb);

  GATE_DEBUG_PRINT("[gate_nav] init: min_q=%ld blind_q=%ld align_tol=%d center_x=%d\n",
                   (long)GATE_FUSION_MIN_QUALITY,
                   (long)GATE_FUSION_BLIND_QUALITY,
                   GATE_FUSION_ALIGN_TOL_PX,
                   GATE_FUSION_IMAGE_CENTER_X);
}

/* --------------------------------------------------------- */
/* Main loop                                                 */
/* --------------------------------------------------------- */

void navigation_controller_periodic(void)
{
  if (!autopilot_in_flight()) {
    return;
  }

  debug_cycle_count++;

  switch (navigation_state) {

    case SEARCH_FOR_GATE:
      increase_nav_heading(heading_increment);
      debug_print_periodic_status("search", gate_px - GATE_FUSION_IMAGE_CENTER_X);

      if (gate_is_good()) {
        set_navigation_state(ALIGN_TO_GATE, "gate acquired");
      }
      break;

    case ALIGN_TO_GATE: {
      if (!gate_is_good()) {
        set_navigation_state(SEARCH_FOR_GATE, "lost gate during align");
        break;
      }

      int16_t err_x = gate_px - GATE_FUSION_IMAGE_CENTER_X;
      debug_print_periodic_status("align", err_x);

      if (err_x < -GATE_FUSION_ALIGN_TOL_PX) {
        increase_nav_heading(GATE_FUSION_ALIGN_HEADING_INC_DEG);
      } else if (err_x > GATE_FUSION_ALIGN_TOL_PX) {
        increase_nav_heading(-GATE_FUSION_ALIGN_HEADING_INC_DEG);
      } else {
        set_navigation_state(FLY_TO_GATE, "gate centered");
      }
      break;
    }

    case FLY_TO_GATE:
      if (!gate_is_good()) {
        set_navigation_state(SEARCH_FOR_GATE, "lost gate during approach");
        break;
      }

      debug_print_periodic_status("fly", gate_px - GATE_FUSION_IMAGE_CENTER_X);

      if (gate_is_close()) {
        blind_cycles_remaining = GATE_FUSION_BLIND_CYCLES;
        set_navigation_state(BLIND_THROUGH_GATE, "gate close enough");
        break;
      }

      moveWaypointForward(WP_TRAJECTORY, GATE_FUSION_TRAJ_FORWARD_DIST);
      moveWaypointForward(WP_GOAL, GATE_FUSION_FORWARD_DIST);
      break;

    case BLIND_THROUGH_GATE:
      moveWaypointForward(WP_TRAJECTORY, GATE_FUSION_TRAJ_FORWARD_DIST);
      moveWaypointForward(WP_GOAL, GATE_FUSION_FORWARD_DIST);

      if (blind_cycles_remaining > 0) {
        blind_cycles_remaining--;
      }

      if (blind_cycles_remaining == 0) {
        set_navigation_state(SEARCH_FOR_GATE, "blind pass complete");
      }
      break;

    default:
      set_navigation_state(SEARCH_FOR_GATE, "invalid state");
      break;
  }
}

/* --------------------------------------------------------- */
/* Helpers                                                   */
/* --------------------------------------------------------- */

static uint8_t gate_is_good(void)
{
  return (gate_detected && gate_quality >= GATE_FUSION_MIN_QUALITY);
}

static uint8_t gate_is_close(void)
{
  if (!gate_detected) return 0;

  return (gate_quality >= GATE_FUSION_BLIND_QUALITY ||
          gate_pw >= GATE_FUSION_BLIND_MIN_WIDTH ||
          gate_ph >= GATE_FUSION_BLIND_MIN_HEIGHT);
}

static const char *navigation_state_name(enum navigation_state_t state)
{
  switch (state) {
    case SEARCH_FOR_GATE:
      return "SEARCH";
    case ALIGN_TO_GATE:
      return "ALIGN";
    case FLY_TO_GATE:
      return "FLY";
    case BLIND_THROUGH_GATE:
      return "BLIND";
    default:
      return "UNKNOWN";
  }
}

static void set_navigation_state(enum navigation_state_t new_state, const char *reason)
{
  if (navigation_state == new_state) {
    return;
  }

  GATE_DEBUG_PRINT("[gate_nav] %s -> %s: %s (q=%ld px=%d w=%d h=%d blind=%u)\n",
                   navigation_state_name(navigation_state),
                   navigation_state_name(new_state),
                   reason,
                   (long)gate_quality,
                   gate_px,
                   gate_pw,
                   gate_ph,
                   blind_cycles_remaining);

  navigation_state = new_state;
}

static void debug_print_periodic_status(const char *phase, int16_t err_x)
{
#if GATE_FUSION_DEBUG
  if ((debug_cycle_count % GATE_FUSION_DEBUG_PERIOD) != 0) {
    return;
  }

  GATE_DEBUG_PRINT("[gate_nav] %s: state=%s det=%u good=%u close=%u q=%ld px=%d err=%d w=%d h=%d\n",
                   phase,
                   navigation_state_name(navigation_state),
                   gate_detected,
                   gate_is_good(),
                   gate_is_close(),
                   (long)gate_quality,
                   gate_px,
                   err_x,
                   gate_pw,
                   gate_ph);
#else
  (void)phase;
  (void)err_x;
#endif
}

static uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading = stateGetNedToBodyEulers_f()->psi;

  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distanceMeters);
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distanceMeters);
  return false;
}

static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}
