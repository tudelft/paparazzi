/*
Author: Tommaso Calzolari
email: tcalzolari@tudelft.nl
Group 9 from MAVLab 2026 

Gate-only navigation module:
 - subscribes ONLY to VISUAL_DETECTION
 - searches for gate
 - aligns to gate
 - flies toward gate
 - performs blind pass-through when close

Revised logic:
 - does NOT use CNN quality threshold here; the detector already decides
    whether a gate is present before setting quality > 0.
 - does NOT use bounding-box width/height as a distance proxy.
 - uses dynamic image width from the incoming message (extra field) to
   compute image center instead of using a fixed center_x value.
 - expects regular VISUAL_DETECTION messages, including negative detections
   (quality == 0), so stale detections can be rejected cleanly.
 */

#include "modules/custom_avoider/custom_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "generated/flight_plan.h"

/* --------------------------------------------------------- */
/* ABI                                                       */
/* --------------------------------------------------------- */

#ifndef GATE_FUSION_VISUAL_DETECTION_ID
#define GATE_FUSION_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event gate_ev;

/* --------------------------------------------------------- */
/* Latest detector output                                    */
/* --------------------------------------------------------- */
/*
  Expected message convention from the detector:
 
    quality > 0  -> positive detection
    quality == 0 -> negative detection (no gate seen in this frame)
 
    extra = image width in pixels
 
  This lets nav:
  - know the current horizontal image center dynamically
  - distinguish a true "no gate now" from "no message arrived"

 */
static uint8_t gate_detected = 0;
static int16_t gate_px = 0;
static int16_t gate_py = 0;
static int16_t gate_pw = 0;
static int16_t gate_ph = 0;
static int32_t gate_quality = 0;
static int16_t gate_img_w = 0;   /* sent by detector in extra */
static uint8_t gate_msg_valid = 0;

/* --------------------------------------------------------- */
/* Tunable settings                                          */
/* --------------------------------------------------------- */

/*
 * Horizontal centering tolerance in pixels.
 * TUNING:
 * - too small  -> oscillations / slow alignment
 * - too large  -> drone starts forward motion while still misaligned
 */
#ifndef GATE_FUSION_ALIGN_TOL_PX
#define GATE_FUSION_ALIGN_TOL_PX 10
#endif

/*
 * Number of nav periodic cycles after the last received detector message
 * before we consider the visual data stale.
 *
 * TUNING:
 * - if detector publishes slowly, this must be large enough
 * - if this is too large, nav may act on stale visual data
 * - if this is too small, nav may keep dropping valid detections
 */
#ifndef GATE_FUSION_MAX_MSG_AGE_CYCLES
#define GATE_FUSION_MAX_MSG_AGE_CYCLES 3
#endif

/*
 * Number of FLY_TO_GATE cycles with continuous fresh detection before we
 * commit to blind pass-through.
 *
 * This remains one of the blind-pass engagement conditions.
 *
 * TUNING:
 * - too small  -> blind mode may start too early
 * - too large  -> drone may hesitate too long before committing
 *
 * Best tuned from flight tests using your actual forward speed and nav rate.
 */
#ifndef GATE_FUSION_APPROACH_CYCLES_BEFORE_BLIND
#define GATE_FUSION_APPROACH_CYCLES_BEFORE_BLIND 8
#endif

/*
 * Additional blind-pass engagement condition:
 * require the gate to remain horizontally centered for a minimum number of
 * consecutive FLY_TO_GATE cycles.
 *
 * This complements the approach-cycle heuristic by requiring some visual
 * stability before committing to blind forward motion.
 */
#ifndef GATE_FUSION_CENTERED_CYCLES_BEFORE_BLIND
#define GATE_FUSION_CENTERED_CYCLES_BEFORE_BLIND 3u
#endif

/*
 * While flying toward the gate, apply heading corrections less aggressively
 * than in ALIGN_TO_GATE:
 * - use a smaller heading increment
 * - only react to every N-th new detector message
 *
 * TUNING:
 * - smaller increment reduces oscillations during approach
 * - larger RX stride makes the behavior less twitchy, but slower to correct
 */
#ifndef GATE_FUSION_FLY_HEADING_INC_DEG
#define GATE_FUSION_FLY_HEADING_INC_DEG 4.0f
#endif

#ifndef GATE_FUSION_FLY_CORRECTION_EVERY_RX
#define GATE_FUSION_FLY_CORRECTION_EVERY_RX 1u
#endif

/*
 * Number of cycles to continue flying forward without vision after deciding
 * to pass through the gate.
 *
 * TUNING:
 * - too small  -> may stop before fully crossing the gate
 * - too large  -> may overshoot significantly after the gate
 */
#ifndef GATE_FUSION_BLIND_CYCLES
#define GATE_FUSION_BLIND_CYCLES 10
#endif

/*
 * Forward distances used to move the reference waypoints along the current
 * heading direction.
 *
 * TUNING:
 * - too large  -> aggressive motion, less time for correction
 * - too small  -> sluggish, hesitant approach
 */
#ifndef GATE_FUSION_FORWARD_DIST
#define GATE_FUSION_FORWARD_DIST 1.0f
#endif

#ifndef GATE_FUSION_TRAJ_FORWARD_DIST
#define GATE_FUSION_TRAJ_FORWARD_DIST 1.5f
#endif

/*
 * Heading increment used while searching for a gate.
 *
 * TUNING:
 * - too large  -> search may spin too fast and skip detections
 * - too small  -> search becomes slow
 */
#ifndef GATE_FUSION_SEARCH_HEADING_INC_DEG
#define GATE_FUSION_SEARCH_HEADING_INC_DEG 4.0f
#endif

/*
 * Heading increment used while aligning the gate horizontally to the image
 * center.
 *
 * TUNING:
 * - too large  -> overshoot / oscillation
 * - too small  -> slow alignment
 */
#ifndef GATE_FUSION_ALIGN_HEADING_INC_DEG
#define GATE_FUSION_ALIGN_HEADING_INC_DEG 3.0f
#endif

#ifndef GATE_FUSION_DEBUG
#define GATE_FUSION_DEBUG true
#endif

#ifndef GATE_FUSION_DEBUG_PERIOD
#define GATE_FUSION_DEBUG_PERIOD 20
#endif

#ifndef GATE_FUSION_DEBUG_ALL_DECISIONS
#define GATE_FUSION_DEBUG_ALL_DECISIONS true
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
static float heading_increment = GATE_FUSION_SEARCH_HEADING_INC_DEG;
static uint8_t blind_cycles_remaining = 0;
static uint16_t debug_cycle_count = 0;
static uint32_t nav_periodic_counter = 0u;
static uint32_t gate_rx_counter = 0u;
static uint32_t last_gate_rx_periodic_counter = 0u;
static uint32_t last_fly_heading_correction_rx_count = 0u;

/*
 * Counts how many consecutive FLY_TO_GATE cycles we have had with a fresh
 * positive detection.
 */
static uint16_t fly_cycles_with_gate = 0u;
static uint16_t fly_centered_cycles = 0u;

/* --------------------------------------------------------- */
/* Forward declarations                                      */
/* --------------------------------------------------------- */

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);

static uint8_t gate_msg_is_fresh(void);
static uint8_t gate_msg_geometry_is_valid(void);
static uint8_t gate_is_good(void);
static uint8_t gate_is_close(void);

static int16_t gate_image_center_x(void);
static int16_t gate_error_x(void);

static const char *navigation_state_name(enum navigation_state_t state);
static void set_navigation_state(enum navigation_state_t new_state, const char *reason);
static void debug_print_periodic_status(const char *phase, int16_t err_x);
static void debug_print_decision(const char *phase, const char *decision, int16_t err_x);

#if GATE_FUSION_DEBUG
#define GATE_DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define GATE_DEBUG_PRINT(...) do { } while (0)
#endif

/* --------------------------------------------------------- */
/* ABI callback                                              */
/* --------------------------------------------------------- */

static void gate_detection_cb(uint8_t sender_id,
                              int16_t pixel_x,
                              int16_t pixel_y,
                              int16_t pixel_width,
                              int16_t pixel_height,
                              int32_t quality,
                              int16_t extra)
{
  gate_px = pixel_x;
  gate_py = pixel_y;
  gate_pw = pixel_width;
  gate_ph = pixel_height;
  gate_quality = quality;
  gate_img_w = extra; /* detector should send image width here */

  /*
   * Detection semantics:
   * - quality > 0  => positive detection
   * - quality == 0 => negative detection (detector ran, no gate found)
   */
  gate_detected = (quality > 0) ? 1u : 0u;

  /*
   * Validity of the incoming message geometry:
   * - image width must be > 0
   * - for positive detections, pixel_x must lie inside the reported image width
   */
  gate_msg_valid = 1u;
  if (gate_img_w <= 1) {
    gate_msg_valid = 0u;
  }
  if (gate_detected && (gate_px < 0 || gate_px >= gate_img_w)) {
    gate_msg_valid = 0u;
  }

  gate_rx_counter++;
  last_gate_rx_periodic_counter = nav_periodic_counter;

  GATE_DEBUG_PRINT("[custom_avoider.c][rx VISUAL_DETECTION from gate_cnn_detector.c] sender_id=%u px=%d py=%d w=%d h=%d quality=%ld img_w=%d det=%u valid=%u rx_count=%lu\n",
                   sender_id,
                   pixel_x,
                   pixel_y,
                   pixel_width,
                   pixel_height,
                   (long)quality,
                   gate_img_w,
                   gate_detected,
                   gate_msg_valid,
                   (unsigned long)gate_rx_counter);
}

/* --------------------------------------------------------- */
/* Init                                                      */
/* --------------------------------------------------------- */

void navigation_controller_init(void)
{
  navigation_state = SEARCH_FOR_GATE;
  heading_increment = GATE_FUSION_SEARCH_HEADING_INC_DEG;
  blind_cycles_remaining = 0;
  debug_cycle_count = 0;
  nav_periodic_counter = 0u;
  gate_rx_counter = 0u;
  last_gate_rx_periodic_counter = 0u;
  last_fly_heading_correction_rx_count = 0u;
  fly_cycles_with_gate = 0u;
  fly_centered_cycles = 0u;

  gate_detected = 0u;
  gate_px = 0;
  gate_py = 0;
  gate_pw = 0;
  gate_ph = 0;
  gate_quality = 0;
  gate_img_w = 0;
  gate_msg_valid = 0u;

  AbiBindMsgVISUAL_DETECTION(GATE_FUSION_VISUAL_DETECTION_ID,
                             &gate_ev,
                             gate_detection_cb);

  GATE_DEBUG_PRINT("[gate_nav] init: align_tol=%d stale_cycles=%u search_inc=%.2f align_inc=%.2f blind_after_fly_cycles=%u blind_cycles=%u\n",
                   GATE_FUSION_ALIGN_TOL_PX,
                   (unsigned)GATE_FUSION_MAX_MSG_AGE_CYCLES,
                   (double)GATE_FUSION_SEARCH_HEADING_INC_DEG,
                   (double)GATE_FUSION_ALIGN_HEADING_INC_DEG,
                   (unsigned)GATE_FUSION_APPROACH_CYCLES_BEFORE_BLIND,
                   (unsigned)GATE_FUSION_BLIND_CYCLES);
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
  nav_periodic_counter++;

  switch (navigation_state) {

    case SEARCH_FOR_GATE:
      /*
       * Search behavior:
       * rotate at a fixed heading increment until a fresh positive detection
       * with valid geometry arrives.
       *
       * TUNING:
       * search heading increment strongly affects how aggressively the drone
       * scans the scene.
       */
      debug_print_decision("search", "rotate_search", gate_error_x());
      increase_nav_heading(heading_increment);
      debug_print_periodic_status("search", gate_error_x());

      if (gate_is_good()) {
        debug_print_decision("search", "gate_acquired", gate_error_x());
        set_navigation_state(ALIGN_TO_GATE, "fresh gate acquired");
      }
      break;

    case ALIGN_TO_GATE: {
      /*
       * Alignment behavior:
       * use the image width received from the detector to compute the current
       * image center dynamically. This avoids assuming a fixed center_x = 120.
       */
      if (!gate_is_good()) {
        debug_print_decision("align", "lost_gate", gate_error_x());
        fly_centered_cycles = 0u;
        set_navigation_state(SEARCH_FOR_GATE, "lost gate during align");
        break;
      }

      int16_t err_x = gate_error_x();
      debug_print_periodic_status("align", err_x);

      if (err_x < -GATE_FUSION_ALIGN_TOL_PX) {
        debug_print_decision("align", "turn_left", err_x);
        increase_nav_heading(GATE_FUSION_ALIGN_HEADING_INC_DEG);
      } else if (err_x > GATE_FUSION_ALIGN_TOL_PX) {
        debug_print_decision("align", "turn_right", err_x);
        increase_nav_heading(-GATE_FUSION_ALIGN_HEADING_INC_DEG);
      } else {
        debug_print_decision("align", "gate_centered", err_x);
        fly_cycles_with_gate = 0u;
        fly_centered_cycles = 0u;
        last_fly_heading_correction_rx_count = gate_rx_counter;
        set_navigation_state(FLY_TO_GATE, "gate centered");
      }
      break;
    }

    case FLY_TO_GATE:
      /*
       * Approach behavior:
       * continue moving forward while the gate remains visible and fresh.
       * While approaching, keep applying small heading corrections, but only
       * on a subsampled set of incoming detections to avoid reacting to every
       * single update.
       *
       * Since bbox-based closeness was removed, we use a simple heuristic:
       * after N consecutive FLY cycles with a fresh gate, commit to blind mode.
       *
       */
      if (!gate_is_good()) {
        debug_print_decision("fly", "lost_gate", gate_error_x());
        fly_cycles_with_gate = 0u;
        fly_centered_cycles = 0u;
        set_navigation_state(SEARCH_FOR_GATE, "lost gate during approach");
        break;
      }

      debug_print_periodic_status("fly", gate_error_x());

      fly_cycles_with_gate++;

      int16_t fly_err_x = gate_error_x();

      if (abs(fly_err_x) <= GATE_FUSION_ALIGN_TOL_PX) {
        fly_centered_cycles++;
      } else {
        fly_centered_cycles = 0u;
      }

      uint32_t rx_since_last_fly_correction = gate_rx_counter - last_fly_heading_correction_rx_count;

      if (rx_since_last_fly_correction >= GATE_FUSION_FLY_CORRECTION_EVERY_RX) {
        if (fly_err_x < -GATE_FUSION_ALIGN_TOL_PX) {
          debug_print_decision("fly", "correct_left", fly_err_x);
          increase_nav_heading(GATE_FUSION_FLY_HEADING_INC_DEG);
          last_fly_heading_correction_rx_count = gate_rx_counter;
        } else if (fly_err_x > GATE_FUSION_ALIGN_TOL_PX) {
          debug_print_decision("fly", "correct_right", fly_err_x);
          increase_nav_heading(-GATE_FUSION_FLY_HEADING_INC_DEG);
          last_fly_heading_correction_rx_count = gate_rx_counter;
        }
      }

      if (gate_is_close()) {
        blind_cycles_remaining = GATE_FUSION_BLIND_CYCLES;
        debug_print_decision("fly", "switch_to_blind", fly_err_x);
        set_navigation_state(BLIND_THROUGH_GATE, "approach duration reached");
        break;
      }

      debug_print_decision("fly", "advance_waypoints", fly_err_x);
      moveWaypointForward(WP_TRAJECTORY, GATE_FUSION_TRAJ_FORWARD_DIST);
      moveWaypointForward(WP_GOAL, GATE_FUSION_FORWARD_DIST);
      break;

    case BLIND_THROUGH_GATE:
      /*
       * Blind pass-through:
       * keep flying forward for a fixed number of cycles. This is intentionally
       * independent from vision, because detector output can become unstable
       * very close to and while crossing the gate.
       *
       * TUNING:
       * blind_cycles_remaining must be tuned against actual airspeed and nav
       * loop rate.
       */
      debug_print_decision("blind", "blind_forward", gate_error_x());
      moveWaypointForward(WP_TRAJECTORY, GATE_FUSION_TRAJ_FORWARD_DIST);
      moveWaypointForward(WP_GOAL, GATE_FUSION_FORWARD_DIST);

      if (blind_cycles_remaining > 0) {
        blind_cycles_remaining--;
      }

      GATE_DEBUG_PRINT("[custom_avoider.c][state BLIND] remaining_cycles=%u\n",
                       blind_cycles_remaining);

      if (blind_cycles_remaining == 0) {
        fly_cycles_with_gate = 0u;
        fly_centered_cycles = 0u;
        debug_print_decision("blind", "blind_complete", gate_error_x());
        set_navigation_state(SEARCH_FOR_GATE, "blind pass complete");
      }
      break;

    default:
      debug_print_decision("unknown", "invalid_state_reset", gate_error_x());
      fly_cycles_with_gate = 0u;
      fly_centered_cycles = 0u;
      set_navigation_state(SEARCH_FOR_GATE, "invalid state");
      break;
  }
}

/* --------------------------------------------------------- */
/* Helpers                                                   */
/* --------------------------------------------------------- */

static uint8_t gate_msg_is_fresh(void)
{
  uint32_t msg_age_cycles = nav_periodic_counter - last_gate_rx_periodic_counter;
  return (msg_age_cycles <= GATE_FUSION_MAX_MSG_AGE_CYCLES) ? 1u : 0u;
}

static uint8_t gate_msg_geometry_is_valid(void)
{
  return gate_msg_valid;
}

static uint8_t gate_is_good(void)
{

  return (gate_msg_is_fresh() &&
          gate_msg_geometry_is_valid() &&
          gate_detected);
}

static uint8_t gate_is_close(void)
{
  /*
   * Blind-pass engagement heuristic:
   * - enough continuous FLY_TO_GATE cycles with fresh detections
   * - plus a minimum consecutive centered window
   */
  if (!gate_is_good()) {
    return 0u;
  }

  return ((fly_cycles_with_gate >= GATE_FUSION_APPROACH_CYCLES_BEFORE_BLIND) &&
          (fly_centered_cycles >= GATE_FUSION_CENTERED_CYCLES_BEFORE_BLIND)) ? 1u : 0u;
}

static int16_t gate_image_center_x(void)
{
  if (gate_img_w <= 1) {
    return 0;
  }
  return gate_img_w / 2;
}

static int16_t gate_error_x(void)
{
  if (!gate_msg_geometry_is_valid()) {
    return 0;
  }
  return gate_px - gate_image_center_x();
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

  GATE_DEBUG_PRINT("[gate_nav] %s -> %s: %s (det=%u fresh=%u valid=%u q=%ld px=%d py=%d img_w=%d err=%d blind=%u fly_cycles=%u)\n",
                   navigation_state_name(navigation_state),
                   navigation_state_name(new_state),
                   reason,
                   gate_detected,
                   gate_msg_is_fresh(),
                   gate_msg_geometry_is_valid(),
                   (long)gate_quality,
                   gate_px,
                   gate_py,
                   gate_img_w,
                   gate_error_x(),
                   blind_cycles_remaining,
                   fly_cycles_with_gate);

  navigation_state = new_state;
}

static void debug_print_periodic_status(const char *phase, int16_t err_x)
{
#if GATE_FUSION_DEBUG
  if ((debug_cycle_count % GATE_FUSION_DEBUG_PERIOD) != 0) {
    return;
  }

  GATE_DEBUG_PRINT("[gate_nav] %s: state=%s det=%u fresh=%u valid=%u good=%u close=%u q=%ld px=%d py=%d img_w=%d center_x=%d err=%d w=%d h=%d fly_cycles=%u centered_cycles=%u\n",
                   phase,
                   navigation_state_name(navigation_state),
                   gate_detected,
                   gate_msg_is_fresh(),
                   gate_msg_geometry_is_valid(),
                   gate_is_good(),
                   gate_is_close(),
                   (long)gate_quality,
                   gate_px,
                   gate_py,
                   gate_img_w,
                   gate_image_center_x(),
                   err_x,
                   gate_pw,
                   gate_ph,
                   fly_cycles_with_gate,
                   fly_centered_cycles);
#else
  (void)phase;
  (void)err_x;
#endif
}

static void debug_print_decision(const char *phase, const char *decision, int16_t err_x)
{
#if GATE_FUSION_DEBUG && GATE_FUSION_DEBUG_ALL_DECISIONS
  uint32_t msg_age_cycles = nav_periodic_counter - last_gate_rx_periodic_counter;
  GATE_DEBUG_PRINT("[custom_avoider.c][decision] phase=%s state=%s action=%s det=%u fresh=%u valid=%u q=%ld px=%d py=%d img_w=%d center_x=%d err=%d w=%d h=%d rx_count=%lu msg_age_cycles=%lu fly_cycles=%u centered_cycles=%u\n",
                   phase,
                   navigation_state_name(navigation_state),
                   decision,
                   gate_detected,
                   gate_msg_is_fresh(),
                   gate_msg_geometry_is_valid(),
                   (long)gate_quality,
                   gate_px,
                   gate_py,
                   gate_img_w,
                   gate_image_center_x(),
                   err_x,
                   gate_pw,
                   gate_ph,
                   (unsigned long)gate_rx_counter,
                   (unsigned long)msg_age_cycles,
                   fly_cycles_with_gate,
                   fly_centered_cycles);
#else
  (void)phase;
  (void)decision;
  (void)err_x;
#endif
}

static uint8_t increase_nav_heading(float incrementDegrees)
{
  float current_heading = stateGetNedToBodyEulers_f()->psi;
  float new_heading = current_heading + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;

  GATE_DEBUG_PRINT("[custom_avoider.c][tx nav.heading] current=%.3f rad inc=%.3f deg new=%.3f rad\n",
                   (double)current_heading,
                   (double)incrementDegrees,
                   (double)new_heading);
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
  float heading = nav.heading;

  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distanceMeters);
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distanceMeters);
  return false;
}

static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  GATE_DEBUG_PRINT("[custom_avoider.c][tx waypoint_move_xy_i] waypoint=%u x=%ld y=%ld\n",
                   waypoint,
                   (long)new_coor->x,
                   (long)new_coor->y);
  return false;
}
