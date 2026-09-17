/**
 * @file modules/nav/precision_landing.c
 * @brief Fixed-Wing Precision Landing Module Implementation
 * @author TU Delft IMAV 2026 Team / Paparazzi UAV
 *
 * @details
 * Implementation of autonomous precision landing for fixed-wing aircraft using
 * rangefinder-based final-stop prediction, dual-band predictive crow braking,
 * and safety-gated go-around management.
 *
 * ### Architectural & Operational Design
 * 1. **Runway Geometry Initialization (`precision_landing_setup`)**:
 *    Computes the 2D ENU unit vector \f$(\hat{e}_{final}, \hat{n}_{final})\f$ from Approach Fix
 *    \f$AF\f$ to Touchdown Waypoint \f$TD\f$, along with nominal runway slope \f$\gamma_{final}\f$.
 *
 * 2. **Navigation Frame Projection**:
 *    Transforms current ENU position \f$(p_x, p_y)\f$ and velocity \f$(v_x, v_y)\f$ into:
 *    - Along-track remaining distance \f$d_{rem}\f$ and groundspeed \f$v_{along}\f$.
 *    - Cross-track error \f$c_{cross}\f$ and lateral drift rate \f$v_{cross}\f$.
 *
 * 3. **AGL Range & Sink Rate Estimation**:
 *    Consumes rangefinder distance (`agl_dist_value_filtered`). When new rangefinder ABI
 *    samples arrive, computes instantaneous sink rate and updates an exponentially weighted
 *    moving average (EWMA) filter with time constant \f$\tau = 0.15\,\text{s}\f$.
 *    If rangefinder updates stall, resets sink rate to fused vertical speed \f$-v_z\f$.
 *
 * 4. **Dual-Band Predictive Braking (`precision_landing_run`)**:
 *    - **Upper Altitude Band** (above `brake_agl` or before rangefinder acquisition):
 *      Uses fused barometric altitude to compare aircraft height against nominal geometric slope.
 *      Applies up to 50% (`UPPER_MAX_BRAKE`) proportional crow braking if high on slope,
 *      allowing early energy dissipation without waiting for laser lock.
 *    - **Precision Decision Band** (below `brake_agl` with fresh AGL):
 *      Computes time-to-ground \f$t_{contact} = h / v_z\f$ and projects touchdown location.
 *      Applies spoileron drag proportionally to predicted final-stop overshoot beyond `TD`.
 *      Retains bounded predictive braking near ground while airspeed is valid;
 *      the aircraft mixer applies its configured actuator rate limits.
 *
 * 5. **Safety Persistence & Abort State Machine**:
 *    - Aborts if airspeed drops below `MIN_AIRSPEED` (protecting stall margin).
 *    - Aborts if lateral drift prediction exceeds \f$1.2\,\text{m}\f$ (retaining \f$1.3\,\text{m}\f$ margin inside the v5 rules' \f$2.5\,\text{m}\f$ half-width).
 *    - Rejects predictions beyond 8 m past TD or more than 8 m short of the configured upstream aim.
 *    - Evaluates prediction rejection across a \f$0.2\,\text{s}\f$ persistence timer to prevent single-sample sensor noise from triggering premature go-arounds.
 *    - At/below commit height, latches no-powered-abort; failures then request flare.
 *    - Pilot takeover cancels the sequence until another explicit landing entry.
 */

#include "modules/nav/precision_landing.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/core/commands.h"
#include "modules/sonar/agl_dist.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "state.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "modules/air_data/air_data.h"
#if USE_GPS
#include "modules/gps/gps.h"
#endif

#include <math.h>

/* --- Default Configuration Parameters (Overridable via Airframe XML) --- */

/** Minimum approach airspeed (m/s). Below this, release brakes and reject; committed flight requests flare. */
#ifndef PRECISION_LANDING_MIN_AIRSPEED
#define PRECISION_LANDING_MIN_AIRSPEED 8.2f
#endif

/** Maximum allowed predicted cross-track error at touchdown (m). Leaves 1.3m margin inside the 2.5m precision-area edge. */
#ifndef PRECISION_LANDING_MAX_CROSS_TRACK
#define PRECISION_LANDING_MAX_CROSS_TRACK 1.2f
#endif

#ifndef PRECISION_LANDING_APPROACH_CORRIDOR_SLOPE
#define PRECISION_LANDING_APPROACH_CORRIDOR_SLOPE 0.25f
#endif

/** Maximum allowed longitudinal short/overshoot prediction error before aborting (m). */
#ifndef PRECISION_LANDING_MAX_LONG_ERROR
#define PRECISION_LANDING_MAX_LONG_ERROR 8.0f
#endif

/* The geometric aim compensates for airborne tracking/flare travel, not measured ground stopping.
 * Keep its tuning range separate from MAX_LONG_ERROR so tuning cannot widen contact acceptance. */
#ifndef PRECISION_LANDING_MAX_AIM_DISTANCE
#define PRECISION_LANDING_MAX_AIM_DISTANCE 20.0f
#endif

/** Range freshness limit (s). Fused altitude is an upper-band fallback, not a replacement for low-AGL ranging. */
#ifndef PRECISION_LANDING_AGL_TIMEOUT
#define PRECISION_LANDING_AGL_TIMEOUT 0.25f
#endif

#ifndef PRECISION_LANDING_AIRSPEED_TIMEOUT
#define PRECISION_LANDING_AIRSPEED_TIMEOUT 0.5f
#endif

/** Default AGL altitude below which close-range predictive braking is enabled (m). */
#ifndef PRECISION_LANDING_BRAKE_ENABLE_AGL
#define PRECISION_LANDING_BRAKE_ENABLE_AGL 4.0f
#endif

/** Altitude threshold below which powered go-around is forbidden to avoid low-altitude ground strikes (m). */
#ifndef PRECISION_LANDING_ABORT_AGL
#define PRECISION_LANDING_ABORT_AGL 2.5f
#endif

/** Minimum positive sink rate used in denominator during time-to-ground calculation (m/s). Prevents div-by-zero. */
#ifndef PRECISION_LANDING_MIN_SINK_RATE
#define PRECISION_LANDING_MIN_SINK_RATE 0.25f
#endif

/** Time constant (seconds) for range-derived sink rate EWMA filter. 0.15s balances noise rejection & response. */
#ifndef PRECISION_LANDING_AGL_RATE_TAU
#define PRECISION_LANDING_AGL_RATE_TAU 0.15f
#endif

/** Proportional gain converting predicted overshoot meters to normalized crow brake demand (fraction/m). */
#ifndef PRECISION_LANDING_BRAKE_GAIN
#define PRECISION_LANDING_BRAKE_GAIN 0.08f
#endif

/** Maximum allowable total brake command fraction (1.0 = 100% full crow deflection). */
#ifndef PRECISION_LANDING_MAX_BRAKE
#define PRECISION_LANDING_MAX_BRAKE 1.0f
#endif

/** Flare roll setpoint cap (radians). Limits commanded bank; does not guarantee wingtip clearance. */
#ifndef PRECISION_LANDING_FLARE_MAX_ROLL
#define PRECISION_LANDING_FLARE_MAX_ROLL RadOfDeg(8.f)
#endif

/** Maximum crow brake demand allowed in upper barometric altitude band (50% cap preserves pitch authority). */
#ifndef PRECISION_LANDING_UPPER_MAX_BRAKE
#define PRECISION_LANDING_UPPER_MAX_BRAKE 0.5f
#endif

/** Persistence delay requirement (seconds) for boundary violation before confirming abort. Filters noise spikes. */
#ifndef PRECISION_LANDING_REJECT_DELAY
#define PRECISION_LANDING_REJECT_DELAY 0.2f
#endif

#ifndef PRECISION_LANDING_APPROACH_AIRSPEED
#define PRECISION_LANDING_APPROACH_AIRSPEED 9.f
#endif
#ifndef PRECISION_LANDING_FINAL_HEIGHT
#define PRECISION_LANDING_FINAL_HEIGHT 17.f
#endif
#ifndef PRECISION_LANDING_BRAKE_AGL
#define PRECISION_LANDING_BRAKE_AGL 3.5f
#endif
#ifndef PRECISION_LANDING_FLARE_AGL
#define PRECISION_LANDING_FLARE_AGL 1.2f
#endif
#ifndef PRECISION_LANDING_AIM_BEFORE_TD
#define PRECISION_LANDING_AIM_BEFORE_TD 12.f
#endif
#ifndef PRECISION_LANDING_STOP_DISTANCE
#define PRECISION_LANDING_STOP_DISTANCE 0.f
#endif
#ifndef PRECISION_LANDING_TOUCHDOWN_PITCH
#define PRECISION_LANDING_TOUCHDOWN_PITCH 0.f
#endif
#ifndef PRECISION_LANDING_FLARE_BRAKE
#define PRECISION_LANDING_FLARE_BRAKE 0.65f
#endif
#ifndef PRECISION_LANDING_MAX_RETRIES
#define PRECISION_LANDING_MAX_RETRIES 0
#endif

/* --- Global Module State Definitions --- */

float precision_landing_remaining_m;             /**< Along-track distance to TD (m) */
float precision_landing_cross_track_m;            /**< Lateral cross-track offset from runway centerline (m) */
float precision_landing_predicted_cross_track_m;  /**< Projected cross-track displacement at touchdown (m) */
float precision_landing_predicted_error_m;        /**< Projected longitudinal offset from TD at touchdown (m) */
float precision_landing_brake_fraction;           /**< Currently commanded crow brake demand (0.0 to 1.0) */
bool precision_landing_agl_fresh;                 /**< Fresh rangefinder measurement available */
bool precision_landing_abort;                     /**< True if approach safety boundary violated */
bool precision_landing_commit_flare;              /**< True if below commit height (forces flare) */
float precision_landing_approach_airspeed = PRECISION_LANDING_APPROACH_AIRSPEED;
float precision_landing_final_height = PRECISION_LANDING_FINAL_HEIGHT;
float precision_landing_brake_agl = PRECISION_LANDING_BRAKE_AGL;
float precision_landing_flare_agl = PRECISION_LANDING_FLARE_AGL;
float precision_landing_aim_before_td = PRECISION_LANDING_AIM_BEFORE_TD;
float precision_landing_stop_distance = PRECISION_LANDING_STOP_DISTANCE;
float precision_landing_touchdown_pitch = PRECISION_LANDING_TOUCHDOWN_PITCH;
float precision_landing_flare_brake = PRECISION_LANDING_FLARE_BRAKE;
uint8_t precision_landing_max_retries = PRECISION_LANDING_MAX_RETRIES;

/* --- Module Private Static Variables --- */

static float final_unit_east;      /**< Unit vector East component along final approach direction */
static float final_unit_north;     /**< Unit vector North component along final approach direction */
static float final_slope;          /**< Nominal runway glide slope angle ratio (dz / dx) */
static float brake_enable_agl_m;   /**< Active AGL threshold for close-range predictive braking (m) */
static float aim_before_td_m;      /**< Upstream target distance from TD for the nominal glide path (m) */
static float stop_distance_m;      /**< Calibrated distance from predicted first contact to final rest (m) */
static float previous_agl_m;       /**< Previous AGL sample stored for numerical differentiation (m) */
static float previous_agl_time;    /**< Timestamp of previous AGL sample (s) */
static float range_sink_rate_mps;  /**< Filtered vertical sink rate derived from rangefinder differentiation (m/s) */
static uint8_t range_rate_samples; /**< Number of consecutive valid range-rate samples accumulated */
static uint8_t touchdown_wp;       /**< Waypoint index of target touchdown location */
static uint8_t approach_wp;
static float approach_east, approach_north, approach_altitude;
static float touchdown_east, touchdown_north, touchdown_altitude;
static uint8_t landing_zone_wp[4];
static float landing_zone_east[4], landing_zone_north[4];
static bool landing_zone_configured;
static bool precision_landing_ready;/**< Flag confirming valid runway vector setup */
static float rejection_since;      /**< Timestamp when prediction rejection condition first triggered (s) */
static bool landing_committed;
bool precision_landing_cancelled;
static bool landing_active;
static bool bench_active;
static bool roll_limit_owned;
static float saved_roll_limit;
static uint8_t landing_retry_count;
static enum PrecisionLandingPhase landing_phase;

enum PrecisionLandingPhase precision_landing_get_phase(void)
{
  return landing_phase;
}

void precision_landing_reset_retries(void)
{
  landing_retry_count = 0;
}

void precision_landing_record_retry(void)
{
  if (landing_retry_count < 6) {
    landing_retry_count++;
  }
}

bool precision_landing_retry_allowed(void)
{
  return landing_retry_count <= precision_landing_max_retries;
}

static bool airspeed_safe(void)
{
  const float age = get_sys_time_float() - air_data_airspeed_time;
  return air_data_airspeed_time >= 0.f && age >= 0.f && age < PRECISION_LANDING_AIRSPEED_TIMEOUT
         && precision_landing_airspeed_safe(stateIsAirspeedValid(), stateGetAirspeed_f(),
                                           PRECISION_LANDING_MIN_AIRSPEED);
}

static void release_controls(void)
{
  precision_landing_brake_fraction = 0.f;
  commands[COMMAND_BRAKE] = 0;
  if (roll_limit_owned) {
    h_ctl_roll_max_setpoint = saved_roll_limit;
    roll_limit_owned = false;
  }
}

static bool geometry_unchanged(void)
{
  /* A waypoint edit or reference-frame reset invalidates the captured slope and axes together.
   * Do not silently combine new waypoints with old geometry during an established final. */
    bool zone_unchanged = true;
    for (uint8_t corner = 0; landing_zone_configured && corner < 4; corner++) {
      zone_unchanged = zone_unchanged && landing_zone_wp[corner] < NB_WAYPOINT
        && WaypointX(landing_zone_wp[corner]) == landing_zone_east[corner]
        && WaypointY(landing_zone_wp[corner]) == landing_zone_north[corner];
    }
    return precision_landing_ready && zone_unchanged
      && approach_wp < NB_WAYPOINT && touchdown_wp < NB_WAYPOINT
         && WaypointX(approach_wp) == approach_east && WaypointY(approach_wp) == approach_north
         && WaypointAlt(approach_wp) == approach_altitude
         && WaypointX(touchdown_wp) == touchdown_east && WaypointY(touchdown_wp) == touchdown_north
         && WaypointAlt(touchdown_wp) == touchdown_altitude;
}

static bool point_inside_landing_zone(float east, float north)
{
  bool inside = false;
  uint8_t previous = 3;
  for (uint8_t corner = 0; corner < 4; previous = corner++) {
    const float north_delta = landing_zone_north[previous] - landing_zone_north[corner];
    if ((landing_zone_north[corner] > north) != (landing_zone_north[previous] > north)
        && fabsf(north_delta) > 1e-6f
        && east < (landing_zone_east[previous] - landing_zone_east[corner])
                  * (north - landing_zone_north[corner]) / north_delta
                  + landing_zone_east[corner]) {
      inside = !inside;
    }
  }
  return inside;
}

static bool touchdown_prediction_rejected(float longitudinal_error, float lateral_error)
{
  if (!isfinite(longitudinal_error) || !isfinite(lateral_error)
      || fabsf(lateral_error) > PRECISION_LANDING_MAX_CROSS_TRACK) {
    return true;
  }
  if (!landing_zone_configured) {
    return precision_landing_prediction_rejected(longitudinal_error, lateral_error,
             PRECISION_LANDING_MAX_LONG_ERROR, PRECISION_LANDING_MAX_CROSS_TRACK);
  }
  const float predicted_east = touchdown_east - longitudinal_error * final_unit_east
                               - lateral_error * final_unit_north;
  const float predicted_north = touchdown_north - longitudinal_error * final_unit_north
                                + lateral_error * final_unit_east;
  return !isfinite(predicted_east) || !isfinite(predicted_north)
         || !point_inside_landing_zone(predicted_east, predicted_north);
}

static void cancel_invalid_geometry(void)
{
  const float age = get_sys_time_float() - agl_measurement_time;
  const bool fresh = agl_dist_valid && isfinite(agl_dist_value_filtered) && agl_dist_value_filtered >= 0.f
                     && age >= 0.f && age < PRECISION_LANDING_AGL_TIMEOUT;
  const float height = fresh ? agl_dist_value_filtered
                            : (precision_landing_ready ? GetPosAlt() - touchdown_altitude : NAN);
  const bool final_active = precision_landing_ready || landing_committed;
  if (final_active && (landing_committed || !isfinite(height)
                      || precision_landing_should_commit(height, PRECISION_LANDING_ABORT_AGL))) {
    release_controls();
    precision_landing_ready = false;
    landing_committed = true;
    landing_phase = PRECISION_LANDING_PHASE_FLARE_COMMITTED;
    precision_landing_commit_flare = true;
    precision_landing_abort = false;
    return;
  }
  precision_landing_stop();
  precision_landing_cancelled = true;
  landing_phase = PRECISION_LANDING_PHASE_CANCELLED;
  precision_landing_abort = true;
}

bool precision_landing_parameters_valid(void)
{
  return isfinite(precision_landing_approach_airspeed)
         && precision_landing_approach_airspeed >= PRECISION_LANDING_MIN_AIRSPEED
         && precision_landing_approach_airspeed <= 12.f
         && isfinite(precision_landing_final_height)
         && precision_landing_final_height >= 10.f && precision_landing_final_height <= 25.f
         && isfinite(precision_landing_brake_agl)
         && precision_landing_brake_agl >= PRECISION_LANDING_ABORT_AGL
         && precision_landing_brake_agl <= PRECISION_LANDING_BRAKE_ENABLE_AGL
         && isfinite(precision_landing_flare_agl)
         && precision_landing_flare_agl >= 0.5f
         && precision_landing_flare_agl <= PRECISION_LANDING_ABORT_AGL
         && isfinite(precision_landing_aim_before_td)
         && precision_landing_aim_before_td >= 2.f
         && precision_landing_aim_before_td <= PRECISION_LANDING_MAX_AIM_DISTANCE
         && isfinite(precision_landing_stop_distance)
         && precision_landing_stop_distance >= 0.f
         && precision_landing_stop_distance <= PRECISION_LANDING_MAX_AIM_DISTANCE
         && isfinite(precision_landing_touchdown_pitch)
         && precision_landing_touchdown_pitch >= -5.f && precision_landing_touchdown_pitch <= 10.f
         && isfinite(precision_landing_flare_brake)
         && precision_landing_flare_brake >= 0.f
         && precision_landing_flare_brake <= fminf(PRECISION_LANDING_MAX_BRAKE, 0.75f)
         && precision_landing_max_retries <= 5;
}

bool precision_landing_entry_valid(uint8_t af_wp, uint8_t td_wp, float radius)
{
  if (af_wp >= NB_WAYPOINT || td_wp >= NB_WAYPOINT
      || !isfinite(precision_landing_final_height) || precision_landing_final_height <= 0.f
      || !isfinite(radius) || fabsf(radius) < 1.f) {
    return false;
  }
  const float east = WaypointX(td_wp) - WaypointX(af_wp);
  const float north = WaypointY(td_wp) - WaypointY(af_wp);
  const float length = hypotf(east, north);
  const float altitude = WaypointAlt(td_wp) + precision_landing_final_height;
  if (!isfinite(length) || length <= 1.f || !isfinite(east * east + north * north)
      || !isfinite(radius * radius) || !isfinite(altitude) || altitude <= WaypointAlt(td_wp)) {
    return false;
  }
  const float baseleg_east = WaypointX(af_wp) + north / length * radius;
  const float baseleg_north = WaypointY(af_wp) - east / length * radius;
  return isfinite(baseleg_east) && isfinite(baseleg_north);
}

void precision_landing_start(void)
{
  bench_active = false;
  release_controls();
  landing_active = autopilot_get_mode() == AP_MODE_AUTO2;
  precision_landing_cancelled = !landing_active;
  landing_committed = false;
  precision_landing_ready = false;
  precision_landing_abort = false;
  precision_landing_commit_flare = false;
  landing_phase = landing_active ? PRECISION_LANDING_PHASE_APPROACH
                                 : PRECISION_LANDING_PHASE_CANCELLED;
}

void precision_landing_release(void)
{
  /* Flight-plan stage exits release outputs without reopening the powered-abort option.
   * Only an explicit new landing start resets the commitment latch. */
  release_controls();
}

bool precision_landing_is_active(void)
{
  return landing_active && !precision_landing_cancelled && autopilot_get_mode() == AP_MODE_AUTO2;
}

static bool bench_interlocks_ok(void)
{
  const struct EnuCoor_f *speed = stateGetSpeedEnu_f();
  const float age = get_sys_time_float() - agl_measurement_time;
  return autopilot_get_mode() == AP_MODE_AUTO2 && autopilot.kill_throttle
         && !autopilot.launch && autopilot.flight_time == 0
         && agl_dist_valid && isfinite(agl_dist_value_filtered)
         && agl_dist_value_filtered >= 0.f && agl_dist_value_filtered <= 0.5f
         && age >= 0.f && age < PRECISION_LANDING_AGL_TIMEOUT
         && isfinite(speed->x) && isfinite(speed->y) && isfinite(speed->z)
         && hypotf(speed->x, speed->y) <= 0.5f && fabsf(speed->z) <= 0.2f;
}

void precision_landing_bench_start(void)
{
  precision_landing_stop();
  bench_active = bench_interlocks_ok();
  landing_phase = bench_active ? PRECISION_LANDING_PHASE_BENCH : PRECISION_LANDING_PHASE_IDLE;
}

bool precision_landing_bench_run(void)
{
  if (!bench_active || !bench_interlocks_ok()) {
    bench_active = false;
    landing_phase = PRECISION_LANDING_PHASE_IDLE;
    release_controls();
    return false;
  }
  commands[COMMAND_BRAKE] = -MAX_PPRZ;
  return true;
}

void precision_landing_check_abort(void)
{
  if (!precision_landing_is_active()) {
    precision_landing_cancelled = true;
    landing_phase = PRECISION_LANDING_PHASE_CANCELLED;
    release_controls();
    return;
  }
  if (landing_phase == PRECISION_LANDING_PHASE_GO_AROUND_COMMITTED) {
    precision_landing_commit_flare = false;
    precision_landing_abort = true;
    release_controls();
    return;
  }
  if (landing_phase == PRECISION_LANDING_PHASE_FLARE_COMMITTED) {
    precision_landing_commit_flare = true;
    precision_landing_abort = false;
    release_controls();
    return;
  }
  const float age = get_sys_time_float() - agl_measurement_time;
  const bool fresh = agl_dist_valid && isfinite(agl_dist_value_filtered) && agl_dist_value_filtered >= 0.f
                     && age >= 0.f && age < PRECISION_LANDING_AGL_TIMEOUT;
  const float height = fresh ? agl_dist_value_filtered
                            : (precision_landing_ready ? GetPosAlt() - touchdown_altitude : NAN);
  landing_committed |= !isfinite(height) || precision_landing_should_commit(height, PRECISION_LANDING_ABORT_AGL);
  landing_phase = landing_committed ? PRECISION_LANDING_PHASE_FLARE_COMMITTED
                                    : PRECISION_LANDING_PHASE_GO_AROUND_COMMITTED;
  precision_landing_commit_flare = landing_committed;
  precision_landing_abort = !landing_committed;
  release_controls();
}

void precision_landing_on_mode_change(uint8_t mode)
{
  /* Clear demand even outside an active final: stale crow must not survive a mode transition.
   * Returning to AUTO2 must not resume landing without an explicit landing entry. */
  if (mode != autopilot_get_mode()) {
    precision_landing_cancelled = true;
    precision_landing_stop();
  }
}

static void reject_approach(void)
{
  landing_phase = landing_committed ? PRECISION_LANDING_PHASE_FLARE_COMMITTED
                                    : PRECISION_LANDING_PHASE_GO_AROUND_COMMITTED;
  precision_landing_commit_flare = landing_committed;
  precision_landing_abort = !landing_committed;
  release_controls();
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/**
 * @brief Packs precision landing diagnostic data into a DEBUG_VECT telemetry message.
 *
 * @details
 * Transmits a 11-element float vector containing module telemetry over Paparazzi datalink:
 * [0] Uptime (s), [1] Filtered AGL (m), [2] Range sink rate (m/s), [3] Remaining distance (m),
 * [4] Cross-track (m), [5] Predicted long error (m), [6] Predicted cross error (m),
 * [7] Brake fraction, [8] AGL fresh flag, [9] Abort flag, [10] Commit flare flag.
 */
static void send_precision_landing(struct transport_tx *trans, struct link_device *dev)
{
  char name[] = "precision_landing";
  float values[] = {
    get_sys_time_float(), agl_dist_value_filtered, range_sink_rate_mps,
    precision_landing_remaining_m, precision_landing_cross_track_m,
    precision_landing_predicted_error_m, precision_landing_predicted_cross_track_m,
    precision_landing_brake_fraction, precision_landing_agl_fresh,
    precision_landing_abort, precision_landing_commit_flare
  };
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID, sizeof(name) - 1, name,
                          sizeof(values) / sizeof(values[0]), values);
}
#endif

void precision_landing_init(void)
{
#if PERIODIC_TELEMETRY
  /* Registers periodic telemetry callback under DEBUG_VECT message ID. */
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_precision_landing);
#endif
}

void precision_landing_stop(void)
{
  release_controls();
  bench_active = false;
  landing_active = false;
  precision_landing_ready = false;
  landing_phase = precision_landing_cancelled ? PRECISION_LANDING_PHASE_CANCELLED
                                              : PRECISION_LANDING_PHASE_IDLE;
}

void precision_landing_flare(float brake_fraction)
{
  if (!landing_active || precision_landing_cancelled || autopilot_get_mode() != AP_MODE_AUTO2) {
    precision_landing_cancelled = true;
    return;
  }
  landing_active = true;
  landing_committed = true;
  landing_phase = PRECISION_LANDING_PHASE_FLARE_COMMITTED;
  if (!roll_limit_owned) {
    saved_roll_limit = h_ctl_roll_max_setpoint;
    roll_limit_owned = true;
  }
  /* Defensive check against NaN inputs from uninitialized flight-plan variables. */
  if (!isfinite(brake_fraction) || !airspeed_safe()) {
    brake_fraction = 0.f;
  }
  Bound(brake_fraction, 0.f, fminf(fmaxf(PRECISION_LANDING_MAX_BRAKE, 0.f), 1.f));
  precision_landing_brake_fraction = brake_fraction;
  
  /* Paparazzi mixer convention: negative command value corresponds to upward spoileron deflection. */
  commands[COMMAND_BRAKE] = (pprz_t)(-brake_fraction * MAX_PPRZ);
  
  /* Restrict maximum roll setpoint during touchdown flare (e.g. 8 degrees). Belly-landing foam airframes
   * have low ground clearance; excessive bank during contact risks catching a wingtip and cartwheeling. */
  h_ctl_roll_max_setpoint = fminf(saved_roll_limit, PRECISION_LANDING_FLARE_MAX_ROLL);
}

void precision_landing_flare_run(void)
{
  if (!landing_active || precision_landing_cancelled || autopilot_get_mode() != AP_MODE_AUTO2) {
    precision_landing_cancelled = true;
    return;
  }
  precision_landing_flare(precision_landing_flare_brake);
  float pitch = isfinite(precision_landing_touchdown_pitch)
                ? RadOfDeg(precision_landing_touchdown_pitch) : 0.f;
  Bound(pitch, H_CTL_PITCH_MIN_SETPOINT, H_CTL_PITCH_MAX_SETPOINT);
  /* Own the local throttle setpoint, not the global kill latch, so pilot takeover retains authority. */
  NavVerticalAutoThrottleMode(pitch);
  NavVerticalThrottleMode(0);
  NavAttitude(0.f);
#if USE_GPS
  const struct EnuCoor_f *position = stateGetPositionEnu_f();
  if (geometry_unchanged() && GpsFixValid() && isfinite(position->x) && isfinite(position->y)
      && !NavApproachingFrom(touchdown_wp, approach_wp, 0)) {
    NavSegment(approach_wp, touchdown_wp);
  }
#endif
}

void precision_landing_setup(uint8_t af_wp, uint8_t td_wp)
{
  landing_zone_configured = false;
  if (!landing_active || precision_landing_cancelled || autopilot_get_mode() != AP_MODE_AUTO2) {
    precision_landing_cancelled = true;
    landing_phase = PRECISION_LANDING_PHASE_CANCELLED;
    return;
  }
  landing_active = true;
  landing_phase = PRECISION_LANDING_PHASE_APPROACH;
  release_controls();
  if (af_wp >= NB_WAYPOINT || td_wp >= NB_WAYPOINT) {
    cancel_invalid_geometry();
    return;
  }
  const float east = WaypointX(td_wp) - WaypointX(af_wp);
  const float north = WaypointY(td_wp) - WaypointY(af_wp);
  const float length = hypotf(east, north);
  const float altitude_difference = WaypointAlt(af_wp) - WaypointAlt(td_wp);
  const float slope = length > 1.f ? altitude_difference / length : NAN;

  /* Reject degenerate or non-descending geometry before navigation can consume its slope.
   * Invalid geometry requests flare if already committed, rather than a powered recovery. */
  precision_landing_ready = isfinite(length) && length > 1.f && isfinite(precision_landing_brake_agl)
                           && isfinite(precision_landing_aim_before_td) && isfinite(WaypointAlt(af_wp))
                           && isfinite(WaypointAlt(td_wp)) && isfinite(altitude_difference)
                           && isfinite(slope) && slope > 0.f;
  touchdown_wp = td_wp;
  approach_wp = af_wp;
  if (precision_landing_ready) {
    final_unit_east = east / length;
    final_unit_north = north / length;
    
    final_slope = slope;
    approach_east = WaypointX(af_wp);
    approach_north = WaypointY(af_wp);
    approach_altitude = WaypointAlt(af_wp);
    touchdown_east = WaypointX(td_wp);
    touchdown_north = WaypointY(td_wp);
    touchdown_altitude = WaypointAlt(td_wp);
  } else {
    final_unit_east = 0.f;
    final_unit_north = 0.f;
    cancel_invalid_geometry();
    return;
  }

  precision_landing_remaining_m = length;
  precision_landing_cross_track_m = 0.f;
  precision_landing_predicted_cross_track_m = 0.f;
  precision_landing_predicted_error_m = length;
  brake_enable_agl_m = precision_landing_brake_agl;
  Bound(brake_enable_agl_m, PRECISION_LANDING_ABORT_AGL, PRECISION_LANDING_BRAKE_ENABLE_AGL);
  aim_before_td_m = precision_landing_aim_before_td;
  Bound(aim_before_td_m, 0.f, PRECISION_LANDING_MAX_AIM_DISTANCE);
  stop_distance_m = precision_landing_stop_distance;
  Bound(stop_distance_m, 0.f, PRECISION_LANDING_MAX_AIM_DISTANCE);
  precision_landing_agl_fresh = false;
  rejection_since = -1.f;
  previous_agl_m = 0.f;
  previous_agl_time = 0.f;
  range_sink_rate_mps = 0.f;
  range_rate_samples = 0;
  precision_landing_abort = !precision_landing_ready;
  precision_landing_commit_flare = false;
  landing_committed = false;
}

void precision_landing_setup_zone(uint8_t af_wp, uint8_t td_wp, uint8_t corner_1_wp,
                                  uint8_t corner_2_wp, uint8_t corner_3_wp, uint8_t corner_4_wp)
{
  precision_landing_setup(af_wp, td_wp);
  if (!precision_landing_ready) {
    return;
  }
  const uint8_t corners[4] = {corner_1_wp, corner_2_wp, corner_3_wp, corner_4_wp};
  for (uint8_t corner = 0; corner < 4; corner++) {
    if (corners[corner] >= NB_WAYPOINT || !isfinite(WaypointX(corners[corner]))
        || !isfinite(WaypointY(corners[corner]))) {
      cancel_invalid_geometry();
      return;
    }
    landing_zone_wp[corner] = corners[corner];
    landing_zone_east[corner] = WaypointX(corners[corner]);
    landing_zone_north[corner] = WaypointY(corners[corner]);
  }
  landing_zone_configured = point_inside_landing_zone(touchdown_east, touchdown_north);
  if (!landing_zone_configured) {
    cancel_invalid_geometry();
  }
}

void precision_landing_run(void)
{
  if (!landing_active || precision_landing_cancelled || autopilot_get_mode() != AP_MODE_AUTO2) {
    precision_landing_cancelled = true;
    landing_phase = PRECISION_LANDING_PHASE_CANCELLED;
    return;
  }
  if (landing_phase == PRECISION_LANDING_PHASE_GO_AROUND_COMMITTED) {
    precision_landing_commit_flare = false;
    precision_landing_abort = true;
    release_controls();
    return;
  }
  if (landing_phase == PRECISION_LANDING_PHASE_FLARE_COMMITTED) {
    precision_landing_commit_flare = true;
    precision_landing_abort = false;
    release_controls();
    return;
  }
  if (!geometry_unchanged()) {
    cancel_invalid_geometry();
    return;
  }
  const float now = get_sys_time_float();
  const float measurement_age = now - agl_measurement_time;
  precision_landing_agl_fresh = agl_dist_valid
                              && isfinite(agl_dist_value_filtered) && agl_dist_value_filtered >= 0.f
                              && measurement_age >= 0.f && measurement_age < PRECISION_LANDING_AGL_TIMEOUT;
  const float barometric_agl = precision_landing_ready ? GetPosAlt() - WaypointAlt(touchdown_wp) : NAN;
  const float decision_height = precision_landing_agl_fresh ? agl_dist_value_filtered : barometric_agl;
  /* Latch before evaluating faults: height noise or later sensor loss must not re-enable a
   * powered abort after entering the low-altitude commitment region. This safety latch is
   * separate from normal flare initiation at precision_landing_flare_agl. */
  landing_committed |= precision_landing_should_commit(decision_height, PRECISION_LANDING_ABORT_AGL);
  landing_committed |= !isfinite(decision_height);
  if (precision_landing_agl_fresh
      && precision_landing_should_commit(agl_dist_value_filtered, precision_landing_flare_agl)) {
    landing_phase = PRECISION_LANDING_PHASE_FLARE_COMMITTED;
    precision_landing_abort = false;
    precision_landing_commit_flare = true;
    release_controls();
    return;
  }
  precision_landing_abort = false;
  precision_landing_commit_flare = false;

  if (!precision_landing_ready) {
    reject_approach();
    return;
  }

  const struct EnuCoor_f *position = stateGetPositionEnu_f();
  const struct EnuCoor_f *speed = stateGetSpeedEnu_f();
#if USE_GPS
  if (!GpsFixValid()) {
    reject_approach();
    return;
  }
#endif
  
  /* Fail-safe floating-point verification: if state estimation produces NaN or Inf (e.g. after sensor failure),
   * commit to flare if close to ground or abort if airborne to protect flight controls from corrupt values. */
  if (!isfinite(position->x) || !isfinite(position->y) || !isfinite(speed->x)
      || !isfinite(speed->y) || !isfinite(speed->z) || !isfinite(barometric_agl)) {
    reject_approach();
    return;
  }
  
  /* 2D Coordinate Transformation into Runway Frame */
  const float to_td_east = WaypointX(touchdown_wp) - position->x;
  const float to_td_north = WaypointY(touchdown_wp) - position->y;
  
  /* Ground velocity includes head/tailwind travel and crosswind drift without adding wind twice.
   * Airspeed remains a separate brake-safety gate; groundspeed cannot establish stall margin.
   * This projects first contact, not the subsequent surface-dependent slide to final rest. */
  const float along_speed = speed->x * final_unit_east + speed->y * final_unit_north;
  const float cross_speed = speed->x * final_unit_north - speed->y * final_unit_east;

  precision_landing_remaining_m = to_td_east * final_unit_east + to_td_north * final_unit_north;
  precision_landing_cross_track_m = to_td_east * final_unit_north - to_td_north * final_unit_east;
  if (!isfinite(along_speed) || !isfinite(cross_speed) || !isfinite(precision_landing_remaining_m)
      || !isfinite(precision_landing_cross_track_m)) {
    reject_approach();
    return;
  }
  
  /* Rangefinder Freshness & Differentiated Sink Rate Calculation */
  if (precision_landing_agl_fresh && agl_measurement_time > previous_agl_time) {
    if (previous_agl_time > 0.f && agl_measurement_time - previous_agl_time < PRECISION_LANDING_AGL_TIMEOUT) {
      const float dt = agl_measurement_time - previous_agl_time;
      const float measured_sink_rate = (previous_agl_m - agl_dist_value_filtered) / dt;
      
      /* EWMA low-pass filter: alpha = dt / (tau + dt).
       * With tau = 0.15s, this filters out rangefinder measurement noise (e.g. grass/ground texture variation)
       * while remaining responsive enough to track vertical deceleration during flare. */
      const float alpha = dt / (PRECISION_LANDING_AGL_RATE_TAU + dt);
      range_sink_rate_mps += alpha * (measured_sink_rate - range_sink_rate_mps);
      if (range_rate_samples < UINT8_MAX) {
        range_rate_samples++;
      }
    } else {
      /* Reset filter state to fused state vertical speed if a measurement gap occurred. */
      range_sink_rate_mps = -speed->z;
      range_rate_samples = 0;
    }
    previous_agl_m = agl_dist_value_filtered;
    previous_agl_time = agl_measurement_time;
  }

  if (!airspeed_safe() || along_speed < 1.f || precision_landing_remaining_m <= 0.f) {
    reject_approach();
    return;
  }

  /* --- UPPER ALTITUDE BAND & BAROMETRIC FALLBACK --- */
  if (!precision_landing_agl_fresh || agl_dist_value_filtered > brake_enable_agl_m) {
    if (barometric_agl > brake_enable_agl_m || precision_landing_agl_fresh) {
      /* Compute barometric prediction above brake_agl to allow early, smooth dissipation of excess potential energy. */
      const struct PrecisionLandingPrediction prediction = precision_landing_predict(precision_landing_remaining_m,
          precision_landing_cross_track_m, along_speed, cross_speed, barometric_agl, -speed->z,
          PRECISION_LANDING_MIN_SINK_RATE);
      precision_landing_predicted_error_m = prediction.longitudinal_error_m - stop_distance_m;
      precision_landing_predicted_cross_track_m = prediction.cross_track_error_m;
      const float approach_corridor = PRECISION_LANDING_MAX_CROSS_TRACK
          + PRECISION_LANDING_APPROACH_CORRIDOR_SLOPE * fmaxf(decision_height - brake_enable_agl_m, 0.f);
      if (!isfinite(prediction.longitudinal_error_m)
          || precision_landing_lateral_approach_rejected(precision_landing_cross_track_m,
            prediction.cross_track_error_m, approach_corridor, PRECISION_LANDING_MAX_CROSS_TRACK)) {
        if (rejection_since < 0.f) {
          rejection_since = now;
        }
        if (now - rejection_since >= PRECISION_LANDING_REJECT_DELAY) {
          reject_approach();
          return;
        }
      } else {
        rejection_since = -1.f;
      }
      float brake = 0.f;
      if (isfinite(barometric_agl) && isfinite(prediction.longitudinal_error_m)
          && airspeed_safe()
          && along_speed > 1.f && -speed->z > PRECISION_LANDING_MIN_SINK_RATE) {
        const float desired_height = fmaxf(precision_landing_remaining_m - aim_before_td_m, 0.f) * final_slope;
        brake = (barometric_agl - desired_height) * 0.3f;
        
        /* Cap upper band brake at 50% max to ensure sufficient pitch/roll control authority remains. */
        Bound(brake, 0.f, fminf(fmaxf(PRECISION_LANDING_UPPER_MAX_BRAKE, 0.f),
                  fminf(fmaxf(PRECISION_LANDING_MAX_BRAKE, 0.f), 1.f)));
      }
      precision_landing_brake_fraction = brake;
      commands[COMMAND_BRAKE] = (pprz_t)(-brake * MAX_PPRZ);
      return;
    }
    
    /* If below brake_agl but rangefinder data is missing, commit to flare if below 2.5m, otherwise abort. */
    reject_approach();
    return;
  }

  /* --- CLOSE-RANGE PRECISION DECISION BAND (RANGEFINDER ACTIVE) --- */
  /* Blend the independent height/rate sources through the decision band. A hard source
   * switch at brake_agl previously changed the projected contact by tens of metres in one cycle. */
  const struct PrecisionLandingPrediction fused_prediction = precision_landing_predict(
      precision_landing_remaining_m, precision_landing_cross_track_m, along_speed, cross_speed,
      barometric_agl, -speed->z, PRECISION_LANDING_MIN_SINK_RATE);
  const struct PrecisionLandingPrediction range_prediction = precision_landing_predict(precision_landing_remaining_m,
      precision_landing_cross_track_m, along_speed, cross_speed, agl_dist_value_filtered,
      range_rate_samples >= 4 ? range_sink_rate_mps : -speed->z,
      PRECISION_LANDING_MIN_SINK_RATE);
  float range_weight = (brake_enable_agl_m - agl_dist_value_filtered)
                       / fmaxf(brake_enable_agl_m - PRECISION_LANDING_ABORT_AGL, 0.1f);
  Bound(range_weight, 0.f, 1.f);
  const struct PrecisionLandingPrediction prediction = {
    .longitudinal_error_m = fused_prediction.longitudinal_error_m
                            + range_weight * (range_prediction.longitudinal_error_m
                                              - fused_prediction.longitudinal_error_m),
    .cross_track_error_m = fused_prediction.cross_track_error_m
                           + range_weight * (range_prediction.cross_track_error_m
                                             - fused_prediction.cross_track_error_m)
  };
  precision_landing_predicted_error_m = prediction.longitudinal_error_m - stop_distance_m;
  precision_landing_predicted_cross_track_m = prediction.cross_track_error_m;

  /* --- SAFETY BOUNDARY & PERSISTENCE EVALUATION --- */
  if (agl_dist_value_filtered > PRECISION_LANDING_ABORT_AGL) {
    /* Immediate abort if airspeed drops below stall safety margin or along-track groundspeed halts. */
    precision_landing_abort = !airspeed_safe() || along_speed < 1.f;
    
    /* Require a persistent box violation before rejecting a close-range approach. */
    if (touchdown_prediction_rejected(precision_landing_predicted_error_m,
                                      precision_landing_predicted_cross_track_m)) {
      if (rejection_since < 0.f) {
        rejection_since = now;
      }
      precision_landing_abort |= now - rejection_since >= PRECISION_LANDING_REJECT_DELAY;
    } else {
      rejection_since = -1.f;
    }
  }

  if (precision_landing_abort) {
    reject_approach();
    return;
  }

  /* --- PROPORTIONAL CROW BRAKE CALCULATION --- */
  /* Positive prediction error is upstream of TD, so negative error means the predicted final stop passes TD.
   * Do not fade demand solely with altitude: that previously retracted crow while overshooting,
   * leaving the rate-limited actuators to extend again at flare. Airspeed gating and caps remain. */
  float brake = 0.f;
  if (!precision_landing_abort && airspeed_safe()
      && agl_dist_value_filtered < brake_enable_agl_m
      && precision_landing_predicted_error_m < 0.f) {
    brake = -precision_landing_predicted_error_m * PRECISION_LANDING_BRAKE_GAIN;
            
    brake = isfinite(brake) ? brake : 0.f;
    Bound(brake, 0.f, fminf(fmaxf(PRECISION_LANDING_MAX_BRAKE, 0.f), 1.f));
  }

  precision_landing_brake_fraction = brake;
  commands[COMMAND_BRAKE] = (pprz_t)(-brake * MAX_PPRZ);
}

void precision_landing_glide(void)
{
  if (!precision_landing_ready || precision_landing_abort || precision_landing_commit_flare
      || precision_landing_cancelled || autopilot_get_mode() != AP_MODE_AUTO2) {
    return;
  }
  if (!geometry_unchanged()) {
    cancel_invalid_geometry();
    return;
  }
  const struct EnuCoor_f *speed = stateGetSpeedEnu_f();
  const float along_speed = speed->x * final_unit_east + speed->y * final_unit_north;
  
  /* Dynamically update flight controller vertical altitude target along the nominal glide slope slope.
   * Shifting the target height upstream by aim_before_td_m ensures the unbraked trajectory targets
   * the upstream box entry rather than the box center, absorbing flare float. */
  const float approach_height = approach_altitude - touchdown_altitude;
  const float uncapped_height = fmaxf(precision_landing_remaining_m - aim_before_td_m, 0.f) * final_slope;
  const float height = fminf(uncapped_height, approach_height);
  const float altitude = WaypointAlt(touchdown_wp) + height;
  const float preclimb = uncapped_height < approach_height ? -fmaxf(along_speed, 0.f) * final_slope : 0.f;
  if (!isfinite(along_speed) || !isfinite(approach_height) || !isfinite(uncapped_height)
      || !isfinite(height) || !isfinite(altitude) || !isfinite(preclimb)) {
    reject_approach();
    return;
  }
  flight_altitude = altitude;
  NavVerticalAltitudeMode(altitude, preclimb);
}
