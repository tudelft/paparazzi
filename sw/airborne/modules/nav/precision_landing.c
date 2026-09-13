/**
 * @file modules/nav/precision_landing.c
 * @brief Fixed-Wing Precision Landing Module Implementation
 * @author TU Delft IMAV 2026 Team / Paparazzi UAV
 *
 * @details
 * Implementation of autonomous precision landing for fixed-wing aircraft using
 * rangefinder-based touchdown prediction, dual-band predictive crow braking,
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
 *      Applies spoileron drag proportionally to predicted overshoot beyond `aim_before_td`.
 *      Linear blending ramps brake demand down near ground ($0.5\,\text{m}$ AGL) to prevent
 *      abrupt aerodynamic pitch transients during flare transition.
 *
 * 5. **Safety Persistence & Abort State Machine**:
 *    - Aborts if airspeed drops below `MIN_AIRSPEED` (protecting stall margin).
 *    - Aborts if lateral drift prediction exceeds \f$1.2\,\text{m}\f$ (retaining \f$0.3\,\text{m}\f$ margin inside the official \f$1.5\,\text{m}\f$ half-box).
 *    - Aborts if longitudinal prediction breaches bounds (\f$> 8.0\,\text{m}\f$ short or \f$> 8.0\,\text{m}\f$ overshoot with \f$\ge 95\%\f$ brake saturation).
 *    - Evaluates prediction rejection across a \f$0.2\,\text{s}\f$ persistence timer to prevent single-sample sensor noise from triggering premature go-arounds.
 *    - Below commit height (\f$2.5\,\text{m}\f$), disallows powered go-around and forces flare landing.
 */

#include "modules/nav/precision_landing.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/core/commands.h"
#include "modules/sonar/agl_dist.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "state.h"
#include "mcu_periph/sys_time.h"

#include <math.h>

/* --- Default Configuration Parameters (Overridable via Airframe XML) --- */

/** Minimum safe airspeed during approach (m/s). Below this, braking is disengaged and abort triggered. */
#ifndef PRECISION_LANDING_MIN_AIRSPEED
#define PRECISION_LANDING_MIN_AIRSPEED 8.2f
#endif

/** Maximum allowed predicted cross-track error at touchdown (m). Leaves 0.3m margin inside 1.5m box edge. */
#ifndef PRECISION_LANDING_MAX_CROSS_TRACK
#define PRECISION_LANDING_MAX_CROSS_TRACK 1.2f
#endif

/** Maximum allowed longitudinal short/overshoot prediction error before aborting (m). */
#ifndef PRECISION_LANDING_MAX_LONG_ERROR
#define PRECISION_LANDING_MAX_LONG_ERROR 8.0f
#endif

/** Timeout threshold for rangefinder sample freshness (seconds). Stale data switches to barometric fallback. */
#ifndef PRECISION_LANDING_AGL_TIMEOUT
#define PRECISION_LANDING_AGL_TIMEOUT 0.25f
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

/** Maximum roll angle setpoint during touchdown flare (radians). 8 deg prevents wingtip strike on belly landers. */
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

/* --- Global Module State Definitions --- */

float precision_landing_remaining_m;             /**< Along-track distance to TD (m) */
float precision_landing_cross_track_m;            /**< Lateral cross-track offset from runway centerline (m) */
float precision_landing_predicted_cross_track_m;  /**< Projected cross-track displacement at touchdown (m) */
float precision_landing_predicted_error_m;        /**< Projected longitudinal offset from TD at touchdown (m) */
float precision_landing_brake_fraction;           /**< Currently commanded crow brake demand (0.0 to 1.0) */
bool precision_landing_agl_fresh;                 /**< Fresh rangefinder measurement available */
bool precision_landing_abort;                     /**< True if approach safety boundary violated */
bool precision_landing_commit_flare;              /**< True if below commit height (forces flare) */

/* --- Module Private Static Variables --- */

static float final_unit_east;      /**< Unit vector East component along final approach direction */
static float final_unit_north;     /**< Unit vector North component along final approach direction */
static float final_slope;          /**< Nominal runway glide slope angle ratio (dz / dx) */
static float brake_enable_agl_m;   /**< Active AGL threshold for close-range predictive braking (m) */
static float aim_before_td_m;      /**< Upstream target distance from TD for zero-brake nominal trajectory (m) */
static float previous_agl_m;       /**< Previous AGL sample stored for numerical differentiation (m) */
static float previous_agl_time;    /**< Timestamp of previous AGL sample (s) */
static float range_sink_rate_mps;  /**< Filtered vertical sink rate derived from rangefinder differentiation (m/s) */
static uint8_t range_rate_samples; /**< Number of consecutive valid range-rate samples accumulated */
static uint8_t touchdown_wp;       /**< Waypoint index of target touchdown location */
static bool precision_landing_ready;/**< Flag confirming valid runway vector setup */
static float rejection_since;      /**< Timestamp when prediction rejection condition first triggered (s) */

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
  /* Disengage crow brake commands and restore standard flight roll setpoint limit immediately upon
   * exiting landing blocks. Restoring roll authority is critical to prevent restricted roll control
   * during subsequent go-around or standby maneuvers. */
  precision_landing_brake_fraction = 0.f;
  commands[COMMAND_BRAKE] = 0;
  h_ctl_roll_max_setpoint = H_CTL_ROLL_MAX_SETPOINT;
}

void precision_landing_flare(float brake_fraction)
{
  /* Defensive check against NaN inputs from uninitialized flight-plan variables. */
  if (!isfinite(brake_fraction)) {
    brake_fraction = 0.f;
  }
  Bound(brake_fraction, 0.f, PRECISION_LANDING_MAX_BRAKE);
  precision_landing_brake_fraction = brake_fraction;
  
  /* Paparazzi mixer convention: negative command value corresponds to upward spoileron deflection. */
  commands[COMMAND_BRAKE] = (pprz_t)(-brake_fraction * MAX_PPRZ);
  
  /* Restrict maximum roll setpoint during touchdown flare (e.g. 8 degrees). Belly-landing foam airframes
   * have low ground clearance; excessive bank during contact risks catching a wingtip and cartwheeling. */
  h_ctl_roll_max_setpoint = PRECISION_LANDING_FLARE_MAX_ROLL;
}

void precision_landing_setup(uint8_t af_wp, uint8_t td_wp, float brake_enable_agl, float aim_before_td)
{
  const float east = WaypointX(td_wp) - WaypointX(af_wp);
  const float north = WaypointY(td_wp) - WaypointY(af_wp);
  const float length = sqrtf(east * east + north * north);

  /* Verify that touchdown and approach waypoints are physically distinct (>1m apart) and all input
   * floats are finite. Invalid geometry disables module execution and flags immediate abort. */
  precision_landing_ready = isfinite(length) && length > 1.f && isfinite(brake_enable_agl)
                           && isfinite(aim_before_td) && isfinite(WaypointAlt(af_wp))
                           && isfinite(WaypointAlt(td_wp));
  touchdown_wp = td_wp;
  if (precision_landing_ready) {
    final_unit_east = east / length;
    final_unit_north = north / length;
    
    /* Calculate nominal slope dz/dx. Lower-bounded at 0.02 (approx 1.1 deg) to prevent zero division
     * in flat flight plans. */
    final_slope = fmaxf((WaypointAlt(af_wp) - WaypointAlt(td_wp)) / length, 0.02f);
  } else {
    final_unit_east = 0.f;
    final_unit_north = 0.f;
  }

  precision_landing_remaining_m = length;
  precision_landing_cross_track_m = 0.f;
  precision_landing_predicted_cross_track_m = 0.f;
  precision_landing_predicted_error_m = length;
  brake_enable_agl_m = brake_enable_agl;
  Bound(brake_enable_agl_m, PRECISION_LANDING_ABORT_AGL, PRECISION_LANDING_BRAKE_ENABLE_AGL);
  aim_before_td_m = aim_before_td;
  Bound(aim_before_td_m, 0.f, PRECISION_LANDING_MAX_LONG_ERROR);
  precision_landing_agl_fresh = false;
  rejection_since = -1.f;
  previous_agl_m = 0.f;
  previous_agl_time = 0.f;
  range_sink_rate_mps = 0.f;
  range_rate_samples = 0;
  precision_landing_abort = !precision_landing_ready;
  precision_landing_commit_flare = false;
  precision_landing_stop();
}

void precision_landing_run(void)
{
  if (!precision_landing_ready) {
    precision_landing_abort = true;
    precision_landing_stop();
    return;
  }

  const struct EnuCoor_f *position = stateGetPositionEnu_f();
  const struct EnuCoor_f *speed = stateGetSpeedEnu_f();
  
  /* Fail-safe floating-point verification: if state estimation produces NaN or Inf (e.g. after sensor failure),
   * commit to flare if close to ground or abort if airborne to protect flight controls from corrupt values. */
  if (!isfinite(position->x) || !isfinite(position->y) || !isfinite(speed->x)
      || !isfinite(speed->y) || !isfinite(speed->z)) {
    precision_landing_commit_flare = precision_landing_should_commit(GetPosAlt() - WaypointAlt(touchdown_wp),
                                      PRECISION_LANDING_ABORT_AGL);
    precision_landing_abort = !precision_landing_commit_flare;
    precision_landing_stop();
    return;
  }
  
  /* 2D Coordinate Transformation into Runway Frame */
  const float to_td_east = WaypointX(touchdown_wp) - position->x;
  const float to_td_north = WaypointY(touchdown_wp) - position->y;
  
  /* Project velocity vector onto along-track and cross-track unit vectors.
   * Note: along_speed represents forward velocity along the approach line (handles crab natively). */
  const float along_speed = speed->x * final_unit_east + speed->y * final_unit_north;
  const float cross_speed = speed->x * final_unit_north - speed->y * final_unit_east;

  precision_landing_remaining_m = to_td_east * final_unit_east + to_td_north * final_unit_north;
  precision_landing_cross_track_m = to_td_east * final_unit_north - to_td_north * final_unit_east;
  
  /* Rangefinder Freshness & Differentiated Sink Rate Calculation */
  const float now = get_sys_time_float();
  const float measurement_age = now - agl_measurement_time;
  precision_landing_agl_fresh = agl_dist_valid
                                && isfinite(agl_dist_value_filtered) && agl_dist_value_filtered >= 0.f
                                && measurement_age >= 0.f && measurement_age < PRECISION_LANDING_AGL_TIMEOUT;
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

  precision_landing_abort = false;
  precision_landing_commit_flare = false;

  /* --- UPPER ALTITUDE BAND & BAROMETRIC FALLBACK --- */
  if (!precision_landing_agl_fresh || agl_dist_value_filtered > brake_enable_agl_m) {
    const float barometric_agl = GetPosAlt() - WaypointAlt(touchdown_wp);
    if (barometric_agl > brake_enable_agl_m || precision_landing_agl_fresh) {
      /* Compute barometric prediction above brake_agl to allow early, smooth dissipation of excess potential energy. */
      const struct PrecisionLandingPrediction prediction = precision_landing_predict(precision_landing_remaining_m,
          precision_landing_cross_track_m, along_speed, cross_speed, barometric_agl, -speed->z,
          PRECISION_LANDING_MIN_SINK_RATE);
      precision_landing_predicted_error_m = prediction.longitudinal_error_m;
      precision_landing_predicted_cross_track_m = prediction.cross_track_error_m;
      float brake = 0.f;
      if (isfinite(barometric_agl) && isfinite(prediction.longitudinal_error_m)
          && precision_landing_airspeed_safe(stateIsAirspeedValid(), stateGetAirspeed_f(), PRECISION_LANDING_MIN_AIRSPEED)
          && along_speed > 1.f && -speed->z > PRECISION_LANDING_MIN_SINK_RATE) {
        const float desired_height = fmaxf(precision_landing_remaining_m - aim_before_td_m, 0.f) * final_slope;
        brake = (barometric_agl - desired_height) * 0.3f;
        
        /* Cap upper band brake at 50% max to ensure sufficient pitch/roll control authority remains. */
        Bound(brake, 0.f, PRECISION_LANDING_UPPER_MAX_BRAKE);
      }
      precision_landing_brake_fraction = brake;
      commands[COMMAND_BRAKE] = (pprz_t)(-brake * MAX_PPRZ);
      return;
    }
    
    /* If below brake_agl but rangefinder data is missing, commit to flare if below 2.5m, otherwise abort. */
    precision_landing_commit_flare = precision_landing_should_commit(barometric_agl, PRECISION_LANDING_ABORT_AGL);
    precision_landing_abort = !precision_landing_commit_flare;
    precision_landing_stop();
    return;
  }

  /* --- CLOSE-RANGE PRECISION DECISION BAND (RANGEFINDER ACTIVE) --- */
  /* Require at least 4 valid range-rate samples to warm up EWMA filter; use fused -speed->z until then. */
  const struct PrecisionLandingPrediction prediction = precision_landing_predict(precision_landing_remaining_m,
      precision_landing_cross_track_m, along_speed, cross_speed, agl_dist_value_filtered,
      range_rate_samples >= 4 ? range_sink_rate_mps : -speed->z,
      PRECISION_LANDING_MIN_SINK_RATE);
  precision_landing_predicted_error_m = prediction.longitudinal_error_m;
  precision_landing_predicted_cross_track_m = prediction.cross_track_error_m;

  /* --- SAFETY BOUNDARY & PERSISTENCE EVALUATION --- */
  if (agl_dist_value_filtered > PRECISION_LANDING_ABORT_AGL) {
    /* Immediate abort if airspeed drops below stall safety margin or along-track groundspeed halts. */
    precision_landing_abort = !precision_landing_airspeed_safe(stateIsAirspeedValid(), stateGetAirspeed_f(),
                              PRECISION_LANDING_MIN_AIRSPEED) || along_speed < 1.f;
    
    /* Persistence filter for spatial prediction violations:
     * Triggers abort if prediction exceeds 1.2m cross-track or 8.0m longitudinal limits, OR if the aircraft
     * is overshooting with crow brakes saturated at >= 95% max deflection.
     * Requiring 0.2s continuous violation prevents single-sample sensor noise from canceling a good approach. */
    if (precision_landing_prediction_rejected(precision_landing_predicted_error_m,
        precision_landing_predicted_cross_track_m, PRECISION_LANDING_MAX_LONG_ERROR,
        PRECISION_LANDING_MAX_CROSS_TRACK)
        || (precision_landing_predicted_error_m < -PRECISION_LANDING_MAX_LONG_ERROR
          && precision_landing_brake_fraction >= PRECISION_LANDING_MAX_BRAKE * 0.95f)) {
      if (rejection_since < 0.f) {
        rejection_since = now;
      }
      precision_landing_abort |= now - rejection_since >= PRECISION_LANDING_REJECT_DELAY;
    } else {
      rejection_since = -1.f;
    }
  }

  /* --- PROPORTIONAL CROW BRAKE CALCULATION & FLARE BLENDING --- */
  float brake = 0.f;
  if (!precision_landing_abort && precision_landing_airspeed_safe(stateIsAirspeedValid(), stateGetAirspeed_f(),
      PRECISION_LANDING_MIN_AIRSPEED)
      && agl_dist_value_filtered < brake_enable_agl_m
      && precision_landing_predicted_error_m < aim_before_td_m) {
    brake = (aim_before_td_m - precision_landing_predicted_error_m)
            * PRECISION_LANDING_BRAKE_GAIN;
            
    /* Linear blend from 3.5m down to 0.5m AGL: smoothly tapers off predictive braking as the aircraft
     * nears the ground to prevent abrupt pitch trim shifts during touchdown flare transition. */
    float flare_blend = (agl_dist_value_filtered - 0.5f) / fmaxf(brake_enable_agl_m - 0.5f, 0.1f);
    Bound(flare_blend, 0.f, 1.f);
    brake *= flare_blend;
    Bound(brake, 0.f, PRECISION_LANDING_MAX_BRAKE);
  }

  precision_landing_brake_fraction = brake;
  commands[COMMAND_BRAKE] = (pprz_t)(-brake * MAX_PPRZ);
}

void precision_landing_glide(void)
{
  if (!precision_landing_ready) {
    return;
  }
  const struct EnuCoor_f *speed = stateGetSpeedEnu_f();
  const float along_speed = speed->x * final_unit_east + speed->y * final_unit_north;
  
  /* Dynamically update flight controller vertical altitude target along the nominal glide slope slope.
   * Shifting the target height upstream by aim_before_td_m ensures the unbraked trajectory targets
   * the upstream box entry rather than the box center, absorbing flare float. */
  const float height = fmaxf(precision_landing_remaining_m - aim_before_td_m, 0.f) * final_slope;
  NavVerticalAltitudeMode(WaypointAlt(touchdown_wp) + height, -fmaxf(along_speed, 0.f) * final_slope);
}
