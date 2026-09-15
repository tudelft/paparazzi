/**
 * @file modules/nav/precision_landing.h
 * @brief Fixed-Wing Precision Landing Module Header
 * @author TU Delft IMAV 2026 Team / Paparazzi UAV
 *
 * @details
 * This header defines the public interface, data structures, and inline kinematic
 * prediction functions for the fixed-wing autonomous precision landing module.
 *
 * ### Architecture & Mathematical Overview
 * The precision landing algorithm uses a 2D ENU kinematic projection model.
 * Given a touchdown waypoint \f$TD\f$ and approach fix \f$AF\f$, it projects the
 * aircraft's ground position and velocity vector onto along-track (runway heading)
 * and cross-track (lateral offset) axes.
 *
 * Using rangefinder AGL height \f$h\f$ and filtered range sink rate \f$v_z\f$,
 * the estimated time-to-impact is:
 * \f[
 *   t_{contact} = \frac{h}{\max(v_z, v_{z,min})}
 * \f]
 * The predicted touchdown coordinates relative to \f$TD\f$ are projected as:
 * \f[
 *   e_{long} = d_{remaining} - v_{along} \cdot t_{contact}
 * \f]
 * \f[
 *   e_{cross} = c_{cross} - v_{cross} \cdot t_{contact}
 * \f]
 *
 * Where:
 * - \f$e_{long} > 0\f$: predicted touchdown occurs upstream (short) of \f$TD\f$.
 * - \f$e_{long} < 0\f$: predicted touchdown occurs downstream (long/overshoot) of \f$TD\f$.
 * - \f$e_{cross}\f$: predicted lateral displacement from runway centerline at contact.
 *
 * Predictive crow braking (spoileron deflection) is commanded proportionally to
 * predicted overshoot beyond a user-configured aim point (\f$d_{aim}\f$ upstream of \f$TD\f$).
 */

#ifndef PRECISION_LANDING_H
#define PRECISION_LANDING_H

#include "std.h"
#include <math.h>

/**
 * @brief Evaluates whether a predicted touchdown location breaches safety boundaries.
 *
 * @details
 * Checks for non-finite floating-point outputs (defensive against sensor dropouts)
 * and evaluates if either longitudinal overshoot or lateral cross-track drift
 * exceeds hard safety limits.
 *
 * @param longitudinal_error Predicted distance upstream (+) or downstream (-) of TD (meters).
 * @param lateral_error Predicted cross-track offset from runway centerline (meters).
 * @param max_short_error Maximum absolute longitudinal error (meters).
 * @param max_lateral_error Maximum acceptable lateral cross-track error (meters).
 * @return true If the prediction is non-finite or outside allowable spatial bounds.
 * @return false If the contact prediction is within bounds; this does not guarantee a safe final stop.
 *
 * @note A persistence filter in precision_landing.c requires this predicate to remain
 *       true for a continuous duration (e.g. 0.2s) before triggering an abort, preventing
 *       single-sample optical or GNSS noise spikes from interrupting a valid approach.
 */
static inline bool precision_landing_prediction_rejected(float longitudinal_error, float lateral_error,
		float max_short_error, float max_lateral_error)
{
	/* Non-finite check prevents undefined comparison behavior if sensor state becomes NaN.
	 * Negative longitudinal error means landing past TD (overshoot); error > max_short_error
	 * means landing too far short of the box. */
	return !isfinite(longitudinal_error) || !isfinite(lateral_error)
				 || fabsf(longitudinal_error) > max_short_error || fabsf(lateral_error) > max_lateral_error;
}

/**
 * @brief Validates airspeed safety for drag device (crow brake) deployment.
 *
 * @details
 * Ensures that measured pitot airspeed is valid, finite, and at or above
 * the minimum stall margin airspeed before allowing active drag application.
 *
 * @param valid Boolean flag indicating whether the airspeed state estimator is healthy.
 * @param airspeed Measured true or calibrated airspeed (m/s).
 * @param minimum Minimum safe approach airspeed threshold (m/s).
 * @return true If airspeed is valid and safely above stall threshold.
 * @return false If airspeed is invalid or dangerously close to stall.
 *
 * @warning This predicate is not an aerodynamic envelope guarantee. The controller
 *          also checks measurement freshness before authorizing brake demand.
 */
static inline bool precision_landing_airspeed_safe(bool valid, float airspeed, float minimum)
{
	return valid && isfinite(airspeed) && airspeed >= minimum;
}

/**
 * @brief Determines if altitude has dropped below the point of no return for aborts.
 *
 * @details
 * Evaluates whether current AGL or fused barometric height is at or below the
 * commit height (typically 2.5m AGL). Below this height, initiating a full-power
 * go-around turn presents higher ground-strike and tip-stall risks than completing
 * a straight-ahead flare landing.
 *
 * @param height Current measured AGL or fused altitude above touchdown elevation (meters).
 * @param commit_height Altitude threshold below which go-around is disallowed (meters).
 * @return true If the aircraft is at or below commit height and must flare.
 * @return false If above commit height or unknown; callers must handle unknown height.
 */
static inline bool precision_landing_should_commit(float height, float commit_height)
{
	return isfinite(height) && height <= commit_height;
}

/**
 * @struct PrecisionLandingPrediction
 * @brief Predicted touchdown position relative to the target touchdown waypoint.
 */
struct PrecisionLandingPrediction {
	float longitudinal_error_m; /**< Along-track error from TD: positive = short, negative = long (m) */
	float cross_track_error_m;  /**< Cross-track error from runway centerline: positive = right, negative = left (m) */
};

/**
 * @brief Computes projected touchdown position using current ENU position, velocity, and AGL sink rate.
 *
 * @details
 * Predicts the point of first ground contact by projecting current horizontal velocity
 * over the estimated time-to-ground.
 * Ground velocity already includes wind-induced travel; adding a wind vector here would
 * double-count it. This constant-velocity estimate does not anticipate gusts, flare dynamics
 * or surface-dependent stopping after contact. Keep final-rest scoring separate from this
 * predictor, and do not interpret the upstream geometric aim as a calibrated slide distance.
 *
 * To avoid numerical division-by-zero or negative time-to-ground during thermals or up-drafts,
 * sink rate is lower-bounded by `min_sink_rate_mps` (typically 0.25 m/s).
 *
 * @param remaining_m Distance along runway vector from current position to TD (meters).
 * @param cross_track_m Current perpendicular offset from runway vector (meters).
 * @param along_speed_mps Groundspeed component parallel to runway direction (m/s).
 * @param cross_speed_mps Groundspeed component perpendicular to runway direction (m/s).
 * @param agl_m Current filtered altitude above ground level (meters).
 * @param sink_rate_mps Filtered vertical descent rate (m/s, positive downwards).
 * @param min_sink_rate_mps Minimum sink rate clamp to prevent division by zero (m/s).
 * @return struct PrecisionLandingPrediction Predicted longitudinal and cross-track errors.
 */
static inline struct PrecisionLandingPrediction precision_landing_predict(float remaining_m, float cross_track_m,
		float along_speed_mps, float cross_speed_mps, float agl_m, float sink_rate_mps, float min_sink_rate_mps)
{
	/* Clamp sink rate to a positive non-zero minimum. If the aircraft is temporarily floating
	 * or climbing in a thermal, this prevents infinite/negative time-to-ground projections. */
	const float bounded_sink_rate = sink_rate_mps > min_sink_rate_mps ? sink_rate_mps : min_sink_rate_mps;
	const float time_to_ground = agl_m / bounded_sink_rate;
	const struct PrecisionLandingPrediction prediction = {
		.longitudinal_error_m = remaining_m - along_speed_mps * time_to_ground,
		.cross_track_error_m = cross_track_m - cross_speed_mps * time_to_ground
	};
	return prediction;
}

/* --- Public Global Variables (Exposed for Flight Plan & Telemetry) --- */

extern float precision_landing_remaining_m;             /**< Current along-track distance to TD (m) */
extern float precision_landing_cross_track_m;            /**< Current cross-track distance to runway centerline (m) */
extern float precision_landing_predicted_cross_track_m;  /**< Projected cross-track offset at touchdown (m) */
extern float precision_landing_predicted_error_m;        /**< Projected longitudinal offset from TD at touchdown (m) */
extern float precision_landing_brake_fraction;           /**< Currently commanded crow brake fraction (0.0 to 1.0) */
extern bool precision_landing_agl_fresh;                 /**< Flag indicating if rangefinder data was updated recently */
extern bool precision_landing_abort;                     /**< Flag set when safety limits are violated (triggers go-around) */
extern bool precision_landing_commit_flare;              /**< Flag set when below commit height (forces flare) */
extern bool precision_landing_cancelled; /**< Takeover latch cleared only by explicit start. */
/** @brief Arm a new landing sequence in AUTO2, without enabling throttle. */
extern void precision_landing_start(void);
/** @brief Check the example's live tuning values before navigation or integer conversion. */
extern bool precision_landing_parameters_valid(float airspeed, float height, float brake_height,
	float flare_height, float aim, float pitch, float brake, float retries);
/** @brief Check approach geometry before the example mutates AF altitude or computes a baseleg. */
extern bool precision_landing_entry_valid(uint8_t af_wp, uint8_t td_wp, float height, float radius);
/** @brief True only while an explicitly started landing owns AUTO2 control. */
extern bool precision_landing_is_active(void);
/** @brief Start a ground-only crow check without changing throttle kill or launch state. */
extern void precision_landing_bench_start(void);
/** @brief Refresh bench demand only while ground, zero-motion and killed-throttle interlocks hold. */
extern bool precision_landing_bench_run(void);
/** @brief Synchronously cancel landing on an autopilot mode change. */
extern void precision_landing_on_mode_change(uint8_t mode);
/** @brief Release brake demand and owned roll limit without clearing commitment. */
extern void precision_landing_release(void);
/** @brief Recheck height before/during go-around; unknown height prohibits powered abort. */
extern void precision_landing_check_abort(void);

/* --- Public Function Declarations --- */

/**
 * @brief Module initialization function registered with Paparazzi telemetry subsystem.
 */
extern void precision_landing_init(void);

/**
 * @brief Configures touchdown geometry and parameters upon entering final approach.
 * @param af_wp Waypoint ID of Approach Fix (start of final approach).
 * @param td_wp Waypoint ID of Touchdown target (center of precision box).
 * @param brake_enable_agl Altitude below which predictive crow braking is activated (meters).
 * @param aim_before_td Upstream offset from TD used as the nominal zero-brake target (meters).
 */
extern void precision_landing_setup(uint8_t af_wp, uint8_t td_wp, float brake_enable_agl, float aim_before_td);

/**
 * @brief Main periodic routine called on every navigation step during final approach (`pre_call`).
 */
extern void precision_landing_run(void);

/**
 * @brief Updates vertical glide slope setpoint based on remaining runway distance (`post_call`).
 */
extern void precision_landing_glide(void);

/**
 * @brief Applies airspeed-gated flare braking and a bounded roll limit.
 * @param brake_fraction Normalized spoileron brake deflection to hold during flare (0.0 to 1.0).
 */
extern void precision_landing_flare(float brake_fraction);
/** @brief Apply zero throttle, bounded pitch (degrees), and GPS-aware lateral flare guidance. */
extern void precision_landing_flare_run(float pitch_deg, float brake_fraction);

/**
 * @brief End the sequence, clear brake demand, and restore the previously owned roll limit.
 */
extern void precision_landing_stop(void);

#endif /* PRECISION_LANDING_H */
