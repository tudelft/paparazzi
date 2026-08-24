#ifndef TRAFFIC_INFO_MESH_MODE_H
#define TRAFFIC_INFO_MESH_MODE_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Inputs for deciding whether single-aircraft telemetry is safe. */
struct mesh_mode_policy_input {
  bool self_ping_fresh;  /**< The GCS recently addressed this aircraft. */
  bool other_ping_fresh; /**< The shared channel recently carried a peer PING. */
  bool peer_present;     /**< A live aircraft peer owns a learned TDMA slot. */
  uint64_t peer_quiet_ms; /**< Time since the last peer @c MESH_STATE. */
  uint32_t required_quiet_ms; /**< Minimum quiet interval before solo mode. */
};

/** @brief Decide whether the high-rate single-aircraft profile is safe.
 *
 * GCS reachability alone is insufficient because a delayed membership view
 * could hide another aircraft. Requiring both radio-side silence and absence
 * of GCS evidence makes the transition conservative.
 *
 * @param[in] input Current GCS and peer-presence evidence.
 * @return @c true only when this aircraft is the sole observed airborne peer.
 */
static inline bool mesh_mode_should_use_solo(
  const struct mesh_mode_policy_input *input)
{
  return input->self_ping_fresh
         && !input->other_ping_fresh
         && !input->peer_present
         && input->peer_quiet_ms >= input->required_quiet_ms;
}

/** @brief Decide whether a received frame is airborne peer evidence.
 *
 * Zero and `UINT8_MAX` are protocol sentinels, while the local ID is loopback;
 * counting any of them would suppress solo mode without a real peer.
 */
static inline bool mesh_mode_is_peer_sender(uint8_t sender_id,
                                            uint8_t local_id)
{
  return sender_id != 0u && sender_id != UINT8_MAX
         && sender_id != local_id;
}

/** Spread simultaneous boot traffic deterministically over a bounded window.
 *
 * Identity-derived jitter avoids a shared random-state dependency while still
 * separating repeated fleet-wide power-up announcements.
 */
static inline uint32_t mesh_mode_boot_spread_ms(uint32_t identity,
                                                uint32_t window_ms)
{
  identity ^= identity >> 16;
  identity *= UINT32_C(0x7feb352d);
  identity ^= identity >> 15;
  identity *= UINT32_C(0x846ca68b);
  identity ^= identity >> 16;
  return window_ms == 0u ? 0u : identity % window_ms;
}

/** @brief Apply hysteresis to mesh-versus-manifold telemetry selection.
 *
 * Distinct enter and exit thresholds prevent profile flapping when membership
 * hovers at the boundary.
 *
 * @param[in] neighbours Number of distinct live aircraft peers.
 * @param[in] manifold_active Whether the manifold profile is currently selected.
 * @param[in] enter_neighbours Enter manifold mode at or above this count.
 * @param[in] exit_neighbours Leave manifold mode at or below this count.
 * @return @c true when the manifold telemetry profile should be selected.
 */
static inline bool mesh_mode_should_use_manifold(uint8_t neighbours,
                                                  bool manifold_active,
                                                  uint8_t enter_neighbours,
                                                  uint8_t exit_neighbours)
{
  return manifold_active ? neighbours > exit_neighbours
                         : neighbours >= enter_neighbours;
}

/** Closed interval for one randomized asynchronous transmission deadline. */
struct mesh_async_interval {
  uint32_t min_ms; /**< Earliest permitted origination. */
  uint32_t max_ms; /**< Latest permitted origination. */
};

/** @brief Decide whether estimator kinematics may outlive the GNSS fix.
 *
 * A short inclusive grace period bridges ordinary fix churn; after it expires,
 * advertising dead-reckoned position would conceal loss of navigation quality.
 *
 * @param[in] fix_seen Whether this boot has observed a valid 3D GNSS fix.
 * @param[in] elapsed_ms Monotonic time since that last valid fix.
 * @param[in] holdover_ms Configured estimator-position grace interval.
 * @return @c true through the inclusive grace boundary.
 */
static inline bool mesh_position_holdover_valid(bool fix_seen,
                                                uint64_t elapsed_ms,
                                                uint32_t holdover_ms)
{
  return fix_seen && elapsed_ms <= holdover_ms;
}

/** @brief Adapt GPS-free origination rate to recently observed membership.
 *
 * Unknown membership retains the configured worst-case interval. Once a node
 * has synchronized with the fleet or heard an asynchronous peer, sparse
 * fleets can safely spend otherwise unused channel capacity on fresher state.
 * Four or more observed peers retain the fully validated dense-fleet rate.
 *
 * @param[in] recent_peers Distinct peers heard within the fallback lease.
 * @param[in] population_known Whether the peer count has a trustworthy basis.
 * @param[in] base_min_ms Dense-fleet minimum randomized interval.
 * @param[in] base_max_ms Dense-fleet maximum randomized interval.
 * @return Randomized interval bounds for the current membership evidence.
 */
static inline struct mesh_async_interval mesh_async_interval_for_peers(
  uint8_t recent_peers, bool population_known,
  uint32_t base_min_ms, uint32_t base_max_ms)
{
  struct mesh_async_interval interval = {
    .min_ms = base_min_ms,
    .max_ms = base_max_ms
  };
  if (!population_known) {
    return interval;
  }
  if (recent_peers == 0) {
    interval.min_ms = base_min_ms / 4u;
    interval.max_ms = base_max_ms / 3u;
  } else if (recent_peers <= 3u) {
    interval.min_ms = base_min_ms / 2u;
    interval.max_ms = base_max_ms - base_max_ms / 3u;
  }
  if (interval.min_ms == 0) {
    interval.min_ms = 1;
  }
  if (interval.max_ms < interval.min_ms) {
    interval.max_ms = interval.min_ms;
  }
  return interval;
}

#endif /* TRAFFIC_INFO_MESH_MODE_H */