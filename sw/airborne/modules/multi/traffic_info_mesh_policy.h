/*
 * Copyright (C) 2026 The Paparazzi Team
 *
 * This file is part of paparazzi; see the COPYING file.
 */

/**
 * @file modules/multi/traffic_info_mesh_policy.h
 * @brief Dependency-light timing and mode policies for mesh traffic exchange.
 *
 * Mesh implementation details live here instead of the public mesh API so
 * traffic consumers do not inherit policy state or unnecessary dependencies.
 */

#ifndef TRAFFIC_INFO_MESH_POLICY_H
#define TRAFFIC_INFO_MESH_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/** Timing authority used for local MESH_STATE originations. */
enum MeshClockMode {
  MESH_CLOCK_GPS = 0,
  MESH_CLOCK_HOLDOVER = 1,
  MESH_CLOCK_ASYNC = 2,
  MESH_CLOCK_RECOVERY = 3
};

/** Persistent state for the mesh clock transition policy. */
struct MeshClockState {
  uint64_t anchor_network_ms;
  uint64_t anchor_local_ms;
  uint64_t holdover_started_ms;
  uint64_t acquire_started_ms;
  enum MeshClockMode mode;
  bool acquiring;
};

/** Initialize without assuming that GPS time exists at boot. */
static inline void mesh_clock_init(struct MeshClockState *clock)
{
  clock->anchor_network_ms = 0;
  clock->anchor_local_ms = 0;
  clock->holdover_started_ms = 0;
  clock->acquire_started_ms = 0;
  clock->mode = MESH_CLOCK_ASYNC;
  clock->acquiring = false;
}

/** Project an anchored network clock from monotonic local time. */
static inline uint64_t mesh_clock_project(const struct MeshClockState *clock,
                                          uint64_t local_ms)
{
  return clock->anchor_network_ms + (local_ms - clock->anchor_local_ms);
}

/** Absolute value of a timestamp difference without signed overflow. */
static inline uint64_t mesh_clock_difference_ms(uint64_t first, uint64_t second)
{
  return first >= second ? first - second : second - first;
}

/** Whether holdover must leave TDMA before the next complete superframe.
 *
 * Leaving one superframe early lets fallback propagate before peers can use
 * divergent clocks to claim the same deterministic slot.
 */
static inline bool mesh_clock_holdover_expiring(const struct MeshClockState *clock,
                                                uint64_t local_ms,
                                                uint32_t holdover_max_ms,
                                                uint32_t lead_ms)
{
  return clock->mode == MESH_CLOCK_HOLDOVER
         && local_ms - clock->holdover_started_ms
            >= holdover_max_ms - lead_ms;
}

/** Whether a modulo-32-bit frame has reached a nonzero target. */
static inline bool mesh_clock_frame_reached(uint32_t frame, uint32_t target)
{
  return target != 0 && (int32_t)(frame - target) >= 0;
}

/** Whether a nonzero candidate is later in modulo-32-bit frame order. */
static inline bool mesh_clock_frame_is_later(uint32_t candidate, uint32_t current)
{
  return candidate != 0
         && (current == 0 || (int32_t)(candidate - current) > 0);
}

/** Select the next shared recovery epoch; zero remains an inactive sentinel.
 *
 * Epoch alignment gives all recovering nodes one independently computable
 * boundary, avoiding piecemeal return to GPS-scheduled TDMA.
 */
static inline uint32_t mesh_clock_next_recovery_frame(uint32_t frame,
                                                      uint32_t epoch_frames)
{
  const uint32_t remainder = frame % epoch_frames;
  uint32_t target = frame + epoch_frames - remainder;
  if (target == 0) {
    target = epoch_frames;
  }
  return target;
}

/** Validate and, when necessary, safely advance a received recovery target.
 *
 * Targets must be aligned, unambiguously future in serial-number arithmetic,
 * and near enough to prevent a malformed peer from delaying recovery without
 * bound. A valid but short lead is moved to the next common epoch so fallback
 * leases can expire before deterministic slot ownership resumes.
 */
static inline uint32_t mesh_clock_sanitize_recovery_frame(uint32_t frame,
                                                          uint32_t candidate,
                                                          uint32_t epoch_frames,
                                                          uint32_t min_lead_frames)
{
  if (candidate == 0 || candidate % epoch_frames != 0) {
    return 0;
  }
  const uint32_t delta = candidate - frame;
  if (delta == 0 || delta >= UINT32_C(0x80000000)
      || delta > min_lead_frames + epoch_frames) {
    return 0;
  }
  if (delta < min_lead_frames) {
    return mesh_clock_next_recovery_frame(frame + min_lead_frames,
                                          epoch_frames);
  }
  return candidate;
}

/** Update the mesh clock and return its current network timestamp.
 *
 * GPS loss first preserves the established schedule through bounded holdover.
 * Forced or expired fallback uses local monotonic time and resets slots because
 * ownership learned in another clock domain is no longer collision-safe.
 * Stable GPS must persist through @p acquire_ms before re-entry; a correction
 * larger than @p step_max_ms also invalidates existing slot leases.
 *
 * @param[in,out] clock Clock policy state.
 * @param[in] local_ms Monotonic time since boot.
 * @param[in] gps_valid Whether the current GPS time sample is usable.
 * @param[in] gps_ms Absolute GPS week/TOW in milliseconds when valid.
 * @param[in] force_async Whether a GPS-denied peer requires fleet fallback.
 * @param[in] holdover_max_ms Maximum duration of bounded holdover.
 * @param[in] acquire_ms Continuous valid-GPS time required for acquisition.
 * @param[in] step_max_ms Maximum holdover-to-GPS correction that preserves slots.
 * @param[out] reset_slots Whether existing TDMA ownership must be discarded.
 */
static inline uint64_t mesh_clock_update(struct MeshClockState *clock,
                                         uint64_t local_ms,
                                         bool gps_valid,
                                         uint64_t gps_ms,
                                         bool force_async,
                                         uint32_t holdover_max_ms,
                                         uint32_t acquire_ms,
                                         uint32_t step_max_ms,
                                         bool *reset_slots)
{
  *reset_slots = false;

  if (force_async) {
    clock->acquiring = false;
    if (clock->mode != MESH_CLOCK_ASYNC) {
      clock->mode = MESH_CLOCK_ASYNC;
      *reset_slots = true;
    }
    return local_ms;
  }

  if (clock->mode == MESH_CLOCK_GPS) {
    if (gps_valid) {
      clock->anchor_network_ms = gps_ms;
      clock->anchor_local_ms = local_ms;
      return gps_ms;
    }
    clock->mode = MESH_CLOCK_HOLDOVER;
    clock->holdover_started_ms = local_ms;
    clock->acquiring = false;
    return mesh_clock_project(clock, local_ms);
  }

  if (clock->mode == MESH_CLOCK_HOLDOVER) {
    const uint64_t holdover_ms = local_ms - clock->holdover_started_ms;
    if (gps_valid) {
      if (!clock->acquiring) {
        clock->acquiring = true;
        clock->acquire_started_ms = local_ms;
      } else if (local_ms - clock->acquire_started_ms >= acquire_ms) {
        const uint64_t held_ms = mesh_clock_project(clock, local_ms);
        *reset_slots = mesh_clock_difference_ms(gps_ms, held_ms) > step_max_ms;
        clock->anchor_network_ms = gps_ms;
        clock->anchor_local_ms = local_ms;
        clock->mode = MESH_CLOCK_GPS;
        clock->acquiring = false;
        return gps_ms;
      }
    } else {
      clock->acquiring = false;
    }

    if (holdover_ms <= holdover_max_ms) {
      return mesh_clock_project(clock, local_ms);
    }

    clock->mode = MESH_CLOCK_ASYNC;
    clock->acquiring = false;
    *reset_slots = true;
    return local_ms;
  }

  if (gps_valid) {
    if (!clock->acquiring) {
      clock->acquiring = true;
      clock->acquire_started_ms = local_ms;
    } else if (local_ms - clock->acquire_started_ms >= acquire_ms) {
      clock->anchor_network_ms = gps_ms;
      clock->anchor_local_ms = local_ms;
      clock->mode = MESH_CLOCK_GPS;
      clock->acquiring = false;
      *reset_slots = true;
      return gps_ms;
    }
  } else {
    clock->acquiring = false;
  }
  return local_ms;
}

/** Inputs for deciding whether single-aircraft telemetry is safe. */
struct mesh_mode_policy_input {
  bool self_ping_fresh;  /**< The GCS recently addressed this aircraft. */
  bool other_ping_fresh; /**< The shared channel recently carried a peer PING. */
  bool peer_present;     /**< A live aircraft peer owns a learned TDMA slot. */
  uint64_t peer_quiet_ms; /**< Time since the last peer @c MESH_STATE. */
  uint32_t required_quiet_ms; /**< Minimum quiet interval before solo mode. */
};

/** Decide whether the high-rate single-aircraft profile is safe.
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

/** Decide whether a received frame is airborne peer evidence.
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

/** Apply hysteresis to mesh-versus-manifold telemetry selection.
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

/** Decide whether estimator kinematics may outlive the GNSS fix.
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

/** Adapt GPS-free origination rate to recently observed membership.
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

#endif /* TRAFFIC_INFO_MESH_POLICY_H */