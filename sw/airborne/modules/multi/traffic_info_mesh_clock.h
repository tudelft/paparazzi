/*
 * Copyright (C) 2026 The Paparazzi Team
 *
 * This file is part of paparazzi; see the COPYING file.
 */

#ifndef TRAFFIC_INFO_MESH_CLOCK_H
#define TRAFFIC_INFO_MESH_CLOCK_H

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

/** Whether holdover must leave TDMA before the next complete superframe. */
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

/** Select the next shared recovery epoch; zero remains an inactive sentinel. */
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

/** Validate and, when necessary, safely advance a received recovery target. */
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

#endif /* TRAFFIC_INFO_MESH_CLOCK_H */