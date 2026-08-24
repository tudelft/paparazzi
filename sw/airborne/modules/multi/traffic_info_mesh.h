/*
 * Copyright (C) OpenUAS (2026)
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

/**
 * @file modules/multi/traffic_info_mesh.h
 * @brief Optional mesh transport configuration and public traffic APIs.
 */

#ifndef TRAFFIC_INFO_MESH_H
#define TRAFFIC_INFO_MESH_H

#include "std.h"

#ifndef TRAFFIC_INFO_USE_MESH
#define TRAFFIC_INFO_USE_MESH 0
#endif

/** Traffic status bit identifying the MESH_STATE source. */
#define AC_INFO_SOURCE_MESH 9

#if TRAFFIC_INFO_USE_MESH
#include "math/pprz_geodetic_float.h"
#include "modules/multi/traffic_info_mesh_clock.h"

/**
 * @defgroup mesh_state Optional MESH_STATE transport
 * @brief Compact state exchange for narrowband broadcast mesh radios.
 * @{
 */

/** Length of one TDMA superframe in ms. */
#ifndef MESH_TDMA_SUPERFRAME_MS
#define MESH_TDMA_SUPERFRAME_MS 16000
#endif

/** Slots per superframe. */
#ifndef MESH_TDMA_NB_SLOTS
#define MESH_TDMA_NB_SLOTS 32
#endif

/** Slots distributed by the fair-share allocator. */
#ifndef MESH_TDMA_FAIR_SLOTS
#define MESH_TDMA_FAIR_SLOTS 25
#endif

/** Maximum slots one node may occupy in a sparse mesh. */
#ifndef MESH_TDMA_MAX_REUSE
#define MESH_TDMA_MAX_REUSE 8
#endif

/** Superframes listened before choosing a primary slot. */
#ifndef MESH_ENTRY_FRAMES
#define MESH_ENTRY_FRAMES 3
#endif

/** Primary-slot lease bounds, in superframes. */
#ifndef MESH_PRIMARY_HOLD_MIN
#define MESH_PRIMARY_HOLD_MIN 120
#endif
#ifndef MESH_PRIMARY_HOLD_SPAN
#define MESH_PRIMARY_HOLD_SPAN 120
#endif

/** Opportunistic-slot lease bounds, in superframes. */
#ifndef MESH_SLOT_HOLD_MIN
#define MESH_SLOT_HOLD_MIN 6
#endif
#ifndef MESH_SLOT_HOLD_SPAN
#define MESH_SLOT_HOLD_SPAN 8
#endif

/** Superframes a learned slot owner remains current. */
#ifndef MESH_SLOT_AGE_FRAMES
#define MESH_SLOT_AGE_FRAMES 4
#endif

/** Superframes between one-rank rotations of remainder-slot entitlement. */
#ifndef MESH_REMAINDER_EPOCH_FRAMES
#define MESH_REMAINDER_EPOCH_FRAMES                                            \
  (4 * MESH_SLOT_AGE_FRAMES + 2 * MESH_TDMA_NB_SLOTS + 1)
#endif

/** Nominal integral slot length in ms. */
#define MESH_TDMA_SLOT_MS (MESH_TDMA_SUPERFRAME_MS / MESH_TDMA_NB_SLOTS)

/** Maximum age of mesh kinematics before a legacy source may take over. */
#ifndef TRAFFIC_INFO_MESH_DROP_MS
#define TRAFFIC_INFO_MESH_DROP_MS                                              \
  (2u * MESH_TDMA_SUPERFRAME_MS + MESH_TDMA_SLOT_MS + 1000u)
#endif

/** Marker for an unowned mesh slot; 0 remains the valid GCS identity. */
#define MESH_SLOT_FREE 255u

#if MESH_TDMA_SUPERFRAME_MS < 1
#error "MESH_TDMA_SUPERFRAME_MS must be positive"
#endif
#if MESH_TDMA_NB_SLOTS < 1
#error "MESH_TDMA_NB_SLOTS must be positive"
#endif
#if MESH_TDMA_SUPERFRAME_MS < MESH_TDMA_NB_SLOTS
#error "MESH_TDMA_SUPERFRAME_MS is too short for MESH_TDMA_NB_SLOTS"
#endif
#if MESH_TDMA_NB_SLOTS > 255
#error "MESH_TDMA_NB_SLOTS must fit in uint8_t"
#endif
#if MESH_TDMA_FAIR_SLOTS < 1 || MESH_TDMA_FAIR_SLOTS > MESH_TDMA_NB_SLOTS
#error "MESH_TDMA_FAIR_SLOTS must be between 1 and MESH_TDMA_NB_SLOTS"
#endif
#if MESH_TDMA_MAX_REUSE < 1
#error "MESH_TDMA_MAX_REUSE must be at least 1"
#endif
#if MESH_SLOT_HOLD_SPAN < 1
#error "MESH_SLOT_HOLD_SPAN must be positive"
#endif
#if MESH_PRIMARY_HOLD_SPAN < 1
#error "MESH_PRIMARY_HOLD_SPAN must be positive"
#endif
#if MESH_REMAINDER_EPOCH_FRAMES <=                                             \
    (2 * MESH_SLOT_AGE_FRAMES + MESH_TDMA_NB_SLOTS)
#error "MESH_REMAINDER_EPOCH_FRAMES is too short for safe remainder rotation"
#endif

/** Maximum GPS-to-monotonic correction tolerated without relearning slots. */
#ifndef MESH_CLOCK_STEP_MAX_MS
#define MESH_CLOCK_STEP_MAX_MS 10
#endif

/** Maximum bounded TDMA holdover after losing GPS time. */
#ifndef MESH_CLOCK_HOLDOVER_MAX_MS
#define MESH_CLOCK_HOLDOVER_MAX_MS 60000u
#endif

/** Continuous valid-GPS interval required before entering GPS TDMA. */
#ifndef MESH_CLOCK_ACQUIRE_MS
#define MESH_CLOCK_ACQUIRE_MS 2000u
#endif

/** Maximum estimator global-position age after the last 3D GNSS fix. */
#ifndef MESH_POSITION_HOLDOVER_MS
#define MESH_POSITION_HOLDOVER_MS MESH_TDMA_SUPERFRAME_MS
#endif

/** Randomized origination interval without bounded network time. */
#ifndef MESH_ASYNC_MIN_INTERVAL_MS
#define MESH_ASYNC_MIN_INTERVAL_MS 16000u
#endif
#ifndef MESH_ASYNC_MAX_INTERVAL_MS
#define MESH_ASYNC_MAX_INTERVAL_MS 24000u
#endif

/** Fleet fallback lease after a GPS-denied peer advertisement. */
#ifndef MESH_ASYNC_PEER_HOLD_MS
#define MESH_ASYNC_PEER_HOLD_MS                                                \
  (2u * MESH_ASYNC_MAX_INTERVAL_MS + MESH_TDMA_SUPERFRAME_MS)
#endif

/** Fixed GPS-frame epoch used to select a common recovery target. */
#ifndef MESH_RECOVERY_EPOCH_FRAMES
#define MESH_RECOVERY_EPOCH_FRAMES 8u
#endif

/** Minimum recovery-target lead beyond fallback leases. */
#ifndef MESH_RECOVERY_MIN_LEAD_FRAMES
#define MESH_RECOVERY_MIN_LEAD_FRAMES                                          \
  (MESH_ASYNC_PEER_HOLD_MS / MESH_TDMA_SUPERFRAME_MS + 2u)
#endif

#if MESH_CLOCK_HOLDOVER_MAX_MS == 0
#error "MESH_CLOCK_HOLDOVER_MAX_MS must be positive"
#endif
#if MESH_CLOCK_HOLDOVER_MAX_MS <= MESH_TDMA_SUPERFRAME_MS
#error "MESH_CLOCK_HOLDOVER_MAX_MS must exceed one complete superframe"
#endif
#if MESH_CLOCK_ACQUIRE_MS == 0
#error "MESH_CLOCK_ACQUIRE_MS must be positive"
#endif
#if MESH_POSITION_HOLDOVER_MS > MESH_CLOCK_HOLDOVER_MAX_MS
#error "MESH_POSITION_HOLDOVER_MS must not outlive bounded clock holdover"
#endif
#if MESH_ASYNC_MIN_INTERVAL_MS == 0
#error "MESH_ASYNC_MIN_INTERVAL_MS must be positive"
#endif
#if MESH_ASYNC_MAX_INTERVAL_MS < MESH_ASYNC_MIN_INTERVAL_MS
#error "MESH_ASYNC_MAX_INTERVAL_MS must be at least MESH_ASYNC_MIN_INTERVAL_MS"
#endif
#if MESH_ASYNC_PEER_HOLD_MS <= MESH_ASYNC_MAX_INTERVAL_MS
#error "MESH_ASYNC_PEER_HOLD_MS must exceed one maximum fallback interval"
#endif
#if MESH_RECOVERY_EPOCH_FRAMES <=                                              \
    (MESH_ASYNC_PEER_HOLD_MS / MESH_TDMA_SUPERFRAME_MS)
#error "MESH_RECOVERY_EPOCH_FRAMES must exceed the denied-peer lease"
#endif
#if MESH_RECOVERY_MIN_LEAD_FRAMES <=                                           \
    (MESH_ASYNC_PEER_HOLD_MS / MESH_TDMA_SUPERFRAME_MS)
#error "MESH_RECOVERY_MIN_LEAD_FRAMES must exceed the denied-peer lease"
#endif

/** Estimated time for one frame to leave the modem transmit cache, in ms. */
#ifndef MESH_MODEM_DRAIN_MS
#define MESH_MODEM_DRAIN_MS 60
#endif

/** Stop local admission at this estimated local queue depth. */
#ifndef MESH_CACHE_HIGH_WATER
#define MESH_CACHE_HIGH_WATER 3
#endif

#ifndef MESH_AUTO_TELEMETRY
#define MESH_AUTO_TELEMETRY FALSE
#endif

#ifndef MESH_GCS_PING_TIMEOUT_MS
#define MESH_GCS_PING_TIMEOUT_MS 60000u
#endif

#ifndef MESH_SOLO_QUIET_MS
#define MESH_SOLO_QUIET_MS MESH_TDMA_SUPERFRAME_MS
#endif

#if MESH_CACHE_HIGH_WATER >= 5
#error "MESH_CACHE_HIGH_WATER must stay below the 5 frame hardware cache"
#endif

/** Unified, firmware-independent mode stored in MESH_STATE flags.
 *
 * These values and the flag bits below are the on-air ABI. Do not reorder or
 * reuse them without an atomic fleet protocol migration.
 */
#define MESH_MODE_MANUAL 0u
#define MESH_MODE_ASSISTED 1u
#define MESH_MODE_AUTO 2u
#define MESH_MODE_HOME 3u
#define MESH_MODE_NOGPS 4u
#define MESH_MODE_FAILSAFE 5u
#define MESH_MODE_KILL 6u
#define MESH_MODE_UNKNOWN 7u

#define MESH_FLAG_MODE_MASK 0x07u
#define MESH_FLAG_ROTORCRAFT 0x08u
#define MESH_FLAG_POS_VALID 0x10u
#define MESH_FLAG_AIRBORNE 0x20u
#define MESH_FLAG_ALERT 0x40u
#define MESH_FLAG_EMERGENCY 0x80u

/** Advance mesh timing, membership, and the single MESH_STATE transmit gate.
 *
 * Generated telemetry only supplies the transport; this task remains the sole
 * transmission authority so no callback can bypass TDMA or queue admission.
 */
extern void traffic_info_mesh_periodic(void);

/** Copy selected mesh kinematics and project position at constant velocity.
 *
 * Projection is limited to @p max_prediction_ms. The returned @p age_ms is the
 * actual unprojected observation age; callers retain responsibility for their
 * own freshness threshold.
 *
 * @param[in] ac_id Aircraft identity to query.
 * @param[in] max_prediction_ms Maximum extrapolation interval.
 * @param[out] position Projected local ENU position in meters.
 * @param[out] velocity Observed local ENU velocity in meters per second.
 * @param[out] age_ms Age of the original MESH_STATE observation.
 * @return @c true only when complete, finite mesh kinematics are available.
 */
extern bool traffic_info_get_mesh_snapshot(uint8_t ac_id,
                                           uint32_t max_prediction_ms,
                                           struct EnuCoor_f *position,
                                           struct EnuCoor_f *velocity,
                                           uint32_t *age_ms);

/** Return the age of the last position-valid MESH_STATE observation.
 *
 * Presence-only heartbeats deliberately do not refresh this timestamp, so a
 * peer may remain visible while its kinematics continue to age out.
 *
 * @param[in] ac_id Aircraft identity to query.
 * @param[out] age_ms Monotonic age, saturated at `UINT32_MAX`.
 * @return @c true when the selected mesh track has valid kinematics history.
 */
extern bool traffic_info_get_mesh_valid_age(uint8_t ac_id, uint32_t *age_ms);

/** Return whether mesh remains the record's selected source.
 *
 * Source selection does not imply fresh or position-valid kinematics; use
 * traffic_info_get_mesh_snapshot() or traffic_info_get_mesh_valid_age() when
 * making safety decisions.
 */
extern bool traffic_info_is_mesh_track(uint8_t ac_id);

/** @} */
#endif /* TRAFFIC_INFO_USE_MESH */

#endif /* TRAFFIC_INFO_MESH_H */