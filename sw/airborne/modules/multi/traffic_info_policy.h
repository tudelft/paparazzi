/*
 * Copyright (C) 2026 The Paparazzi Team
 *
 * This file is part of paparazzi; see the COPYING file.
 */

/**
 * @file modules/multi/traffic_info_policy.h
 * @brief Dependency-light policies shared by traffic-info implementations.
 *
 * These helpers remain independent of generated firmware headers so their
 * boundary behavior can be tested directly on the host.
 */

#ifndef TRAFFIC_INFO_POLICY_H
#define TRAFFIC_INFO_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#define TRAFFIC_INFO_GPS_WEEK_MS 604800000u
#define TRAFFIC_INFO_GPS_HALF_WEEK_MS (TRAFFIC_INFO_GPS_WEEK_MS / 2u)

/** Prefer an eligible, older slot when reclaiming a full traffic table.
 *
 * Selection by last activity, rather than slot or aircraft ID, preserves every
 * track that has demonstrated more recent liveness. Unsigned elapsed time is
 * intentional because @p now_ms and activity timestamps share one monotonic
 * clock domain.
 *
 * @return @c true when this slot should replace the current candidate.
 */
static inline bool traffic_info_reclaim_candidate_preferred(
  uint64_t now_ms, uint64_t activity_ms, uint32_t reclaim_ms,
  bool candidate_selected, uint64_t selected_activity_ms)
{
  return now_ms - activity_ms >= reclaim_ms
         && (!candidate_selected || activity_ms < selected_activity_ms);
}

/** Return whether incoming GPS TOW is strictly newer than stored GPS TOW.
 *
 * The comparison uses modulo-week serial arithmetic. Equal timestamps and the
 * ambiguous exact half-week distance are not newer and must not refresh a
 * safety freshness timer.
 */
static inline bool traffic_info_itow_is_newer(uint32_t incoming, uint32_t stored)
{
  incoming %= TRAFFIC_INFO_GPS_WEEK_MS;
  stored %= TRAFFIC_INFO_GPS_WEEK_MS;
  const uint32_t forward = (incoming + TRAFFIC_INFO_GPS_WEEK_MS - stored)
                           % TRAFFIC_INFO_GPS_WEEK_MS;
  return forward > 0u && forward < TRAFFIC_INFO_GPS_HALF_WEEK_MS;
}

/** Decide whether changed payload may temporarily reuse the same source TOW.
 *
 * Some legacy senders publish live data with a frozen clock. A bounded episode
 * preserves compatibility without allowing that clock defect to refresh a
 * safety track forever. Unchanged payload never refreshes freshness.
 *
 * @param[in] payload_changed Whether any ordered observation field changed.
 * @param[in] episode_active Whether an equal-TOW episode is already running.
 * @param[in] now_ms Current monotonic receipt time.
 * @param[in] episode_started_ms Start of the current equal-TOW episode.
 * @param[in] equal_tow_max_ms Maximum compatible episode duration.
 * @return @c true when this equal-TOW observation may be accepted.
 */
static inline bool traffic_info_equal_tow_episode_accepts(
  bool payload_changed, bool episode_active, uint64_t now_ms,
  uint64_t episode_started_ms, uint32_t equal_tow_max_ms)
{
  return payload_changed
         && (!episode_active
             || now_ms - episode_started_ms <= equal_tow_max_ms);
}

/** Convert centimeters to millimeters without signed overflow.
 *
 * Datalink altitude fields can span the full signed 32-bit range. Scaling in
 * 64 bits and rejecting an unrepresentable result prevents malformed traffic
 * from wrapping across the vertical reference frame.
 *
 * @param[in] centimeters Input distance in centimeters.
 * @param[out] millimeters Converted distance; unchanged on failure.
 * @return @c true when the scaled value fits in an `int32_t`.
 */
static inline bool traffic_info_cm_to_mm(int32_t centimeters, int32_t *millimeters)
{
  const int64_t scaled = (int64_t)centimeters * INT64_C(10);
  if (scaled < INT32_MIN || scaled > INT32_MAX) {
    return false;
  }
  *millimeters = (int32_t)scaled;
  return true;
}

#endif /* TRAFFIC_INFO_POLICY_H */