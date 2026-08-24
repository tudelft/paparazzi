/*
 * Copyright (C) 2026 The Paparazzi Team
 *
 * This file is part of paparazzi; see the COPYING file.
 */

#ifndef TRAFFIC_INFO_TIME_H
#define TRAFFIC_INFO_TIME_H

#include <stdbool.h>
#include <stdint.h>

#define TRAFFIC_INFO_GPS_WEEK_MS 604800000u
#define TRAFFIC_INFO_GPS_HALF_WEEK_MS (TRAFFIC_INFO_GPS_WEEK_MS / 2u)

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

static inline bool traffic_info_equal_tow_episode_accepts(
  bool payload_changed, bool episode_active, uint64_t now_ms,
  uint64_t episode_started_ms, uint32_t equal_tow_max_ms)
{
  return payload_changed
         && (!episode_active
             || now_ms - episode_started_ms <= equal_tow_max_ms);
}

#endif /* TRAFFIC_INFO_TIME_H */
