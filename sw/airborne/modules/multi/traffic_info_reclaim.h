#ifndef TRAFFIC_INFO_RECLAIM_H
#define TRAFFIC_INFO_RECLAIM_H

#include <stdbool.h>
#include <stdint.h>

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

#endif /* TRAFFIC_INFO_RECLAIM_H */