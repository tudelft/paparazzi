#ifndef CATIA_CLOCK_ALIGNMENT_H
#define CATIA_CLOCK_ALIGNMENT_H

#include <stddef.h>
#include "protocol.h"

#define CLOCK_ALIGNMENT_MAX_RTT_US UINT64_C(100000)
#define CLOCK_ALIGNMENT_MAX_AGE_US UINT64_C(2000000)
#define CLOCK_ALIGNMENT_DRIFT_PPM UINT64_C(1000)

struct clock_interval {
  uint64_t earliest_us;
  uint64_t latest_us;
};

struct clock_alignment {
  union catia_clock_request_union pending_token;
  union catia_clock_request_union mapped_token;
  uint64_t sent_us;
  uint64_t received_us;
  uint64_t mapped_sent_us;
  uint32_t receive_fc_us;
  struct clock_interval anchor;
  uint32_t anchor_fc_us;
  bool pending;
  bool valid;
};

bool clock_alignment_request(struct clock_alignment *clock, union catia_clock_request_union token, uint64_t sent_us);
bool clock_alignment_reply(struct clock_alignment *clock, const uint8_t *payload, size_t length, uint64_t received_us);
bool clock_alignment_map(const struct clock_alignment *clock, union catia_clock_request_union token,
                         uint32_t fc_us, uint64_t now_us, struct clock_interval *interval);

#endif