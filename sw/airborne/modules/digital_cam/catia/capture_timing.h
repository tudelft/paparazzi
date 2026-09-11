#ifndef CATIA_CAPTURE_TIMING_H
#define CATIA_CAPTURE_TIMING_H

#include <stdbool.h>
#include <stdint.h>

struct capture_timing {
  uint64_t request_monotonic_us;
  uint64_t arrival_monotonic_us;
  uint64_t callback_sequence;
  uint64_t callback_drops;
  bool callback_arrival;
};

static inline bool capture_timing_valid(const struct capture_timing *timing)
{
  return timing != 0 && timing->request_monotonic_us > 0
         && timing->arrival_monotonic_us >= timing->request_monotonic_us
         && timing->arrival_monotonic_us - timing->request_monotonic_us <= UINT64_C(20000000)
         && (timing->callback_arrival
             ? timing->callback_sequence > 0 && timing->callback_drops < timing->callback_sequence
               && timing->arrival_monotonic_us > timing->request_monotonic_us
             : timing->callback_sequence == 0 && timing->callback_drops == 0);
}

#endif