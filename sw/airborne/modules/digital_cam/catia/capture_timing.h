#ifndef CATIA_CAPTURE_TIMING_H
#define CATIA_CAPTURE_TIMING_H

/**
 * @file capture_timing.h
 * @brief Evidence recorded for a thermal request and the callback frame that served it.
 * @details Timing is deliberately represented with monotonic clocks only; wall-clock
 * adjustments must never turn an otherwise valid capture into a negative duration.
 */

#include <stdbool.h>
#include <stdint.h>

/** @brief Correlated request and frame-delivery timestamps for one LWIR capture. */
struct capture_timing {
  uint64_t request_monotonic_us;
  uint64_t arrival_monotonic_us;
  uint64_t callback_sequence;
  uint64_t callback_drops;
  bool callback_arrival;
};

/**
 * @brief Check whether timing evidence is internally consistent and recent.
 * @param timing Evidence to inspect.
 * @return True for a plausible request/arrival pair, false otherwise.
 * @details The 20-second bound detects stale parser state without rejecting normal
 * warmup/USB recovery behavior. Callback and polling modes have intentionally distinct
 * sequence invariants, reflected in the final conditional.
 */
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