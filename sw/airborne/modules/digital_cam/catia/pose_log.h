#ifndef CATIA_POSE_LOG_H
#define CATIA_POSE_LOG_H

#include <stddef.h>
#include <stdint.h>
#include "clock_alignment.h"

struct pose_clock_evidence {
  union catia_clock_request_union token;
  struct clock_interval sample_time;
  uint64_t probe_sent_us;
  uint64_t probe_received_us;
  uint32_t probe_receive_fc_us;
  uint32_t probe_transmit_fc_us;
  bool mapped;
};

struct pose_log_stats {
  uint64_t accepted;
  uint64_t synced;
  uint64_t dropped;
  uint64_t rejected;
  int error;
};

int pose_log_start(const char *directory);
int pose_log_record(const uint8_t *payload, size_t length, uint64_t receive_monotonic_us);
int pose_log_record_clocked(const uint8_t *payload, size_t length, uint64_t receive_monotonic_us,
                            const struct pose_clock_evidence *evidence);
struct pose_log_stats pose_log_status(void);
int pose_log_stop(void);

#endif