/** @file pose_log.h @brief Non-blocking pose-evidence CSV logging API. */
#ifndef CATIA_POSE_LOG_H
#define CATIA_POSE_LOG_H

#include <stddef.h>
#include <stdint.h>
#include "clock_alignment.h"

/** @brief Optional token-bound evidence mapping a controller pose interval to MORA time. */
struct pose_clock_evidence {
  union catia_clock_request_union token;
  struct clock_interval sample_time;
  uint64_t probe_sent_us;
  uint64_t probe_received_us;
  uint32_t probe_receive_fc_us;
  uint32_t probe_transmit_fc_us;
  bool mapped;
};

/** @brief Snapshot counters for accepted, flushed, dropped, and rejected pose records. */
struct pose_log_stats {
  uint64_t accepted;
  uint64_t synced;
  uint64_t dropped;
  uint64_t rejected;
  int error;
};

/** @brief Start a durable asynchronous CSV session, or disable logging when directory is empty.
 * @param directory Writable destination directory, optionally using `~` shorthand.
 * @return 0 on active/disabled success, otherwise -1. */
int pose_log_start(const char *directory);
/** @brief Validate and enqueue a raw pose payload without clock mapping evidence.
 * @return 1 when queued, 0 when intentionally dropped due to contention/capacity, or -1 when invalid. */
int pose_log_record(const uint8_t *payload, size_t length, uint64_t receive_monotonic_us);
/** @brief Validate and enqueue a pose payload with optional mapped timing provenance.
 * @details Trylock protects capture latency: logging is best-effort evidence, never a reason
 * to block the real-time UART path. */
int pose_log_record_clocked(const uint8_t *payload, size_t length, uint64_t receive_monotonic_us,
                            const struct pose_clock_evidence *evidence);
/** @brief Return a synchronized snapshot of logging health and loss counters. */
struct pose_log_stats pose_log_status(void);
/** @brief Flush queued records, join the writer, and close the CSV file.
 * @return 0 after durable orderly shutdown, or -1 if a pending I/O error exists. */
int pose_log_stop(void);

#endif