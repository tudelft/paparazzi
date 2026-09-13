#ifndef CATIA_SERIAL_TX_H
#define CATIA_SERIAL_TX_H

/**
 * @file serial_tx.h
 * @brief Thread-safe, non-blocking framed UART transmission for CATIA.
 * @details Worker threads may finish camera jobs concurrently. This API serializes
 * complete CATIA frames through a bounded ring buffer so a slow UART cannot intermix
 * bytes from independent messages or block a capture worker.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief Attach an already-open non-blocking UART descriptor.
 * @param descriptor Descriptor configured with @c O_NONBLOCK.
 * @return 0 on success, or -1 with errno describing an invalid/busy descriptor. */
int serial_tx_start(int descriptor);
/** @brief Discard pending output and detach the UART during shutdown/restart. */
void serial_tx_stop(void);
/** @brief Queue and opportunistically flush one complete CATIA frame.
 * @param message Protocol message ID.
 * @param payload Payload bytes, or NULL when @p length is zero.
 * @param length Payload size, limited by the one-byte protocol length field.
 * @return 0 if queued/flushed, or -1 when the buffer or transport cannot accept it. */
int serial_tx_send(uint8_t message, const uint8_t *payload, size_t length);
/** @brief Send only when no earlier frame is queued, for low-priority probes.
 * @details Avoids delaying flight-critical status traffic with a clock probe.
 * @return 0 on acceptance, or -1 with EAGAIN when output is busy. */
int serial_tx_send_if_idle(uint8_t message, const uint8_t *payload, size_t length);
/** @brief Continue writing queued bytes after POLLOUT reports readiness.
 * @return 0 when output remains usable, or -1 for a terminal transport failure. */
int serial_tx_flush(void);
/** @brief Report queued data or a latched output error needing event-loop handling.
 * @return True while the event loop should request POLLOUT and call serial_tx_flush(). */
bool serial_tx_pending(void);

#endif