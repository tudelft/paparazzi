#ifndef TRANSPORT_DELAY_H
#define TRANSPORT_DELAY_H

#include "paparazzi.h"

#define TRANSPORT_DELAY_BUFFER_SIZE 20

struct TransportDelay {
  uint8_t delay_samples; // Number of samples to delay
  uint8_t write_index; // Current write index
  float buffer[TRANSPORT_DELAY_BUFFER_SIZE];
};


/**
 * Initialize a transport delay buffer.
 *
 * @param td Pointer to the transport_delay_t structure to initialize.
 * @param delay_samples Number of samples to delay. If this value exceeds TRANSPORT_DELAY_BUFFER_SIZE,
 *        it will be clamped to TRANSPORT_DELAY_BUFFER_SIZE.
 * @param initial_value Initial value to fill the buffer with.
 *
 * @note: If delay_samples > TRANSPORT_DELAY_BUFFER_SIZE, it will be clamped to TRANSPORT_DELAY_BUFFER_SIZE
 *       and the requested delay will not be fully honored.
 */
static inline void init_transport_delay(struct TransportDelay *td, uint8_t delay_samples, float initial_value)
{
  if (delay_samples > TRANSPORT_DELAY_BUFFER_SIZE) {
    delay_samples = TRANSPORT_DELAY_BUFFER_SIZE;
  }
  td->delay_samples = delay_samples;
  td->write_index = 0;
  for (uint8_t i = 0; i < TRANSPORT_DELAY_BUFFER_SIZE; i++) {
    td->buffer[i] = initial_value;
  }
}

/**
 * Propagate a new input value through the transport delay buffer.
 *
 * @param td Pointer to the transport_delay_t structure.
 * @param input New input value to add to the buffer.
 * @return Delayed output value from the buffer.
 */
static inline float update_transport_delay(struct TransportDelay *td, float input)
{
  td->buffer[td->write_index] = input;
  uint8_t read_index = (td->write_index + TRANSPORT_DELAY_BUFFER_SIZE - td->delay_samples) % TRANSPORT_DELAY_BUFFER_SIZE;
  float output = td->buffer[read_index];
  td->write_index = (td->write_index + 1) % TRANSPORT_DELAY_BUFFER_SIZE;
  return output;
}

/**
 * Get the current output value from the transport delay buffer without updating it.
 *
 * @param td Pointer to the transport_delay_t structure.
 * @return Current delayed output value from the buffer.
 */
static inline float get_transport_delay(const struct TransportDelay *td)
{
  uint8_t read_index = (td->write_index + TRANSPORT_DELAY_BUFFER_SIZE - td->delay_samples - 1) % TRANSPORT_DELAY_BUFFER_SIZE;
  return td->buffer[read_index];
}

#endif // TRANSPORT_DELAY_H