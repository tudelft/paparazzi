#ifndef TRANSPORT_DELAY_TYPES_H
#define TRANSPORT_DELAY_TYPES_H

#include "std.h"
#include "paparazzi.h"
#include "filters/transport_delay.h"

static inline void init_transport_delay_array(uint8_t n, struct TransportDelay td_array[restrict n], const uint8_t delay_samples[restrict n], float initial_value[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    init_transport_delay(&td_array[i], delay_samples[i], initial_value[i]);
  }
}

static inline void update_transport_delay_array(uint8_t n, struct TransportDelay td_array[restrict n], const float input_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    update_transport_delay(&td_array[i], input_array[i]);
  }
}

static inline void get_transport_delay_array(uint8_t n, const struct TransportDelay td_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    output_array[i] = get_transport_delay(&td_array[i]);
  }
}

#endif // TRANSPORT_DELAY_TYPES_H