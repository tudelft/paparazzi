#ifndef TEST_SDP3X_MESSAGES_H
#define TEST_SDP3X_MESSAGES_H

#include <stdint.h>

struct transport_tx {
  int unused;
};
struct link_device;

static inline void pprz_msg_send_AIRSPEED_RAW(struct transport_tx *trans,
                                               struct link_device *device,
                                               uint8_t aircraft_id,
                                               uint8_t *sensor_id,
                                               uint16_t *raw,
                                               float *offset,
                                               float *pressure,
                                               float *temperature,
                                               float *airspeed)
{
  (void)trans;
  (void)device;
  (void)aircraft_id;
  (void)sensor_id;
  (void)raw;
  (void)offset;
  (void)pressure;
  (void)temperature;
  (void)airspeed;
}

#endif
