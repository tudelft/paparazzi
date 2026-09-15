#pragma once
#include "modules/datalink/telemetry.h"
extern void pprz_msg_send_RANGEFINDER(struct transport_tx *trans, struct link_device *device, uint8_t aircraft,
                                    const uint8_t *address, const uint16_t *raw, const float *distance);