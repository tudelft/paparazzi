#pragma once
#include "std.h"
extern void test_rangefinder_debug_send(const uint8_t *address, const uint16_t *raw, const float *distance);
#define DOWNLINK_SEND_RANGEFINDER(channel, device, address, raw, distance) test_rangefinder_debug_send(address, raw, distance)