#pragma once
#include "std.h"
#define AP_MODE_AUTO2 2
extern uint8_t test_mode;
struct TestAutopilot {
	bool kill_throttle;
	bool launch;
	uint16_t flight_time;
};
extern struct TestAutopilot autopilot;
static inline uint8_t autopilot_get_mode(void) { return test_mode; }