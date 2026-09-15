#pragma once
#include "std.h"
struct transport_tx { int unused; };
struct link_device { int unused; };
typedef void (*test_telemetry_callback_t)(struct transport_tx *, struct link_device *);
#define DefaultPeriodic 0
#define PPRZ_MSG_ID_RANGEFINDER 1
extern void register_periodic_telemetry(int periodic, int message, test_telemetry_callback_t callback);