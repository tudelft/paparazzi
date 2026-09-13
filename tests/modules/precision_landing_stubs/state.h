#pragma once
#include "std.h"
struct EnuCoor_f { float x, y, z; };
extern struct EnuCoor_f test_position, test_speed;
extern float test_airspeed;
extern bool test_airspeed_valid;
static inline const struct EnuCoor_f *stateGetPositionEnu_f(void) { return &test_position; }
static inline const struct EnuCoor_f *stateGetSpeedEnu_f(void) { return &test_speed; }
static inline bool stateIsAirspeedValid(void) { return test_airspeed_valid; }
static inline float stateGetAirspeed_f(void) { return test_airspeed; }