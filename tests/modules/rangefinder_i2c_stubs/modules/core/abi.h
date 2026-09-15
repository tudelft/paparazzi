#pragma once
#include "std.h"
#define AGL_RANGEFINDER_I2C_ID 12
extern uint32_t get_sys_time_usec(void);
extern void AbiSendMsgAGL(uint8_t sender, uint32_t stamp, float distance);