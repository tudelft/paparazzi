#pragma once
#include "std.h"
#include "mcu_periph/sys_time.h"
#define ABI_BROADCAST 0
typedef int abi_event;
extern uint8_t test_agl_sender;
extern void (*test_agl_callback)(uint8_t, uint32_t, float);
#define AbiBindMsgAGL(sender, event, callback) ((void)(event), test_agl_sender = (sender), test_agl_callback = (callback))