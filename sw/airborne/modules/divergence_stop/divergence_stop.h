#ifndef DIVERGENCE_STOP_H
#define DIVERGENCE_STOP_H

#include <std.h>

// Demo with own guidance_h
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

// But re-using an existing altitude-hold controller
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_RC_DIRECT

extern float divergence_stop_threshold;
extern float divergence_stop_avoidance_pitch;
extern float divergence_stop_avoidance_duration;
float last_time_exceeded;

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

extern void divergence_stop_init(void);
extern void divergence_stop_periodic(void);


#endif


