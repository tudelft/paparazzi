/*
 * Copyright (C) 2022 Gervase Lovell-Prescod <gervase.prescod@gmail.com>
 */

/** @file modules/nav/nav_pivot_takeoff_landing.h
 *
 */

#ifndef NAV_PIVOT_TAKEOFF_LANDING_H
#define NAV_PIVOT_TAKEOFF_LANDING_H

#include "std.h"
#include "paparazzi.h"

extern struct FloatEulers guidance_euler_cmd;

struct nav_pivot{
	float initial;
	float duration;
	float goal;
};

extern void nav_pivot_ramp_init(void);
extern bool nav_pivot_ramp(void);

#endif  /* NAV_PIVOT_TAKEOFF_LANDING_H */
