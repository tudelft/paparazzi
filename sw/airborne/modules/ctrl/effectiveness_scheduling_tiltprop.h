/*
 * Copyright (C) 2025
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/ctrl_module_innerloop_demo.c
 * @brief example empty controller
 *
 * Implements an example simple rate controller in a module.
 */

#ifndef EFFECTIVENESS_SCHEDULING_TILTPROP_H_
#define EFFECTIVENESS_SCHEDULING_TILTPROP_H_

#include <std.h>


struct MassProperties {
	float mass;
	float I_xx;
	float I_yy;
	float I_zz;
};

struct MotorCoefficients {
	float k1;
	float k2;
	float k3;
};

extern float c_delta_a;
extern float cda_offset;

extern float cde_offset;
extern float c_delta_e;
extern float mapping;

extern struct MassProperties mass_property;
extern float thrust_lower_lim;
extern float airspeed_scaling;

extern void eff_sched_tiltprop_init(void);
extern void eff_sched_tiltprop_periodic(void);

#endif /* EFFECTIVENESS_SCHEDULING_TILTPROP_H_ */
