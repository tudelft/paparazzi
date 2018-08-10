/*
 * Copyright (C) Joost Meulenbeld
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/helicopter/sys_id_doublet.h"
 * @author Joost Meulenbeld
 * System identification doublet, add doublet signal to controller setpoint to measure controller response.
 * 
 * Implementation of doublet function: 
 *  out = 
 *     1    if (0   <= t_since_start < l/2)  
 *    -1    if (l/2 <= t_since_start < l  ) 
 *     0    otherwise
 * 
 * Usage: add module to airframe file, add sys_id_doublet_add_rate/sys_id_doublet_add_attitude to
 * the beginning of the stabilization_*_run() function to alter the setpoint for the controller
 */

#ifndef SYS_ID_DOUBLET_H
#define SYS_ID_DOUBLET_H

#include <std.h>
#include <stdbool.h>
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"

extern uint8_t doublet_axis; // 0: roll, 1: pitch, 2: yaw
extern bool  doublet_active;
extern float doublet_amplitude_deg;
extern float doublet_length_s;

extern void sys_id_doublet_init(void);

// Handler for changing the doublet_active variable in the GCS
extern void sys_id_doublet_doublet_activate_handler(uint8_t activate);

// Add the current doublet value to rates_cmd at the selected axis
extern void sys_id_doublet_add_rate(struct FloatRates* rates_sp);

// Add the current doublet value to att_cmd at the selected axis
extern void sys_id_doublet_add_attitude(struct Int32Quat* att_sp_quat);

#endif // SYS_ID_DOUBLET_H
