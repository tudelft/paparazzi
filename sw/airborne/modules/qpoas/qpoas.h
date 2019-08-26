/*
 * Copyright (C) mavlab
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
 * @file "modules/qpoas/qpoas.h"
 * @author mavlab
 * opti
 */

#ifndef QPOAS_H
#define QPOAS_H


#include <sys/time.h>


#define MAX_N 100

#ifdef __cplusplus
extern "C" {
#endif

extern void optimal_enter(void);
extern void periodic_10Hz_demo(void);
extern void qp_init(void);
extern void replan(void);
extern void dronerace_get_cmd(float* roll, float* pitch);

#ifdef __cplusplus
}
#endif

#endif

