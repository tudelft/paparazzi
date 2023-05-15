/*
 * Copyright (C) 2023 Tomaso De Ponti <tmldeponti@tudelft.nl>
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

/** @file "modules/wind_tunnel/oneloop_indi.h"
 * @author Tomaso De Ponti <tmldeponti@tudelft.nl>
 * One loop (Guidance + Stabilization) INDI controller for the rotating wing drone RW3C
 */

#ifndef ONELOOP_INDI_H
#define ONELOOP_INDI_H

extern void oneloop_indi_init(void);
extern void oneloop_indi_enter(void);
extern void oneloop_indi_set_failsafe_setpoint(void);
extern void oneloop_indi_attitude_run(struct FloatVect3 att_ref, struct FloatVect3 att_d_ref, struct FloatVect3 att_2d_ref, struct FloatVect3 att_3d_ref, bool in_flight);
#endif  // ONELOOP_INDI_H
