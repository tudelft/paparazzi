/*
 * Copyright (C) 2021 Guido de Croon <g.c.h.e.decroon@tudelft.nl>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/ins/ins_flow.h
 *
 * "Inertial" navigation system.
 */

#ifndef INS_FLOW_H
#define INS_FLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_int_cmpl_quat.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/ins.h"

extern void ins_flow_init(void);
extern void ins_flow_update(void);

extern bool reset_filter;
extern bool run_filter;
extern bool use_filter;

#ifdef __cplusplus
}
#endif

#endif /* INS_FLOW_H */
