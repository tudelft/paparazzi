/*
 * Copyright (C) 2026 Evangelos Ntouros <e.ntouros@tudelft.nl>
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

/** @file "modules/flatness/flatness_stabilization.h"
 * @author Evangelos Ntouros <e.ntouros@tudelft.nl>
 * A differential flatness-based INDI stabilization controller.
 */

#ifndef FLATNESS_STABILIZATION_H
#define FLATNESS_STABILIZATION_H

#include <stdint.h>
#include "firmwares/rotorcraft/stabilization.h"

struct Fl_stabilization {
    int32_t cmd[COMMANDS_NB];
    struct StabilizationSetpoint rc_sp;
    struct AttitudeRCInput rc_in;
};

// debugging
typedef struct {
    float *u_cmd;
    float *u;
    float *u_filt;
} dbg_t;
extern dbg_t dbg;

extern struct Fl_stabilization fl_stabilization;

extern void flatness_stabilization_init(void);
extern void flatness_stabilization_run(bool, struct StabilizationSetpoint *, struct ThrustSetpoint *, int32_t *);

#endif  // FLATNESS_STABILIZATION_H
