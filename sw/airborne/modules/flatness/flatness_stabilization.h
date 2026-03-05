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

typedef struct {
    float cmd[4];
    float state[4];
    float state_filt[4];
} Act_t;

// debugging
typedef struct {
    float timestamp;
    float voltage;
    int32_t throttle;
    struct FloatQuat *quat;
    struct FloatQuat *quat_sp;
    struct FloatRates *rates;
    struct FloatRates *rates_sp;
    struct FloatRates *ang_accel_sp;
    float *ang_accel_filt;
    Act_t *act;
} dbg_t;
extern dbg_t dbg;

extern struct Fl_stabilization fl_stabilization;

extern void flatness_stabilization_init(void);
extern void flatness_stabilization_run(bool, struct StabilizationSetpoint *, struct ThrustSetpoint *, int32_t *);

#endif  // FLATNESS_STABILIZATION_H
