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

/** @file "modules/flatness/flatness_stabilization.c"
 * @author Evangelos Ntouros <e.ntouros@tudelft.nl>
 * A differential flatness-based INDI stabilization controller.
 */

#include "modules/flatness/flatness_stabilization.h"
#include "modules/actuators/actuators.h"

// #include "modules/datalink/telemetry.h"

struct Fl_stabilization fl_stabilization;

void flatness_stabilization_run(struct ThrustSetpoint *thrust, int32_t *cmd)
{
  for (int i = 0; i < 4; i++) {
    actuators_pprz[i] = (int16_t) 2000;
  }

  cmd[COMMAND_THRUST] = (actuators_pprz[0] + actuators_pprz[1] + actuators_pprz[2] + actuators_pprz[3])/4;

}