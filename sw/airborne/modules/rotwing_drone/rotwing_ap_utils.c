/*
 * Copyright (C) 2025 Noah Wechtler <noahwechtler@tudelft.nl>
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

/** @file "sw/airborne/modules/rotwing_drone/rotwing_ap_utils.c"
 * @author Noah Wechtler <noahwechtler@tudelft.nl>
 * helper functions for the rotating wing drone generated autopilot
 */

#include "modules/rotwing_drone/rotwing_ap_utils.h"
#include "firmwares/rotorcraft/autopilot_utils.h"
#include "modules/core/commands.h"

struct rotwing_ap_utils_t rotwing_ap = {0};

void rotwing_ap_utils_init(void) {
  rotwing_ap.ctrl_mode = ROTWING_CTRL_MODE_INDI;
  rotwing_ap.h_mode = GUIDANCE_INDI_HYBRID_H_SPEED;
  rotwing_ap.v_mode = GUIDANCE_INDI_HYBRID_V_SPEED;
}

void rotwing_ap_utils_configure_indi(void) {
  rotwing_ap.h_mode = GUIDANCE_INDI_HYBRID_H_SPEED;
  rotwing_ap.v_mode = GUIDANCE_INDI_HYBRID_V_SPEED;
}

void rotwing_ap_utils_configure_andi(void) {
  rotwing_ap.h_mode = GUIDANCE_ONELOOP_H_SPEED;
  rotwing_ap.v_mode = GUIDANCE_ONELOOP_V_SPEED;
}

void set_rotorcraft_commands(pprz_t *cmd_out, int32_t *cmd_in, bool in_flight __attribute__((unused)), bool motors_on __attribute__((unused)))
{
#if !ROTORCRAFT_IS_HELI
#if !ROTORCRAFT_COMMANDS_YAW_ALWAYS_ENABLED && defined(COMMAND_YAW)
  if (!in_flight) {
    cmd_in[COMMAND_YAW] = 0;
  }
#endif
  if (!motors_on) {
    cmd_in[COMMAND_THRUST] = 0;
  }
#endif

  for (uint8_t i = 0; i < COMMANDS_NB; i++) {
    cmd_out[i] = cmd_in[i];
  }
}