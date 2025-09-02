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

/** @file "sw/airborne/modules/rotwing_drone/rotwing_ap_utils.h"
 * @author Noah Wechtler <noahwechtler@tudelft.nl>
 * helper functions for the rotating wing drone generated autopilot
 */

#ifndef ROTWING_AP_UTILS
#define ROTWING_AP_UTILS

#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"

enum rotwing_ctrl_mode {
  ROTWING_CTRL_MODE_INDI,
  ROTWING_CTRL_MODE_ANDI
};

struct rotwing_ap_utils_t {
  enum rotwing_ctrl_mode ctrl_mode;
  uint8_t h_mode;
  uint8_t v_mode;
};

extern struct rotwing_ap_utils_t rotwing_ap;

void rotwing_ap_utils_init(void);
void rotwing_ap_utils_configure_indi(void);
void rotwing_ap_utils_configure_andi(void);

#endif