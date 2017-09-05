/*
 * Copyright (C) Charlelie M
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
 * @file "modules/slamdunk/slamdunk.c"
 * @author Charlelie M
 * Uses the information from the slamdunk's depth map to move forward without hitting walls
 */

#include <stdio.h>
#include <math.h>
#include "modules/slamdunk/slamdunk.h"
#include "state.h"
#include "subsystems/gps.h"
#include "firmwares/rotorcraft/navigation.h"

struct slamdunk_t slamdunk = {
  .device = (&((UART_LINK).device)),
  .msg_available = false
};
static uint8_t stereocam_msg_buf[256] __attribute__((aligned)); ///< The message buffer for the stereocamera


static void avoid_walls(float mean_left, float mean_right)
{
  float delta_heading, delta_right, delta_left;

  delta_right = (5*M_PI/18) / (mean_right - 0.2) - 0.38;
  delta_left = (5*M_PI/18) / (mean_left - 0.2) - 0.38;
  delta_heading = delta_right - delta_left;
  
  new_heading = stateGetNedToBodyEulers_f()->psi + delta_heading;

  nav_set_heading_rad(new_heading);
}

void slamdunk_parse_IMCU_LR_MEAN_DIST(void)
{

  // Check if we got some message from the Slamdunk
  pprz_check_and_parse(stereocam.device, &stereocam.transport, stereocam_msg_buf, &stereocam.msg_available);

  avoid_walls(DL_IMCU_LR_MEAN_DIST_mean_left(dl_buffer),
              DL_IMCU_LR_MEAN_DIST_mean_right(dl_buffer));
}
