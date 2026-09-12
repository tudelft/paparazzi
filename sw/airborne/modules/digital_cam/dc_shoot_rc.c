/*
 * Copyright (C) 2014 Eduardo Lavratti <agressiva@hotmail.com>
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

/** @file modules/digital_cam/dc_shoot_rc.c
 * Digital Camera remote shoot using radio channel.
 *
 * Use radio channel to take a picture.
 * Only works with fixedwing firmware.
 */

#include "dc_shoot_rc.h"
#include "modules/radio_control/radio_control.h"
#include "generated/modules.h"
#include "dc.h"

#ifndef DC_RADIO_SHOOT
#error "You need to define DC_RADIO_SHOOT to a RADIO_xxx channel to use this module"
#endif

#define DC_RADIO_SHOOT_THRESHOLD 3000

void dc_shoot_rc_periodic(void)
{
  static uint8_t ticks_until_shot = 0;

  if (radio_control.status != RC_OK ||
      radio_control_get(DC_RADIO_SHOOT) <= DC_RADIO_SHOOT_THRESHOLD) {
    ticks_until_shot = 0;
    return;
  }

  if (ticks_until_shot == 0) {
    dc_send_command(DC_SHOOT);
    ticks_until_shot = 4;
  }
  ticks_until_shot--;
}
