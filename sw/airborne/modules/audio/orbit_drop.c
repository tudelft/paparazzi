/*
 * Copyright (C) 2026 TU Delft
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

/** @file modules/audio/orbit_drop.c
 *  Site model and payload hatch for the orbit search drop, see orbit_drop.h.
 */

#include "modules/audio/orbit_drop.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/core/commands.h"

/* The Inside* tests are generated from the flight plan's sectors, so the site
   model only exists when the active plan has them. */
#if defined(WP__OW1) && defined(WP__OE1) && defined(WP__ON1)
bool orbit_drop_obstacle(float x, float y)
{
  return InsideM4_OW(x, y) || InsideM4_OE(x, y) || InsideM4_ON(x, y) || !InsideGeofence(x, y);
}

bool orbit_drop_outside_fence(float x, float y)
{
  return !InsideGeofence(x, y);
}
#endif

void orbit_drop_hatch_close(void)
{
  commands[COMMAND_HATCH] = SERVO_HATCH_CLOSED;
}

void orbit_drop_hatch_open(void)
{
  commands[COMMAND_HATCH] = SERVO_HATCH_OPEN;
}
