/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/gps/gps_lost_time.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module that keeps track of GPS lost time when fix is not valid
 */

#include "modules/gps/gps_lost_time.h"
#include "modules/gps/gps.h"

uint32_t gps_lost_time;

void init_gps_lost_time(void)
{
  // your init code here
  gps_lost_time = 0;
}

void periodic_gps_lost_time(void)
{
  // @1HZ
  if (gps.fix >= GPS_FIX_3D) {
    gps_lost_time = 0;
  } else {
    gps_lost_time++;
  }
}



