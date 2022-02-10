/*
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
 * @file "modules/system_identification/pprz_doublet.c"
 * @author Tomaso De Ponti
 * Mathematical implementation of the doublet
 */
#include "pprz_doublet.h"
#include "std.h"

void doublet_init(struct doublet_t *doublet, float length_s, float current_time_s)
{
  doublet->length_s = length_s;
  doublet->start_time_s = current_time_s;
  doublet->current_value = 0;
  doublet->percentage_done = 0;
}

void doublet_reset(struct doublet_t *doublet, float current_time_s)
{
  doublet->current_time_s = current_time_s;
  doublet->start_time_s = current_time_s;
  doublet->current_value = 0;
  doublet->percentage_done = 0;
}

bool doublet_is_running(struct doublet_t *doublet, float current_time_s)
{
  float t = current_time_s - doublet->start_time_s;
  return (t >= 0) && (t <= doublet->length_s);
}

float doublet_update(struct doublet_t *doublet, float current_time_s)
{
  if (!doublet_is_running(doublet, current_time_s)) { // Outside the doublet interval, return 0
    doublet->current_value = 0.0f;
    return 0;
  }

  float t = current_time_s - doublet->start_time_s; // Time since the start of the doublet
  doublet->current_time_s = current_time_s;
  // Protect against divide by zero
  if (doublet->length_s <= 0) {
    doublet->length_s = 0.01;
  }
  doublet->percentage_done = t / doublet->length_s;
  
  if (t <= doublet->length_s/2.0) {
    doublet->current_value = 1.0;
  } else{
    doublet->current_value = -2.0;
  }
  return doublet->current_value;
}
