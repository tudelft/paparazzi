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
 *
 * Doublet command implementation in Paparazzi
 *
 *
 */

#ifndef PPRZ_DOUBLET_H
#define PPRZ_DOUBLET_H

#include "std.h"

/**
 * Initialize with doublet_init
 */
struct doublet_t {
  float start_time_s;
  float length_s; // Amount of seconds of the chirp, excluding fade-in if applicable
  float current_value; // Value is [-1, 1]
  float current_time_s;
  float percentage_done; // t / total_length: [0, 1]
};

/**
 * Allocate and initialize a new chirp struct. set start_time to the current time
 * @param length_s: Time interval in s (starting from start_time_s) in which the doublet is active
 * @param current_time_s: Current time in s, starting point of the doublet
  */

void doublet_init(struct doublet_t *doublet, float length_s, float current_time_s);

/**
 * Reset the time of the doublet
 * @param doublet: The doublet struct pointer to reset
 * @param current_time_s: The time to set the doublet start at
 **/
void doublet_reset(struct doublet_t *doublet, float current_time_s);

/**
 * Return if the current_time is within the chirp manoeuvre
 */
bool doublet_is_running(struct doublet_t *doublet, float current_time_s);

/**
 * Calculate the value at current_time_s and update the struct value
 * @return Current value doublet->doublet_value
 */
float doublet_update(struct doublet_t *doublet, float current_time_s);

#endif
