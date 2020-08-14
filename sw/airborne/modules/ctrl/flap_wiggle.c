/*
 * Copyright (C) 2020 Ewoud Smeur <ewoud_smeur@msn.com>
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

#include "modules/ctrl/flap_wiggle.h"
#include "generated/airframe.h"

/* The frequency of this module is 500 hz. It has a counter which
 * functions as clock to generate the signals. */

bool flap_wiggle_state;
uint32_t counter;
float flap_wiggle_gain;

int32_t wiggle_val[8];

void flap_wiggle_init(void)
{
  flap_wiggle_state = 0;
  counter = 0;
  flap_wiggle_gain = 0.0;
}

void flap_wiggle_periodic(void)
{
  counter++;

  int32_t phase = counter % 3200;

  int k;

  for(k=0; k<6; k++) {
    wiggle_val[k] = 0;
  }

  for(k=0; k<6; k++) {
    if (phase < 200) {
      wiggle_val[0] = flap_wiggle_gain;
    }
    if (phase > 400 && phase < 600) {
      wiggle_val[1] = flap_wiggle_gain;
    }
    if (phase > 800 && phase < 1000) {
      wiggle_val[2] = flap_wiggle_gain;
    }
    if (phase > 1200 && phase < 1400) {
      wiggle_val[3] = flap_wiggle_gain;
    }
    if (phase > 1600 && phase < 1800) {
      wiggle_val[4] = flap_wiggle_gain;
    }
    if (phase > 2000 && phase < 2200) {
      wiggle_val[5] = flap_wiggle_gain;
    }
    if (phase > 2400 && phase < 2600) {
      wiggle_val[6] = flap_wiggle_gain;
    }
    if (phase > 2800 && phase < 3000) {
      wiggle_val[7] = flap_wiggle_gain;
    }
  }
}

