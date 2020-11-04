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
uint32_t wiggle_counter;
float flap_wiggle_gain;

int32_t wiggle_val[8];
int32_t deflection_duration = 200; // ticks at 500 Hz
int32_t total_wiggle_period = 2*deflection_duration*8;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_wiggle_counter(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_WIGGLE_COUNTER(trans, dev, AC_ID, &wiggle_counter, &flap_wiggle_gain);
}
#endif

void flap_wiggle_init(void)
{
  flap_wiggle_state = true;
  wiggle_counter = 0;
  flap_wiggle_gain = 0.0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIGGLE_COUNTER, send_wiggle_counter);
#endif
}

void flap_wiggle_periodic(void)
{
  wiggle_counter++;

  int32_t phase = wiggle_counter % total_wiggle_period;

  int k;

  for(k=0; k<6; k++) {

    if (phase > deflection_duration*(2*k) && phase < deflection_duration*(2*k+1) && flap_wiggle_state)
    {
      wiggle_val[k] = flap_wiggle_gain;
    } else {
      // reset wiggle_val input
      wiggle_val[k] = 0;
    }
  }
}

