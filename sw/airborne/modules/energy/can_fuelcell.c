/*
 * Copyright (C) 2020 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/energy/can_fuelcell.c"
 * @author F. van Tienen
 * Fuel-cell data through CAN bus
 */

#include "modules/energy/can_fuelcell.h"
#include "modules/datalink/telemetry.h"


bool can_fuelcell_has_new_data = false;


extern void can_fuelcell_periodic(void)
{
  can_fuelcell_has_new_data = true;
}


/* Event function to read UART message and forward to downlink */
void can_fuelcell_event(void) {

  uint8_t pressure = 56;
  float press_reg = 0.78;
  float volt_bat = 45.9;
  float power_out = 1457.0;
  float power_cell = 1234.0;
  float power_batt = -223.0;
  uint8_t state = 3;
  uint8_t error = 0;
  uint8_t suberror = 0;

#if NPS
    // Simulate data

#else
// Read from CAN

#endif

  if (can_fuelcell_has_new_data) {
    // Forward
    can_fuelcell_has_new_data = false;
    DOWNLINK_SEND_FUELCELL(DefaultChannel, DefaultDevice, &pressure, &press_reg, &volt_bat, &power_out, 
      &power_cell, &power_batt, &state, &error, &suberror);
  }
}
