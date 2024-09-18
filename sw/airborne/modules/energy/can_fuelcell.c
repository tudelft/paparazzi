/*
 * Copyright (C) 2024 Freek van Tienen <freek.v.tienen@gmail.com>
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
#include "uavcan/uavcan.h"


/* uavcan EQUIPMENT_ESC_STATUS message definition */
#define UAVCAN_EQUIPMENT_FUELCELL_STATUS_MAX_SIZE 11
#define UAVCAN_EQUIPMENT_FUELCELL_STATUS_SIGNATURE (0x475459FF3AA36FE5ULL)
#define UAVCAN_EQUIPMENT_FUELCELL_STATUS_ID 1141


/* Fuel-Cell */
struct uavcan_equipment_fuelcell {
  bool can_fuelcell_has_new_data;

  uint8_t pressure;
  float press_reg;
  float volt_bat;
  float power_out;
  float power_cell;
  float power_batt;
  uint8_t state;
  uint8_t error;
  uint8_t suberror;
};
static struct uavcan_equipment_fuelcell can_fuelcell_data = {false, 67, 0.78, 45.9, 1457.0, 1234.0, -223.0, 2, 12, 7};

#if !(USE_NPS)

/* CAN feedback */
static uavcan_event fuelcell_uavcan_ev;

static void fuelcell_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{

  uint32_t bit_ofs = 0;
  canardDecodeScalar(transfer, bit_ofs, 7, false, &can_fuelcell_data.pressure);
  bit_ofs += 7;
  canardDecodeScalar(transfer, bit_ofs, 8, false, &can_fuelcell_data.press_reg);
  bit_ofs += 8;
  canardDecodeScalar(transfer, bit_ofs, 10, false, &can_fuelcell_data.volt_bat);
  bit_ofs += 10;
  canardDecodeScalar(transfer, bit_ofs, 14, false, &can_fuelcell_data.power_out);
  bit_ofs += 14;
  canardDecodeScalar(transfer, bit_ofs, 13, false, &can_fuelcell_data.power_cell);
  bit_ofs += 13;
  canardDecodeScalar(transfer, bit_ofs, 16, true, &can_fuelcell_data.power_batt);
  bit_ofs += 16;
  canardDecodeScalar(transfer, bit_ofs, 4, false, &can_fuelcell_data.state);
  bit_ofs += 4;
  canardDecodeScalar(transfer, bit_ofs, 8, false, &can_fuelcell_data.error);
  bit_ofs += 8;
  canardDecodeScalar(transfer, bit_ofs, 8, false, &can_fuelcell_data.suberror);
  bit_ofs += 8;

  can_fuelcell_data.can_fuelcell_has_new_data = true;

}
#endif


extern void can_fuelcell_periodic(void)
{
#if USE_NPS
    // Simulate data
    can_fuelcell_data.can_fuelcell_has_new_data = true;
#endif
}


/* Event function to read UART message and forward to downlink */
void can_fuelcell_event(void) {

  if (can_fuelcell_data.can_fuelcell_has_new_data) {
    // Forward
    can_fuelcell_data.can_fuelcell_has_new_data = false;
    DOWNLINK_SEND_FUELCELL(DefaultChannel, DefaultDevice, &can_fuelcell_data.pressure, &can_fuelcell_data.press_reg,
      &can_fuelcell_data.volt_bat, &can_fuelcell_data.power_out, &can_fuelcell_data.power_cell,
      &can_fuelcell_data.power_batt, &can_fuelcell_data.state, &can_fuelcell_data.error, &can_fuelcell_data.suberror);
  }
}


void can_fuelcell_init(void)
{
#if !(USE_NPS)

    // Bind uavcan BATTERYINFO message from EQUIPMENT.POWER
  uavcan_bind(UAVCAN_EQUIPMENT_FUELCELL_STATUS_ID, UAVCAN_EQUIPMENT_FUELCELL_STATUS_SIGNATURE, &fuelcell_uavcan_ev,
              &fuelcell_uavcan_cb);

#endif
}