/*
 * Copyright (C) 2009  Martin Mueller
 *               2014  Felix Ruess <felix.ruess@gmail.com>
 *               2026  OpenUAS
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
 *
 */

/**
 * @file usb_tunnel.c
 * @brief Standalone USB CDC to hardware UART tunnel firmware.
 *
 * This setup firmware exposes the UART selected by @c TUNNEL_PORT as a raw USB
 * serial connection. It allows a host tool to communicate directly with an
 * installed peripheral, such as a GNSS receiver or telemetry modem.
 *
 * Configure the target in the airframe with @c TUNNEL_PORT and
 * @c TUNNEL_BAUD. The host serial setting must match the compiled baud rate.
 * Connect peripheral TX to flight-controller RX, peripheral RX to
 * flight-controller TX, and use a common ground.
 */

#include "std.h"
#include "mcu.h"
#include "modules/usb_uart_tunnel/usb_uart_tunnel.h"

#ifndef USB_TUNNEL_UART
#error USB_TUNNEL_UART not defined. Add <configure name="TUNNEL_PORT" value="UARTx"/>
#endif

PRINT_CONFIG_VAR(USB_TUNNEL_UART)

int main(void)
{
  mcu_init();

  /* Match the UART worker priority to keep high-baud tunnel traffic responsive. */
  chThdSetPriority(NORMALPRIO + 1);

  while (1) {
    mcu_event();
    usb_uart_tunnel_event();
  }

  return 0;
}
