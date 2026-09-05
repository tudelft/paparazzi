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
 * @brief Bidirectional USB CDC to hardware UART tunnel.
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
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/usb_serial.h"

#ifndef USB_TUNNEL_UART
#error USB_TUNNEL_UART not defined. Add <configure name="TUNNEL_PORT" value="UARTx"/>
#endif

PRINT_CONFIG_VAR(USB_TUNNEL_UART)

PRINT_CONFIG_VAR(TUNNEL_RX_LED)
PRINT_CONFIG_VAR(TUNNEL_TX_LED)

/** Minimum duration of the optional traffic indication in milliseconds. */
#define BLINK_MIN 100

/** Maximum bytes forwarded in one direction before servicing the other. */
#define USB_TUNNEL_BURST_SIZE 64U

/**
 * @brief Forward queued bytes in both directions without blocking.
 *
 * Host-to-UART traffic is handled first so configuration commands are not
 * delayed by a busy peripheral. Bounded batches keep both directions
 * responsive under sustained traffic.
 */
static inline void tunnel_event(void)
{
#if LED_AVAILABLE(TUNNEL_RX_LED)
  static uint32_t rx_time = 0;
  if (get_sys_time_msec() > rx_time + BLINK_MIN) {
    LED_OFF(TUNNEL_RX_LED);
  }
#endif
#if LED_AVAILABLE(TUNNEL_TX_LED)
  static uint32_t tx_time = 0;
  if (get_sys_time_msec() > tx_time + BLINK_MIN) {
    LED_OFF(TUNNEL_TX_LED);
  }
#endif

  uint16_t usb_available = usb_serial.device.char_available(usb_serial.device.periph);
  uint16_t usb_to_uart_len = Min(usb_available, USB_TUNNEL_BURST_SIZE);
  long uart_fd = 0;
  if (usb_to_uart_len > 0 && uart_check_free_space(&USB_TUNNEL_UART, &uart_fd, usb_to_uart_len)) {
    for (uint16_t count = 0; count < usb_to_uart_len; count++) {
      uint8_t byte = usb_serial.device.get_byte(usb_serial.device.periph);
      uart_put_byte(&USB_TUNNEL_UART, uart_fd, byte);
#if LED_AVAILABLE(TUNNEL_TX_LED)
      LED_ON(TUNNEL_TX_LED);
      tx_time = get_sys_time_msec();
#endif
    }
  /* Commit the reserved batch: release the UART lock and wake its TX worker. */
    uart_send_message(&USB_TUNNEL_UART, uart_fd);
  }

  long usb_fd = 0;
  for (uint8_t count = 0; count < USB_TUNNEL_BURST_SIZE; count++) {
    if (!uart_char_available(&USB_TUNNEL_UART) ||
        !usb_serial.device.check_free_space(usb_serial.device.periph, &usb_fd, 1)) {
      break;
    }
    uint8_t byte = uart_getch(&USB_TUNNEL_UART);
    usb_serial.device.put_byte(usb_serial.device.periph, usb_fd, byte);
#if LED_AVAILABLE(TUNNEL_RX_LED)
    LED_ON(TUNNEL_RX_LED);
    rx_time = get_sys_time_msec();
#endif
  }
}

int main(void)
{
  mcu_init();

  /* Match the UART worker priority to keep high-baud tunnel traffic responsive. */
  chThdSetPriority(NORMALPRIO + 1);

  while (1) {
    mcu_event();
    tunnel_event();
  }

  return 0;
}
