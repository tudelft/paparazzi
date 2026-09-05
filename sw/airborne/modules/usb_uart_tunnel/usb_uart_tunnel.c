/*
 * Copyright (C) 2026  OpenUAS
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 */

/**
 * @file usb_uart_tunnel.c
 * @brief Nonblocking USB CDC to hardware UART forwarding.
 */

#include "std.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/usb_serial.h"
#include "modules/usb_uart_tunnel/usb_uart_tunnel.h"

#ifndef USB_TUNNEL_UART
#error USB_TUNNEL_UART not defined. Configure TUNNEL_PORT for usb_uart_tunnel.
#endif

#define USB_UART_TUNNEL_BURST_SIZE 64U

void usb_uart_tunnel_event(void)
{
  uint16_t usb_available = usb_serial.device.char_available(usb_serial.device.periph);
  uint16_t usb_to_uart_len = Min(usb_available, USB_UART_TUNNEL_BURST_SIZE);
  long uart_fd = 0;

  if (usb_to_uart_len > 0 && uart_check_free_space(&USB_TUNNEL_UART, &uart_fd, usb_to_uart_len)) {
    for (uint16_t count = 0; count < usb_to_uart_len; count++) {
      uint8_t byte = usb_serial.device.get_byte(usb_serial.device.periph);
      uart_put_byte(&USB_TUNNEL_UART, uart_fd, byte);
    }
    /* Commit the reserved batch: release the UART lock and wake its TX worker. */
    uart_send_message(&USB_TUNNEL_UART, uart_fd);
  }

  long usb_fd = 0;
  for (uint8_t count = 0; count < USB_UART_TUNNEL_BURST_SIZE; count++) {
    if (!uart_char_available(&USB_TUNNEL_UART) ||
        !usb_serial.device.check_free_space(usb_serial.device.periph, &usb_fd, 1)) {
      break;
    }
    uint8_t byte = uart_getch(&USB_TUNNEL_UART);
    usb_serial.device.put_byte(usb_serial.device.periph, usb_fd, byte);
  }
}