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

#ifndef USB_UART_TUNNEL_H
#define USB_UART_TUNNEL_H

/** Forward queued bytes between USB CDC and the configured hardware UART. */
extern void usb_uart_tunnel_event(void);

#endif /* USB_UART_TUNNEL_H */