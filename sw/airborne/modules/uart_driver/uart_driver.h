/*
 * Copyright (C) nilay
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
 * @file "modules/uart_driver/uart_driver.h"
 * @author nilay
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mcu_periph/uart.h"

#define UART_MAX_LEN 30

typedef enum {
  ACK_FRAME = 0,
  DATA_FRAME,
  OTHER_FRAME
} frame_type_t;

// decoder state
typedef enum {
  JTSN_SYNC = 0,
  JTSN_INFO,
  JTSN_DATA,
  JTSN_ERR_CHK,  // TODO: shift to crc
  JTSN_RX_OK,
  JTSN_RX_ERR
} jetson_state_t;

typedef struct __attribute__((packed)) {
  float pot;
  uint8_t but0:1;
  uint8_t but1:1;
  uint8_t but2:1;
  uint8_t but3:1;
  uint8_t but4:1;
} data_frame_t;

typedef struct __attribute__((packed)) {
  uint8_t packet_type;
  // packet_length is counted from start byte to the end of data frame
  // in this case: 1 start byte + 2 info bytes + 5 byte of data = 8 bytes
  uint8_t packet_length; 
} info_frame_t;


typedef struct __attribute__((packed)) {
  info_frame_t info;
  data_frame_t data;
} uart_packet_t;

extern data_frame_t uart_rx_buffer;

extern void uart_driver_rx_event(void);
extern void uart_driver_init(void);
extern void uart_driver_tx_loop(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
