/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/linux/mcu_periph/uart_arch.c
 * linux uart handling
 */
 
#include BOARD_CONFIG

#include "mcu_periph/uart.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "serial_port.h"
#include "rt_priority.h"

#include <poll.h>
#include <pthread.h>

#ifndef UART_THREAD_PRIO
#define UART_THREAD_PRIO 11
#endif

#ifndef UART_RECONNECT_INTERVAL_USEC
#define UART_RECONNECT_INTERVAL_USEC 200000
#endif

#ifndef UART_RX_BATCH_SIZE
#define UART_RX_BATCH_SIZE 256
#endif

static void uart_receive_handler(struct uart_periph *periph, int expected_fd);
static void *uart_thread(void *data __attribute__((unused)));
static void uart_periph_open(struct uart_periph *periph);
static void uart_periph_close_locked(struct uart_periph *periph);
static size_t uart_periph_index(const struct uart_periph *periph);
static pthread_mutex_t uart_mutex = PTHREAD_MUTEX_INITIALIZER;

static struct uart_periph *const uart_peripherals[] = {
#if USE_UART0
  &uart0,
#endif
#if USE_UART1
  &uart1,
#endif
#if USE_UART2
  &uart2,
#endif
#if USE_UART3
  &uart3,
#endif
#if USE_UART4
  &uart4,
#endif
#if USE_UART5
  &uart5,
#endif
#if USE_UART6
  &uart6,
#endif
#if USE_UART7
  &uart7,
#endif
#if USE_UART8
  &uart8,
#endif
};

#define UART_PERIPHERAL_COUNT (sizeof(uart_peripherals) / sizeof(uart_peripherals[0]))
static bool uart_reconnect_requested[UART_PERIPHERAL_COUNT];
static bool uart_connection_was_established[UART_PERIPHERAL_COUNT];
static uint64_t uart_connection_generation[UART_PERIPHERAL_COUNT];

//#define TRACE(fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(fmt,args...)

void uart_arch_init(void)
{
  pthread_t tid;
  if (pthread_create(&tid, NULL, uart_thread, NULL) != 0) {
    fprintf(stderr, "uart_arch_init: Could not create UART reading thread.\n");
    return;
  }
#ifndef __APPLE__
  pthread_setname_np(tid, "uart");
#endif
}

static void *uart_thread(void *data __attribute__((unused)))
{
  get_rt_prio(UART_THREAD_PRIO);

  while (1) {
    for (size_t index = 0; index < UART_PERIPHERAL_COUNT; index++) {
      struct uart_periph *periph = uart_peripherals[index];
      pthread_mutex_lock(&uart_mutex);
      if (uart_reconnect_requested[index]) {
        fprintf(stderr, "UART: connection to %s lost; reconnecting\n", periph->dev);
        uart_periph_close_locked(periph);
        uart_reconnect_requested[index] = false;
      }
      bool should_open = periph->reg_addr == NULL && periph->dev[0] != '\0' && periph->baudrate > 0;
      pthread_mutex_unlock(&uart_mutex);
      if (should_open) {
        uart_periph_open(periph);
      }
    }

    struct pollfd poll_fds[UART_PERIPHERAL_COUNT];
    struct uart_periph *polled_peripherals[UART_PERIPHERAL_COUNT];
    uint64_t polled_generations[UART_PERIPHERAL_COUNT];
    nfds_t poll_count = 0;
    pthread_mutex_lock(&uart_mutex);
    for (size_t index = 0; index < UART_PERIPHERAL_COUNT; index++) {
      struct uart_periph *periph = uart_peripherals[index];
      if (periph->reg_addr != NULL) {
        int fd = ((struct SerialPort *)periph->reg_addr)->fd;
        if (fd >= 0) {
          poll_fds[poll_count].fd = fd;
          poll_fds[poll_count].events = POLLIN;
          poll_fds[poll_count].revents = 0;
          polled_peripherals[poll_count] = periph;
          polled_generations[poll_count] = uart_connection_generation[index];
          poll_count++;
        }
      }
    }
    pthread_mutex_unlock(&uart_mutex);

    int poll_result = poll(poll_fds, poll_count, UART_RECONNECT_INTERVAL_USEC / 1000);
    if (poll_result < 0) {
      if (errno != EINTR) {
        fprintf(stderr, "uart_thread: poll failed: %s\n", strerror(errno));
      }
      continue;
    }
    if (poll_result == 0) {
      continue;
    }

    for (nfds_t index = 0; index < poll_count; index++) {
      if ((poll_fds[index].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        pthread_mutex_lock(&uart_mutex);
        struct uart_periph *periph = polled_peripherals[index];
        size_t peripheral_index = uart_periph_index(periph);
        if (periph->reg_addr != NULL
          && peripheral_index < UART_PERIPHERAL_COUNT
          && uart_connection_generation[peripheral_index] == polled_generations[index]
            && ((struct SerialPort *)periph->reg_addr)->fd == poll_fds[index].fd) {
          fprintf(stderr, "UART: connection to %s closed; reconnecting\n", periph->dev);
          uart_periph_close_locked(periph);
        }
        pthread_mutex_unlock(&uart_mutex);
      } else if ((poll_fds[index].revents & POLLIN) != 0) {
        uart_receive_handler(polled_peripherals[index], poll_fds[index].fd);
      }
    }
  }

  return 0;
}

// open serial link
// close first if already openned
static void uart_periph_open(struct uart_periph *periph)
{
  pthread_mutex_lock(&uart_mutex);

  // Another thread may have opened the port after the retry decision.
  if (periph->reg_addr != NULL) {
    pthread_mutex_unlock(&uart_mutex);
    return;
  }
  // open serial port
  struct SerialPort *port = serial_port_new();
  if (port == NULL) {
    pthread_mutex_unlock(&uart_mutex);
    return;
  }
  // use register address to store SerialPort structure pointer...
  periph->reg_addr = (void *)port;

  // TODO: Normalize quoted and unquoted UARTx_DEV definitions before adding a runtime path override.
  int ret = serial_port_open_raw(port, periph->dev, periph->baudrate);
  if (ret != 0) {
    TRACE("Error opening %s code %d\n", periph->dev, ret);
    serial_port_free(port);
    periph->reg_addr = NULL;
  } else {
    for (size_t index = 0; index < UART_PERIPHERAL_COUNT; index++) {
      if (uart_peripherals[index] == periph) {
        fprintf(stderr, "UART: %s to %s\n",
                uart_connection_was_established[index] ? "reconnected" : "connected",
                periph->dev);
        uart_connection_was_established[index] = true;
        break;
      }
    }
  }
  pthread_mutex_unlock(&uart_mutex);
}

static void uart_periph_close_locked(struct uart_periph *periph)
{
  if (periph->reg_addr == NULL) {
    return;
  }
  struct SerialPort *port = (struct SerialPort *)periph->reg_addr;
  serial_port_close(port);
  serial_port_free(port);
  periph->reg_addr = NULL;
  size_t index = uart_periph_index(periph);
  if (index < UART_PERIPHERAL_COUNT) {
    uart_connection_generation[index]++;
  }
}

static size_t uart_periph_index(const struct uart_periph *periph)
{
  for (size_t index = 0; index < UART_PERIPHERAL_COUNT; index++) {
    if (uart_peripherals[index] == periph) {
      return index;
    }
  }
  return UART_PERIPHERAL_COUNT;
}

void uart_periph_set_baudrate(struct uart_periph *periph, uint32_t baud)
{
  pthread_mutex_lock(&uart_mutex);
  if (!serial_port_baudrate_supported(baud)) {
    pthread_mutex_unlock(&uart_mutex);
    return;
  }
  periph->baudrate = baud;
  if (periph->reg_addr != NULL) {
    struct SerialPort *port = (struct SerialPort *)periph->reg_addr;
    if (serial_port_set_baudrate(port, baud) != 0) {
      uart_periph_close_locked(periph);
    }
  }
  bool should_open = periph->reg_addr == NULL && periph->dev[0] != '\0';
  pthread_mutex_unlock(&uart_mutex);
  if (should_open) {
    uart_periph_open(periph);
  }
}

void uart_periph_set_bits_stop_parity(struct uart_periph *periph, uint8_t bits, uint8_t stop, uint8_t parity)
{
  pthread_mutex_lock(&uart_mutex);
  if (periph->reg_addr == NULL) {
    pthread_mutex_unlock(&uart_mutex);
    return;
  }
  struct SerialPort *port = (struct SerialPort *)periph->reg_addr;
  if (serial_port_set_bits_stop_parity(port, bits, stop, parity) != 0) {
    uart_periph_close_locked(periph);
  }
  pthread_mutex_unlock(&uart_mutex);
}

void uart_put_byte(struct uart_periph *periph, long fd __attribute__((unused)), uint8_t data)
{
  pthread_mutex_lock(&uart_mutex);
  if (periph->reg_addr == NULL) {
    pthread_mutex_unlock(&uart_mutex);
    return;
  }

  /* write single byte to serial port */
  struct SerialPort *port = (struct SerialPort *)(periph->reg_addr);

  int ret = 0;
  do {
    ret = write((int)(port->fd), &data, 1);
  } while (ret < 0 && errno == EINTR);

  if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    TRACE("uart_put_byte: write %d failed [%d: %s]\n", data, ret, strerror(errno));
    for (size_t index = 0; index < UART_PERIPHERAL_COUNT; index++) {
      if (uart_peripherals[index] == periph) {
        uart_reconnect_requested[index] = true;
        break;
      }
    }
  }
  pthread_mutex_unlock(&uart_mutex);
}


static void __attribute__((unused)) uart_receive_handler(struct uart_periph *periph, int expected_fd)
{
  unsigned char c = 'D';

  pthread_mutex_lock(&uart_mutex);

  if (periph->reg_addr == NULL
      || ((struct SerialPort *)periph->reg_addr)->fd != expected_fd) {
    pthread_mutex_unlock(&uart_mutex);
    return;
  }

  ssize_t read_result;
    size_t received = 0;
    while (received < UART_RX_BATCH_SIZE
      && (read_result = read(expected_fd, &c, 1)) > 0) {
    uint16_t temp = (periph->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
    // check for more room in queue
    if (temp != periph->rx_extract_idx) {
      periph->rx_buf[periph->rx_insert_idx] = c;
      periph->rx_insert_idx = temp;  // update insert index
    } else {
      TRACE("uart_receive_handler: rx_buf full! discarding received byte: %x %c\n", c, c);
    }
    received++;
  }
  if (read_result == 0
      || (read_result < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
    fprintf(stderr, "UART: connection to %s closed; reconnecting\n", periph->dev);
    uart_periph_close_locked(periph);
  }
  pthread_mutex_unlock(&uart_mutex);
}

uint8_t uart_getch(struct uart_periph *p)
{
  pthread_mutex_lock(&uart_mutex);
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UART_RX_BUFFER_SIZE;
  pthread_mutex_unlock(&uart_mutex);
  return ret;
}

int uart_char_available(struct uart_periph *p)
{
  pthread_mutex_lock(&uart_mutex);
  int available = p->rx_insert_idx - p->rx_extract_idx;
  if (available < 0) {
    available += UART_RX_BUFFER_SIZE;
  }
  pthread_mutex_unlock(&uart_mutex);
  return available;
}

#if USE_UART0
void uart0_init(void)
{
  uart_periph_init(&uart0);
  strncpy(uart0.dev, STRINGIFY(UART0_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart0, UART0_BAUD);
}
#endif /* USE_UART0 */

#if USE_UART1
void uart1_init(void)
{
  uart_periph_init(&uart1);
  strncpy(uart1.dev, STRINGIFY(UART1_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart1, UART1_BAUD);
}
#endif /* USE_UART1 */

#if USE_UART2
void uart2_init(void)
{
  uart_periph_init(&uart2);
  strncpy(uart2.dev, STRINGIFY(UART2_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart2, UART2_BAUD);
}
#endif /* USE_UART2 */

#if USE_UART3
void uart3_init(void)
{
  uart_periph_init(&uart3);
  strncpy(uart3.dev, STRINGIFY(UART3_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart3, UART3_BAUD);
}
#endif /* USE_UART3 */

#if USE_UART4
void uart4_init(void)
{
  uart_periph_init(&uart4);
  strncpy(uart4.dev, STRINGIFY(UART4_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart4, UART4_BAUD);
}
#endif /* USE_UART4 */

#if USE_UART5
void uart5_init(void)
{
  uart_periph_init(&uart5);
  strncpy(uart5.dev, STRINGIFY(UART5_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart5, UART5_BAUD);
}
#endif /* USE_UART5 */

#if USE_UART6
void uart6_init(void)
{
  uart_periph_init(&uart6);
  strncpy(uart6.dev, STRINGIFY(UART6_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart6, UART6_BAUD);
}
#endif /* USE_UART6 */

#if USE_UART7
void uart7_init(void)
{
  uart_periph_init(&uart7);
  strncpy(uart7.dev, STRINGIFY(UART7_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart7, UART7_BAUD);
}
#endif /* USE_UART7 */

#if USE_UART8
void uart8_init(void)
{
  uart_periph_init(&uart8);
  strncpy(uart8.dev, STRINGIFY(UART8_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart8, UART8_BAUD);
}
#endif /* USE_UART8 */
