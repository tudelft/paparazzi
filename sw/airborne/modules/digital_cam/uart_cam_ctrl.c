/*
 * Copyright (C) OpenUAS
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

/**
 * @file modules/digital_cam/uart_cam_ctrl.c
 * @brief CATIA camera control and payload exchange over a serial link.
 *
 * @details This module translates Paparazzi digital-camera commands into CATIA
 * frames, attaches the aircraft pose to capture requests, parses camera status
 * and payload replies, and optionally streams pose samples. On flight hardware,
 * `CAMERA_LINK` is the ordinary UART selected by `CAMERA_PORT`.
 *
 * Linux simulation targets use a private, nonblocking ::link_device adapter.
 * The adapter prefers the local CATIA pseudo-terminal (normally
 * `/tmp/catia-sim`) and falls back to a desk FTDI device (normally
 * `/dev/ttyUSB0`). Keeping this policy here is intentional: endpoint priority
 * is camera-module behavior, not a property of Paparazzi's generic UART
 * architecture. It also avoids changing a generic UART peripheral while its
 * Linux receive thread may be using it.
 *
 * @par Endpoint boundary invariant
 * Each endpoint is a different CATIA byte stream and may represent a different
 * peer and clock domain. Switching therefore discards queued RX/TX bytes,
 * partial parser state, and pose-clock alignment. Carrying any of that state
 * across the boundary could splice frames from two peers or timestamp samples
 * against the wrong clock. Commands queued at the instant of a switch are
 * intentionally not replayed; callers may issue a later command after
 * reconnection.
 *
 * @par Scheduling model
 * The simulation adapter has no worker thread. The module event and periodic
 * callbacks perform zero-timeout I/O service, while endpoint reconciliation is
 * rate-limited. This preserves Paparazzi's single-threaded module semantics and
 * keeps parser and ring-buffer state free from concurrent access.
 */

#include "uart_cam_ctrl.h"
#include "generated/airframe.h"

// Include Standard Camera Control Interface
#include "modules/digital_cam/dc.h"

// Telemetry
#include "modules/datalink/telemetry.h"

#include BOARD_CONFIG

// Communication
#include "modules/digital_cam/catia/protocol.h"

#include "state.h"

#ifdef DIGITAL_CAM_UART_SIM_BACKEND
#include "arch/linux/serial_port.h"
#include "mcu_periph/sys_time.h"

#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <unistd.h>

/** Preferred CATIA endpoint created by the local simulation launcher. */
#ifndef DIGITAL_CAM_UART_LOCAL_DEVICE
#define DIGITAL_CAM_UART_LOCAL_DEVICE /tmp/catia-sim
#endif
/** Physical desk UART used while local CATIA is unavailable. */
#ifndef DIGITAL_CAM_UART_FALLBACK_DEVICE
#define DIGITAL_CAM_UART_FALLBACK_DEVICE /dev/ttyUSB0
#endif
/** Interval between endpoint-priority checks, in microseconds. */
#ifndef DIGITAL_CAM_UART_RECONNECT_INTERVAL_USEC
#define DIGITAL_CAM_UART_RECONNECT_INTERVAL_USEC 200000
#endif
/** RX ring capacity; one slot remains unused to distinguish full from empty. */
#ifndef DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE
#define DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE 1024
#endif
/** TX ring capacity; complete CATIA frames must fit before they are enqueued. */
#ifndef DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE
#define DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE 1024
#endif
/** Maximum received bytes processed by one service pass. */
#ifndef DIGITAL_CAM_UART_SIM_RX_BATCH_SIZE
#define DIGITAL_CAM_UART_SIM_RX_BATCH_SIZE 256
#endif

static void digital_cam_uart_reset_protocol_state(void);

/** Serial endpoint currently owned by the simulation adapter. */
enum digital_cam_uart_endpoint {
  DIGITAL_CAM_UART_ENDPOINT_NONE,     /**< No usable endpoint is open. */
  DIGITAL_CAM_UART_ENDPOINT_LOCAL,    /**< Preferred local CATIA pseudo-terminal. */
  DIGITAL_CAM_UART_ENDPOINT_FALLBACK  /**< Physical desk serial adapter. */
};

/** Private state implementing the simulation target's ::link_device contract. */
struct digital_cam_uart_sim_periph {
  struct link_device device;                /**< Transport-neutral CATIA interface. */
  struct SerialPort port;                   /**< Currently open Linux serial port. */
  enum digital_cam_uart_endpoint endpoint;  /**< Identity of `port`, or NONE. */
  uint8_t rx_buf[DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE]; /**< Bytes ready for the parser. */
  uint16_t rx_insert_idx;                   /**< Producer position in `rx_buf`. */
  uint16_t rx_extract_idx;                  /**< Consumer position in `rx_buf`. */
  uint8_t tx_buf[DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE]; /**< Bytes awaiting OS writes. */
  uint16_t tx_insert_idx;                   /**< Producer position in `tx_buf`. */
  uint16_t tx_extract_idx;                  /**< Consumer position in `tx_buf`. */
  uint32_t last_reconnect_us;               /**< Wrap-safe reconciliation timestamp. */
};

static struct digital_cam_uart_sim_periph digital_cam_uart_sim_link;

static int digital_cam_uart_sim_check_free_space(void *periph, long *fd, uint16_t len);
static void digital_cam_uart_sim_put_byte(void *periph, long fd, uint8_t data);
static void digital_cam_uart_sim_put_buffer(void *periph, long fd, const uint8_t *data, uint16_t len);
static void digital_cam_uart_sim_send_message(void *periph, long fd);
static int digital_cam_uart_sim_char_available(void *periph);
static uint8_t digital_cam_uart_sim_get_byte(void *periph);
static void digital_cam_uart_sim_set_baudrate(void *periph, uint32_t baudrate);
static void digital_cam_uart_sim_init(void);
static void digital_cam_uart_sim_periodic(void);
#endif

#ifndef DIGITAL_CAM_UART_POSE_STREAM
#define DIGITAL_CAM_UART_POSE_STREAM 0
#endif

#if DIGITAL_CAM_UART_POSE_STREAM
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#if USE_GPS
#include "modules/gps/gps.h"
#endif
static uint32_t pose_sequence;
static union catia_clock_request_union pose_clock_token;
static uint32_t last_clock_reply_us;
static bool clock_reply_sent;
static void send_pose_sample(void);
static void reply_clock_request(void);
#endif

/**
 * @brief Reset state that belongs to one CATIA peer connection.
 *
 * @details Parser progress and clock tokens cannot survive a simulation
 * endpoint switch: the replacement endpoint may be a different process with a
 * different byte stream and monotonic clock. Hardware also calls this during
 * initialization to start from a known protocol boundary.
 */
static void digital_cam_uart_reset_protocol_state(void)
{
  catia_protocol = (struct catia_transport){0};
#if DIGITAL_CAM_UART_POSE_STREAM
  pose_clock_token = (union catia_clock_request_union){0};
  last_clock_reply_us = 0;
  clock_reply_sent = false;
#endif
}

#if FIXEDWING_FIRMWARE
#include "modules/nav/common_nav.h"
#endif


#define CameraLinkDev (&((CAMERA_LINK).device))
#define CameraLinkTransmit(c) CameraLinkDev->put_byte(CameraLinkDev->periph, 0, c)
#define CameraLinkChAvailable() CameraLinkDev->char_available(CameraLinkDev->periph)
#define CameraLinkGetch() CameraLinkDev->get_byte(CameraLinkDev->periph)
#define CameraLinkCheckFreeSpace(len) CameraLinkDev->check_free_space(CameraLinkDev->periph, NULL, len)

#ifdef DIGITAL_CAM_UART_SIM_BACKEND
/**
 * @brief Return the configured path for an endpoint.
 * @param endpoint LOCAL for the preferred pseudo-terminal, otherwise FALLBACK.
 * @return Static, stringified build-time device path.
 */
static const char *digital_cam_uart_sim_endpoint_path(enum digital_cam_uart_endpoint endpoint)
{
  return endpoint == DIGITAL_CAM_UART_ENDPOINT_LOCAL
         ? STRINGIFY(DIGITAL_CAM_UART_LOCAL_DEVICE) : STRINGIFY(DIGITAL_CAM_UART_FALLBACK_DEVICE);
}

/**
 * @brief Discard all buffered transport and peer-specific protocol state.
 * @param periph Simulation transport instance.
 *
 * @details RX and TX are cleared together because they describe one ordered
 * conversation. Retaining either side while changing peers could complete a
 * partial frame with bytes from the wrong endpoint.
 */
static void digital_cam_uart_sim_clear_queues(struct digital_cam_uart_sim_periph *periph)
{
  periph->rx_insert_idx = 0;
  periph->rx_extract_idx = 0;
  periph->tx_insert_idx = 0;
  periph->tx_extract_idx = 0;
  digital_cam_uart_reset_protocol_state();
}

/**
 * @brief Close the active endpoint and establish a clean disconnected state.
 * @param periph Simulation transport instance.
 */
static void digital_cam_uart_sim_close(struct digital_cam_uart_sim_periph *periph)
{
  if (periph->port.fd >= 0) {
    serial_port_close(&periph->port);
    periph->port.fd = -1;
  }
  periph->endpoint = DIGITAL_CAM_UART_ENDPOINT_NONE;
  digital_cam_uart_sim_clear_queues(periph);
}

/**
 * @brief Open an endpoint as a nonblocking raw serial candidate.
 * @param[out] candidate Receives the opened descriptor on success.
 * @param endpoint Endpoint whose configured path is opened.
 * @return `true` when the candidate is ready for use, otherwise `false`.
 */
static bool digital_cam_uart_sim_open_candidate(struct SerialPort *candidate,
                                                 enum digital_cam_uart_endpoint endpoint)
{
  candidate->fd = -1;
  if (serial_port_open_raw(candidate, digital_cam_uart_sim_endpoint_path(endpoint), B115200) == 0) {
    return true;
  }
  candidate->fd = -1;
  return false;
}

/**
 * @brief Atomically replace the active endpoint with an open candidate.
 * @param periph Simulation transport instance.
 * @param endpoint Desired endpoint.
 * @return `true` if the new endpoint was opened and selected.
 *
 * @details The candidate is opened before the old endpoint is closed. A failed
 * preference probe must not interrupt a working fallback connection. Once the
 * candidate is known to work, closing the old endpoint intentionally resets all
 * connection-bound queues and protocol state before the candidate is installed.
 */
static bool digital_cam_uart_sim_switch(struct digital_cam_uart_sim_periph *periph,
                                        enum digital_cam_uart_endpoint endpoint)
{
  struct SerialPort candidate;
  if (!digital_cam_uart_sim_open_candidate(&candidate, endpoint)) {
    return false;
  }

  digital_cam_uart_sim_close(periph);
  periph->port = candidate;
  periph->endpoint = endpoint;
  fprintf(stderr, "Digital camera UART: connected to %s\n", digital_cam_uart_sim_endpoint_path(endpoint));
  return true;
}

/**
 * @brief Record a link failure and request immediate endpoint reconciliation.
 * @param periph Simulation transport instance.
 */
static void digital_cam_uart_sim_disconnect(struct digital_cam_uart_sim_periph *periph)
{
  if (periph->endpoint != DIGITAL_CAM_UART_ENDPOINT_NONE) {
    fprintf(stderr, "Digital camera UART: disconnected from %s\n",
            digital_cam_uart_sim_endpoint_path(periph->endpoint));
  }
  digital_cam_uart_sim_close(periph);
  periph->last_reconnect_us = 0;
}

/**
 * @brief Count occupied bytes in a wrapping single-producer ring.
 * @param insert_idx Producer index.
 * @param extract_idx Consumer index.
 * @param buffer_size Ring capacity.
 * @return Number of occupied slots.
 */
static uint16_t digital_cam_uart_sim_ring_count(uint16_t insert_idx, uint16_t extract_idx,
                                                uint16_t buffer_size)
{
  return insert_idx >= extract_idx ? insert_idx - extract_idx : buffer_size - extract_idx + insert_idx;
}

/**
 * @brief Move a bounded batch of operating-system bytes into the RX ring.
 * @param periph Simulation transport instance with an open endpoint.
 *
 * @details Bounding each pass prevents a busy camera stream from monopolizing
 * the flight loop. Overflow drops only the newly received byte and increments
 * the standard ::link_device overrun counter.
 */
static void digital_cam_uart_sim_receive(struct digital_cam_uart_sim_periph *periph)
{
  for (uint16_t count = 0; count < DIGITAL_CAM_UART_SIM_RX_BATCH_SIZE; count++) {
    uint8_t byte;
    ssize_t received = read(periph->port.fd, &byte, 1);
    if (received == 1) {
      uint16_t next_idx = (periph->rx_insert_idx + 1) % DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE;
      if (next_idx != periph->rx_extract_idx) {
        periph->rx_buf[periph->rx_insert_idx] = byte;
        periph->rx_insert_idx = next_idx;
      } else {
        periph->device.nb_ovrn++;
      }
      continue;
    }
    if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      digital_cam_uart_sim_disconnect(periph);
    }
    return;
  }
}

/**
 * @brief Flush queued TX bytes without blocking the module scheduler.
 * @param periph Simulation transport instance with an open endpoint.
 *
 * @details Partial writes advance the consumer index by exactly the accepted
 * count. `EAGAIN`, `EWOULDBLOCK`, and `EINTR` retain the unsent suffix for the
 * next service pass; permanent errors disconnect the endpoint.
 */
static void digital_cam_uart_sim_transmit(struct digital_cam_uart_sim_periph *periph)
{
  while (periph->tx_extract_idx != periph->tx_insert_idx) {
    uint16_t available = periph->tx_insert_idx > periph->tx_extract_idx
                         ? periph->tx_insert_idx - periph->tx_extract_idx
                         : DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE - periph->tx_extract_idx;
    ssize_t sent = write(periph->port.fd, &periph->tx_buf[periph->tx_extract_idx], available);
    if (sent > 0) {
      periph->tx_extract_idx = (periph->tx_extract_idx + sent) % DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE;
      periph->device.nb_bytes += sent;
      continue;
    }
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      digital_cam_uart_sim_disconnect(periph);
    }
    return;
  }
}

/**
 * @brief Perform one zero-timeout RX/TX service pass.
 * @param periph Simulation transport instance.
 *
 * @details I/O happens here rather than in `char_available`, so querying the
 * ::link_device never changes connection state or performs hidden system calls.
 * The event and periodic callbacks invoke this function explicitly, avoiding a
 * transport thread and the locking that shared rings and parser state would need.
 */
static void digital_cam_uart_sim_service(struct digital_cam_uart_sim_periph *periph)
{
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_NONE) {
    return;
  }

  struct pollfd poll_fd = {
    .fd = periph->port.fd,
    .events = POLLIN | (periph->tx_extract_idx != periph->tx_insert_idx ? POLLOUT : 0),
    .revents = 0
  };
  int result = poll(&poll_fd, 1, 0);
  if (result < 0) {
    if (errno != EINTR) {
      digital_cam_uart_sim_disconnect(periph);
    }
    return;
  }
  if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    digital_cam_uart_sim_disconnect(periph);
    return;
  }
  if ((poll_fd.revents & POLLIN) != 0) {
    digital_cam_uart_sim_receive(periph);
  }
  if (periph->endpoint != DIGITAL_CAM_UART_ENDPOINT_NONE && (poll_fd.revents & POLLOUT) != 0) {
    digital_cam_uart_sim_transmit(periph);
  }
}

/**
 * @brief Enforce local-CATIA-first endpoint priority.
 * @param periph Simulation transport instance.
 *
 * @details A present and openable local pseudo-terminal always wins. If it is
 * absent or disappears, the physical fallback is attempted. Presence alone is
 * not enough to displace a working fallback: ::digital_cam_uart_sim_switch
 * first proves that the preferred endpoint can be opened.
 */
static void digital_cam_uart_sim_reconcile(struct digital_cam_uart_sim_periph *periph)
{
  bool local_available = access(STRINGIFY(DIGITAL_CAM_UART_LOCAL_DEVICE), F_OK) == 0;
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_LOCAL && !local_available) {
    digital_cam_uart_sim_disconnect(periph);
  }
  if (local_available && periph->endpoint != DIGITAL_CAM_UART_ENDPOINT_LOCAL
      && digital_cam_uart_sim_switch(periph, DIGITAL_CAM_UART_ENDPOINT_LOCAL)) {
    return;
  }
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_NONE) {
    digital_cam_uart_sim_switch(periph, DIGITAL_CAM_UART_ENDPOINT_FALLBACK);
  }
}

/**
 * @brief Check whether one complete caller-requested frame can be queued.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param fd Unused multi-link compatibility argument.
 * @param len Required frame size in bytes.
 * @return Available byte count when at least `len` bytes fit, otherwise zero.
 *
 * @details Reporting zero for insufficient space lets CATIA producers reject a
 * whole frame before writing its header. This is essential because a truncated
 * frame would desynchronize the receiver until its next valid start marker.
 */
static int digital_cam_uart_sim_check_free_space(void *parent, long *fd __attribute__((unused)), uint16_t len)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_NONE) {
    return 0;
  }
  uint16_t used = digital_cam_uart_sim_ring_count(periph->tx_insert_idx, periph->tx_extract_idx,
                                                  DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE);
  uint16_t space = DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE - used - 1;
  return space >= len ? space : 0;
}

/**
 * @brief Append one byte to the simulation TX ring.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param fd Unused multi-link compatibility argument.
 * @param data Byte to enqueue.
 *
 * @warning Frame producers must call `check_free_space` for the complete CATIA
 * frame first. The overrun guard protects memory but cannot make a partial frame
 * valid if a producer violates that contract.
 */
static void digital_cam_uart_sim_put_byte(void *parent, long fd __attribute__((unused)), uint8_t data)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_NONE) {
    return;
  }
  uint16_t next_idx = (periph->tx_insert_idx + 1) % DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE;
  if (next_idx == periph->tx_extract_idx) {
    periph->device.nb_ovrn++;
    return;
  }
  periph->tx_buf[periph->tx_insert_idx] = data;
  periph->tx_insert_idx = next_idx;
}

/**
 * @brief Append a byte array to the simulation TX ring.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param fd Unused multi-link compatibility argument.
 * @param data Bytes to enqueue.
 * @param len Number of bytes in `data`.
 * @note Callers must reserve space for the complete frame before insertion.
 */
static void digital_cam_uart_sim_put_buffer(void *parent, long fd, const uint8_t *data, uint16_t len)
{
  for (uint16_t index = 0; index < len; index++) {
    digital_cam_uart_sim_put_byte(parent, fd, data[index]);
  }
}

/**
 * @brief Give the nonblocking transport an immediate chance to flush a frame.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param fd Unused multi-link compatibility argument.
 */
static void digital_cam_uart_sim_send_message(void *parent, long fd __attribute__((unused)))
{
  digital_cam_uart_sim_service(parent);
}

/**
 * @brief Query bytes already buffered for CATIA parsing.
 * @param parent Simulation transport instance passed through ::link_device.
 * @return Number of bytes currently in the RX ring.
 * @note This callback is deliberately side-effect free; I/O and connection
 * management remain visible in explicit service calls.
 */
static int digital_cam_uart_sim_char_available(void *parent)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  return digital_cam_uart_sim_ring_count(periph->rx_insert_idx, periph->rx_extract_idx,
                                         DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE);
}

/**
 * @brief Consume one buffered RX byte.
 * @param parent Simulation transport instance passed through ::link_device.
 * @return Oldest buffered byte, or zero when called on an empty ring.
 */
static uint8_t digital_cam_uart_sim_get_byte(void *parent)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  if (periph->rx_extract_idx == periph->rx_insert_idx) {
    return 0;
  }
  uint8_t byte = periph->rx_buf[periph->rx_extract_idx];
  periph->rx_extract_idx = (periph->rx_extract_idx + 1) % DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE;
  return byte;
}

/**
 * @brief Apply a baud rate to the active simulation serial endpoint.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param baudrate Platform baud-rate constant.
 */
static void digital_cam_uart_sim_set_baudrate(void *parent, uint32_t baudrate)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  if (periph->endpoint != DIGITAL_CAM_UART_ENDPOINT_NONE
      && serial_port_set_baudrate(&periph->port, baudrate) != 0) {
    digital_cam_uart_sim_disconnect(periph);
  }
}

/**
 * @brief Initialize the private ::link_device and select the best endpoint.
 *
 * @details Hardware builds keep using `CAMERA_PORT`; this adapter gives only
 * simulation builds dynamic selection without changing generic UART ownership.
 */
static void digital_cam_uart_sim_init(void)
{
  struct digital_cam_uart_sim_periph *periph = &digital_cam_uart_sim_link;
  periph->port.fd = -1;
  periph->endpoint = DIGITAL_CAM_UART_ENDPOINT_NONE;
  periph->last_reconnect_us = 0;
  digital_cam_uart_sim_clear_queues(periph);
  periph->device = (struct link_device) {
    .check_free_space = digital_cam_uart_sim_check_free_space,
    .put_byte = digital_cam_uart_sim_put_byte,
    .put_buffer = digital_cam_uart_sim_put_buffer,
    .send_message = digital_cam_uart_sim_send_message,
    .char_available = digital_cam_uart_sim_char_available,
    .get_byte = digital_cam_uart_sim_get_byte,
    .set_baudrate = digital_cam_uart_sim_set_baudrate,
    .periph = periph
  };
  digital_cam_uart_sim_reconcile(periph);
}

/**
 * @brief Service transport I/O and periodically reconsider endpoint priority.
 *
 * @details Service runs before and after reconciliation so a newly selected
 * endpoint participates immediately. Unsigned timestamp subtraction keeps the
 * interval test valid across the microsecond counter wrapping.
 */
static void digital_cam_uart_sim_periodic(void)
{
  struct digital_cam_uart_sim_periph *periph = &digital_cam_uart_sim_link;
  digital_cam_uart_sim_service(periph);
  uint32_t now = get_sys_time_usec();
  if (periph->last_reconnect_us == 0
      || (uint32_t)(now - periph->last_reconnect_us) >= DIGITAL_CAM_UART_RECONNECT_INTERVAL_USEC) {
    periph->last_reconnect_us = now;
    digital_cam_uart_sim_reconcile(periph);
  }
  digital_cam_uart_sim_service(periph);
}
#endif

union dc_shot_union dc_shot_msg;
union catia_status_union catia_status_msg;
int digital_cam_uart_status = 0;
#ifndef DIGITAL_CAM_UART_CAMERA_ID
#define DIGITAL_CAM_UART_CAMERA_ID CATIA_CAMERA_ALL
#endif
#ifndef DIGITAL_CAM_UART_CAMERA_MASK
#if DIGITAL_CAM_UART_CAMERA_ID < CATIA_CAMERA_ALL || DIGITAL_CAM_UART_CAMERA_ID > 8
#error "DIGITAL_CAM_UART_CAMERA_ID must be between 0 and 8"
#endif
#if DIGITAL_CAM_UART_CAMERA_ID == CATIA_CAMERA_ALL
#define DIGITAL_CAM_UART_CAMERA_MASK CATIA_CAMERA_MASK_ALL
#else
#define DIGITAL_CAM_UART_CAMERA_MASK (1U << (DIGITAL_CAM_UART_CAMERA_ID - 1))
#endif
#endif
#if DIGITAL_CAM_UART_CAMERA_MASK < 0 || DIGITAL_CAM_UART_CAMERA_MASK > CATIA_CAMERA_MASK_ALL
#error "DIGITAL_CAM_UART_CAMERA_MASK must fit in eight bits"
#endif
uint8_t digital_cam_uart_camera_mask = DIGITAL_CAM_UART_CAMERA_MASK;
static digital_cam_uart_rx_handler_t rx_handler = NULL;

int digital_cam_uart_thumbnails = 0;
#define THUMB_MSG_SIZE  CATIA_PAYLOAD_MSG_SIZE
#define THUMB_COUNT     10
static uint8_t thumbs[THUMB_COUNT][THUMB_MSG_SIZE];
static uint8_t thumb_pointer = 0;

/** Populate a CATIA shot payload from the aircraft state estimator. */
static void fill_shot_message(union dc_shot_union *msg);


bool uart_cam_ctrl_set_camera(float camera_id)
{
  if (!(camera_id >= CATIA_CAMERA_ALL && camera_id <= 8)) {
    return false;
  }
  uint8_t selection = (uint8_t)camera_id;
  if (camera_id != (float)selection) {
    return false;
  }
  return uart_cam_ctrl_set_camera_mask(selection == CATIA_CAMERA_ALL
                                      ? CATIA_CAMERA_MASK_ALL : 1U << (selection - 1));
}

bool uart_cam_ctrl_set_camera_mask(float camera_mask)
{
  if (!(camera_mask >= 0 && camera_mask <= CATIA_CAMERA_MASK_ALL)) {
    return false;
  }
  uint8_t selection = (uint8_t)camera_mask;
  if (camera_mask != (float)selection) {
    return false;
  }
  digital_cam_uart_camera_mask = selection;
  return true;
}

void digital_cam_uart_set_rx_handler(digital_cam_uart_rx_handler_t handler)
{
  rx_handler = handler;
}

void digital_cam_uart_event(void)
{
#ifdef DIGITAL_CAM_UART_SIM_BACKEND
  digital_cam_uart_sim_service(&digital_cam_uart_sim_link);
#endif
  while (CameraLinkChAvailable()) {
    parse_catia(&catia_protocol, CameraLinkGetch());
    if (catia_protocol.msg_received) {
      switch (catia_protocol.msg_id) {
#if DIGITAL_CAM_UART_POSE_STREAM
        case CATIA_CLOCK_REQUEST:
          reply_clock_request();
          break;
#endif
        case CATIA_STATUS:
          /* A valid checksum only proves transport integrity. Require the complete
           * fixed-layout status before copying, otherwise stale parser-buffer bytes
           * beyond a short frame could become flight-controller state. */
          if (catia_protocol.payload_len != CATIA_STATUS_MSG_SIZE) {
            break;
          }
          for (int i = 0; i < CATIA_STATUS_MSG_SIZE; i++) {
            catia_status_msg.bin[i] = catia_protocol.payload[i];
          }
          digital_cam_uart_status = catia_status_msg.data.shots;
          break;
        case CATIA_PAYLOAD:
          /* Thumbnail packets have one fixed wire size. Reject both short and long
           * variants so this fixed-size copy only consumes bytes from the current frame. */
          if (catia_protocol.payload_len != CATIA_PAYLOAD_MSG_SIZE) {
            break;
          }
          for (int i = 0; i < CATIA_PAYLOAD_MSG_SIZE; i++) {
            thumbs[thumb_pointer][i] = catia_protocol.payload[i];
          }
          break;
        default:
          if (rx_handler != NULL) {
            rx_handler(&catia_protocol);
          }
          break;
      }
      catia_protocol.msg_received = 0;
    }
  }
}

#if PERIODIC_TELEMETRY
static void send_thumbnails(struct transport_tx *trans, struct link_device *dev)
{
  static int cnt = 0;
  if (digital_cam_uart_thumbnails > 0) {
    if (digital_cam_uart_thumbnails == 1) {
      cnt++;
      if (cnt > 1) {
        cnt = 0;
        return;
      }
    }
    pprz_msg_send_PAYLOAD(trans, dev, AC_ID, THUMB_MSG_SIZE, thumbs[thumb_pointer]);

    // Update the write/read pointer: if we receive a new thumb part, that will be sent, otherwise the oldest infor is repeated
    thumb_pointer++;
    if (thumb_pointer >= THUMB_COUNT) {
      thumb_pointer = 0;
    }

    if (CameraLinkCheckFreeSpace(CatiaSizeOf(0))) {
      CatiaHeader(CATIA_BUFFER_EMPTY, 0);
      CatiaTrailer();
    }
  }
}
#endif

void digital_cam_uart_init(void)
{
  digital_cam_uart_reset_protocol_state();
#ifdef DIGITAL_CAM_UART_SIM_BACKEND
  digital_cam_uart_sim_init();
#endif
#if DIGITAL_CAM_UART_POSE_STREAM
  pose_sequence = 0;
#endif
  digital_cam_uart_thumbnails = 0;
  for (int t = 0; t < THUMB_COUNT; t++) {
    for (int i = 0; i < THUMB_MSG_SIZE; i++) {
      thumbs[t][i] = 0;
    }
  }
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD, send_thumbnails);
#endif
}

void digital_cam_uart_periodic(void)
{
#ifdef DIGITAL_CAM_UART_SIM_BACKEND
  digital_cam_uart_sim_periodic();
#endif
  // Common DC Periodic task
  dc_periodic();
#if DIGITAL_CAM_UART_POSE_STREAM
  send_pose_sample();
#endif
}

/**
 * @brief Snapshot navigation and attitude state into a shot record.
 * @param[out] msg Shot record populated with the next photo number and pose.
 */
static void fill_shot_message(union dc_shot_union *msg)
{
  msg->data.nr = dc_photo_nr + 1;
  msg->data.lat = stateGetPositionLla_i()->lat;
  msg->data.lon = stateGetPositionLla_i()->lon;
  msg->data.alt = stateGetPositionLla_i()->alt;
  msg->data.phi = stateGetNedToBodyEulers_i()->phi;
  msg->data.theta = stateGetNedToBodyEulers_i()->theta;
  msg->data.psi = stateGetNedToBodyEulers_i()->psi;
  msg->data.vground = stateGetHorizontalSpeedNorm_i();
  msg->data.course = stateGetHorizontalSpeedDir_i();
#if FIXEDWING_FIRMWARE
  msg->data.groundalt = POS_BFP_OF_REAL(stateGetPositionUtm_f()->alt - ground_alt);
#else
  msg->data.groundalt = POS_BFP_OF_REAL(state.alt_agl_f);
#endif
}

#if DIGITAL_CAM_UART_POSE_STREAM
/**
 * @brief Reply to a valid CATIA clock-alignment request.
 * @details Reply and shot-frame capacity is reserved together so diagnostic
 * clock traffic cannot consume the space needed for a capture command.
 */
static void reply_clock_request(void)
{
  if (catia_protocol.payload_len != CATIA_CLOCK_REQUEST_MSG_SIZE) return;
  union catia_clock_reply_union reply = {0};
  reply.data.receive_us = get_sys_time_usec();
  for (size_t index = 0; index < sizeof(reply.data.request.bin); ++index) {
    reply.data.request.bin[index] = catia_protocol.payload[index];
  }
  if ((reply.data.request.data.token_low == 0 && reply.data.request.data.token_high == 0)
      || (clock_reply_sent && (uint32_t)(reply.data.receive_us - last_clock_reply_us) < 500000)
      || !CameraLinkCheckFreeSpace(CatiaSizeOf(CATIA_CLOCK_REPLY_MSG_SIZE)
                     + CatiaSizeOf(CATIA_SHOOT_TARGETED_MSG_SIZE))) return;
  reply.data.transmit_us = get_sys_time_usec();
  CatiaHeader(CATIA_CLOCK_REPLY, CATIA_CLOCK_REPLY_MSG_SIZE);
  for (size_t index = 0; index < sizeof(reply.bin); ++index) CatiaPutUint8(reply.bin[index]);
  CatiaTrailer();
  pose_clock_token = reply.data.request;
  last_clock_reply_us = reply.data.transmit_us;
  clock_reply_sent = true;
}

/**
 * @brief Send one optional diagnostic pose sample when a complete frame fits.
 * @details Pose samples are best-effort telemetry. Dropping one under TX
 * pressure is preferable to emitting a partial frame or delaying camera
 * commands. Endpoint changes clear the peer-specific clock token.
 */
static void send_pose_sample(void)
{
  const uint32_t sequence = pose_sequence++;
  const bool clocked = pose_clock_token.data.token_low != 0 || pose_clock_token.data.token_high != 0;
  const uint8_t payload_size = clocked ? CATIA_POSE_CLOCKED_MSG_SIZE : CATIA_POSE_SAMPLE_MSG_SIZE;
  const uint16_t required_space = CatiaSizeOf(payload_size)
                                  + CatiaSizeOf(CATIA_SHOOT_TARGETED_MSG_SIZE);
  if (!CameraLinkCheckFreeSpace(required_space)) {
    return;
  }
  union catia_pose_sample_union sample = {0};
  sample.data.sequence = sequence;
  sample.data.sample_begin_us = get_sys_time_usec();
  fill_shot_message(&sample.data.shot);
  const struct NedCoor_i *velocity = stateGetSpeedNed_i();
  sample.data.velocity_north_bfp = velocity->x;
  sample.data.velocity_east_bfp = velocity->y;
  sample.data.velocity_down_bfp = velocity->z;
#if USE_GPS
  sample.data.gps_tow_ms = gps.tow;
  sample.data.gps_week = gps.week;
  sample.data.gps_hacc_cm = gps.hacc;
  sample.data.gps_vacc_cm = gps.vacc;
  sample.data.gps_sacc_cm_s = gps.sacc;
  sample.data.gps_fix = gps.fix;
  sample.data.gps_num_sv = gps.num_sv;
  sample.data.gps_valid_fields = gps.valid_fields;
  sample.data.flags = CATIA_POSE_SAMPLE_GPS_PRESENT;
#endif
  sample.data.sample_end_us = get_sys_time_usec();
  CatiaHeader(clocked ? CATIA_POSE_CLOCKED : CATIA_POSE_SAMPLE, payload_size);
  for (size_t index = 0; index < CATIA_POSE_SAMPLE_MSG_SIZE; ++index) {
    CatiaPutUint8(sample.bin[index]);
  }
  if (clocked) {
    for (size_t index = 0; index < sizeof(pose_clock_token.bin); ++index) CatiaPutUint8(pose_clock_token.bin[index]);
  }
  CatiaTrailer();
}
#endif

/**
 * @brief Queue a pose-tagged shoot frame for one camera or the legacy all-camera target.
 * @param camera_id CATIA camera ID, or ::CATIA_CAMERA_ALL.
 * @return `true` when the complete frame was queued, otherwise `false`.
 * @details Capacity is reserved before writing any byte. This preserves CATIA
 * framing when the link is disconnected or its TX ring is under pressure.
 */
static bool send_shot_frame(uint8_t camera_id)
{
  if (camera_id == CATIA_CAMERA_ALL) {
    if (!CameraLinkCheckFreeSpace(CatiaSizeOf(CATIA_SHOOT_MSG_SIZE))) {
      return false;
    }
    fill_shot_message(&dc_shot_msg);
    CatiaHeader(CATIA_SHOOT, CATIA_SHOOT_MSG_SIZE);
    for (int i = 0; i < CATIA_SHOOT_MSG_SIZE; i++) {
      CatiaPutUint8(dc_shot_msg.bin[i]);
    }
    CatiaTrailer();
    return true;
  }
  if (!CameraLinkCheckFreeSpace(CatiaSizeOf(CATIA_SHOOT_TARGETED_MSG_SIZE))) {
    return false;
  }
  union dc_shot_targeted_union msg;
  fill_shot_message(&msg.data.shot);
  msg.data.camera_id = camera_id;
  CatiaHeader(CATIA_SHOOT_TARGETED, CATIA_SHOOT_TARGETED_MSG_SIZE);
  for (int i = 0; i < CATIA_SHOOT_TARGETED_MSG_SIZE; i++) {
    CatiaPutUint8(msg.bin[i]);
  }
  CatiaTrailer();
  return true;
}

uint8_t digital_cam_uart_shoot(uint8_t camera_id, bool report)
{
  bool sent = send_shot_frame(camera_id);
  if (report) {
    dc_send_shot_position();
  } else if (dc_photo_nr < DC_IMAGE_BUFFER) {
    dc_photo_nr++;   // same numbering as dc_send_shot_position(), without telemetry
  }
  return sent ? 0 : 1;
}

uint8_t digital_cam_uart_stop(uint8_t camera_id, bool keep_session)
{
  if (!CameraLinkCheckFreeSpace(CatiaSizeOf(CATIA_STOP_TARGETED_MSG_SIZE))) {
    return 1;
  }
  int32_t id = camera_id | (keep_session ? CATIA_STOP_FLAG_KEEP : 0);
  uint8_t bin[CATIA_STOP_TARGETED_MSG_SIZE];
  for (int i = 0; i < CATIA_STOP_TARGETED_MSG_SIZE; i++) {
    bin[i] = (uint8_t)((id >> (8 * i)) & 0xFF);
  }
  CatiaHeader(CATIA_STOP_TARGETED, CATIA_STOP_TARGETED_MSG_SIZE);
  for (int i = 0; i < CATIA_STOP_TARGETED_MSG_SIZE; i++) {
    CatiaPutUint8(bin[i]);
  }
  CatiaTrailer();
  return 0;
}


/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  switch (cmd) {
    case DC_SHOOT:
      if (digital_cam_uart_camera_mask != CATIA_CAMERA_MASK_NONE
          && CameraLinkCheckFreeSpace(CatiaSizeOf(CATIA_SHOOT_MASK_MSG_SIZE))) {
        union dc_shot_mask_union msg;
        fill_shot_message(&msg.data.shot);
        msg.data.camera_mask = digital_cam_uart_camera_mask;
        CatiaHeader(CATIA_SHOOT_MASK, CATIA_SHOOT_MASK_MSG_SIZE);
        for (size_t index = 0; index < sizeof(msg.bin); ++index) {
          CatiaPutUint8(msg.bin[index]);
        }
        CatiaTrailer();
        dc_send_shot_position();
      }
      break;
    case DC_TALLER:
      break;
    case DC_WIDER:
      break;
    case DC_ON:
      break;
    case DC_OFF:
      break;
    default:
      break;
  }

  // call command send_command function
  dc_send_command_common(cmd);
}
