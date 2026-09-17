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
 * @file modules/digital_cam/uart_cam_ctrl.h
 * @brief Public API for CATIA digital-camera control over UART.
 *
 * @details The flight controller sends pose-tagged capture commands to an
 * external camera computer and receives status, thumbnails, or application
 * replies. The API is transport-independent: hardware uses `CAMERA_PORT`, while
 * simulation targets may use the module-owned local/fallback serial adapter.
 *
 * @see modules/digital_cam/uart_cam_ctrl.c for transport ownership and endpoint
 * switching rationale.
 */

#ifndef DIGITAL_CAM_UART_H
#define DIGITAL_CAM_UART_H

#include "modules/digital_cam/dc.h"
#include "modules/digital_cam/catia/protocol.h"

/**
 * @defgroup digital_cam_uart Digital camera UART control
 * @brief Pose-tagged CATIA camera commands, replies, and thumbnail relay.
 * @{
 */

/** Initialize CATIA protocol state, camera buffers, and the selected transport. */
extern void digital_cam_uart_init(void);

/** Run camera housekeeping, optional pose streaming, and simulation link failover. */
extern void digital_cam_uart_periodic(void);
/** Service transport input and dispatch complete CATIA frames. */
extern void digital_cam_uart_event(void);

/** Thumbnail downlink mode: zero disables, one throttles, and two sends every opportunity. */
extern int digital_cam_uart_thumbnails;
/** Shot count reported by the most recent CATIA status frame. */
extern int digital_cam_uart_status;

/** Bit mask of cameras triggered by the standard ::DC_SHOOT command. */
extern uint8_t digital_cam_uart_camera_mask;

/**
 * @brief Select one camera for standard capture commands.
 * @param camera_id Integer-valued CATIA camera ID in [1, 8], or zero for all.
 * @return `true` when valid and applied; `false` leaves the selection unchanged.
 * @note This compatibility helper converts the ID to a one-bit camera mask.
 */
extern bool uart_cam_ctrl_set_camera(float camera_id);
/**
 * @brief Select any combination of cameras for standard capture commands.
 * @param camera_mask Integer-valued mask in [0, 255]; zero suppresses captures.
 * @return `true` when valid and applied; `false` leaves the selection unchanged.
 */
extern bool uart_cam_ctrl_set_camera_mask(float camera_mask);

/**
 * @brief Request a pose-tagged capture from one camera or all cameras.
 * @param camera_id CATIA camera ID, or ::CATIA_CAMERA_ALL.
 * @param report When `true`, emit `DC_SHOT` telemetry; when `false`, only advance numbering.
 * @return Zero when the complete CATIA frame was queued, nonzero otherwise.
 * @note The photo number advances even when the serial frame cannot be queued,
 * preserving monotonic numbering across best-effort high-rate capture requests.
 */
extern uint8_t digital_cam_uart_shoot(uint8_t camera_id, bool report);
/**
 * @brief Ask one camera or all cameras to stop or report an interim result.
 * @param camera_id CATIA camera ID, or ::CATIA_CAMERA_ALL.
 * @param keep_session Keep acquisition active after reporting, as used by EARcam refinement.
 * @return Zero when the complete CATIA frame was queued, nonzero otherwise.
 */
extern uint8_t digital_cam_uart_stop(uint8_t camera_id, bool keep_session);

/**
 * @brief Callback type for complete CATIA frames not consumed by this module.
 * @param frame Parsed frame; valid only for the duration of the callback.
 * @return `true` when the application recognized and handled the frame.
 */
typedef bool (*digital_cam_uart_rx_handler_t)(const struct catia_transport *frame);
/**
 * @brief Register the handler for application-specific CATIA replies.
 * @param handler Callback to install, or `NULL` to disable forwarding.
 * @note The callback runs synchronously from ::digital_cam_uart_event.
 */
extern void digital_cam_uart_set_rx_handler(digital_cam_uart_rx_handler_t handler);

/** @} */

#endif // DIGITAL_CAM_UART_H
