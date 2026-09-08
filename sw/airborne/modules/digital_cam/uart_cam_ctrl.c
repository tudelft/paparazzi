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
 * Control the camera via uart to chdk-ptp.
 * Retrieve thumbnails
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

#if FIXEDWING_FIRMWARE
#include "modules/nav/common_nav.h"
#endif


#define CameraLinkDev (&((CAMERA_LINK).device))
#define CameraLinkTransmit(c) CameraLinkDev->put_byte(CameraLinkDev->periph, 0, c)
#define CameraLinkChAvailable() CameraLinkDev->char_available(CameraLinkDev->periph)
#define CameraLinkGetch() CameraLinkDev->get_byte(CameraLinkDev->periph)

union dc_shot_union dc_shot_msg;
union mora_status_union mora_status_msg;
int digital_cam_uart_status = 0;
uint8_t digital_cam_uart_camera_id = MORA_CAMERA_ALL;
static digital_cam_uart_rx_handler_t rx_handler = NULL;

int digital_cam_uart_thumbnails = 0;
#define THUMB_MSG_SIZE  MORA_PAYLOAD_MSG_SIZE
#define THUMB_COUNT     10
static uint8_t thumbs[THUMB_COUNT][THUMB_MSG_SIZE];
static uint8_t thumb_pointer = 0;

static void fill_shot_message(union dc_shot_union *msg);


void digital_cam_uart_set_rx_handler(digital_cam_uart_rx_handler_t handler)
{
  rx_handler = handler;
}

void digital_cam_uart_event(void)
{
  while (CameraLinkChAvailable()) {
    parse_mora(&mora_protocol, CameraLinkGetch());
    if (mora_protocol.msg_received) {
      switch (mora_protocol.msg_id) {
        case MORA_STATUS:
          for (int i = 0; i < MORA_STATUS_MSG_SIZE; i++) {
            mora_status_msg.bin[i] = mora_protocol.payload[i];
          }
          digital_cam_uart_status = mora_status_msg.data.shots;
          break;
        case MORA_PAYLOAD:
          for (int i = 0; i < MORA_PAYLOAD_MSG_SIZE; i++) {
            thumbs[thumb_pointer][i] = mora_protocol.payload[i];
          }
          break;
        default:
          if (rx_handler != NULL) {
            rx_handler(&mora_protocol);
          }
          break;
      }
      mora_protocol.msg_received = 0;
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

    MoraHeader(MORA_BUFFER_EMPTY, 0);
    MoraTrailer();
  }
}
#endif

void digital_cam_uart_init(void)
{
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
  // Common DC Periodic task
  dc_periodic();
}

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

/** Send the pose-tagged shoot frame: legacy MORA_SHOOT for MORA_CAMERA_ALL, else targeted. */
static void send_shot_frame(uint8_t camera_id)
{
  if (camera_id == MORA_CAMERA_ALL) {
    fill_shot_message(&dc_shot_msg);
    MoraHeader(MORA_SHOOT, MORA_SHOOT_MSG_SIZE);
    for (int i = 0; i < MORA_SHOOT_MSG_SIZE; i++) {
      MoraPutUint8(dc_shot_msg.bin[i]);
    }
    MoraTrailer();
    return;
  }
  union dc_shot_targeted_union msg;
  fill_shot_message(&msg.data.shot);
  msg.data.camera_id = camera_id;
  MoraHeader(MORA_SHOOT_TARGETED, MORA_SHOOT_TARGETED_MSG_SIZE);
  for (int i = 0; i < MORA_SHOOT_TARGETED_MSG_SIZE; i++) {
    MoraPutUint8(msg.bin[i]);
  }
  MoraTrailer();
}

uint8_t digital_cam_uart_shoot(uint8_t camera_id, bool report)
{
  send_shot_frame(camera_id);
  if (report) {
    dc_send_shot_position();
  } else if (dc_photo_nr < DC_IMAGE_BUFFER) {
    dc_photo_nr++;   // same numbering as dc_send_shot_position(), without telemetry
  }
  return 0;
}

uint8_t digital_cam_uart_stop(uint8_t camera_id, bool keep_session)
{
  int32_t id = camera_id | (keep_session ? MORA_STOP_FLAG_KEEP : 0);
  uint8_t bin[MORA_STOP_TARGETED_MSG_SIZE];
  for (int i = 0; i < MORA_STOP_TARGETED_MSG_SIZE; i++) {
    bin[i] = (uint8_t)((id >> (8 * i)) & 0xFF);
  }
  MoraHeader(MORA_STOP_TARGETED, MORA_STOP_TARGETED_MSG_SIZE);
  for (int i = 0; i < MORA_STOP_TARGETED_MSG_SIZE; i++) {
    MoraPutUint8(bin[i]);
  }
  MoraTrailer();
  return 0;
}


/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  switch (cmd) {
    case DC_SHOOT:
      // Send Photo Position To Camera selected by the digital_cam_uart_camera_id setting
      send_shot_frame(digital_cam_uart_camera_id);
      dc_send_shot_position();
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
