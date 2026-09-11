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

#if FIXEDWING_FIRMWARE
#include "modules/nav/common_nav.h"
#endif


#define CameraLinkDev (&((CAMERA_LINK).device))
#define CameraLinkTransmit(c) CameraLinkDev->put_byte(CameraLinkDev->periph, 0, c)
#define CameraLinkChAvailable() CameraLinkDev->char_available(CameraLinkDev->periph)
#define CameraLinkGetch() CameraLinkDev->get_byte(CameraLinkDev->periph)

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
          for (int i = 0; i < CATIA_STATUS_MSG_SIZE; i++) {
            catia_status_msg.bin[i] = catia_protocol.payload[i];
          }
          digital_cam_uart_status = catia_status_msg.data.shots;
          break;
        case CATIA_PAYLOAD:
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

    CatiaHeader(CATIA_BUFFER_EMPTY, 0);
    CatiaTrailer();
  }
}
#endif

void digital_cam_uart_init(void)
{
#if DIGITAL_CAM_UART_POSE_STREAM
  pose_sequence = 0;
  pose_clock_token = (union catia_clock_request_union){0};
  last_clock_reply_us = 0;
  clock_reply_sent = false;
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
  // Common DC Periodic task
  dc_periodic();
#if DIGITAL_CAM_UART_POSE_STREAM
  send_pose_sample();
#endif
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

#if DIGITAL_CAM_UART_POSE_STREAM
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
      || !uart_check_free_space(&(CAMERA_LINK), NULL,
                                CatiaSizeOf(CATIA_CLOCK_REPLY_MSG_SIZE) + CatiaSizeOf(CATIA_SHOOT_TARGETED_MSG_SIZE))) return;
  reply.data.transmit_us = get_sys_time_usec();
  CatiaHeader(CATIA_CLOCK_REPLY, CATIA_CLOCK_REPLY_MSG_SIZE);
  for (size_t index = 0; index < sizeof(reply.bin); ++index) CatiaPutUint8(reply.bin[index]);
  CatiaTrailer();
  pose_clock_token = reply.data.request;
  last_clock_reply_us = reply.data.transmit_us;
  clock_reply_sent = true;
}

static void send_pose_sample(void)
{
  const uint32_t sequence = pose_sequence++;
  const bool clocked = pose_clock_token.data.token_low != 0 || pose_clock_token.data.token_high != 0;
  const uint8_t payload_size = clocked ? CATIA_POSE_CLOCKED_MSG_SIZE : CATIA_POSE_SAMPLE_MSG_SIZE;
  const uint16_t required_space = CatiaSizeOf(payload_size)
                                  + CatiaSizeOf(CATIA_SHOOT_TARGETED_MSG_SIZE);
  if (!uart_check_free_space(&(CAMERA_LINK), NULL, required_space)) {
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

/** Send the pose-tagged shoot frame: legacy CATIA_SHOOT for CATIA_CAMERA_ALL, else targeted. */
static void send_shot_frame(uint8_t camera_id)
{
  if (camera_id == CATIA_CAMERA_ALL) {
    fill_shot_message(&dc_shot_msg);
    CatiaHeader(CATIA_SHOOT, CATIA_SHOOT_MSG_SIZE);
    for (int i = 0; i < CATIA_SHOOT_MSG_SIZE; i++) {
      CatiaPutUint8(dc_shot_msg.bin[i]);
    }
    CatiaTrailer();
    return;
  }
  union dc_shot_targeted_union msg;
  fill_shot_message(&msg.data.shot);
  msg.data.camera_id = camera_id;
  CatiaHeader(CATIA_SHOOT_TARGETED, CATIA_SHOOT_TARGETED_MSG_SIZE);
  for (int i = 0; i < CATIA_SHOOT_TARGETED_MSG_SIZE; i++) {
    CatiaPutUint8(msg.bin[i]);
  }
  CatiaTrailer();
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
      if (digital_cam_uart_camera_mask != CATIA_CAMERA_MASK_NONE) {
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
