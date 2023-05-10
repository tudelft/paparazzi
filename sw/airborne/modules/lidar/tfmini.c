/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/lidar/tfmini.c
 *  @brief driver for the TFMini lidar
 *
 */
#include "tfmini.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"

// State interface for rotation compensation
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

//include am7 module
#include "modules/sensors/ca_am7.h"

struct TFMini tfmini = {
  .parse_status = TFMINI_INITIALIZE
};

static void tfmini_parse(uint8_t byte);

static abi_event AM7_in;
// struct am7_data_in myam7_data_in_local;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

#if USE_TFMINI_SERIAL
/**
 * Downlink message lidar
 */
static void tfmini_send_lidar(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = (uint8_t) tfmini.parse_status;
  pprz_msg_send_LIDAR(trans, dev, AC_ID,
                      &tfmini.distance,
                      &tfmini.mode,
                      &status);
}

#else
/**
 * Downlink message lidar with am7
 */
static void tfmini_send_lidar_with_am7(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = 254;
  tfmini.mode = 254;
  pprz_msg_send_LIDAR(trans, dev, AC_ID,
                      &tfmini.distance,
                      &tfmini.mode,
                      &status);
}

#endif

#endif

#if USE_TFMINI_SERIAL
/**
 * Initialization function
 */
void tfmini_init(void)
{
  tfmini.device = &((TFMINI_PORT).device);

  tfmini.update_agl = USE_TFMINI_AGL;
  tfmini.compensate_rotation = TFMINI_COMPENSATE_ROTATION;

  tfmini.strength = 0;
  tfmini.distance = 0;
  tfmini.parse_status = TFMINI_PARSE_HEAD;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, tfmini_send_lidar);
#endif
}

#endif

/**
 * Lidar event function
 * Receive bytes from the UART port and parse them
 */
void tfmini_event(void)
{
  while (tfmini.parse_status != TFMINI_INITIALIZE && tfmini.device->char_available(tfmini.device->periph)) {
    tfmini_parse(tfmini.device->get_byte(tfmini.device->periph));
  }
}

/**
 * Parse the lidar bytes 1 by 1
 */
static void tfmini_parse(uint8_t byte)
{
  switch (tfmini.parse_status) {
    case TFMINI_INITIALIZE:
      break;
    case TFMINI_PARSE_HEAD:
      if (byte == 0x59) {
        tfmini.parse_crc = byte;
        tfmini.parse_status++;
      }
      break;
    case TFMINI_PARSE_HEAD2:
      if (byte == 0x59) {
        tfmini.parse_crc += byte;
        tfmini.parse_status++;
      } else {
        tfmini.parse_status = TFMINI_PARSE_HEAD;
      }
      break;

    case TFMINI_PARSE_DIST_L:
      tfmini.raw_dist = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_DIST_H:
      tfmini.raw_dist |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_STRENGTH_L:
      tfmini.raw_strength = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_STRENGTH_H:
      tfmini.raw_strength |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_MODE:
      tfmini.raw_mode = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_BYTE7:
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_CHECKSUM:
      // When the CRC matches
      if (tfmini.parse_crc == byte) {
        uint32_t now_ts = get_sys_time_usec();
        tfmini.distance = tfmini.raw_dist / 100.f;
        tfmini.strength = tfmini.raw_strength;
        tfmini.mode = tfmini.raw_mode;

        // When the distance is valid
        if (tfmini.distance != 0xFFFF) {
          // compensate AGL measurement for body rotation
          if (tfmini.compensate_rotation) {
            float phi = stateGetNedToBodyEulers_f()->phi;
            float theta = stateGetNedToBodyEulers_f()->theta;
            float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
            tfmini.distance = tfmini.distance * gain;
          }

          // send message (if requested)
          if (tfmini.update_agl) {
            AbiSendMsgAGL(AGL_LIDAR_TFMINI_ID, now_ts, tfmini.distance);
          }
        }
      }

      // Start reading again
      tfmini.parse_status = TFMINI_PARSE_HEAD;
      break;

    default:
      // Error, return to start
      tfmini.parse_status = TFMINI_PARSE_HEAD;
      break;
  }
}

/**
 * ABI routine called by the serial_act_t4 ABI event
 */
static void data_AM7_abi_in_lidar(uint8_t sender_id __attribute__((unused)), struct am7_data_in * myam7_data_in_ptr, float * extra_data_in_ptr){

  int16_t raw_value_lidar_cm = myam7_data_in_ptr->lidar_value_cm;
  int16_t raw_value_lidar_strength = myam7_data_in_ptr->lidar_strength;
  extra_data_in_ptr = extra_data_in_ptr;

  if(raw_value_lidar_cm >= 0){
    tfmini.raw_dist = (uint16_t) raw_value_lidar_cm;
    tfmini.raw_strength = (uint16_t) raw_value_lidar_strength;
    uint32_t now_ts = get_sys_time_usec();
    tfmini.distance = tfmini.raw_dist / 100.f;
    tfmini.strength = tfmini.raw_strength;

    // compensate AGL measurement for body rotation
    if (tfmini.compensate_rotation) {
      float phi = stateGetNedToBodyEulers_f()->phi;
      float theta = stateGetNedToBodyEulers_f()->theta;
      float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
      tfmini.distance = tfmini.distance * gain;
    }
    // send message (if requested)
    if (tfmini.update_agl) {
      AbiSendMsgAGL(AGL_LIDAR_TFMINI_ID, now_ts, tfmini.distance);
    }
  }
}

/**
 * Initialization function to call the AM7 module ABI message to copy the raw value and stength from there. 
 */
void tfmini_init_with_am7(void)
{
  tfmini.update_agl = USE_TFMINI_AGL;
  tfmini.compensate_rotation = TFMINI_COMPENSATE_ROTATION;

  tfmini.strength = 0;
  tfmini.distance = 0;

  //Init abi bind msg to Raspberry Pi:
  AbiBindMsgAM7_DATA_IN(ABI_BROADCAST, &AM7_in, data_AM7_abi_in_lidar);

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, tfmini_send_lidar_with_am7);
  #endif
}

