/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
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
 * @file modules/datalink/datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#define MODULES_DATALINK_C

#include "datalink.h"
#include "modules/datalink/downlink.h"

#include "generated/modules.h"
#include "generated/settings.h"

#include "pprzlink/messages.h"
#include "mcu_periph/sys_time.h"

bool dl_msg_available;
uint16_t datalink_time;
uint16_t datalink_nb_msgs;
static uint8_t dl_buffer[DATALINK_MSG_SIZE]  __attribute__((aligned));
static uint32_t datalink_last_gcs_self_ping_ms;
static uint32_t datalink_last_gcs_other_ping_ms;
static bool datalink_has_gcs_self_ping;
static bool datalink_has_gcs_other_ping;

#if USE_NPS
bool datalink_enabled = true;
#endif

void datalink_init(void)
{
  dl_msg_available = false;
  datalink_time = 0;
  datalink_nb_msgs = 0;
  datalink_last_gcs_self_ping_ms = 0;
  datalink_last_gcs_other_ping_ms = 0;
  datalink_has_gcs_self_ping = false;
  datalink_has_gcs_other_ping = false;
}

void datalink_periodic(void)
{
  datalink_time++; // called at 1Hz
}

void datalink_parse_PING(struct link_device *dev, struct transport_tx *trans, uint8_t *buf)
{
  const uint8_t receiver = pprzlink_get_msg_receiver_id(buf);
  if (pprzlink_get_msg_sender_id(buf) == 0) {
    if (receiver == AC_ID) {
      datalink_last_gcs_self_ping_ms = get_sys_time_msec();
      datalink_has_gcs_self_ping = true;
    } else if (receiver != PPRZLINK_MSG_BROADCAST) {
      datalink_last_gcs_other_ping_ms = get_sys_time_msec();
      datalink_has_gcs_other_ping = true;
    }
  }
  if (receiver != AC_ID && receiver != PPRZLINK_MSG_BROADCAST) {
    return;
  }
  // Reply to the sender of the message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = pprzlink_get_msg_sender_id(buf);
  msg.component_id = 0;
  pprzlink_msg_send_PONG(&msg);
}

bool datalink_gcs_self_ping_is_fresh(uint32_t timeout_ms)
{
  return datalink_has_gcs_self_ping
         && get_sys_time_msec() - datalink_last_gcs_self_ping_ms <= timeout_ms;
}

bool datalink_gcs_other_ping_is_fresh(uint32_t timeout_ms)
{
  return datalink_has_gcs_other_ping
         && get_sys_time_msec() - datalink_last_gcs_other_ping_ms <= timeout_ms;
}

uint8_t* datalink_get_buffer(void)
{
  return dl_buffer;
}

void datalink_fill_buffer(uint8_t *buf, uint16_t len)
{
  // TODO: replace with a memcpy for efficiency
  uint16_t i = 0;
  for (i = 0; i < len; i++) {
    dl_buffer[i] = buf[i];
  }
  dl_msg_available = true;
}

void WEAK dl_parse_msg(struct link_device *dev, struct transport_tx *trans, uint8_t *buf)
{
  uint8_t msg_id = pprzlink_get_msg_id(buf);
  uint8_t class_id = pprzlink_get_msg_class_id(buf);
  /* Parse modules datalink */
  modules_parse_datalink(msg_id, class_id, dev, trans, buf);
}

