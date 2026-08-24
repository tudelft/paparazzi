/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file datalink.h
 *  \brief Handling of messages coming from ground and other A/Cs
 *
 */

#ifndef DATALINK_H
#define DATALINK_H

#include "std.h"
#include "pprzlink/dl_protocol.h"

/** Datalink kinds */
#define PPRZ 1
#define XBEE 2
#define SUPERBITRF 3
#define W5100 4

#define DATALINK_MSG_SIZE 256

/** Flag provided to control calls to ::dl_parse_msg. NOT used in this module*/
extern bool dl_msg_available;

/** Seconds since the last accepted primary uplink frame; an age, not a loss flag. */
extern uint16_t datalink_time;

/** number of datalink/uplink messages received */
extern uint16_t datalink_nb_msgs;

/** get datalink buffer address */
extern uint8_t* datalink_get_buffer(void);

/** Should be called when chars are available in datalink uffer */
extern void dl_parse_msg(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);

/** Parse a message while retaining its received payload length. */
extern void dl_parse_msg_with_length(struct link_device *dev, struct transport_tx *trans,
                                     uint8_t *buf, uint8_t payload_len);

#if USE_NPS
extern bool datalink_enabled;
#endif

/** fill datalink buffer */
extern void datalink_fill_buffer(uint8_t *buf, uint16_t len);

/** init function */
extern void datalink_init(void);

/** periodic function, should be called at 1Hz */
extern void datalink_periodic(void);

extern void datalink_parse_ALIVE_REQ(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);

extern void datalink_parse_PING(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);

/** Return whether GCS recently sent a PING addressed to this aircraft. */
extern bool datalink_gcs_self_ping_is_fresh(uint32_t timeout_ms);

/** Return whether GCS recently sent a PING addressed to another aircraft. */
extern bool datalink_gcs_other_ping_is_fresh(uint32_t timeout_ms);

/** Check for new message and parse */
static inline void DlCheckAndParse(struct link_device *dev, struct transport_tx *trans, uint8_t *buf,
                                   uint8_t payload_len, bool *msg_available, bool update_dl)
{
  // make it possible to disable datalink in NPS sim
#if USE_NPS
  if (!datalink_enabled) {
    return;
  }
#endif

  if (*msg_available) {
    if (update_dl) {
      /* Reset before command-specific parsing: receiving a complete accepted
       * frame proves the uplink path works even if that command is ignored. */
      datalink_time = 0;
      datalink_nb_msgs++;
    }
    dl_parse_msg_with_length(dev, trans, buf, payload_len);
    *msg_available = false;
  }
}

#endif /* DATALINK_H */
