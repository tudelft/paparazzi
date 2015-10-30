/*
 * Copyright (C) 2015 The Paparazzi Team
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

#include <inttypes.h>

#include "subsystems/intermcu.h"
#include "subsystems/electrical.h"

#include "mcu_periph/uart.h"
#include "led.h"


#if COMMANDS_NB > 8
#error "INTERMCU UART CAN ONLY SEND 8 COMMANDS OR THE UART WILL BE OVERFILLED"
#endif

#if RADIO_CONTROL_NB_CHANNEL > 8
#undef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 8
#warning "INTERMCU UART WILL ONLY SEND 8 RADIO CHANNELS"
#endif


//////////////////////////////////////////////////////////////////////////////////////////////
// LINK

// Use uart interface directly
#define InterMcuLinkDev (&((INTERMCU_LINK).device))
#define InterMcuLinkTransmit(c) InterMcuLinkDev->put_byte(InterMcuLinkDev->periph, c)
#define InterMcuLinkChAvailable() InterMcuLinkDev->char_available(InterMcuLinkDev->periph)
#define InterMcuLinkGetch() InterMcuLinkDev->get_byte(InterMcuLinkDev->periph)
#define InterMcuLinkSendMessage() {}

//#define InterMcuUartSetBaudrate(_a) uart_periph_set_baudrate(&(INTERMCU_LINK), _a)

//////////////////////////////////////////////////////////////////////////////////////////////
// PROTOCOL

#define INTERMCU_SYNC1 0xB5
#define INTERMCU_SYNC2 0x62

#define InterMcuInitCheksum() { intermcu_data.send_ck_a = intermcu_data.send_ck_b = 0; }
#define InterMcuUpdateChecksum(c) { intermcu_data.send_ck_a += c; intermcu_data.send_ck_b += intermcu_data.send_ck_a; }
#define InterMcuTrailer() { InterMcuLinkTransmit(intermcu_data.send_ck_a);  InterMcuLinkTransmit(intermcu_data.send_ck_b); InterMcuLinkSendMessage(); }

#define InterMcuSend1(c) { uint8_t i8=c; InterMcuLinkTransmit(i8); InterMcuUpdateChecksum(i8); }
#define InterMcuSend2(c) { uint16_t i16=c; InterMcuSend1(i16&0xff); InterMcuSend1(i16 >> 8); }
#define InterMcuSend1ByAddr(x) { InterMcuSend1(*x); }
#define InterMcuSend2ByAddr(x) { InterMcuSend1(*x); InterMcuSend1(*(x+1)); }
#define InterMcuSend4ByAddr(x) { InterMcuSend1(*x); InterMcuSend1(*(x+1)); InterMcuSend1(*(x+2)); InterMcuSend1(*(x+3)); }

#define InterMcuHeader(nav_id, msg_id, len) {        \
    InterMcuLinkTransmit(INTERMCU_SYNC1);                    \
    InterMcuLinkTransmit(INTERMCU_SYNC2);                    \
    InterMcuInitCheksum();                           \
    InterMcuSend1(nav_id);                           \
    InterMcuSend1(msg_id);                           \
    InterMcuSend2(len);                              \
  }

//////////////////////////////////////////////////////////////////////////////////////////////
// MESSAGES

// class
#define MSG_INTERMCU_ID 100


#define MSG_INTERMCU_COMMAND_ID 0x05
#define MSG_INTERMCU_COMMAND_LENGTH  (2*(COMMANDS_NB))
#define MSG_INTERMCU_COMMAND(_intermcu_payload, nr) (uint16_t)(*((uint8_t*)_intermcu_payload+0+(2*(nr)))|*((uint8_t*)_intermcu_payload+1+(2*(nr)))<<8)

#define InterMcuSend_INTERMCU_COMMAND(cmd) { \
    InterMcuHeader(MSG_INTERMCU_ID, MSG_INTERMCU_COMMAND_ID, MSG_INTERMCU_COMMAND_LENGTH);\
    for (int i=0;i<COMMANDS_NB;i++) { \
      uint16_t _cmd = cmd[i];  \
      InterMcuSend2ByAddr((uint8_t*)&_cmd);\
    } \
    InterMcuTrailer();\
  }

#define MSG_INTERMCU_RADIO_ID 0x08
#define MSG_INTERMCU_RADIO_LENGTH  (2*(RADIO_CONTROL_NB_CHANNEL))
#define MSG_INTERMCU_RADIO(_intermcu_payload, nr) (uint16_t)(*((uint8_t*)_intermcu_payload+0+(2*(nr)))|*((uint8_t*)_intermcu_payload+1+(2*(nr)))<<8)

#define InterMcuSend_INTERMCU_RADIO(cmd) { \
    InterMcuHeader(MSG_INTERMCU_ID, MSG_INTERMCU_RADIO_ID, MSG_INTERMCU_RADIO_LENGTH);\
    for (int i=0;i<RADIO_CONTROL_NB_CHANNEL;i++) { \
      uint16_t _cmd = cmd[i];  \
      InterMcuSend2ByAddr((uint8_t*)&_cmd);\
    } \
    InterMcuTrailer();\
  }

#define MSG_INTERMCU_FBW_ID 0x06
#define MSG_INTERMCU_FBW_MOD(_intermcu_payload) (uint8_t)(*((uint8_t*)_intermcu_payload+0))
#define MSG_INTERMCU_FBW_STAT(_intermcu_payload) (uint8_t)(*((uint8_t*)_intermcu_payload+1))
#define MSG_INTERMCU_FBW_ERR(_intermcu_payload) (uint8_t)(*((uint8_t*)_intermcu_payload+2))
#define MSG_INTERMCU_FBW_VOLT(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+3)|*((uint8_t*)_intermcu_payload+1+3)<<8)
//FIXME: Current is now 4BYTES
#define MSG_INTERMCU_FBW_CURRENT(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+5)|*((uint8_t*)_intermcu_payload+1+5)<<8)

#define InterMcuSend_INTERMCU_FBW(mod,stat,err,volt,current) { \
    InterMcuHeader(MSG_INTERMCU_ID, MSG_INTERMCU_FBW_ID, 7);\
    uint8_t _mod = mod; InterMcuSend1ByAddr((uint8_t*)&_mod);\
    uint8_t _stat = stat; InterMcuSend1ByAddr((uint8_t*)&_stat);\
    uint8_t _err = err; InterMcuSend1ByAddr((uint8_t*)&_err);\
    uint16_t _volt = volt; InterMcuSend2ByAddr((uint8_t*)&_volt);\
    uint16_t _current = current; InterMcuSend2ByAddr((uint8_t*)&_current);\
    InterMcuTrailer();\
  }

#define MSG_INTERMCU_TRIM_ID 0x07
#define MSG_INTERMCU_TRIM_ROLL(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+0)|*((uint8_t*)_intermcu_payload+1)<<8)
#define MSG_INTERMCU_TRIM_PITCH(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+2)|*((uint8_t*)_intermcu_payload+3)<<8)

#define InterMcuSend_INTERMCU_TRIM(roll,pitch) { \
    InterMcuHeader(MSG_INTERMCU_ID, MSG_INTERMCU_TRIM_ID, 4);\
    uint16_t _roll = roll; InterMcuSend2ByAddr((uint8_t*)&_roll);\
    uint16_t _pitch = pitch; InterMcuSend2ByAddr((uint8_t*)&_pitch);\
    InterMcuTrailer();\
  }

//////////////////////////////////////////////////////////////////////////////////////////////
// PARSER

/* parser status */
#define LINK_MCU_UNINIT        0
#define LINK_MCU_GOT_SYNC1     1
#define LINK_MCU_GOT_SYNC2     2
#define LINK_MCU_GOT_CLASS     3
#define LINK_MCU_GOT_ID        4
#define LINK_MCU_GOT_LEN1      5
#define LINK_MCU_GOT_LEN2      6
#define LINK_MCU_GOT_PAYLOAD   7
#define LINK_MCU_GOT_CHECKSUM1 8


#define INTERMCU_MAX_PAYLOAD 255
struct InterMcuData {
  bool_t msg_available;
  uint8_t msg_buf[INTERMCU_MAX_PAYLOAD] __attribute__((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t send_ck_a, send_ck_b;
  uint8_t error_cnt;
};

struct InterMcuData intermcu_data;

/* INTERMCU parsing */
void intermcu_parse(uint8_t c);
void intermcu_parse(uint8_t c)
{
  if (intermcu_data.status < LINK_MCU_GOT_PAYLOAD) {
    intermcu_data.ck_a += c;
    intermcu_data.ck_b += intermcu_data.ck_a;
  }
  switch (intermcu_data.status) {
    case LINK_MCU_UNINIT:
      if (c == INTERMCU_SYNC1) {
        intermcu_data.status++;
      }
      break;
    case LINK_MCU_GOT_SYNC1:
      if (c != INTERMCU_SYNC2) {
        goto error;
      }
      intermcu_data.ck_a = 0;
      intermcu_data.ck_b = 0;
      intermcu_data.status++;
      break;
    case LINK_MCU_GOT_SYNC2:
      if (intermcu_data.msg_available) {
        /* Previous message has not yet been parsed: discard this one */
        goto error;
      }
      intermcu_data.msg_class = c;
      intermcu_data.status++;
      break;
    case LINK_MCU_GOT_CLASS:
      intermcu_data.msg_id = c;
      intermcu_data.status++;
      break;
    case LINK_MCU_GOT_ID:
      intermcu_data.len = c;
      intermcu_data.status++;
      break;
    case LINK_MCU_GOT_LEN1:
      intermcu_data.len |= (c << 8);
      if (intermcu_data.len > INTERMCU_MAX_PAYLOAD) {
        goto error;
      }
      intermcu_data.msg_idx = 0;
      intermcu_data.status++;
      break;
    case LINK_MCU_GOT_LEN2:
      intermcu_data.msg_buf[intermcu_data.msg_idx] = c;
      intermcu_data.msg_idx++;
      if (intermcu_data.msg_idx >= intermcu_data.len) {
        intermcu_data.status++;
      }
      break;
    case LINK_MCU_GOT_PAYLOAD:
      if (c != intermcu_data.ck_a) {
        goto error;
      }
      intermcu_data.status++;
      break;
    case LINK_MCU_GOT_CHECKSUM1:
      if (c != intermcu_data.ck_b) {
        goto error;
      }
      intermcu_data.msg_available = TRUE;
      goto restart;
      break;
    default:
      goto error;
  }
  return;
error:
  intermcu_data.error_cnt++;
restart:
  intermcu_data.status = LINK_MCU_UNINIT;
  return;
}



//////////////////////////////////////////////////////////////////////////////////////////////
// USER



struct fbw_state from_fbw;
struct ap_state  from_ap;

struct fbw_state* fbw_state = &from_fbw;
struct ap_state*  ap_state  = &from_ap;

bool_t inter_mcu_received_fbw = FALSE;
bool_t inter_mcu_received_ap = FALSE;
bool_t link_mcu_received;


inline void parse_mavpilot_msg(void);


#ifdef INTER_MCU_AP
#include "subsystems/datalink/telemetry.h"

#define RC_OK          0
#define RC_LOST        1
#define RC_REALLY_LOST 2


static void send_commands(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_COMMANDS(trans, dev, AC_ID, COMMANDS_NB, ap_state->commands);
}


static void send_fbw_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t rc_status = 0;
  uint8_t fbw_status = 0;
  if (bit_is_set(fbw_state->status, STATUS_MODE_AUTO)) {
    fbw_status = FBW_MODE_AUTO;
  }
  if (bit_is_set(fbw_state->status, STATUS_MODE_FAILSAFE)) {
    fbw_status = FBW_MODE_FAILSAFE;
  }
  if (bit_is_set(fbw_state->status, STATUS_RADIO_REALLY_LOST)) {
    rc_status = RC_REALLY_LOST;
  } else if (bit_is_set(fbw_state->status, RC_OK)) {
    rc_status = RC_OK;
  } else {
    rc_status = RC_LOST;
  }
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &(rc_status), &(fbw_state->ppm_cpt), &(fbw_status), &(fbw_state->vsupply), &(fbw_state->current));
}
#endif

void intermcu_init(void)
{
  intermcu_data.status = LINK_MCU_UNINIT;
  intermcu_data.msg_available = FALSE;
  intermcu_data.error_cnt = 0;
#ifdef INTER_MCU_AP
#if PERIODIC_TELEMETRY
  // If FBW has not telemetry, then AP can send some of the info
  register_periodic_telemetry(DefaultPeriodic, "COMMANDS", send_commands);
  register_periodic_telemetry(DefaultPeriodic, "FBW_STATUS", send_fbw_status);
#endif
#endif
}

void parse_mavpilot_msg(void)
{
  if (intermcu_data.msg_class == MSG_INTERMCU_ID) {
    if (intermcu_data.msg_id == MSG_INTERMCU_COMMAND_ID) {
      for (int i = 0; i < COMMANDS_NB; i++) {
        ap_state->commands[i] = ((pprz_t)MSG_INTERMCU_COMMAND(intermcu_data.msg_buf, i));
      }

#ifdef LINK_MCU_LED
      LED_TOGGLE(LINK_MCU_LED);
#endif
      inter_mcu_received_ap = TRUE;
    } else if (intermcu_data.msg_id == MSG_INTERMCU_RADIO_ID) {

      for (int i = 0; i < RADIO_CONTROL_NB_CHANNEL; i++) {
        fbw_state->channels[i] = ((pprz_t)MSG_INTERMCU_RADIO(intermcu_data.msg_buf, i));
      }
    } else if (intermcu_data.msg_id == MSG_INTERMCU_TRIM_ID) {
      ap_state->command_roll_trim  = ((pprz_t) MSG_INTERMCU_TRIM_ROLL(intermcu_data.msg_buf));
      ap_state->command_pitch_trim = ((pprz_t) MSG_INTERMCU_TRIM_PITCH(intermcu_data.msg_buf));
    } else if (intermcu_data.msg_id == MSG_INTERMCU_FBW_ID) {
      fbw_state->ppm_cpt = MSG_INTERMCU_FBW_MOD(intermcu_data.msg_buf);
      fbw_state->status = MSG_INTERMCU_FBW_STAT(intermcu_data.msg_buf);
      fbw_state->nb_err = MSG_INTERMCU_FBW_ERR(intermcu_data.msg_buf);
      fbw_state->vsupply = MSG_INTERMCU_FBW_VOLT(intermcu_data.msg_buf);
      fbw_state->current = MSG_INTERMCU_FBW_CURRENT(intermcu_data.msg_buf);

#ifdef LINK_MCU_LED
      LED_TOGGLE(LINK_MCU_LED);
#endif
      inter_mcu_received_fbw = TRUE;
    }
  }
}


#ifdef INTER_MCU_AP
void link_mcu_send(void)
{
  InterMcuSend_INTERMCU_COMMAND(ap_state->commands);
  InterMcuSend_INTERMCU_TRIM(ap_state->command_roll_trim, ap_state->command_pitch_trim);
}
#endif

#ifdef INTER_MCU_FBW

void intermcu_on_rc_frame(void)
{
  //TODO
  //inter_mcu_fill_fbw_state(); /** Prepares the next message for AP */

  InterMcuSend_INTERMCU_RADIO(fbw_state->channels);

}
void intermcu_send_status(uint8_t mode)
{
  // Send Status
  InterMcuSend_INTERMCU_FBW(
    fbw_state->ppm_cpt,
    fbw_state->status,
    fbw_state->nb_err,
    electrical.vsupply,
    fbw_state->current);
}

#endif


struct InterMCU inter_mcu;


void intermcu_periodic(void)
{
  if (inter_mcu.time_since_last_frame >= INTERMCU_LOST_CNT) {
    inter_mcu.status = INTERMCU_LOST;
  } else {
    inter_mcu.time_since_last_frame++;
  }
}

void InterMcuEvent(void (*frame_handler)(void))
{
  /* A message has been received */
  if (InterMcuLinkChAvailable()) {
    while (InterMcuLinkChAvailable()) {
      intermcu_parse(InterMcuLinkGetch());
      if (intermcu_data.msg_available) {
        parse_mavpilot_msg();
        intermcu_data.msg_available = FALSE;
        inter_mcu.time_since_last_frame = 0;
        (*frame_handler)();
      }
    }
  }
}
