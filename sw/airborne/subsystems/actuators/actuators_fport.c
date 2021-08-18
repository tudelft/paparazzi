/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file actuators_fport.c
 *  Fport actuator driver, which can output as 7 fport channels at ~11ms.
 *  Channels min, averga and maximum should be: 340, 1024, 1708
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_fport.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"

/* Currently we only support 7 channels */
#if SERVOS_FPORT_NB > ACTUATORS_FPORT_MAX_NB
#error FPORT actuators only support less then 7 servos
#endif

/* Calculate the frequency divider to aim at 7 ms */
#if PERIODIC_FREQUENCY < 150
#error Fport actuators need at leest a frequency of 150 Hz
#else
static uint8_t freq_trig = PERIODIC_FREQUENCY / 142.0 + 0.5; // Round it to nearest value
#endif

/* Main actuator structure */
struct ActuatorsFport actuators_fport;
static inline void actuators_fport_send(struct link_device *dev);

/*
 * Initialize the fport devices (UART output devices)
 */
void actuators_fport_init(void)
{
  actuators_fport.device = &((ACTUATORS_FPORT_DEV).device);

  uart_periph_set_bits_stop_parity(&ACTUATORS_FPORT_DEV, UBITS_8, USTOP_1, UPARITY_NO);
  uart_periph_set_baudrate(&ACTUATORS_FPORT_DEV, B115200);
  //uart_periph_invert_data_logic(&ACTUATORS_FPORT_DEV, false, false);
}

/*
 * Transmit the fport output at 1 / 7ms
 */
void actuators_fport_set(void)
{
  static uint8_t cnt = 0;

  // Only send every 7 ms
  cnt++;
  if (cnt == freq_trig) {
    actuators_fport_send(actuators_fport.device);
    cnt = 0;
  }
}

/*
 * Actually transmit the fport output on a link device
 *
 * The protocol is 25 Byte long and is send every 14ms (analog mode) or 7ms (highspeed mode).
 * One Byte = 1 startbit + 8 databit + 1 paritybit + 2 stopbit (8E2), baudrate = 100'000 bit/s
 * The highest bit is send first. The logic is inverted (Level High = 1)
 *
 * [startbyte] [data1] [data2] .... [data22] [flags][endbyte]
 *
 * FPORT protocol
 */

#define FPORT_START_BYTE  0x7E
#define FPORT_END_BYTE    0x7E
#define FPORT_DLE         0x7D
#define FPORT_XOR         0x20

#define FPORT_BIT_PER_CHANNEL 11

static void calc_crc(uint16_t *crc, uint8_t byte) {
  *crc += byte;
  *crc += *crc >> 8;
  *crc &= 0xFF;
}

static inline void actuators_fport_send(struct link_device *dev)
{
  uint8_t i = 0;
  uint16_t crc = 0;
  uint8_t bits_sent = 0;
  //uint8_t data[22];

  /* start */
  dev->put_byte(dev->periph, 0, FPORT_START_BYTE);
  dev->put_byte(dev->periph, 0, 0x19); // Length
  calc_crc(&crc, 0x19);
  dev->put_byte(dev->periph, 0, 0x00); // Type
  calc_crc(&crc, 0x00);

  /* Fill all channels */
  // actuators_fport.cmds[0] = 1500;
  // for (i = 0; i < 22; i++) {
  //   data[i] = 0x00;
  // }
  // for (i = 0; i < ACTUATORS_FPORT_MAX_NB; i++) {
  //   uint16_t chn = actuators_fport.cmds[i] & 0x07ff; // 11 bit
  //   uint8_t ind = bits_sent / FPORT_BIT_PER_CHANNEL;
  //   uint8_t shift = bits_sent % FPORT_BIT_PER_CHANNEL;
  //   data[ind] |= (chn >> (3 + shift)) & 0xff; // Sends (8 - shift) bits of the 11: 11-(8-shift) remain = 3 + shift
  //   if (shift > 5) { // need 3 bytes to fit the 11 bits
  //     data[ind + 1] |= (chn >> (shift - 5)) & 0xff; // Sends next 8
  //     data[ind + 2] |= (chn << (3 - shift)) &
  //                      0xff; // Sends remaining 3 + shift - 8 bits = shift-5 bits: left aligned: 8 - (shift-5) = 3-shift
  //   } else { // (shift <= 5) then it fits in 2 bytes
  //     data[ind + 1] |= (chn << (5 - shift)) &
  //     0xff; // Sends remaining 3 + shift bits left aligned: 8 - (3 + shift) = 5 - shift
  //   }
  //   bits_sent += FPORT_BIT_PER_CHANNEL;
  // }

  uint8_t data[22] = {0x43, 0x03, 0xDE, 0xD0, 0xD0, 0x97, 0x3E, 0x56, 0x4C, 0x9C, 0x15, 0xAC, 0x48, 0xDF, 0xC4, 0x93, 0x07, 0x3E, 0xF0, 0x41, 0x7B, 0xE2};

  /* Transmit all channels */
  for (i = 0; i < 22; i++) {
    if(data[i] == FPORT_START_BYTE || data[i] == FPORT_DLE) {
      dev->put_byte(dev->periph, 0, FPORT_DLE);
      calc_crc(&crc, FPORT_DLE);
      dev->put_byte(dev->periph, 0, data[i] ^ FPORT_XOR);
      calc_crc(&crc, data[i] ^ FPORT_XOR);
    }
    else {
      dev->put_byte(dev->periph, 0, data[i]);
      calc_crc(&crc, data[i]);
    }
  }

  /* flags */
  dev->put_byte(dev->periph, 0, 0x00); // No frame lost, switches off, no failsafe
  calc_crc(&crc, 0x00);

  /* RSSI */
  dev->put_byte(dev->periph, 0, 0x0F);
  calc_crc(&crc, 0x0F);

  /* CRC */
  uint8_t crc8 = 0xff - ((crc & 0xff) + (crc >> 8));
  if(crc8 == FPORT_START_BYTE || crc8 == FPORT_DLE) {
    dev->put_byte(dev->periph, 0, FPORT_DLE);
    dev->put_byte(dev->periph, 0, crc8 ^ FPORT_XOR);
  }
  else {
    dev->put_byte(dev->periph, 0, crc8);
  }

  /* End Byte */
  dev->put_byte(dev->periph, 0, FPORT_END_BYTE);

  /*uint8_t data2[12] = {0x7E, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF6, 0x7E};
  for(i = 0; i < 12; i++)
    dev->put_byte(dev->periph, 0, data2[i]);*/
}
