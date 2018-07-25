/*
 * Copyright (C) 2010 Martin Muller
 * Copyright (C) 2016 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/adcs/adc_generic.c
 *
 * This module can be used to read one or two values from the ADC channels
 * in a generic way. Data is reported through the default telemetry
 * channel (by default) or can be redirected to an other one (alternate
 * telemetry, datalogger) at a frequency defined in the telemetry xml file.
 *
 */

#include "adc_generic.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

uint16_t adc_val1;
uint16_t adc_val2;
uint16_t adc_val3;
uint16_t adc_val4;
uint16_t servo_pitch;
uint16_t servo_yaw;

#ifndef ADC_CHANNEL_GENERIC_NB_SAMPLES
#define ADC_CHANNEL_GENERIC_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#ifndef ADC_GENERIC_PERIODIC_SEND
#define ADC_GENERIC_PERIODIC_SEND TRUE
#endif

static struct adc_buf buf1;
static struct adc_buf buf2;
static struct adc_buf buf3;
static struct adc_buf buf4;

static void adc_msg_send(struct transport_tx *trans, struct link_device *dev) {
  adc_val1 = buf1.sum / buf1.av_nb_sample;
  adc_val2 = buf2.sum / buf2.av_nb_sample;
  adc_val3 = buf3.sum / buf3.av_nb_sample;
  adc_val4 = buf4.sum / buf4.av_nb_sample;
  servo_pitch = 10000*adc_val1/adc_val3;
  servo_yaw = 10000*adc_val2/adc_val4;

  pprz_msg_send_ADC_GENERIC(trans, dev, AC_ID, &adc_val1, &adc_val2, &adc_val3, &adc_val4, &servo_pitch, &servo_yaw);
}

void adc_generic_init(void)
{
  adc_val1 = 0;
  adc_val2 = 0;
  adc_val3 = 0;
  adc_val4 = 0;
  servo_pitch = 0;
  servo_yaw = 0;

  adc_buf_channel(ADC_2, &buf1, ADC_CHANNEL_GENERIC_NB_SAMPLES);
  adc_buf_channel(ADC_3, &buf2, ADC_CHANNEL_GENERIC_NB_SAMPLES);
  adc_buf_channel(ADC_4, &buf3, ADC_CHANNEL_GENERIC_NB_SAMPLES);
  adc_buf_channel(ADC_6, &buf4, ADC_CHANNEL_GENERIC_NB_SAMPLES);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ADC_GENERIC, adc_msg_send);
#endif

}

void adc_generic_periodic(void)
{
#if ADC_GENERIC_PERIODIC_SEND
  adc_msg_send(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
}

