/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003-2006 Pascal Brisset, Antoine Drouin
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

/** \file main_fbw.c
 *  \brief FBW ( FlyByWire ) process
 *
 *   This process is responsible for decoding radio control, generating actuators 
 * signals either from the radio control or from the commands provided by the 
 * AP (autopilot) process. It also performs a telemetry task and a low level monitoring
 * ( for parameters like the supply )
 */

#include "main_fbw.h"

#include "init_hw.h"
#include "interrupt_hw.h"
#include "led.h"
#include "uart.h"
#include "spi.h"
#include "adc.h"

#include "sys_time.h"
#include "commands.h"
#include "actuators.h"
#include "radio_control.h"
#include "fbw_downlink.h"
#include "autopilot.h"

#ifdef MCU_SPI_LINK
#include "link_mcu.h"
#endif
#ifdef ADC
struct adc_buf vsupply_adc_buf;
#endif

uint8_t fbw_vsupply_decivolt;

uint8_t fbw_mode;

#include "inter_mcu.h"

/********** INIT *************************************************************/
void init_fbw( void ) {
  hw_init();
  sys_time_init();
#ifdef LED
  led_init();
#endif
#ifdef USE_UART0
  uart0_init_tx();
#endif
#ifdef ADC
  adc_init();
  adc_buf_channel(ADC_CHANNEL_VSUPPLY, &vsupply_adc_buf, DEFAULT_AV_NB_SAMPLE);
#endif
#ifdef ACTUATORS
  actuators_init();
  /* Load the failsafe defaults */
  SetCommands(commands_failsafe);
#endif
#ifdef RADIO_CONTROL
 ppm_init();
#endif
#ifdef MCU_SPI_LINK
  spi_init();
#endif

  int_enable();
}


void set_failsafe_mode( void ) {
  fbw_mode = FBW_MODE_FAILSAFE;
  SetCommands(commands_failsafe);
}


/********** EVENT ************************************************************/
void event_task_fbw( void) {

#ifdef RADIO_CONTROL
  if (ppm_valid) {
    ppm_valid = FALSE;
    radio_control_event_task();
    if (rc_values_contains_avg_channels) {
      fbw_mode = FBW_MODE_OF_PPRZ(rc_values[RADIO_MODE]);
    }
    if (fbw_mode == FBW_MODE_MANUAL)
      SetCommandsFromRC(commands);
  }
#endif

#ifdef ADC
  fbw_vsupply_decivolt = VoltageOfAdc(vsupply_adc_buf.sum/vsupply_adc_buf.av_nb_sample) * 10;
#endif

#ifdef INTER_MCU
  if (from_ap_receive_valid) {
    from_ap_receive_valid = FALSE;
    inter_mcu_event_task();
    if (fbw_mode == FBW_MODE_AUTO) {
      SetCommands(from_ap.from_ap.commands);
    }

  }
#endif

#ifdef MCU_SPI_LINK
  //  if ( !SpiIsSelected() && spi_was_interrupted ) {
  //    spi_was_interrupted = FALSE;
  //    spi_reset();
  //  }
  if (!link_mcu_is_busy && link_mcu_was_busy) {
    link_mcu_was_busy = FALSE;
    link_mcu_restart();
  }
#endif

}

/************* PERIODIC ******************************************************/
void periodic_task_fbw( void ) {

#ifdef RADIO_CONTROL
  radio_control_periodic_task();
  if (fbw_mode == FBW_MODE_MANUAL && rc_status == RC_REALLY_LOST) {
    fbw_mode = FBW_MODE_AUTO;
  }
#endif

#ifdef INTER_MCU
  inter_mcu_periodic_task();
  if (fbw_mode == FBW_MODE_AUTO && !ap_ok)
    set_failsafe_mode();
#endif

#ifdef DOWNLINK
  static uint8_t _10Hz; /* FIXME : sys_time should provide it */
  _10Hz++;
  if (_10Hz >= 6) _10Hz = 0;
  if (!_10Hz)
    fbw_downlink_periodic_task();
#endif

#ifdef ACTUATORS
  SetActuatorsFromCommands(commands);
#endif
}
