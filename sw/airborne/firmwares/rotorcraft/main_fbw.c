/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file firmwares/rotorcraft/main_fbw.c
 *
 * Rotorcraft FBW main loop.
 */

#include <inttypes.h>
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif


#include "subsystems/electrical.h"

#include "subsystems/radio_control.h"

#include "firmwares/rotorcraft/main_fbw.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "firmwares/rotorcraft/intermcu.h"

//#include "generated/modules.h"

/** Fly by wire modes */
#define FBW_MODE_MANUAL   0
#define FBW_MODE_AUTO     1
#define FBW_MODE_FAILSAFE 2

uint8_t fbw_mode;
bool_t intermcu_realy_lost;


/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
//PRINT_CONFIG_VAR(MODULES_FREQUENCY)

tid_t main_periodic_tid; ///< id for main_periodic() timer
//tid_t modules_tid;       ///< id for modules_periodic_task() timer
tid_t failsafe_tid;      ///< id for failsafe_check() timer
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer
tid_t electrical_tid;    ///< id for electrical_periodic() timer
tid_t telemetry_tid;     ///< id for telemetry_periodic() timer

#ifndef SITL
int main(void)
{
  main_init();

  while (1) {
    handle_periodic_tasks();
    main_event();
  }

  return 0;
}
#endif /* SITL */

STATIC_INLINE void main_init(void)
{
  // fbw_init
  fbw_mode = FBW_MODE_FAILSAFE;
  intermcu_realy_lost = TRUE;

  mcu_init();

  electrical_init();

  actuators_init();
#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

  radio_control_init();

  // TODO
  //modules_init();

  mcu_int_enable();

  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
//  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
  radio_control_tid = sys_time_register_timer((1. / 60.), NULL);
  failsafe_tid = sys_time_register_timer(0.05, NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
}


STATIC_INLINE void handle_periodic_tasks(void)
{
  if (sys_time_check_and_ack_timer(main_periodic_tid)) {
    main_periodic();
  }
  //if (sys_time_check_and_ack_timer(modules_tid)) {
    // TODO
    //modules_periodic_task();
  //}
  if (sys_time_check_and_ack_timer(radio_control_tid)) {
    radio_control_periodic_task();
  }
  if (sys_time_check_and_ack_timer(failsafe_tid)) {
    failsafe_check();
  }
  if (sys_time_check_and_ack_timer(electrical_tid)) {
    electrical_periodic();
  }
}

STATIC_INLINE void main_periodic(void)
{

  intermcu_periodic();
  /* run control loops */
  // TODO
  //autopilot_periodic();
  /* set actuators     */
  //actuators_set(autopilot_motors_on);
  SetActuatorsFromCommands(commands, autopilot_mode);

  RunOnceEvery(10, LED_PERIODIC());
}

#define THRESHOLD_1_PPRZ (MIN_PPRZ / 2)

/** get autopilot fbw mode as set by RADIO_MODE 3-way switch */
static uint8_t fbw_mode_of_3way_switch(void)
{
  if (radio_control.values[RADIO_MODE] < THRESHOLD_1_PPRZ) {
    return FBW_MODE_MANUAL;
  }
  else {
    return FBW_MODE_AUTO;
  }
}

/** mode to enter when RC is lost while using a mode with RC input (not AP_MODE_NAV) */
#ifndef AP_LOST_FBW_MODE
#define AP_LOST_FBW_MODE FBW_MODE_FAILSAFE
#endif

void fbw_set_mode(uint8_t new_fbw_mode);
void fbw_set_mode(uint8_t new_fbw_mode) {
  if (new_fbw_mode == FBW_MODE_AUTO)
  if (intermcu_realy_lost && (new_fbw_mode == FBW_MODE_AUTO))
  {
    new_fbw_mode = AP_LOST_FBW_MODE;
  }

  fbw_mode = new_fbw_mode;
  if (fbw_mode == FBW_MODE_FAILSAFE) {
    SetCommands(commands_failsafe);
  }
}

void autopilot_on_rc_frame(void);
void autopilot_on_rc_frame(void)
{
  uint8_t new_autopilot_mode = fbw_mode_of_3way_switch();

  fbw_set_mode(new_autopilot_mode);

  /* if there are some commands that should always be set from RC, do it */
#ifdef SetAutoCommandsFromRC
  SetAutoCommandsFromRC(commands, radio_control.values);
#endif

  /* if manual */
  if (fbw_mode == FBW_MODE_MANUAL) {

    /* if not in NAV_MODE set commands from the rc */
#ifdef SetCommandsFromRC
    SetCommandsFromRC(commands, radio_control.values);
#else
#warning "FBW: needs commands from RC in order to be useful."
#endif

  }
}


/** mode to enter when RC is lost while using a mode with RC input (not AP_MODE_NAV) */
#ifndef RC_LOST_FBW_MODE
#define RC_LOST_FBW_MODE FBW_MODE_FAILSAFE
#endif

STATIC_INLINE void failsafe_check(void)
{
  // Loose RC while using RC
  if (radio_control.status == RC_REALLY_LOST &&
      fbw_mode != FBW_MODE_AUTO) {
    fbw_set_mode(RC_LOST_FBW_MODE);
  }
/*  // Loose AP while using AP
  if (intermcu_realy_lost && (fbw_mode == FBW_MODE_AUTO))
  {
     fbw_set_mode(AP_LOST_FBW_MODE);
  }
*/
}



STATIC_INLINE void main_event(void)
{
  /* event functions for mcu peripherals: i2c, usb_serial.. */
  mcu_event();

  // Handle RC
  RadioControlEvent(autopilot_on_rc_frame);

  // InterMCU
  intermcu_event();

  // TODO Modules
  //modules_event_task();
}
