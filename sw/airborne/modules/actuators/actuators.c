/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2012 Gautier Hattenberger
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

/** @file modules/actuators/actuators.c
 *  Hardware independent actuators code.
 *
 */

#include "modules/actuators/actuators.h"
#include "modules/core/commands.h"
#include "mcu_periph/sys_time.h"
#include "modules/core/abi.h"
#include "modules/radio_control/radio_control.h"
#include "modules/sensors/serial_act_t4.h"
#ifdef INTERMCU_AP
#include "modules/intermcu/intermcu_ap.h"
#endif
#ifdef INTERMCU_FBW
#include "main_fbw.h"
#endif

#if ACTUATORS_NB

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_actuators(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ACTUATORS(trans, dev, AC_ID , ACTUATORS_NB, actuators);
}
#endif

int16_t actuators[ACTUATORS_NB];

// Can be used to directly control each actuator from the control algorithm
int16_t actuators_pprz[ACTUATORS_NB];

uint32_t actuators_delay_time;
bool   actuators_delay_done;

void actuators_init(void)
{

#if defined ACTUATORS_START_DELAY && ! defined SITL
  actuators_delay_done = false;
  SysTimeTimerStart(actuators_delay_time);
#else
  actuators_delay_done = true;
  actuators_delay_time = 0;
#endif

  // Init macro from generated airframe.h
#if (defined INTERMCU_AP)
  // TODO ApOnlyActuatorsInit();
#elif (defined INTERMCU_FBW)
  AllActuatorsInit();
#else
  // default, init all actuators
  AllActuatorsInit();
  // TODO ApOnlyActuatorsInit();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS, send_actuators);
#endif
}

/** Actuators periodic
 *
 *  Set actuators from trimmed commands
 */
void actuators_periodic(void)
{
#if USE_COMMANDS
  pprz_t trimmed_commands[COMMANDS_NB];
  int i;
  for (i = 0; i < COMMANDS_NB; i++) {trimmed_commands[i] = commands[i];}

#ifdef COMMAND_ROLL
  trimmed_commands[COMMAND_ROLL] += ClipAbs(command_roll_trim, MAX_PPRZ / 10);
#endif /* COMMAND_ROLL */

#ifdef COMMAND_PITCH
  trimmed_commands[COMMAND_PITCH] += ClipAbs(command_pitch_trim, MAX_PPRZ / 10);
#endif /* COMMAND_PITCH */

#ifdef COMMAND_YAW
  trimmed_commands[COMMAND_YAW] += ClipAbs(command_yaw_trim, MAX_PPRZ);
#endif /* COMMAND_YAW */

#if (defined INTERMCU_AP)
  intermcu_send_commands(trimmed_commands, autopilot_get_mode());
  // TODO SetApOnlyActuatorsFromCommands(ap_commands, autopilot_get_mode());
#elif (defined INTERMCU_FBW)
  SetActuatorsFromCommands(trimmed_commands, autopilot_get_mode());
#else
  // default, apply all commands
  SetActuatorsFromCommands(trimmed_commands, autopilot_get_mode());

  //trimmed_commands[COMMAND_ROLL]
  //trimmed_commands[ELEVON_LEFT]
  //trimmed_commands[COMMAND_YAW]
  

  float extra = 0;
  struct serial_act_t4_out test;
  test.servo_arm_int = 1;

  float offset = 0; //deg * 100

#ifdef SERVO_ELEVON_LEFT_IDX
  test.servo_6_cmd_int = 2.0 * 3000.0 * ( 1.0f * actuators[SERVO_ELEVON_LEFT_IDX] - 1500.0) / 500 - 2.0 * offset;
#endif
#ifdef SERVO_ELEVON_RIGHT_IDX
  test.servo_4_cmd_int = 2.0 * 3000.0 * ( 1.0f * actuators[SERVO_ELEVON_RIGHT_IDX] - 1500.0) / 500 + 2.0 * offset;
#endif
#ifdef SERVO_ARM_LEFT_IDX
  test.servo_1_cmd_int = 45.0/19.0 * 4000.0 * ( 1.0f * actuators[SERVO_ARM_LEFT_IDX] - 1500.0) / 500 - 45.0/19.0 * offset;
#endif
#ifdef SERVO_ARM_RIGHT_IDX
  test.servo_3_cmd_int = 45.0/19.0 * 4000.0 * ( 1.0f * actuators[SERVO_ARM_RIGHT_IDX] - 1500.0) / 500 + 45.0/19.0 * offset;
#endif
#ifdef SERVO_ELEVATOR_IDX
  test.servo_5_cmd_int = 39.0/19.0 * 2500.0 * (( 1.0f * actuators[SERVO_ELEVATOR_IDX] - 1500.0) / 500) - 39.0/19.0 * 1500.0 * (radio_control.values[6] / 9600.0);
#endif
#ifdef SERVO_SPOILERON_IDX
  test.servo_2_cmd_int = 45.0/19.0 * 3000.0 * ( 1.0f * actuators[SERVO_SPOILERON_IDX] - 1500.0) / 500;
#endif
  
  AbiSendMsgSERIAL_ACT_T4_OUT(ABI_SERIAL_ACT_T4_OUT_ID, &test, &extra);
  // TODO SetApOnlyActuatorsFromCommands(ap_commands, autopilot_get_mode());
#endif
#endif // USE_COMMANDS
}

#else // No command_laws section or no actuators

void actuators_init(void) {}
void actuators_periodic(void) {}

#endif
