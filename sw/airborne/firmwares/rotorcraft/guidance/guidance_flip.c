/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2015 Ewoud Smeur
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
 * @file firmwares/rotorcraft/guidance/guidance_flip.c
 *
 * Open Loop guidance for making a flip. You need to tune this before using.
 * When entering this mode it saves the previous guidance mode and changes AUTO2 back to
 * the previous mode after finishing the flip.
 * Use it with caution!
 */

#include "guidance_flip.h"
#include "paparazzi.h"
#include "math/pprz_algebra_int.h"

#include "autopilot.h"
#include "guidance_h.h"
#include "stabilization/stabilization_attitude_rc_setpoint.h"
#include "stabilization/stabilization_attitude.h"
#include "stabilization/stabilization_attitude_quat_int.h"

#include "subsystems/radio_control.h"

#ifndef MANEUVER_PITCH_DOUBLET
#define MANEUVER_PITCH_DOUBLET 1
#endif
#ifndef MANEUVER_PITCH_PULSE
#define MANEUVER_PITCH_PULSE 2
#endif
#ifndef INITIAL_IDLE_TIME
#define INITIAL_IDLE_TIME 1.0
#endif
#ifndef FINAL_IDLE_TIME
#define FINAL_IDLE_TIME 1.0
#endif



uint32_t maneuver_counter;
uint8_t flip_state;
struct Int32Quat nominal_quaternion_cmd;
int32_t dblt_angle;
float pulse_duration;
uint8_t maneuver_type;
uint32_t timer;
static uint32_t timer_save;
int32_t nominal_thrust_cmd;
struct Int32Quat deviation_quaternion;
struct Int32Vect3 rot_axis;
//struct Int32Quat new_stab_att_quat_sp;

void guidance_flip_enter(void)
{ 
  maneuver_counter = 0;
  flip_state = 0;
  stabilization_attitude_read_rc(autopilot_in_flight, FALSE, FALSE);
  QUAT_COPY(nominal_quaternion_cmd,stab_att_sp_quat);
  nominal_thrust_cmd = stabilization_cmd[COMMAND_THRUST];
  timer_save = 0;

}

void guidance_flip_run(void)
{
  timer = (maneuver_counter++ << 12) / PERIODIC_FREQUENCY;
  
  switch (flip_state) {
    case 0:
      // trimmed flight before maneuver
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST] = nominal_thrust_cmd; //Thrust to go up first
      timer_save = 0;
      //FIXME make pulse_duration a setting
      if (timer > BFP_OF_REAL(INITIAL_IDLE_TIME, 12)) {
        flip_state++;
      }
      break;

    case 1:
      
      // Beginning of doublet or positive part of pulse
      // Select pitch axis
      rot_axis.x = 0;
      rot_axis.y = 1;
      rot_axis.z = 0;
      //FIXME make angle and axis a setting
      int32_quat_of_axis_angle(&deviation_quaternion, &rot_axis, &dblt_angle);
      int32_quat_comp(&stab_att_sp_quat, &nominal_quaternion_cmd, &deviation_quaternion);
      //stab_att_quat_sp = new_stab_att_quat_sp;
      stabilization_attitude_run(autopilot_in_flight);
      stabilization_cmd[COMMAND_THRUST] = nominal_thrust_cmd; 
      
      if (timer > BFP_OF_REAL(pulse_duration, 12)) {
	switch (maneuver_type) {
	  case MANEUVER_PITCH_DOUBLET:
	     flip_state++;
             break;
	  case MANEUVER_PITCH_PULSE:
	     flip_state = 3;
             break;
          default:
             flip_state = 99;
    	     break;
	}
      }
      break;

    case 2:
      rot_axis.x = 0;
      rot_axis.y = -1;
      rot_axis.z = 0;
      
      int32_quat_of_axis_angle(&deviation_quaternion, &rot_axis, &dblt_angle);
      int32_quat_comp(&stab_att_sp_quat, &nominal_quaternion_cmd, &deviation_quaternion);
      //stab_att_quat_sp = new_stab_att_quat_sp;
      stabilization_attitude_run(autopilot_in_flight);
      stabilization_cmd[COMMAND_THRUST] = nominal_thrust_cmd; 

      if (timer > BFP_OF_REAL(pulse_duration, 12)) {
        timer_save = timer;
        flip_state++;
      }
      break;

    case 3:
      stab_att_sp_quat = nominal_quaternion_cmd;
      stabilization_attitude_run(autopilot_in_flight);
      stabilization_cmd[COMMAND_THRUST] = nominal_thrust_cmd; 
      
      if ((timer - timer_save) > BFP_OF_REAL(FINAL_IDLE_TIME, 12)) {
        flip_state++;
      }
      break;

    default:

      maneuver_counter = 0;
      timer_save = 0;
      stabilization_attitude_read_rc(autopilot_in_flight, FALSE, FALSE);
      stabilization_attitude_run(autopilot_in_flight);
      stabilization_cmd[COMMAND_THRUST] = radio_control.values[RADIO_THROTTLE];
      break;
  }
}
