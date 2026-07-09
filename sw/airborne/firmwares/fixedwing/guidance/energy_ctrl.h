/*
 * Copyright (C) 2012 TUDelft, Tobias Muench
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
 * @file firmwares/fixedwing/guidance/energy_ctrl.h
 * Vertical control using total energy control for fixed wing vehicles.
 *
 */

#ifndef FW_V_CTL_ENERGY_H
#define FW_V_CTL_ENERGY_H

#include "firmwares/fixedwing/guidance/guidance_common.h"

//To be unified with control NEW, these can be used in a unified flightplan
//but ETECS has no use of it ofcourse, speed mode always V_CTL_SPEED_AIRSPEED
//FIXME An idea to move it to guidance_common
#define V_CTL_SPEED_THROTTLE    0
#define V_CTL_SPEED_AIRSPEED    1
#define V_CTL_SPEED_GROUNDSPEED 2

extern uint8_t v_ctl_speed_mode;

/* outer loop */
// extern float v_ctl_altitude_error;    ///< in meters, (setpoint - alt) -> positive = too low
extern float v_ctl_altitude_setpoint; ///< in meters above MSL
extern float v_ctl_altitude_pre_climb; ///< Path Angle
extern float v_ctl_altitude_pgain;
extern float v_ctl_altitude_accel_capture; ///< m/s^2, constant-deceleration altitude capture, 0 = classic linear law
extern float v_ctl_airspeed_pgain;

extern float v_ctl_auto_airspeed_setpoint; ///< in meters per second

extern float v_ctl_max_climb;
extern float v_ctl_max_acceleration;

/* "auto throttle" inner loop parameters */
extern float v_ctl_desired_acceleration;

extern float v_ctl_auto_throttle_nominal_cruise_throttle;
extern float v_ctl_auto_throttle_nominal_cruise_pitch;
extern float v_ctl_auto_throttle_climb_throttle_increment;
extern float v_ctl_auto_throttle_pitch_of_vz_pgain;

extern float v_ctl_auto_throttle_of_airspeed_pgain;
extern float v_ctl_auto_throttle_of_airspeed_igain;
extern float v_ctl_auto_pitch_of_airspeed_pgain;
extern float v_ctl_auto_pitch_of_airspeed_igain;
extern float v_ctl_auto_pitch_of_airspeed_dgain;

extern float v_ctl_energy_total_pgain;
extern float v_ctl_energy_total_igain;

extern float v_ctl_energy_diff_pgain;
extern float v_ctl_energy_diff_igain;

/** Bank-angle energy feedforward.
 *
 * In a coordinated turn the wing must produce lift for a load factor
 * n = 1/cos(phi), and the induced drag grows with n^2. This energy loss
 * is perfectly predictable from the commanded bank, so it can be paid
 * for the moment the aircraft rolls, instead of waiting for the
 * altitude/speed feedback loops to detect it (1-2 s later).
 *
 * throttle += bank_throttle_gain * (1/cos^2(phi) - 1)    [steady physics]
 *          +  bank_washout_gain  * washout(1/cos^2 - 1)   [transient lead]
 * pitch    += bank_pitch_gain    * (1/cos(phi)   - 1)     [distribution]
 *
 * The washout term is a high-passed copy (time constant
 * V_CTL_ENERGY_BANK_WASHOUT_TAU, default 1 s): full authority during
 * the roll-in where the throttle/prop/speed-loop lag lives, zero in a
 * sustained turn (no mid-turn energy surplus), negative on roll-out
 * (suppresses the exit balloon).
 *
 * All default to 0 (feature disabled, no behaviour change).
 * The bank angle used is the larger of the commanded roll setpoint and
 * the measured roll (command leads the turn: true feedforward), limited
 * to 60 deg so a knife-edge upset can never command more than 3x the
 * gain value; the terms are purely additive, are NOT integrated (no
 * trim/adaptation pollution), and vanish wings-level.
 *
 * Note: if H_CTL_PITCH_OF_ROLL is already used at stabilization level,
 * leave bank_pitch_gain at 0 to avoid compensating twice.
 */
extern float v_ctl_energy_bank_throttle_gain;
extern float v_ctl_energy_bank_washout_gain;
extern float v_ctl_energy_bank_pitch_gain;

extern float v_ctl_auto_groundspeed_pgain;
extern float v_ctl_auto_groundspeed_igain;
extern float v_ctl_auto_groundspeed_sum_err;

/////////////////////////////////////////////////
// Automatically found airplane characteristics

extern float ac_char_climb_pitch;
extern float ac_char_climb_max;
extern float ac_char_descend_pitch;
extern float ac_char_descend_max;
extern float ac_char_cruise_throttle;
extern float ac_char_cruise_pitch;

#endif /* FW_V_CTL_H */
