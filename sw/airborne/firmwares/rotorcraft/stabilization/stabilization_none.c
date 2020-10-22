/*
 * Copyright (C) 2011-2012 The Paparazzi Team
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

/** @file stabilization_none.c
 *  Dummy stabilization for rotorcrafts.
 *
 *  Doesn't actually do any stabilization,
 *  just directly passes the RC commands along.
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_none.h"

#include "subsystems/radio_control.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"
//#include "state.h"
#include "modules/fbw_settings/fbw_settings.h"

struct Int32Rates stabilization_none_rc_cmd;

void stabilization_none_init(void)
{
  INT_RATES_ZERO(stabilization_none_rc_cmd);
}

void stabilization_none_read_rc(void)
{

  stabilization_none_rc_cmd.p = (int32_t)radio_control.values[RADIO_YAW]/4;
  stabilization_none_rc_cmd.q = (int32_t)radio_control.values[RADIO_PITCH]/2;
  stabilization_none_rc_cmd.r = -(int32_t)radio_control.values[RADIO_ROLL]/4;
}

void stabilization_none_enter(void)
{
  INT_RATES_ZERO(stabilization_none_rc_cmd);
}

void stabilization_none_run(bool in_flight __attribute__((unused)))
{
  //float pitchrate = stateGetBodyRates_f()->q; //RATE_FLOAT_OF_BFP(imu.gyro.p);
  float pitchrate = RATE_FLOAT_OF_BFP(imu.gyro.q);
  //float yawrate = stateGetBodyRates_f()->p;//RATE_FLOAT_OF_BFP(imu.gyro.r); // Radians/second
  float yawrate = RATE_FLOAT_OF_BFP(imu.gyro.r); // Radians/second
  /* just directly pass rc commands through */

  int32_t pitchdamper =  (int32_t) (pitchrate * fbw_setting_damper_pitch);
  Bound(pitchdamper , -3000, 3000);
  int32_t yawdamper =  (int32_t) (yawrate * fbw_setting_damper_yaw);
  Bound(yawdamper , -3000, 3000);

  stabilization_cmd[COMMAND_ROLL]  = stabilization_none_rc_cmd.p + yawdamper;
  stabilization_cmd[COMMAND_PITCH] = stabilization_none_rc_cmd.q + pitchdamper;
  stabilization_cmd[COMMAND_YAW]   = stabilization_none_rc_cmd.r;
}
