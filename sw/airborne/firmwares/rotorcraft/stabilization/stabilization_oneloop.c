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
#include "firmwares/rotorcraft/stabilization/stabilization_oneloop.h"

#include "modules/radio_control/radio_control.h"
#include "generated/airframe.h"
#include "generated/modules.h"


struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
struct FloatRates  stab_att_ff_rates;



void stabilization_attitude_init(void)
{
  // indi init is already done through module init

}

void stabilization_attitude_enter(void)
{

}

void stabilization_attitude_set_failsafe_setpoint(void)
{

}

void stabilization_attitude_set_rpy_setpoint_i(UNUSED struct Int32Eulers *rpy)
{

}

void stabilization_attitude_set_quat_setpoint_i(UNUSED struct Int32Quat *quat)
{

}

void stabilization_attitude_set_earth_cmd_i(UNUSED struct Int32Vect2 *cmd, int32_t heading)
{

}

void stabilization_attitude_set_stab_sp(UNUSED struct StabilizationSetpoint *sp)
{

}

void stabilization_attitude_run(UNUSED bool in_flight)
{
  //if att mode
  // half loop

}


void stabilization_attitude_read_rc(UNUSED bool in_flight, bool in_carefree, bool coordinated_turn)
{

}
