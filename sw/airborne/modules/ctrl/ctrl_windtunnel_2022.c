/*
 * Copyright (C) 2022 Tomaso De Ponti <tomasodp@gmail.com>
 *
 * This file is part of paparazzi
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

/** @file "modules/ctrl/ctrl_windtunnel_2022.c"
 * @author Tomaso De Ponti <tomasodp@gmail.com>
 * Module to test actuator effectiveness of rotwingdrone
 */

#include "modules/ctrl/ctrl_windtunnel_2022.h"
#include "modules/system_identification/extended_eff_message.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "subsystems/actuators.h"
#include "modules/system_identification/sys_id_doublet.h"
#include "modules/system_identification/sys_id_chirp.h"
#include "modules/rot_wing_drone/wing_rotation_controller.h"

static void windtunnel_control(struct transport_tx *trans, struct link_device *dev)
{
 
}

void extended_eff_message_init(void)
{

}
