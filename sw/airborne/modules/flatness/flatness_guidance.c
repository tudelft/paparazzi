/*
 * Copyright (C) 2026 Evangelos Ntouros <e.ntouros@tudelft.nl>
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

/** @file "modules/flatness/flatness_guidance.c"
 * @author Evangelos Ntouros <e.ntouros@tudelft.nl>
 * Differential flatness based guidance.
 */

#include "modules/flatness/flatness_guidance.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "modules/radio_control/radio_control.h"
#include "modules/radio_control/rc_datalink.h"
#include "modules/core/abi.h"


static abi_event rc_ev;
static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc);

struct ThrustSetpoint thrust_sp;

void flatness_guidance_init(void)
{
     AbiBindMsgRADIO_CONTROL(ABI_BROADCAST, &rc_ev, rc_cb);
}

struct ThrustSetpoint get_thrust(void)
{
    return thrust_sp;
}

static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc)
{
    int32_t rc_throttle = (int32_t)rc->values[RADIO_THROTTLE];
    printf("throttle = %d", rc_throttle);

    // THRUST_SP_SET_ZERO(thrust_sp);
    thrust_sp = th_sp_from_thrust_i(rc_throttle, THRUST_AXIS_Z);
}

