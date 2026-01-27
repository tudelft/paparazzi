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

void flatness_guidance_init(void)
{
     AbiBindMsgRADIO_CONTROL(ABI_BROADCAST, &rc_ev, rc_cb);
}

struct ThrustSetpoint get_thrust(void)
{
    struct ThrustSetpoint thrust_sp;

    return thrust_sp;
}

static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc)
{
    /* used in RC_DIRECT directly and as saturation in CLIMB and HOVER */
    int32_t throttle = (int32_t)rc->values[RADIO_THROTTLE];
    printf("throttle = %d", throttle);
}

