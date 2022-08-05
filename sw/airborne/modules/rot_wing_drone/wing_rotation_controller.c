/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rot_wing_drone/wing_rotation_controller.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module to control wing rotation servo command based on prefered angle setpoint
 */

#include "modules/rot_wing_drone/wing_rotation_controller.h"

#include "mcu_periph/adc.h"

/*
#ifndef WING_ROTATION_SERVO_IDX
#error "No WING_ROTATION_SERVO_IDX defined"
#endif
*/

#ifndef ADC_CHANNEL_WING_ROTATION_POSITION
#define ADC_CHANNEL_WING_ROTATION_POSITION ADC_6
#endif

#ifndef ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES
#define ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES 16
#endif

/*
#ifndef WING_ROTATION_POSITION_ADC_0
#error "No WING_ROTATION_POSITION_ADC_0 defined"
#endif

#ifndef WING_ROTATION_POSITION_ADC_90
#error "No WING_ROTATION_POSITION_ADC_90 defined"
#endif
*/

static struct adc_buf buf_wing_rot_pos;

void wing_rotation_init(void)
{
  // your init code here

  adc_buf_channel(ADC_CHANNEL_WING_ROTATION_POSITION, &buf_wing_rot_pos, ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES);
}

void wing_rotation_periodic(void)
{
  // your periodic code here.
  // freq = 1.0 Hz
}

void wing_rotation_event(void)
{
  // your event code here
}


