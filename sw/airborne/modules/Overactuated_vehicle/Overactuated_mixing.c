/*
 * Copyright (C) 2021 A. Mancinelli
 *
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
/**
 * @file "modules/Overactuated_vehicle/Overactuated_vehicle.c"
 * @author Alessandro Mancinelli
 * Control laws for Overactuated Vehicle
 */

#include "Overactuated_mixing.h"
#include "subsystems/radio_control.h"
struct overactuated_mixing_t overactuated_mixing;

/* Coeficients per motor  --> MANUAL MODE *//*

static const float roll_coef[SW_NB]   = SW_MIXING_ROLL_COEF;
static const float pitch_coef[SW_NB]  = SW_MIXING_PITCH_COEF;
static const float coll_coef[SW_NB]   = SW_MIXING_COLL_COEF;

*/


/**
 * Initialize the motor mixing and calculate the trim values
 */
void overactuated_mixing_init() {
    uint8_t i;

    // Go trough all the motors and calculate the trim value and set the initial command
    for (i = 0; i < N_ACT; i++) {
        overactuated_mixing.commands[i] = 0;
        if (i % 2 != 0)   //Odd value --> (elevation angle)
        {
            overactuated_mixing.commands[i] = radio_control.values[RADIO_ROLL];
        } else           //Even value --> (azimuth angle)
        {
            overactuated_mixing.commands[i] = radio_control.values[RADIO_PITCH];
        }
    }
}

/*
 * Run the swashplate mixing
 * This depends on the ROLL and PITCH command
 * It also depends on the throttle_curve.collective
 */
void overactuated_mixing_run()
{
    uint8_t i;

    // Go trough all the motors and calculate the trim value and set the initial command
    for (i = 0; i < N_ACT; i++) {
        overactuated_mixing.commands[i] = 0;
        if (i % 2 != 0)   //Odd value --> (elevation angle)
        {
            overactuated_mixing.commands[i] = radio_control.values[RADIO_ROLL];
        } else           //Even value --> (azimuth angle)
        {
            overactuated_mixing.commands[i] = radio_control.values[RADIO_PITCH];
        }
        BoundAbs(overactuated_mixing.commands[i], MAX_PPRZ);
    }
}
