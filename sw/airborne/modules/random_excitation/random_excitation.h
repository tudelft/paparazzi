/*
 * Copyright (C) Ewoud Smeur and Dennis van Wijngaarden <e.j.j.smeur@tudelft.nl>
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

/** @file "modules/random_excitation/random_excitation.h"
 * @author Ewoud Smeur and Dennis van Wijngaarden <e.j.j.smeur@tudelft.nl>
 * This module will provide random excitation to each axis in order to make sure also the null space is excited for parameter estimation (online or offline).
 */

#ifndef RANDOM_EXCITATION_H
#define RANDOM_EXCITATION_H

#include "std.h"
#include "math/pprz_random.h"

extern float rand_excitation_cutoff_frequency;
extern uint32_t amplitude;
extern uint32_t rand_excitation_amp;

extern uint8_t excitation_group1[]; // Group1 of actuators for which the prefered excitation can differ from the other actuators
extern uint32_t excitation_group1_amp; // How much higher is the preffered state of the excitation group1

extern uint8_t excitation_group2[]; // Group2 of actuators for which the prefered excitation can differ from the other actuators
extern uint32_t excitation_group2_amp; // How much higher is the preffered state of the excitation group2

extern void random_excitation_init(void);
extern void random_excitation_periodic(void);

#endif  // RANDOM_EXCITATION_H
