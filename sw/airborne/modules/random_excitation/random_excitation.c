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

/** @file "modules/random_excitation/random_excitation.c"
 * @author Ewoud Smeur and Dennis van Wijngaarden <e.j.j.smeur@tudelft.nl>
 * This module will provide random excitation to each axis in order to make sure also the null space is excited for parameter estimation (online or offline).
 */

#include "modules/random_excitation/random_excitation.h"

void random_excitation_init(void)
{
  // your init code here
}

void random_excitation_periodic(void)
{
  // your periodic code here.
  // freq = 1.0 Hz
  
}


