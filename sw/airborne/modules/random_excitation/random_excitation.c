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

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

#include "filters/low_pass_filter.h"
#include "math/pprz_random.h"

#include <stdio.h>

#ifndef RANDOM_EXCITATION_AMPLITUDE
#define RANDOM_EXCITATION_AMPLITUDE 0 // Set amplitude to 0 if not defined 
#endif

#ifndef RANDOM_EXCITATION_CUTOFF_FREQUENCY
#define RANDOM_EXCITATION_CUTOFF_FREQUENCY 5.0 // HZ
#endif

#ifdef RANDOM_EXCITATION_GROUP1
  uint8_t excitation_group1[] = RANDOM_EXCITATION_GROUP1;
#else
  uint8_t excitation_group1[] = {};
#endif

#ifdef RANDOM_EXCITATION_GROUP1_AMP
  uint32_t excitation_group1_amp = RANDOM_EXCITATION_GROUP1_AMP;
#else
  uint32_t excitation_group1_amp = 0;
#endif

#ifdef RANDOM_EXCITATION_GROUP2
  uint8_t excitation_group2[] = RANDOM_EXCITATION_GROUP2;
#else
  uint8_t excitation_group2[] = {};
#endif

#ifdef RANDOM_EXCITATION_GROUP2_AMP
  uint32_t excitation_group2_amp = RANDOM_EXCITATION_GROUP1_AMP;
#else
  uint32_t excitation_group2_amp = 0;
#endif

float rand_excitation_cutoff_frequency = RANDOM_EXCITATION_CUTOFF_FREQUENCY;
uint32_t amplitude_bound = 1000; // bounding value of amplitude for Random excitation
uint32_t rand_excitation_amp = RANDOM_EXCITATION_AMPLITUDE;

// Define filters
Butterworth2LowPass random_control_lowpass_filters[INDI_NUM_ACT];

void random_excitation_init(void)
{
  // Init 2nd order butterworth filter
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * RANDOM_EXCITATION_CUTOFF_FREQUENCY);
  float sample_time = 1.0; // Pertiodic functions runs every 1.0 second
  // Filtering of the random signals
  uint8_t i;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&random_control_lowpass_filters[i], tau, sample_time, 0.0);
  }

  // Bound apmplitude
  rand_excitation_amp = fmin(rand_excitation_amp, amplitude_bound);

  // Init randomizer
  init_random();
}

void random_excitation_periodic(void)
{
  // freq = 1.0 Hz
  // Generate random signals for each actuator
  uint8_t i;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    double rand_number = rand_uniform();
    // Propagate filter
    update_butterworth_2_low_pass(&random_control_lowpass_filters[i], rand_number);
    // Update prefered actuator state
    act_pref[i] = rand_number * rand_excitation_amp;
  }

  // Loop through excitation groups to add amplitude to prefered state
  for (i = 0; i < sizeof(excitation_group1); i++) {
    // Check if motor number in range
    if (excitation_group1[i] < INDI_NUM_ACT) {
      act_pref[excitation_group1[i]] += excitation_group1_amp;
    }
  }

  for (i = 0; i < sizeof(excitation_group2); i++) {
    // Check if motor number in range
    if (excitation_group2[i] < INDI_NUM_ACT) {
      act_pref[excitation_group2[i]] += excitation_group2_amp;
    }
  }
}


