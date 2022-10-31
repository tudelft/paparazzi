/*
 * Paparazzi UBLox to Ivy
 *
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef ARDUINO2IVY_H
#define ARDUINO2IVY_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include <stdio.h>

#define GYRO_BUF_SIZE 200

// TODO: create structs for filters etc
struct landing_platform_dynamics {
  // raw data
  bool initialised;
  struct FloatVect3 gyro_buf[GYRO_BUF_SIZE]; // buffer gyro data in 3 axis

  // calculated data
  uint32_t itow_deck_level[2]; // calculate 2 estimations of deck level to parse to drone
  uint32_t itow_deck_top_heave[2]; // calculate 2 estimations of deck level to parse to drone
  uint32_t roll_frequency_ms;  // time in milliseonds to finish a roll (only half of the sine wave)
  double heave_amplitude_m; // the amplitde the deck is moving up and down
  // TODO: max heave or avg heave ?
  
};

#endif /* UBLOX2IVY_H */