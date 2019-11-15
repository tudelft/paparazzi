/*
 * Copyright (C) Huizerd
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
 * @file "modules/spiking_net/spiking_net.c"
 * @author Huizerd
 * Spiking neural networks for control.
 */

// Header for this file
#include "modules/spiking_net/spiking_net.h"

// tinysnn headers
#include "tinysnn/Network.h"

// Paparazzi headers
// TODO: do we need all this? And in what order?
#include "navigation.h"
#include "state.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/gps.h"
#include "subsystems/gps/gps_datalink.h"

#include "generated/flight_plan.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_int.h"

#include "generated/airframe.h"
#include "subsystems/abi.h"

#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_indi.h"
#include "guidance/guidance_v.h"

#include "subsystems/abi.h"

// C standard library headers
#include <stdbool.h>

// Constants
// TODO: is this the way to do this? Seems a bit overdreven, and inconsistent
//  between the different constants
// Thrust settings
// From G to thrust percentage
float thrust_effect = 0.05f;

// Closed-loop thrust control, else linear transform
#define ACTIVE_CONTROL true

// Gains for closed-loop control
#ifndef THRUST_P_GAIN
#define THRUST_P_GAIN 0.7
#endif
#ifndef THRUST_I_GAIN
#define THRUST_I_GAIN 0.3
#endif
float thrust_p_gain = THRUST_P_GAIN;
float thrust_i_gain = THRUST_I_GAIN;

// Optical flow settings
#ifndef OF_FILTER_CUTOFF
#define OF_FILTER_CUTOFF 1.5f
#endif

// Low-pass filters for acceleration and thrust
static Butterworth2LowPass accel_filt;
static Butterworth2LowPass thrust_filt;

void snn_init() {}
void snn_periodic() {}
void snn_control() {}
