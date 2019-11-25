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
 * @file "modules/spiking_spiking_landing/spiking_landing.h"
 * @author Huizerd
 * Spiking neural networks for optical flow landing.
 */

#pragma once

// tinysnn headers
#include "Network.h"

// C standard library headers
#include <stdbool.h>
#include <stdint.h>

// Without OptiTrack set to: GUIDANCE_H_MODE_ATTITUDE
// With OptiTrack set to: GUIDANCE_H_MODE_HOVER / NAV
//#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_NAV

// Own guidance_v
//#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own vertical loops
//extern void guidance_v_module_init();
//extern void guidance_v_module_enter();
//extern void guidance_v_module_run(bool in_flight);
extern void spiking_landing_init();
extern void spiking_landing_event();

// Spiking network
extern Network net;

// Struct to hold settings
struct SpikingLandingSettings {
  float agl;                    ///< agl = height from sonar (only used when using "fake" divergence)
  float agl_lp;                 ///< low-pass version of agl
  float lp_const;               ///< low-pass filter constant
  float vel;                    ///< vertical velocity as determined with sonar (only used when using "fake" divergence)
  float divergence_setpoint;    ///< setpoint for constant divergence approach
  float pgain;                  ///< P-gain for constant divergence control (from divergence error to thrust)
  float igain;                  ///< I-gain for constant divergence control
  float dgain;                  ///< D-gain for constant divergence control
  float divergence;             ///< Divergence estimate
  float previous_err;           ///< Previous divergence tracking error
  float sum_err;                ///< integration of the error for I-gain
  float d_err;                  ///< difference of error for the D-gain
  float nominal_thrust;         ///< nominal thrust around which the PID-control operates
  uint32_t VISION_METHOD;       ///< whether to use vision (1) or Optitrack / sonar (0)
  uint32_t CONTROL_METHOD;      ///< type of divergence control: 0 = fixed gain, 1 = adaptive gain
  float cov_set_point;          ///< for adaptive gain control, setpoint of the covariance (oscillations)
  float cov_limit;              ///< for fixed gain control, what is the cov limit triggering the landing
  float pgain_adaptive;         ///< P-gain for adaptive gain control
  float igain_adaptive;         ///< I-gain for adaptive gain control
  float dgain_adaptive;         ///< D-gain for adaptive gain control
  uint32_t COV_METHOD;          ///< method to calculate the covariance: between thrust and div (0) or div and div past (1)
  uint32_t delay_steps;         ///< number of delay steps for div past
  uint32_t window_size;         ///< number of time steps in "window" used for getting the covariance
  float reduction_factor_elc;   ///< reduction factor - after oscillation, how much to reduce the gain...
  float lp_cov_div_factor;      ///< low-pass factor for the covariance of divergence in order to trigger the second landing phase in the exponential strategy.
  float t_transition;           ///< how many seconds the drone has to be oscillating in order to transition to the hover phase with reduced gain
  float p_land_threshold;       ///< if during the exponential landing the gain reaches this value, the final landing procedure is triggered
  bool elc_oscillate;           ///< whether or not to oscillate at beginning of elc to find optimum gain
  float thrust_effect;          ///< thrust effectiveness
  float thrust_p_gain;          ///< P-gain for active thrust control
  float thrust_i_gain;          ///< I-gain for active thrust control
};

extern struct SpikingLandingSettings sl_settings;
