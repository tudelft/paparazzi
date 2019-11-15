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
 * @file "modules/spiking_net/spiking_landing.h"
 * @author Huizerd
 * Spiking neural networks for optical flow landing.
 */

#pragma once

// Module functions
extern void snn_init();
extern void snn_print_debug();
extern void snn_filter_control();

// Thrust settings
// TODO: is this necessary? Is this because of dl_settings in
// spiking_landing.xml?
extern float thrust_effect;
extern float thrust_p_gain;
extern float thrust_i_gain;
