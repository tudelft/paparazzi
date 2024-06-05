
/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/gcnet/gcnet_main.h"
 * @brief example empty controller
 *
 * Implements an example simple rate controller in a module.
 */

#ifndef GCNET_MAIN_H
#define GCNET_MAIN_H

// paparazzi library with standard definitions
#include "std.h" 

// paparazzi-based libraries
#include "math/pprz_algebra_float.h"
#include "state.h"

// neural network library
// #include "modules/gcnet/nn_operations.h"

#include "Network.h"
#include "SNN_250_neurons/nn_parameters.h"

// Include child structs

// post, pre, w
extern ConnectionConf const conf_inhid;

// post, pre, w
// const float *ptr_weights_out = &weights_out[0][0];
extern ConnectionConf const conf_hidout;

// post, pre, w
// const float *ptr_weights_hid_layers1_2 = &weights_hid_layers1_2[0][0];
extern ConnectionConf const conf_hid_1;

// post, pre, w
// const float *ptr_weights_hid_layers2_3 = &weights_hid_layers2_3[0][0];
extern ConnectionConf const conf_hid_2;

// size, leak_i, leak_v, thresh, v_rest
extern NeuronConf const conf_layer1;

// size, leak_i, leak_v, thresh, v_rest
extern NeuronConf const conf_layer2;

// size, leak_i, leak_v, thresh, v_rest
extern NeuronConf const conf_layer3;

// type, decoding_scale, centers, in_size, in_enc_size, hid_size, out_size,
// inhid, hid, hidout, out
extern NetworkConf const qz_conf;

extern float control_nn[NUM_CONTROLS];


//
extern void gcnet_init(void);
extern void gcnet_run(void);


// Logging
extern void gnc_net_log_header(FILE *file);
extern void gnc_net_log_data(FILE *file);


#endif
