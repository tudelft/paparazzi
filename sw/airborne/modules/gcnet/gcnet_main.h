
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
#include "SNN_500_neurons/nn_parameters.h"

// Include child structs

// post, pre, w
ConnectionConf const conf_inhid = {NUM_NODES, NUM_STATES, weights_in};

// post, pre, w
// const float *ptr_weights_out = &weights_out[0][0];
ConnectionConf const conf_hidout = {NUM_CONTROLS, NUM_NODES, weights_out};

// post, pre, w
// const float *ptr_weights_hid_layers1_2 = &weights_hid_layers1_2[0][0];
ConnectionConf const conf_hid_1 = {NUM_NODES, NUM_NODES, weights_hid_layers1_2};

// post, pre, w
// const float *ptr_weights_hid_layers2_3 = &weights_hid_layers2_3[0][0];
ConnectionConf const conf_hid_2 = {NUM_NODES, NUM_NODES, weights_hid_layers2_3};

// size, leak_i, leak_v, thresh, v_rest
NeuronConf const conf_layer1 = {NUM_NODES, leak_i_layer1, leak_v_layer1, 
                                thresh_layer1, 0.0f};

// size, leak_i, leak_v, thresh, v_rest
NeuronConf const conf_layer2 = {NUM_NODES, leak_i_layer2, leak_v_layer2, 
                                thresh_layer2, 0.0f};

// size, leak_i, leak_v, thresh, v_rest
NeuronConf const conf_layer3 = {NUM_NODES, leak_i_layer3, leak_v_layer3, 
                                thresh_layer3, 0.0f};

const float out_scale_min = 3.000000000e+03;

const float out_scale_max = 1.200000000e+04;

// type, decoding_scale, centers, in_size, in_enc_size, hid_size, out_size,
// inhid, hid, hidout, out
NetworkConf const qz_conf = {in_norm_min, in_norm_max, out_scale_min, out_scale_max, 
                          NUM_STATES, NUM_HIDDEN_LAYERS, NUM_NODES, NUM_CONTROLS,
                          &conf_inhid, &conf_hid_1, &conf_hid_2, &conf_hidout,
                          &conf_layer1, &conf_layer2, &conf_layer3};

extern float control_nn[NUM_CONTROLS];


//
extern void gcnet_init(void);
extern void gcnet_run(void);


// Logging
extern void gnc_net_log_header(FILE *file);
extern void gnc_net_log_data(FILE *file);


#endif
