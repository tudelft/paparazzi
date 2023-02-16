/*
File: nn_operations.h
Goal: Include header of different functions used to compute neural network operations
*/

#ifndef NN_OPERATIONS_H
#define NN_OPERATIONS_H

#include <stdio.h>
#include <math.h> // use softplus and sigmoid operations 

#include "nn_parameters.h"

extern void preprocess_input(float *input, float *input_norm, const float *input_norm_mean, const float *input_norm_std);

extern void postprocess_output(float *output, const float output_scale_min, const float output_scale_max);

extern void sigmoid_activation(float* vec, int numNodes);

extern void softplus_activation(float* vec, int numNodes);

extern void relu_activation(float* vec, int numNodes);

extern void copy_vec(float* vec_1, float* vec_2, int len_vec_1);

extern void fully_connected_layer(float* in, float* out, const float* weight_ptr, const float* bias_ptr, int dim_curr_layer, int dim_next_layer);

extern void nn_predict(float *nn_input, float *nn_output,
                       const float weights_input[NUM_NODES][NUM_STATES],
                       const float bias_input[NUM_NODES],
                       const float weights_hidden[NUM_HIDDEN_LAYERS-1][NUM_NODES][NUM_NODES],
                       const float bias_hidden[NUM_HIDDEN_LAYERS-1][NUM_NODES],
                       const float weights_output[NUM_CONTROLS][NUM_NODES],
                       const float bias_output[NUM_CONTROLS]);

extern void nn_control(float* state, float* control);

#endif