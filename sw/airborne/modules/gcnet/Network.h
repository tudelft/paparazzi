#pragma once

#ifndef NETWORK_H
#define NETWORK_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Connection.h"
#include "Neuron.h"

// Struct that defines a network of two spiking layers
typedef struct Network {
  // Normalisation arrays with min and max values for all states
  float *in_norm_min, *in_norm_max;
  // Normalisation arrays with min and max values for all states
  float output_scale_min, output_scale_max;
  // Input, encoded input, hidden and output layer sizes
  int in_size, hid_layer_size, hid_neuron_size, out_size;
  // Two input place holders: one for scalar values
  // and one for encoded currents (size in_size)
  float *input, *input_norm;
  // Two output place holders: one for network output values and 
  // one for decoded output
  float *output, *output_decoded;
  // Connection input -> hidden
  Connection *inhid;
  Connection *hid1;
  Connection *hid2;
  Connection *hidout;
  // Hidden neurons
  Neuron *layer1;
  Neuron *layer2;
  Neuron *layer3;
} Network;

// Struct that holds the configuration of a two-layer network
// To be used when loading parameters from a header file
typedef struct NetworkConf {
  // Normalisation arrays with min and max values for all states
  const float *in_norm_min, *in_norm_max;
  // Normalisation arrays with min and max values for all states
  const float output_scale_min, output_scale_max;
  // Input, encoded input, hidden and output layer sizes
  int const in_size, hid_layer_size, hid_neuron_size, out_size;
  // Connections
  ConnectionConf const *inhid;
  ConnectionConf const *hid1;
  ConnectionConf const *hid2;
  ConnectionConf const *hidout;
  // Hidden neurons
  NeuronConf const *layer1;
  NeuronConf const *layer2;
  NeuronConf const *layer3;
} NetworkConf;

// Build network: calls build functions for children
Network build_network(int const in_size, int const hid_layer_size,
                      int const hid_neuron_size, int const out_size);

// Init network: calls init functions for children
void init_network(Network *net);

// Reset network: calls reset functions for children
void reset_network(Network *net);

// Load parameters for network from header file and call load functions for
// children
void load_network_from_header(Network *net, NetworkConf const *conf);

// Free allocated memory for network and call free functions for children
void free_network(Network *net);

// Normalise input
void preprocess_input(float *input, float *input_norm, const float *in_norm_min, const float *in_norm_max, const int in_size);

//Normalise output
void postprocess_output(float *output, float *output_decoded, const float output_scale_min, const float output_scale_max, const int out_size);

// Forward network and call forward functions for children
// Encoding and decoding inside
float *forward_network(Network *net);
#endif