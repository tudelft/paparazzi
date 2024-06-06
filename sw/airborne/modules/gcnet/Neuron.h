#ifndef NEURON_H
#define NEURON_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define EULER_NUMBER 2.71828182846

// Struct that defines a layer of neurons
// "Neuron" before and after {} to define both tag and typedef alias (as is most
// common)
typedef struct Neuron {
  // Neuron layer size
  int size;
  // Inputs
  float *x;
  // Cell current
  float *i;
  // Cell voltage
  float *v;
  // Cell spikes
  int *s;
  // Constants for leak of current, leak of voltage and threshold
  float *leak_i, *leak_v, *thresh;
  // Constants for resetting voltage and threshold
  float v_rest;
  // Counter for spikes
  int s_count;
} Neuron;

// Struct that holds the configuration of a layer of neurons
// To be used when loading parameters from a header file
typedef struct NeuronConf {
  // Neuron layer size
  int const size;
  // Constants for leak of current, leak of voltage and threshold
  float const *leak_i, *leak_v, *thresh;
  // Constant for resetting voltage
  float const v_rest;
} NeuronConf;

// Build neuron
Neuron build_neuron(int const size);

// Init neuron (addition/decay/reset constants, inputs, voltage, spikes,
// threshold, trace)
void init_neuron(Neuron *n);

// Sigmoid function
float sigmoidf(float n);

// Relu function
float relu(float n);

// Reset neuron (inputs, voltage, spikes, threshold, trace)
void reset_neuron(Neuron *n);

// Load parameters for neuron from header file (using the NeuronConf struct)
// a_v, a_th, a_t, d_v, d_th, d_t, v_rest, th_rest, type
void load_neuron_from_header(Neuron *n, NeuronConf const *conf);

// Free allocated memory for neuron
void free_neuron(Neuron *n);

// Forward
void forward_neuron(Neuron *n);
#endif