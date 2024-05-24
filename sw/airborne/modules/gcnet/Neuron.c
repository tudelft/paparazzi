#include "Neuron.h"

// Build neuron
Neuron build_neuron(int const size) {
  // Neuron struct
  Neuron n;

  // Set size
  n.size = size;

  // Allocate memory for arrays: inputs, voltage, threshold, spikes, trace
  // No need for type casting
  n.x = calloc(size, sizeof(*n.x));
  n.v = calloc(size, sizeof(*n.v));
  n.thresh = calloc(size, sizeof(*n.thresh));
  n.leak_i = calloc(size, sizeof(*n.leak_i));  
  n.leak_v = calloc(size, sizeof(*n.leak_v));
  n.s = calloc(size, sizeof(*n.s));
  n.i = calloc(size, sizeof(*n.i));
  // Reset constants
  n.v_rest = 0.0f;

  return n;
}

// Init neuron (addition/decay/reset constants, inputs, voltage, spikes,
// threshold, trace)
void init_neuron(Neuron *n) {
  // Loop over neurons
  for (int i = 0; i < n->size; i++) {
    // Inputs
    n->x[i] = 0.0f;
    // Voltage
    n->v[i] = n->v_rest;
    // Spikes
    n->s[i] = 0.0f;
    // Trace
    n->i[i] = 0.0f;
    // Leak of current
    n->leak_i[i] = 0.0f;
    // Leak of voltage
    n->leak_v[i] = 0.0f;
    // Threshold
    n->thresh[i] = 1.0f;
  }
  // Spike counter
  n->s_count = 0;
}

// Reset neuron (inputs, voltage, spikes, threshold, trace)
void reset_neuron(Neuron *n) {
  // Loop over neurons
  for (int i = 0; i < n->size; i++) {
    // Inputs
    n->x[i] = 0.0f;
    // Voltage
    n->v[i] = n->v_rest;
    // Spikes
    n->s[i] = 0.0f;
    // Trace
    n->i[i] = 0.0f;
  }
  // Spike counter
  n->s_count = 0;
}

// Load parameters for neuron from header file (using the NeuronConf struct)
void load_neuron_from_header(Neuron *n, NeuronConf const *conf) {
  // Check shape
  if (n->size != conf->size) {
    printf("Neuron has a different shape than specified in the NeuronConf!\n");
    exit(1);
  }
  // Loop over neurons
  // TODO: could also be done by just exchanging pointers to arrays?
  for (int i = 0; i < n->size; i++) {
    // Constants for addition of voltage, threshold and trace
    n->thresh[i] = conf->thresh[i];
    n->leak_i[i] = conf->leak_i[i];
    n->leak_v[i] = conf->leak_v[i];
  }
  // Constant for resetting voltage
  n->v_rest = conf->v_rest;
}

// Free allocated memory for neuron
void free_neuron(Neuron *n) {
  // calloc() was used for voltage/decay/reset constants, inputs, voltage,
  // threshold, spike and trace arrays
  free(n->x);
  free(n->v);
  free(n->i);
  free(n->thresh);
  free(n->s);
  free(n->leak_i);
  free(n->leak_v);
}

float sigmoidf(float n) {
    return (1 / (1 + powf(EULER_NUMBER, -n)));
}

float relu(float n){
    return (n < 0.0f ? 0.0f : n);
}

// Check spikes
static void spiking(Neuron *n) {
  // Loop over neurons
  for (int i = 0; i < n->size; i++) {
    // If above/equal to threshold: set spike, else don't
    float thresh = relu(n->thresh[i]);
    n->s[i] = n->v[i] >= thresh ? 1 : 0;
  }
}

// Do refraction
static void refrac(Neuron *n) {
  // Loop over neurons
  for (int i = 0; i < n->size; i++) {
    // If spike, then refraction
    // We don't have a refractory period, so no need to take care of that
    // TODO: how dangerous is checking for equality with floats?
    n->v[i] = n->s[i] == 1 ? n->v_rest : n->v[i];
    // Also increment spike counter!
    n->s_count += n->s[i] == 1 ? 1 : 0;
  }
}

// Update voltage
static void update_voltage(Neuron *n) {
  // Loop over neurons
  for (int i = 0; i < n->size; i++) {
    // Decay difference with resting potential, then increase for incoming
    // spikes
    float leak_i = sigmoidf(n->leak_i[i]);
    float leak_v = sigmoidf(n->leak_v[i]);
    n->i[i] = (n->i[i] * leak_i) + n->x[i];
    n->v[i] = ((n->v[i] - n->v_rest) * leak_v) + n->i[i];
  }
}

// Update/reset inputs (otherwise accumulation over time)
static void update_inputs(Neuron *n) {
  // Loop over neurons
  for (int i = 0; i < n->size; i++) {
    // Set to zero
    n->x[i] = 0.0f;
  }
}

// Forward: encompasses voltage/trace/threshold updates, spiking and refraction
// TODO: use above functions or write new loop which does all in one (and use
//  above for inspiration)
void forward_neuron(Neuron *n) {
  // Update voltage
  // Pass n because this is already a pointer
  update_voltage(n);
  // Get spikes
  spiking(n);
  // Refraction
  refrac(n);
  // Reset inputs (otherwise we get accumulation over time)
  update_inputs(n);
  // No return, spikes are a member of Neuron struct and can be used for next
  // layer
}
