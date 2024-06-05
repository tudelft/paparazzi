#include "Network.h"

// Build network: calls build functions for children
Network build_network(int const in_size, int const hid_layer_size,
                      int const hid_neuron_size, int const out_size) {
  // Network struct
  Network net;

  net.in_size = in_size;
  net.hid_layer_size = hid_layer_size;
  net.hid_neuron_size = hid_neuron_size;
  net.out_size = out_size;

  // Allocate memory for input placeholders, place cell centers and underlying
  // neurons and connections
  net.input = calloc(in_size, sizeof(*net.input));
  net.input_norm = calloc(in_size, sizeof(*net.input_norm));
  net.in_norm_min = calloc(in_size, sizeof(*net.in_norm_min));
  net.in_norm_max = calloc(in_size, sizeof(*net.in_norm_max));
  net.output = calloc(out_size, sizeof(*net.output));
  net.output_decoded = calloc(out_size, sizeof(*net.output_decoded));
  // TODO: is this the best way to do this? Or let network struct consist of
  //  actual structs instead of pointers to structs?
  net.inhid = malloc(sizeof(*net.inhid));
  net.hid1 = malloc(sizeof(*net.hid1));
  net.hid2 = malloc(sizeof(*net.hid2));
  net.hidout = malloc(sizeof(*net.hidout));
  net.layer1 = malloc(sizeof(*net.layer1));
  net.layer2 = malloc(sizeof(*net.layer2));
  net.layer3 = malloc(sizeof(*net.layer3));

  // Call build functions for underlying neurons and connections
  *net.inhid = build_connection(hid_neuron_size, in_size);
  *net.hid1 = build_connection(hid_neuron_size, hid_neuron_size);
  *net.hid2 = build_connection(hid_neuron_size, hid_neuron_size);
  *net.hidout = build_connection(out_size, hid_neuron_size);
  *net.layer1 = build_neuron(hid_neuron_size);
  *net.layer2 = build_neuron(hid_neuron_size);
  *net.layer3 = build_neuron(hid_neuron_size);

  return net;
}

// Init network: calls init functions for children
void init_network(Network *net) {
  // Loop over input placeholders
  for (int i = 0; i < net->in_size; i++) {
    net->input[i] = 0.0f;
    net->input_norm[i] = 0.0f;
    net->in_norm_min[i] = 0.0f;
    net->in_norm_max[i] = 0.0f;
  }

  for (int i = 0; i < net->out_size; i++) {
    net->output[i] = 0.5f;
    net->output_decoded[i] = 0.5f;
  }
  // Call init functions for children
  init_connection(net->inhid);
  init_connection(net->hid1);
  init_connection(net->hid2);
  init_connection(net->hidout);
  init_neuron(net->layer1);
  init_neuron(net->layer2);
  init_neuron(net->layer3);
}

// Reset network: calls reset functions for children
void reset_network(Network *net) {
  reset_connection(net->inhid);
  reset_connection(net->hid1);
  reset_connection(net->hid2);
  reset_connection(net->hidout);
  reset_neuron(net->layer1);
  reset_neuron(net->layer2);
  reset_neuron(net->layer3);
}

// Load parameters for network from header file and call load functions for
// children
void load_network_from_header(Network *net, NetworkConf const *conf) {
  // Check shapes
  if ((net->in_size != conf->in_size) ||
      (net->hid_layer_size != conf->hid_layer_size) ||
      (net->hid_neuron_size != conf->hid_neuron_size) || (net->out_size != conf->out_size)) {
    printf(
        "Network has a different shape than specified in the NetworkConf!\n");
    exit(1);
  }
  // Decoding
  net->output_scale_min = conf->output_scale_min;
  net->output_scale_max = conf->output_scale_max;
  // Encoding
  for (int i = 0; i < net->in_size; i++) {
    net->in_norm_min[i] = conf->in_norm_min[i];
    net->in_norm_max[i] = conf->in_norm_max[i];
  }
  // Connection input -> hidden
  load_connection_from_header(net->inhid, conf->inhid);
  // Hidden neuron
  load_connection_from_header(net->hid1, conf->hid1);
  // Hidden neuron
  load_connection_from_header(net->hid2, conf->hid2);
  // Connection hidden -> output
  load_connection_from_header(net->hidout, conf->hidout);
  // Layer 1
  load_neuron_from_header(net->layer1, conf->layer1);
  // Layer 2
  load_neuron_from_header(net->layer2, conf->layer2);
  // Layer 3
  load_neuron_from_header(net->layer3, conf->layer3);
}

// Free allocated memory for network and call free functions for children
void free_network(Network *net) {
  // Call free functions for children
  // Freeing in a bottom-up manner
  // TODO: or should we call this before freeing the network struct members?
  free_connection(net->inhid);
  free_connection(net->hid1);
  free_connection(net->hid2);
  free_connection(net->hidout);
  free_neuron(net->layer1);
  free_neuron(net->layer2);
  free_neuron(net->layer3);
  // calloc() was used for input placeholders and underlying neurons and
  // connections
  free(net->input);
  free(net->input_norm);
  free(net->in_norm_min);
  free(net->in_norm_max);
  free(net->inhid);
  free(net->hid1);
  free(net->hid2);
  free(net->hidout);
  free(net->layer1);
  free(net->layer2);
  free(net->layer3);
}

void preprocess_input(float *input, float *input_norm, const float *in_norm_min, const float *in_norm_max, const int in_size)
{   
    int idx;

    for(idx = 0; idx < in_size - 3; idx++)
    {
        input_norm[idx] = (input[idx] - in_norm_min[idx])/(in_norm_max[idx] - in_norm_min[idx]);
    }
    input_norm[in_size - 3] = 0.0f;
    input_norm[in_size - 2] = 0.0f;
    input_norm[in_size - 1] = 0.0f;
}

/*
Function: Post-process outputs (unscaling layers)
*/
void postprocess_output(float *output, float *output_decoded, const float output_scale_min, const float output_scale_max, const int out_size)
{
    int idx;
    
    for(idx = 0; idx < out_size; idx++)
    {
        // unscale from [0,1] to the interval [control_min, control_max]
        output_decoded[idx] = output_scale_min + output[idx] * (output_scale_max - output_scale_min);
    }
}

// Forward network and call forward functions for children
// Encoding and decoding inside
// TODO: but we still need to check the size of the array we put in net->in
void forward_network(Network *net) {
  preprocess_input(net->input, net->input_norm, net->in_norm_min, net->in_norm_max, net->in_size);

  // Call forward functions for children
  forward_connection_float(net->inhid, net->layer1->x, net->input_norm);
  forward_neuron(net->layer1);
  forward_connection_int(net->hid1, net->layer2->x, net->layer1->s);
  forward_neuron(net->layer2);
  forward_connection_int(net->hid2, net->layer3->x, net->layer2->s);
  forward_neuron(net->layer3);
  forward_connection_int(net->hidout, net->output, net->layer3->s);
  // Decode output neuron traces to scalar value
  for (int i = 0; i < net->out_size; i++) {
    net->output[i] = 0.6f;
  }
  postprocess_output(net->output, net->output_decoded, net->output_scale_min, net->output_scale_max, net->out_size);

  // return net->output_decoded;
}
