#include "snn_operations.h"

Network initialise_network2(void){
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

	// type, decoding_scale, centers, in_size, in_enc_size, hid_size, out_size,
	// inhid, hid, hidout, out
	NetworkConf const conf = {in_norm_min, in_norm_max, out_scale_min, out_scale_max, 
														NUM_STATES, NUM_HIDDEN_LAYERS, NUM_NODES, NUM_CONTROLS,
														&conf_inhid, &conf_hid_1, &conf_hid_2, &conf_hidout,
														&conf_layer1, &conf_layer2, &conf_layer3};

	Network net = build_network(conf.in_size, conf.hid_layer_size, 
                              conf.hid_neuron_size, conf.out_size);

  // Init network
  init_network(&net);

	  // Load network parameters from header file
  load_network_from_header(&net, &conf);
  reset_network(&net);

  return net;
}

void run_network(Network *net, float* state_nn, float* control_nn){
  for (int i=0; i<net->in_size; i++){
    net->input[i] = state_nn[i];
  }

  // Forward network
  forward_network(net);

	for (int i=0; i<net->out_size; i++){
    control_nn[i] = net->output_decoded[i];
  }
}