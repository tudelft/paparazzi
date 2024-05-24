#ifndef NN_PARAMETERS_H
#define NN_PARAMETERS_H

#define NUM_STATES 19
#define NUM_CONTROLS 4
#define NUM_LAYERS 5
#define NUM_HIDDEN_LAYERS 3
#define NUM_NODES 1000


// NN network parameters -- define variables

extern const float in_norm_min[NUM_STATES];

extern const float in_norm_max[NUM_STATES];

extern const float out_scale_min;

extern const float out_scale_max;

extern float leak_i_layer1[NUM_NODES];

extern float leak_i_layer2[NUM_NODES];

extern float leak_i_layer3[NUM_NODES];

extern float leak_v_layer1[NUM_NODES];

extern float leak_v_layer2[NUM_NODES];

extern float leak_v_layer3[NUM_NODES];

extern float thresh_layer1[NUM_NODES];

extern float thresh_layer2[NUM_NODES];

extern float thresh_layer3[NUM_NODES];

extern float weights_in[NUM_NODES*NUM_STATES];

extern float weights_out[NUM_CONTROLS*NUM_NODES];

extern float weights_hid_layers1_2[NUM_NODES*NUM_NODES];

extern float weights_hid_layers2_3[NUM_NODES*NUM_NODES];
#endif
