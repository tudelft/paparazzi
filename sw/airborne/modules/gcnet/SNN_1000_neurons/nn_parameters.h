#ifndef NN_PARAMETERS_H
#define NN_PARAMETERS_H

#define NUM_STATES 19
#define NUM_CONTROLS 4
#define NUM_LAYERS 5
#define NUM_HIDDEN_LAYERS 3
#define NUM_NODES 1000


// NN network parameters -- define variables

extern const float in_norm_min[];

extern const float in_norm_max[];

extern const float leak_i_layer1[];

extern const float leak_i_layer2[];

extern const float leak_i_layer3[];

extern const float leak_v_layer1[];

extern const float leak_v_layer2[];

extern const float leak_v_layer3[];

extern const float thresh_layer1[];

extern const float thresh_layer2[];

extern const float thresh_layer3[];

extern const float weights_in[];

extern const float weights_out[];

extern const float weights_hid_layers1_2[];

extern const float weights_hid_layers2_3[];
#endif
