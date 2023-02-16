#ifndef NN_PARAMETERS_H
#define NN_PARAMETERS_H

#define NUM_STATES 15
#define NUM_CONTROLS 4
#define NUM_LAYERS 3
#define NUM_HIDDEN_LAYERS 1
#define NUM_NODES 120


// NN network parameters -- define variables
extern const float weights_in[NUM_NODES][NUM_STATES];

extern const float bias_in[NUM_NODES];

extern const float weights_out[NUM_CONTROLS][NUM_NODES];

extern const float bias_out[NUM_CONTROLS];

extern const float in_norm_mean[NUM_STATES];

extern const float in_norm_std[NUM_STATES];

extern const float out_scale_min;

extern const float out_scale_max;

extern const float weights_hid[NUM_HIDDEN_LAYERS-1][NUM_NODES][NUM_NODES];

extern const float bias_hid[NUM_HIDDEN_LAYERS-1][NUM_NODES];

#endif
