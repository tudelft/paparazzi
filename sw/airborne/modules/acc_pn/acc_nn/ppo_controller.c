// ppo_controller.c

#include <math.h>
#include "weights/dr_00/ppo_controller_weights.h"
#include "ppo_controller.h"

#define INPUT_DIM 24
#define HIDDEN1_DIM 64
#define HIDDEN2_DIM 64
#define OUTPUT_DIM 3

void dense(const float *input, const float *weight, const float *bias,
           int in_dim, int out_dim, float *output, int apply_activation)
{
    for (int i = 0; i < out_dim; i++)
    {
        output[i] = bias[i];
        for (int j = 0; j < in_dim; j++)
        {
            output[i] += weight[i * in_dim + j] * input[j];
        }
        if (apply_activation)
        {
            output[i] = tanhf(output[i]);
        }
    }
}

static inline float clip01(float x)
{
    if (x < -1.f)
        return -1.f;
    if (x > 1.f)
        return 1.f;
    return x; // unchanged when -1 ≤ x ≤ 1
}

// This is the function you'll call from Python
__attribute__((visibility("default"))) void get_action(const float *obs, float *action_out)
{
    float h1[HIDDEN1_DIM];
    float h2[HIDDEN2_DIM];
    float raw[OUTPUT_DIM];

    dense(obs, mlp_extractor_policy_net_0_weight, mlp_extractor_policy_net_0_bias,
          INPUT_DIM, HIDDEN1_DIM, h1, 1);

    dense(h1, mlp_extractor_policy_net_2_weight, mlp_extractor_policy_net_2_bias,
          HIDDEN1_DIM, HIDDEN2_DIM, h2, 1);

    dense(h2, action_net_weight, action_net_bias,
          HIDDEN2_DIM, OUTPUT_DIM, raw, 0);

    for (int i = 0; i < OUTPUT_DIM; i++)
    {
        action_out[i] = clip01(raw[i]); // squash to [-1, 1]
    }
}
