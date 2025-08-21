#include <math.h>
#include "ppo_controller.h"
#include "weights/testing/ppo_controller_weights.h"

#define INPUT_DIM 8
#define HIDDEN1_DIM 64
#define HIDDEN2_DIM 64
#define HIDDEN3_DIM 64
#define OUTPUT_DIM 3

void dense(const float *input, const float *weight, const float *bias,
           int in_dim, int out_dim, float *output, int apply_activation)
{
    for (int i = 0; i < out_dim; i++)
    {
        float acc = bias[i];
        const float *wrow = &weight[i * in_dim];
        for (int j = 0; j < in_dim; j++)
        {
            acc += wrow[j] * input[j];
        }
        output[i] = apply_activation ? tanhf(acc) : acc;
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
    float h3[HIDDEN3_DIM];
    float raw[OUTPUT_DIM];

    // Layer 1: INPUT_DIM -> 64 (tanh)
    dense(obs,
          mlp_extractor_policy_net_0_weight,
          mlp_extractor_policy_net_0_bias,
          INPUT_DIM, HIDDEN1_DIM, h1, 1);

    // Layer 2: 64 -> 64 (tanh)
    dense(h1,
          mlp_extractor_policy_net_2_weight,
          mlp_extractor_policy_net_2_bias,
          HIDDEN1_DIM, HIDDEN2_DIM, h2, 1);

    // NEW Layer 3: 64 -> 64 (tanh)
    dense(h2,
          mlp_extractor_policy_net_4_weight, // add these to ppo_weights.h
          mlp_extractor_policy_net_4_bias,
          HIDDEN2_DIM, HIDDEN3_DIM, h3, 1);

    // Output layer: 64 -> 4 (linear)
    dense(h3,
          action_net_weight,
          action_net_bias,
          HIDDEN3_DIM, OUTPUT_DIM, raw, 0);

    // Squash to [-1, 1]
    for (int i = 0; i < OUTPUT_DIM; i++)
    {
        action_out[i] = clip01(raw[i]);
    }
}