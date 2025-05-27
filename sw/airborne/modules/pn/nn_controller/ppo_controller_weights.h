#ifndef PPO_CONTROLLER_WEIGHTS_H
#define PPO_CONTROLLER_WEIGHTS_H

extern float log_std[3];

extern float mlp_extractor_policy_net_0_weight[768];

extern float mlp_extractor_policy_net_0_bias[64];

extern float mlp_extractor_policy_net_2_weight[4096];

extern float mlp_extractor_policy_net_2_bias[64];

extern float mlp_extractor_value_net_0_weight[768];

extern float mlp_extractor_value_net_0_bias[64];

extern float mlp_extractor_value_net_2_weight[4096];

extern float mlp_extractor_value_net_2_bias[64];

extern float action_net_weight[192];

extern float action_net_bias[3];

extern float value_net_weight[64];

extern float value_net_bias[1];

#endif // PPO_CONTROLLER_WEIGHTS_H