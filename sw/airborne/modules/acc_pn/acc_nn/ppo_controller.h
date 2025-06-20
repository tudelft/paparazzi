// ppo_controller.h
#ifndef PPO_CONTROLLER_H
#define PPO_CONTROLLER_H

// Declare your public API here
void get_action(const float *obs, float *action_out);
void dense(const float *input, const float *weight, const float *bias,
           int in_dim, int out_dim, float *output, int apply_activation);

#endif // PPO_CONTROLLER_H
