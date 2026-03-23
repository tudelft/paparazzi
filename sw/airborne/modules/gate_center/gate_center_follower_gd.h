#ifndef GATE_CENTER_FOLLOWER_GD_H
#define GATE_CENTER_FOLLOWER_GD_H

#include <stdbool.h>

extern float gcf_forward_speed;
extern float gcf_detection_timeout_s;
extern float gcf_target_alt;
extern float gcf_turn_degrees;
extern float gcf_turn_rate_deg_s;
extern float gcf_post_loss_straight_s;

extern void gate_center_follower_gd_init(void);
extern void gate_center_follower_gd_periodic(void);

#endif
