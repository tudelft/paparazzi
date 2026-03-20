#ifndef MAV_PLANT_AVOIDER_H
#define MAV_PLANT_AVOIDER_H

#include "std.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float pa_straight_bias;
extern float pa_forward_speed;
extern float pa_turn_speed;

extern float pa_load_left;
extern float pa_load_straight;
extern float pa_load_right;
extern int8_t pa_last_direction;

extern float pa_weight_left;
extern float pa_weight_straight;
extern float pa_weight_right;

void plant_avoider_init(void);
void plant_avoider_periodic(void);

#ifdef __cplusplus
}
#endif

#endif
