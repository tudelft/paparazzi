#include <stdio.h>
#include <stdbool.h>


struct estimators {
    float delta_t[3];
    float delta_pos[3];
    float delta_vel[3];
};



extern struct estimators estimator_pitch_fwd ;
extern struct estimators estimator_pitch_bckwd ;
extern struct estimators estimator_roll ;


void find_losses(float v_initial, float delta_angle,struct estimators *estimator);
float delta_angle_in;