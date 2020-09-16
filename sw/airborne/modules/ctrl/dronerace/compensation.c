#include "compensation.h"
#include "bangbang.h"



float estimator_time[3] = {0.285600580827510, -0.022035744273907, -0.012053303063961};
float estimator_pos[3] = {0.164209236667037, 0.120797076901804, 0.162129652187799}; 
float estimator_vel[3] = {-0.149450000852137, 0.613672580954269, -0.099748395433426};


// f=e[0]+e[1]*delta_angle+e[2]*Vi
// pitch forward: delta_angle>0, Vi>0, delta_t>0, delta_pos >0  ,delta_v both
// pitch backward: delta_angle<0, Vi<0, delta_t>0, delta_pos <0 ,delta_v both 
struct estimators estimator_pitch_fwd = {
    {0.285600580827510, -0.022035744273907, -0.012053303063961}, //delta_t 
    {0.164209236667037, 0.120797076901804, 0.162129652187799},  //delta_pos
    {-0.149450000852137, 0.613672580954269, -0.099748395433426}, //delta_v
};


struct estimators estimator_pitch_bckwd = { // the dynamics are different in backward flight 
    {0.285600580827510, 0.022035744273907, 0.012053303063961},
    {-0.164209236667037, 0.120797076901804, 0.162129652187799},
    {0.149450000852137, 0.613672580954269, -0.099748395433426},
};

struct estimators estimator_roll = {
    {0.269378649139438, 0.034156640976302, 0.008765690584792},
    {0.040579756282273, 0.186559584777099, 0.226100562632777},
    {0.139837868851582, -0.436404615562183, -0.166030706027535,},
};




void find_losses(float v_initial, float delta_angle, struct estimators *estimator){
    delta_angle_in=delta_angle; // for logging. 
    controllerstate.delta_t = estimator->delta_t[0] + estimator->delta_t[1]*delta_angle + estimator->delta_t[2]*v_initial;
    controllerstate.delta_y = estimator->delta_pos[0] + estimator->delta_pos[1]*delta_angle + estimator->delta_pos[2]*v_initial;
    controllerstate.delta_v = estimator->delta_vel[0] + estimator->delta_vel[1]*delta_angle + estimator->delta_vel[2]*v_initial;
}