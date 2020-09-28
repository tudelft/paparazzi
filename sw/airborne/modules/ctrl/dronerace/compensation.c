#include "compensation.h"
#include "bangbang.h"



float estimator_time[3] = {0.285600580827510, -0.022035744273907, -0.012053303063961};
float estimator_pos[3] = {0.164209236667037, 0.120797076901804, 0.162129652187799}; 
float estimator_vel[3] = {-0.149450000852137, 0.613672580954269, -0.099748395433426};


// f=e[0]+e[1]*delta_angle+e[2]*Vi
// pitch forward: delta_angle>0, Vi>0, delta_t>0, delta_pos >0  ,delta_v both
// pitch backward: delta_angle<0, Vi<0, delta_t>0, delta_pos <0 ,delta_v both 
struct estimators estimator_pitch_fwd = {
    {2.921427362336e-01,-2.992940895311e-02,-1.175549738194e-02,},//estimator_t
    {1.853299468091e-01,1.124016852795e-01,1.558936042547e-01,},//estimator_y
    {-1.311459624648e-01,5.868308333745e-01,-9.413627737498e-02,},//estimator_v
};


struct estimators estimator_pitch_bckwd = { // the dynamics are different in backward flight 
    {2.816201935254e-01,2.195028141416e-02,6.425117075959e-03,},//estimator_t
{-4.906456780272e-02,1.557485598903e-01,1.998197820396e-01,},//estimator_y
{2.101656271909e-01,6.433650961749e-01,-9.845211488288e-02,},//estimator_v
};

struct estimators estimator_roll = {
   {2.752101469940e-01,7.068618044362e-03,2.671105874111e-03,},//estimator_t
{-7.715106397255e-03,7.831175130592e-02,2.935124632160e-01,},//estimator_y
{6.481105549059e-02,-2.766912287644e-01,-1.061272462132e-01,},//estimator_v
};




void find_losses(float v_initial, float delta_angle, struct estimators *estimator){
    delta_angle_in=delta_angle; // for logging. 
    controllerstate.delta_t = estimator->delta_t[0] + estimator->delta_t[1]*delta_angle + estimator->delta_t[2]*v_initial;
    controllerstate.delta_y = estimator->delta_pos[0] + estimator->delta_pos[1]*delta_angle + estimator->delta_pos[2]*v_initial;
    controllerstate.delta_v = estimator->delta_vel[0] + estimator->delta_vel[1]*delta_angle + estimator->delta_vel[2]*v_initial;
    controllerstate.delta_v=0;
}