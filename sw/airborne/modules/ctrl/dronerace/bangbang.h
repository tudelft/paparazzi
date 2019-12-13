#include <stdio.h>
struct BangDim{
    float x;
    float y; 
};


// struct BangDim sign_correct;
// struct BangDim sat_angle; 

int satdim;
int satdim_prev; 
float t_s; 
float t_t;
extern void optimize(float pos_error_vel_x, float pos_error_vel_y,float v_desired);
float get_E_pos(int dim);
float predict_path_analytical(float t_s, float angle);
extern float bang_ctrl[3];