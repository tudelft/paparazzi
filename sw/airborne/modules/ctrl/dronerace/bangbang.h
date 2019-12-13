#include <stdio.h>
struct BangDim{
    float x;
    float y; 
};

struct anaConstant{
    float c1;
    float c2;
};

// struct BangDim sign_correct;
// struct BangDim sat_angle; 

int satdim;
int satdim_prev; 
float t_s; 
float t_t;
extern void optimize(float pos_error_vel_x, float pos_error_vel_y,float v_desired);
float get_E_pos(float Vd);
float predict_path_analytical(float t_s, float angle, float Vd);
void find_constants(float yi,float vi);
float get_position_analytical(float t);
float get_velocity_analytical(float t);
float get_time_analytical(float V);
extern float bang_ctrl[3];