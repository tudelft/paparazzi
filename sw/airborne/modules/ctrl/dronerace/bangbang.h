#include <stdio.h>
#include <stdbool.h>
struct BangDim{
    float x;
    float y; 
};

struct anaConstant{
    float c1;
    float c2;
};

struct controllerstatestruct{
    bool apply_compensation;
    bool in_transition; // when in transition from acceleration to braking the saturation dimension should not be optimized 
    float delta_t; 
    float delta_v; 
    float delta_y; 
};

// struct BangDim sign_correct;
// struct BangDim sat_angle; 


extern void optimizeBangBang(float pos_error_vel_x, float pos_error_vel_y,float v_desired);
float get_E_pos(float Vd, float angle);
float predict_path_analytical(float t_s, float angle, float Vd);
void find_constants(float yi,float vi);
float get_position_analytical(float t);
float get_velocity_analytical(float t);
float get_time_analytical(float V);
float psi_command; 
extern float bang_ctrl[3];
extern bool brake; 
extern struct controllerstatestruct controllerstate;
 FILE *comp_log_t;
FILE *bang_bang_t;
char filename5[512];
// FILE *comp_measure_log_t;
// FILE *brake_log_t;