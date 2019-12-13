#include "bangbang.h"
#include <math.h>
#include <stdbool.h>
#include "std.h"
#define SATURATION 0
#define SECOND 1
#define r2d 180./M_PI
#define d2r 1./r2d
int type; 
bool brake = false;
float bang_ctrl[3]; //control inputs that will be the final product of the bangbang optimizer 

struct BangDim sat_angle = {
    -10*d2r,
    10*d2r,
};
// struct BangDim sat_corr;
struct BangDim sign_corr=
{
    1,
    1,    
};
// struct BangDim pos_error;

float satangle;
float secangle;
float signcorrsat;
float signcorrsec;
float pos_error[2];
float sat_corr[2];

void optimize(float pos_error_vel_x, float pos_error_vel_y, float v_desired){
    pos_error[0]=pos_error_vel_x;
    pos_error[1]=pos_error_vel_y;
    

    // Determine which dimension will be saturated 

    float error_thresh = 1e-3; 
    if(pos_error[0]<0){
        sat_corr[0]=-sat_angle.x; //sat_corr are the saturated angles,but corrected for which side of the wp the drone is at. 
        sign_corr.x=-1; // sign_corr is required to correct the position error for the bisection part for which side of the gate we're at.
    }
    else
    {
        sat_corr[0]=sat_angle.x;
        sign_corr.x=1;
    }
    if(pos_error[1]<0){
        sat_corr[1] = -sat_angle.y;
        sign_corr.y=-1;
    }
    else
    {
        sat_corr[1]=sat_angle.y;
        sign_corr.y=1;
    }

    if(fabs(pos_error[0])>=fabs(pos_error[1])){
        satdim=0;
        satangle=sat_corr[0];
        secangle=sat_corr[1];
        signcorrsat=sign_corr.x;
        signcorrsec=sign_corr.y;
    }
    else{
        satdim=1;
        satangle=sat_corr[1];
        secangle=sat_corr[0];
        signcorrsat=sat_corr[1];
        signcorrsec=sat_corr[0];
    }

    if(satdim_prev!=satdim){
        brake=false;
    }
    satdim_prev = satdim;
    
    float E_pos = 1e9;

    // first optimize for saturation dimension 
    type=SATURATION;
    
    if(!brake){
        float t0 = 0; 
        float t1 = fabs(pos_error[satdim])+1;
        t_s = (t0+t1)/2.0;
        float t_s_old = 1e2;
        
        while(fabs(E_pos)>error_thresh&&fabs(t_s-t_s_old)>0.02){
            t_s_old=t_s; 
            E_pos=get_E_pos(satdim);
        }
    }


    bang_ctrl[0]+=0.1;
    bang_ctrl[1]+=0.2;
    bang_ctrl[2]+=0.3;
    printf("satdim: %d, pEx: %f, pEy: %f\n",satdim,pos_error[0],pos_error[1]);
};

float get_E_pos(int dim){
    float y_target=predict_path_analytical(t_s,sat_corr[dim]);

    return pos_error[dim]-y_target;
}

float predict_path_analytical(float t_s, float angle){
    return 2.0;
}