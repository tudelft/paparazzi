#include "bangbang.h"
#include <math.h>

#include "std.h"
#include "filter.h"
#include "subsystems/datalink/telemetry.h"
#define SATURATION 0
#define SECOND 1
#define r2d 180./M_PI
#define d2r M_PI/180.0

// float m = 0.42; //bebop mass [kg]
float g = 9.80665;//gravity
int type; 
bool brake = false;
float bang_ctrl[3]; //control inputs that will be the final product of the bangbang optimizer 

struct BangDim sat_angle = {
    -30*d2r,
    10*d2r,
};
// struct BangDim sat_corr;
struct BangDim sign_corr=
{
    1,
    1,    
};

struct anaConstant constant;
struct anaConstant constant_sat_accel;
struct anaConstant constant_sat_brake; //braking part of prediction
struct anaConstant constant_sec; //for logging 
// struct BangDim pos_error;

float satangle;
float secangle;
float signcorrsat;
float signcorrsec;
float pos_error[2];
float sat_corr[2];
float T;
float T_sat;
float T_sec; // for debugging purposes
int dim;
float Cd = 0.56;
float ang0 ;
float ang1 ;
float angc ;
float angc_old ;

float v0[2];
float mass=0.42;
float t_target;
float y_target;
float t_s=1e9;

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

    if(!brake){
        if(t_s<0.5 && t_target>0 &&t_s<t_target){
            brake=true;
        }
    }
    else{
        if(!(t_target>0)){
            brake = false;
        }
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
    dim=satdim;
    if(!brake){
        float t0 = 0; 
        float t1 = fabs(pos_error[satdim])+3;
        t_s = (t0+t1)/2.0;
        float t_s_old = 1e2;
        
        while(fabs(E_pos)>error_thresh&&fabs(t_s-t_s_old)>0.02){
            t_s_old=t_s; 
            E_pos=get_E_pos(v_desired, sat_corr[dim]);
            // printf("satdim: %i, E_pos: %f, t_s: %f , t0: %f, t1: %f\n",dim,E_pos*signcorrsat,t_s,t0,t1);
            if(E_pos*signcorrsat>0){
                t1=t_s;
            }
            else{
                t0=t_s;
            }
            t_s=(t0+t1)/2.0;
            
        }
        bang_ctrl[dim]=sat_corr[dim];
        printf("\n");
    }
    //if braking:
    else{
        ang0=-sat_corr[dim];
        ang1=sat_corr[dim];
        angc=(ang0+ang1)/2.0;
        angc_old = 1e9;
        E_pos = 1e9;
        while(fabs(E_pos)>error_thresh && fabs(angc-angc_old)>(0.1*d2r)){
            angc_old=angc;
            E_pos = get_E_pos(v_desired, angc);

            if(E_pos*signcorrsat>0){
                ang0=angc;
            }
            else{
                ang1=angc;
            }
            angc=(ang0+ang1)/2.;
        }
        bang_ctrl[dim]=angc;
    }
    //optimize second parameter
    type = SECOND;
    dim = 1-satdim;

    ang0 = -sat_corr[dim];
    ang1 = sat_corr[dim];
    angc = (ang0+ang1)/2.0;
    angc_old = 1e9;
    E_pos = 1e9;

    while(fabs(E_pos)>error_thresh&&fabs(angc-angc_old)>0.1*d2r){
        angc_old=angc;
        E_pos=get_E_pos(v_desired,angc);
        if(E_pos*signcorrsec>0){/* code */
            ang0=angc;
        }
        else
        {
            ang1=angc;
        }
        angc=(ang0+ang1)/2;
        // printf("angc: %f, angc_old: %f E_pos: %f\n",angc,angc_old,E_pos);
    }
    bang_ctrl[dim]=angc;
    printf("satdim: %i, brake: %i, theta_cmd: %f, phi_cmd: %f, errx: %f, erry: %f, t_s: %f, t_t: %f\n",satdim,brake,bang_ctrl[0],bang_ctrl[1],pos_error[0],pos_error[1],t_s,t_target);
    // printf("bang_ctrl[0]: %f, bang_ctrl[1]: %f \n",bang_ctrl[0],bang_ctrl[1]);
    //  fprintf(bang_bang_t,"time,satdim, brake, t_s, t_target, error_x, error_y, posx, posy, vxvel, vyvel, c1_sat,c2_sat, c1_sec, c2_sec\n");
    // fprintf(bang_bang_t,"%f, %i, %i, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",get_sys_time_float(), satdim, brake, t_s, t_target, pos_error_vel_x, pos_error_vel_y, dr_state.x, dr_state.y, v0[0], v0[1],
    // constant_sat_accel.c1,constant_sat_accel.c2, constant_sat_brake.c1, constant_sat_brake.c2, constant_sec.c1, constant_sec.c2, T_sat, T_sec);
};

float get_E_pos(float Vd, float angle){
    float y_target=predict_path_analytical(t_s,angle,Vd);

    return y_target-pos_error[dim];
}

float predict_path_analytical(float t_s, float angle,float Vd){
    if(dim==0){

        T = (mass*g)*tanf(-angle);//Thrust component forward 
    }
    else{
        T = (mass*g)*tanf(angle);//Thrust component forward 
    }

    //Correct NED velocity for the drone's heading which is the initial speed in path prediction
    v0[0]=dr_state.vx*cosf(dr_state.psi)-dr_state.vy*sinf(dr_state.psi); 
    v0[1]=dr_state.vx*sinf(dr_state.psi)+dr_state.vy*cosf(dr_state.psi);
    // v0[0]=2;
    // v0[1]=0.1;
    if(type==0){ // if saturation
        T_sat = T; 
        if(!brake){
            find_constants(0.0,v0[dim]); // find constant for accelerating part; 
            constant_sat_accel = constant;
            float ys = get_position_analytical(t_s);
            float vs = get_velocity_analytical(t_s);

            //predict braking part
            T=-1*T; 
            find_constants(ys,vs); //update constants for second part;
            constant_sat_brake = constant;
            t_target = get_time_analytical(Vd);
            y_target = get_position_analytical(t_target);
            t_target=t_target+t_s;
            
        }
        else{ //if braking 
            find_constants(0.0,v0[dim]);
            constant_sat_brake = constant;
            t_target=get_time_analytical(Vd);
            y_target = get_position_analytical(t_target);
            
        }
    }
    else{ //if second dimension
        T_sec = T;
        find_constants(0.0,v0[dim]);
        constant_sec = constant; 
        y_target=get_position_analytical(t_target);//find position in lateral direction using the predicted eta of the first dimension;    }
        // printf("y_target_sec: %f, t_target: %f, angle: %f, tanf: %f",y_target,t_target,angle,tanf(angle));
    // printf("\n");
    return y_target;
    }
}

 void find_constants(float yi,float vi){
    constant.c1 = (vi-T*(mass/Cd))*(-mass/Cd);
    constant.c2 = yi-constant.c1;
}
float get_position_analytical(float t){
    return constant.c1*expf(-(Cd/mass)*t)+constant.c2+T*(mass/Cd)*t;
}
float get_velocity_analytical(float t){
    return constant.c1*(-Cd/mass)*expf(-(Cd/mass)*t)+T*(mass/Cd);
}
float get_time_analytical(float V){
    float t_t = (-mass/Cd)*logf((V-(T*(mass/Cd)))/(constant.c1*(-Cd/mass)));
    if(isnan(t_t)){
        t_t=0;
    }
    return t_t;
}