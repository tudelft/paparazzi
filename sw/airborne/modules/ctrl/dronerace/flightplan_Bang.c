#include "std.h"
#include "stdio.h"
#include "filter.h"
#include "state.h"
#include "flightplan_Bang.h"
#include "subsystems/datalink/telemetry.h"

#define LOG
struct bangbang_fp_struct dr_bang;
int next_gate_nr;
int timer1=0;
const struct bangbang_fp_struct Banggates[MAX_GATES] = {
{0, -2.0,0.0,-1.75,M_PI,0.2,STARTPOINT,PID,0,0},
{1, 2.0,0.0,-1.75,0,0.2,GATE,BANGBANG,0,0}
};

static void update_gate_setpoints(void){
    dr_bang.gate_x= Banggates[dr_bang.gate_nr].gate_x;
    dr_bang.gate_y= Banggates[dr_bang.gate_nr].gate_y;
    dr_bang.gate_z= Banggates[dr_bang.gate_nr].gate_z;
    

    dr_bang.gate_speed= Banggates[dr_bang.gate_nr].gate_speed;
    dr_bang.gate_type= Banggates[dr_bang.gate_nr].gate_type;
    dr_bang.controller_type = Banggates[dr_bang.gate_nr].controller_type;
    dr_bang.turning=0;
    dr_bang.psi_offset=Banggates[dr_bang.gate_nr].psi_offset;
    dr_bang.gate_psi= Banggates[dr_bang.gate_nr].gate_psi;
    
    if(dr_bang.gate_nr==MAX_GATES-1){
        next_gate_nr=0;
    }
    else{
        next_gate_nr=dr_bang.gate_nr+1;
    }
    
}

void flightplan_reset(){
    dr_bang.gate_nr=0;
    update_gate_setpoints();
    dr_bang.controller_type=PID; // in a reset fly to the first waypoint in PID mode
    timer1=0;
    
}
    float pos_error_x;
    float pos_error_z;
    float pos_error_y;
    float pos_error_x_vel; 

    float error_speed;
    float error_psi; 

void flightplan_run(void){
    
   

    // update_gate_setpoints();

    pos_error_x=dr_bang.gate_x-dr_state.x; 
    pos_error_y=dr_bang.gate_y-dr_state.y;
    pos_error_z=dr_bang.gate_z-dr_state.z;
    pos_error_x_vel=cosf(dr_state.psi)*pos_error_x+sinf(dr_state.psi)*pos_error_y;
    // dist2gate=sqrtf((pos_error_x*pos_error_x)+(pos_error_y*pos_error_y));
    error_speed=dr_bang.gate_speed-((dr_state.vx*dr_state.vx)+(dr_state.vy*dr_state.vy));
    if(dist2gate<0.5){
        
        if(abs(pos_error_x_vel)<0.3 && abs(dr_state.theta)<0.5){
            dr_bang.controller_type=PID;
            dr_bang.gate_psi=Banggates[next_gate_nr].gate_psi;//atan2f(Banggates[next_gate_nr].gate_y-dr_state.y,Banggates[next_gate_nr].gate_x-dr_state.x);
            printf("\n Gate_psi: %f\n",dr_bang.gate_psi);
            dr_bang.turning=TURNING;            
        }
        if(abs(dr_state.psi-dr_bang.gate_psi)<0.5){ //only go toward next waypoint after a pause
                timer1+=1;
                if(timer1>512){
                dr_bang.gate_nr=next_gate_nr;
                printf("new gate: %i\n",next_gate_nr);
                timer1=0;
                }
            }
    }

    if(dr_bang.gate_nr==next_gate_nr){ //only update setpoints if the gate identifier has changed (todo: fix discrepancy when there is only one gate)
        update_gate_setpoints();
    }

    #ifdef LOG
        fprintf(fp_logger_t,"%f, %d, %d, %d, %f, %f, %f, %f\n",get_sys_time_float(),dr_bang.gate_nr,dr_bang.gate_type,dr_bang.controller_type,dr_bang.gate_x,dr_bang.gate_y,dr_bang.gate_z,dr_bang.gate_psi);
    #endif
  
    // printf("Controltype: %d, error_speed: %f, dist2gate: %f, psi: %f \n",dr_bang.controller_type,error_speed,dist2gate,dr_state.psi);
}

float wrap_angle(float ang,float ang2){
    if(ang>ang2){
        ang=ang-2*M_PI;
    }
    if(ang<-ang2){
        ang=ang+2*M_PI;
    }
    return ang;
}