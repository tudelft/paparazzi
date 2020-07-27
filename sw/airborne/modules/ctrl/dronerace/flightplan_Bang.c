#include "std.h"
#include "stdio.h"
#include "filter.h"
#include "state.h"
#include "flightplan_Bang.h"
#include "subsystems/datalink/telemetry.h"

#define LOG
struct bangbang_fp_struct dr_bang;
int next_gate_nr;
const struct bangbang_fp_struct Banggates[MAX_GATES] = {
{0, -2.0, 0.0,-1.75,0.0,0,STARTPOINT,PID},
{1, 2.0, 0.0,-1.75,0.0,0.2,GATE,BANGBANG},
};

static void update_gate_setpoints(void){
    dr_bang.gate_x= Banggates[dr_bang.gate_nr].gate_x;
    dr_bang.gate_y= Banggates[dr_bang.gate_nr].gate_y;
    dr_bang.gate_z= Banggates[dr_bang.gate_nr].gate_z;
    dr_bang.gate_psi= Banggates[dr_bang.gate_nr].gate_psi;

    dr_bang.gate_speed= Banggates[dr_bang.gate_nr].gate_speed;
    dr_bang.gate_type= Banggates[dr_bang.gate_nr].gate_type;
    dr_bang.controller_type = Banggates[dr_bang.gate_nr].controller_type;

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
    
    if(abs(pos_error_x_vel)<0.5 && abs(error_speed)<1){
        dr_bang.gate_psi=atan2f(Banggates[next_gate_nr].gate_y-Banggates[dr_bang.gate_nr].gate_y,Banggates[next_gate_nr].gate_x-Banggates[dr_bang.gate_nr].gate_x);
        dr_bang.controller_type=PID;
        if(abs(dr_bang.gate_psi-dr_state.psi)<0.35){
        dr_bang.gate_nr=next_gate_nr;
        printf("new gate: %i\n",next_gate_nr);
        }
    }

    if(dr_bang.gate_nr==next_gate_nr){ //only update setpoints if the gate identifier has changed (todo: fix discrepancy when there is only one gate)
        update_gate_setpoints();
    }

    #ifdef LOG
        fprintf(fp_logger_t,"%f, %d, %d, %d, %f, %f, %f, %f\n",get_sys_time_float(),dr_bang.gate_nr,dr_bang.gate_type,dr_bang.controller_type,dr_bang.gate_x,dr_bang.gate_y,dr_bang.gate_z,dr_bang.gate_psi);
    #endif
  
    printf("Controltype: %d, error_speed: %f, dist2gate: %f, psi: %f \n",dr_bang.controller_type,error_speed,dist2gate,dr_state.psi);
}