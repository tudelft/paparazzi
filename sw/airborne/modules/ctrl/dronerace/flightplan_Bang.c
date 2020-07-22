#include "std.h"
#include "stdio.h"
#include "filter.h"
#include "state.h"
#include "flightplan_Bang.h"

struct bangbang_fp_struct dr_bang;

const struct bangbang_fp_struct Banggates[MAX_GATES] = {
{0, -3.0, 0.0,-1.75,0.0,0,STARTPOINT,PID},
{0, 3.0, 0.0,-1.75,0.0,0.2,GATE,BANGBANG},
};

static void update_gate_setpoints(void){
    dr_bang.gate_x= Banggates[dr_bang.gate_nr].gate_x;
    dr_bang.gate_y= Banggates[dr_bang.gate_nr].gate_y;
    dr_bang.gate_z= Banggates[dr_bang.gate_nr].gate_z;
    dr_bang.gate_psi= Banggates[dr_bang.gate_nr].gate_psi;

    dr_bang.gate_speed= Banggates[dr_bang.gate_nr].gate_speed;
    dr_bang.gate_type= Banggates[dr_bang.gate_nr].gate_type;
    dr_bang.controller_type = Banggates[dr_bang.gate_nr].controller_type;
}

void flightplan_reset(){
    dr_bang.gate_nr=0;
}

void flightplan_run(void){
    float pos_error_x;
    float pos_error_z;
    float pos_error_y;
    int next_gate_nr;
    float error_speed;
    float error_psi; 
    float dist2gate=0;
    update_gate_setpoints();

    if(dr_bang.gate_nr==MAX_GATES-1){
        next_gate_nr=0;
    }
    else{
        next_gate_nr=dr_bang.gate_nr+1;
    }

    pos_error_x=dr_bang.gate_x-dr_state.x; 
    pos_error_y=dr_bang.gate_y-dr_state.y;
    pos_error_z=dr_bang.gate_z-dr_state.z;
    dist2gate=sqrtf((pos_error_x*pos_error_x)+(pos_error_y*pos_error_y));
    error_speed=dr_bang.gate_speed-((dr_state.vx*dr_state.vx)+(dr_state.vy*dr_state.vy));
    
    if(dist2gate<0.5 && error_speed<0.5){
        dr_bang.gate_psi=atan2f(Banggates[next_gate_nr].gate_y-Banggates[dr_bang.gate_nr].gate_y,Banggates[next_gate_nr].gate_x-Banggates[dr_bang.gate_nr].gate_x);
        dr_bang.controller_type=PID;
        if(abs(dr_bang.gate_psi-dr_state.psi)<0.15){
        dr_bang.gate_nr=next_gate_nr;
        }
        printf("new gate: %i\n",next_gate_nr);
    }
}