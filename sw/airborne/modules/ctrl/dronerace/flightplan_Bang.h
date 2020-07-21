#define GATE 0 
#define STARTPOINT 1
#define BANGBANG 2
#define PID 3 

struct bangbang_fp_struct
{
    int gate_nr;
    float gate_x; 
    float gate_y;
    float gate_z; 
    float gate_psi; 
    float gate_speed; 
    int gate_type;
    int controller_type
}
;
#define MAX_GATES 2 

extern const struct bangbang_fp_struct Banggates[MAX_GATES];
extern struct bangbang_fp_struct dr_bang;

// Functions 
extern void flightplan_reset(void);
extern void flightplan_run(void);
