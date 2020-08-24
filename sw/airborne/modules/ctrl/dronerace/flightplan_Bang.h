#include <stdbool.h>
#define GATE 0 
#define STARTGATE 1
#define ENDGATE 2
#define BANGBANG 2
#define PID 3 
#define HIGHPID 4 // high-gain pid
#define TURNING 1
struct bangbang_fp_struct
{
    int gate_nr;
    float gate_x; 
    float gate_y;
    float gate_z; 
    float gate_psi; 
    float gate_speed; 
    int gate_type;
    int controller_type;
    int turning;
    float psi_forced;
    bool overwrite_psi;
     float sat_angle;
};
#define MAX_GATES 12

extern const struct bangbang_fp_struct Banggates[MAX_GATES];
extern struct bangbang_fp_struct dr_bang;

float dist2gate;
// Functions 
extern void flightplan_reset(void);
extern void flightplan_run(void);
float wrap_angle(float ang,float ang2);
FILE *fp_logger_t;