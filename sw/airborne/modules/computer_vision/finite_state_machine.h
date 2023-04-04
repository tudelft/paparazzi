#ifndef FINITE_STATE_MACHINE_H
#define FINITE_STATE_MACHINE_H
#pragma


struct OpticalFlow
{
    float left;
    float right;
    float middle;
    float middle_prev;
    float middle_prev_prev;
    float middle_divergence;
    float middle_divergence_prev;
    float right_left_ratio;
};

struct OpticalFlow run_fsm(float flow_left, float flow_right, float flow_middle);
void log_flow(void);
void state_safe(void);
void state_obs_right(void);
void state_obs_left(void);
void state_obs_middle(void);
void state_just_turned(void);
void state_out_of_bounds(void);
void reset_counters_turn(void);

#endif