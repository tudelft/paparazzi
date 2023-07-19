#ifndef TAILSITTER_AUTO_TOFF_H
#define TAILSITTER_AUTO_TOFF_H

extern int16_t pwm2pprz(float pwm);
extern int16_t take_off_stage(float theta);
extern int16_t take_off_thrust(void);
extern void take_off_enter(void);

extern float Kq;
extern int16_t stage_1_thrust;
#endif