/*
Extened kalman filter based on https://en.wikipedia.org/wiki/Extended_Kalman_filter
Section 5.3: Non-additive noise formulation and equations
*/

#ifndef EKF_ROBIN_H
#define EKF_ROBIN_H

#define EKF_NUM_STATES 15
#define EKF_NUM_INPUTS 6
#define EKF_NUM_OUTPUTS 6
#include "std.h" 
#include "math/pprz_algebra_float.h"


extern float ekf_X[EKF_NUM_STATES];
extern float ekf_U[EKF_NUM_INPUTS];
extern float ekf_Z[EKF_NUM_OUTPUTS];
extern float ekf_P[EKF_NUM_STATES][EKF_NUM_STATES];
extern float ekf_Q[EKF_NUM_INPUTS][EKF_NUM_INPUTS];
extern float ekf_R[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
extern float ekf_H[EKF_NUM_OUTPUTS][EKF_NUM_STATES];

extern float ev_pos[3];
extern float ev_att[3];

void ekf_set_diag(float **a, float *b, int n);

extern void ekf_init(void);

void ekf_f(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES]);
void ekf_F(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES][EKF_NUM_STATES]);
void ekf_L(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES][EKF_NUM_INPUTS]);

void ekf_f_rk4(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], const float dt, float out[EKF_NUM_STATES]);

extern void ekf_step(const float U[EKF_NUM_INPUTS], const float Z[EKF_NUM_OUTPUTS], const float dt);
extern void ekf_prediction_step(const float U[EKF_NUM_INPUTS], const float dt);
extern void ekf_measurement_step(const float Z[EKF_NUM_OUTPUTS]);

extern void ekf_run(void);

extern void external_vision_update(uint8_t *buf);

#endif
