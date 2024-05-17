#include "aero_angles_ekf.h"
#include "state.h"
#include "modules/actuators/actuators.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "modules/datalink/telemetry.h"
#include "modules/rot_wing_drone/rotwing_state.h"
#include "modules/air_data/air_data.h"
#include "modules/meteo/ekf_aw.h"
#include <stdio.h>

// Global value holders
struct EKF_AOA_AOS ekf;
int periodic_counter = 0;

// Exposed to GCS
uint8_t reset_ekf = 0;
uint8_t set_Q_R = 0;
float Q_diag = 0.001f;
float R_diag = 0.01f;

// Coefficients and other non-changing
const float k_T_m4 = 0.004030000000000;
const float k_L_de = 2.044000000000000e-06;
const float k_L_da = 4.630000000000000e-06;
const float k_L_df = 5.110000000000000e-06;
const float k_T_mp = 0.003850000000000;
const float k_L_dr = 2.044000000000000e-06;
const float k_L_wing[3] = {0.335811392000000,0.099800000000000,0.615611143000000};
const float k_L_fuse = 0.050653927000000;
const float m = 7.0f;
const float g = 9.81f;

// Function declarations
// EKF Functions
void ekf_aoa_aos_predict(struct EKF_AOA_AOS *ekf, float dt);
void ekf_aoa_aos_update(struct EKF_AOA_AOS *ekf);
// Predict step helper functions
void predict_calc_f(struct EKF_AOA_AOS *ekf);
void predict_calc_x(struct EKF_AOA_AOS *ekf, float dt);
void predict_calc_A(struct EKF_AOA_AOS *ekf);
void predict_calc_P(struct EKF_AOA_AOS *ekf, float dt);
// Update step helper functions
void update_calc_y(struct EKF_AOA_AOS *ekf);
void update_calc_h(struct EKF_AOA_AOS *ekf, struct body_forces *bf);
void update_calc_C(struct EKF_AOA_AOS *ekf, struct body_forces *bf);
void update_calc_K(struct EKF_AOA_AOS *ekf);
void update_calc_x(struct EKF_AOA_AOS *ekf);
void update_calc_P(struct EKF_AOA_AOS *ekf);
// Helper helper functionss
struct body_forces calc_body_forces(struct EKF_AOA_AOS *ekf);
float calc_forces_body_north(struct EKF_AOA_AOS *ekf);
float calc_forces_body_east(struct EKF_AOA_AOS *ekf);
float calc_forces_body_down(struct EKF_AOA_AOS *ekf);
static void telemetry_send_aero_angles(struct transport_tx *trans, struct link_device *dev);
static void telemetry_send_aero_angles_aw(struct transport_tx *trans, struct link_device *dev);
void ekf_aoa_aos_update_aircraft_state(struct EKF_AOA_AOS *ekf);
void wrap_x(struct EKF_AOA_AOS *ekf);

//---------------------------------------------------------------------------------------
// Main EKF functions
//---------------------------------------------------------------------------------------

void ekf_aoa_aos_init() {
  // Init function for EKF, used in the xml as well
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AERO_ANGLES, telemetry_send_aero_angles);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AERO_ANGLES_AW, telemetry_send_aero_angles_aw);

  // Initialize state vector to zero
  ekf.x[0] = 0.0f; //aoa
  ekf.x[1] = 0.0f; //aos

  // Initialize covariance matrices
  float P_init[4] = {0.1f, 0.0f, 0.0f, 0.1f};  // Small, non-zero, diagonal values
  memcpy(ekf.P, P_init, sizeof(P_init));
  float Q_init[4] = {Q_diag, 0.0f, 0.0f, Q_diag};  // TODO: Make into setting
  memcpy(ekf.Q, Q_init, sizeof(Q_init));
  float R_init[4] = {R_diag, 0.0f, 0.0f, R_diag};  // TODO: Make into setting
  memcpy(ekf.R, R_init, sizeof(R_init));
}

void ekf_aoa_aos_periodic() {
  // Check for ekf reset
  if (reset_ekf) {
    ekf_aoa_aos_init();
    reset_ekf = 0;
  }
  // Check for setting Q and R values
  if (set_Q_R) {
    ekf.Q[0] = Q_diag;
    ekf.Q[3] = Q_diag;
    ekf.R[0] = R_diag;
    ekf.R[3] = R_diag;
    set_Q_R = 0;
  }
  // Periodic function for EKF, used in the xml as well
  periodic_counter++;
  // Runs at 30Hz
  ekf_aoa_aos_update_aircraft_state(&ekf);
  if (ekf.ac_state.V > 2){  // Protect from 0div
    ekf_aoa_aos_predict(&ekf, 0.033333333); // Predict step
    // Should only run every 10 calls (3Hz)
    if (periodic_counter % 10 == 0) {
      ekf_aoa_aos_update(&ekf); // Update step
    }
  }
  wrap_x(&ekf);
}

void ekf_aoa_aos_predict(struct EKF_AOA_AOS *ekf, float dt) {
  // Prediction step
  predict_calc_f(ekf);  // Simulate model
  predict_calc_x(ekf, dt);  // Update state vector
  predict_calc_A(ekf); // Update Jacobian of f
  predict_calc_P(ekf, dt); // Update covariance matrix
  //printf("PREDICT x: %f, %f, P: %f, %f, %f, %f\n", ekf->x[0], ekf->x[1], ekf->P[0], ekf->P[1], ekf->P[2], ekf->P[3]);
  //printf("PREDICT f: %f, %f, A: %f, %f, %f, %f\n", ekf->f[0], ekf->f[1], ekf->A[0], ekf->A[1], ekf->A[2], ekf->A[3]);
}

void ekf_aoa_aos_update(struct EKF_AOA_AOS *ekf) {
  // Update step
  // All model-based calculations are based on MATLAB model by Tomaso de Ponti
  update_calc_y(ekf); // Calculate measurement vector
  struct body_forces bf = calc_body_forces(ekf); // Calculate body forces
  update_calc_h(ekf, &bf); // Calculate measurement matrix
  update_calc_C(ekf, &bf); // Calculate Jacobian of h
  update_calc_K(ekf); // Calculate Kalman gain
  update_calc_x(ekf); // Update state vector
  update_calc_P(ekf); // Update covariance matrix
  //printf("UPDATE x: %f, %f, P: %f, %f, %f, %f, K: %f, %f, %f, %f\n", ekf->x[0], ekf->x[1], ekf->P[0], ekf->P[1], ekf->P[2], ekf->P[3], ekf->K[0], ekf->K[1], ekf->K[2], ekf->K[3]);
}

//---------------------------------------------------------------------------------------
// Sub-routines for PREDICT function
//---------------------------------------------------------------------------------------

void predict_calc_f(struct EKF_AOA_AOS *ekf){
  struct EKF_AOA_AOS_AC_STATE *ac = &ekf->ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC *pc = &ekf->ac_state_pre_calc;
  // Calc f
  // ⎡                                                            -ax⋅sin(α) + az⋅cos(α) + g⋅sin(α)⋅sin(θ) + g⋅cos(α)⋅cos(φ)⋅cos(θ)                          ⎤
  // ⎢                         q - (p⋅cos(α) + r⋅sin(α))⋅tan(β) + ─────────────────────────────────────────────────────────────────                          ⎥
  // ⎢                                                                                         V⋅cos(β)                                                      ⎥
  // ⎢                                                                                                                                                       ⎥
  // ⎢                      -ax⋅sin(β)⋅cos(α) + ay⋅cos(β) - az⋅sin(α)⋅sin(β) + g⋅(-sin(α)⋅sin(β)⋅cos(φ)⋅cos(θ) + sin(β)⋅sin(θ)⋅cos(α) + sin(φ)⋅cos(β)⋅cos(θ))⎥
  // ⎢p⋅sin(α) - r⋅cos(α) + ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────     ⎥
  // ⎣                                                                                      V                                                                ⎦
  ekf->f[0] = ac->q - ((ac->p*pc->calpha + ac->r*pc->salpha)*(pc->tbeta)) + ((-ac->ax*pc->salpha + ac->az*pc->calpha + g*pc->stheta*pc->salpha + g*pc->calpha*pc->cphi*pc->ctheta)/(ac->V*pc->cbeta));
  ekf->f[1] = (ac->p*pc->salpha - ac->r*pc->calpha) + ((-ac->ax*pc->sbeta*pc->calpha + ac->ay*pc->cbeta - ac->az*pc->salpha*pc->sbeta + g*(-pc->salpha*pc->sbeta*pc->cphi*pc->ctheta + pc->sbeta*pc->stheta*pc->calpha + pc->sphi*pc->cbeta*pc->ctheta))/ac->V);
}

void predict_calc_x(struct EKF_AOA_AOS *ekf, float dt){
  // x_plus = x + T*f
  float x_old[2] = {ekf->x[0], ekf->x[1]};
  ekf->x[0] = x_old[0] + dt*ekf->f[0];
  ekf->x[1] = x_old[1] + dt*ekf->f[1];
}

void predict_calc_A(struct EKF_AOA_AOS *ekf){
  struct EKF_AOA_AOS_AC_STATE *ac = &ekf->ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC *pc = &ekf->ac_state_pre_calc;
  // ⎡       V⋅(p⋅sin(α) - r⋅cos(α))⋅sin(β) - ax⋅cos(α) - az⋅sin(α) - g⋅sin(α)⋅cos(φ)⋅cos(θ) + g⋅sin(θ)⋅cos(α)                        -V⋅(p⋅cos(α) + r⋅sin(α)) + (-ax⋅sin(α) + az⋅cos(α) + g⋅sin(α)⋅sin(θ) + g⋅cos(α)⋅cos(φ)⋅cos(θ))⋅sin(β)                    ⎤
  // ⎢       ─────────────────────────────────────────────────────────────────────────────────────────────────                        ─────────────────────────────────────────────────────────────────────────────────────────────────────                    ⎥
  // ⎢                                                    V⋅cos(β)                                                                                                                           2                                                                 ⎥
  // ⎢                                                                                                                                                                                  V⋅cos (β)                                                              ⎥
  // ⎢                                                                                                                                                                                                                                                         ⎥
  // ⎢V⋅(p⋅cos(α) + r⋅sin(α)) + ax⋅sin(α)⋅sin(β) - az⋅sin(β)⋅cos(α) - g⋅(sin(α)⋅sin(θ) + cos(α)⋅cos(φ)⋅cos(θ))⋅sin(β)       -(ax⋅cos(α)⋅cos(β) + ay⋅sin(β) + az⋅sin(α)⋅cos(β) + g⋅(sin(α)⋅cos(β)⋅cos(φ)⋅cos(θ) + sin(β)⋅sin(φ)⋅cos(θ) - sin(θ)⋅cos(α)⋅cos(β))) ⎥
  // ⎢───────────────────────────────────────────────────────────────────────────────────────────────────────────────  ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────            ⎥
  // ⎣                                                       V                                                                                                                              V                                                                  ⎦
  ekf->A[0] = (ac->V*(ac->p*pc->salpha - ac->r*pc->calpha)*pc->sbeta - ac->ax*pc->calpha - ac->az*pc->salpha - g*pc->salpha*pc->cphi*pc->ctheta + g*pc->stheta*pc->calpha)/(ac->V*pc->cbeta);
  ekf->A[1] = (-ac->V*(ac->p*pc->calpha + ac->r*pc->salpha) + (-ac->ax*pc->salpha + ac->az*pc->calpha + g*pc->stheta*pc->salpha + g*pc->calpha*pc->cphi*pc->ctheta)*pc->sbeta)/(ac->V*pc->cbeta*pc->cbeta);
  ekf->A[2] = (ac->V*(ac->p*pc->calpha + ac->r*pc->salpha) + ac->ax*pc->salpha*pc->sbeta - ac->az*pc->sbeta*pc->calpha - g*(pc->salpha*pc->stheta + pc->calpha*pc->cphi*pc->ctheta)*pc->sbeta)/ac->V;
  ekf->A[3] = (-(ac->ax*pc->calpha*pc->cbeta + ac->ay*pc->sbeta + ac->az*pc->salpha*pc->cbeta + g*(pc->salpha*pc->cbeta*pc->cphi*pc->ctheta + pc->sbeta*pc->sphi*pc->ctheta - pc->stheta*pc->calpha*pc->cbeta)))/ac->V;
}

void predict_calc_P(struct EKF_AOA_AOS *ekf, float dt){
  // Updating P in the prediction step
  // P_plus = P + T*(A*P + P*A_t + Q)
  // ⎡   T⋅(2⋅a₀⋅p₀ + a₁⋅p₁ + a₁⋅p₂ + q₀) + p₀     T⋅(a₀⋅p₁ + a₁⋅p₃ + a₂⋅p₀ + a₃⋅p₁ + q₁) + p₁⎤
  // ⎢                                                                                        ⎥
  // ⎣T⋅(a₀⋅p₂ + a₁⋅p₃ + a₂⋅p₀ + a₃⋅p₂ + q₂) + p₂     T⋅(a₂⋅p₁ + a₂⋅p₂ + 2⋅a₃⋅p₃ + q₃) + p₃   ⎦
  float P_old[4] = {ekf->P[0], ekf->P[1], ekf->P[2], ekf->P[3]};
  ekf->P[0] = dt*(2*ekf->A[0]*P_old[0] + ekf->A[1]*P_old[1] +   ekf->A[1]*P_old[2] + ekf->Q[0]) + P_old[0];
  ekf->P[1] = dt*(  ekf->A[0]*P_old[1] + ekf->A[1]*P_old[3] +   ekf->A[2]*P_old[0] + ekf->A[3]*P_old[1] + ekf->Q[1]) + P_old[1];
  ekf->P[2] = dt*(  ekf->A[0]*P_old[2] + ekf->A[1]*P_old[3] +   ekf->A[2]*P_old[0] + ekf->A[3]*P_old[2] + ekf->Q[2]) + P_old[2];
  ekf->P[3] = dt*(  ekf->A[2]*P_old[1] + ekf->A[2]*P_old[2] + 2*ekf->A[3]*P_old[3] + ekf->Q[3]) + P_old[3];
}

//---------------------------------------------------------------------------------------
// Sub-routines for UPDATE function
//---------------------------------------------------------------------------------------

void update_calc_y(struct EKF_AOA_AOS *ekf){
  // "Measurement" vector
  // ⎡                      2⎤
  // ⎢-V_sq⋅k_L_wing_1⋅sskew ⎥
  // ⎢                       ⎥
  // ⎣         ay⋅m          ⎦
  struct EKF_AOA_AOS_AC_STATE *ac = &ekf->ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC *pc = &ekf->ac_state_pre_calc;
  ekf->y[0] = -(pc->V_sq) * k_L_wing[1] * (pc->sskew*pc->sskew);
  ekf->y[1] = ac->ay * m;
}

void update_calc_h(struct EKF_AOA_AOS *ekf, struct body_forces *bf){
  // "Sensor model" output vector
  //   ⎡                                                                   2                                                               ⎤
  //   ⎢-F_body_down⋅cos(α) + F_body_north⋅sin(α) + V_sq⋅α⋅k_L_wing_2⋅sskew  + V_sq⋅α  ⋅(k_L_fuse + k_L_wing_0) - m⋅(ax⋅sin(α) - az⋅cos(α))⎥
  //   ⎢                                                                                                                                   ⎥
  //   ⎣                          -F_body_down⋅sin(α)⋅sin(β) + F_body_east⋅cos(β) - F_body_north⋅sin(β)⋅cos(α)                             ⎦
  struct EKF_AOA_AOS_AC_STATE *ac = &ekf->ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC *pc = &ekf->ac_state_pre_calc;
  float alpha = ekf->x[0];
  ekf->h[0] = -bf->F_body_down*pc->calpha + bf->F_body_north*pc->salpha + pc->V_sq*alpha*k_L_wing[2]*pc->sskew*pc->sskew + pc->V_sq*alpha*(k_L_fuse + k_L_wing[0]) - m*(ac->ax*pc->salpha - ac->az*pc->calpha);
  ekf->h[1] = -bf->F_body_down*pc->salpha*pc->sbeta + bf->F_body_east*pc->cbeta - bf->F_body_north*pc->sbeta*pc->calpha;
}

void update_calc_C(struct EKF_AOA_AOS *ekf, struct body_forces *bf){
  // Jacobian of h(x,u) wrt x
  //   ⎡F_body_down⋅sin(α) + F_body_north⋅cos(α) + + V_sq⋅k_L_wing_2⋅sskew^2 + V_sq⋅(k_L_fuse + k_L_wing_0) - m⋅(ax⋅cos(α) + az⋅sin(α))                                                 0                         ⎤          
  //   ⎢                                                                                                                                                                                                      ⎥                                                      
  //   ⎣                      -F_body_down⋅sin(β)⋅cos(α) + F_body_north⋅sin(α)⋅sin(β)                                                 -F_body_down⋅sin(α)⋅cos(β) - F_body_east⋅sin(β) - F_body_north⋅cos(α)⋅cos(β)⎦
  struct EKF_AOA_AOS_AC_STATE *ac = &ekf->ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC *pc = &ekf->ac_state_pre_calc;
  ekf->C[0] = bf->F_body_down*pc->salpha + bf->F_body_north*pc->calpha + pc->V_sq*k_L_wing[2]*pc->sskew*pc->sskew + pc->V_sq*(k_L_fuse + k_L_wing[0]) - m*(ac->ax*pc->calpha + ac->az*pc->salpha);
  ekf->C[1] = 0;
  ekf->C[2] = -bf->F_body_down*pc->sbeta*pc->calpha + bf->F_body_north*pc->salpha*pc->sbeta;
  ekf->C[3] = -bf->F_body_down*pc->salpha*pc->cbeta - bf->F_body_east*pc->sbeta - bf->F_body_north*pc->calpha*pc->cbeta;
}

void update_calc_K(struct EKF_AOA_AOS *ekf){
  // Kalman gain matrix
  // G = C*P*C_t + R
  // ⎡c₀⋅(c₀⋅p₀ + c₁⋅p₂) + c₁⋅(c₀⋅p₁ + c₁⋅p₃) + r₀  c₂⋅(c₀⋅p₀ + c₁⋅p₂) + c₃⋅(c₀⋅p₁ + c₁⋅p₃) + r₁⎤
  // ⎢                                                                                          ⎥
  // ⎣c₀⋅(c₂⋅p₀ + c₃⋅p₂) + c₁⋅(c₂⋅p₁ + c₃⋅p₃) + r₂  c₂⋅(c₂⋅p₀ + c₃⋅p₂) + c₃⋅(c₂⋅p₁ + c₃⋅p₃) + r₃⎦
  float G[4] = {ekf->C[0]*(ekf->C[0]*ekf->P[0] + ekf->C[1]*ekf->P[2]) + ekf->C[1]*(ekf->C[0]*ekf->P[1] + ekf->C[1]*ekf->P[3]) + ekf->R[0], ekf->C[2]*(ekf->C[0]*ekf->P[0] + ekf->C[1]*ekf->P[2]) + ekf->C[3]*(ekf->C[0]*ekf->P[1] + ekf->C[1]*ekf->P[3]) + ekf->R[1],
                ekf->C[0]*(ekf->C[2]*ekf->P[0] + ekf->C[3]*ekf->P[2]) + ekf->C[1]*(ekf->C[2]*ekf->P[1] + ekf->C[3]*ekf->P[3]) + ekf->R[2], ekf->C[2]*(ekf->C[2]*ekf->P[0] + ekf->C[3]*ekf->P[2]) + ekf->C[3]*(ekf->C[2]*ekf->P[1] + ekf->C[3]*ekf->P[3]) + ekf->R[3]};
  float G_det = G[0]*G[3] - G[1]*G[2];
  float G_inv[4] = {G[3]/G_det, -G[1]/G_det, -G[2]/G_det, G[0]/G_det};
  // K = P*C_t*(C*P*C_t + R)^-1 = P*C_t*G_inv
  // ⎡g₀⋅(c₀⋅p₀ + c₁⋅p₁) + g₂⋅(c₂⋅p₀ + c₃⋅p₁)  g₁⋅(c₀⋅p₀ + c₁⋅p₁) + g₃⋅(c₂⋅p₀ + c₃⋅p₁)⎤
  // ⎢                                                                                ⎥  
  // ⎣g₀⋅(c₀⋅p₂ + c₁⋅p₃) + g₂⋅(c₂⋅p₂ + c₃⋅p₃)  g₁⋅(c₀⋅p₂ + c₁⋅p₃) + g₃⋅(c₂⋅p₂ + c₃⋅p₃)⎦
  // (g = elements of G_inv)
  ekf->K[0] = G_inv[0]*(ekf->C[0]*ekf->P[0] + ekf->C[1]*ekf->P[1]) + G_inv[2]*(ekf->C[2]*ekf->P[0] + ekf->C[3]*ekf->P[1]);
  ekf->K[1] = G_inv[1]*(ekf->C[0]*ekf->P[0] + ekf->C[1]*ekf->P[1]) + G_inv[3]*(ekf->C[2]*ekf->P[0] + ekf->C[3]*ekf->P[1]);
  ekf->K[2] = G_inv[0]*(ekf->C[0]*ekf->P[2] + ekf->C[1]*ekf->P[3]) + G_inv[2]*(ekf->C[2]*ekf->P[2] + ekf->C[3]*ekf->P[3]);
  ekf->K[3] = G_inv[1]*(ekf->C[0]*ekf->P[2] + ekf->C[1]*ekf->P[3]) + G_inv[3]*(ekf->C[2]*ekf->P[2] + ekf->C[3]*ekf->P[3]);
}

void update_calc_x(struct EKF_AOA_AOS *ekf){
  // NOTE: x after update does not move in time
  // x_plus = x_plus + K*(y - h)
  // ⎡k₀⋅(-h₀ + y₀) + k₁⋅(-h₁ + y₁) + x₀⎤
  // ⎢                                  ⎥
  // ⎣k₂⋅(-h₀ + y₀) + k₃⋅(-h₁ + y₁) + x₁⎦
  float x_old[2] = {ekf->x[0], ekf->x[1]};
  ekf->x[0] = ekf->K[0]*(-ekf->h[0] + ekf->y[0]) + ekf->K[1]*(-ekf->h[1] + ekf->y[1]) + x_old[0];
  ekf->x[1] = ekf->K[2]*(-ekf->h[0] + ekf->y[0]) + ekf->K[3]*(-ekf->h[1] + ekf->y[1]) + x_old[1];
}

void update_calc_P(struct EKF_AOA_AOS *ekf){
  // Updating P in the update step, after K is recalced
  // P = (I - K*C)*P
  // ⎡p₀⋅(-c₀⋅k₀ - c₂⋅k₁ + 1) + p₂⋅(-c₁⋅k₀ - c₃⋅k₁)  p₁⋅(-c₀⋅k₀ - c₂⋅k₁ + 1) + p₃⋅(-c₁⋅k₀ - c₃⋅k₁)⎤
  // ⎢                                                                                            ⎥
  // ⎣p₀⋅(-c₀⋅k₂ - c₂⋅k₃) + p₂⋅(-c₁⋅k₂ - c₃⋅k₃ + 1)  p₁⋅(-c₀⋅k₂ - c₂⋅k₃) + p₃⋅(-c₁⋅k₂ - c₃⋅k₃ + 1)⎦
  float P_old[4] = {ekf->P[0], ekf->P[1], ekf->P[2], ekf->P[3]};
  ekf->P[0] = P_old[0]*(-ekf->C[0]*ekf->K[0] - ekf->C[2]*ekf->K[1] + 1) + P_old[2]*(-ekf->C[1]*ekf->K[0] - ekf->C[3]*ekf->K[1]);
  ekf->P[1] = P_old[1]*(-ekf->C[0]*ekf->K[0] - ekf->C[2]*ekf->K[1] + 1) + P_old[3]*(-ekf->C[1]*ekf->K[0] - ekf->C[3]*ekf->K[1]);
  ekf->P[2] = P_old[0]*(-ekf->C[0]*ekf->K[2] - ekf->C[2]*ekf->K[3])     + P_old[2]*(-ekf->C[1]*ekf->K[2] - ekf->C[3]*ekf->K[3] + 1);
  ekf->P[3] = P_old[1]*(-ekf->C[0]*ekf->K[2] - ekf->C[2]*ekf->K[3])     + P_old[3]*(-ekf->C[1]*ekf->K[2] - ekf->C[3]*ekf->K[3] + 1);
}

//---------------------------------------------------------------------------------------
// Sub-sub-routines
//---------------------------------------------------------------------------------------

struct body_forces calc_body_forces(struct EKF_AOA_AOS *ekf){
  float F_body_north = calc_forces_body_north(ekf); // towards nose
  float F_body_east = calc_forces_body_east(ekf); // towards right
  float F_body_down = calc_forces_body_down(ekf);
  struct body_forces bf = {F_body_down, F_body_north, F_body_east};
  return bf;
}

float calc_forces_body_north(struct EKF_AOA_AOS *ekf){
  struct EKF_AOA_AOS_AC_STATE *ac = &ekf->ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC *pc = &ekf->ac_state_pre_calc;

  float T_mp = k_T_mp * ac->mp;         // front (north)
  float W_n = -pc->stheta * m * g;  // front (north)
  float F_body_north = T_mp + W_n;
  return F_body_north;
}

float calc_forces_body_east(struct EKF_AOA_AOS *ekf){
  struct EKF_AOA_AOS_AC_STATE *ac = &ekf->ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC *pc = &ekf->ac_state_pre_calc;

  float L_dr = k_L_dr * ac->dr * pc->V_sq;   // left (west)
  float W_e = pc->sphi * m * g;              // right (east)
  float F_body_east = W_e - L_dr;
  return F_body_east;
}

float calc_forces_body_down(struct EKF_AOA_AOS *ekf){
  struct EKF_AOA_AOS_AC_STATE *ac = &ekf->ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC *pc = &ekf->ac_state_pre_calc;

  float T_m4 = k_T_m4 * (ac->mf + ac->mb + ac->ml + ac->mr);              // up
  float L_de = k_L_de * ac->de * pc->V_sq;                                // up
  float L_da = k_L_da * ac->da * (pc->sskew * pc->sskew) * pc->V_sq;      // up
  float L_df = k_L_df * ac->df * (pc->sskew * pc->sskew) * pc->V_sq;      // up
  float W_d = pc->ctheta * pc->cphi * m * g;                              // down
  float F_body_down = W_d - (T_m4 + L_de + L_da + L_df);
  return F_body_down;
}

//---------------------------------------------------------------------------------------
// Other support functions
//---------------------------------------------------------------------------------------

static void telemetry_send_aero_angles(struct transport_tx *trans, struct link_device *dev) {
  // Telemetry callback function
  float x[2] = {ekf.x[0], ekf.x[1]};
  pprz_msg_send_AERO_ANGLES(trans, dev, AC_ID, &x[0], &x[1]);
}

static void telemetry_send_aero_angles_aw(struct transport_tx *trans, struct link_device *dev) {
  // Telemetry callback function
  pprz_msg_send_AERO_ANGLES_AW(trans, dev, AC_ID, &ekf_aw_aoa_aos.aoa, &ekf_aw_aoa_aos.aos);
}

void ekf_aoa_aos_update_aircraft_state(struct EKF_AOA_AOS *ekf) {
  // Update state vector
  struct EKF_AOA_AOS_AC_STATE new_ac_state;
  new_ac_state.ax = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->x);
  new_ac_state.ay = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y);
  new_ac_state.az = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
  new_ac_state.phi = stateGetNedToBodyEulers_f()->phi;
  new_ac_state.theta = stateGetNedToBodyEulers_f()->theta;
  new_ac_state.p = stateGetBodyRates_f()->p;
  new_ac_state.q = stateGetBodyRates_f()->q;
  new_ac_state.r = stateGetBodyRates_f()->r;
  new_ac_state.V = air_data.airspeed;
  // Read actuator values
  new_ac_state.mf = actuators[0];
  new_ac_state.mr = actuators[1];
  new_ac_state.mb = actuators[2];
  new_ac_state.ml = actuators[3];
  new_ac_state.mp = actuators[4];
  new_ac_state.skew = rotwing_state_skewing.wing_angle_deg * 0.017453293; // deg to rad
  new_ac_state.de = actuators[6];
  new_ac_state.dr = actuators[7];
  new_ac_state.da = actuators[13];  // TODO: Is it the right way around?
  new_ac_state.df = actuators[14];  // TODO: Is it the right way around?
  // Pre-calculation
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC new_ac_state_pre_calc;
  new_ac_state_pre_calc.V_sq = new_ac_state.V * new_ac_state.V;
  new_ac_state_pre_calc.sphi = sin(new_ac_state.phi);
  new_ac_state_pre_calc.cphi = cos(new_ac_state.phi);
  new_ac_state_pre_calc.stheta = sin(new_ac_state.theta);
  new_ac_state_pre_calc.ctheta = cos(new_ac_state.theta);
  new_ac_state_pre_calc.salpha = sin(ekf->x[0]);
  new_ac_state_pre_calc.calpha = cos(ekf->x[0]);
  new_ac_state_pre_calc.sbeta = sin(ekf->x[1]);
  new_ac_state_pre_calc.cbeta = cos(ekf->x[1]);
  new_ac_state_pre_calc.tbeta = tan(ekf->x[1]);
  new_ac_state_pre_calc.sskew = sin(new_ac_state.skew);
  // Update EKF struct
  ekf->ac_state = new_ac_state;
  ekf->ac_state_pre_calc = new_ac_state_pre_calc;
}

void wrap_x(struct EKF_AOA_AOS *ekf){
  // Wrap aoa and aos angles to be between -pi and pi radians
  while (ekf->x[0] > M_PI) {
    ekf->x[0] -= 2 * M_PI;
  }
  while (ekf->x[0] < -M_PI) {
    ekf->x[0] += 2 * M_PI;
  }
  while (ekf->x[1] > M_PI) {
    ekf->x[1] -= 2 * M_PI;
  }
  while (ekf->x[1] < -M_PI) {
    ekf->x[1] += 2 * M_PI;
  }
}
