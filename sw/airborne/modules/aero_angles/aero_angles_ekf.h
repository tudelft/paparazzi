#ifndef EKF_AOA_AOS_H
#define EKF_AOA_AOS_H

#include "math/pprz_algebra_float.h"

struct EKF_AOA_AOS_AC_STATE {
  // State of motion
  float V;
  float ax, ay, az;
  float p, q, r;
  float phi, theta;
  // Actuator state
  float skew; // Skew angle of the wing
  float mf, mb, mr, ml, mp; // Hover motors, pusher motor
  float de, da, dr, df; // Elevator, aileron, rudder, flaps
};

struct EKF_AOA_AOS_AC_STATE_PRE_CALC {
  // Helper to avoid calculating trig functions repetitively
  float V_sq;
  float sphi;
  float cphi;
  float stheta;
  float ctheta;
  float salpha;
  float calpha;
  float sbeta;
  float cbeta;
  float tbeta;
  float sskew;
};

struct EKF_AOA_AOS {
  // Helpers
  struct EKF_AOA_AOS_AC_STATE ac_state;
  struct EKF_AOA_AOS_AC_STATE_PRE_CALC ac_state_pre_calc;
  // General
  float x[2];     // State vector [aoa; aos]
  float P[4];     // State covariance matrix
  // Predict step
  float f[2];     // State transition matrix (predict step)
  float A[4];     // Jacobian of the state transition matrix w.r.t. state vector
  float Q[4];     // Process noise covariance matrix "Model noise"
  // Update step
  float y[2];     // Measurement "Real sensor output"
  float h[2];     // Measurement matrix "Sensor model ouput"
  float C[4];     // Jacobian of the measurement matrix w.r.t. state vector
  float R[4];     // Measurement noise covariance matrix "Sensor noise"
  float K[4];     // Kalman gain
};

struct body_forces {
  // Helper struct
  float F_body_down;
  float F_body_north;
  float F_body_east;
};

// GCS exposed variables
extern uint8_t reset_ekf;
extern uint8_t set_Q_R;
extern float Q_diag;
extern float R_diag;

// XML Functions
extern void ekf_aoa_aos_init();
extern void ekf_aoa_aos_periodic();

#endif // EKF_AOA_AOS_H