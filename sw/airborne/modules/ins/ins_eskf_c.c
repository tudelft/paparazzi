/**
 * @file modules/ins/ins_eskf_c.c
 * @brief Error-State Kalman Filter (ESKF) implementation in pure C.
 *
 * @ingroup ins_eskf_c
 *
 * This filter fuses IMU (accelerometer and gyroscope) measurements with GPS
 * and magnetometer data to estimate the attitude, velocity, and position 
 * of the vehicle, alongside gyroscope and accelerometer biases.
 * 
 * Operating Model:
 * 1. Nominal State Update: Predicts state forward using kinematic equations.
 * 2. Error State Update: Propagates error covariance matrix forward in time.
 * 3. Measurement Update: Corrects the nominal state by computing an error state
 *    from exteroceptive sensors (GPS/Mag) and applying it to the nominal state.
 */

#include "modules/ins/ins_eskf_c.h"
#include "math/pprz_isa.h"
#include "state.h"

struct ekf2_c_t ekf2_c_state;

#ifndef INS_ESKF_C_GPS_P_NOISE
#define INS_ESKF_C_GPS_P_NOISE 0.5f
#endif

#ifndef INS_ESKF_C_GPS_V_NOISE
#define INS_ESKF_C_GPS_V_NOISE 0.3f
#endif

float ins_eskf_c_gps_p_noise = INS_ESKF_C_GPS_P_NOISE;
float ins_eskf_c_gps_v_noise = INS_ESKF_C_GPS_V_NOISE;
static struct FloatVect3 mag_earth_ref = {0.38925f, 0.00179f, 0.92113f};

/* ABI Bindings Configuration defaults from XML */
#ifndef INS_ESKF_C_GYRO_ID
#define INS_ESKF_C_GYRO_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_C_ACCEL_ID
#define INS_ESKF_C_ACCEL_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_C_MAG_ID
#define INS_ESKF_C_MAG_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_C_GPS_ID
#define INS_ESKF_C_GPS_ID GPS_MULTI_ID
#endif
#ifndef INS_ESKF_C_AIRSPEED_ID
#define INS_ESKF_C_AIRSPEED_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_C_BARO_ID
#define INS_ESKF_C_BARO_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_C_INCIDENCE_ID
#define INS_ESKF_C_INCIDENCE_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_C_AGL_ID
#define INS_ESKF_C_AGL_ID ABI_BROADCAST
#endif

/**
 * @name ABI Callbacks
 * @brief Handlers for asynchronous sensory data parsed from ABI messages.
 * @{
 */
/**
 * @brief Gyroscope ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param gyro Raw gyroscope data
 */
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro)
{
  (void)sender_id; /* unused in basic filter */
  static uint32_t last_stamp = 0;
  if (last_stamp > 0) {
    ekf2_c_state.gyro_dt = stamp - last_stamp;
  } else {
    ekf2_c_state.gyro_dt = 10000; // default 10ms initial
  }
  last_stamp = stamp;
  
  RATES_FLOAT_OF_BFP(ekf2_c_state.delta_gyro, *gyro);
  ekf2_c_state.gyro_valid = true;
}

/**
 * @brief Accelerometer ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param accel Raw accel data
 */
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel)
{
  (void)sender_id; /* unused in basic filter */
  static uint32_t last_stamp = 0;
  if (last_stamp > 0) {
    ekf2_c_state.accel_dt = stamp - last_stamp;
  } else {
    ekf2_c_state.accel_dt = 10000;
  }
  last_stamp = stamp;

  ACCELS_FLOAT_OF_BFP(ekf2_c_state.delta_accel, *accel);
  ekf2_c_state.accel_valid = true;
  ekf2_c_state.got_imu_data = ekf2_c_state.gyro_valid && ekf2_c_state.accel_valid;
}

/**
 * @brief Magnetometer ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param mag Raw mag data
 */
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag)
{
  (void)sender_id;
  (void)stamp;
  MAGS_FLOAT_OF_BFP(ekf2_c_state.mag, *mag);
  ekf2_c_state.mag_valid = true;
  ins_ekf2_c_mea_mag();
}

/**
 * @brief GPS ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param gps_s Standard GPS struct
 */
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s)
{
  (void)sender_id; (void)stamp;
  if (gps_s->fix >= GPS_FIX_3D) {
    /* Populate pos from GPS ned_vel / hmsl / etc and run measure */
    struct FloatVect3 pos_meas;
    pos_meas.x = gps_s->ecef_pos.x / 100.0f; /* cm to m */
    pos_meas.y = gps_s->ecef_pos.y / 100.0f;
    pos_meas.z = gps_s->ecef_pos.z / 100.0f;
    struct FloatVect3 pos_noise = {ins_eskf_c_gps_p_noise, ins_eskf_c_gps_p_noise, ins_eskf_c_gps_p_noise};
    ins_ekf2_c_mea_pos(&pos_meas, &pos_noise);
  }
}

/**
 * @brief Airspeed ABI callback.
 * @param sender_id ABI sender ID
 * @param airspeed Calculated airspeed float
 */
static void airspeed_cb(uint8_t sender_id, float airspeed)
{
  (void)sender_id;
  ekf2_c_state.airspeed = airspeed;
  ekf2_c_state.airspeed_valid = true;
  ins_ekf2_c_mea_airspeed(airspeed, 1.0f);
}

/**
 * @brief Barometer ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param pressure Pressure derived altitude/float
 */
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure)
{
  (void)sender_id; (void)stamp;
  static float baro_qfe = 0.0f;
  if (pressure <= 0.0f) return;
  if (baro_qfe == 0.0f) baro_qfe = pressure; // Zero ground upon boot

  float alt = pprz_isa_height_of_pressure(pressure, baro_qfe);
  
  ekf2_c_state.baro_valid = true;
  // Note: NED Z is Down, meaning positive upward altitude is strongly negative Z.
  ins_ekf2_c_mea_baro(-alt, 2.0f); 
}

/**
 * @brief Incidence / Sideslip ABI callback.
 * @param sender_id ABI sender ID
 * @param flag Quality/availability flag
 * @param aoa Derived Angle of Attack
 * @param sideslip Derived Sideslip Angle
 */
static void incidence_cb(uint8_t sender_id, uint8_t flag, float aoa, float sideslip)
{
  (void)sender_id; (void)flag; (void)aoa;
  /* Convert sideslip angle flag if valid */
  ekf2_c_state.sideslip_valid = true;
  // Assume sideslip noise around 0.1 rad
  ins_ekf2_c_mea_sideslip(sideslip, 0.1f);
}

/**
 * @brief Rangefinder / AGL ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param distance Distance scalar facing down
 */
static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance)
{
  (void)sender_id; (void)stamp;
  /* Feed AGL distance into the solution */
  ekf2_c_state.agl = distance;
  ekf2_c_state.agl_valid = true;
  ins_ekf2_c_mea_agl(distance, 0.5f); // 0.5m noise std dev approximation 
}

/**
 * @brief Geomagnetic Field ABI callback.
 * Updates the static generic Earth magnetic field reference if the geo_mag.xml module runs.
 * @param sender_id ABI sender ID
 * @param h Calculated Geomagnetic vector from geo_mag module based on current GPS loc
 */
static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  float n = float_vect3_norm(h);
  if (n > 0.01f) {
    mag_earth_ref.x = h->x / n;
    mag_earth_ref.y = h->y / n;
    mag_earth_ref.z = h->z / n;
  }
}

/** @} */

/**
 * Filter dimensions:
 * 0-2: Attitude error (delta rotation vector)
 * 3-5: Velocity error
 * 6-8: Position error
 * 9-11: Gyroscope bias error
 * 12-14: Accelerometer bias error
 */
#define EKF_N 15

// State error covariance matrix (P) and Process Noise covariance (Q)
static float P[EKF_N][EKF_N];
static float Q[EKF_N];

// Constants
static const float g_earth = 9.81f; // Standard local gravity

// Magic generic earth magnetic field vector (normalized approximation)
// In a real application, this should be initialized based on GPS location.
 

/* ------------------------------------------------------------------------- *
 * Helper Matrix Operations 
 * Fixed 15x15 sizes to avoid dynamic allocation overhead and improve speed.
 * ------------------------------------------------------------------------- */

/**
 * @brief Multiply two 15x15 matrices: C = A * B
 * @param C Output result matrix
 * @param A Left operand matrix
 * @param B Right operand matrix
 */
static void mat_mult_15x15(float C[15][15], const float A[15][15], const float B[15][15]) {
    for (int i=0; i<15; i++) {
        for (int j=0; j<15; j++) {
            C[i][j] = 0.0f;
        }
    }
    for (int i=0; i<15; i++) {
        for (int k=0; k<15; k++) {
            float a_ik = A[i][k];
            if (a_ik != 0.0f) {
                for (int j=0; j<15; j++) {
                    C[i][j] += a_ik * B[k][j];
                }
            }
        }
    }
}

/**
 * @brief Multiply 15x15 matrices with the second matrix transposed: C = A * B^T
 * @param C Output result matrix
 * @param A Left operand matrix
 * @param B Right operand matrix (will be multiplied as B^T)
 */
static void mat_mult_15x15_transB(float C[15][15], const float A[15][15], const float B[15][15]) {
    for (int i=0; i<15; i++) {
        for (int j=0; j<15; j++) {
            C[i][j] = 0.0f;
        }
    }
    for (int j=0; j<15; j++) {
        for (int k=0; k<15; k++) {
            float b_jk = B[j][k];
            if (b_jk != 0.0f) {
                for (int i=0; i<15; i++) {
                    C[i][j] += A[i][k] * b_jk;
                }
            }
        }
    }
}

/**
 * @brief Initializes the ESKF.
 * Sets the initial nominal state, prediction covariances (P), and process noise (Q).
 */
void ins_ekf2_c_init(void) {
    struct ekf2_c_t zero_state = {0};
    ekf2_c_state = zero_state;
    
    // Initialize standard kinematics to zero/identity
    float_quat_identity(&ekf2_c_state.quat);
    FLOAT_VECT3_ZERO(ekf2_c_state.vel);
    FLOAT_VECT3_ZERO(ekf2_c_state.pos);
    FLOAT_RATES_ZERO(ekf2_c_state.gyro_bias);
    FLOAT_VECT3_ZERO(ekf2_c_state.accel_bias);

    // Init state covariance matrix (P) with sensible initial uncertainty bounds
    for(int i=0; i<15; i++) { for(int j=0; j<15; j++) { P[i][j] = 0.0f; } }
    for (int i=0; i<3; i++) P[i][i] = 0.01f;     // Attitude uncertainty
    for (int i=3; i<6; i++) P[i][i] = 1.0f;      // Velocity uncertainty
    for (int i=6; i<9; i++) P[i][i] = 1.0f;      // Position uncertainty
    for (int i=9; i<12; i++) P[i][i] = 0.0001f;  // Gyro Bias uncertainty
    for (int i=12; i<15; i++) P[i][i] = 0.01f;   // Accel Bias uncertainty
    
    // Init Process noise (Q) diagonals. 
    // Represents confidence in our mathematical model vs sensor integration noise.
    for (int i=0; i<3; i++) Q[i] = 1e-4f;        // Attitude noise
    for (int i=3; i<6; i++) Q[i] = 1e-3f;        // Velocity noise
    for (int i=6; i<9; i++) Q[i] = 1e-5f;        // Position noise
    for (int i=9; i<12; i++) Q[i] = 1e-7f;       // Gyro bias wander
    for (int i=12; i<15; i++) Q[i] = 1e-5f;      // Accel bias wander

    /* Binding ABI messages */
    AbiBindMsgIMU_GYRO(INS_ESKF_C_GYRO_ID, &ekf2_c_state.gyro_ev, gyro_cb);
    AbiBindMsgIMU_ACCEL(INS_ESKF_C_ACCEL_ID, &ekf2_c_state.accel_ev, accel_cb);
    AbiBindMsgIMU_MAG(INS_ESKF_C_MAG_ID, &ekf2_c_state.mag_ev, mag_cb);
    AbiBindMsgGPS(INS_ESKF_C_GPS_ID, &ekf2_c_state.gps_ev, gps_cb);
    AbiBindMsgAIRSPEED(INS_ESKF_C_AIRSPEED_ID, &ekf2_c_state.airspeed_ev, airspeed_cb);
    AbiBindMsgBARO_ABS(INS_ESKF_C_BARO_ID, &ekf2_c_state.baro_ev, baro_cb);
    AbiBindMsgINCIDENCE(INS_ESKF_C_INCIDENCE_ID, &ekf2_c_state.incidence_ev, incidence_cb);
    AbiBindMsgAGL(INS_ESKF_C_AGL_ID, &ekf2_c_state.agl_ev, agl_cb);
    AbiBindMsgGEO_MAG(ABI_BROADCAST, &ekf2_c_state.geo_mag_ev, geo_mag_cb);
}

/**
 * @brief Normalizes a quaternion to ensure it represents a valid rotation.
 * @param q Pointer to the quaternion to normalize
 * Optimized for systems without an FPU by calculating the inverse once.
 */
static void normalize_quat(struct FloatQuat *q) {
    float n = sqrtf(q->qi*q->qi + q->qx*q->qx + q->qy*q->qy + q->qz*q->qz);
    if (n > 1e-7f) {
        float inv_n = 1.0f / n; // One division, multiplicative application is faster
        q->qi *= inv_n; q->qx *= inv_n; q->qy *= inv_n; q->qz *= inv_n;
    } else {
        /* Fallback if totally degraded to prevent NaN/DivZero explosion */
        q->qi = 1.0f; q->qx = 0.0f; q->qy = 0.0f; q->qz = 0.0f;
    }
}

/**
 * @brief Skew-symmetric matrix generator.
 * @param m 3x3 output matrix
 * @param v 3D input vector
 * Converts a 3D vector [x, y, z] into a 3x3 anti-symmetric cross-product matrix.
 */
static void skew_symmetric(float m[3][3], const struct FloatVect3 *v) {
    m[0][0]=0;      m[0][1]=-v->z;  m[0][2]=v->y;
    m[1][0]=v->z;   m[1][1]=0;      m[1][2]=-v->x;
    m[2][0]=-v->y;  m[2][1]=v->x;   m[2][2]=0;
}

/**
 * @brief Core Periodic Update (Predict Step).
 * Should be called whenever new IMU measurements (Gyro & Accel) are ready.
 * Integrates the IMU readings to update the nominal state (Position, Velocity, Attitude)
 * and propagates the state error covariance matrix (P) forward in time.
 */
void ins_ekf2_c_update(void) {
    if (!ekf2_c_state.gyro_valid || !ekf2_c_state.accel_valid) return;
    
    // dt measured in seconds
    float dt = (float)ekf2_c_state.gyro_dt * 0.000001f;
    
    /* SAFEGUARD: Limit maximum prediction step and protect against negative time */
    if (dt <= 0.0f) {
        ekf2_c_state.gyro_valid = false;
        ekf2_c_state.accel_valid = false;
        return;
    }
    if (dt > 0.2f) dt = 0.2f; // Assuming 5Hz is lowest practical IMU frequency
    
    // 1. Subtract estimated biases from raw IMU readings
    struct FloatRates omega;
    omega.p = ekf2_c_state.delta_gyro.p - ekf2_c_state.gyro_bias.p;
    omega.q = ekf2_c_state.delta_gyro.q - ekf2_c_state.gyro_bias.q;
    omega.r = ekf2_c_state.delta_gyro.r - ekf2_c_state.gyro_bias.r;
    
    struct FloatVect3 acc;
    acc.x = ekf2_c_state.delta_accel.x - ekf2_c_state.accel_bias.x;
    acc.y = ekf2_c_state.delta_accel.y - ekf2_c_state.accel_bias.y;
    acc.z = ekf2_c_state.delta_accel.z - ekf2_c_state.accel_bias.z;

    /* --------------------------------------------------------------------- *
     * NOMINAL STATE UPDATE (Kinematics Integration)
     * --------------------------------------------------------------------- */
    
    // Integrate Attitude (Quaternion)
    // q_new = q_old + 0.5 * q \otimes [0, omega] * dt
    float dt_half = 0.5f * dt;
    struct FloatQuat dq;
    dq.qi = (-ekf2_c_state.quat.qx*omega.p - ekf2_c_state.quat.qy*omega.q - ekf2_c_state.quat.qz*omega.r) * dt_half;
    dq.qx = ( ekf2_c_state.quat.qi*omega.p + ekf2_c_state.quat.qy*omega.r - ekf2_c_state.quat.qz*omega.q) * dt_half;
    dq.qy = ( ekf2_c_state.quat.qi*omega.q - ekf2_c_state.quat.qx*omega.r + ekf2_c_state.quat.qz*omega.p) * dt_half;
    dq.qz = ( ekf2_c_state.quat.qi*omega.r + ekf2_c_state.quat.qx*omega.q - ekf2_c_state.quat.qy*omega.p) * dt_half;

    // First order Euler integration
    ekf2_c_state.quat.qi += dq.qi;
    ekf2_c_state.quat.qx += dq.qx;
    ekf2_c_state.quat.qy += dq.qy;
    ekf2_c_state.quat.qz += dq.qz;
    normalize_quat(&ekf2_c_state.quat); // Must remain normalized

    // Get rotational matrix C (Body to NED) from our updated quaternion
    struct FloatRMat C;
    float_rmat_of_quat(&C, &ekf2_c_state.quat);
    
    // Transform specific force into Navigation frame (NED) and subtract gravity
    struct FloatVect3 acc_ned;
    float_rmat_vmult(&acc_ned, &C, &acc);
    acc_ned.z += g_earth; // Gravity down, so we add to neutralize constant upward specific force
    
    // Integrate Velocity and Position
    ekf2_c_state.vel.x += acc_ned.x * dt;
    ekf2_c_state.vel.y += acc_ned.y * dt;
    ekf2_c_state.vel.z += acc_ned.z * dt;
    
    ekf2_c_state.pos.x += ekf2_c_state.vel.x * dt;
    ekf2_c_state.pos.y += ekf2_c_state.vel.y * dt;
    ekf2_c_state.pos.z += ekf2_c_state.vel.z * dt;

    /* --------------------------------------------------------------------- *
     * ERROR STATE COVARIANCE UPDATE (F = Jacobian of Error Dynamics)
     * --------------------------------------------------------------------- */
    
    // F is the Error-State Transition Matrix: F = I + f_cont * dt
    float F[EKF_N][EKF_N] = {0};
    
    for(int i=0; i<EKF_N; i++) F[i][i] = 1.0f; // Start with Identity
    
    // Error Jacobian Blocks:
    // Attitude error wrt Attitude (delta_theta_dot = -skew(omega) * delta_theta)
    struct FloatVect3 v_om = {omega.p, omega.q, omega.r};
    float omega_skew[3][3];
    skew_symmetric(omega_skew, &v_om);
    for(int i=0; i<3; i++) for(int j=0; j<3; j++) F[0+i][0+j] -= omega_skew[i][j] * dt;
    
    // Attitude error wrt Gyro Bias (delta_theta_dot = -delta_bg)
    for(int i=0; i<3; i++) F[0+i][9+i] = -dt;
    
    // Velocity error wrt Attitude (delta_v_dot = -C * skew(acc) * delta_theta)
    float acc_skew[3][3];
    skew_symmetric(acc_skew, &acc);
    float C_acc_skew[3][3];
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            C_acc_skew[i][j] = C.m[i*3+0]*acc_skew[0][j] + C.m[i*3+1]*acc_skew[1][j] + C.m[i*3+2]*acc_skew[2][j];
            F[3+i][0+j] = -C_acc_skew[i][j] * dt;
        }
    }
    
    // Velocity error wrt Accel Bias (delta_v_dot = -C * delta_ba)
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            F[3+i][12+j] = -C.m[i*3+j] * dt;
        }
    }
    
    // Position error wrt Velocity (delta_p_dot = delta_v)
    for(int i=0; i<3; i++) F[6+i][3+i] = dt;

    // Propagate Covariance: P_new = F * P_old * F^T + Q
    float F_P[EKF_N][EKF_N];
    mat_mult_15x15(F_P, F, P);   // F * P
    
    float F_P_FT[EKF_N][EKF_N];
    mat_mult_15x15_transB(F_P_FT, F_P, F); // (F * P) * F^T
    
    // Save to P and add process noise Q
    for(int i=0; i<EKF_N; i++) {
        for(int j=0; j<EKF_N; j++) {
            P[i][j] = F_P_FT[i][j];
            if(i==j) {
                P[i][j] += Q[i]*dt; // Add process noise only on diagonals
                /* SAFEGUARD: Prevent Diagonal Variances from dropping below zero due to float precision */
                if(P[i][i] < 1e-9f) P[i][i] = 1e-9f; 
            }
        }
    }

    // Reset IMU availability flags for the next run
    ekf2_c_state.gyro_valid = false;
    ekf2_c_state.accel_valid = false;

    /* SAFEGUARD: NaN Infestation Check */
    if (isnan(ekf2_c_state.pos.x) || isnan(ekf2_c_state.quat.qi) || isnan(ekf2_c_state.vel.x)) {
        ins_ekf2_c_init(); // Exploded, re-init.
        return;
    }

    // Output mapped states to the generic Paparazzi framework
    struct NedCoor_f ned_pos, ned_vel;
    ned_pos.x = ekf2_c_state.pos.x; ned_pos.y = ekf2_c_state.pos.y; ned_pos.z = ekf2_c_state.pos.z;
    ned_vel.x = ekf2_c_state.vel.x; ned_vel.y = ekf2_c_state.vel.y; ned_vel.z = ekf2_c_state.vel.z;
    
    stateSetPositionNed_f(0, &ned_pos);
    stateSetSpeedNed_f(0, &ned_vel);
    stateSetNedToBodyQuat_f(0, &ekf2_c_state.quat);
}

/**
 * @brief Analytically computes the inverse of a 3x3 matrix using the determinant.
 * @param minv 3x3 inverted output matrix
 * @param m 3x3 input matrix
 * @return True if successful, False if matrix is singular
 * Fails safely (returns false) if the determinant is practically zero (singular).
 */
static bool invert_3x3(float minv[3][3], const float m[3][3]) {
    float det = m[0][0]*(m[1][1]*m[2][2] - m[2][1]*m[1][2]) -
                m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0]) +
                m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
    
    if (fabsf(det) < 1e-6f) return false; // Matrix is non-invertible
    
    float invdet = 1.0f / det; // Singular division operation for performance
    
    // Compute the adjugate / determinant
    minv[0][0] = (m[1][1]*m[2][2] - m[2][1]*m[1][2]) * invdet;
    minv[0][1] = (m[0][2]*m[2][1] - m[0][1]*m[2][2]) * invdet;
    minv[0][2] = (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * invdet;
    minv[1][0] = (m[1][2]*m[2][0] - m[1][0]*m[2][2]) * invdet;
    minv[1][1] = (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * invdet;
    minv[1][2] = (m[1][0]*m[0][2] - m[0][0]*m[1][2]) * invdet;
    minv[2][0] = (m[1][0]*m[2][1] - m[2][0]*m[1][1]) * invdet;
    minv[2][1] = (m[2][0]*m[0][1] - m[0][0]*m[2][1]) * invdet;
    minv[2][2] = (m[0][0]*m[1][1] - m[1][0]*m[0][1]) * invdet;
    return true;
}

/**
 * @brief State Injection.
 * @param err_X Float array of error states representing delta corrections
 * Modifies the nominal states based on the calculated error state dx.
 */
static void apply_error_state(const float err_X[EKF_N]) {
    struct FloatVect3 delta_or = {err_X[0], err_X[1], err_X[2]};
    struct FloatQuat dq;
    
    // Convert angle error into quaternion error term (small angle approximation)
    // dq = [1, 0.5 * delta_or]
    dq.qi = 1.0f;
    dq.qx = 0.5f * delta_or.x;
    dq.qy = 0.5f * delta_or.y;
    dq.qz = 0.5f * delta_or.z;
    normalize_quat(&dq);
    
    // Inject attitude error (q_new = q_old * dq)
    struct FloatQuat q_new;
    float_quat_comp(&q_new, &ekf2_c_state.quat, &dq);
    ekf2_c_state.quat = q_new;
    normalize_quat(&ekf2_c_state.quat);

    // Directly apply linear error to Velocity & Position
    ekf2_c_state.vel.x += err_X[3];
    ekf2_c_state.vel.y += err_X[4];
    ekf2_c_state.vel.z += err_X[5];
    
    ekf2_c_state.pos.x += err_X[6];
    ekf2_c_state.pos.y += err_X[7];
    ekf2_c_state.pos.z += err_X[8];
    
    // Update estimated biases
    ekf2_c_state.gyro_bias.p += err_X[9];
    ekf2_c_state.gyro_bias.q += err_X[10];
    ekf2_c_state.gyro_bias.r += err_X[11];
    
    ekf2_c_state.accel_bias.x += err_X[12];
    ekf2_c_state.accel_bias.y += err_X[13];
    ekf2_c_state.accel_bias.z += err_X[14];
}

/**
 * @brief Standard Multi-dimensional ESKF Kalman Update.
 * Computes the Kalman Gain and applies the calculated error to both the State and Covariance.
 * Uses the mathematically stable "Joseph Form" covariance update: P = (I - KH) P.
 * 
 * @param H Observability Matrix relating measurement to state error.
 * @param R Measurement Noise Matrix.
 * @param z Measurement Innovation/Residual (Actual measurement - Expected measurement).
 */
static void eskf_update_3d(float H[3][EKF_N], float R[3][3], float z[3]) {
    
    // Calculate Innovation Covariance S: S = H * P * H^T + R
    float HP[3][EKF_N] = {0}; // Stores result of (H * P)
    for(int i=0; i<3; i++) {
        for(int k=0; k<EKF_N; k++) {
            float H_ik = H[i][k];
            if (H_ik != 0.0f) { // Accelerate sparse observability multiplication
                for(int j=0; j<EKF_N; j++) {
                    HP[i][j] += H_ik * P[k][j];
                }
            }
        }
    }
    
    float S[3][3]; // Stores result of (HP * H^T + R)
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            S[i][j] = R[i][j]; // Add R
            // Multiply HP by H^T (which is just multiplying by H with swapped indices)
            for(int k=0; k<EKF_N; k++) {
                if (H[j][k] != 0.0f) {
                    S[i][j] += HP[i][k] * H[j][k];
                }
            }
        }
    }
    
    // Invert the Innovation Covariance matrix
    float S_inv[3][3];
    if (!invert_3x3(S_inv, S)) return; // If inversion fails due to singularity, drop update
    
    // Calculate Kalman Gain K: K = P * H^T * S_inv
    float P_HT[EKF_N][3] = {0}; // Stores (P * H^T)
    for(int j=0; j<3; j++) {
        for(int k=0; k<EKF_N; k++) {
            float H_jk = H[j][k];
            if (H_jk != 0.0f) {
                for(int i=0; i<EKF_N; i++) {
                    P_HT[i][j] += P[i][k] * H_jk;
                }
            }
        }
    }
    
    float K[EKF_N][3]; // Stores final Kalman Gain
    for(int i=0; i<EKF_N; i++) {
        for(int j=0; j<3; j++) {
            K[i][j] = 0.0f;
            for(int k=0; k<3; k++) K[i][j] += P_HT[i][k] * S_inv[k][j];
        }
    }
    
    // Calculate Error State vector: dx = K * z
    float dx[EKF_N] = {0};
    for(int i=0; i<EKF_N; i++) {
        for(int j=0; j<3; j++) {
            dx[i] += K[i][j] * z[j];
        }
    }
    
    // Inject computed error state into Nominal States
    apply_error_state(dx);
    
    // Update Covariance Matrix: P = P - K * (H * P)
    // We already computed HP = H * P earlier.
    // Update Covariance Matrix symmetrically
    for(int i=0; i<EKF_N; i++) {
        for(int j=i; j<EKF_N; j++) {
            float K_HP_ij = 0.0f;
            float K_HP_ji = 0.0f;
            for(int k=0; k<3; k++) {
                K_HP_ij += K[i][k] * HP[k][j];
                K_HP_ji += K[j][k] * HP[k][i];
            }
            float sym_update = P[i][j] - 0.5f * (K_HP_ij + K_HP_ji);
            P[i][j] = sym_update;
            P[j][i] = sym_update;
        }
        /* SAFEGUARD: Variance strict positivity */
        if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;
    }
}

/**
 * @brief 3D Position Measurement Update.
 * @param pos_meas Measured position float vector
 * @param pos_noise Float vector representing measurement covariance noise
 * Often fed by GPS or visual odometry. Provides observability into NED Position
 * vectors driving the internal biases into convergence.
 */
void ins_ekf2_c_mea_pos(struct FloatVect3 *pos_meas, struct FloatVect3 *pos_noise) {
    float H[3][EKF_N] = {0};
    
    // Jacobian mapping states to expected measurements.
    // Positions map 1:1 on indexes 6, 7, 8 in our Error State
    H[0][6] = 1.0f; 
    H[1][7] = 1.0f; 
    H[2][8] = 1.0f; 
    
    // Fill the Measurement Noise covariance (R) from dynamic config
    float R[3][3] = {0};
    R[0][0] = pos_noise->x; 
    R[1][1] = pos_noise->y; 
    R[2][2] = pos_noise->z;
    
    // Calculate Innovation / Residual (difference between measured & expected Pos)
    float z[3];
    z[0] = pos_meas->x - ekf2_c_state.pos.x;
    z[1] = pos_meas->y - ekf2_c_state.pos.y;
    z[2] = pos_meas->z - ekf2_c_state.pos.z;
    
    eskf_update_3d(H, R, z);
}

/**
 * @brief 3D Magnetometer Measurement Update.
 * Connects Earth's magnetic forces to our internal State. Heavily relies
 * on Attitude error states since compass rotations provide direct attitude constraints.
 */
void ins_ekf2_c_mea_mag(void) {
    if(!ekf2_c_state.mag_valid) return;
    
    float H[3][EKF_N] = {0};
    
    // Retrieve the Rotation Matrix C (Body into Frame)
    struct FloatRMat C;
    float_rmat_of_quat(&C, &ekf2_c_state.quat);
    
    // Project the expected mathematical Mag Earth Reference back into Body Frame
    // Expected Measurement: m_hat = C^T * m_earth
    struct FloatVect3 m_hat;
    m_hat.x = C.m[0]*mag_earth_ref.x + C.m[3]*mag_earth_ref.y + C.m[6]*mag_earth_ref.z;
    m_hat.y = C.m[1]*mag_earth_ref.x + C.m[4]*mag_earth_ref.y + C.m[7]*mag_earth_ref.z;
    m_hat.z = C.m[2]*mag_earth_ref.x + C.m[5]*mag_earth_ref.y + C.m[8]*mag_earth_ref.z;
    
    float m_hat_skew[3][3];
    skew_symmetric(m_hat_skew, &m_hat);
    
    // Fill the H matrix. The magnetometer primarily corrects Attitude Error
    // H(mag, attitude) = m_hat_skew
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            H[i][j] = m_hat_skew[i][j];
        }
    }
    
    // Magnetometer noise characteristics (Hardcoded approx for compass)
    float R[3][3] = {0};
    R[0][0] = 0.05f; R[1][1] = 0.05f; R[2][2] = 0.05f;
    
    // Computes measurement residuals: z = actual measurements - expected measurements 
    float z[3];
    z[0] = ekf2_c_state.mag.x - m_hat.x;
    z[1] = ekf2_c_state.mag.y - m_hat.y;
    z[2] = ekf2_c_state.mag.z - m_hat.z;
    
    eskf_update_3d(H, R, z);
    
    // Flag to wait for next reading
    ekf2_c_state.mag_valid = false;
}

/**
 * @brief 1D ESKF Kalman Update.
 * Used for scalar measurements like Airspeed or Barometer Altitude.
 * 
 * @param H Observability Vector (1xN).
 * @param R Measurement Noise Variance (scalar).
 * @param z Measurement Innovation/Residual (scalar).
 */
static void eskf_update_1d(const float H[EKF_N], float R, float z) {
    // Calculate Innovation Variance S: S = H * P * H^T + R
    float HP[EKF_N] = {0}; // Stores result of (H * P)
    for(int k=0; k<EKF_N; k++) {
        float h_k = H[k];
        if (h_k != 0.0f) { // Accelerate sparse observability multiplication
            for(int j=0; j<EKF_N; j++) {
                HP[j] += h_k * P[k][j];
            }
        }
    }
    
    float S = R;
    for(int j=0; j<EKF_N; j++) {
        if (H[j] != 0.0f) {
            S += HP[j] * H[j];
        }
    }
    
    // Invert Innovation Variance
    if (S < 1e-6f) return;
    float S_inv = 1.0f / S;
    
    // Calculate Kalman Gain K: K = P * H^T * S_inv
    float K[EKF_N] = {0};
    for(int k=0; k<EKF_N; k++) {
        float h_k = H[k];
        if (h_k != 0.0f) {
            for(int i=0; i<EKF_N; i++) {
                K[i] += P[i][k] * h_k;
            }
        }
    }
    for(int i=0; i<EKF_N; i++) {
        K[i] *= S_inv;
    }

    // Calculate Error State vector: dx = K * z
    float dx[EKF_N];
    for(int i=0; i<EKF_N; i++) {
        dx[i] = K[i] * z;
    }
    
    // Inject computed error state into Nominal States
    apply_error_state(dx);
    
    // Update Covariance Matrix: P = (I - K * H) * P = P - K * (H * P)
    // Ensure symmetry directly (P = (P + P^T)/2)
    for(int i=0; i<EKF_N; i++) {
        for(int j=i; j<EKF_N; j++) {
            float sym = P[i][j] - 0.5f * (K[i] * HP[j] + K[j] * HP[i]);
            P[i][j] = sym;
            P[j][i] = sym;
        }
        /* SAFEGUARD: Variance strict positivity */
        if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;
    }
}

/**
 * @brief 1D Airspeed Measurement Update.
 * Uses a forward-facing Pitot tube reading to correct velocity and attitude.
 * Assumes the aircraft is flying with strictly forward velocity in its body frame
 * (minimal side-slip) and zero wind (or treats wind as sensor noise).
 * 
 * @param airspeed_meas True forward airspeed measurement from sensor.
 * @param airspeed_noise Uncertainty/Variance in the airspeed reading.
 */
void ins_ekf2_c_mea_airspeed(float airspeed_meas, float airspeed_noise) {
    if(!ekf2_c_state.airspeed_valid) return;
    
    float H[EKF_N] = {0};
    
    // Retrieve Rotation Matrix C (Body to NED) -> C^T is NED to Body
    struct FloatRMat C;
    float_rmat_of_quat(&C, &ekf2_c_state.quat);
    
    // Expected forward airspeed in Body Frame (v_x_body)
    // v^B = C^T * v^N
    // v_x^B = C_00 * v_N + C_10 * v_E + C_20 * v_D
    float v_x_body = C.m[0]*ekf2_c_state.vel.x + C.m[3]*ekf2_c_state.vel.y + C.m[6]*ekf2_c_state.vel.z;
    float v_y_body = C.m[1]*ekf2_c_state.vel.x + C.m[4]*ekf2_c_state.vel.y + C.m[7]*ekf2_c_state.vel.z;
    float v_z_body = C.m[2]*ekf2_c_state.vel.x + C.m[5]*ekf2_c_state.vel.y + C.m[8]*ekf2_c_state.vel.z;
    
    // Jacobian wrt Velocity (indexes 3, 4, 5)
    H[3] = C.m[0];
    H[4] = C.m[3];
    H[5] = C.m[6];
    
    // Jacobian wrt Attitude (indexes 0, 1, 2)
    // Derivative of v_x^B wrt delta_theta is the first row of skew(v^B)
    // skew(v^B) row 0 = [0, -v_z^B, v_y^B]
    H[0] = 0.0f;
    H[1] = -v_z_body;
    H[2] = v_y_body;
    
    // Innovation: Measured Airspeed - Expected Forward Velocity
    float z = airspeed_meas - v_x_body;
    
    // Perform Kalman update
    eskf_update_1d(H, airspeed_noise, z);
    
    // Reset flag for next reading
    ekf2_c_state.airspeed_valid = false;
}

/**
 * @brief 1D Barometer Altitude Measurement Update.
 * @param baro_alt_meas Altitude measurement from barometric sensors
 * @param baro_alt_noise Measurement uncertainty/variance
 */
void ins_ekf2_c_mea_baro(float baro_alt_meas, float baro_alt_noise) {
  if(!ekf2_c_state.baro_valid) return;

  ekf2_c_state.baro_alt = baro_alt_meas;

  float H[EKF_N] = {0};
  // Maps to global Z Position (NED downward altitude: index 8)
  H[8] = 1.0f;

  float z = baro_alt_meas - ekf2_c_state.pos.z;
  eskf_update_1d(H, baro_alt_noise * baro_alt_noise, z);
  ekf2_c_state.baro_valid = false;
}

/**
 * @brief 1D Rangefinder / AGL Measurement Update.
 * @param agl_meas Distance measurement facing downwards to terrain
 * @param agl_noise Measurement uncertainty/variance
 */
void ins_ekf2_c_mea_agl(float agl_meas, float agl_noise) {
  if(!ekf2_c_state.agl_valid) return;
  
  // Transform rangefinder distance pointing down (Body Z axis) to Earth Z axis (Alt)
  // Range * cos(pitch) * cos(roll) gives delta Altitude in generic approximation,
  // more accurately mathematically derived from C_33 (Rotation Matrix Z dot Z)
  struct FloatRMat C;
  float_rmat_of_quat(&C, &ekf2_c_state.quat);
  float z_measured = -agl_meas * C.m[8]; // Negative because ground is down, and AGL is positive distance.

  float H[EKF_N] = {0};
  // Maps to global Z Position (NED downward altitude: index 8)
  H[8] = 1.0f;

  float z_innovation = z_measured - ekf2_c_state.pos.z;
  eskf_update_1d(H, agl_noise * agl_noise, z_innovation);
  ekf2_c_state.agl_valid = false;
}

/**
 * @brief Fuses sideslip angle measurement.
 * @param sideslip_meas Measurement angle in radians
 * @param sideslip_noise Estimate variance for sideslip constraint
 * Sideslip (beta) relates to the lateral body velocity (v_y^B).
 * v_y^B = V_airspeed * sin(beta) ~ V_airspeed * beta.
 * Often, this is used as a synthetic "zero sideslip" measurement (beta=0) 
 * to correct lateral estimation and heading when flying forward.
 */
void ins_ekf2_c_mea_sideslip(float sideslip_meas, float sideslip_noise) {
  /* Current attitude matrix C: Body to NED */
  struct FloatRMat C;
  float_rmat_of_quat(&C, &ekf2_c_state.quat);
  
  /* Map NED velocity to Body Frame */
  float v_bx = C.m[0]*ekf2_c_state.vel.x + C.m[3]*ekf2_c_state.vel.y + C.m[6]*ekf2_c_state.vel.z;
  float v_by = C.m[1]*ekf2_c_state.vel.x + C.m[4]*ekf2_c_state.vel.y + C.m[7]*ekf2_c_state.vel.z;
  float v_bz = C.m[2]*ekf2_c_state.vel.x + C.m[5]*ekf2_c_state.vel.y + C.m[8]*ekf2_c_state.vel.z;

  /* Determine total airspeed estimate to scale sideslip angle appropriately */
  float V_est = sqrtf(v_bx*v_bx + v_by*v_by + v_bz*v_bz);
  if (V_est < 1.0f) return; // Discard fusion at near-zero speeds to avoid singularity

  /* Estimated sideslip observation y_est = v_by */
  float y_est = v_by;
  float y_meas = V_est * sinf(sideslip_meas);

  /* Jacobian H for v_by. H_theta = [-v_bz, 0, v_bx]. H_vel = C^T row 1 (Y axis) */
  float H[EKF_N] = {0};
  H[0] = -v_bz;
  H[2] = v_bx;
  H[3] = C.m[1]; H[4] = C.m[4]; H[5] = C.m[7];

  float R = (V_est * sideslip_noise) * (V_est * sideslip_noise);
  eskf_update_1d(H, R, y_meas - y_est);
}
