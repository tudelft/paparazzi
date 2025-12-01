#include "firmwares/rotorcraft/stabilization/stabilization_andi.h"
#include<math.h>

#if ANDI_NUM_ACT != 4
#error Cyclone expects 4 actuators
#endif

#if ANDI_OUTPUTS != 4
#error The cyclone model provides 4 outputs
#endif

union CycloneCoefficients {
  struct {
    float ce_11;
    float ce_12;
    float ce_13;
    float ce_14;
    float ce_21;
    float ce_22;
    float ce_23;
    float ce_24;
    float ce_31;
    float ce_32;
    float ce_33;
    float ce_34;
    float ce_41;
    float ce_42;
    float ce_43;
    float ce_44;
    
  };
  float data[16];
}; 

// Model coefficinets
union CycloneCoefficients obm_coefficients = {
  .ce_11 = 0.0f,        .ce_12 = 0.0f,        .ce_13 = 3.9e-5f,     .ce_14 = -3.9e-5f, // Roll 
  .ce_21 = -29.917439f, .ce_22 = -29.917439f, .ce_23 = 0.0f,        .ce_24 = 0.0f,     // Pitch
  .ce_31 = -19.968481f, .ce_32 = 19.968481f,  .ce_33 = 0.0f,        .ce_34 = 0.0f,     // Yaw
  .ce_41 = 0.0f,        .ce_42 = 0.0f,        .ce_43 = 7e-6f,       .ce_44 = 7e-6f,    // Thrust
};

struct FloatVect3 evaluate_obm_forces(const struct FloatRates *rates, const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT], const float actuator_state_dot[ANDI_NUM_ACT])
{
  float vel_body_array[3];
  (void)rates;
  (void)actuator_state_dot;

  vel_body_array[0] = vel_body->x;
  vel_body_array[1] = vel_body->y;
  vel_body_array[2] = vel_body->z;

  float forces_array[3];
  cyclone_obm_forces(vel_body_array, actuator_state, obm_coefficients.data, forces_array);

  struct FloatVect3 forces;
  forces.x = forces_array[0];
  forces.y = forces_array[1];
  forces.z = forces_array[2];

  return forces;
}

struct FloatVect3 evaluate_obm_moments(const struct FloatRates *rates, const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT], const float actuator_state_dot[ANDI_NUM_ACT])
{
  float vel_body_array[3];
  float rates_array[3];

  vel_body_array[0] = vel_body->x;
  vel_body_array[1] = vel_body->y;
  vel_body_array[2] = vel_body->z;

  rates_array[0] = rates->p;
  rates_array[1] = rates->q;
  rates_array[2] = rates->r;

  // Ignore actuator_state_dot for now
  (void)actuator_state_dot;
  float zeros[4];
  float_vect_zero(zeros, 4);


  float moments_array[3];
  cyclone_obm_moments(rates_array, vel_body_array, actuator_state, zeros, obm_coefficients.data, moments_array);  
  struct FloatVect3 moments;
  moments.x = moments_array[0];
  moments.y = moments_array[1];
  moments.z = moments_array[2];

  return moments;
}


void evaluate_obm_f_stb_u(float fu_mat[ANDI_NUM_ACT * ANDI_OUTPUTS], const struct FloatRates *rates, const struct FloatVect3 *vel_body, const float actuator_state[ANDI_NUM_ACT])
{
  float vel_body_array[3];
  (void)rates; 

  vel_body_array[0] = vel_body->x;
  vel_body_array[1] = vel_body->y;
  vel_body_array[2] = vel_body->z;

  // Bound min motor speed in actuator_state to prevent really low control effectiveness which may result in instabilities.
  // This should make "free fall" more stable at the cost of some model inaccuracy at very low thrust.
  // FIXME: Rethink this solution.
  // FIXME: Apply this bounding directly on the elevon effectiveness terms in the final matrix instead of modifying actuator_state.
  // FIXME: Would it be possible to dynamically identify or saturate the control effectiveness?
  float actuator_state_bounded[ANDI_NUM_ACT];
  actuator_state_bounded[0] = actuator_state[0];
  actuator_state_bounded[1] = actuator_state[1];
  actuator_state_bounded[2] = fmaxf(actuator_state[2], 360000.0f);
  actuator_state_bounded[3] = fmaxf(actuator_state[3], 360000.0f);

  cyclone_f_stb_u(vel_body_array, actuator_state_bounded, obm_coefficients.data, fu_mat);
}

void evaluate_obm_f_stb_x(float nu_obm[ANDI_OUTPUTS], const struct FloatRates *rates, const struct FloatVect3 *vel_body, const struct FloatVect3 *ang_accel, const struct FloatVect3 *accel_body, const float actuator_state[ANDI_NUM_ACT])
{
  float rates_array[3];
  float ang_accel_array[3];
  float vel_body_array[3];
  float accel_body_array[3];

  rates_array[0] = rates->p;
  rates_array[1] = rates->q;
  rates_array[2] = rates->r;

  ang_accel_array[0] = ang_accel->x;
  ang_accel_array[1] = ang_accel->y;
  ang_accel_array[2] = ang_accel->z;

  vel_body_array[0] = vel_body->x;
  vel_body_array[1] = vel_body->y;
  vel_body_array[2] = vel_body->z;

  accel_body_array[0] = accel_body->x;
  accel_body_array[1] = accel_body->y;
  accel_body_array[2] = accel_body->z;


  cyclone_f_stb_x(rates_array, vel_body_array, ang_accel_array, accel_body_array, actuator_state, obm_coefficients.data, nu_obm);
}

float evaluate_obm_thrust_z(const float actuator_state[ANDI_NUM_ACT])
{
  return obm_coefficients.fx_motor_squared * (actuator_state[2] + actuator_state[3]);
}