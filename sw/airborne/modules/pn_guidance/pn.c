#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "modules/ins/ins_int.h"
#include "state.h"
#include "modules/imu/imu.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"

struct FloatVect3 speed_des = {0.0f, 0.0f, 0.0f};
struct FloatVect3 speed_target = {0.5f, 1.0f, 0.0f};
struct FloatVect3 speed_interceptor = {0.0f, 0.0f, 0.0f};
struct FloatVect3 pos_target = {5.0f, 5.0f, 5.0f};
struct FloatVect3 accel_des = {0.0f, 0.0f, 0.0f};
struct FloatVect3 rel_pos = { 0 };
struct FloatVect3 los_rate_num = {0.0f, 0.0f, 0.0f};
struct FloatVect3 los_rate = {0.0f, 0.0f, 0.0f};
struct FloatVect3 rel_speed = {0.0f, 0.0f, 0.0f};
struct FloatVect3 accel_des_temp = {0.0f, 0.0f, 0.0f};
struct FloatVect3 speed_err = {0.0f, 0.0f, 0.0f};
struct StabilizationSetpoint quat_des;
struct Int32Quat q_des;
float speed_gain = 1.5;
float r_los = 0.0;
float v_mag = 0.0;
float los_rate_denom = 0.0;
int N_pn = -3;
float speed_mag = 0.0;
float heading_cmd = 0.0;
bool in_flight;
bool interception = false;
// Initialize a pointer to a FloatVect3 struct and assign the address of 'vector'
//struct FloatVect3 *acceleration = &accel_des;
float b = 7.5;
void pn_init(void);
void pn_run(void);
float eucld(struct FloatVect3 v);
static void saturate(struct FloatVect3 *vector); 
/**
 * @brief Init function
 */
void pn_init(void)
{
      rel_pos.x = pos_target.x- stateGetPositionNed_f()->x;
      rel_pos.y = pos_target.y- stateGetPositionNed_f()->y;
      rel_pos.z = pos_target.z- stateGetPositionNed_f()->z;
      r_los = eucld(rel_pos);
      speed_des.x = b*rel_pos.x/r_los + speed_target.x;
      speed_des.y = b*rel_pos.y/r_los + speed_target.y;
      speed_des.z = b*rel_pos.z/r_los + speed_target.z;
      v_mag = eucld(speed_target);
      //guidance_indi_init();
      //guidance_indi_enter();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_indi_guidance);
#endif
}

void pn_run(void){
  speed_interceptor.x = stateGetSpeedNed_f()->x;
  speed_interceptor.y = stateGetSpeedNed_f()->x;
  speed_interceptor.z = stateGetSpeedNed_f()->z;
  if (interception){
    speed_des.x = 0.0;
    speed_des.y = 0.0;
    speed_des.z = 0.0;
    accel_des.x = speed_gain*(speed_des.x - speed_interceptor.x); 
    accel_des.y = speed_gain*(speed_des.y - speed_interceptor.y); 
    accel_des.z = speed_gain*(speed_des.z - speed_interceptor.z); 
    quat_des = guidance_indi_run(&accel_des, heading_cmd);
    q_des = stab_sp_to_quat_i(&quat_des);
    in_flight = autopilot_in_flight();
    stabilization_indi_attitude_run(q_des, in_flight);
  }
  else{
    pos_target.x = pos_target.x + speed_target.x*PERIODIC_FREQUENCY;
    pos_target.y = pos_target.y + speed_target.y*PERIODIC_FREQUENCY;
    pos_target.z = pos_target.z + speed_target.z*PERIODIC_FREQUENCY;
    rel_pos.x = pos_target.x- stateGetPositionNed_f()->x;
    rel_pos.y = pos_target.y- stateGetPositionNed_f()->y;
    rel_pos.z = pos_target.z- stateGetPositionNed_f()->z;
    r_los = eucld(rel_pos);
    if (r_los <= 2.0){
      interception = true;
    }
    rel_speed.x = speed_target.x - speed_interceptor.x;
    rel_speed.y = speed_target.y - speed_interceptor.y;
    rel_speed.z = speed_target.z - speed_interceptor.z;
    VECT3_CROSS_PRODUCT(los_rate_num, rel_pos, rel_speed);
    los_rate_denom = VECT3_DOT_PRODUCT(rel_pos, rel_pos);
    los_rate.x = los_rate_num.x/los_rate_denom;
    los_rate.y = los_rate_num.y/los_rate_denom;
    los_rate.z = los_rate_num.z/los_rate_denom;
    VECT3_CROSS_PRODUCT(accel_des_temp, speed_interceptor, los_rate);
    accel_des.x = accel_des_temp.x*N_pn;
    accel_des.y = accel_des_temp.y*N_pn;
    accel_des.z = accel_des_temp.z*N_pn;
    saturate(accel_des);
    speed_des.x = speed_des.x + PERIODIC_FREQUENCY*accel_des.x;
    speed_des.y = speed_des.y + PERIODIC_FREQUENCY*accel_des.y;
    speed_des.z = speed_des.z + PERIODIC_FREQUENCY*accel_des.z;
    speed_mag = eucld(speed_des);
    speed_des.x = v_mag*speed_des.x/speed_mag;
    speed_des.y = v_mag*speed_des.y/speed_mag;
    speed_des.z = v_mag*speed_des.z/speed_mag; 
    accel_des.x = speed_gain*(speed_des.x - speed_interceptor.x) + accel_des.x;
    accel_des.y = speed_gain*(speed_des.y - speed_interceptor.y) + accel_des.y;
    accel_des.z = speed_gain*(speed_des.z - speed_interceptor.z) + accel_des.z;
    quat_des = guidance_indi_run(&accel_des, heading_cmd);
    q_des = stab_sp_to_quat_i(&quat_des);
    in_flight = autopilot_in_flight();
    stabilization_indi_attitude_run(q_des, in_flight);
  }
 
}

float eucld(struct FloatVect3 v){
  float dist;
  dist = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
  return dist;
}

static void saturate(struct FloatVect3 *vector) {
    vector->x = fmax(-5.0, fmin(5.0, vector->x));
    vector->y = fmax(-5.0, fmin(5.0, vector->y));
    vector->z = fmax(-5.0, fmin(5.0, vector->z));
}



