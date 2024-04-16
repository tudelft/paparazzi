#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/pn/pn.h"
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
#include "modules/loggers/logger_file.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/intermcu_msg.h"

#define PRINT(string,...) fprintf(stderr, "[pn->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define PN_PERIODIC_FREQUENCY 500.0



struct pnmessage pninfo = {
  .device = (&((DOWNLINK_DEVICE).device)),
  .enabled = true,
  .msg_available = false,
};

uint8_t pn_msg_buf[256] __attribute__((aligned));  ///< The InterMCU message buffer


struct FloatVect3 speed_des = {0.0f, 0.0f, 0.0f};
struct FloatVect3 pos_des = {0.0f, 0.0f, -1.0f};
struct FloatVect3 speed_target = {6.0f, 6.0f, -4.0f};
struct FloatVect3 speed_interceptor = {0.0f, 0.0f, 0.0f};
struct FloatVect3 pos_target = {85.0f, 20.0f, -30.0f};
struct FloatVect3 accel_des = {0.0f, 0.0f, 0.0f};
struct FloatVect3 accel_des_pn = {0.0f, 0.0f, 0.0f};
struct FloatVect3 rel_pos = { 0 };
struct FloatVect3 los_rate_num = {0.0f, 0.0f, 0.0f};
struct FloatVect3 los_rate = {0.0f, 0.0f, 0.0f};
struct FloatVect3 rel_speed = {0.0f, 0.0f, 0.0f};
struct FloatVect3 accel_des_temp = {0.0f, 0.0f, 0.0f};
struct FloatVect3 speed_err = {0.0f, 0.0f, 0.0f};
struct FloatVect3 pos_err = {0.0f, 0.0f, 0.0f};
struct FloatVect3 acc_actual = {0.0f, 0.0f, 0.0f};
struct StabilizationSetpoint quat_des;
struct Int32Quat q_des;
struct Proportional_nav pn_log;
float speed_gain = 1.8;
float pos_gain = 0.5;
float speed_gainz = 1.1;
float pos_gainy = 0.7;
float r_los = 0.0;
float v_mag = 0.0;
float los_rate_denom = 0.0;
int N_pn = -5.0;
float speed_mag = 0.0;
float heading_cmd = 0.0;
bool in_flight;
bool interception = false;
bool stage2 = false;
bool message = false;
uint8_t flag = 1;
bool los_saturate = false;
bool begin = false;
int counter = 0;
float debug;


// Initialize a pointer to a FloatVect3 struct and assign the address of 'vector'
//struct FloatVect3 *acceleration = &accel_des;
float b = 1.5;
void pn_init(void);
void pn_run(void);
float eucld(struct FloatVect3 v);
static void saturate(struct FloatVect3 *vector, int ind); 
void pn_start(void);
void pn_stop(void);
void pn_event(void);
void pn_info(struct Proportional_nav *pn_log, struct FloatVect3 *accel_des_pn, struct FloatVect3 *accel_des, struct FloatVect3 *speed_des, struct FloatVect3 *los_rate, struct FloatVect3 *pos_target, struct FloatVect3 *pos_des, struct FloatVect3 *speed_target);
struct Proportional_nav *pn_info_logger(void);
void pn_parse_REMOTE_GPS_LOCAL(uint8_t *buf);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_target_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_FLIGHT_BENCHMARK(trans, dev, AC_ID,
                              &pos_target.x,
                              &pos_target.y,
                              &pos_target.z,
                              &speed_target.x,
                              &speed_target.y,
                              &speed_target.z);
}
#endif


void pn_init(void)
{  
  pprz_transport_init(&pninfo.transport);
  //guidance_indi_init();
  //guidance_indi_enter();
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FLIGHT_BENCHMARK, send_target_info);
  #endif

}

void pn_start(void){
  pn_pn_run_status = MODULES_RUN;
  PRINT("SSSSSSS");
  rel_pos.x = pos_target.x- stateGetPositionNed_f()->x;
  rel_pos.y = pos_target.y- stateGetPositionNed_f()->y;
  rel_pos.z = pos_target.z- stateGetPositionNed_f()->z;
  r_los = eucld(rel_pos);
  speed_des.x = b*rel_pos.x/r_los + speed_target.x;
  speed_des.y = b*rel_pos.y/r_los + speed_target.y;
  speed_des.z = b*rel_pos.z/r_los + speed_target.z;
  saturate(&speed_des, 2);
  v_mag = eucld(speed_des);
  intercept = true;
  interception = false;
  begin = true;

}

void pn_stop(void){
  begin = false;
}

void pn_run(void){
  if (begin==false){

    return;
  }

  //PRINT("%d", intercept);
  speed_interceptor.x = stateGetSpeedNed_f()->x;
  speed_interceptor.y = stateGetSpeedNed_f()->y;
  speed_interceptor.z = stateGetSpeedNed_f()->z;
  //pos_target.x = pos_target.x + speed_target.x*1/PN_PERIODIC_FREQUENCY;
  //pos_target.y = pos_target.y + speed_target.y*1/PN_PERIODIC_FREQUENCY;
  //pos_target.z = pos_target.z + speed_target.z*1/PN_PERIODIC_FREQUENCY;
  if (interception){
    //PRINT("BBBB");
    //logger_file_stop();
    speed_des.x = 0.0;
    speed_des.y = 0.0;
    speed_des.z = 0.0;
    accel_des.x = speed_gain*(speed_des.x - speed_interceptor.x);
    accel_des.y = speed_gain*(speed_des.y - speed_interceptor.y); 
    accel_des.z = speed_gain*(speed_des.z - speed_interceptor.z); 
    saturate(&accel_des, 1);
    AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &accel_des);

  }
  else{
    //PRINT("%f", pos_target.x);
    /*
    if (message==false){
      pos_target.x = pos_target.x + 1/PN_PERIODIC_FREQUENCY*speed_target.x;
      pos_target.y = pos_target.y + 1/PN_PERIODIC_FREQUENCY*speed_target.y;
      pos_target.z = pos_target.z + 1/PN_PERIODIC_FREQUENCY*speed_target.z;
    }
    */
    rel_pos.x = pos_target.x- stateGetPositionNed_f()->x;
    rel_pos.y = pos_target.y- stateGetPositionNed_f()->y;
    rel_pos.z = pos_target.z- stateGetPositionNed_f()->z;
    r_los = eucld(rel_pos);
    
    if (r_los <= 0.5){
      PRINT("AAAAA");
      interception = true;
      intercept = false;
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
    accel_des_pn.x = accel_des_temp.x*N_pn;
    accel_des_pn.y = accel_des_temp.y*N_pn;
    accel_des_pn.z = accel_des_temp.z*N_pn;
    saturate(&accel_des_pn, 1);
    speed_des.x = speed_des.x + 1/PN_PERIODIC_FREQUENCY*accel_des_pn.x;
    speed_des.y = speed_des.y + 1/PN_PERIODIC_FREQUENCY*accel_des_pn.y;
    speed_des.z = speed_des.z + 1/PN_PERIODIC_FREQUENCY*accel_des_pn.z;
    speed_mag = eucld(speed_des);
    speed_des.x = v_mag*speed_des.x/speed_mag;
    speed_des.y = v_mag*speed_des.y/speed_mag;
    speed_des.z = v_mag*speed_des.z/speed_mag; 
    
    /*
    pos_des.x = pos_des.x + speed_des.x*1/PN_PERIODIC_FREQUENCY;
    pos_des.y = pos_des.y + speed_des.y*1/PN_PERIODIC_FREQUENCY;
    pos_des.z = pos_des.z + speed_des.z*1/PN_PERIODIC_FREQUENCY; 
    pos_err.x = pos_des.x - stateGetPositionNed_f()->x;
    pos_err.y = pos_des.y - stateGetPositionNed_f()->y;
    pos_err.z = pos_des.z - stateGetPositionNed_f()->z;
    
    speed_des.x = speed_des.x + pos_gain*pos_err.x;
    speed_des.y = speed_des.y + pos_gain*pos_err.y;
    speed_des.z = speed_des.z + pos_gain*pos_err.z;
    */
    saturate(&speed_des, 3);

    //PRINT("%f", accel_des.x);
    acc_actual.x = stateGetAccelNed_f()->x;
    acc_actual.y = stateGetAccelNed_f()->y;
    acc_actual.z = stateGetAccelNed_f()->z;
    

    accel_des.x = speed_gain*(speed_des.x - speed_interceptor.x) + accel_des_pn.x;
    accel_des.y = speed_gain*(speed_des.y - speed_interceptor.y) + accel_des_pn.y;
    accel_des.z = speed_gain*(speed_des.z - speed_interceptor.z) + accel_des_pn.z;
    saturate(&accel_des, 1);

    pn_info(&pn_log, &accel_des_pn, &accel_des, &speed_des, &los_rate, &pos_target, &acc_actual, &speed_target);
    //intercept = interception;
    AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &accel_des);
    
    
    //quat_des = guidance_indi_run(&accel_des, heading_cmd);
    //q_des = stab_sp_to_quat_i(&quat_des);
    //in_flight = autopilot_in_flight();
    //stabilization_indi_attitude_run(q_des, in_flight);
  }
  
 
}

void pn_event(void)
{
  /* Parse incoming bytes */
  if (pninfo.enabled) {
    pprz_check_and_parse(pninfo.device, &pninfo.transport, pn_msg_buf, &pninfo.msg_available);
    //PRINT("%d", pninfo.msg_available);
    if (counter ==50){
      //PRINT("%d", pninfo.msg_available);
      counter = 0;
    }
    if (pninfo.msg_available) {
      message = true;
      uint8_t class_id = pprzlink_get_msg_class_id(pn_msg_buf);
      
      pninfo.time_since_last_frame = 0;
      dl_parse_msg(pninfo.device, &pninfo.transport.trans_tx, pn_msg_buf);
    }
    pninfo.msg_available = false;
    counter++;
  }
}

float eucld(struct FloatVect3 v){
  float dist;
  dist = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
  return dist;
}

static void saturate(struct FloatVect3 *vector, int ind) {
  if (ind == 1){
    vector->x = fmax(-5.0, fmin(5.0, vector->x));
    vector->y = fmax(-5.0, fmin(5.0, vector->y));
    vector->z = fmax(-5.0, fmin(5.0, vector->z));
  }
  else if(ind == 2){
    vector->x = fmax(-10.0, fmin(10.0, vector->x));
    vector->y = fmax(-10.0, fmin(10.0, vector->y));
    vector->z = fmax(-10.0, fmin(10.0, vector->z));
  }
  else {
    vector->x = fmax(-2.5, fmin(2.5, vector->x));
    vector->y = fmax(-2.5, fmin(2.5, vector->y));
    vector->z = fmax(-2.5, fmin(2.5, vector->z));    
  }

}

void pn_info(struct Proportional_nav *pn_log, struct FloatVect3 *accel_des_pn, struct FloatVect3 *accel_des, struct FloatVect3 *speed_des, struct FloatVect3 *los_rate, struct FloatVect3 *pos_target, struct FloatVect3 *pos_des, struct FloatVect3 *speed_target){
  pn_log->accel_des_pn = *accel_des_pn;
  pn_log->accel_des = *accel_des;
  pn_log->speed_des = *speed_des;
  pn_log->los_rate = *los_rate;
  pn_log->pos_target = *pos_target;
  pn_log->pos_des = *pos_des;
  pn_log->speed_target = *speed_target;

}

struct Proportional_nav *pn_info_logger(void){
  return &pn_log;
}

void pn_parse_REMOTE_GPS_LOCAL(uint8_t *buf)
{
  //if (DL_REMOTE_GPS_LOCAL_ac_id(buf) != AC_ID){return; }
  pos_target.x = DL_REMOTE_GPS_LOCAL_enu_x(buf);
  pos_target.y = DL_REMOTE_GPS_LOCAL_enu_y(buf);
  pos_target.z = DL_REMOTE_GPS_LOCAL_enu_z(buf);
  speed_target.x = DL_REMOTE_GPS_LOCAL_enu_xd(buf);
  speed_target.y = DL_REMOTE_GPS_LOCAL_enu_yd(buf);
  speed_target.z = DL_REMOTE_GPS_LOCAL_enu_zd(buf);
  //PRINT("%f", pos_target.x);
}



