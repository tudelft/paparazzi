/*
 * Copyright (C) 2021 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/ctrl/approach_moving_target.c"
 * @author Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * Approach a moving target (e.g. ship)
 */

#include "approach_moving_target.h"

#include "generated/modules.h"
#include "modules/core/abi.h"

//float amt_err_slowdown_gain = AMT_ERR_SLOWDOWN_GAIN;
float amt_err_slowdown_gain = 0.001;
bool allways_update_ship_wp = true;

float approach_moving_target_angle_deg;


#define DEBUG_AMT TRUE
#include <stdio.h>

// settings how drone should approach the ship
struct Amt amt = {
  .distance = 40,     // [m], diagonal decent line to ship
  .speed = -1.0,      // [m/s], speed over descent line to ship, inverted because software looks from ship to drone
  .pos_gain = 200.2,    // [-], how aggresive drone tracks the descent line
  .psi_ref = 180.0,   // [deg], descent line direction offset w.r.t. heading ship
  .slope_ref = 19.471,  // [deg], slope descent line
  .speed_gain = 1.0,  // [-], how agressive ..................
  .relvel_gain = 1.0, // [-], ................................
  .approach_speed_gain = 1.0,
  .enabled_time = 0,
  .wp_ship_id = 0,
  .wp_approach_id = 0,
  .wp_carrot_id = 0,
  .last_lag = 0,
  .lag = 0,
  .integral_gain_carrot = 1.0,
  .pid_error = 0,
  .pid_errorLast = 0,
  .pid_errorSum = 0,
  .pid_last_timestamp = 0
};

// settings how drone should approach the ship
struct Wave WaveInfl = {
  .RollPitchYaw.x = 0,        // [deg]  the attitude of the deck of the ship
  .RollPitchYaw.y = 0,        // [deg]  the attitude of the deck of the ship
  .RollPitchYaw.z = 0,        // [deg]  the attitude of the deck of the ship
  .WaveOff2StaticPos.x = 0,   // [m]    the offset from the average/static position of the ship
  .WaveOff2StaticPos.y = 0,   // [m]    the offset from the average/static position of the ship
  .WaveOff2StaticPos.z = 0,   // [m]    the offset from the average/static position of the ship
  .estmInterval = 0,          // [sec]  the estimated interval in beteween the waves
  .estmAmp = 0,               // [m]    the estimated amplitude of the waves
  .certainty = 0              // [%]    how certain the system is of the wave shape/interval
};

// Telemetry info to send from drone to base
struct AmtTelem {
  struct FloatVect3 des_pos; // descent point to follow 
  struct FloatVect3 des_vel;
  struct FloatVect3 target_pos; //ship landing position
  struct FloatVect3 target_vel;
  float start_distance;
  float approach_speed;
};

struct AmtTelem amt_telem;

bool approach_moving_target_enabled = false; // NOT USED

static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);

struct FloatVect3 nav_get_speed_sp_from_diagonal(struct EnuCoor_i target, float pos_gain, float rope_heading);
void update_waypoint(uint8_t wp_id, struct FloatVect3 * target_ned);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_approach_moving_target(struct transport_tx *trans, struct link_device *dev)
{
  // pprz_msg_send_APPROACH_MOVING_TARGET(trans, dev, AC_ID,
  //                             &amt_telem.des_pos.x,
  //                             &amt_telem.des_pos.y,
  //                             &amt_telem.des_pos.z,
  //                             &amt_telem.des_vel.x,
  //                             &amt_telem.des_vel.y,
  //                             &amt_telem.des_vel.z,
  //                             &amt.distance
  //                             );
   pprz_msg_send_APPROACH_MOVING_TARGET(trans, dev, AC_ID,
                              &amt_telem.des_pos.x,
                              &amt_telem.des_pos.y,
                              &amt_telem.des_pos.z,
                              &amt_telem.des_vel.x,
                              &amt_telem.des_vel.y,
                              &amt_telem.des_vel.z,

                              &amt_telem.target_pos.x,
                              &amt_telem.target_pos.y,
                              &amt_telem.target_pos.z,
                              &amt_telem.target_vel.x,
                              &amt_telem.target_vel.y,
                              &amt_telem.target_vel.z,

                              &amt_telem.start_distance,
                              &amt.distance,
                              &amt_telem.approach_speed,
                              &amt.slope_ref,
                              &force_forward
                              );
}
#endif

struct LlaCoor_i gps_lla;

void approach_moving_target_init(void)
{
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

  amt_telem.start_distance = amt.distance;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_APPROACH_MOVING_TARGET, send_approach_moving_target);
#endif
}

/**
 * Receive a RC_4CH message from the ground
 * TODO: this function needs a better place in a more general script
 */
void target_parse_RC_4CH(uint8_t *buf)
{
  // Save the receivd rotating knob value
  amt.approach_speed_gain = (float)DL_RC_4CH_rotating_knob(buf)/100;
}


/* Update INS (internal navigation system) based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((uTARGET_POS_INFOnused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
  gps_lla.lat = gps_s->lla_pos.lat;
  gps_lla.lon = gps_s->lla_pos.lon;
  gps_lla.alt = gps_s->lla_pos.alt;
}

// interface with ship position module?

// Update a waypoint such that you can see on the GCS where the drone wants to go
void update_waypoint(uint8_t wp_id, struct FloatVect3 * target_ned) {

  // Update the waypoint
  struct EnuCoor_f target_enu;
  ENU_OF_TO_NED(target_enu, *target_ned);
  waypoint_set_enu(wp_id, &target_enu);

  // Send waypoint update every 0.2 second !!!!!! THIS ONE SEEMD NOT BE USED, if time is long, nothing is different in GCS sim
  //RunOnceEvery(0.2, {
    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                                &waypoints[wp_id].enu_i.x,
                                &waypoints[wp_id].enu_i.y,
                                &waypoints[wp_id].enu_i.z);
  //} );
}

// Function to enable from flight plan (call repeatedly!)
void approach_moving_target_enable(uint8_t wp_ship_id, uint8_t wp_approach_id, uint8_t wp_carrot_id) { 
  amt.enabled_time = get_sys_time_msec(); // this makes folow_diagonal_approach()
  amt.wp_ship_id = wp_ship_id;
  amt.wp_approach_id = wp_approach_id;
  amt.wp_carrot_id = wp_carrot_id;
}

// Function to enable from flight plan (call repeatedly!)
void reset_moving_target_distance(void) { 
  amt.distance = amt_telem.start_distance;
}



void waveEstimation(void){
  const int filter_length = 20;
  static bool filter_full = false;
  static int filter_loopcounter = 0;
  //static struct FloatVect3 bufRPY[filter_length];
  //static struct FloatVect3 bufOff[filter_length];
  
  struct FloatVect3 dummyWaveRPY;
  dummyWaveRPY.x = 20; //roll
  dummyWaveRPY.y = 5;  //pitch
  dummyWaveRPY.z = 10;  //yaw

  // WaveInfl.bufRPY[filter_loopcounter] = dummyWaveRPY;

  VECT3_COPY(WaveInfl.bufRPY[filter_loopcounter], dummyWaveRPY)

  //avg calc
  struct FloatVect3 sumBufRPY;
  for (int i=0; i <filter_length; i++){
    VECT3_ADD(sumBufRPY, WaveInfl.bufRPY[i]);
  }
  struct FloatVect3 avgBufRPY;
  printf("avgBuf %.1f %.1f %.1f \n",  sumBufRPY.x,  sumBufRPY.y,  sumBufRPY.z);
  VECT3_SDIV(avgBufRPY, sumBufRPY, filter_length);

  printf("avgBuf %.1f %.1f %.1f \n",  avgBufRPY.x,  avgBufRPY.y,  avgBufRPY.z);

  Bound(WaveInfl.certainty, 0, 100);
  filter_loopcounter++;
  if (filter_loopcounter >=filter_length) filter_loopcounter = 0;
}



/**
 * @brief Generates a velocity reference from a diagonal approach path
 *
 */
void follow_diagonal_approach(void) {
  /* STEPS
    - check if approach_moving_target_enable() is called recently by flightplan (otherwise return)
    - get target pos, vel, hdg (target is the landing spot)
    - calculate descent line unit vector (is it nesceserry to calculate this over and over every loop?)
    - 
  */
  uint32_t start_time = get_sys_time_msec();

  // waveEstimation(); // TESTING // NOT WORKING YET

  // Check if the flight plan recently called the enable function
  if ( (get_sys_time_msec() - amt.enabled_time) > (2000 / NAVIGATION_FREQUENCY) && !allways_update_ship_wp) {
    return; // if approach_moving_target_enable is not called recently
  }

  // Get the target position, velocity and heading
  struct NedCoor_f rel_target_pos, target_vel_boat = {0};
  float target_heading;
  if(!target_get_pos(&rel_target_pos, &target_heading)) {
    // TODO: What to do? Same can be checked for the velocity
    return;
  }
  //

  target_get_vel(&target_vel_boat); // [m/s] update ground speed of the ship
  VECT3_SMUL(target_vel_boat, target_vel_boat, amt.speed_gain);

  // Reference model (descent line)
  // Translate angles to unit vector
  float gamma_ref = RadOfDeg(amt.slope_ref); // descent rate 
  float psi_ref = RadOfDeg(target_heading + amt.psi_ref); // how descent line lines up with ship heading
  amt.rel_unit_vec.x = cosf(gamma_ref) * cosf(psi_ref);
  amt.rel_unit_vec.y = cosf(gamma_ref) * sinf(psi_ref); 
  amt.rel_unit_vec.z = -sinf(gamma_ref);
  //printf("amt.rel_unit_vec.z %f \n", amt.rel_unit_vec.z);

  // Multiply vector with prefered descent dist to get ref point of descent
  // Desired position = rel_pos + target_pos_boat ??????????????
  struct FloatVect3 ref_relpos;
  VECT3_SMUL(ref_relpos, amt.rel_unit_vec, amt.distance); //calculate decscent point

  //printf("ref_relpos.z %f \n", ref_relpos.z);

  // Add ref point of descent to ship location to get NED coordinate ref point of descent
  // ATTENTION, target_pos_boat is already relative now!
  struct FloatVect3 rel_des_pos;
  VECT3_SUM(rel_des_pos, ref_relpos, rel_target_pos); 
  //printf("ref_relpos.z %f \n", ref_relpos.z);


  // ------------------------------------------------------------------------- ADD MORE COMMENTS FROM HERE ON

  struct FloatVect3 ref_relvel;
  VECT3_SMUL(ref_relvel, amt.rel_unit_vec, amt.speed * amt.relvel_gain); 

  // error controller
  struct FloatVect3 pos_err;
  struct FloatVect3 ec_vel;
  struct NedCoor_f *drone_pos = stateGetPositionNed_f();
  // ATTENTION, target_pos_boat is already relative now!
  // VECT3_DIFF(pos_err, des_pos, *drone_pos);
  VECT3_COPY(pos_err, rel_des_pos);
  VECT3_SMUL(ec_vel, pos_err, amt.pos_gain);

  // desired velocity = rel_vel + target_vel_boat + error_controller(using NED position)
  struct FloatVect3 des_vel = {
    ref_relvel.x + target_vel_boat.x + ec_vel.x,
    ref_relvel.y + target_vel_boat.y + ec_vel.y,
    ref_relvel.z + target_vel_boat.z + ec_vel.z,
  };

  vect_bound_in_3d(&des_vel, 10.0);

  // Bound vertical speed setpoint
  if(stateGetAirspeed_f() > 13.0) {
    Bound(des_vel.z, -4.0, 5.0);
  } else {
    Bound(des_vel.z, -nav_climb_vspeed, -nav_descend_vspeed);
  }

  // ToDo:
  // What does VEL_SP do?
  // this is used by guidance INDI hybrid, but not by normal guidance INDI...
  //AbiSendMsgVEL_SP(VEL_SP_FCR_ID, &des_vel); // ????????????????????????????????????????????? NOT USED ????????????????????????????

  /////////////////////////////// TEST
  /*
  struct FloatVect3 ned_drone_vel;
  ned_drone_vel.x = stateGetSpeedNed_f()->x;
  ned_drone_vel.y = stateGetSpeedNed_f()->y;
  struct FloatVect3 ned_corr_vel;
  VECT3_DIFF(ned_corr_vel, des_vel, ned_drone_vel)
  struct FloatVect3 scaled_ned_corr_vel;
  VECT3_SMUL(scaled_ned_corr_vel, ned_corr_vel, 1.0)
  AbiSendMsgACCEL_SP(VEL_SP_FCR_ID, 1,&scaled_ned_corr_vel);
  printf("scaled_ned_corr_vel X: %f , \t Y: %f , \t Z: %f \n", scaled_ned_corr_vel.x, scaled_ned_corr_vel.y, scaled_ned_corr_vel.z);
  */
  /////////////////////////////// TEST

  /* limit the speed such that the vertical component is small enough
  * and doesn't outrun the vehicle
  
  float min_speed;
  float sin_gamma_ref = sinf(gamma_ref);
  if (sin_gamma_ref > 0.05) {
    min_speed = (nav_descend_vspeed+0.1) / sin_gamma_ref;
  } else {
    min_speed = -5.0; // prevent dividing by zero
  }

  // The upper bound is not very important
  Bound(amt.speed, min_speed, 4.0);
  */

  // Reduce approach speed if the error is large
  float norm_pos_err_sq = VECT3_NORM2(pos_err);
  // int_speed = (default speed / (squared position error [m] * slowdown factor + 1) * speed gain control by joystick
  amt_telem.approach_speed = ((amt.speed) / (norm_pos_err_sq * amt_err_slowdown_gain + 1.0)) * amt.approach_speed_gain * (int)force_forward;
  if (amt.distance < 20) amt_telem.approach_speed = amt_telem.approach_speed / 2;

  // Check if the flight plan recently called the enable function
  // make distance to ship smaller, So descent to ship

  if ( (get_sys_time_msec() - amt.enabled_time) > (2000 / NAVIGATION_FREQUENCY) && force_forward) {
    // integrate speed to get the distance
    float dt = FOLLOW_DIAGONAL_APPROACH_PERIOD;
    amt.distance += amt_telem.approach_speed*dt;
    Bound(amt.distance, 0, 100); // approach dist > 0
    //amt.distance -= 1*dt;
  }
  

  // For display purposes
  struct FloatVect3 ned_pos_target;
  VECT3_SUM(ned_pos_target, rel_target_pos, *drone_pos);
  //struct FloatVect3 ne_up_pos_target; // North, East, UP (alt) for Paparazzi
  //VECT3_COPY(ne_up_pos_target, ned_pos_target);
  //ne_up_pos_target.z = ne_up_pos_target.z*-1;
  update_waypoint(amt.wp_ship_id, &ned_pos_target);
  
  // *drone_pos = NED float
  // des_pos = NED float
  // ned _pos_approach = NED float
  struct FloatVect3 ned_pos_approach; // WAS LLA instead of NED

  VECT3_SUM(ned_pos_approach, rel_des_pos, *drone_pos);

  update_waypoint(amt.wp_approach_id, &ned_pos_approach);  
  //printf("ned_pos_approach.z %f \n", ned_pos_approach.z);

  struct FloatVect3 ned_pos_carrot; // TEST: SHIFT CARROT FORWARD TO REDUCE STEADY STATE ERROR
  struct FloatVect3 carrot_correction; // TEST: SHIFT CARROT FORWARD TO REDUCE STEADY STATE ERROR
  VECT3_COPY(ned_pos_carrot, ned_pos_approach);

  // TEST 1
  /*
  float c_rot = cosf(RadOfDeg(90-target_heading));
  float s_rot = sinf(RadOfDeg(90-target_heading));


  float x_new = pos_err.x*c_rot - pos_err.y*s_rot;
  float y_new = pos_err.x*s_rot + pos_err.y*c_rot;

  //printf(" x: %f   y: %f  \n", x_new, y_new);
  //printf("target heading: %f \n", target_heading);

  Bound(y_new, 0, 5); // bound correction to only forward of the position (wrt moving direction) and max 5 meters

  carrot_correction.x = 0;
  carrot_correction.y = y_new*0.2 + carrot_correction.y*0.8;
  carrot_correction.z = 0;
  //printf(" Carrot corr x: %f   y: %f  \n", carrot_correction.x, carrot_correction.y);

  float c_rot_back = cosf(RadOfDeg(90+target_heading));
  float s_rot_back = sinf(RadOfDeg(90+target_heading));

  float N_carrot = carrot_correction.x*c_rot_back - carrot_correction.y*s_rot_back;
  float E_carrot = carrot_correction.x*s_rot_back + carrot_correction.y*c_rot_back;

  carrot_correction.x = E_carrot;
  carrot_correction.y = N_carrot;
  carrot_correction.z = 0;
 */

  
  //VECT3_ADD_SCALED(ned_pos_carrot, carrot_correction, 1.0);

 

  // TEST 2 
  /*
  float horizontal_error_dist = sqrtf(pos_err.x*pos_err.x + pos_err.y*pos_err.y);
  float horizontal_velocity_boat = sqrtf(target_vel_boat.x*target_vel_boat.x+target_vel_boat.y*target_vel_boat.y);
  float estm_lag = horizontal_error_dist/horizontal_velocity_boat;
  printf("esimated lag: %f effective lag: %f \n", estm_lag, amt.lag);

  if ((estm_lag - amt.last_lag) > 0){ // points diverging
    amt.lag = amt.lag + amt.integral_gain_carrot/1000;
  }
  else {                              // points comming together
    amt.lag = amt.lag - amt.integral_gain_carrot/1000;  
  }
  Bound(amt.lag, 0, 2); // maximum 2 seconds lag compensation

  amt.last_lag = estm_lag;

  carrot_correction.x = target_vel_boat.x*amt.lag;
  carrot_correction.y = target_vel_boat.y*amt.lag;
  carrot_correction.z = 0;

  VECT3_ADD_SCALED(ned_pos_carrot, carrot_correction, 1.0);
  */

  // TEST 3 
  // PID control
  /*
  // x is North
  // y is east
  // z is down

  float rot_sin = sinf(RadOfDeg(target_heading));
  float rot_cos = cosf(RadOfDeg(target_heading));

  float pos_err_forw_ship = pos_err.x * rot_cos  + pos_err.y * rot_sin;
  float pos_err_side_ship = pos_err.x * -rot_sin + pos_err.y * rot_cos;
  //printf("error wrt ship forward: %f , \t side: %f \n", pos_err_forw_ship, pos_err_side_ship);
  
  float kp = 1;
  float ki = 0;
  float kd = 0.5;
  amt.pid_error = pos_err_forw_ship; //pow(pow(pos_err.x, 2) + pow(pos_err.y, 2), 0.5);
  float time_step = (float)(get_sys_time_msec()- amt.pid_last_timestamp) / 1000;
  float derrivative_error = ((amt.pid_error - amt.pid_errorLast) / time_step); // SOMEHOW BIG STEP IN DERRIVATIVE ERROR IN SIMULATION
  printf("D: %f , \t err: %f , \t last_err: %f , \t timestep: %f \n", derrivative_error, amt.pid_error, amt.pid_errorLast, time_step);
  amt.pid_errorSum += amt.pid_error*time_step;

  amt.pid_errorLast = amt.pid_error;
  amt.pid_last_timestamp = get_sys_time_msec();

  float P_val = kp*amt.pid_error;
  float I_val = ki*amt.pid_errorSum;
  float D_val = kd*derrivative_error;

  float PID_value = P_val + I_val + D_val;
  printf("fwrd_err: %f , \t PID value: %f , \t P: %f , \t I: %f , \t D: %f , \t timestep: %f \n", pos_err_forw_ship, PID_value, P_val, I_val, D_val, time_step);
  Bound(PID_value, 0, 5); // bound correction distance to 5 meters

  float unit_vec_x_ship = target_vel_boat.x / pow(pow(target_vel_boat.x, 2)+pow(target_vel_boat.y, 2), 0.5);
  float unit_vec_y_ship = target_vel_boat.y / pow(pow(target_vel_boat.x, 2)+pow(target_vel_boat.y, 2), 0.5);

  carrot_correction.x = unit_vec_x_ship*PID_value;
  carrot_correction.y = unit_vec_y_ship*PID_value;
  carrot_correction.z = 0;
  VECT3_ADD_SCALED(ned_pos_carrot, carrot_correction, 1.0);
  */

  // TEST 4
  /*
  float rot_sin = sinf(RadOfDeg(target_heading));
  float rot_cos = cosf(RadOfDeg(target_heading));

  float pos_err_forw_ship = pos_err.x * rot_cos  + pos_err.y * rot_sin;
  float pos_err_side_ship = pos_err.x * -rot_sin + pos_err.y * rot_cos;
  printf("error wrt ship forward: %f , \t side: %f \n", pos_err_forw_ship, pos_err_side_ship);
  */

  // TEST BASIC:
  VECT3_ADD_SCALED(ned_pos_carrot, pos_err, 1.0);

  update_waypoint(amt.wp_carrot_id, &ned_pos_carrot); // TESTING CARROT

  // Update values for telemetry
  VECT3_COPY(amt_telem.des_pos, rel_des_pos); 
  VECT3_COPY(amt_telem.des_vel, des_vel);

  // Update values for telemetry
  VECT3_COPY(amt_telem.target_pos, rel_target_pos); 
  VECT3_COPY(amt_telem.target_vel, target_vel_boat);

  uint32_t end_time = get_sys_time_msec();
  //printf("loop_time = %i %i \n", end_time, start_time);
}
