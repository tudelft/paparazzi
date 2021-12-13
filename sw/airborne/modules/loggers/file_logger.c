/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "std.h"

#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "boards/bebop/actuators.h"
#include "subsystems/abi.h"
#include "subsystems/actuators/motor_mixing.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"

// Add neural network library 
#include "modules/gcnet/gcnet_main.h"

// For autopilot mode
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

#ifndef RPM_SENSOR_ID
#define RPM_SENSOR_ID ABI_BROADCAST
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** ABI Messaging */
uint16_t rpm_obs[4] = {0,0,0,0};
uint16_t rpm_ref[4] = {0,0,0,0};
static abi_event rpm_read_ev;
static void rpm_read_cb(uint8_t __attribute__((unused)) sender_id, uint16_t *rpm_obs_read, uint16_t *rpm_ref_read, uint8_t __attribute__((unused)) num_act)
{
   rpm_obs[0] = rpm_obs_read[0];
   rpm_obs[1] = rpm_obs_read[1];
   rpm_obs[2] = rpm_obs_read[2];
   rpm_obs[3] = rpm_obs_read[3];

   rpm_ref[0] = rpm_ref_read[0];
   rpm_ref[1] = rpm_ref_read[1];
   rpm_ref[2] = rpm_ref_read[2];
   rpm_ref[3] = rpm_ref_read[3];
}

/** Used for GPS */
struct LtpDef_i ltp_def;

/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in file_logger_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void file_logger_write_header(FILE *file) {
  fprintf(file, "time,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "gps_pos_x,gps_pos_y,gps_pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "gps_vel_x,gps_vel_y,gps_vel_z,");
  fprintf(file, "acc_x,acc_y,acc_z,");
  fprintf(file, "IMU_ax,IMU_ay,IMU_az,");
  fprintf(file, "body_ax,body_ay,body_az,");
  fprintf(file, "body_ax2,body_ay2,body_az2,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "IMU_p,IMU_q,IMU_r,");
  fprintf(file, "rpm_obs_1,rpm_obs_2,rpm_obs_3,rpm_obs_4,");
  fprintf(file, "rpm_ref_1,rpm_ref_2,rpm_ref_3,rpm_ref_4,");
  fprintf(file, "rpm_cmd_1,rpm_cmd_2,rpm_cmd_3,rpm_cmd_4,");
  fprintf(file, "autopilot_mode,");
  fprintf(file, "wp_goal_x,wp_goal_y,wp_goal_z,");
  fprintf(file, "nn_out_1,nn_out_2,nn_out_3,nn_out_4,");
  fprintf(file, "nn_in_1,nn_in_2,nn_in_3,nn_in_4,nn_in_5,nn_in_6,nn_in_7,nn_in_8,nn_in_9,nn_in_10,nn_in_11,nn_in_12,nn_in_13,nn_in_14,nn_in_15,nn_in_16,nn_in_17,nn_in_18,nn_in_19,");
  fprintf(file, "Mx_measured,My_measured,Mz_measured,az_measured,");
  fprintf(file, "Mx_modeled,My_modeled,Mz_modeled,az_modeled,");
#ifdef COMMAND_THRUST
  fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
#else
  fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
#endif
}

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of file_logger_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void file_logger_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct NedCoor_f *acc = stateGetAccelNed_f();

  // Get the acceleration in body axes
  struct FloatVect3 body_accel_f;
  struct Int32Vect3 *body_accel_i;
  body_accel_i = stateGetAccelBody_i();
  ACCELS_FLOAT_OF_BFP(body_accel_f, *body_accel_i);

  // Get the position and velocity from the gps
  struct EcefCoor_i gps_ecef_pos_i = gps.ecef_pos;	// position in ECEF in cm
  struct NedCoor_i  gps_ned_pos_i;                      // position in NED in m
  ned_of_ecef_pos_i(&gps_ned_pos_i, &ltp_def, &gps_ecef_pos_i);

  struct NedCoor_i gps_ned_vel_i = gps.ned_vel;         // position in NED in cm

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  // get waypoint position
  struct NedCoor_f waypoint_ned;
  ENU_OF_TO_NED(waypoint_ned, waypoints[WP_GOAL].enu_f);

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", POS_FLOAT_OF_BFP(gps_ned_pos_i.x), POS_FLOAT_OF_BFP(gps_ned_pos_i.y), POS_FLOAT_OF_BFP(gps_ned_pos_i.z));
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", (float) gps_ned_vel_i.x/100, (float) gps_ned_vel_i.y/100, (float) gps_ned_vel_i.z/100);
  fprintf(file, "%f,%f,%f,", acc->x, acc->y, acc->z);
  fprintf(file, "%f,%f,%f,", ACCEL_FLOAT_OF_BFP(imu.accel.x), ACCEL_FLOAT_OF_BFP(imu.accel.y), ACCEL_FLOAT_OF_BFP(imu.accel.z));
  fprintf(file, "%f,%f,%f,", ACCEL_FLOAT_OF_BFP(body_accel_i->x), ACCEL_FLOAT_OF_BFP(body_accel_i->y), ACCEL_FLOAT_OF_BFP(body_accel_i->z));
  fprintf(file, "%f,%f,%f,", body_accel_f.x, body_accel_f.y, body_accel_f.z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
  fprintf(file, "%f,%f,%f,", RATE_FLOAT_OF_BFP(imu.gyro.p), RATE_FLOAT_OF_BFP(imu.gyro.q), RATE_FLOAT_OF_BFP(imu.gyro.r));
  fprintf(file, "%d,%d,%d,%d,",rpm_obs[0],rpm_obs[1],rpm_obs[2],rpm_obs[3]);
  fprintf(file, "%d,%d,%d,%d,",rpm_ref[0],rpm_ref[1],rpm_ref[2],rpm_ref[3]);
  fprintf(file, "%d,%d,%d,%d,", motor_mixing.commands[0], motor_mixing.commands[1], motor_mixing.commands[2], motor_mixing.commands[3]);
  fprintf(file, "%d,", (autopilot_get_mode()==AP_MODE_ATTITUDE_DIRECT)?(1):(0));
  fprintf(file, "%f,%f,%f,", waypoint_ned.x, waypoint_ned.y, waypoint_ned.z);
  fprintf(file, "%f,%f,%f,%f,",control_nn[0],control_nn[1],control_nn[2],control_nn[3]);
  fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,",state_nn[0], state_nn[1], state_nn[2], state_nn[3], state_nn[4], state_nn[5], state_nn[6], state_nn[7], state_nn[8], state_nn[9], state_nn[10], state_nn[11], state_nn[12], state_nn[13], state_nn[14], state_nn[15], state_nn[16], state_nn[17], state_nn[18]);
  fprintf(file, "%f,%f,%f,%f,", Mx_measured, My_measured, Mz_measured, az_measured);
  fprintf(file, "%f,%f,%f,%f,", Mx_modeled, My_modeled, Mz_modeled, az_modeled);
  
#ifdef COMMAND_THRUST
  fprintf(file, "%d,%d,%d,%d\n",
      stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]);
#else
  fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif
}


/** Start the file logger and open a new file */
void file_logger_start(void)
{
  // ABI messaging for reading rpm
  AbiBindMsgRPM(RPM_SENSOR_ID, &rpm_read_ev, rpm_read_cb);

  // GPS initialization
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  ltp_def_from_lla_i(&ltp_def, &llh_nav0);

  // Create output folder if necessary
  if (access(STRINGIFY(FILE_LOGGER_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(FILE_LOGGER_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[file_logger] Could not create log file directory %s.\n", STRINGIFY(FILE_LOGGER_PATH));
      return;
    }
  }

  // Get current date/time for filename
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), date_time);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH), date_time, counter);
    counter++;
  }

  file_logger = fopen(filename, "w");
  if(!file_logger) {
    printf("[file_logger] ERROR opening log file %s!\n", filename);
    return;
  }

  printf("[file_logger] Start logging to %s...\n", filename);

  file_logger_write_header(file_logger);
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file    */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  file_logger_write_row(file_logger);
}
