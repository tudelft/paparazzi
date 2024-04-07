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

/** @file modules/loggers/logger_file.c
 *  @brief File logger for Linux based autopilots
 */

#include "logger_file.h"

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

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "generated/modules.h"

/** Set the default File logger path to the USB drive */
#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *logger_file = NULL;


/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in logger_file_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void logger_file_write_header(FILE *file) {
  fprintf(file, "time,");
  fprintf(file, "ACT_1,ACT_2,ACT_3,ACT_4,ACT_5,ACT_6,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "att_phi,att_theta,att_psi,");
<<<<<<< Updated upstream
  fprintf(file, "accel_x,accel_y,accel_z,");
=======
>>>>>>> Stashed changes
  fprintf(file, "g1g2[1][4], g1g2[1][5],  g1g2[2][4], g1g2[2][5],");
  fprintf(file, "airspeed,");
  fprintf(file, "guidance_cmd.phi, guidance_cmd.theta, guidance_cmd.psi,");
  fprintf(file, "sp_accel.x, sp_accel.y, sp_accel.z,");
  fprintf(file, "speed_sp.x, speed_sp.y, speed_sp.z,");
<<<<<<< Updated upstream
  fprintf(file, "norm_des_as,");

=======
  fprintf(file, "use_increment,");
>>>>>>> Stashed changes
#ifdef BOARD_BEBOP
  fprintf(file, "rpm_obs_1,rpm_obs_2,rpm_obs_3,rpm_obs_4,");
  fprintf(file, "rpm_ref_1,rpm_ref_2,rpm_ref_3,rpm_ref_4,");
#endif
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_header(file);
#endif
#ifdef COMMAND_THRUST
<<<<<<< Updated upstream
  fprintf(file, "cmd_roll,cmd_pitch,cmd_thrust,cmd_psi\n");
=======
   fprintf(file, "cmd_roll,cmd_pitch,cmd_thrust,cmd_psi\n");
>>>>>>> Stashed changes
#else
  fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
#endif
}

#include "modules/actuators/actuators.h"

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of logger_file_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void logger_file_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  struct Int32Vect3 *acc = stateGetAccelBody_i();
  struct FloatVect3 acc_f = {ACCEL_FLOAT_OF_BFP(acc->x), ACCEL_FLOAT_OF_BFP(acc->y), ACCEL_FLOAT_OF_BFP(acc->z)};
  float airspeed = stateGetAirspeed_f();

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,%f,%f,%f,", indi_u[0],indi_u[1],indi_u[2],indi_u[3],indi_u[4],indi_u[5]);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", eulers_zxy.phi, eulers_zxy.theta, eulers_zxy.psi);
<<<<<<< Updated upstream
  fprintf(file, "%f,%f,%f,", accel_filt.x, accel_filt.y, accel_filt.z);
=======
>>>>>>> Stashed changes
  fprintf(file, "%f,%f,%f,%f,", g1g2[1][4], g1g2[1][5],  g1g2[2][4], g1g2[2][5]);
  fprintf(file, "%f,", airspeed);
  fprintf(file, "%f,%f,%f,", guidance_euler_cmd.phi, guidance_euler_cmd.theta, guidance_euler_cmd.psi);
  fprintf(file, "%f,%f,%f,", sp_accel.x, sp_accel.y, sp_accel.z);
  fprintf(file, "%f,%f,%f,", gi_speed_sp.x, gi_speed_sp.y, gi_speed_sp.z);
<<<<<<< Updated upstream
  fprintf(file, "%f,", norm_des_as);
=======
  fprintf(file, "%f,", use_increment);

>>>>>>> Stashed changes

#ifdef BOARD_BEBOP
  fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_obs[0],actuators_bebop.rpm_obs[1],actuators_bebop.rpm_obs[2],actuators_bebop.rpm_obs[3]);
  fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_ref[0],actuators_bebop.rpm_ref[1],actuators_bebop.rpm_ref[2],actuators_bebop.rpm_ref[3]);
#endif
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_data(file);
#endif
#ifdef COMMAND_THRUST
  fprintf(file, "%f,%f,%f,%f\n",
      // stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      // stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]
      euler_cmd.x,euler_cmd.y,euler_cmd.z,guidance_indi_hybrid_heading_sp);
#else
  fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif
}


/** Start the file logger and open a new file */
void logger_file_start(void)
{
  // Ensure that the module is running when started with this function
  logger_file_logger_file_periodic_status = MODULES_RUN;

  // Create output folder if necessary
  if (access(STRINGIFY(LOGGER_FILE_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(LOGGER_FILE_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[logger_file] Could not create log file directory %s.\n", STRINGIFY(LOGGER_FILE_PATH));
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
  sprintf(filename, "%s/%s.csv", STRINGIFY(LOGGER_FILE_PATH), date_time);
  while ((logger_file = fopen(filename, "r"))) {
    fclose(logger_file);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(LOGGER_FILE_PATH), date_time, counter);
    counter++;
  }

  logger_file = fopen(filename, "w");
  if(!logger_file) {
    printf("[logger_file] ERROR opening log file %s!\n", filename);
    return;
  }

  printf("[logger_file] Start logging to %s...\n", filename);

  logger_file_write_header(logger_file);
}

/** Stop the logger an nicely close the file */
void logger_file_stop(void)
{
  if (logger_file != NULL) {
    fclose(logger_file);
    logger_file = NULL;
  }
}

/** Log the values to a csv file    */
void logger_file_periodic(void)
{
  if (logger_file == NULL) {
    return;
  }
  logger_file_write_row(logger_file);
}
