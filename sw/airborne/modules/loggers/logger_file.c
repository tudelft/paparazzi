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

#include "generated/modules.h"

#include "guidance/guidance_unified.h"

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
  fprintf(file, "timestamp,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "pos_ref_x,pos_ref_y,pos_ref_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "vel_ref_x,vel_ref_y,vel_ref_z,");
  fprintf(file, "vel_meas_x,vel_meas_y,vel_meas_z,");
  fprintf(file, "acc_x,acc_y,acc_z,");
  fprintf(file, "acc_ref_x,acc_ref_y,acc_ref_z,");
  fprintf(file, "acc_meas_x,acc_meas_y,acc_meas_z,");
  fprintf(file, "accel_ref_with_gains_x,accel_ref_with_gains_y,accel_ref_with_gains_z,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "roll_rate_calc,pitch_rate_calc,yaw_rate_calc,");
  fprintf(file, "T_guid,");
  fprintf(file, "T_filt,");
  fprintf(file, "T_cmd,");
  fprintf(file, "d_accel_ref_v.x, d_accel_ref_v.y, d_accel_ref_v.z,");
  fprintf(file, "d_accel_ref_b.x, d_accel_ref_b.y, d_accel_ref_b.z,");
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_header(file);
#endif
  fprintf(file, "cmd_TL,cmd_TR,cmd_BR,cmd_BL\n");
}

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of logger_file_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void logger_file_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct NedCoor_f *acc = stateGetAccelNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", pos_ref[0], pos_ref[1], pos_ref[2]);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", vel_ref[0], vel_ref[1], vel_ref[2]);
  fprintf(file, "%f,%f,%f,", vel_a_filt[0], vel_a_filt[1], vel_a_filt[2]);
  fprintf(file, "%f,%f,%f,", acc->x, acc->y, acc->z);
  fprintf(file, "%f,%f,%f,", accel_ref[0], accel_ref[1], accel_ref[2]);
  fprintf(file, "%f,%f,%f,", accel_a_filt[0], accel_a_filt[1], accel_a_filt[2]);
  fprintf(file, "%f,%f,%f,", accel_ref_with_gains[0], accel_ref_with_gains[1], accel_ref_with_gains[2]);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
  fprintf(file, "%f,%f,%f,", roll_rate_calc, pitch_rate_calc, cmd_yaw_rate);
  fprintf(file, "%f,", T);
  fprintf(file, "%f,", T_filt);
  fprintf(file, "%f,", T_cmd);
  fprintf(file, "%f,%f,%f,", d_accel_ref_v_calc.x, d_accel_ref_v_calc.y, d_accel_ref_v_calc.z);
  fprintf(file, "%f,%f,%f,", d_accel_ref_b_calc.x, d_accel_ref_b_calc.y, d_accel_ref_b_calc.z);
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_data(file);
#endif
  fprintf(file, "%d,%d,%d,%d\n", actuators_pprz[0], actuators_pprz[1], actuators_pprz[2], actuators_pprz[3]);
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