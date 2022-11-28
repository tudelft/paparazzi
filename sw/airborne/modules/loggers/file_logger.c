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


/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;


/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in file_logger_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void file_logger_write_header(FILE *file) {
  fprintf(file, "time,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "accel_b->x, accel_b->y, accel_b->z,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "quat->qi, quat->qx, quat->qy, quat->qz,");
  fprintf(file, "quatsp->qi, quatsp->qx, quatsp->qy, quatsp->qz,");
  fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
}

#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/actuators/actuators.h"
// #include "modules/actuators/motor_mixing.h"

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of file_logger_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void file_logger_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatRates *rates = stateGetBodyRates_f();
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  struct Int32Quat *quatsp = &stab_att_sp_quat;
  struct Int32Vect3 *accel_b = stateGetAccelBody_i();
  // struct NedCoor_f *accel_ned = stateGetAccelNed_f();
  // float sp_x = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x);
  // float sp_y = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y);
  // float sp_z = POS_FLOAT_OF_BFP(guidance_v_z_sp);

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
  fprintf(file, "%d,%d,%d,", accel_b->x, accel_b->y, accel_b->z);
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%d,%d,%d,%d,", quat->qi, quat->qx, quat->qy, quat->qz);
  fprintf(file, "%d,%d,%d,%d,", quatsp->qi, quatsp->qx, quatsp->qy, quatsp->qz);
  fprintf(file, "%d,%d,%d,%d,", actuators_pprz[0], actuators_pprz[1], actuators_pprz[2], actuators_pprz[3]);
  // fprintf(file, "%d,%d,%d,%d,", motor_mixing.commands[MOTOR_FRONT_LEFT], motor_mixing.commands[MOTOR_FRONT_RIGHT], motor_mixing.commands[MOTOR_BACK_RIGHT], motor_mixing.commands[MOTOR_BACK_LEFT]);
#ifdef SITL
  fprintf(file, "0,0,0,0,");
#else
  fprintf(file, "%d,%d,%d,%d,", actuators_bebop.rpm_obs[0], actuators_bebop.rpm_obs[1], actuators_bebop.rpm_obs[2], actuators_bebop.rpm_obs[3]);
#endif
  fprintf(file, "%d,%d,%d,%d\n",
      stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]);
}


/** Start the file logger and open a new file */
void file_logger_start(void)
{
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
