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

#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"

#include "generated/modules.h"

/** Set the default File logger path to the USB drive */
#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /data/ftp/internal_000
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
  fprintf(file,"ap_mode,ap_in_flight,");
  fprintf(file,"phi_ref,theta_ref,psi_ref,phi,theta,psi,");
  fprintf(file,"p_ref,q_ref,r_ref,p,q,r,");
  fprintf(file,"p_dot_ref,q_dot_ref,r_dot_ref,p_dot,q_dot,r_dot,");
  fprintf(file,"pos_N_ref,pos_E_ref,pos_D_ref,pos_N,pos_E,pos_D,");
  fprintf(file,"vel_N_ref,vel_E_ref,vel_D_ref,vel_N,vel_E,vel_D,");
  fprintf(file,"acc_N_ref,acc_E_ref,acc_D_ref,acc_N,acc_E,acc_D,");
  fprintf(file,"jerk_N,jerk_E,jerk_D,");
  fprintf(file,"nu_0,nu_1,nu_2,nu_3,nu_4,nu_5,");
  fprintf(file,"u_0,u_1,u_2,u_3,");
  fprintf(file,"G1_aN_0,G1_aN_1,G1_aN_2,G1_aN_3,G1_aN_4,G1_aN_5,");
  fprintf(file,"G1_aE_0,G1_aE_1,G1_aE_2,G1_aE_3,G1_aE_4,G1_aE_5,");
  fprintf(file,"G1_aD_0,G1_aD_1,G1_aD_2,G1_aD_3,G1_aD_4,G1_aD_5,");
  fprintf(file,"G1_roll_0,G1_roll_1,G1_roll_2,G1_roll_3,G1_roll_4,G1_roll_5,");
  fprintf(file,"G1_pitch_0,G1_pitch_1,G1_pitch_2,G1_pitch_3,G1_pitch_4,G1_pitch_5,");
  fprintf(file,"G1_yaw_0,G1_yaw_1,G1_yaw_2,G1_yaw_3,G1_yaw_4,G1_yaw_5,");
  fprintf(file,"andi_u_0,andi_u_1,andi_u_2,andi_u_3,andi_u_4,andi_u_5,");
  fprintf(file,"andi_du_0,andi_du_1,andi_du_2,andi_du_3,andi_du_4,andi_du_5\n");

  // fprintf(file, "pos_x,pos_y,pos_z,");
  // fprintf(file, "vel_x,vel_y,vel_z,");
  // fprintf(file, "att_phi,att_theta,att_psi,");
  // fprintf(file, "rate_p,rate_q,rate_r,");
// #ifdef BOARD_BEBOP
//   fprintf(file, "rpm_obs_1,rpm_obs_2,rpm_obs_3,rpm_obs_4,");
//   fprintf(file, "rpm_ref_1,rpm_ref_2,rpm_ref_3,rpm_ref_4\n");
// #endif
// fprintf(file,"\n");

// #ifdef INS_EXT_POSE_H
//   ins_ext_pos_log_header(file);
// #endif

// #ifdef COMMAND_THRUST
//   fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
// #else
//   fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
// #endif
}
/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of logger_file_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void logger_file_write_row(FILE *file) {
  // struct NedCoor_f *pos = stateGetPositionNed_f();
  // struct NedCoor_f *vel = stateGetSpeedNed_f();
  // struct FloatEulers *att = stateGetNedToBodyEulers_f();
  // struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
#ifdef BOARD_BEBOP
  fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_obs[0],actuators_bebop.rpm_obs[1],actuators_bebop.rpm_obs[2],actuators_bebop.rpm_obs[3]);
  fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_ref[0],actuators_bebop.rpm_ref[1],actuators_bebop.rpm_ref[2],actuators_bebop.rpm_ref[3]);
#endif
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_data(file);
#endif
#ifdef COMMAND_THRUST
  fprintf(file, "%d,%d,%d,%d\n",
      stabilization.cmd[COMMAND_THRUST], stabilization.cmd[COMMAND_ROLL],
      stabilization.cmd[COMMAND_PITCH], stabilization.cmd[COMMAND_YAW]);
#else
  fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif
}


/** Start the file logger and open a new file */
void logger_file_start(void)
{
  printf("STARTED LOGGER\n");
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
