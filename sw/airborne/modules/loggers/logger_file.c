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
// static void logger_file_write_header(FILE *file) {
//   fprintf(file, "timestamp,");
//   // fprintf(file, "pos_x,pos_y,pos_z,");
//   // fprintf(file, "vel_x,vel_y,vel_z,");
//   fprintf(file, "acc_imu_x,acc_imu_y,acc_imu_z,");
//   fprintf(file, "rate_imu_p,rate_imu_q,rate_imu_r,");
//   fprintf(file, "att_cmd_phi,att_cmd_theta,att_cmd_psi,");
//   fprintf(file, "att_phi,att_theta,att_psi,");
//   fprintf(file, "rate_cmd_p,rate_cmd_q,rate_cmd_r,");
//   fprintf(file, "rate_filt_p,rate_filt_q,rate_filt_r,");
//   fprintf(file, "ang_accel_cmd_pdot,ang_accel_cmd_qdot,ang_accel_cmd_rdot,");
//   fprintf(file, "ang_accel_pdot,ang_accel_qdot,ang_accel_rdot,");
//   fprintf(file, "act_cmd_TL,act_cmd_TR,act_cmd_BR,act_cmd_BL\n");
// }

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of logger_file_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
// extern struct FloatEulers dbg_stab_att_sp_euler_f;
// extern struct FloatRates dbg_rate_sp;
// extern struct FloatRates dbg_rates_filt;
// extern struct FloatRates angular_accel_ref;
// extern float angular_acceleration[3];
//
// static void logger_file_write_row(FILE *file) {
//   struct FloatVect3 acc_f;
//   struct FloatEulers att;
//   // struct NedCoor_f *pos = stateGetPositionNed_f();
//   // struct NedCoor_f *vel = stateGetSpeedNed_f();
//   struct Int32Vect3 *acc_i = stateGetAccelBody_i();
//   ACCELS_FLOAT_OF_BFP(acc_f, *acc_i);
//   float_eulers_of_quat_zxy(&att, stateGetNedToBodyQuat_f());
//   struct FloatRates *rates = stateGetBodyRates_f();

//   fprintf(file, "%f,", get_sys_time_float());
//   // fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
//   // fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
//   fprintf(file, "%f,%f,%f,", acc_f.x, acc_f.y, acc_f.z);
//   fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
//   fprintf(file, "%f,%f,%f,", dbg_stab_att_sp_euler_f.phi, dbg_stab_att_sp_euler_f.theta, dbg_stab_att_sp_euler_f.psi);
//   fprintf(file, "%f,%f,%f,", att.phi, att.theta, att.psi);
//   fprintf(file, "%f,%f,%f,", dbg_rate_sp.p, dbg_rate_sp.q, dbg_rate_sp.r);
//   fprintf(file, "%f,%f,%f,", dbg_rates_filt.p, dbg_rates_filt.q, dbg_rates_filt.r);
//   fprintf(file, "%f,%f,%f,", angular_accel_ref.p, angular_accel_ref.q, angular_accel_ref.r);
//   fprintf(file, "%f,%f,%f,", angular_acceleration[0], angular_acceleration[1], angular_acceleration[2]);
//   fprintf(file, "%d,%d,%d,%d\n", actuators_pprz[0], actuators_pprz[1], actuators_pprz[2], actuators_pprz[3]);
// }

static void logger_file_write_header(FILE *file) {
    
    //metadata
    fprintf(file, "#Kq,%.1f,%.1f,%.1f\n", dbg.Kq.x, dbg.Kq.y, dbg.Kq.z);
    fprintf(file, "#Komega,%.1f,%.1f,%.1f\n", dbg.Komega.x, dbg.Komega.y, dbg.Komega.z);
    fprintf(file, "#Kp,%.1f,%.1f,%.1f\n", dbg.Kp.x, dbg.Kp.y, dbg.Kp.z);
    fprintf(file, "#Kv,%.1f,%.1f,%.1f\n", dbg.Kv.x, dbg.Kv.y, dbg.Kv.z);
    fprintf(file, "#MU_X_v_1e_8,%.2f\n", dbg.MU_X_v*1e8);
    fprintf(file, "#MU_Y_v_1e_8,%.2f\n", dbg.MU_Y_v*1e8);
    fprintf(file, "#MU_Z_v_1e_8,%.2f\n", dbg.MU_Z_v*1e8);
    fprintf(file, "#C_T_v_1e_8,%.2f\n", dbg.C_T_v*1e8);

    // data
    fprintf(file, "timestamp");
    fprintf(file, ",guided");
    fprintf(file, ",dt");
    fprintf(file, ",Ts");
    fprintf(file, ",voltage");
    fprintf(file, ",throttle");
    fprintf(file, ",spec_thrust_sp");
    fprintf(file, ",acc_x,acc_y,acc_z");

    fprintf(file, ",pos_ref_n,pos_ref_e,pos_ref_d");
    fprintf(file, ",vel_ref_n,vel_ref_e,vel_ref_d");
    fprintf(file, ",acc_ref_n,acc_ref_e,acc_ref_d");
    fprintf(file, ",psi_ref");
    fprintf(file, ",pos_n,pos_e,pos_d");
    fprintf(file, ",vel_sp_n,vel_sp_e,vel_sp_d");
    fprintf(file, ",vel_n,vel_e,vel_d");
    fprintf(file, ",acc_sp_n,acc_sp_e,acc_sp_d");
    fprintf(file, ",f_cmd_n,f_cmd_e,f_cmd_d");
    fprintf(file, ",acc_filt_n,acc_filt_e,acc_filt_d");
    
    fprintf(file, ",qs_sp,qx_sp,qy_sp,qz_sp");
    fprintf(file, ",qs,qx,qy,qz");
    fprintf(file, ",rates_p_sp,rates_q_sp,rates_r_sp");
    fprintf(file, ",rates_p,rates_q,rates_r");
    fprintf(file, ",pdot_sp,qdot_sp,rdot_sp");
    fprintf(file, ",pdot_filt,qdot_filt,rdot_filt");
    fprintf(file, ",act_cmd_1,act_cmd_2,act_cmd_3,act_cmd_4");
    fprintf(file, ",act_state_1,act_state_2,act_state_3,act_state_4");
    fprintf(file, ",act_state_filt_1,act_state_filt_2,act_state_filt_3,act_state_filt_4");
    
    fprintf(file, "\n");
}

static void logger_file_write_row(FILE *file) {
    struct NedCoor_f *pos = stateGetPositionNed_f();
    struct NedCoor_f *vel = stateGetSpeedNed_f();
    struct FloatVect3 acc_f;
    struct Int32Vect3 *acc_i = stateGetAccelBody_i();
    ACCELS_FLOAT_OF_BFP(acc_f, *acc_i);

    fprintf(file, "%f", dbg.timestamp);
    fprintf(file, ",%d", dbg.guided);
    fprintf(file, ",%f", dbg.dt);
    fprintf(file, ",%f", dbg.Ts);
    fprintf(file, ",%f", dbg.voltage);
    fprintf(file, ",%d", dbg.throttle);
    fprintf(file, ",%f", dbg.spec_thrust_sp);
    fprintf(file, ",%f,%f,%f", acc_f.x, acc_f.y, acc_f.z);

    fprintf(file, ",%f,%f,%f", dbg.pos_ref.x, dbg.pos_ref.y, dbg.pos_ref.z);
    fprintf(file, ",%f,%f,%f", dbg.vel_ref.x, dbg.vel_ref.y, dbg.vel_ref.z);
    fprintf(file, ",%f,%f,%f", dbg.accel_ref.x, dbg.accel_ref.y, dbg.accel_ref.z);
    fprintf(file, ",%f", dbg.psi_ref);
    fprintf(file, ",%f,%f,%f", pos->x, pos->y, pos->z);
    fprintf(file, ",%f,%f,%f", dbg.vel_sp.x, dbg.vel_sp.y, dbg.vel_sp.z);
    fprintf(file, ",%f,%f,%f", vel->x, vel->y, vel->z);
    fprintf(file, ",%f,%f,%f", dbg.accel_sp.x, dbg.accel_sp.y, dbg.accel_sp.z);
    fprintf(file, ",%f,%f,%f", dbg.f_cmd.x, dbg.f_cmd.y, dbg.f_cmd.z);
    fprintf(file, ",%f,%f,%f", dbg.accel_filt.x, dbg.accel_filt.y, dbg.accel_filt.z);

    fprintf(file, ",%f,%f,%f,%f", dbg.quat_sp->qi, dbg.quat_sp->qx, dbg.quat_sp->qy, dbg.quat_sp->qz);
    fprintf(file, ",%f,%f,%f,%f", dbg.quat->qi, dbg.quat->qx, dbg.quat->qy, dbg.quat->qz);
    fprintf(file, ",%f,%f,%f", dbg.rates_sp->p, dbg.rates_sp->q, dbg.rates_sp->r);
    fprintf(file, ",%f,%f,%f", dbg.rates->p, dbg.rates->q, dbg.rates->r);
    fprintf(file, ",%f,%f,%f", dbg.ang_accel_sp->p, dbg.ang_accel_sp->q, dbg.ang_accel_sp->r);
    fprintf(file, ",%f,%f,%f", dbg.ang_accel_filt[0], dbg.ang_accel_filt[1], dbg.ang_accel_filt[2]);
    fprintf(file, ",%f,%f,%f,%f", dbg.act->cmd[0], dbg.act->cmd[1], dbg.act->cmd[2], dbg.act->cmd[3]);
    fprintf(file, ",%f,%f,%f,%f", dbg.act->state[0], dbg.act->state[1], dbg.act->state[2], dbg.act->state[3]);
    fprintf(file, ",%f,%f,%f,%f", dbg.act->state_filt[0], dbg.act->state_filt[1], dbg.act->state_filt[2], dbg.act->state_filt[3]);
    
    fprintf(file, "\n");
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
  sprintf(filename, "%s/test.csv", STRINGIFY(LOGGER_FILE_PATH));
  while ((logger_file = fopen(filename, "r"))) {
    fclose(logger_file);

    counter++;
    sprintf(filename, "%s/test_%02d.csv", STRINGIFY(LOGGER_FILE_PATH), counter);
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