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
#include <pthread.h>
#include <string.h>
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

// Async logging structures
struct logger_data {
    float timestamp;
    bool guided;
    float dt;
    float Ts;
    float voltage;
    int32_t throttle;
    float spec_thrust_sp;
    struct FloatVect3 acc_f;
    struct FloatVect3 pos_ref;
    struct FloatVect3 vel_ref;
    struct FloatVect3 accel_ref;
    float psi_ref;
    struct FloatRates rates_ref;
    struct NedCoor_f pos;
    struct FloatVect3 vel_sp;
    struct NedCoor_f vel;
    struct FloatVect3 accel_sp;
    struct FloatVect3 f_cmd;
    struct FloatVect3 accel_filt;
    struct FloatQuat quat_sp;
    struct FloatQuat quat;
    struct FloatRates rates_sp;
    struct FloatRates rates;
    struct FloatRates ang_accel_sp;
    float ang_accel_filt[3];
    float act_cmd[4];
    float act_state[4];
    float act_state_filt[4];
};

#define LOG_QUEUE_SIZE 512
static struct logger_data log_queue[LOG_QUEUE_SIZE];
static volatile int log_q_head = 0;
static volatile int log_q_tail = 0;

static pthread_t log_thread;
static pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t log_cond = PTHREAD_COND_INITIALIZER;
static volatile bool log_thread_running = false;

static void *logger_file_thread_func(void *arg) {
    FILE *file = (FILE *)arg;
    while (1) {
        pthread_mutex_lock(&log_mutex);
        while (log_q_tail == log_q_head && log_thread_running) {
            pthread_cond_wait(&log_cond, &log_mutex);
        }
        
        if (log_q_tail == log_q_head && !log_thread_running) {
            pthread_mutex_unlock(&log_mutex);
            break;
        }

        struct logger_data data = log_queue[log_q_tail];
        log_q_tail = (log_q_tail + 1) % LOG_QUEUE_SIZE;
        pthread_mutex_unlock(&log_mutex);

        fprintf(file, "%f", data.timestamp);
        fprintf(file, ",%d", data.guided);
        fprintf(file, ",%f", data.dt);
        fprintf(file, ",%f", data.Ts);
        fprintf(file, ",%f", data.voltage);
        fprintf(file, ",%d", data.throttle);
        fprintf(file, ",%f", data.spec_thrust_sp);
        fprintf(file, ",%f,%f,%f", data.acc_f.x, data.acc_f.y, data.acc_f.z);

        fprintf(file, ",%f,%f,%f", data.pos_ref.x, data.pos_ref.y, data.pos_ref.z);
        fprintf(file, ",%f,%f,%f", data.vel_ref.x, data.vel_ref.y, data.vel_ref.z);
        fprintf(file, ",%f,%f,%f", data.accel_ref.x, data.accel_ref.y, data.accel_ref.z);
        fprintf(file, ",%f", data.psi_ref);
        fprintf(file, ",%f,%f,%f", data.rates_ref.p, data.rates_ref.q, data.rates_ref.r);

        fprintf(file, ",%f,%f,%f", data.pos.x, data.pos.y, data.pos.z);
        fprintf(file, ",%f,%f,%f", data.vel_sp.x, data.vel_sp.y, data.vel_sp.z);
        fprintf(file, ",%f,%f,%f", data.vel.x, data.vel.y, data.vel.z);
        fprintf(file, ",%f,%f,%f", data.accel_sp.x, data.accel_sp.y, data.accel_sp.z);
        fprintf(file, ",%f,%f,%f", data.f_cmd.x, data.f_cmd.y, data.f_cmd.z);
        fprintf(file, ",%f,%f,%f", data.accel_filt.x, data.accel_filt.y, data.accel_filt.z);

        fprintf(file, ",%f,%f,%f,%f", data.quat_sp.qi, data.quat_sp.qx, data.quat_sp.qy, data.quat_sp.qz);
        fprintf(file, ",%f,%f,%f,%f", data.quat.qi, data.quat.qx, data.quat.qy, data.quat.qz);
        fprintf(file, ",%f,%f,%f", data.rates_sp.p, data.rates_sp.q, data.rates_sp.r);
        fprintf(file, ",%f,%f,%f", data.rates.p, data.rates.q, data.rates.r);
        fprintf(file, ",%f,%f,%f", data.ang_accel_sp.p, data.ang_accel_sp.q, data.ang_accel_sp.r);
        fprintf(file, ",%f,%f,%f", data.ang_accel_filt[0], data.ang_accel_filt[1], data.ang_accel_filt[2]);
        fprintf(file, ",%f,%f,%f,%f", data.act_cmd[0], data.act_cmd[1], data.act_cmd[2], data.act_cmd[3]);
        fprintf(file, ",%f,%f,%f,%f", data.act_state[0], data.act_state[1], data.act_state[2], data.act_state[3]);
        fprintf(file, ",%f,%f,%f,%f", data.act_state_filt[0], data.act_state_filt[1], data.act_state_filt[2], data.act_state_filt[3]);

        fprintf(file, "\n");
    }
    return NULL;
}

/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in logger_file_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
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
    fprintf(file, ",rates_ref_p,rates_ref_q,rates_ref_r");

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
    if (!log_thread_running) return;

    pthread_mutex_lock(&log_mutex);
    int next_head = (log_q_head + 1) % LOG_QUEUE_SIZE;
    if (next_head != log_q_tail) {
        struct logger_data *data = &log_queue[log_q_head];
        data->timestamp = dbg.timestamp;
        data->guided = dbg.guided;
        data->dt = dbg.dt;
        data->Ts = dbg.Ts;
        data->voltage = dbg.voltage;
        data->throttle = dbg.throttle;
        data->spec_thrust_sp = dbg.spec_thrust_sp;

        struct NedCoor_f *pos = stateGetPositionNed_f();
        struct NedCoor_f *vel = stateGetSpeedNed_f();
        struct Int32Vect3 *acc_i = stateGetAccelBody_i();
        ACCELS_FLOAT_OF_BFP(data->acc_f, *acc_i);

        data->pos_ref = dbg.pos_ref;
        data->vel_ref = dbg.vel_ref;
        data->accel_ref = dbg.accel_ref;
        data->psi_ref = dbg.psi_ref;
        data->rates_ref = dbg.rates_ref;
        data->pos = *pos;
        data->vel_sp = dbg.vel_sp;
        data->vel = *vel;
        data->accel_sp = dbg.accel_sp;
        data->f_cmd = dbg.f_cmd;
        data->accel_filt = dbg.accel_filt;

        data->quat_sp = *dbg.quat_sp;
        data->quat = *dbg.quat;
        data->rates_sp = *dbg.rates_sp;
        data->rates = *dbg.rates;
        data->ang_accel_sp = *dbg.ang_accel_sp;

        for (int i = 0; i < 3; i++) data->ang_accel_filt[i] = dbg.ang_accel_filt[i];
        for (int i = 0; i < 4; i++) {
            data->act_cmd[i] = dbg.act->cmd[i];
            data->act_state[i] = dbg.act->state[i];
            data->act_state_filt[i] = dbg.act->state_filt[i];
        }

        log_q_head = next_head;
        pthread_cond_signal(&log_cond);
    }
    pthread_mutex_unlock(&log_mutex);
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

  //setvbuf(logger_file, NULL, _IOFBF, 65536);

  printf("[logger_file] Start logging to %s...\n", filename);

  logger_file_write_header(logger_file);

  log_q_head = 0;
  log_q_tail = 0;
  log_thread_running = true;
  pthread_create(&log_thread, NULL, logger_file_thread_func, logger_file);
}

/** Stop the logger an nicely close the file */
void logger_file_stop(void)
{
  if (logger_file != NULL) {
    if (log_thread_running) {
        pthread_mutex_lock(&log_mutex);
        log_thread_running = false;
        pthread_cond_signal(&log_cond);
        pthread_mutex_unlock(&log_mutex);
        pthread_join(log_thread, NULL);
    }
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