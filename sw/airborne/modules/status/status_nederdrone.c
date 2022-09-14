/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file modules/status/status_nederdrone.c
 * Status information for the Nederdrone logging and datalink
 *
 */

#include "status_nederdrone.h"
#include "mcu_periph/sys_time.h"
#include "modules/loggers/sdlog_chibios.h"
#include "gps/gps.h"
#include "actuators/actuators_uavcan.h"
#include "modules/core/sys_mon_rtos.h"
#include "pprz_version.h"

static bool debug_enable = true;

void status_nederdrone_init(void) {
  sdLogWriteLog(pprzLogFile, "[%.5f] Nederdrone status module started\n", get_sys_time_float());
}

/**
 * @brief Check the GPS status
 * - Checks if we lose (and or gain) GPS fix
 */
static void status_nederdrone_gps(void) {
  static bool gps_valid = false;

  if(gps_valid != GpsFixValid()) {
    if(!GpsFixValid())
      sdLogWriteLog(pprzLogFile, "[%.5f] GPS lost [%lu]\n", get_sys_time_float(), gps.last_msg_time);
    else if(debug_enable)
      sdLogWriteLog(pprzLogFile, "[%.5f] GPS (re)gained [%lu]\n", get_sys_time_float(), gps.last_msg_time);
    
    gps_valid = GpsFixValid();
  }
}

/**
 * @brief Check the uavcan busses status
 * - Check if there are any transmit errors
 */
static void status_nederdrone_uavcan(void) {
#if UAVCAN_USE_CAN1
  static uint32_t error_count1 = 0;

  // Error count changes
  if(error_count1 != (uavcan1.transmit_err_cnt + uavcan1.transmit_err_flush_cnt)) {
    sdLogWriteLog(pprzLogFile, "[%.5f] Errors on uavcan1 interface [transmit_err: %lu, transmit_flush_err: %lu]\n", get_sys_time_float(), uavcan1.transmit_err_cnt, uavcan1.transmit_err_flush_cnt);
    error_count1 = uavcan1.transmit_err_cnt + uavcan1.transmit_err_flush_cnt;
  }
#endif

#if UAVCAN_USE_CAN1
  static uint32_t error_count2 = 0;

  // Error count changes
  if(error_count2 != (uavcan2.transmit_err_cnt + uavcan2.transmit_err_flush_cnt)) {
    sdLogWriteLog(pprzLogFile, "[%.5f] Errors on uavcan2 interface [transmit_err: %lu, transmit_flush_err: %lu]\n", get_sys_time_float(), uavcan2.transmit_err_cnt, uavcan2.transmit_err_flush_cnt);
    error_count2 = uavcan2.transmit_err_cnt + uavcan2.transmit_err_flush_cnt;
  }
#endif
}

static void status_nederdrone_actuator_uavcan(uint8_t bus_num, struct actuators_uavcan_telem_t *telem, uint8_t uavcan_nb, uint32_t *error_count, uint8_t min_id, uint8_t max_id) {
  static const float esc_telem_timeout = 0.80f;
  float cur_timeout = get_sys_time_float() - esc_telem_timeout;
  bool found_id[max_id-min_id+1];
  for(uint8_t i  = 0; i <= (max_id-min_id); i++)
    found_id[i] = false;

  for(uint8_t i = 0; i < uavcan_nb; i++) {
    // Check if we found an expected id
    if(telem[i].node_id > 0) {
      if(telem[i].node_id >= min_id && telem[i].node_id <= max_id) {
        // Found an expected id
        found_id[telem[i].node_id-min_id] = true;

        // Check if we have a timeout
        if(telem[i].timestamp < cur_timeout) {
          sdLogWriteLog(pprzLogFile, "[%.5f] Lost telemetry of uavcan%u %d [node_id: %d, timestamp: %f]\n", get_sys_time_float(), bus_num, i, telem[i].node_id, telem[i].timestamp);
        }
      } else {
        // Got an unexpected ID
        sdLogWriteLog(pprzLogFile, "[%.5f] Unexpected ID on uavcan%u actuator %d [node_id: %d, timestamp: %f, volt: %fV, current: %fA, temp: %fC, rpm: %ld, errors: %lu]\n",
          get_sys_time_float(), bus_num, i, telem[i].node_id, telem[i].timestamp, telem[i].voltage, telem[i].current,
          (telem[i].temperature - 274.15f), telem[i].rpm, telem[i].error_count);
      }
    }

    // Error count changes
    if(error_count[i] != telem[i].error_count) {
      sdLogWriteLog(pprzLogFile, "[%.5f] Errors on uavcan%u actuator %d [node_id: %d, timestamp: %f, volt: %fV, current: %fA, temp: %fC, rpm: %ld, errors: %lu]\n",
        get_sys_time_float(), bus_num, i, telem[i].node_id, telem[i].timestamp, telem[i].voltage, telem[i].current,
        (telem[i].temperature - 274.15f), telem[i].rpm, telem[i].error_count);
      error_count[i] = telem[i].error_count;
    }
  }

  // Check if we found all the expected IDs
  for(uint8_t i = min_id; i < max_id; i++) {
    if(!found_id[i-min_id]) {
      // Missing an ID
      sdLogWriteLog(pprzLogFile, "[%.5f] Missing an ID on uavcan%u %d\n", get_sys_time_float(), bus_num, i);
    }
  }
}

/**
 * @brief Check the actuator status
 * - Check if all actuators still publish status information
 * - Check if there are errors in the uavcan interface
 */
static void status_nederdrone_actuators(void) {
  /* Check the UAVCAN actuators on bus 1 */
#ifdef SERVOS_UAVCAN1_NB
  static uint32_t error_count1[SERVOS_UAVCAN1_NB] = {0};

  // Expected IDs: 1->0,2->1,3->2(left front) 4->3,5->4,6->5(right front)
  status_nederdrone_actuator_uavcan(1, uavcan1_telem, SERVOS_UAVCAN1_NB, error_count1, 1, 6);
#endif

  /* Check the UAVCAN actuators on bus 2 */
#ifdef SERVOS_UAVCAN2_NB
  static uint32_t error_count2[SERVOS_UAVCAN2_NB] = {0};

  // Expected IDS: 11->0,12->1,13->2(left back) 14->3,15->4,16->5(right back) (Nederdrone 4)
  // Expected IDS: 10->6,11->0,12->1,13->2(left back) 14->3,15->4,16->5,17->7(right back) (Nederdrone 6+)
  status_nederdrone_actuator_uavcan(2, uavcan2_telem, SERVOS_UAVCAN2_NB, error_count2, 10, 17);
#endif
}

/**
 * @brief Check the airspeed status
 * - Checks if the airspeed hangs
 */
static void status_nederdrone_airspeed(void) {

}

/**
 * @brief Check the system performance
 * 
 */
static void status_nederdrone_sysmon(void) {
  static uint8_t cnt = 0;

  if(rtos_mon.cpu_load > 85 || (debug_enable && cnt++ > 10)) {
    sdLogWriteLog(pprzLogFile, "Data reported in the RTOS_MON message:\r\n");
    sdLogWriteLog(pprzLogFile, " core free mem: %lu\r\n", rtos_mon.core_free_memory);
    sdLogWriteLog(pprzLogFile, " heap free mem: %lu\r\n", rtos_mon.heap_free_memory);
    sdLogWriteLog(pprzLogFile, " heap fragments: %lu\r\n", rtos_mon.heap_fragments);
    sdLogWriteLog(pprzLogFile, " heap largest: %lu\r\n", rtos_mon.heap_largest);
    sdLogWriteLog(pprzLogFile, " CPU load: %d %%\r\n", rtos_mon.cpu_load);
    sdLogWriteLog(pprzLogFile, " number of threads: %d\r\n", rtos_mon.thread_counter);
    sdLogWriteLog(pprzLogFile, " thread names: %s\r\n", rtos_mon.thread_names);
    for (int i = 0; i < rtos_mon.thread_counter; i++) {
      sdLogWriteLog(pprzLogFile, " thread %d load: %0.1f, free stack: %d\r\n", i,
              (float)rtos_mon.thread_load[i] / 10.f, rtos_mon.thread_free_stack[i]);
    }
    sdLogWriteLog(pprzLogFile, " CPU time: %.2f\r\n", rtos_mon.cpu_time);

    cnt = 0;
  }
}

static void status_nederdrone_i2c(void) {
  
}

void status_nederdrone_period(void) {
  if(pprzLogFile == -1)
    return;
  static uint8_t cnt = 0;

  /* Check the system load */
  status_nederdrone_sysmon();

  /* Check the I2C status */
  status_nederdrone_i2c();

  /* Check the GPS status */
  status_nederdrone_gps();

  /* Check the battery level */
  /* Check the RC */
  /* Check the datalink */

  /* Check the uavcan status */
  status_nederdrone_uavcan();

  /* Check the actuators */
  status_nederdrone_actuators();

  /* Check the airspeed */
  status_nederdrone_airspeed();

  /* Flush the log to make sure data is written */
  if(cnt++ > 10) {
    sdLogWriteLog(pprzLogFile, "[%.5f] %s status module running (AC_ID: %u,ver: %s)\n", get_sys_time_float(), AIRFRAME_NAME, AC_ID, PPRZ_VERSION_DESC);
    sdLogFlushLog(pprzLogFile);
    cnt = 0;
  }
}
