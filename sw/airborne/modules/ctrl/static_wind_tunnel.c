/*
 * Copyright (C) 2024 Ziqing Ma
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/static_wind_tunnel.c
 * @brief Windtunnel automatic step controller
 *
 */

#include "modules/ctrl/static_wind_tunnel.h"

#define STATIC_WIND_TUNNEL_NUM_CMD 6

struct ForceSensorData {
    float Fx;
    float Fy;
    float Fz;
    float Tx;
    float Ty;
    float Tz;
} force_sensor_data;

struct WT_data wt_data = {
  .measurement_time = 10.0,
  .commands = {0},
  .run = false,
  .counter = 0,
  .stage = 0,
  .dynamic_test = false,
  .vehicle_type = 0,
  .kp = 0.8,
};

void force_sensor_callback(float Fx, float Fy, float Fz, float Tx, float Ty, float Tz) {
    force_sensor_data.Fx = Fx;
    force_sensor_data.Fy = Fy;
    force_sensor_data.Fz = Fz;
    force_sensor_data.Tx = Tx;
    force_sensor_data.Ty = Ty;
    force_sensor_data.Tz = Tz;
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_wt(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_WIND_TUNNEL(trans, dev, AC_ID,
                   &force_sensor_data.Fx,
                   &force_sensor_data.Fy,
                   &force_sensor_data.Fz,
                   &force_sensor_data.Tx,
                   &force_sensor_data.Ty,
                   &force_sensor_data.Tz,
                   6, &wt_data.commands);
}
#endif

void wt_init(void) {
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_TUNNEL, send_wt);
#endif
}

#define STATIC_WIND_TUNNEL_NUM_CONFIGS 65
// [tilt right, aileron right, right motor, left motor, tilt left ]
// Define actuator inputs for each condition in PPRZ units
int32_t configurations[STATIC_WIND_TUNNEL_NUM_CONFIGS][STATIC_WIND_TUNNEL_NUM_CMD] = {
//no motor, no tilt, flap deflected
  {0,0, 0, -9600, -9600, 0},
  {0, 2880, -9600, -9600, 0},
  {0,0, 5760, -9600, -9600, 0},
  {0,0, 9600, -9600, -9600, 0},
  {0,0, -2880, -9600, -9600, 0},
  {0,0, -5760, -9600, -9600, 0},
  {0,0, -9600, -9600, -9600, 0},
  {0,0, 0, 1000, -9600, 0},
// //30% motor, 0, 30%, 60%, 100% flap
//   {0,    0, 2880, -9600, 0},
//   {0, 2880, 2880, -9600, 0},
//   {0, 5760, 2880, -9600, 0},
//   {0, 9600, 2880, -9600, 0},
//   {0, -2880, 2880, -9600, 0},
//   {0, -5760, 2880, -9600, 0},
//   {0, -9600, 2880, -9600, 0},
//   {0, 0, 1000, -9600, 0},
// //50% motor,0, 30%, 60%, 100% flap
//   {0,    0, 4800, -9600, 0},
//   {0, 2880, 4800, -9600, 0},
//   {0, 5760, 4800, -9600, 0},
//   {0, 9600, 4800, -9600, 0},
//   {0, -2880, 4800, -9600, 0},
//   {0, -5760, 4800, -9600, 0},
//   {0, -9600, 4800, -9600, 0},
//   {0, 0, 1000, -9600, 0},
// //70% motor,0, 30%, 60%, 100% flap
//   {0,   0,  6720, -9600, 0},
//   {0, 2880, 6720, -9600, 0},
//   {0, 5760, 6720, -9600, 0},
//   {0, 9600, 6720, -9600, 0},
//   {0, -2880, 6720, -9600, 0},
//   {0, -5760, 6720, -9600, 0},
//   {0, -9600, 6720, -9600, 0},
//   {0, 0, 1000, -9600, 0},
// //no motor,tilt angles, no flap
//   {0, 0, -9600, -9600, 0},
//   {2880, 0, -9600, -9600, 0},
//   {5760, 0, -9600, -9600, 0},
//   {9600, 0, -9600, -9600, 0},
//   {-2880, 0, -9600, -9600, 0},
//   {-5760, 0, -9600, -9600, 0},
//   {-9600, 0, -9600, -9600, 0},
//   {0, 0, -9600, 1000, 0},
// ////30% motor, 0, 30%, 60%, 100% tilt
//   {0,    0, -9600, 2880, 0},
//   {2880, 0, -9600, 2880, 0},
//   {5760, 0, -9600, 2880, 0},
//   {9600, 0, -9600, 2880, 0},
//   {-2880, 0, -9600, 2880,  0},
//   {-5760, 0, -9600, 2880,  0},
//   {-9600, 0, -9600, 2880,  0},
//   {0, 0, -9600, 1000, 0},
// //50% motor,0, 30%, 60%, 100% tilt
//   {0,    0, -9600, 4800, 0},
//   {2880, 0, -9600, 4800, 0},
//   {5760, 0, -9600, 4800, 0},
//   {9600, 0, -9600, 4800, 0},
//   {-2880, 0, -9600, 4800, 0},
//   {-5760, 0, -9600, 4800, 0},
//   {-9600, 0, -9600, 4800, 0},
//   {0, 0, -9600, 1000, 0},
// //70% motor,0, 30%, 60%, 100% tilt
//   {0,    0, -9600, 6720, 0},
//   {2880, 0, -9600, 6720, 0},
//   {5760, 0, -9600, 6720, 0},
//   {9600, 0, -9600,6720, 0},
//   {-2880, 0, -9600, 6720, 0},
//   {-5760, 0, -9600, 6720, 0},
//   {-9600, 0, -9600, 6720, 0},
//   {0,    0, -9600, 4000, 0},//decrease throttle gradually
//   {0,    0, -9600, 800, 0},
};

/**
 * Periodic function
 *
 * wt_data.commands are used in the airframe file to set the actuators
 */
void wt_run(void)
{
  if (wt_data.run) {
    if(wt_data.dynamic_test){
    wt_data.commands[3] = wt_data.commands[2];
    float error = -force_sensor_data.Ty;  // 目标My为0 FIXME: in English please!
    float delta_pprz = wt_data.kp * error;

    // 计算新的PWM值 FIXME: in English please!
    if(wt_data.vehicle_type == E)
    {
    wt_data.commands[0] = 0;
    wt_data.commands[1] = 0;
    wt_data.commands[5] -= delta_pprz*0.1;
    wt_data.commands[4] = -wt_data.commands[4];
    }
    else if(wt_data.vehicle_type == TR)
    {
    wt_data.commands[0] += delta_pprz*0.1;
    wt_data.commands[1] = wt_data.commands[0];
    wt_data.commands[4] = 0;
    wt_data.commands[5] = 0;
    }
    else if(wt_data.vehicle_type == TRE)
    {
    wt_data.commands[0] += delta_pprz*0.1;
    wt_data.commands[1] = wt_data.commands[0];
    wt_data.commands[5] -= delta_pprz*0.1;
    wt_data.commands[4] = -wt_data.commands[4];
    }
    }
    else{
    wt_data.counter = wt_data.counter + 1;

    wt_data.stage = floor((wt_data.counter / STATIC_WIND_TUNNEL_FREQUENCY) / wt_data.measurement_time);

    if (wt_data.stage > (STATIC_WIND_TUNNEL_NUM_CONFIGS - 1)) {
      wt_data.counter = 0;
      wt_data.run = false;
      return;
    }

    for (int i=0; i<STATIC_WIND_TUNNEL_NUM_CMD; i++) {
      wt_data.commands[i] = configurations[wt_data.stage][i];
    }
    }

  } else {
    // Set everything to 0 by default
    for (int i=0; i<STATIC_WIND_TUNNEL_NUM_CMD; i++) {
      wt_data.commands[i] = 0;
    }

    wt_data.counter = 0;
  }

}
