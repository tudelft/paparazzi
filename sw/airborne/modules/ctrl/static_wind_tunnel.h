/*
 * Copyright (C) 2020 Freek van Tienen
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
 * @file modules/ctrl/static_wind_tunnel.h
 * @brief Wind tunnel actuator settings
 *
 * Sets the actuators to a predefined set of values 
 */

#ifndef STATIC_WIND_TUNNEL_H_
#define STATIC_WIND_TUNNEL_H_

#include <std.h>

struct WT_data {
  float measurement_time;
  int32_t commands[6];
  bool run;
  int32_t counter;
  int32_t stage;
};
extern float force_sensor_data.Fx;
extern float force_sensor_data.Fy;
extern float force_sensor_data.Fz;
extern float force_sensor_data.Tx;
extern float force_sensor_data.Ty;
extern float force_sensor_data.Tz;

extern struct WT_data wt_data;
extern bool DYNAMIC_TEST;
extern bool STATIC_WIND_TUNNEL_E;
extern bool STATIC_WIND_TUNNEL_TR;
extern bool STATIC_WIND_TUNNEL_TRE;
extern void wt_init(void);
extern void wt_run(void);
extern float kp;


#endif /* STATIC_WIND_TUNNEL_H_ */

