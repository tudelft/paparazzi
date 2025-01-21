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

enum VehicleType {
  E,
  TR,
  TRE
};

struct WT_data {
  float max_stage_time;
  float des_measurement_time;
  int32_t commands[6]; // tilt right, tilt left, motor right, motor left, elevon right, elevon left
  bool run;
  int32_t counter;
  int32_t measurement_counter;
  int32_t wait_for_controller_counter;
  bool dynamic_test;
  enum VehicleType vehicle_type;
  float ki;
  int32_t integrator;
};

extern struct WT_data wt_data;
extern void wt_init(void);
extern void wt_run(void);

extern void wt_parse_force_sensor_dl(uint8_t *buf);


#endif /* STATIC_WIND_TUNNEL_H_ */

