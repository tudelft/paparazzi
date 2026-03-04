/*
 * Copyright (C) 2026 OpenUAS <noreply@openuas.org>
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

/** @file modules/sensors/rangefinder_i2c.h
 *  @brief Driver for a rangfinder sensor when used via I2C bus
 */
#ifndef RANGEFINDER_I2C_H
#define RANGEFINDER_I2C_H

#include "stdbool.h"
#include "mcu_periph/i2c.h"

enum RangefinderI2CStatus {
  RANGEFINDER_I2C_REQ_DATA,
  RANGEFINDER_I2C_READ_DATA,
  RANGEFINDER_I2C_PARSE_DATA
};

struct RangefinderI2C {
  struct i2c_transaction trans;
  uint8_t addr;
  enum RangefinderI2CStatus status;
  uint16_t raw; ///< raw measured non scaled range value from sensor
  float distance; ///< Distance scaled to [m]
  bool update_agl; ///< Do or don't update AGL ABI message
};

extern struct RangefinderI2C rangefinder_i2c;

extern void rangefinder_i2c_init(void);
extern void rangefinder_i2c_event(void);
extern void rangefinder_i2c_periodic(void);
extern void rangefinder_i2c_report(void);

#endif /* RANGEFINDER_I2C_H */
