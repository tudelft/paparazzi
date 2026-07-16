/*
 * Copyright (C) 2026 OpenUAS
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
 *
 */

#ifndef DPS310_I2C_H
#define DPS310_I2C_H

#include "peripherals/dps310_regs.h"
#include "mcu_periph/i2c.h"

enum Dps310Status {
  DPS310_STATUS_UNINIT,
  DPS310_STATUS_GET_ID,
  DPS310_STATUS_TEMP_FIX,
  DPS310_STATUS_GET_COEF_SRCE,
  DPS310_STATUS_GET_CALIB,
  DPS310_STATUS_CONFIGURE_REGS,
  DPS310_STATUS_CONFIGURE_MEAS,
  DPS310_STATUS_READ_DATA
};

struct Dps310_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Dps310Status status;           ///< state machine status
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  struct dps310_reg_calib_data calib; ///< calibration data
  uint8_t temp_coef_srce;          ///< temp coef source
  uint8_t temp_fix_step;           ///< current step of the temperature errata fix sequence

  int32_t raw_pressure;            ///< uncompensated pressure
  int32_t raw_temperature;         ///< uncompensated temperature
  float pressure;                   ///< pressure in Pascal
  float temperature;                ///< temperature in deg Celcius
};

extern void dps310_i2c_init(struct Dps310_I2c *dps, struct i2c_periph *i2c_p, uint8_t addr);
extern void dps310_i2c_periodic(struct Dps310_I2c *dps);
extern void dps310_i2c_event(struct Dps310_I2c *dps);

#endif /* DPS310_I2C_H */
