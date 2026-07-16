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

#include "peripherals/dps310_i2c.h"

/**
 * DPS310 temperature errata fix.
 *
 * Some DPS310 silicon revisions report temperature roughly 60 degC too high
 * this corrupts the temperature-compensated pressure. Writing this specific 
 * sequence of register writes fixes the issue.
 */
static const uint8_t dps310_temp_fix_seq[][2] = {
  { 0x0E, 0xA5 },
  { 0x0F, 0x96 },
  { 0x62, 0x02 },
  { 0x0E, 0x00 },
  { 0x0F, 0x00 }
};

#define DPS310_TEMP_FIX_SEQ_LEN (sizeof(dps310_temp_fix_seq) / sizeof(dps310_temp_fix_seq[0]))

static int32_t getTwosComplement(uint32_t raw, uint8_t length)
{
  if (raw & ((uint32_t)1 << (length - 1))) {
    return ((int32_t)raw) - ((int32_t)1 << length);
  }
  return raw;
}

static void parse_calib_data(struct Dps310_I2c *dps)
{
  uint8_t *coef = (uint8_t *)dps->i2c_trans.buf;

  dps->calib.c0 = getTwosComplement(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12);
  dps->calib.c1 = getTwosComplement((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);
  dps->calib.c00 = getTwosComplement(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((
                                       uint32_t)coef[5] >> 4) & 0x0F), 20);
  dps->calib.c10 = getTwosComplement((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7],
                                     20);
  dps->calib.c01 = getTwosComplement(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);
  dps->calib.c11 = getTwosComplement(((uint32_t)coef[10] << 8) | (uint32_t)coef[11], 16);
  dps->calib.c20 = getTwosComplement(((uint32_t)coef[12] << 8) | (uint32_t)coef[13], 16);
  dps->calib.c21 = getTwosComplement(((uint32_t)coef[14] << 8) | (uint32_t)coef[15], 16);
  dps->calib.c30 = getTwosComplement(((uint32_t)coef[16] << 8) | (uint32_t)coef[17], 16);

  // DPS310 ignores c31 and c40 (only used on SPL07_003)
  dps->calib.c31 = 0;
  dps->calib.c40 = 0;
}

static void parse_sensor_data(struct Dps310_I2c *dps)
{
  uint32_t data_xlsb, data_lsb, data_msb;
  uint32_t prs_raw, tmp_raw;

  // Registers 0x00 to 0x02 for pressure
  data_msb = (uint32_t)dps->i2c_trans.buf[0] << 16;
  data_lsb = (uint32_t)dps->i2c_trans.buf[1] << 8;
  data_xlsb = (uint32_t)dps->i2c_trans.buf[2];
  prs_raw = data_msb | data_lsb | data_xlsb;
  dps->raw_pressure = getTwosComplement(prs_raw, 24);

  // Registers 0x03 to 0x05 for temperature
  data_msb = (uint32_t)dps->i2c_trans.buf[3] << 16;
  data_lsb = (uint32_t)dps->i2c_trans.buf[4] << 8;
  data_xlsb = (uint32_t)dps->i2c_trans.buf[5];
  tmp_raw = data_msb | data_lsb | data_xlsb;
  dps->raw_temperature = getTwosComplement(tmp_raw, 24);
}

static void compensate_sensor(struct Dps310_I2c *dps)
{
  // Standard scaling for 16x oversampling
  float kT = 253952.0f;
  float kP = 253952.0f;

  float Traw_sc = (float)dps->raw_temperature / kT;
  float Praw_sc = (float)dps->raw_pressure / kP;

  struct dps310_reg_calib_data *c = &dps->calib;

  dps->temperature = (c->c0 * 0.5f) + (c->c1 * Traw_sc);
  dps->pressure = c->c00 + Praw_sc * (c->c10 + Praw_sc * (c->c20 + Praw_sc * c->c30)) +
                  Traw_sc * c->c01 + Traw_sc * Praw_sc * (c->c11 + Praw_sc * c->c21);
}

void dps310_i2c_init(struct Dps310_I2c *dps, struct i2c_periph *i2c_p, uint8_t addr)
{
  dps->i2c_p = i2c_p;
  dps->i2c_trans.slave_addr = addr;
  dps->i2c_trans.status = I2CTransDone;
  dps->data_available = false;
  dps->initialized = false;
  dps->status = DPS310_STATUS_UNINIT;
  dps->temp_coef_srce = 0;
  dps->temp_fix_step = 0;
}

void dps310_i2c_periodic(struct Dps310_I2c *dps)
{
  if (dps->i2c_trans.status != I2CTransDone) {
    return;
  }

  switch (dps->status) {
    case DPS310_STATUS_UNINIT:
      dps->data_available = false;
      dps->initialized = false;
      dps->temp_fix_step = 0;
      dps->status = DPS310_STATUS_GET_ID;
      break;

    case DPS310_STATUS_GET_ID:
      dps->i2c_trans.buf[0] = DPS310_REG_ID;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 1);
      break;

    case DPS310_STATUS_TEMP_FIX:
      dps->i2c_trans.buf[0] = dps310_temp_fix_seq[dps->temp_fix_step][0];
      dps->i2c_trans.buf[1] = dps310_temp_fix_seq[dps->temp_fix_step][1];
      i2c_transmit(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 2);
      break;

    case DPS310_STATUS_GET_COEF_SRCE:
      dps->i2c_trans.buf[0] = DPS310_REG_COEF_SRCE;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 1);
      break;

    case DPS310_STATUS_GET_CALIB:
      dps->i2c_trans.buf[0] = DPS310_REG_COEF;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 18);
      break;

    case DPS310_STATUS_CONFIGURE_REGS:
      dps->i2c_trans.buf[0] = DPS310_REG_PRS_CFG;
      // Sensor-internal background rates, decoupled from the (e.g. 50Hz) periodic polling rate.
      // Datasheet budget: total conversion time < 1s/s. At 16x oversampling (27.6ms/conversion),
      // 16Hz P + 16Hz T = 883ms/s, so 16Hz is the maximum legal rate for both channels.
      // Note: 16x oversampling requires P_SHIFT/T_SHIFT below and kT=kP=253952 in compensate_sensor().
      dps->i2c_trans.buf[1] = DPS310_PRS_CFG_PM_RATE_16HZ | DPS310_PRS_CFG_PM_PRC_16;
      dps->i2c_trans.buf[2] = DPS310_TMP_CFG_TMP_RATE_16HZ | DPS310_TMP_CFG_TMP_PRC_16 | dps->temp_coef_srce;
      dps->i2c_trans.buf[3] = 0x00; // Idle MEAS_CFG initially until CFG_REG is correctly established below!
      dps->i2c_trans.buf[4] = DPS310_CFG_REG_P_SHIFT | DPS310_CFG_REG_T_SHIFT;
      i2c_transmit(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 5);
      break;

    case DPS310_STATUS_CONFIGURE_MEAS:
      // Explicitly enable continuous measurements AFTER CFG_REG is written to avoid start-up timing bugs
      dps->i2c_trans.buf[0] = DPS310_REG_MEAS_CFG;
      dps->i2c_trans.buf[1] = DPS310_MEAS_CTRL_CONT;
      i2c_transmit(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 2);
      break;

    case DPS310_STATUS_READ_DATA:
      dps->i2c_trans.buf[0] = DPS310_REG_PSR_B2;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 6);
      break;

    default:
      break;
  }
}

void dps310_i2c_event(struct Dps310_I2c *dps)
{
  if (dps->i2c_trans.status == I2CTransSuccess) {
    switch (dps->status) {
      case DPS310_STATUS_GET_ID:
        // Apply the temperature errata fix only on genuine DPS310 silicon.
        // Register-compatible parts (e.g. SPL07-003, ID 0x11) skip it.
        if (dps->i2c_trans.buf[0] == DPS310_CHIP_ID) {
          dps->temp_fix_step = 0;
          dps->status = DPS310_STATUS_TEMP_FIX;
        } else {
          dps->status = DPS310_STATUS_GET_COEF_SRCE;
        }
        break;

      case DPS310_STATUS_TEMP_FIX:
        dps->temp_fix_step++;
        if (dps->temp_fix_step >= DPS310_TEMP_FIX_SEQ_LEN) {
          dps->status = DPS310_STATUS_GET_COEF_SRCE;
        }
        break;

      case DPS310_STATUS_GET_COEF_SRCE:
        // Isolate the bit handling internal/external temperature sensor src logic
        dps->temp_coef_srce = dps->i2c_trans.buf[0] & DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE;
        dps->status = DPS310_STATUS_GET_CALIB;
        break;

      case DPS310_STATUS_GET_CALIB:
        parse_calib_data(dps);
        dps->status = DPS310_STATUS_CONFIGURE_REGS;
        break;

      case DPS310_STATUS_CONFIGURE_REGS:
        dps->status = DPS310_STATUS_CONFIGURE_MEAS;
        break;

      case DPS310_STATUS_CONFIGURE_MEAS:
        dps->status = DPS310_STATUS_READ_DATA;
        dps->initialized = true;
        break;

      case DPS310_STATUS_READ_DATA:
        parse_sensor_data(dps);
        compensate_sensor(dps);
        dps->data_available = true;
        break;

      default:
        break;
    }
    dps->i2c_trans.status = I2CTransDone;
  } else if (dps->i2c_trans.status == I2CTransFailed) {
    if (!dps->initialized) {
      dps->status = DPS310_STATUS_UNINIT;
    }
    dps->i2c_trans.status = I2CTransDone;
  }
}
