/*
 * Copyright (C) 2022 Paparazzi Team
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
 * @file peripherals/qmc5883l.c
 *
 * QST QMC5883L 3-axis magnetometer driver interface (I2C).
 */

#include "peripherals/qmc5883l.h"

#ifndef QMC5883L_USE_LOWPASS_FILTER
#define QMC5883L_USE_LOWPASS_FILTER TRUE
#endif

#ifndef QMC5883L_LOWPASS_CUTOFF
#define QMC5883L_LOWPASS_CUTOFF 3.f
#endif

#ifndef MAG_QMC5883L_PERIODIC_FREQUENCY
#define MAG_QMC5883L_PERIODIC_FREQUENCY 50.f
#endif

#if QMC5883L_USE_LOWPASS_FILTER
#include "filters/low_pass_filter.h"
#endif

#if QMC5883L_USE_LOWPASS_FILTER
static Butterworth2LowPass qmc5883l_lowpass[3];
static bool qmc5883l_lowpass_initialized;
#endif

/* Registers Axis X,Y,Z */
#define QMC5883L_REG_DATXL  0x00
#define QMC5883L_REG_DATXM  0x01
#define QMC5883L_REG_DATYL  0x02
#define QMC5883L_REG_DATYM  0x03
#define QMC5883L_REG_DATZL  0x04
#define QMC5883L_REG_DATZM  0x05

/* Register I2C bus transaction Status */
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_STATUS_DRDY 0x01
#define QMC5883L_STATUS_OVL  0x02

/* Registers Temperature, relative thus not so useful ATM, therefore not implemented in reading */
#define QMC5883L_REG_TEMPM  0x07
#define QMC5883L_REG_TEMPL  0x08 

/* Registers Config */
#define QMC5883L_REG_CONTROL_1    0x09  /* settings for MODE */
#define QMC5883L_REG_CONTROL_2    0x0A  /* settings for INT_ENB */
#define QMC5883L_REG_RESET_PERIOD 0x0B

#define QMC5883L_REG_IDC    0x0C  /* OEM reserved */
#define QMC5883L_REG_IDD    0x0D  /* OEM reserved */

/* Options for CONTROL_1 */
#define QMC5883L_MODE_STBY 0x00
#define QMC5883L_MODE_CONT 0x01

/* Options for scale RaNGe(RNG) Gauss */
#define QMC5883L_RNG_2G 0x00
#define QMC5883L_RNG_8G 0x10

/* options for Over-Sample Ratio (OSR) */
#define QMC5883L_OSR_512 0x00  /* Use 512 if powerusage of chip is not an issue */
#define QMC5883L_OSR_256 0x40
#define QMC5883L_OSR_128 0x80
#define QMC5883L_OSR_64  0xC0

void qmc5883l_init(struct Qmc5883l *mag, struct i2c_periph *i2c_p, uint8_t addr, uint8_t data_rate)
{
#if QMC5883L_USE_LOWPASS_FILTER
  qmc5883l_lowpass_initialized = false;
#endif
  /* set i2c_peripheral */
  mag->i2c_p = i2c_p;
  /* set i2c address */
  mag->i2c_trans.slave_addr = addr;
  mag->i2c_trans.status = I2CTransDone;
  /* store data rate */
  mag->data_rate = data_rate;
  mag->initialized = false;
  mag->status = QMC5883L_CONF_UNINIT;
  mag->data_available = false;
}

void qmc5883l_configure(struct Qmc5883l *mag)
{
  // Only configure when not busy
  if (mag->i2c_trans.status != I2CTransSuccess && mag->i2c_trans.status != I2CTransFailed
      && mag->i2c_trans.status != I2CTransDone) {
    return;
  }

  // Only when successful continue with next
  if (mag->i2c_trans.status == I2CTransSuccess) {
    mag->status++; //Here the Enum Counter goes to the next one
  }

  mag->i2c_trans.status = I2CTransDone;
  switch (mag->status) {

    case QMC5883L_CONF_UNINIT:
      /* prepare config request */
      mag->i2c_trans.buf[0] = QMC5883L_REG_RESET_PERIOD;
      mag->i2c_trans.buf[1] = 0x01;
      /* send config request, ask for i2c frame for set/reset period */
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 2);
      break;

    case QMC5883L_CONF_CCR_DONE:
      mag->i2c_trans.buf[0] = QMC5883L_REG_CONTROL_1;
      mag->i2c_trans.buf[1] = QMC5883L_MODE_CONT | mag->data_rate | QMC5883L_RNG_8G | QMC5883L_OSR_512;
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 2);
      break;

    case QMC5883L_CONF_TMRC_DONE:
      mag->i2c_trans.buf[0] = QMC5883L_REG_CONTROL_1;
      mag->i2c_trans.buf[1] = QMC5883L_MODE_CONT | mag->data_rate | QMC5883L_RNG_8G | QMC5883L_OSR_512;
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 2);
      break;

    case QMC5883L_CONF_CCM_DONE:
      mag->status = QMC5883L_STATUS_IDLE;
      mag->initialized = true;
      break;

    default:
      break;
  }
}

void qmc5883l_read(struct Qmc5883l *mag)
{
  if (mag->status != QMC5883L_STATUS_IDLE) {
    return;
  }

  mag->i2c_trans.buf[0] = QMC5883L_REG_STATUS;
  i2c_transceive(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 1, 1);
  mag->status = QMC5883L_STATUS_CHECK;
}
/* Convert and align raw values */
#define Int16FromBuf(_buf,_idx) ((int16_t)(_buf[_idx] | (_buf[_idx+1] << 8)))


void qmc5883l_event(struct Qmc5883l *mag)
{
  if (!mag->initialized) {
    return;
  }

  switch (mag->status) {

    case QMC5883L_STATUS_CHECK:
      if (mag->i2c_trans.status == I2CTransSuccess) {
        const uint8_t sample_status = mag->i2c_trans.buf[0];
        mag->i2c_trans.status = I2CTransDone;
        if ((sample_status & (QMC5883L_STATUS_DRDY | QMC5883L_STATUS_OVL)) == QMC5883L_STATUS_DRDY) {
          mag->i2c_trans.buf[0] = QMC5883L_REG_DATXL;
          i2c_transceive(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 1, 6);
          mag->status = QMC5883L_STATUS_MEAS;
        } else {
          mag->status = QMC5883L_STATUS_IDLE;
        }
      } else if (mag->i2c_trans.status == I2CTransFailed) {
        mag->status = QMC5883L_STATUS_IDLE;
      }
      break;

    case QMC5883L_STATUS_MEAS:
      if (mag->i2c_trans.status == I2CTransSuccess) {
        mag->unfiltered_data.vect.x = Int16FromBuf(mag->i2c_trans.buf, 0);
        mag->unfiltered_data.vect.y = Int16FromBuf(mag->i2c_trans.buf, 2);
        mag->unfiltered_data.vect.z = Int16FromBuf(mag->i2c_trans.buf, 4);
        mag->data.vect = mag->unfiltered_data.vect;
#if QMC5883L_USE_LOWPASS_FILTER
        if (!qmc5883l_lowpass_initialized) {
          const float tau = 1.f / (2.f * M_PI * QMC5883L_LOWPASS_CUTOFF);
          const float sample_time = 1.f / MAG_QMC5883L_PERIODIC_FREQUENCY;
          init_butterworth_2_low_pass(&qmc5883l_lowpass[0], tau, sample_time, mag->data.vect.x);
          init_butterworth_2_low_pass(&qmc5883l_lowpass[1], tau, sample_time, mag->data.vect.y);
          init_butterworth_2_low_pass(&qmc5883l_lowpass[2], tau, sample_time, mag->data.vect.z);
          qmc5883l_lowpass_initialized = true;
        } else {
          mag->data.vect.x = update_butterworth_2_low_pass(&qmc5883l_lowpass[0], mag->data.vect.x);
          mag->data.vect.y = update_butterworth_2_low_pass(&qmc5883l_lowpass[1], mag->data.vect.y);
          mag->data.vect.z = update_butterworth_2_low_pass(&qmc5883l_lowpass[2], mag->data.vect.z);
        }
#endif
        mag->data_available = true;
        /* End of measure reading, go back to idle */
        mag->status = QMC5883L_STATUS_IDLE;
      }
      else if (mag->i2c_trans.status == I2CTransFailed) {
        mag->status = QMC5883L_STATUS_IDLE;
      }
      break;

    default:
      if (mag->i2c_trans.status == I2CTransSuccess || mag->i2c_trans.status == I2CTransFailed) {
        /* Per default set to idle */
        mag->i2c_trans.status = I2CTransDone;
        mag->status = QMC5883L_STATUS_IDLE;
      }
      break;
  }
}

