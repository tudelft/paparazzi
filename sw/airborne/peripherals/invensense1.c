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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file peripherals/invensense1.c
 *
 * Driver for the Invensense V1 IMUs
 * MPU6000, MPU9250, ICM20608, ICM20602, ICM20601, ICM20789 and ICM20689
 */

#include "peripherals/invensense1.h"
#include "peripherals/invensense1_regs.h"
#include "math/pprz_isa.h"
#include "math/pprz_algebra_int.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/gpio_arch.h"


/* Local functions */
static void invensense1_parse_data(struct invensense1_t *inv, volatile uint8_t *data, uint16_t len);
static void invensense1_fix_config(struct invensense1_t *inv);
static void invensense1_register_write(struct invensense1_t *inv, uint16_t reg, uint8_t value);
static void invensense1_register_read(struct invensense1_t *inv, uint16_t reg, uint16_t size);
static bool invensense1_config(struct invensense1_t *inv);

/* Default gyro scalings */
static const struct Int32Rates invensense1_gyro_scale[5][2] = {
  { {30267, 30267, 30267},
    {55463, 55463, 55463} }, // 250DPS
  { {60534, 60534, 60534},
    {55463, 55463, 55463} }, // 500DPS
  { {40147, 40147, 40147},
    {18420, 18420, 18420} }, // 1000DPS
  { {40147, 40147, 40147},
    {9210,  9210,  9210} },  // 2000DPS
  { {40147, 40147, 40147},
    {4605,  4605,  4605} }   // 4000DPS
};

/* Default accel scalings */
static const struct Int32Vect3 invensense1_accel_scale[5][2] = {
  { {3189, 3189, 3189},
    {5203, 5203, 5203} },   // 2G
  { {6378, 6378, 6378},
    {5203, 5203, 5203} },   // 4G
  { {12756, 12756, 12756},
    {5203,  5203,  5203} }, // 8G
  { {25512, 25512, 25512},
    {5203,  5203,  5203} }, // 16G
  { {51024, 51024, 51024},
    {5203,  5203,  5203} }  // 30G
};

/**
 * @brief Initialize the invensense v2 sensor instance
 * 
 * @param inv The structure containing the configuratio of the invensense v2 instance
 */
void invensense1_init(struct invensense1_t *inv) {
  /* General setup */
  inv->status = INVENSENSE1_IDLE;
  inv->device = INVENSENSE1_UNKOWN;
  inv->config_idx = 0;

  /* SPI setup */
  if(inv->bus == INVENSENSE1_SPI) {
    inv->spi.trans.cpol = SPICpolIdleHigh;
    inv->spi.trans.cpha = SPICphaEdge2;
    inv->spi.trans.dss = SPIDss8bit;
    inv->spi.trans.bitorder = SPIMSBFirst;
    inv->spi.trans.cdiv = SPIDiv16;

    inv->spi.trans.select = SPISelectUnselect;
    inv->spi.trans.slave_idx = inv->spi.slave_idx;
    inv->spi.trans.output_length = 0;
    inv->spi.trans.input_length = 0;
    inv->spi.trans.before_cb = NULL;
    inv->spi.trans.after_cb = NULL;
    inv->spi.trans.input_buf = inv->spi.rx_buf;
    inv->spi.trans.output_buf = inv->spi.tx_buf;
    inv->spi.trans.status = SPITransDone;
  }
  /* I2C setup */
  else {
    inv->i2c.trans.slave_addr = inv->i2c.slave_addr;
    inv->i2c.trans.status = I2CTransDone;
  }
}

/**
 * @brief Should be called periodically to request sensor readings
 * - First detects the sensor using WHOAMI reading
 * - Configures the sensor according the users requested configuration
 * - Requests a sensor reading by reading the FIFO_COUNT register
 * 
 * @param inv The invensense v2 instance
 */
void invensense1_periodic(struct invensense1_t *inv) {
  /* Idle */
  if((inv->bus == INVENSENSE1_SPI && inv->spi.trans.status == SPITransDone) || 
     (inv->bus == INVENSENSE1_I2C && inv->i2c.trans.status == I2CTransDone)) {

    /* Depending on the status choose what to do */
    switch(inv->status) {
      case INVENSENSE1_IDLE:
        /* Request WHOAMI */
        invensense1_register_read(inv, INV1REG_WHOAMI, 1);
        break;
      case INVENSENSE1_CONFIG:
        /* Start configuring */
        if(invensense1_config(inv)) {
          inv->status = INVENSENSE1_RUNNING;
        }
        break;
      case INVENSENSE1_RUNNING:
        /* Request a sensor reading */
        invensense1_register_read(inv, INV2REG_FIFO_COUNTH, 2);
        break;
    }
  }
}

/**
 * @brief Should be called in the event thread
 * - Checks the response of the WHO_AM_I reading
 * - Configures the sensor and reads the responses
 * - Parse and request the sensor data from the FIFO buffers
 * 
 * @param inv The invensense v2 instance
 */
void invensense1_event(struct invensense1_t *inv) {
  volatile uint8_t *rx_buffer, *tx_buffer;
  uint16_t rx_length = 0;

  /* Set the buffers depending on the interface */
  if(inv->bus == INVENSENSE1_SPI) {
    rx_buffer = inv->spi.rx_buf;
    tx_buffer = inv->spi.tx_buf;
    rx_length = inv->spi.trans.input_length;
  }
  else {
    rx_buffer = inv->i2c.trans.buf;
    tx_buffer = inv->i2c.trans.buf;
    rx_length = inv->i2c.trans.len_r;
  }

  /* Successful transfer */
  if((inv->bus == INVENSENSE1_SPI && inv->spi.trans.status == SPITransSuccess) || 
     (inv->bus == INVENSENSE1_I2C && inv->i2c.trans.status == I2CTransSuccess)) {

    /* Set the transaction as done and update register bank if needed */
    if(inv->bus == INVENSENSE1_SPI)
      inv->spi.trans.status = SPITransDone;
    else
      inv->i2c.trans.status = I2CTransDone;
    
    /* Look at the results */
    switch(inv->status) {
      case INVENSENSE1_IDLE:
        /* Check the response of the WHO_AM_I */
        inv->status = INVENSENSE1_CONFIG;
        switch(rx_buffer[1]) {
          case INV1_WHOAMI_MPU6000:
            inv->device = INVENSENSE1_MPU6000;
            break;
          case INV1_WHOAMI_MPU6500:
            inv->device = INVENSENSE1_MPU6500;
            break;
          case INV1_WHOAMI_MPU9250:
            inv->device = INVENSENSE1_MPU9250;
            break;
          case INV1_WHOAMI_MPU9255:
            inv->device = INVENSENSE1_MPU9255;
            break;
          case MPU_WHOAMI_20608D:
          case MPU_WHOAMI_20608G:
            inv->device = INVENSENSE1_ICM20608;
            break;
          case MPU_WHOAMI_20602:
            inv->device = INVENSENSE1_ICM20602;
            break;
          case MPU_WHOAMI_20601:
            inv->device = INVENSENSE1_ICM20601;
            break;
          case MPU_WHOAMI_20789:
          case MPU_WHOAMI_20789_R1:
            inv->device = INVENSENSE1_ICM20789;
            break;
          case MPU_WHOAMI_20689:
            inv->device = INVENSENSE1_ICM20689;
            break;
          default:
            inv->status = INVENSENSE1_IDLE;
            break;
        }

        /* Fix the configuration and set the scaling */
        if(inv->status == INVENSENSE1_CONFIG)
          invensense1_fix_config(inv);
        break;
      case INVENSENSE1_CONFIG:
        /* Apply the next configuration register */
        if(invensense1_config(inv)) {
          inv->status = INVENSENSE1_RUNNING;
        }
        break;
      case INVENSENSE1_RUNNING: {
        /* Parse the results */
        static const uint16_t max_bytes = sizeof(inv->spi.rx_buf) - 3;
        uint16_t fifo_bytes = (uint16_t)rx_buffer[1] << 8 | rx_buffer[2];

        // We read an incorrect length (try again)
        if(fifo_bytes > 4096) {
          invensense1_register_read(inv, INV2REG_FIFO_COUNTH, 2);
          return;
        }

        // Parse the data
        if((rx_length - 3) > 0) {
          uint16_t valid_bytes = ((rx_length - 3) < fifo_bytes)? (rx_length - 3) : fifo_bytes;
          invensense1_parse_data(inv, &rx_buffer[3], valid_bytes);
          inv->timer -= valid_bytes;
        } else {
          fifo_bytes -= fifo_bytes%INVENSENSE1_SAMPLE_SIZE;
          inv->timer = fifo_bytes;
        }

        // If we have more data request more
        if(inv->timer > 0) {
          uint16_t read_bytes = (inv->timer > max_bytes)? max_bytes : inv->timer;
          invensense1_register_read(inv, INV2REG_FIFO_COUNTH, 2 + read_bytes);
        }
        break;
      }
    }
  }
  /* Failed transaction */
  if((inv->bus == INVENSENSE1_SPI && inv->spi.trans.status == SPITransFailed) || 
     (inv->bus == INVENSENSE1_I2C && inv->i2c.trans.status == I2CTransFailed)) {

    /* Set the transaction as done and update register bank if needed */
    if(inv->bus == INVENSENSE1_SPI) {
      inv->spi.trans.status = SPITransDone;
    }
    else {
      inv->i2c.trans.status = I2CTransDone;
    }

    /* Retry or ignore */
    switch(inv->status) {
      case INVENSENSE1_CONFIG:
        /* If was not a bus switch decrease the index */
        if(inv->config_idx > 0 && ((inv->bus == INVENSENSE1_SPI && inv->spi.tx_buf[0] != INV2REG_BANK_SEL) ||
        (inv->bus == INVENSENSE1_I2C && inv->i2c.trans.buf[0] != INV2REG_BANK_SEL))) {
          inv->config_idx--;
        }
        /* Try again */
        if(invensense1_config(inv)) {
          inv->status = INVENSENSE1_RUNNING;
        }
        break;
      case INVENSENSE1_IDLE:
      case INVENSENSE1_RUNNING:
        /* Ignore while idle/running */
        break;
    }
  }
}

/**
 * @brief Parse the FIFO buffer data
 * 
 * @param inv The invensense v2 instance
 * @param data The FIFO buffer data to parse
 * @param len The length of the FIFO buffer
 */
static void invensense1_parse_data(struct invensense1_t *inv, volatile uint8_t *data, uint16_t len) {
  uint8_t samples = len  / INVENSENSE1_SAMPLE_SIZE;
  static struct Int32Vect3 accel[INVENSENSE1_SAMPLE_CNT] = {0};
  static struct Int32Rates gyro[INVENSENSE1_SAMPLE_CNT] = {0};

  if(samples > INVENSENSE1_SAMPLE_CNT)
    samples = INVENSENSE1_SAMPLE_CNT;

  uint16_t gyro_samplerate = 9000;
  if(inv->gyro_dlpf != INVENSENSE1_GYRO_DLPF_OFF)
    gyro_samplerate = 1125;

  uint16_t accel_samplerate = 4500;
  if(inv->accel_dlpf != INVENSENSE1_ACCEL_DLPF_OFF)
    accel_samplerate = 1125;

  // Go through the different samples
  uint8_t j = 0;
  uint8_t downsample = gyro_samplerate / accel_samplerate;
  int32_t temp = 0;
  for(uint8_t i = 0; i < samples; i++) { 
    if(i % downsample == 0) {
      accel[j].x = (int16_t)((uint16_t)data[2] << 8 | data[3]);
      accel[j].y = (int16_t)((uint16_t)data[0] << 8 | data[1]);
      accel[j].z = -(int16_t)((uint16_t)data[4] << 8 | data[5]);
      j++;
    }

    gyro[i].p = (int16_t)((uint16_t)data[8] << 8 | data[9]);
    gyro[i].q = (int16_t)((uint16_t)data[6] << 8 | data[7]);
    gyro[i].r = -(int16_t)((uint16_t)data[10] << 8 | data[11]);

    temp += (int16_t)((uint16_t)data[12] << 8 | data[13]);
    data += INVENSENSE1_SAMPLE_SIZE;
  }

  float temp_f = ((float)temp / samples) / 333.87f - 21.f;

  // Send the scaled values over ABI
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgIMU_GYRO_RAW(inv->abi_id, now_ts, gyro, samples);
  AbiSendMsgIMU_ACCEL_RAW(inv->abi_id, now_ts, accel, j);
  AbiSendMsgTEMPERATURE(inv->abi_id, temp_f);
}

/**
 * @brief This fixes the configuration errors and sets the correct scalings
 * 
 * @param inv The invensense v2 instance
 */
static void invensense1_fix_config(struct invensense1_t *inv) {
  /* Fix wrong configuration settings (prevent user error) */
  if(inv->device == INVENSENSE1_ICM20649) {
    if(inv->gyro_range == INVENSENSE1_GYRO_RANGE_250DPS)
      inv->gyro_range = INVENSENSE1_GYRO_RANGE_500DPS;
    if(inv->accel_range == INVENSENSE1_ACCEL_RANGE_2G)
      inv->accel_range = INVENSENSE1_ACCEL_RANGE_4G;
  }
  else {
    if(inv->gyro_range == INVENSENSE1_GYRO_RANGE_4000DPS)
      inv->gyro_range = INVENSENSE1_GYRO_RANGE_2000DPS;
    if(inv->accel_range == INVENSENSE1_ACCEL_RANGE_30G)
      inv->accel_range = INVENSENSE1_ACCEL_RANGE_16G;
  }

  /* Set the default values */
  imu_set_defaults_gyro(inv->abi_id, NULL, NULL, invensense1_gyro_scale[inv->gyro_range]);
  imu_set_defaults_accel(inv->abi_id, NULL, NULL, invensense1_accel_scale[inv->accel_range]);
}

/**
 * @brief Write a register based on a combined bank/regsiter value
 * 
 * @param inv The invensense v2 instance
 * @param reg The register to write to
 * @param value The value to write to the register
 */
static void invensense1_register_write(struct invensense1_t *inv, uint16_t reg, uint8_t value) {
  /* SPI transaction */
  if(inv->bus == INVENSENSE1_SPI) {
    inv->spi.trans.output_length = 2;
    inv->spi.trans.input_length = 0;
    inv->spi.tx_buf[0] = reg;
    inv->spi.tx_buf[1] = value;
    spi_submit(inv->spi.p, &(inv->spi.trans));
  }
  /* I2C transaction */
  else {
    inv->i2c.trans.buf[0] = reg;
    inv->i2c.trans.buf[1] = value;
    i2c_transmit(inv->i2c.p, &(inv->i2c.trans), inv->i2c.slave_addr, 2);
  }
}

/**
 * @brief Read a register based on a combined bank/regsiter value
 * 
 * @param inv The invensense v2 instance
 * @param reg The register to read from
 * @param size The size to read (already 1 is added for the transmission of the register to read)
 */
static void invensense1_register_read(struct invensense1_t *inv, uint16_t reg, uint16_t size) {
  /* SPI transaction */
  if(inv->bus == INVENSENSE1_SPI) {
    inv->spi.trans.output_length = 2;
    inv->spi.trans.input_length = 1 + size;
    inv->spi.tx_buf[0] = reg | INV2_READ_FLAG;
    inv->spi.tx_buf[1] = 0;
    spi_submit(inv->spi.p, &(inv->spi.trans));
  }
  /* I2C transaction */
  else {
    inv->i2c.trans.buf[0] = reg | INV2_READ_FLAG;
    i2c_transceive(inv->i2c.p, &(inv->i2c.trans), inv->i2c.slave_addr, 1, (1 + size));
  }
}

/**
 * @brief Configure the Invensense 2 device register by register
 * 
 * @param inv The invensense v2 instance
 * @return true When the configuration is completed
 * @return false Still busy configuring
 */
static bool invensense1_config(struct invensense1_t *inv) {
  switch(inv->config_idx) {
    case 0:
      /* Reset the device */
      if(invensense1_register_write(inv, INV2REG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET))
        inv->config_idx++;
      inv->timer = get_sys_time_usec();
      break;
    case 1: {
      /* Reset I2C and FIFO SRAM, disable I2C if using SPI */
      uint8_t user_ctrl = BIT_USER_CTRL_I2C_MST_RESET | BIT_USER_CTRL_SRAM_RESET;
      if(inv->bus == INVENSENSE1_SPI)
        user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;

      /* Because reset takes time wait ~100ms */
      if((get_sys_time_usec() - inv->timer) < 100000)
        break;

      if(invensense1_register_write(inv, INV2REG_USER_CTRL, user_ctrl))
        inv->config_idx++;
      break;
    }
    case 2:
      /* Wakeup the device in auto clock mode */
      if(invensense1_register_write(inv, INV2REG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_AUTO))
        inv->config_idx++;
      inv->timer = get_sys_time_usec();
      break;
    case 3: {
      /* Configure gyro */
      uint8_t gyro_config = 0;
      if(inv->gyro_dlpf != INVENSENSE1_GYRO_DLPF_OFF)
        gyro_config |= BIT_GYRO_DLPF_ENABLE | ((inv->gyro_dlpf - 1) << GYRO_DLPF_CFG_SHIFT);
      if((inv->device == INVENSENSE1_ICM20649 && inv->gyro_range > 0) || inv->gyro_range > 3)
        gyro_config |= (inv->gyro_range - 1) << GYRO_FS_SEL_SHIFT;
      else
        gyro_config |= inv->gyro_range << GYRO_FS_SEL_SHIFT;

      /* Because reset takes time wait ~100ms */
      if((get_sys_time_usec() - inv->timer) < 100000)
        break;

      if(invensense1_register_write(inv, INV2REG_GYRO_CONFIG_1, gyro_config))
        inv->config_idx++;
      break;
    }
    case 4: {
      /* Configure accelerometer */
      uint8_t accel_config = 0;
      if(inv->accel_dlpf != INVENSENSE1_ACCEL_DLPF_OFF)
        accel_config |= BIT_ACCEL_DLPF_ENABLE | ((inv->accel_dlpf - 1) << ACCEL_DLPF_CFG_SHIFT);
      if((inv->device == INVENSENSE1_ICM20649 && inv->accel_range > 0) || inv->accel_range > 3)
        accel_config |= (inv->accel_range - 1) << ACCEL_FS_SEL_SHIFT;
      else
        accel_config |= inv->accel_range << ACCEL_FS_SEL_SHIFT;
      if(invensense1_register_write(inv, INV2REG_ACCEL_CONFIG, accel_config))
        inv->config_idx++;
      break;
    }
    case 5:
      /* Set the FIFO mode */
      if(invensense1_register_write(inv, INV2REG_FIFO_MODE, 0xF))
        inv->config_idx++;
      break;
    case 6:
      /* Set the GYRO sample rate divider */
      if(invensense1_register_write(inv, INV2REG_GYRO_SMPLRT_DIV, 0))
        inv->config_idx++;
      break;
    case 7:
      /* FIFO reset 1 */
      if(invensense1_register_write(inv, INV2REG_FIFO_RST, 0x0F))
        inv->config_idx++;
      break;
    case 8:
      /* FIFO reset 2 */
      if(invensense1_register_write(inv, INV2REG_FIFO_RST, 0x00))
        inv->config_idx++;
      break;
    case 9: {
      /* Enable FIFO */
      uint8_t user_ctrl = BIT_USER_CTRL_FIFO_EN;
      if(inv->bus == INVENSENSE1_SPI)
        user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
      if(invensense1_register_write(inv, INV2REG_USER_CTRL, user_ctrl))
        inv->config_idx++;
      break;
    }
    case 10:
      /* Cofigure FIFO enable */
      if(invensense1_register_write(inv, INV2REG_FIFO_EN_2, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN))
        inv->config_idx++;
      break;
    case 11:
      /* Enable interrupt pin/status */
      if(invensense1_register_write(inv, INV2REG_INT_ENABLE_1, 0x1))
        inv->config_idx++;
      break;
    default:
      inv->timer = 0;
      return true;
  }
  return false;
}
