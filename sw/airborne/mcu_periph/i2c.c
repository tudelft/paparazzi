/*
 * Copyright (C) 2010-2012 The Paparazzi Team
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

/**
 * @file mcu_periph/i2c.c
 * Architecture independent I2C (Inter-Integrated Circuit Bus) API.
 */

#include "mcu_periph/i2c.h"

#ifdef USE_I2C0

struct i2c_periph i2c0;

void i2c0_init(void) {
  i2c_init(&i2c0);
  i2c0_hw_init();
}

#endif /* USE_I2C0 */


#ifdef USE_I2C1

struct i2c_periph i2c1;

void i2c1_init(void) {
  i2c_init(&i2c1);
  i2c1_hw_init();
}

#endif /* USE_I2C1 */


#ifdef USE_I2C2

struct i2c_periph i2c2;

void i2c2_init(void) {
  i2c_init(&i2c2);
  i2c2_hw_init();
}

#endif /* USE_I2C2 */

void i2c_init(struct i2c_periph* p) {
  p->trans_insert_idx = 0;
  p->trans_extract_idx = 0;
  p->status = I2CIdle;
}


