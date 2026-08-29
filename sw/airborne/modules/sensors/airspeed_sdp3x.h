/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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

/** @file modules/sensors/airspeed_sdp3x.h
 *  Airspeed driver for the SDP3X pressure sensor via I2C.
 */

#ifndef AIRSPEED_SDP3X_H
#define AIRSPEED_SDP3X_H

#include "std.h"

#define SDP3X_SCALE_PRESSURE_SDP31 60U
#define SDP3X_SCALE_PRESSURE_SDP32 240U
#define SDP3X_SCALE_PRESSURE_SDP33 20U

static inline int16_t sdp3x_decode_int16(const uint8_t msb, const uint8_t lsb)
{
  const uint16_t word = ((uint16_t)msb << 8) | (uint16_t)lsb;
  return (word & 0x8000U) ? (int16_t)((int32_t)word - 65536L) : (int16_t)word;
}

static inline bool sdp3x_crc_valid(const uint8_t data[], unsigned size, uint8_t checksum)
{
  uint8_t crc = 0xff;
  for (unsigned i = 0; i < size; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1);
    }
  }
  return crc == checksum;
}

static inline bool sdp3x_scale_is_valid(uint16_t scale)
{
  return scale == SDP3X_SCALE_PRESSURE_SDP31 ||
         scale == SDP3X_SCALE_PRESSURE_SDP32 ||
         scale == SDP3X_SCALE_PRESSURE_SDP33;
}

static inline float sdp3x_pressure_from_raw(int16_t raw, float pressure_scale, bool dynamic_pressure_swapped)
{
  const float pressure = (float)raw / pressure_scale;
  return dynamic_pressure_swapped ? -pressure : pressure;
}

static inline float sdp3x_pressure_for_airspeed(float pressure, bool bidirectional)
{
  return bidirectional || pressure > 0.f ? pressure : 0.f;
}

static inline float sdp3x_eas_from_pressure(float pressure, float airspeed_scale)
{
  const float magnitude = sqrtf(fabsf(pressure) * airspeed_scale);
  return pressure < 0.f ? -magnitude : magnitude;
}

struct AirspeedSdp3x {
  float pressure;              ///< (differential) pressure in Pascal
  float temperature;           ///< Temperature in degrees Celsius
  float airspeed;              ///< Airspeed in m/s estimated from (differential) pressure.
  float airspeed_scale;        ///< Equivalent airspeed scale in (m/s)^2/Pa
  float pressure_scale;        ///< Sensor output scale in counts/Pa
  float pressure_offset;       ///< Offset in Pascal
  bool autoset_offset;         ///< Set offset value from current filtered value
  uint16_t raw_p;              ///< Raw signed pressure word, stored as its 16-bit representation
};

extern struct AirspeedSdp3x sdp3x;

extern void sdp3x_init(void);
extern void sdp3x_periodic(void);
extern void sdp3x_event(void);

#endif
