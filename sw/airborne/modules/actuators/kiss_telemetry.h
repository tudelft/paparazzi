/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>, 2026 OpenUAS
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
/**
 * @file "modules/actuators/kiss_telemetry.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * @brief Retrieve live telemetry data from a KISS or KISS compatible ESCs via the ESC datapin output a serial byte stream.
 */

#ifndef KISS_TELEMETRY_H
#define KISS_TELEMETRY_H

#include "pprzlink/pprzlink_device.h"

#define KISS_FRAME_LENGTH 10

struct kiss_telemetry_t {
    uint8_t servo_idx;                 ///< Servo index communicated with
    struct link_device *dev;           ///< Device used for receiving data
    uint8_t buf_idx;                   ///< Buffer index
    uint8_t buffer[KISS_FRAME_LENGTH]; ///< Buffer for the data packets
    uint8_t crc;                       ///< The calculated CRC
    uint16_t energy[4];                ///< Accumulated energy
};

// Note: In order to get the real Rpm of the motor one divides the Erpm result by the magnetpole count divided by two(2).
// For example with a 16 magnet poles motor: Rpm = Erpm/8, this is already handled in the code.
struct kiss_message_t {
    int8_t   temperature; ///< Temperature in 1°C -127 to +127°C
    uint16_t voltage;     ///< Volt *100 therefore 1000 would mean 10.00V
    uint16_t current;     ///< Current in Amperes * 100 therefore 2000 would mean 20.00A
    uint16_t consumption; ///< Consumption in 1mAh
    uint16_t rpm;         ///< Rpm /100 therefore 100 would mean 10000 rpm
    uint8_t  crc;         ///< CRC8 checksum to validate data if no corruption occurred during transfer
};

extern void kiss_telemetry_init(void);
extern void kiss_telemetry_periodic(void);
extern void kiss_telemetry_event(void);

#endif

