/*
 * Copyright (C) 2024 Alessandro Mancinelli <alessandro.mancinelli@outlook.com>
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
 * @file "modules/tilt_servo_calibration/tilt_calibration.h"
 * @author Alessandro Mancinelli <alessandro.mancinelli@outlook.com>
 * Calibration of the dual-axis tilting rotor quad-plane servos
 */

#ifndef TILT_CALIBRATION_H
#define TILT_CALIBRATION_H

#include "std.h"
#include <stdarg.h>

extern bool rotor_1_calibrate;
extern bool rotor_2_calibrate;
extern bool rotor_3_calibrate;
extern bool rotor_4_calibrate;

extern void tilt_calibration_init(void);
extern void tilt_calibration_run(void);

#endif /* TILT_CALIBRATION_H */
