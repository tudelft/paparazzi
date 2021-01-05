/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

#ifndef CONFIG_RASPBERRY
#define CONFIG_RASPBERRY

#define BOARD_RASPBERRY

#include "std.h"

/** UART connected to GPS internally */
#define UART1_DEV /dev/ttyS0

//#define GPS_UBX_ENABLE_NMEA_DATA_MASK 0xff

/** For using serial devices via USB to serial converter electronics
 *  E.g. a XBee modem, a 3DR radio modem, Serial Stereocam etc. etc.
 */
#ifndef UART2_DEV
#define UART2_DEV /dev/ttyUSB0
#endif

/** uart connected to SBUS input */
#ifndef UART3_DEV
#define UART3_DEV /dev/uart-sbus
#endif

/* Default actuators driver */
#define DEFAULT_ACTUATORS "boards/raspberry/actuators.h"
#define ActuatorDefaultSet(_x,_y) ActuatorsRaspberrySet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsRaspberryInit()
#define ActuatorsDefaultCommit() ActuatorsRaspberryCommit()

/* Cameras */
#include "peripherals/video_device.h"

#undef USE_ADC0
#define USE_ADC0 0

// re-use the Parrot Bebop video drivers
//#include "boards/bebop/mt9v117.h"
//#include "boards/bebop/mt9f002.h"

//extern struct video_config_t bottom_camera;
extern struct video_config_t front_camera;

/* ISP */
//struct mt9f002_t mt9f002;

/* by default activate onboard baro */
//#ifndef USE_BARO_BOARD
//#define USE_BARO_BOARD 1
//#endif

/* The ADC from the sonar */
/*
#if USE_ADC0
#define ADC0_ID             0
#define ADC0_CHANNELS       2
#define ADC0_CHANNELS_CNT   1
#define ADC0_BUF_LENGTH     8192
#endif
*/

/* The SPI from the sonar */
/*
#if USE_SPI0
#define SPI0_MODE           0
#define SPI0_BITS_PER_WORD  8
#define SPI0_MAX_SPEED_HZ   320000
#endif
*/


#endif /* CONFIG_RASPBERRY */
