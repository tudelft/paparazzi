/*
 * Copyright (C) 2006- Pascal Brisset, Antoine Drouin
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
 * @file firmwares/fixedwing/fbw_downlink.h
 *
 * Set of macros defining the periodic telemetry messages of FBW process.
 *
 * The PeriodicSendAp() macro is generated from the telemetry description
 * (named in conf.xml, usually in conf/telemetry directory). This macro
 * is a sequence of calls to PERIODIC_SEND_message() which have to be defined
 * in the present file.
 *
 */

#ifndef FBW_DOWNLINK_H
#define FBW_DOWNLINK_H

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_FBW_DEVICE
#endif
#include "subsystems/datalink/downlink.h"
#include "generated/periodic_telemetry.h"

static inline void fbw_downlink_periodic_task(void) {
  periodic_telemetry_send_Fbw();
}


#endif /* FBW_DOWNLINK_H */
