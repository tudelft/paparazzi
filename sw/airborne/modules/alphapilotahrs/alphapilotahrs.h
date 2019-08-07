/*
 * Copyright (C) Jelle Westenberger
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
 * @file "modules/alphapilotahrs/alphapilotahrs.h"
 * @author Jelle Westenberger
 * AHRS used in ALPHAPILOT2019
 */

#ifndef ALPHAPILOTAHRS_H
#define ALPHAPILOTAHRS_H

extern void alphapilot_ahrs_init();
extern void alphapilot_ahrs_periodic();
extern void alphapilot_ahrs_event();
extern void alphapilot_datalink_call();



inline double wrapAngle(double ang){
    if(ang>3.14159265){
        ang=ang-2.*3.14159265;
    }
    if(ang<-3.14159265){
        ang=ang+2.*3.14159265;
    }
return ang;
};

#endif



