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

extern void alphapilot_ahrs_init(void);
extern void alphapilot_ahrs_periodic(void);
extern void alphapilot_ahrs_event(void);
extern void alphapilot_datalink_call(void);



inline double wrapAngle(double ang){
    if(ang>3.14159265){
        ang=ang-2.*3.14159265;
    }
    if(ang<-3.14159265){
        ang=ang+2.*3.14159265;
    }
return ang;
};

inline void transform2Body(float euler[3], float *output){
  float phi = euler[0];
  float theta = euler[1];
  float psi = euler[2];
  float X[3]= {};

  X[0] = cosf(theta)*cosf(psi)*output[0] + cosf(theta)*sinf(psi)*output[1] -sinf(theta)*output[2];
  X[1] = (sinf(phi)*sinf(theta)*cosf(psi)-cosf(phi)*sinf(psi))*output[0] + (sinf(phi)*sinf(theta)*sinf(psi)+cosf(phi)*cosf(psi))*output[1] + (sinf(phi)*cosf(theta))*output[2];
  X[2] = (cosf(phi)*sinf(theta)*cosf(psi)+sinf(phi)*sinf(psi))*output[0] + (cosf(phi)*sinf(theta)*sinf(psi)-sinf(phi)*cosf(psi))*output[1] + (cosf(phi)*cosf(theta))*output[2];
  output[0] = X[0];
  output[1] = X[1];
  output[2] = X[2];
};

inline void transform2Earth(float euler[3], float *output){
  float phi = euler[0];
  float theta = euler[1];
  float psi = euler[2];
  float X[3]= {};

  X[0] = cosf(theta)*cosf(psi)*output[0] + (sinf(phi)*sinf(theta)*cosf(psi)-cosf(phi)*sinf(psi))*output[1]+(cosf(phi)*sinf(theta)*cosf(psi)+sinf(phi)*sinf(psi))*output[2];
  X[1] = cosf(theta)*sinf(psi)*output[0] + (sinf(phi)*sinf(theta)*sinf(psi)+cosf(phi)*cosf(psi))*output[1] + (cosf(phi)*sinf(theta)*sinf(psi)-sinf(phi)*cosf(psi))*output[2];
  X[2] = -sinf(theta)*output[0] + (sinf(phi)*cosf(theta))*output[1] + (cosf(phi)*cosf(theta))*output[2];
  output[0] = X[0];
  output[1] = X[1];
  output[2] = X[2];
};

#endif



