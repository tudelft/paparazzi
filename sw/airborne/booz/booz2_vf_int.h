/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef BOOZ2_VF_INT_H
#define BOOZ2_VF_INT_H

#include "std.h"
#include "booz_geometry_int.h"

extern void booz2_vfi_init( int32_t z0, int32_t zd0, int32_t bias0 );
extern void booz2_vfi_propagate( int32_t accel_reading );

/* z_meas : altitude measurement in meter       */
/* Q23.8 : accuracy 0.004m range 8388km         */
extern void booz2_vfi_update( int32_t z_meas );
#define B2_VFI_Z_MEAS_FRAC IPOS_FRAC

/* propagate frequency : 512 Hz */
#define B2_VFI_F_UPDATE_FRAC 9
#define B2_VFI_F_UPDATE   (1<<B2_VFI_F_UPDATE_RES)

/* vertical acceleration in m/s^2                */
/* Q21.10 : accuracy 0.001m/s^2, range 2097km/s2 */
extern int32_t b2_vfi_zdd;
#define B2_VFI_ZDD_FRAC IACCEL_RES

/* vertical accelerometer bias in m/s^2          */
/* Q21.10 : accuracy 0.001m/s^2, range 2097km/s2 */
extern int32_t b2_vfi_abias;
#define B2_VFI_BIAS_FRAC IACCEL_RES

/* vertical speed in m/s                         */
/* Q12.19 : accuracy 0.000002 , range 4096m/s2   */
extern int32_t b2_vfi_zd;
#define B2_VFI_ZD_FRAC (B2_VFI_ZDD_FRAC + B2_VFI_F_UPDATE_FRAC)

/* altitude in m                                 */
/* Q35.28 : accuracy 3.7e-9 , range 3.4e10m      */
extern int64_t b2_vfi_z;
#define B2_VFI_Z_FRAC   (B2_VFI_ZD_FRAC + B2_VFI_F_UPDATE_FRAC)

/* Kalman filter state                           */
#define B2_VFI_S_Z    0
#define B2_VFI_S_ZD   1
#define B2_VFI_S_AB   2
#define B2_VFI_S_SIZE 3
/* Kalman filter covariance                      */
/* Q3.28                                         */
extern int32_t b2_vfi_P[B2_VFI_S_SIZE][B2_VFI_S_SIZE];
#define B2_VFI_P_FRAC  28




#endif /* BOOZ2_VF_INT_H */
