/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Blais, Antoine Drouin
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

#include "vor_int_demod.h"

#include <math.h>

#include  "vor_int_filters.h"


#define VID_ANGLE_RES  16
#define VID_ANGLE_FACT (1 << VID_ANGLE_RES)
#define VID_PANGLE(a)  ((a) * (float)VID_ANGLE_FACT + 0.5)
#define VID_NANGLE(a)  ((a) * (float)VID_ANGLE_FACT - 0.5)

//#define VID_DANGLE VID_PANGLE(2. * M_PI * 502. / 15000000.)

const int32_t vid_ref_freq;
      int32_t vid_ref_angle;
      int32_t vid_ref_phi;
      int32_t vid_ref_err;
const int32_t vid_ref_alpha = VID_NANGLE(-1.2);


void vor_int_demod_init( void) {

  vid_ref_angle = 0;
  vid_ref_phi = VID_PANGLE(M_PI); 
  vid_ref_err = 0;

}


void vor_int_demod_run ( uint16_t sample) {

  // phase error re-injection
  vid_ref_phi -= vid_ref_alpha * vid_ref_err;
  
  // local oscillator phase
  // ti = i * 502/15000000
  // angle_i = 2*pi*9960.*ti
  // angle += 2*pi*9960*502/15000000
  //  int32_t angle = 
  //  const float vfd_ref_phase = 2. * M_PI * vfd_ref_freq * ti + vfd_ref_phi;

}

