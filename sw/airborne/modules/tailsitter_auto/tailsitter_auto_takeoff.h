/*
 * Copyright (C) 2023 Yamac Birol
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

#ifndef TAILSITTER_AUTO_TOFF_H
#define TAILSITTER_AUTO_TOFF_H

extern int16_t take_off_stage(float theta, float rate_q);
extern float take_off_theta(void);
extern void take_off_enter(void);

extern float theta_ref;
extern float t_scale_to_theta;

#endif // TAILSITTER_AUTO_TOFF_H