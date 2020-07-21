/*
 * Copyright (C) 2020 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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
 * @file modules/ctrl/motor_vs_flap_slider.h
 */

#ifndef MOTOR_VS_FLAP_SLIDER_H
#define MOTOR_VS_FLAP_SLIDER_H

extern float pitch_slider;
extern float yaw_slider;

/**
 * Initialises periodic loop;
 */
extern void motor_vs_flap_slider_init(void);

/**
 * Periodic function 
 */
extern void motor_vs_flap_slider_periodic(void);

#endif  /* MOTOR_VS_FLAP_SLIDER_H */

