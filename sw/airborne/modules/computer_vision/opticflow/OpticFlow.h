/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.h"
 * @author C. De Wagter
 * opencv
 */
#include "linear_flow_fit.h"

#ifndef CV_OPENCVDEMO_H
#define CV_OPENCVDEMO_H

bool determine_flow(char* prev, char* curr, int height, int width, uint16_t winSize_i, uint16_t maxLevel, int OPTICFLOW_ERROR_THRESHOLD, int OPTICFLOW_N_ITERATIONS, int OPTICFLOW_N_SAMPLES, struct linear_flow_fit_info * info);

#endif


