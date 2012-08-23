/*
 * $Id$
 *
 * Copyright (C) 2004-2006  Pascal Brisset, Antoine Drouin
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

/** \file estimator.h
 * \brief State estimation, fusioning sensors
 * TODO will be removed when the remaining variables are integrated to the state interface
 */

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <inttypes.h>

#include "std.h"

#include "state.h"

void estimator_init( void );

#endif /* ESTIMATOR_H */
