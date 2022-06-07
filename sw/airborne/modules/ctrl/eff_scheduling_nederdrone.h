/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/ctrl/eff_scheduling_nederdrone.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Interpolation of control effectivenss matrix 
of the Nederdrone.
      
If instead using online adaptation is an option, be sure to 
not use this module at the same time!
 */

#ifndef EFF_SCHEDULING_NEDERDRONE_H
#define EFF_SCHEDULING_NEDERDRONE_H

extern void ctrl_eff_scheduling_init(void);
extern void ctrl_eff_scheduling_periodic(void);

#endif  // EFF_SCHEDULING_NEDERDRONE_H
