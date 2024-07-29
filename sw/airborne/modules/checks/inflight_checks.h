/*
 * Copyright (C) 2024 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/checks/inflight_checks.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Adds checks which are performed during flight
 */

#ifndef INFLIGHT_CHECKS_H
#define INFLIGHT_CHECKS_H

#include "std.h"
#include <stdarg.h>

typedef void (*inflight_check_f)(struct inflight_check_t *check);

struct inflight_check_t {
  inflight_check_f func;          ///< Function to run periodically
  struct inflight_check_t *next;  ///< Next inflight check
  float period;                   ///< Period to run the check at (max PERIODIC_FREQUENCY)
  float last_run;                 ///< Last time the check was run
  float last_send;                ///< Last time the check was send (error/warning)
  uint16_t errors;                ///< Error counter
  uint16_t warnings;              ///< Warning counter
};

extern void inflight_check_register(struct inflight_check_t *check, float period, preflight_check_f func);
extern void inflight_error(struct inflight_check_t *check, const char *fmt, ...);
extern void inflight_warning(struct inflight_check_t *check, const char *fmt, ...);

#endif /* INFLIGHT_CHECKS_H */
