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
 * @file "modules/checks/inflight_checks.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Adds checks which are performed during flight
 */

#include "inflight_checks.h"
#include "modules/datalink/telemetry.h"
#include <stdio.h>

/** Only send messages down every xx amount of seconds */
#ifndef INFLIGHT_CHECK_TIMEOUT
#define INFLIGHT_CHECK_TIMEOUT 2
#endif

/** The seperator used in the inflight check message */
#define INFLIGHT_CHECK_SEPERATOR ';'


static struct inflight_check_t *inflight_head = NULL;

/**
 * @brief Register a inflight check and add it to the linked list
 *
 * @param check The check to add containing a linked list
 * @param period The period to run the checking function at (Max PERIODIC_FREQUENCY)
 * @param func The function to register for the check
 */
void inflight_check_register(struct inflight_check_t *check, float period, preflight_check_f func)
{
  // Prepend the inflight check
  struct inflight_check_t *next = inflight_head;
  inflight_head = check;
  check->func = func;
  check->period = period;
  check->last_run = 0;
  check->next = next;
}

static void ic_period_check(struct inflight_check_t *check) {
  static float last_time = 0;
  if(last_time == 0) {
    last_time = sys_time_float();
    return;
  }

  float drift = sys_time_float() - last_time - (1.0f / PERIODIC_FREQUENCY);
  if(fabsf(drift) > (1.0f / PERIODIC_FREQUENCY * 0.5)) {
    inflight_warning(check, "Main periodic is drifting %f", drift);
  } else if(fabsf(drift) > (1.0f / PERIODIC_FREQUENCY * 1)) {
    inflight_error(check, "Main periodic is drifting %f", drift);
  }
}


/**
 * @brief Initialize the inflight checks
*/
void inflight_check_init(void)
{
  // Create a circular buffer for sending down errors and warnings
  circular_buffer_init(&inflight_buffer, inflight_buffer_data, INFLIGHT_BUFFER_SIZE);

  // Register some checks ourself
  inflight_check_register(&inflight_check_period, 1.0f / PERIODIC_FREQUENCY, ic_period_check);
}

/**
 * @brief Perform all the inflight checks
 */
void inflight_check_period(void)
{
  float sys_time = sys_time_float();
  // Go through all the checks
  while(inflight_head) {
    // Run the check at the selected frequency
    if((inflight_head->last_run + inflight_head->period) > sys_time) {
      inflight_head->func(inflight_head);
      inflight_head->last_run = sys_time;
      continue;
    }

    inflight_head = inflight_head->next;
  }

  // Send the inflight checks to the GCS
}

/**
 * @brief An inflight error has occured and must be logged and send to the GCS
 *
 * @param fmt A formatted string describing the error used in a vsnprintf
 * @param ... The arguments for the vsnprintf
 */
void inflight_error(const char *fmt, ...)
{
  // Record the error count
  result->fail_cnt++;

  // No more space in the message
  if (result->max_len <= 0) {
    return;
  }

  // Add the error
  va_list args;
  va_start(args, fmt);
  int rc = vsnprintf(result->message, result->max_len, fmt, args);
  va_end(args);

  // Remove the length from the buffer if it was successfull
  if (rc > 0) {
    result->max_len -= rc;
    result->message += rc;

    // Add seperator if it fits
    if (result->max_len > 0) {
      result->message[0] = PREFLIGHT_CHECK_SEPERATOR;
      result->max_len--;
      result->message++;

      // Add the '\0' character
      if (result->max_len > 0) {
        result->message[0] = 0;
      }
    }
  }
}

/**
 * @brief Register a preflight warning used inside the preflight checking functions
 *
 * @param result Where the error gets registered
 * @param fmt A formatted string describing the error used in a vsnprintf
 * @param ... The arguments for the vsnprintf
 */
void preflight_warning(struct preflight_result_t *result, const char *fmt, ...)
{
  // Record the warning count
  result->warning_cnt++;

  // No more space in the message
  if (result->max_len <= 0) {
    return;
  }

  // Add the warning to the error string
  va_list args;
  va_start(args, fmt);
  int rc = vsnprintf(result->message, result->max_len, fmt, args);
  va_end(args);

  // Remove the length from the buffer if it was successfull
  if (rc > 0) {
    result->max_len -= rc;
    result->message += rc;

    // Add seperator if it fits
    if (result->max_len > 0) {
      result->message[0] = PREFLIGHT_CHECK_SEPERATOR;
      result->max_len--;
      result->message++;

      // Add the '\0' character
      if (result->max_len > 0) {
        result->message[0] = 0;
      }
    }
  }
}

/**
 * @brief Register a preflight success used inside the preflight checking functions
 *
 * @param result
 * @param __attribute__
 * @param ...
 */
void preflight_success(struct preflight_result_t *result, const char *fmt __attribute__((unused)), ...)
{
  // Record the success count
  result->success_cnt++;
}
