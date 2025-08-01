/** @file "modules/intermcu/timesync.h"
 * @author LT <LT>
 * Timesync module for companion computers.
 */


#include "std.h"
#include "math/pprz_algebra_float.h"

#ifndef TIMESYNC_H
#define TIMESYNC_H

extern void timesync_perform_sync(uint8_t *buf);

#endif  // TIMESYNC_H
