/** @file "modules/intermcu/timesync.c"
 * @author LT <LT>
 * Timesync module for companion computers.
 */

#include "modules/intermcu/timesync.h"
#include "modules/datalink/extra_pprz_dl.h"
#include "modules/datalink/telemetry.h"
#include "modules/datalink/downlink.h"
#include "pprzlink/intermcu_msg.h"


#include "modules/datalink/telemetry.h"

void timesync_perform_sync(uint8_t *buf)
{
  uint64_t ts1 = pprzlink_get_DL_IMCU_TIMESYNC_ts1(buf);
  uint32_t cur_time = get_sys_time_usec(); // time in usec [since boot]
  // TODO: check for tc1==0?

  // Respond in kind
  pprz_msg_send_IMCU_TIMESYNC(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID,
          &cur_time,
          &ts1);
}


