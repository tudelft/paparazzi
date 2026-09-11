#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "modules/lidar/tfmini.h"
#include "modules/core/abi.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "nps_fdm.h"

struct NpsFdm fdm;
struct uart_periph uart3;
abi_event *abi_queues[ABI_MESSAGE_NB];

static unsigned agl_count;
static unsigned obstacle_count;
static float last_distance;

uint32_t get_sys_time_usec(void)
{
  return 123456;
}

static void receive_agl(uint8_t sender_id, uint32_t stamp, float distance)
{
  assert(sender_id == AGL_LIDAR_TFMINI_ID);
  assert(stamp == 123456);
  last_distance = distance;
  agl_count++;
}

static void receive_obstacle(uint8_t sender_id, float distance, float elevation, float heading)
{
  (void)sender_id;
  (void)distance;
  (void)elevation;
  (void)heading;
  obstacle_count++;
}

int main(void)
{
  abi_event agl_event = {0};
  abi_event obstacle_event = {0};
  AbiBindMsgAGL(ABI_BROADCAST, &agl_event, receive_agl);
  AbiBindMsgOBSTACLE_DETECTION(ABI_BROADCAST, &obstacle_event, receive_obstacle);
  tfmini_init();
  assert(tfmini.update_agl == USE_TFMINI_AGL);

  fdm.agl = 2.5;
  tfmini_event();
  assert(tfmini.distance == 2.5f);
  assert(agl_count == (USE_TFMINI_AGL ? 1U : 0U));
  if (USE_TFMINI_AGL) {
    assert(last_distance == 2.5f);
  }

  tfmini.update_agl = false;
  fdm.agl = 3.0;
  unsigned previous_count = agl_count;
  tfmini_event();
  assert(tfmini.distance == 3.0f);
  assert(agl_count == previous_count);

  tfmini.update_agl = true;
  const double invalid_heights[] = {-1.0, 0.0, LIDAR_MIN_RANGE / 2.0,
                                    LIDAR_MAX_RANGE + 1.0, NAN, INFINITY};
  for (unsigned index = 0; index < sizeof(invalid_heights) / sizeof(invalid_heights[0]); index++) {
    fdm.agl = invalid_heights[index];
    tfmini_event();
    assert(tfmini.distance == 0.0f);
    assert(agl_count == previous_count);
  }

  fdm.agl = 4.0;
  tfmini_event();
  assert(last_distance == 4.0f);
  assert(agl_count == previous_count + 1);
  assert(obstacle_count == 0);
  puts("TFMini NPS AGL checks passed");
  return 0;
}