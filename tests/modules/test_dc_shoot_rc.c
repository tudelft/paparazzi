#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define AIRFRAME_H
#define GENERATED_MODULES_H
#define DC_H
#define DC_RADIO_SHOOT 7
#define DC_SHOOT 32

#include "modules/radio_control/radio_control.h"

struct RadioControl radio_control;
static unsigned shot_count;

void dc_send_command(uint8_t command)
{
  assert(command == DC_SHOOT);
  shot_count++;
}

#include "modules/digital_cam/dc_shoot_rc.c"

static void tick(unsigned count)
{
  for (unsigned index = 0; index < count; index++) {
    dc_shoot_rc_periodic();
  }
}

int main(void)
{
  radio_control.nb_channel = DC_RADIO_SHOOT + 1;
  radio_control.status = RC_REALLY_LOST;
  radio_control_set(DC_RADIO_SHOOT, MAX_PPRZ);
  tick(12);
  assert(shot_count == 0);

  radio_control.status = RC_OK;
  radio_control_set(DC_RADIO_SHOOT, MIN_PPRZ);
  tick(12);
  assert(shot_count == 0);
  radio_control_set(DC_RADIO_SHOOT, DC_RADIO_SHOOT_THRESHOLD);
  tick(12);
  assert(shot_count == 0);

  radio_control_set(DC_RADIO_SHOOT, DC_RADIO_SHOOT_THRESHOLD + 1);
  tick(1);
  assert(shot_count == 1);
  tick(3);
  assert(shot_count == 1);
  tick(1);
  assert(shot_count == 2);
  tick(40);
  assert(shot_count == 12);

  radio_control_set(DC_RADIO_SHOOT, 0);
  tick(1);
  radio_control_set(DC_RADIO_SHOOT, MAX_PPRZ);
  tick(1);
  assert(shot_count == 13);
  radio_control_set(DC_RADIO_SHOOT, MIN_PPRZ);
  tick(20);
  assert(shot_count == 13);

  radio_control_set(DC_RADIO_SHOOT, MAX_PPRZ);
  tick(1);
  assert(shot_count == 14);
  radio_control.status = RC_LOST;
  tick(20);
  assert(shot_count == 14);
  radio_control.status = RC_REALLY_LOST;
  tick(20);
  assert(shot_count == 14);
  radio_control.status = RC_OK;
  tick(1);
  assert(shot_count == 15);

  radio_control.nb_channel = DC_RADIO_SHOOT;
  tick(20);
  assert(shot_count == 15);
  radio_control.nb_channel = DC_RADIO_SHOOT + 1;
  tick(1);
  assert(shot_count == 16);

  puts("RC shutter: timing, release, RC loss/recovery and channel bounds passed");
  return 0;
}