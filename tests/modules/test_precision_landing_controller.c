#include <assert.h>
#include <stdio.h>
#include "modules/nav/precision_landing.h"
#include <generated/airframe.h>
#include <generated/flight_plan.h>
#include <state.h>

struct TestWaypoint test_waypoints[2] = {{-150.f, 0.f, 17.f}, {0.f, 0.f, 0.f}};
struct EnuCoor_f test_position, test_speed;
pprz_t commands[1];
float h_ctl_roll_max_setpoint, test_altitude, test_target_altitude, test_preclimb;
float test_airspeed, test_time;
bool test_airspeed_valid;
float agl_dist_valid, agl_dist_value, agl_dist_value_filtered, agl_measurement_time;

static void reset(void)
{
  test_time = 10.f;
  test_position = (struct EnuCoor_f){-20.f, 0.f, 3.f};
  test_speed = (struct EnuCoor_f){9.f, 0.f, -1.f};
  test_altitude = 3.f;
  test_airspeed = 9.f;
  test_airspeed_valid = true;
  agl_dist_valid = true;
  agl_dist_value_filtered = 3.f;
  agl_measurement_time = test_time;
  precision_landing_setup(0, 1, 3.5f, 6.f);
}

int main(void)
{
  reset();
  precision_landing_run();
  assert(!precision_landing_abort && commands[0] < 0);
  precision_landing_glide();
  assert(fabsf(test_target_altitude - 14.f * 17.f / 150.f) < 0.001f);
  assert(test_preclimb < 0.f);

  reset();
  test_altitude = 3.4f;
  agl_dist_value_filtered = 3.55f;
  precision_landing_run();
  assert(!precision_landing_abort);

  reset();
  agl_measurement_time = test_time - 0.3f;
  precision_landing_run();
  assert(precision_landing_abort && !precision_landing_commit_flare && commands[0] == 0);
  test_altitude = 1.f;
  precision_landing_run();
  assert(!precision_landing_abort && precision_landing_commit_flare);

  reset();
  agl_measurement_time = test_time + 1.f;
  precision_landing_run();
  assert(precision_landing_abort && !precision_landing_agl_fresh);

  reset();
  test_airspeed_valid = false;
  precision_landing_run();
  assert(precision_landing_abort && commands[0] == 0);
  test_altitude = agl_dist_value_filtered = 1.f;
  precision_landing_run();
  assert(!precision_landing_abort && commands[0] == 0);

  reset();
  test_speed.x = NAN;
  precision_landing_run();
  assert(precision_landing_abort && commands[0] == 0);

  reset();
  test_position.y = 3.f;
  precision_landing_run();
  assert(!precision_landing_abort);
  test_time += 0.25f;
  agl_measurement_time = test_time;
  precision_landing_run();
  assert(precision_landing_abort);
  precision_landing_setup(0, 1, 3.5f, 6.f);
  assert(!precision_landing_abort && !precision_landing_commit_flare);

  reset();
  test_position.x = 0.f;
  precision_landing_brake_fraction = 1.f;
  precision_landing_run();
  test_time += 0.21f;
  agl_measurement_time = test_time;
  precision_landing_run();
  assert(precision_landing_abort && commands[0] == 0);

  precision_landing_setup(1, 1, 3.5f, 6.f);
  precision_landing_run();
  assert(precision_landing_abort && commands[0] == 0);
  precision_landing_flare(NAN);
  assert(commands[0] == 0);
  precision_landing_stop();
  assert(h_ctl_roll_max_setpoint == H_CTL_ROLL_MAX_SETPOINT);
  puts("Precision landing controller checks passed");
  return 0;
}