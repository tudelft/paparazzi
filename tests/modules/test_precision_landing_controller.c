#include <assert.h>
#include <stdio.h>
#include <float.h>
#include "modules/nav/precision_landing.h"
#include <generated/airframe.h>
#include <generated/flight_plan.h>
#include <state.h>
#include "autopilot.h"

struct TestWaypoint test_waypoints[2] = {{-150.f, 0.f, 17.f}, {0.f, 0.f, 0.f}};
struct EnuCoor_f test_position, test_speed;
pprz_t commands[1];
float h_ctl_roll_max_setpoint, test_altitude, test_target_altitude, test_preclimb;
float test_airspeed, test_time;
float air_data_airspeed_time, flight_altitude;
uint8_t test_mode;
struct TestAutopilot autopilot;
bool test_airspeed_valid;
bool test_gps_valid;
float test_nav_pitch, test_nav_throttle;
bool test_nav_route;
float agl_dist_valid, agl_dist_value, agl_dist_value_filtered, agl_measurement_time;

static void set_parameters(float airspeed, float height, float brake_height, float flare_height,
                           float aim, float pitch, float brake, uint8_t retries)
{
  precision_landing_approach_airspeed = airspeed;
  precision_landing_final_height = height;
  precision_landing_brake_agl = brake_height;
  precision_landing_flare_agl = flare_height;
  precision_landing_aim_before_td = aim;
  precision_landing_touchdown_pitch = pitch;
  precision_landing_flare_brake = brake;
  precision_landing_max_retries = retries;
}

static void reset(void)
{
  precision_landing_stop();
  test_mode = 2;
  h_ctl_roll_max_setpoint = H_CTL_ROLL_MAX_SETPOINT;
  precision_landing_start();
  test_time = 10.f;
  air_data_airspeed_time = test_time;
  test_position = (struct EnuCoor_f){-20.f, 0.f, 3.f};
  test_speed = (struct EnuCoor_f){9.f, 0.f, -1.f};
  test_altitude = 3.f;
  test_airspeed = 9.f;
  test_airspeed_valid = true;
  test_gps_valid = true;
  agl_dist_valid = true;
  agl_dist_value_filtered = 3.f;
  agl_measurement_time = test_time;
  set_parameters(9.f, 17.f, 3.5f, 2.f, 6.f, 4.f, 0.3f, 2);
  precision_landing_setup(0, 1);
}

int main(void)
{
  commands[0] = -MAX_PPRZ;
  reset();
  assert(commands[0] == 0 && precision_landing_brake_fraction == 0.f);
  precision_landing_flare(0.3f);
  assert(commands[0] == (pprz_t)(-0.3f * MAX_PPRZ));
  assert(precision_landing_brake_fraction == 0.3f);
  precision_landing_stop();
  assert(commands[0] == 0 && precision_landing_brake_fraction == 0.f);
  assert(h_ctl_roll_max_setpoint == H_CTL_ROLL_MAX_SETPOINT);

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
  assert(!precision_landing_abort && precision_landing_commit_flare && commands[0] == 0);

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
  precision_landing_setup(0, 1);
  assert(!precision_landing_abort && !precision_landing_commit_flare);

  reset();
  test_position.x = 0.f;
  precision_landing_brake_fraction = 1.f;
  precision_landing_run();
  test_time += 0.21f;
  agl_measurement_time = test_time;
  precision_landing_run();
  assert(precision_landing_abort && commands[0] == 0);

  precision_landing_setup(1, 1);
  precision_landing_run();
  assert((precision_landing_abort || precision_landing_commit_flare) && commands[0] == 0);
  precision_landing_flare(NAN);
  assert(commands[0] == 0);
  precision_landing_stop();
  assert(h_ctl_roll_max_setpoint == H_CTL_ROLL_MAX_SETPOINT);
  reset();
  test_altitude = agl_dist_value_filtered = 10.f;
  test_airspeed = 1.f;
  precision_landing_run();
  assert(precision_landing_abort && commands[0] == 0);
  reset();
  test_airspeed_valid = false;
  precision_landing_flare(0.65f);
  assert(commands[0] == 0);
  reset();
  test_altitude = agl_dist_value_filtered = 2.2f;
  test_position.x = 1.f;
  precision_landing_run();
  assert(precision_landing_commit_flare && !precision_landing_abort);
  reset();
  test_altitude = -0.1f;
  agl_dist_valid = false;
  precision_landing_run();
  assert(precision_landing_commit_flare && !precision_landing_abort);
  reset();
  test_altitude = agl_dist_value_filtered = 2.6f;
  test_position.x = -14.4f;
  precision_landing_run();
  test_time += 0.21f;
  precision_landing_run();
  assert(precision_landing_abort && commands[0] == 0);
  reset();
  air_data_airspeed_time = test_time - 0.6f;
  precision_landing_run();
  assert(precision_landing_abort && commands[0] == 0);
  precision_landing_flare(0.65f);
  assert(commands[0] == 0);
  reset();
  h_ctl_roll_max_setpoint = 0.4f;
  precision_landing_flare(0.3f);
  precision_landing_on_mode_change(3);
  test_mode = 3;
  assert(precision_landing_cancelled && commands[0] == 0);
  assert(h_ctl_roll_max_setpoint == 0.4f);
  test_mode = 2;
  precision_landing_flare(0.3f);
  assert(commands[0] == 0);
  reset();
  precision_landing_run();
  precision_landing_glide();
  assert(fabsf(flight_altitude - test_target_altitude) < 0.0001f);
  reset();
  test_gps_valid = false;
  precision_landing_run();
  assert(precision_landing_abort && !precision_landing_commit_flare);
  reset();
  agl_dist_value_filtered = 1.f;
  test_altitude = 20.f;
  test_gps_valid = false;
  precision_landing_run();
  assert(!precision_landing_abort && precision_landing_commit_flare);
  agl_dist_valid = false;
  test_altitude = 30.f;
  precision_landing_check_abort();
  assert(precision_landing_commit_flare && !precision_landing_abort);
  reset();
  test_altitude = NAN;
  agl_dist_valid = false;
  precision_landing_run();
  assert(precision_landing_commit_flare && !precision_landing_abort && commands[0] == 0);
  reset();
  precision_landing_setup(0, 255);
  precision_landing_run();
  assert(commands[0] == 0);
  reset();
  test_altitude = agl_dist_value_filtered = 17.f;
  test_position.y = 3.f;
  precision_landing_run();
  test_time += 0.21f;
  precision_landing_run();
  assert(!precision_landing_abort);
  test_position.y = 50.f;
  precision_landing_run();
  test_time += 0.21f;
  air_data_airspeed_time = test_time;
  precision_landing_run();
  assert(precision_landing_abort);
  reset();
  test_nav_throttle = 9600;
  precision_landing_flare_run();
  assert(test_nav_throttle == 0 && test_nav_route);
  assert(fabsf(test_nav_pitch - (float)RadOfDeg(4.f)) < 0.0001f);
  test_gps_valid = false;
  precision_landing_flare_run();
  assert(!test_nav_route && test_nav_throttle == 0);
  precision_landing_touchdown_pitch = NAN;
  precision_landing_flare_run();
  assert(test_nav_pitch == 0);
  precision_landing_stop();
  test_nav_throttle = 1234;
  precision_landing_flare_run();
  assert(commands[0] == 0 && test_nav_throttle == 1234);
  assert(precision_landing_cancelled);
  reset();
  precision_landing_stop();
  precision_landing_setup(0, 1);
  assert(precision_landing_cancelled && commands[0] == 0);
  assert(!precision_landing_is_active());
  precision_landing_check_abort();
  assert(precision_landing_cancelled && commands[0] == 0);
  precision_landing_start();
  assert(precision_landing_is_active());
  precision_landing_stop();
  commands[0] = -MAX_PPRZ;
  precision_landing_on_mode_change(0);
  assert(commands[0] == 0);
  test_mode = 0;
  commands[0] = -MAX_PPRZ;
  precision_landing_on_mode_change(2);
  assert(commands[0] == 0 && !precision_landing_is_active());
  reset();
  test_speed = (struct EnuCoor_f){0.f, 0.f, 0.f};
  agl_dist_value_filtered = 0.1f;
  autopilot = (struct TestAutopilot){.kill_throttle = true};
  precision_landing_bench_start();
  assert(precision_landing_bench_run() && commands[0] == -MAX_PPRZ);
  precision_landing_on_mode_change(0);
  test_mode = 0;
  assert(!precision_landing_bench_run() && commands[0] == 0);
  precision_landing_on_mode_change(2);
  test_mode = 2;
  assert(!precision_landing_bench_run() && commands[0] == 0);
  for (unsigned fault = 0; fault < 7; fault++) {
    autopilot = (struct TestAutopilot){.kill_throttle = true};
    test_speed = (struct EnuCoor_f){0.f, 0.f, 0.f};
    agl_dist_value_filtered = 0.1f;
    agl_measurement_time = test_time;
    precision_landing_bench_start();
    assert(precision_landing_bench_run());
    switch (fault) {
      case 0: autopilot.launch = true; break;
      case 1: autopilot.flight_time = 1; break;
      case 2: autopilot.kill_throttle = false; break;
      case 3: agl_dist_value_filtered = 2.f; break;
      case 4: agl_measurement_time = test_time - 1.f; break;
      case 5: test_speed.x = 2.f; break;
      default: test_speed.z = NAN; break;
    }
    assert(!precision_landing_bench_run() && commands[0] == 0);
    const bool was_killed = autopilot.kill_throttle;
    precision_landing_bench_start();
    assert(!precision_landing_bench_run() && commands[0] == 0);
    assert(autopilot.kill_throttle == was_killed);
  }
  reset();
  test_waypoints[0].altitude = FLT_MAX;
  test_waypoints[1].altitude = -FLT_MAX;
  test_target_altitude = 123.f;
  test_preclimb = 0.f;
  precision_landing_setup(0, 1);
  precision_landing_run();
  precision_landing_glide();
  assert(isfinite(test_target_altitude) && isfinite(test_preclimb));
  assert(commands[0] == 0);
  test_waypoints[0].altitude = 17.f;
  test_waypoints[1].altitude = 0.f;
  reset();
  test_waypoints[1].east = NAN;
  precision_landing_run();
  assert(commands[0] == 0 && (precision_landing_abort || precision_landing_commit_flare));
  precision_landing_flare_run();
  assert(!test_nav_route);
  test_waypoints[1].east = 0.f;
  for (unsigned index = 2; index <= UINT8_MAX; index++) {
    reset();
    precision_landing_setup(0, index);
    precision_landing_run();
    precision_landing_glide();
    assert(commands[0] == 0);
  }
  set_parameters(9.f, 17.f, 3.5f, 2.f, 7.f, 4.f, 0.3f, 2);
  assert(precision_landing_parameters_valid());
  const float invalid[] = {NAN, INFINITY, -INFINITY, FLT_MAX, -FLT_MAX};
  float *params[] = {&precision_landing_approach_airspeed, &precision_landing_final_height,
                     &precision_landing_brake_agl, &precision_landing_flare_agl,
                     &precision_landing_aim_before_td, &precision_landing_touchdown_pitch,
                     &precision_landing_flare_brake};
  for (unsigned field = 0; field < sizeof(params) / sizeof(params[0]); field++) {
    for (unsigned value = 0; value < sizeof(invalid) / sizeof(invalid[0]); value++) {
      const float saved = *params[field];
      *params[field] = invalid[value];
      assert(!precision_landing_parameters_valid());
      *params[field] = saved;
    }
  }
  precision_landing_max_retries = 6;
  assert(!precision_landing_parameters_valid());
  precision_landing_max_retries = 2;
  assert(precision_landing_entry_valid(0, 1, 32.f));
  assert(!precision_landing_entry_valid(0, 0, 32.f));
  assert(!precision_landing_entry_valid(0, 255, 32.f));
  assert(!precision_landing_entry_valid(0, 1, 0.f));
  precision_landing_final_height = NAN;
  assert(!precision_landing_entry_valid(0, 1, 32.f));
  precision_landing_final_height = 17.f;
  test_waypoints[0].east = FLT_MAX / 4.f;
  assert(!precision_landing_entry_valid(0, 1, 32.f));
  test_waypoints[0].east = -150.f;
  assert(!precision_landing_entry_valid(0, 1, FLT_MAX));
  precision_landing_reset_retries();
  assert(precision_landing_retry_allowed());
  precision_landing_record_retry();
  precision_landing_record_retry();
  assert(precision_landing_retry_allowed());
  precision_landing_record_retry();
  assert(!precision_landing_retry_allowed());
  reset();
  agl_dist_value_filtered = 1.f;
  test_waypoints[1].east = NAN;
  precision_landing_run();
  assert(precision_landing_commit_flare && !precision_landing_abort && !precision_landing_cancelled);
  precision_landing_flare_run();
  assert(test_nav_throttle == 0 && !test_nav_route);
  test_waypoints[1].east = 0.f;
  reset();
  agl_dist_valid = false;
  test_altitude = 1.f;
  test_waypoints[1].altitude = -1000.f;
  precision_landing_run();
  assert(precision_landing_commit_flare && !precision_landing_cancelled);
  test_waypoints[1].altitude = 0.f;
  reset();
  test_altitude = agl_dist_value_filtered = 1.f;
  test_position.x = -3.f;
  precision_landing_run();
  assert(!precision_landing_abort && precision_landing_brake_fraction > 0.9f);
  test_airspeed_valid = false;
  precision_landing_run();
  assert(commands[0] == 0 && precision_landing_commit_flare);
  set_parameters(9.f, 17.f, 3.5f, 1.2f, 12.f, 0.f, 0.65f, 2);
  assert(precision_landing_parameters_valid());
  precision_landing_aim_before_td = 20.1f;
  assert(!precision_landing_parameters_valid());
  reset();
  precision_landing_aim_before_td = 12.f;
  precision_landing_setup(0, 1);
  precision_landing_run();
  precision_landing_glide();
  assert(fabsf(test_target_altitude - 8.f * 17.f / 150.f) < 0.001f);
  assert(precision_landing_prediction_rejected(8.1f, 0.f, 8.f, 1.2f));
  assert(precision_landing_prediction_rejected(-8.1f, 0.f, 8.f, 1.2f));
  puts("Precision landing controller checks passed");
  return 0;
}