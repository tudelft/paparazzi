#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "std.h"
#include "modules/sonar/agl_dist.h"

float test_time;
uint8_t test_agl_sender;
void (*test_agl_callback)(uint8_t, uint32_t, float);

int main(void)
{
  test_time = 10.f;
  agl_dist_init();
  assert(test_agl_sender == 42);
  test_time += 0.01f;
  test_agl_callback(42, 0, 5.5f);
  assert(agl_dist_valid && agl_dist_value_filtered == 5.5f);
  test_time += 0.1f;
  test_agl_callback(42, 0, 4.5f);
  assert(fabsf(agl_dist_value_filtered - 5.3f) < 0.0001f);
  test_agl_callback(42, 0, 7.f);
  assert(!agl_dist_valid);
  test_time += 0.01f;
  test_agl_callback(42, 0, 3.f);
  assert(agl_dist_valid && agl_dist_value_filtered == 3.f);
  test_time += 1.f;
  test_agl_callback(42, 0, 5.5f);
  assert(agl_dist_value_filtered == 5.5f);
  test_agl_callback(42, 0, 0.004f);
  assert(!agl_dist_valid);
  test_agl_callback(42, 0, NAN);
  assert(!agl_dist_valid);
  puts("Landing AGL configuration and reacquisition checks passed");
  return 0;
}