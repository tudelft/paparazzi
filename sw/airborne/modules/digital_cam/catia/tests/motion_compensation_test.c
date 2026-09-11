#include "../motion_compensation.h"
#include <assert.h>
#include <stdlib.h>

int main(void)
{
  int32_t latitude = 0, longitude = 0;
  assert(compensate_ground_position(&latitude, &longitude, 15, 0, 0.32));
  assert(abs(latitude - 434) <= 1 && longitude == 0);
  latitude = longitude = 0;
  assert(compensate_ground_position(&latitude, &longitude, 15, acos(-1) / 2, 0.32));
  assert(latitude == 0 && abs(longitude - 431) <= 1);
  latitude = 488100000;
  longitude = 78530000;
  assert(compensate_ground_position(&latitude, &longitude, 0, 1, 0.32));
  assert(latitude == 488100000 && longitude == 78530000);
  assert(!compensate_ground_position(&latitude, &longitude, 15, 0, 3));
  assert(!compensate_ground_position(&latitude, &longitude, 15, 0, NAN));
  assert(!compensate_ground_position(&latitude, &longitude, -1, 0, .1));
  assert(latitude == 488100000 && longitude == 78530000);
  latitude = 0;
  longitude = 1799999999;
  assert(compensate_ground_position(&latitude, &longitude, 15, acos(-1) / 2, .32));
  assert(longitude < -1799999000);
}