#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_int.h"
#include "pprz_algebra_print.h"

static void test_1(void);

int main(int argc, char **argv)
{

  test_1();
  return 0;

}

static void test_1(void)
{
  struct FloatQuat att_err;
  struct FloatQuat att_quat = {0.9914, -0.0170, 0.0663, 0.1110};
  struct FloatQuat quat_sp_f = {0.9713, 0.0027, 0.1977, 0.1319};
  float_quat_inv_comp_norm_shortest(&att_err, &att_quat, &quat_sp_f);

  struct FloatQuat quat = {0.7200,   -0.4018,    0.2724 ,   0.4959};
  struct FloatQuat tilt;
  struct FloatQuat twist;

  float_quat_tilt_twist(&tilt, &twist, &att_err);

  DISPLAY_FLOAT_QUAT("tilt", tilt);
  DISPLAY_FLOAT_QUAT("twist", twist);

}