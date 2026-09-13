#include "modules/nav/precision_landing.h"

#include <assert.h>
#include <math.h>

static void assert_close(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
  assert(precision_landing_prediction_rejected(9.f, 0.f, 8.f, 1.2f));
  assert(precision_landing_prediction_rejected(0.f, -1.3f, 8.f, 1.2f));
  assert(precision_landing_prediction_rejected(NAN, 0.f, 8.f, 1.2f));
  assert(!precision_landing_prediction_rejected(-3.f, 0.5f, 8.f, 1.2f));
  assert(!precision_landing_airspeed_safe(false, 10.f, 8.f));
  assert(!precision_landing_airspeed_safe(true, NAN, 8.f));
  assert(!precision_landing_airspeed_safe(true, 7.f, 8.f));
  assert(precision_landing_airspeed_safe(true, 9.f, 8.f));
  assert(precision_landing_should_commit(1.5f, 2.5f));
  assert(!precision_landing_should_commit(3.f, 2.5f));
  assert(!precision_landing_should_commit(NAN, 2.5f));
  struct PrecisionLandingPrediction prediction;

  prediction = precision_landing_predict(30.f, 0.f, 9.f, 0.f, 3.f, 1.f, 0.25f);
  assert_close(prediction.longitudinal_error_m, 3.f);
  assert_close(prediction.cross_track_error_m, 0.f);

  prediction = precision_landing_predict(30.f, 0.5f, 9.f, 0.f, 3.f, 1.f, 0.25f);
  assert_close(prediction.longitudinal_error_m, 3.f);
  assert_close(prediction.cross_track_error_m, 0.5f);

  prediction = precision_landing_predict(30.f, 0.f, 9.f, 0.5f, 3.f, 1.f, 0.25f);
  assert_close(prediction.cross_track_error_m, -1.5f);

  prediction = precision_landing_predict(30.f, 0.f, 9.f, -0.5f, 3.f, 1.f, 0.25f);
  assert_close(prediction.cross_track_error_m, 1.5f);

  prediction = precision_landing_predict(30.f, 0.f, 7.f, 0.f, 3.f, 1.f, 0.25f);
  assert_close(prediction.longitudinal_error_m, 9.f);

  prediction = precision_landing_predict(20.f, 0.f, 9.f, 0.f, 3.f, 1.f, 0.25f);
  assert(prediction.longitudinal_error_m < 0.f);

  prediction = precision_landing_predict(30.f, 0.f, 9.f, 0.f, 1.f, 0.f, 0.25f);
  assert_close(prediction.longitudinal_error_m, -6.f);

  return 0;
}