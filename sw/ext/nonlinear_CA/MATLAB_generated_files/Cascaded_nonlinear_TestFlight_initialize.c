/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Cascaded_nonlinear_TestFlight_initialize.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_initialize.h"
#include "Cascaded_nonlinear_TestFlight_data.h"
#include "CoderTimeAPI.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void Cascaded_nonlinear_TestFlight_initialize(void)
{
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  isInitialized_Cascaded_nonlinear_TestFlight = true;
}

/*
 * File trailer for Cascaded_nonlinear_TestFlight_initialize.c
 *
 * [EOF]
 */
