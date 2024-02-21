/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Cascaded_nonlinear_TestFlight_rtwutil.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:30:21
 */

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_rtwutil.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : int
 */
int div_nde_s32_floor(int numerator, int denominator)
{
  int i;
  if (((numerator < 0) != (denominator < 0)) &&
      (numerator % denominator != 0)) {
    i = -1;
  } else {
    i = 0;
  }
  return numerator / denominator + i;
}

/*
 * File trailer for Cascaded_nonlinear_TestFlight_rtwutil.c
 *
 * [EOF]
 */
