/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_controller_w_ailerons_rebuttal_rtwutil.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Mar-2024 00:35:29
 */

/* Include Files */
#include "Nonlinear_controller_w_ailerons_rebuttal_rtwutil.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int numerator
 * Return Type  : int
 */
int div_nde_s32_floor(int numerator)
{
  int i;
  if ((numerator < 0) && (numerator % 31 != 0)) {
    i = -1;
  } else {
    i = 0;
  }
  return numerator / 31 + i;
}

/*
 * File trailer for Nonlinear_controller_w_ailerons_rebuttal_rtwutil.c
 *
 * [EOF]
 */
