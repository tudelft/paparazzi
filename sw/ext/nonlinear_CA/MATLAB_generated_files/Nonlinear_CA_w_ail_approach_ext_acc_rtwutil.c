/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_CA_w_ail_approach_ext_acc_rtwutil.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

/* Include Files */
#include "Nonlinear_CA_w_ail_approach_ext_acc_rtwutil.h"
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
 * File trailer for Nonlinear_CA_w_ail_approach_ext_acc_rtwutil.c
 *
 * [EOF]
 */
