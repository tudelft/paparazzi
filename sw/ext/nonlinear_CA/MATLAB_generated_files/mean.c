/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mean.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

/* Include Files */
#include "mean.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x[4]
 * Return Type  : double
 */
double mean(const double x[4])
{
  return (((x[0] + x[1]) + x[2]) + x[3]) / 4.0;
}

/*
 * File trailer for mean.c
 *
 * [EOF]
 */
