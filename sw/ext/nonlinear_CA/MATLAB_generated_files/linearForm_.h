/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: linearForm_.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 07-May-2024 15:18:42
 */

#ifndef LINEARFORM__H
#define LINEARFORM__H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[378],
                   const double H[169], const double f[14], const double x[14]);

void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[496],
                 const double H[225], const double f[16], const double x[16]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for linearForm_.h
 *
 * [EOF]
 */
