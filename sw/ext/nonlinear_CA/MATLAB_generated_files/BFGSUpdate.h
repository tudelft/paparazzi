/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: BFGSUpdate.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 28-May-2024 17:44:56
 */

#ifndef BFGSUPDATE_H
#define BFGSUPDATE_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
bool BFGSUpdate(int nvar, double Bk[225], const double sk[16], double yk[16],
                double workspace[496]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for BFGSUpdate.h
 *
 * [EOF]
 */
