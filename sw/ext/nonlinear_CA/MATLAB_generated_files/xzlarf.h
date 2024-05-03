/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlarf.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef XZLARF_H
#define XZLARF_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xzlarf(int m, int n, int iv0, double tau, double C[729], int ic0,
              double work[27]);

void xzlarf(int m, int n, int iv0, double tau, double C[961], int ic0,
            double work[31]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xzlarf.h
 *
 * [EOF]
 */
