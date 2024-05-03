/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgeqp3.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef XGEQP3_H
#define XGEQP3_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xgeqp3(double A[729], int m, int n, int jpvt[27], double tau[27]);

void xgeqp3(double A[961], int m, int n, int jpvt[31], double tau[31]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgeqp3.h
 *
 * [EOF]
 */
