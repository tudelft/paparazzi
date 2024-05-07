/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: timeKeeper.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-May-2024 00:26:53
 */

#ifndef TIMEKEEPER_H
#define TIMEKEEPER_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double b_timeKeeper(double *outTime_tv_nsec);

void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec);

void timeKeeper_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for timeKeeper.h
 *
 * [EOF]
 */
