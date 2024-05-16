/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: timeKeeper.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 16-May-2024 19:15:38
 */

/* Include Files */
#include "timeKeeper.h"
#include "Nonlinear_controller_w_ail_basic_aero_sl_data.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"
#include <string.h>

/* Variable Definitions */
static coderTimespec savedTime;

static bool savedTime_not_empty;

/* Function Definitions */
/*
 * Arguments    : double *outTime_tv_nsec
 * Return Type  : double
 */
double b_timeKeeper(double *outTime_tv_nsec)
{
  double outTime_tv_sec;
  outTime_tv_sec = savedTime.tv_sec;
  *outTime_tv_nsec = savedTime.tv_nsec;
  return outTime_tv_sec;
}

/*
 * Arguments    : double newTime_tv_sec
 *                double newTime_tv_nsec
 * Return Type  : void
 */
void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec)
{
  if (!savedTime_not_empty) {
    coderTimespec b_timespec;
    if (!freq_not_empty) {
      freq_not_empty = true;
      coderInitTimeFunctions(&freq);
    }
    coderTimeClockGettimeMonotonic(&b_timespec, freq);
    savedTime_not_empty = true;
  }
  savedTime.tv_sec = newTime_tv_sec;
  savedTime.tv_nsec = newTime_tv_nsec;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void timeKeeper_init(void)
{
  savedTime_not_empty = false;
}

/*
 * File trailer for timeKeeper.c
 *
 * [EOF]
 */
