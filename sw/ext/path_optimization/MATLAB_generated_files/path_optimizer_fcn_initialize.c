/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: path_optimizer_fcn_initialize.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "path_optimizer_fcn_initialize.h"
#include "CoderTimeAPI.h"
#include "path_optimizer_fcn_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void path_optimizer_fcn_initialize(void)
{
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  isInitialized_path_optimizer_fcn = true;
}

/*
 * File trailer for path_optimizer_fcn_initialize.c
 *
 * [EOF]
 */
