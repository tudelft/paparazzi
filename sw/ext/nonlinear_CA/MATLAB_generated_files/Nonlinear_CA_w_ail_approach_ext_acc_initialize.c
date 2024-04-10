/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_CA_w_ail_approach_ext_acc_initialize.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

/* Include Files */
#include "Nonlinear_CA_w_ail_approach_ext_acc_initialize.h"
#include "CoderTimeAPI.h"
#include "Nonlinear_CA_w_ail_approach_ext_acc_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void Nonlinear_CA_w_ail_approach_ext_acc_initialize(void)
{
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  isInitialized_Nonlinear_CA_w_ail_approach_ext_acc = true;
}

/*
 * File trailer for Nonlinear_CA_w_ail_approach_ext_acc_initialize.c
 *
 * [EOF]
 */
