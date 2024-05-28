/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_controller_w_ail_new_aero_sl_initialize.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 28-May-2024 17:44:56
 */

/* Include Files */
#include "Nonlinear_controller_w_ail_new_aero_sl_initialize.h"
#include "CoderTimeAPI.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void Nonlinear_controller_w_ail_new_aero_sl_initialize(void)
{
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  isInitialized_Nonlinear_controller_w_ail_new_aero_sl = true;
}

/*
 * File trailer for Nonlinear_controller_w_ail_new_aero_sl_initialize.c
 *
 * [EOF]
 */
