/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Cascaded_nonlinear_controller_w_ail_new_aero_initialize.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-May-2024 01:26:38
 */

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_initialize.h"
#include "Cascaded_nonlinear_controller_w_ail_new_aero_data.h"
#include "CoderTimeAPI.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void Cascaded_nonlinear_controller_w_ail_new_aero_initialize(void)
{
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  isInitialized_Cascaded_nonlinear_controller_w_ail_new_aero = true;
}

/*
 * File trailer for Cascaded_nonlinear_controller_w_ail_new_aero_initialize.c
 *
 * [EOF]
 */
