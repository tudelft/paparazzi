/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rtGetNaN.c
 *
 * MATLAB Coder version            : 5.4
<<<<<<< HEAD:sw/ext/nonlinear_CA/MATLAB_generated_files/rtGetNaN.c
 * C/C++ source code generated on  : 26-Oct-2023 15:50:45
=======
 * C/C++ source code generated on  : 16-Nov-2023 01:21:31
>>>>>>> tudelft/overactuated_quadplane_outer_inner_loop_nonlinear_ca:sw/ext/Nonlinear_CA/MATLAB_generated_files/rtGetNaN.c
 */

/*
 * Abstract:
 *       MATLAB for code generation function to initialize non-finite, NaN
 */
/* Include Files */
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

/*
 * Function: rtGetNaN
 * ======================================================================
 *  Abstract:
 * Initialize rtNaN needed by the generated code.
 *  NaN is initialized as non-signaling. Assumes IEEE.
 */
real_T rtGetNaN(void)
{
  return rtNaN;
}

/*
 * Function: rtGetNaNF
 * =====================================================================
 *  Abstract:
 *  Initialize rtNaNF needed by the generated code.
 *  NaN is initialized as non-signaling. Assumes IEEE
 */
real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/*
 * File trailer for rtGetNaN.c
 *
 * [EOF]
 */
