/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rt_nonfinite.h
 *
 * MATLAB Coder version            : 5.4
<<<<<<< HEAD:sw/ext/nonlinear_CA/MATLAB_generated_files/rt_nonfinite.h
 * C/C++ source code generated on  : 26-Oct-2023 15:50:45
=======
 * C/C++ source code generated on  : 16-Nov-2023 01:21:31
>>>>>>> tudelft/overactuated_quadplane_outer_inner_loop_nonlinear_ca:sw/ext/Nonlinear_CA/MATLAB_generated_files/rt_nonfinite.h
 */

#ifndef RT_NONFINITE_H
#define RT_NONFINITE_H

/* Include Files */
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;

extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

#ifdef __cplusplus
}
#endif
#endif
/*
 * File trailer for rt_nonfinite.h
 *
 * [EOF]
 */
