/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: step.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:29:34
 */

#ifndef STEP_H
#define STEP_H

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
bool b_step(int *STEP_TYPE, double Hessian[169], const double lb[13],
            const double ub[13], p_struct_T *TrialState,
            b_struct_T *MeritFunction, k_struct_T *memspace,
            q_struct_T *WorkingSet, i_struct_T *QRManager,
            j_struct_T *CholManager, c_struct_T *QPObjective,
            r_struct_T *qpoptions);

bool step(int *STEP_TYPE, double Hessian[225], const double lb[15],
          const double ub[15], n_struct_T *TrialState,
          b_struct_T *MeritFunction, h_struct_T *memspace,
          o_struct_T *WorkingSet, f_struct_T *QRManager,
          g_struct_T *CholManager, struct_T *QPObjective,
          r_struct_T *qpoptions);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for step.h
 *
 * [EOF]
 */
