/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fmincon.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-May-2024 01:26:38
 */

#ifndef FMINCON_H
#define FMINCON_H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double b_fmincon(e_struct_T *fun_workspace, double x0[13], const double lb[13],
                 const double ub[13], double *exitflag,
                 double *output_iterations, double *output_funcCount,
                 char output_algorithm[3], double *output_constrviolation,
                 double *output_stepsize, double *output_lssteplength,
                 double *output_firstorderopt);

double fmincon(d_struct_T *fun_workspace, double x0[15], const double lb[15],
               const double ub[15], double *exitflag, double *output_iterations,
               double *output_funcCount, char output_algorithm[3],
               double *output_constrviolation, double *output_stepsize,
               double *output_lssteplength, double *output_firstorderopt);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for fmincon.h
 *
 * [EOF]
 */
