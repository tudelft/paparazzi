/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fmincon.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef FMINCON_H
#define FMINCON_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double fmincon(const double x0[18], const double nonlcon_workspace_P0[3],
               const double nonlcon_workspace_V0[3],
               const double nonlcon_workspace_A0[3],
               const double nonlcon_workspace_v_max[3],
               const double nonlcon_workspace_v_min[3],
               const double nonlcon_workspace_a_max[3],
               const double nonlcon_workspace_a_min[3],
               double nonlcon_workspace_landing_time,
               double nonlcon_workspace_num_points,
               const double c_nonlcon_workspace_coeffs_ship[18],
               double nonlcon_workspace_pos_gain,
               double nonlcon_workspace_speed_gain,
               double nonlcon_workspace_acc_gain, double x[18],
               double *exitflag, double *output_iterations,
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
