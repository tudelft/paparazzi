/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: checkNonlinearInputs.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "checkNonlinearInputs.h"
#include "path_optimizer_fcn.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x0[18]
 *                const double nonlcon_workspace_P0[3]
 *                const double nonlcon_workspace_V0[3]
 *                const double nonlcon_workspace_A0[3]
 *                const double nonlcon_workspace_v_max[3]
 *                const double nonlcon_workspace_v_min[3]
 *                const double nonlcon_workspace_a_max[3]
 *                const double nonlcon_workspace_a_min[3]
 *                double nonlcon_workspace_landing_time
 *                double nonlcon_workspace_num_points
 *                const double c_nonlcon_workspace_coeffs_ship[18]
 *                double nonlcon_workspace_pos_gain
 *                double nonlcon_workspace_speed_gain
 *                double nonlcon_workspace_acc_gain
 *                int *mNonlinEq
 * Return Type  : int
 */
int checkNonlinearInputs(
    const double x0[18], const double nonlcon_workspace_P0[3],
    const double nonlcon_workspace_V0[3], const double nonlcon_workspace_A0[3],
    const double nonlcon_workspace_v_max[3],
    const double nonlcon_workspace_v_min[3],
    const double nonlcon_workspace_a_max[3],
    const double nonlcon_workspace_a_min[3],
    double nonlcon_workspace_landing_time, double nonlcon_workspace_num_points,
    const double c_nonlcon_workspace_coeffs_ship[18],
    double nonlcon_workspace_pos_gain, double nonlcon_workspace_speed_gain,
    double nonlcon_workspace_acc_gain, int *mNonlinEq)
{
  double varargout_3_data[2304];
  double varargout_4_data[324];
  double varargout_1_data[128];
  double varargout_2_data[18];
  int varargout_1_size[2];
  int varargout_2_size[2];
  int varargout_3_size[2];
  int varargout_4_size[2];
  int mNonlinIneq;
  path_optimizer_fcn_anonFcn2(
      nonlcon_workspace_P0, nonlcon_workspace_V0, nonlcon_workspace_A0,
      nonlcon_workspace_v_max, nonlcon_workspace_v_min, nonlcon_workspace_a_max,
      nonlcon_workspace_a_min, nonlcon_workspace_landing_time,
      nonlcon_workspace_num_points, c_nonlcon_workspace_coeffs_ship,
      nonlcon_workspace_pos_gain, nonlcon_workspace_speed_gain,
      nonlcon_workspace_acc_gain, x0, varargout_1_data, varargout_1_size,
      varargout_2_data, varargout_2_size, varargout_3_data, varargout_3_size,
      varargout_4_data, varargout_4_size);
  mNonlinIneq = 128;
  *mNonlinEq = 18;
  return mNonlinIneq;
}

/*
 * File trailer for checkNonlinearInputs.c
 *
 * [EOF]
 */
