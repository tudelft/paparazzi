/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: path_optimizer_fcn.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "path_optimizer_fcn.h"
#include "compute_constraints_and_constraints_gradient_w_ship_coeff.h"
#include "fmincon.h"
#include "linspace.h"
#include "path_optimizer_fcn_data.h"
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_initialize.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include <stdio.h>
#include <string.h>

/* Function Definitions */
/*
 * Coefficients for a 5th-order polynomial in each dimension (30 coefficients
 * total)
 *
 * Arguments    : const double coeffs_ship_prediction[18]
 *                double landing_time_init
 *                double time_sample_while
 *                const double P0[3]
 *                const double V0[3]
 *                const double A0[3]
 *                const double v_max[3]
 *                const double v_min[3]
 *                const double a_max[3]
 *                const double a_min[3]
 *                double num_points
 *                double num_points_plot
 *                double pos_gain
 *                double speed_gain
 *                double acc_gain
 *                double max_time_of_landing_seconds
 *                double verbose
 *                double optimal_coeffs[18]
 *                double *landing_time
 * Return Type  : void
 */
void path_optimizer_fcn(const double coeffs_ship_prediction[18],
                        double landing_time_init, double time_sample_while,
                        const double P0[3], const double V0[3],
                        const double A0[3], const double v_max[3],
                        const double v_min[3], const double a_max[3],
                        const double a_min[3], double num_points,
                        double num_points_plot, double pos_gain,
                        double speed_gain, double acc_gain,
                        double max_time_of_landing_seconds, double verbose,
                        double optimal_coeffs[18], double *landing_time)
{
  double initial_guess_coeff[18];
  double b_expl_temp;
  double c_expl_temp;
  double d_expl_temp;
  double e_expl_temp;
  double elapsed_time;
  double exitflag;
  double f_expl_temp;
  int i;
  (void)num_points_plot;
  if (!isInitialized_path_optimizer_fcn) {
    path_optimizer_fcn_initialize();
  }
  initial_guess_coeff[4] = V0[0];
  initial_guess_coeff[5] = P0[0];
  initial_guess_coeff[10] = V0[1];
  initial_guess_coeff[11] = P0[1];
  initial_guess_coeff[0] = 0.0;
  initial_guess_coeff[6] = 0.0;
  initial_guess_coeff[12] = 0.0;
  initial_guess_coeff[1] = 0.0;
  initial_guess_coeff[7] = 0.0;
  initial_guess_coeff[13] = 0.0;
  initial_guess_coeff[2] = 0.0;
  initial_guess_coeff[8] = 0.0;
  initial_guess_coeff[14] = 0.0;
  initial_guess_coeff[3] = 0.0;
  initial_guess_coeff[9] = 0.0;
  initial_guess_coeff[15] = 0.0;
  initial_guess_coeff[16] = V0[2];
  initial_guess_coeff[17] = P0[2];
  memcpy(&optimal_coeffs[0], &initial_guess_coeff[0], 18U * sizeof(double));
  tic();
  *landing_time = landing_time_init;
  exitflag = -1.0;
  while ((exitflag < 0.0) && (*landing_time < max_time_of_landing_seconds)) {
    char expl_temp[3];
    *landing_time += time_sample_while;
    /*  Objective function to minimize time (function of coefficients) */
    /*  Constraints: speed, acceleration, and matching landing speed with
     * platform's speed */
    /*  Perform optimization */
    fmincon(initial_guess_coeff, P0, V0, A0, v_max, v_min, a_max, a_min,
            *landing_time, num_points, coeffs_ship_prediction, pos_gain,
            speed_gain, acc_gain, optimal_coeffs, &exitflag, &elapsed_time,
            &b_expl_temp, expl_temp, &c_expl_temp, &d_expl_temp, &e_expl_temp,
            &f_expl_temp);
  }
  elapsed_time = toc();
  /*  Display the total travel time and computational time.  */
  if (verbose != 0.0) {
    if (*landing_time > max_time_of_landing_seconds) {
      printf("Landing time computed exceeds max_time_of_landing_seconds. "
             "Solution unfeasible.\n");
      fflush(stdout);
    } else {
      printf("Total travel time for the optimized path: %.2f seconds, computed "
             "in %.2f seconds\n",
             *landing_time, elapsed_time);
      fflush(stdout);
      printf("Optimization exit flag: %f \n", exitflag);
      fflush(stdout);
      printf("Path coefficients determined:\n");
      fflush(stdout);
      printf("[ ");
      fflush(stdout);
      for (i = 0; i < 18; i++) {
        printf("%.2f ", optimal_coeffs[i]);
        fflush(stdout);
        if ((i + 1 == 6) || (i + 1 == 12)) {
          printf("; \n");
          fflush(stdout);
        }
      }
      printf("]");
      fflush(stdout);
      printf("\n");
      fflush(stdout);
    }
  }
}

/*
 * Arguments    : const double P0[3]
 *                const double V0[3]
 *                const double A0[3]
 *                const double v_max[3]
 *                const double v_min[3]
 *                const double a_max[3]
 *                const double a_min[3]
 *                double landing_time
 *                double num_points
 *                const double coeffs_ship_prediction[18]
 *                double pos_gain
 *                double speed_gain
 *                double acc_gain
 *                const double coeffs[18]
 *                double varargout_1_data[]
 *                int varargout_1_size[2]
 *                double varargout_2_data[]
 *                int varargout_2_size[2]
 *                double varargout_3_data[]
 *                int varargout_3_size[2]
 *                double varargout_4_data[]
 *                int varargout_4_size[2]
 * Return Type  : void
 */
void path_optimizer_fcn_anonFcn2(
    const double P0[3], const double V0[3], const double A0[3],
    const double v_max[3], const double v_min[3], const double a_max[3],
    const double a_min[3], double landing_time, double num_points,
    const double coeffs_ship_prediction[18], double pos_gain, double speed_gain,
    double acc_gain, const double coeffs[18], double varargout_1_data[],
    int varargout_1_size[2], double varargout_2_data[], int varargout_2_size[2],
    double varargout_3_data[], int varargout_3_size[2],
    double varargout_4_data[], int varargout_4_size[2])
{
  static creal_T c_gradient_data[2304];
  emxArray_real_T *t_array;
  double ceq_gradient_data[324];
  double *t_array_data;
  int c_gradient_size[2];
  int ceq_gradient_size[2];
  int i;
  int i1;
  /*  Path constraints function for quadratic path */
  emxInit_real_T(&t_array, 2);
  linspace(landing_time, num_points, t_array);
  t_array_data = t_array->data;
  c_compute_constraints_and_const(
      A0[0], a_min[0], a_max[0], A0[1], a_min[1], a_max[1], A0[2], a_min[2],
      a_max[2], V0[0], v_min[0], v_max[0], V0[1], v_min[1], v_max[1], V0[2],
      v_min[2], v_max[2], P0[0], P0[1], P0[2], acc_gain, coeffs[0], coeffs[1],
      coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7],
      coeffs[8], coeffs[9], coeffs[10], coeffs[11], coeffs[12], coeffs[13],
      coeffs[14], coeffs[15], coeffs[16], coeffs[17], coeffs_ship_prediction[0],
      coeffs_ship_prediction[1], coeffs_ship_prediction[2],
      coeffs_ship_prediction[3], coeffs_ship_prediction[4],
      coeffs_ship_prediction[5], coeffs_ship_prediction[6],
      coeffs_ship_prediction[7], coeffs_ship_prediction[8],
      coeffs_ship_prediction[9], coeffs_ship_prediction[10],
      coeffs_ship_prediction[11], coeffs_ship_prediction[12],
      coeffs_ship_prediction[13], coeffs_ship_prediction[14],
      coeffs_ship_prediction[15], coeffs_ship_prediction[16],
      coeffs_ship_prediction[17], pos_gain, speed_gain, t_array_data[1],
      t_array_data[2], t_array_data[3], t_array_data[4], t_array_data[5],
      t_array_data[6], t_array_data[7], t_array_data[8],
      t_array_data[t_array->size[1] - 1], varargout_1_data, varargout_1_size,
      varargout_2_data, varargout_2_size, c_gradient_data, c_gradient_size,
      ceq_gradient_data, ceq_gradient_size);
  emxFree_real_T(&t_array);
  varargout_3_size[0] = 18;
  varargout_3_size[1] = 128;
  for (i = 0; i < 128; i++) {
    for (i1 = 0; i1 < 18; i1++) {
      varargout_3_data[i1 + 18 * i] = c_gradient_data[i + 128 * i1].re;
    }
  }
  varargout_4_size[0] = 18;
  varargout_4_size[1] = 18;
  for (i = 0; i < 18; i++) {
    for (i1 = 0; i1 < 18; i1++) {
      varargout_4_data[i1 + 18 * i] = ceq_gradient_data[i + 18 * i1];
    }
  }
}

/*
 * File trailer for path_optimizer_fcn.c
 *
 * [EOF]
 */
