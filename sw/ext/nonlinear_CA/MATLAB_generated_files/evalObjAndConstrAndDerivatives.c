/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstrAndDerivatives.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

/* Include Files */
#include "evalObjAndConstrAndDerivatives.h"
#include "Nonlinear_CA_w_ail_approach_ext_acc.h"
#include "Nonlinear_CA_w_ail_approach_ext_acc_internal_types.h"
#include "Nonlinear_CA_w_ail_approach_ext_acc_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const c_struct_T *c_obj_next_next_next_next_next_
 *                const double x[15]
 *                double grad_workspace[16]
 *                int *status
 * Return Type  : double
 */
double evalObjAndConstrAndDerivatives(
    const c_struct_T *c_obj_next_next_next_next_next_, const double x[15],
    double grad_workspace[16], int *status)
{
  double varargout_2[15];
  double fval;
  bool allFinite;
  fval = compute_cost_and_gradient_fcn(
      c_obj_next_next_next_next_next_->dv_global,
      c_obj_next_next_next_next_next_->gain_theta,
      c_obj_next_next_next_next_next_->Cl_alpha,
      c_obj_next_next_next_next_next_->S, c_obj_next_next_next_next_next_->V,
      c_obj_next_next_next_next_next_->rho,
      c_obj_next_next_next_next_next_->flight_path_angle,
      c_obj_next_next_next_next_next_->Beta,
      c_obj_next_next_next_next_next_->K_Cd,
      c_obj_next_next_next_next_next_->Cd_zero,
      c_obj_next_next_next_next_next_->gain_phi,
      c_obj_next_next_next_next_next_->K_p_T,
      c_obj_next_next_next_next_next_->gain_motor,
      c_obj_next_next_next_next_next_->gain_el,
      c_obj_next_next_next_next_next_->gain_az,
      c_obj_next_next_next_next_next_->m, c_obj_next_next_next_next_next_->I_zz,
      c_obj_next_next_next_next_next_->p, c_obj_next_next_next_next_next_->r,
      c_obj_next_next_next_next_next_->I_xx,
      c_obj_next_next_next_next_next_->I_yy,
      c_obj_next_next_next_next_next_->l_z,
      c_obj_next_next_next_next_next_->K_p_M,
      c_obj_next_next_next_next_next_->Cm_zero,
      c_obj_next_next_next_next_next_->wing_chord,
      c_obj_next_next_next_next_next_->l_4,
      c_obj_next_next_next_next_next_->l_3,
      c_obj_next_next_next_next_next_->Cm_alpha,
      c_obj_next_next_next_next_next_->q, c_obj_next_next_next_next_next_->l_1,
      c_obj_next_next_next_next_next_->l_2,
      c_obj_next_next_next_next_next_->CL_aileron,
      c_obj_next_next_next_next_next_->gain_ailerons,
      c_obj_next_next_next_next_next_->W_dv_4,
      c_obj_next_next_next_next_next_->W_dv_6,
      c_obj_next_next_next_next_next_->W_act_motor,
      c_obj_next_next_next_next_next_->gamma_quadratic_du,
      c_obj_next_next_next_next_next_->desired_motor_value,
      c_obj_next_next_next_next_next_->W_dv_5,
      c_obj_next_next_next_next_next_->W_dv_3,
      c_obj_next_next_next_next_next_->W_dv_1,
      c_obj_next_next_next_next_next_->W_dv_2,
      c_obj_next_next_next_next_next_->W_act_tilt_el,
      c_obj_next_next_next_next_next_->desired_el_value,
      c_obj_next_next_next_next_next_->W_act_tilt_az,
      c_obj_next_next_next_next_next_->desired_az_value,
      c_obj_next_next_next_next_next_->W_act_theta,
      c_obj_next_next_next_next_next_->desired_theta_value,
      c_obj_next_next_next_next_next_->W_act_phi,
      c_obj_next_next_next_next_next_->desired_phi_value,
      c_obj_next_next_next_next_next_->W_act_ailerons,
      c_obj_next_next_next_next_next_->desired_ailerons_value, x, varargout_2);
  memcpy(&grad_workspace[0], &varargout_2[0], 15U * sizeof(double));
  *status = 1;
  allFinite = rtIsNaN(fval);
  if (rtIsInf(fval) || allFinite) {
    if (allFinite) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    int idx_current;
    allFinite = true;
    idx_current = 0;
    while (allFinite && (idx_current + 1 <= 15)) {
      allFinite = ((!rtIsInf(grad_workspace[idx_current])) &&
                   (!rtIsNaN(grad_workspace[idx_current])));
      idx_current++;
    }
    if (!allFinite) {
      idx_current--;
      if (rtIsNaN(grad_workspace[idx_current])) {
        *status = -3;
      } else if (grad_workspace[idx_current] < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    }
  }
  if (*status == 1) {
    *status = 1;
  }
  return fval;
}

/*
 * File trailer for evalObjAndConstrAndDerivatives.c
 *
 * [EOF]
 */
