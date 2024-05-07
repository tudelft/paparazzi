/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-May-2024 00:26:53
 */

/* Include Files */
#include "evalObjAndConstr.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "compute_cost_and_gradient_w_Mx_noah.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : c_struct_T *c_obj_next_next_next_next_next_
 *                const double x[15]
 *                int *status
 * Return Type  : double
 */
double evalObjAndConstr(c_struct_T *c_obj_next_next_next_next_next_,
                        const double x[15], int *status)
{
  double gradient_data[15];
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double fval;
  int gradient_size;
  bool b;
  c_obj_next_next_next_next_next_->Omega_1_scaled->contents = x[0];
  c_obj_next_next_next_next_next_->Omega_2_scaled->contents = x[1];
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  fval = c_compute_cost_and_gradient_w_M(
      c_obj_next_next_next_next_next_->Beta->contents,
      c_obj_next_next_next_next_next_->CL_aileron->contents,
      c_obj_next_next_next_next_next_->Cd_zero->contents,
      c_obj_next_next_next_next_next_->Cl_alpha->contents,
      c_obj_next_next_next_next_next_->Cm_zero->contents,
      c_obj_next_next_next_next_next_->Cm_alpha->contents,
      c_obj_next_next_next_next_next_->I_xx->contents,
      c_obj_next_next_next_next_next_->I_yy->contents,
      c_obj_next_next_next_next_next_->I_zz->contents,
      c_obj_next_next_next_next_next_->K_Cd->contents,
      c_obj_next_next_next_next_next_->Omega_1_scaled->contents,
      c_obj_next_next_next_next_next_->Omega_2_scaled->contents, x[2], x[3],
      x[13], c_obj_next_next_next_next_next_->S->contents, x[12],
      c_obj_next_next_next_next_next_->V->contents,
      c_obj_next_next_next_next_next_->V_scaled->contents,
      c_obj_next_next_next_next_next_->W_act_phi->contents,
      c_obj_next_next_next_next_next_->W_act_theta->contents,
      c_obj_next_next_next_next_next_->W_act_motor->contents,
      c_obj_next_next_next_next_next_->W_dv_1->contents,
      c_obj_next_next_next_next_next_->W_dv_2->contents,
      c_obj_next_next_next_next_next_->W_dv_3->contents,
      c_obj_next_next_next_next_next_->W_dv_4->contents,
      c_obj_next_next_next_next_next_->W_dv_5->contents,
      c_obj_next_next_next_next_next_->W_dv_6->contents,
      c_obj_next_next_next_next_next_->W_act_tilt_el->contents,
      c_obj_next_next_next_next_next_->W_act_tilt_az->contents,
      c_obj_next_next_next_next_next_->W_act_ailerons->contents, x[4], x[5],
      x[6], x[7], x[14],
      c_obj_next_next_next_next_next_->desired_el_value->contents,
      c_obj_next_next_next_next_next_->desired_az_value->contents,
      c_obj_next_next_next_next_next_->desired_phi_value->contents,
      c_obj_next_next_next_next_next_->desired_theta_value->contents,
      c_obj_next_next_next_next_next_->desired_motor_value->contents,
      c_obj_next_next_next_next_next_->desired_ailerons_value->contents,
      dv_global_1, dv_global_2, dv_global_3, dv_global_4, dv_global_5,
      dv_global_6, c_obj_next_next_next_next_next_->flight_path_angle->contents,
      x[8], x[9], x[10], x[11],
      c_obj_next_next_next_next_next_->gain_el->contents,
      c_obj_next_next_next_next_next_->gain_az->contents,
      c_obj_next_next_next_next_next_->gain_phi->contents,
      c_obj_next_next_next_next_next_->gain_theta->contents,
      c_obj_next_next_next_next_next_->gain_motor->contents,
      c_obj_next_next_next_next_next_->gain_ailerons->contents,
      c_obj_next_next_next_next_next_->gamma_quadratic_du->contents,
      c_obj_next_next_next_next_next_->l_1->contents,
      c_obj_next_next_next_next_next_->l_2->contents,
      c_obj_next_next_next_next_next_->l_3->contents,
      c_obj_next_next_next_next_next_->l_4->contents,
      c_obj_next_next_next_next_next_->l_z->contents,
      c_obj_next_next_next_next_next_->m->contents,
      c_obj_next_next_next_next_next_->p->contents,
      c_obj_next_next_next_next_next_->power_Cd_0->contents,
      c_obj_next_next_next_next_next_->power_Cd_a->contents,
      c_obj_next_next_next_next_next_->prop_R->contents,
      c_obj_next_next_next_next_next_->prop_Cd_0->contents,
      c_obj_next_next_next_next_next_->prop_Cl_0->contents,
      c_obj_next_next_next_next_next_->prop_Cd_a->contents,
      c_obj_next_next_next_next_next_->prop_Cl_a->contents,
      c_obj_next_next_next_next_next_->prop_delta->contents,
      c_obj_next_next_next_next_next_->prop_sigma->contents,
      c_obj_next_next_next_next_next_->prop_theta->contents,
      c_obj_next_next_next_next_next_->q->contents,
      c_obj_next_next_next_next_next_->r->contents,
      c_obj_next_next_next_next_next_->rho->contents,
      c_obj_next_next_next_next_next_->wing_span->contents,
      c_obj_next_next_next_next_next_->wing_chord->contents, gradient_data,
      &gradient_size);
  *status = 1;
  b = rtIsNaN(fval);
  if (rtIsInf(fval) || b) {
    if (b) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  }
  if (*status == 1) {
    *status = 1;
  }
  return fval;
}

/*
 * File trailer for evalObjAndConstr.c
 *
 * [EOF]
 */
