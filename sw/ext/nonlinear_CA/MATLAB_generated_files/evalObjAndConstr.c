/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Mar-2024 00:35:29
 */

/* Include Files */
#include "evalObjAndConstr.h"
#include "Nonlinear_controller_w_ailerons_rebuttal_internal_types.h"
#include "Nonlinear_controller_w_ailerons_rebuttal_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const c_struct_T *c_obj_next_next_next_next_next_
 *                const double x[15]
 *                int *status
 * Return Type  : double
 */
double evalObjAndConstr(const c_struct_T *c_obj_next_next_next_next_next_,
                        const double x[15], int *status)
{
  double a;
  double ab_sigma_1_tmp;
  double b_a;
  double b_sigma_1_tmp;
  double bb_sigma_1_tmp;
  double c_a;
  double c_sigma_1_tmp;
  double cb_sigma_1_tmp;
  double d_a;
  double d_sigma_1_tmp;
  double e_a;
  double e_sigma_1_tmp;
  double f_a;
  double f_sigma_1_tmp;
  double fval;
  double g_a;
  double g_sigma_1_tmp;
  double h_a;
  double h_sigma_1_tmp;
  double i_a;
  double i_sigma_1_tmp;
  double j_a;
  double j_sigma_1_tmp;
  double k_a;
  double k_sigma_1_tmp;
  double l_a;
  double l_sigma_1_tmp;
  double m_a;
  double m_sigma_1_tmp;
  double n_a;
  double n_sigma_1_tmp;
  double o_a;
  double o_sigma_1_tmp;
  double p_a;
  double p_sigma_1_tmp;
  double q_a;
  double q_sigma_1_tmp;
  double r_a;
  double r_sigma_1_tmp;
  double s_a;
  double s_sigma_1_tmp;
  double sigma_1;
  double sigma_1_tmp;
  double sigma_2;
  double sigma_3;
  double sigma_4;
  double sigma_5;
  double sigma_6;
  double t_a;
  double t_sigma_1_tmp;
  double u_a;
  double u_sigma_1_tmp;
  double v_a;
  double v_sigma_1_tmp;
  double w_a;
  double w_sigma_1_tmp;
  double x_sigma_1_tmp;
  double y_sigma_1_tmp;
  bool b;
  sigma_1_tmp =
      cos(c_obj_next_next_next_next_next_->flight_path_angle->contents -
          x[12] * c_obj_next_next_next_next_next_->gain_theta->contents);
  b_sigma_1_tmp =
      sin(c_obj_next_next_next_next_next_->flight_path_angle->contents -
          x[12] * c_obj_next_next_next_next_next_->gain_theta->contents);
  c_sigma_1_tmp = cos(c_obj_next_next_next_next_next_->Beta->contents);
  d_sigma_1_tmp =
      cos(x[12] * c_obj_next_next_next_next_next_->gain_theta->contents);
  e_sigma_1_tmp =
      cos(x[13] * c_obj_next_next_next_next_next_->gain_phi->contents);
  f_sigma_1_tmp =
      sin(x[12] * c_obj_next_next_next_next_next_->gain_theta->contents);
  sigma_6 = cos(x[4] * c_obj_next_next_next_next_next_->gain_el->contents);
  g_sigma_1_tmp =
      cos(x[5] * c_obj_next_next_next_next_next_->gain_el->contents);
  h_sigma_1_tmp =
      cos(x[6] * c_obj_next_next_next_next_next_->gain_el->contents);
  i_sigma_1_tmp =
      cos(x[7] * c_obj_next_next_next_next_next_->gain_el->contents);
  j_sigma_1_tmp =
      sin(x[13] * c_obj_next_next_next_next_next_->gain_phi->contents);
  k_sigma_1_tmp = sin(c_obj_next_next_next_next_next_->Beta->contents);
  l_sigma_1_tmp =
      sin(x[4] * c_obj_next_next_next_next_next_->gain_el->contents);
  m_sigma_1_tmp =
      sin(x[5] * c_obj_next_next_next_next_next_->gain_el->contents);
  n_sigma_1_tmp =
      sin(x[6] * c_obj_next_next_next_next_next_->gain_el->contents);
  o_sigma_1_tmp =
      sin(x[7] * c_obj_next_next_next_next_next_->gain_el->contents);
  p_sigma_1_tmp =
      sin(x[8] * c_obj_next_next_next_next_next_->gain_az->contents);
  q_sigma_1_tmp =
      sin(x[9] * c_obj_next_next_next_next_next_->gain_az->contents);
  r_sigma_1_tmp =
      sin(x[10] * c_obj_next_next_next_next_next_->gain_az->contents);
  s_sigma_1_tmp =
      sin(x[11] * c_obj_next_next_next_next_next_->gain_az->contents);
  t_sigma_1_tmp =
      cos(x[8] * c_obj_next_next_next_next_next_->gain_az->contents);
  u_sigma_1_tmp =
      cos(x[9] * c_obj_next_next_next_next_next_->gain_az->contents);
  v_sigma_1_tmp =
      cos(x[10] * c_obj_next_next_next_next_next_->gain_az->contents);
  w_sigma_1_tmp =
      cos(x[11] * c_obj_next_next_next_next_next_->gain_az->contents);
  a = c_obj_next_next_next_next_next_->V->contents;
  b_a = c_obj_next_next_next_next_next_->V->contents;
  c_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  d_a = c_obj_next_next_next_next_next_->gain_theta->contents;
  e_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  f_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  g_a = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  h_a = c_obj_next_next_next_next_next_->V->contents;
  i_a = c_obj_next_next_next_next_next_->V->contents;
  j_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  k_a = c_obj_next_next_next_next_next_->gain_theta->contents;
  l_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  m_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  n_a = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  o_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  p_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  q_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  r_a = c_obj_next_next_next_next_next_->V->contents;
  s_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  t_a = c_obj_next_next_next_next_next_->gain_theta->contents;
  u_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  v_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  w_a = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  x_sigma_1_tmp = x[12] * x[12];
  y_sigma_1_tmp = x[0] * x[0];
  sigma_5 = x[1] * x[1];
  ab_sigma_1_tmp = x[2] * x[2];
  bb_sigma_1_tmp = x[3] * x[3];
  cb_sigma_1_tmp = ((l_sigma_1_tmp * y_sigma_1_tmp + m_sigma_1_tmp * sigma_5) +
                    n_sigma_1_tmp * ab_sigma_1_tmp) +
                   o_sigma_1_tmp * bb_sigma_1_tmp;
  sigma_1 =
      c_obj_next_next_next_next_next_->dv_global->contents[0] -
      (((((d_sigma_1_tmp *
               (c_obj_next_next_next_next_next_->Cl_alpha->contents *
                    c_obj_next_next_next_next_next_->S->contents * (a * a) *
                    c_obj_next_next_next_next_next_->rho->contents *
                    b_sigma_1_tmp *
                    (c_obj_next_next_next_next_next_->flight_path_angle
                         ->contents -
                     x[12] * c_obj_next_next_next_next_next_->gain_theta
                                 ->contents) /
                    2.0 -
                c_obj_next_next_next_next_next_->S->contents * (b_a * b_a) *
                    c_obj_next_next_next_next_next_->rho->contents *
                    sigma_1_tmp * c_sigma_1_tmp *
                    (((c_obj_next_next_next_next_next_->K_Cd->contents *
                           (c_a * c_a) * x_sigma_1_tmp * (d_a * d_a) -
                       2.0 * c_obj_next_next_next_next_next_->K_Cd->contents *
                           (e_a * e_a) * x[12] *
                           c_obj_next_next_next_next_next_->flight_path_angle
                               ->contents *
                           c_obj_next_next_next_next_next_->gain_theta
                               ->contents) +
                      c_obj_next_next_next_next_next_->K_Cd->contents *
                          (f_a * f_a) * (g_a * g_a)) +
                     c_obj_next_next_next_next_next_->Cd_zero->contents) /
                    2.0) +
           e_sigma_1_tmp * f_sigma_1_tmp *
               (c_obj_next_next_next_next_next_->Cl_alpha->contents *
                    c_obj_next_next_next_next_next_->S->contents * (h_a * h_a) *
                    c_obj_next_next_next_next_next_->rho->contents *
                    sigma_1_tmp *
                    (c_obj_next_next_next_next_next_->flight_path_angle
                         ->contents -
                     x[12] * c_obj_next_next_next_next_next_->gain_theta
                                 ->contents) /
                    2.0 +
                c_obj_next_next_next_next_next_->S->contents * (i_a * i_a) *
                    c_obj_next_next_next_next_next_->rho->contents *
                    b_sigma_1_tmp * c_sigma_1_tmp *
                    (((c_obj_next_next_next_next_next_->K_Cd->contents *
                           (j_a * j_a) * x_sigma_1_tmp * (k_a * k_a) -
                       2.0 * c_obj_next_next_next_next_next_->K_Cd->contents *
                           (l_a * l_a) * x[12] *
                           c_obj_next_next_next_next_next_->flight_path_angle
                               ->contents *
                           c_obj_next_next_next_next_next_->gain_theta
                               ->contents) +
                      c_obj_next_next_next_next_next_->K_Cd->contents *
                          (m_a * m_a) * (n_a * n_a)) +
                     c_obj_next_next_next_next_next_->Cd_zero->contents) /
                    2.0)) -
          c_obj_next_next_next_next_next_->K_p_T->contents * (o_a * o_a) *
              d_sigma_1_tmp * cb_sigma_1_tmp) -
         c_obj_next_next_next_next_next_->K_p_T->contents * (p_a * p_a) *
             e_sigma_1_tmp * f_sigma_1_tmp *
             (((sigma_6 * t_sigma_1_tmp * y_sigma_1_tmp +
                g_sigma_1_tmp * u_sigma_1_tmp * sigma_5) +
               h_sigma_1_tmp * v_sigma_1_tmp * ab_sigma_1_tmp) +
              i_sigma_1_tmp * w_sigma_1_tmp * bb_sigma_1_tmp)) +
        c_obj_next_next_next_next_next_->K_p_T->contents * (q_a * q_a) *
            j_sigma_1_tmp * f_sigma_1_tmp *
            (((sigma_6 * p_sigma_1_tmp * y_sigma_1_tmp +
               g_sigma_1_tmp * q_sigma_1_tmp * sigma_5) +
              h_sigma_1_tmp * r_sigma_1_tmp * ab_sigma_1_tmp) +
             i_sigma_1_tmp * s_sigma_1_tmp * bb_sigma_1_tmp)) -
       c_obj_next_next_next_next_next_->S->contents * (r_a * r_a) *
           c_obj_next_next_next_next_next_->rho->contents * j_sigma_1_tmp *
           f_sigma_1_tmp * k_sigma_1_tmp *
           (((c_obj_next_next_next_next_next_->K_Cd->contents * (s_a * s_a) *
                  x_sigma_1_tmp * (t_a * t_a) -
              2.0 * c_obj_next_next_next_next_next_->K_Cd->contents *
                  (u_a * u_a) * x[12] *
                  c_obj_next_next_next_next_next_->flight_path_angle->contents *
                  c_obj_next_next_next_next_next_->gain_theta->contents) +
             c_obj_next_next_next_next_next_->K_Cd->contents * (v_a * v_a) *
                 (w_a * w_a)) +
            c_obj_next_next_next_next_next_->Cd_zero->contents) /
           2.0) /
          c_obj_next_next_next_next_next_->m->contents;
  a = c_obj_next_next_next_next_next_->V->contents;
  b_a = c_obj_next_next_next_next_next_->V->contents;
  c_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  d_a = c_obj_next_next_next_next_next_->gain_theta->contents;
  e_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  f_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  g_a = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  h_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  i_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  j_a = c_obj_next_next_next_next_next_->V->contents;
  k_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  l_a = c_obj_next_next_next_next_next_->gain_theta->contents;
  m_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  n_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  o_a = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  sigma_2 =
      c_obj_next_next_next_next_next_->dv_global->contents[1] +
      (((j_sigma_1_tmp *
             (c_obj_next_next_next_next_next_->Cl_alpha->contents *
                  c_obj_next_next_next_next_next_->S->contents * (a * a) *
                  c_obj_next_next_next_next_next_->rho->contents * sigma_1_tmp *
                  (c_obj_next_next_next_next_next_->flight_path_angle
                       ->contents -
                   x[12] *
                       c_obj_next_next_next_next_next_->gain_theta->contents) /
                  2.0 +
              c_obj_next_next_next_next_next_->S->contents * (b_a * b_a) *
                  c_obj_next_next_next_next_next_->rho->contents *
                  b_sigma_1_tmp * c_sigma_1_tmp *
                  (((c_obj_next_next_next_next_next_->K_Cd->contents *
                         (c_a * c_a) * x_sigma_1_tmp * (d_a * d_a) -
                     2.0 * c_obj_next_next_next_next_next_->K_Cd->contents *
                         (e_a * e_a) * x[12] *
                         c_obj_next_next_next_next_next_->flight_path_angle
                             ->contents *
                         c_obj_next_next_next_next_next_->gain_theta
                             ->contents) +
                    c_obj_next_next_next_next_next_->K_Cd->contents *
                        (f_a * f_a) * (g_a * g_a)) +
                   c_obj_next_next_next_next_next_->Cd_zero->contents) /
                  2.0) -
         c_obj_next_next_next_next_next_->K_p_T->contents * (h_a * h_a) *
             j_sigma_1_tmp *
             (((cos(x[4] * c_obj_next_next_next_next_next_->gain_el->contents) *
                    cos(x[8] *
                        c_obj_next_next_next_next_next_->gain_az->contents) *
                    (x[0] * x[0]) +
                cos(x[5] * c_obj_next_next_next_next_next_->gain_el->contents) *
                    cos(x[9] *
                        c_obj_next_next_next_next_next_->gain_az->contents) *
                    (x[1] * x[1])) +
               cos(x[6] * c_obj_next_next_next_next_next_->gain_el->contents) *
                   cos(x[10] *
                       c_obj_next_next_next_next_next_->gain_az->contents) *
                   (x[2] * x[2])) +
              cos(x[7] * c_obj_next_next_next_next_next_->gain_el->contents) *
                  cos(x[11] *
                      c_obj_next_next_next_next_next_->gain_az->contents) *
                  (x[3] * x[3]))) -
        c_obj_next_next_next_next_next_->K_p_T->contents * (i_a * i_a) *
            e_sigma_1_tmp *
            (((cos(x[4] * c_obj_next_next_next_next_next_->gain_el->contents) *
                   sin(x[8] *
                       c_obj_next_next_next_next_next_->gain_az->contents) *
                   (x[0] * x[0]) +
               cos(x[5] * c_obj_next_next_next_next_next_->gain_el->contents) *
                   sin(x[9] *
                       c_obj_next_next_next_next_next_->gain_az->contents) *
                   (x[1] * x[1])) +
              cos(x[6] * c_obj_next_next_next_next_next_->gain_el->contents) *
                  sin(x[10] *
                      c_obj_next_next_next_next_next_->gain_az->contents) *
                  (x[2] * x[2])) +
             cos(x[7] * c_obj_next_next_next_next_next_->gain_el->contents) *
                 sin(x[11] *
                     c_obj_next_next_next_next_next_->gain_az->contents) *
                 (x[3] * x[3]))) +
       c_obj_next_next_next_next_next_->S->contents * (j_a * j_a) *
           c_obj_next_next_next_next_next_->rho->contents * e_sigma_1_tmp *
           k_sigma_1_tmp *
           (((c_obj_next_next_next_next_next_->K_Cd->contents * (k_a * k_a) *
                  x_sigma_1_tmp * (l_a * l_a) -
              2.0 * c_obj_next_next_next_next_next_->K_Cd->contents *
                  (m_a * m_a) * x[12] *
                  c_obj_next_next_next_next_next_->flight_path_angle->contents *
                  c_obj_next_next_next_next_next_->gain_theta->contents) +
             c_obj_next_next_next_next_next_->K_Cd->contents * (n_a * n_a) *
                 (o_a * o_a)) +
            c_obj_next_next_next_next_next_->Cd_zero->contents) /
           2.0) /
          c_obj_next_next_next_next_next_->m->contents;
  a = c_obj_next_next_next_next_next_->gain_motor->contents;
  b_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  c_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  d_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  e_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  f_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  g_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  h_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  i_a = c_obj_next_next_next_next_next_->V->contents;
  j_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  k_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  l_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  m_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  n_a = c_obj_next_next_next_next_next_->V->contents;
  o_a = c_obj_next_next_next_next_next_->V->contents;
  sigma_3 =
      ((((((((((((((((2.0 * c_obj_next_next_next_next_next_->I_zz->contents *
                          c_obj_next_next_next_next_next_->p->contents *
                          c_obj_next_next_next_next_next_->r->contents -
                      2.0 * c_obj_next_next_next_next_next_->I_xx->contents *
                          c_obj_next_next_next_next_next_->p->contents *
                          c_obj_next_next_next_next_next_->r->contents) -
                     2.0 * c_obj_next_next_next_next_next_->I_yy->contents *
                         c_obj_next_next_next_next_next_->dv_global
                             ->contents[4]) +
                    2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
                        y_sigma_1_tmp * (a * a) *
                        c_obj_next_next_next_next_next_->l_z->contents *
                        l_sigma_1_tmp) +
                   2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
                       sigma_5 * (b_a * b_a) *
                       c_obj_next_next_next_next_next_->l_z->contents *
                       m_sigma_1_tmp) +
                  2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
                      ab_sigma_1_tmp * (c_a * c_a) *
                      c_obj_next_next_next_next_next_->l_z->contents *
                      n_sigma_1_tmp) +
                 2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
                     bb_sigma_1_tmp * (d_a * d_a) *
                     c_obj_next_next_next_next_next_->l_z->contents *
                     o_sigma_1_tmp) -
                2.0 * c_obj_next_next_next_next_next_->K_p_M->contents *
                    y_sigma_1_tmp * (e_a * e_a) * sigma_6 * p_sigma_1_tmp) +
               2.0 * c_obj_next_next_next_next_next_->K_p_M->contents *
                   sigma_5 * (f_a * f_a) * g_sigma_1_tmp * q_sigma_1_tmp) -
              2.0 * c_obj_next_next_next_next_next_->K_p_M->contents *
                  ab_sigma_1_tmp * (g_a * g_a) * h_sigma_1_tmp *
                  r_sigma_1_tmp) +
             2.0 * c_obj_next_next_next_next_next_->K_p_M->contents *
                 bb_sigma_1_tmp * (h_a * h_a) * i_sigma_1_tmp * s_sigma_1_tmp) +
            c_obj_next_next_next_next_next_->Cm_zero->contents *
                c_obj_next_next_next_next_next_->S->contents * (i_a * i_a) *
                c_obj_next_next_next_next_next_->rho->contents *
                c_obj_next_next_next_next_next_->wing_chord->contents) +
           2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
               (x[0] * x[0]) * (j_a * j_a) *
               c_obj_next_next_next_next_next_->l_4->contents * sigma_6 *
               t_sigma_1_tmp) +
          2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
              (x[1] * x[1]) * (k_a * k_a) *
              c_obj_next_next_next_next_next_->l_4->contents * g_sigma_1_tmp *
              u_sigma_1_tmp) -
         2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
             (x[2] * x[2]) * (l_a * l_a) *
             c_obj_next_next_next_next_next_->l_3->contents * h_sigma_1_tmp *
             v_sigma_1_tmp) -
        2.0 * c_obj_next_next_next_next_next_->K_p_T->contents * (x[3] * x[3]) *
            (m_a * m_a) * c_obj_next_next_next_next_next_->l_3->contents *
            i_sigma_1_tmp * w_sigma_1_tmp) -
       c_obj_next_next_next_next_next_->Cm_alpha->contents *
           c_obj_next_next_next_next_next_->S->contents * (n_a * n_a) *
           c_obj_next_next_next_next_next_->flight_path_angle->contents *
           c_obj_next_next_next_next_next_->rho->contents *
           c_obj_next_next_next_next_next_->wing_chord->contents) +
      c_obj_next_next_next_next_next_->Cm_alpha->contents *
          c_obj_next_next_next_next_next_->S->contents * x[12] * (o_a * o_a) *
          c_obj_next_next_next_next_next_->gain_theta->contents *
          c_obj_next_next_next_next_next_->rho->contents *
          c_obj_next_next_next_next_next_->wing_chord->contents;
  a = c_obj_next_next_next_next_next_->gain_motor->contents;
  b_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  c_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  d_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  e_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  f_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  g_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  h_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  i_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  j_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  k_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  l_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  m_a = c_obj_next_next_next_next_next_->V->contents;
  sigma_4 =
      ((((((((((((((2.0 * c_obj_next_next_next_next_next_->I_yy->contents *
                        c_obj_next_next_next_next_next_->q->contents *
                        c_obj_next_next_next_next_next_->r->contents -
                    2.0 * c_obj_next_next_next_next_next_->I_xx->contents *
                        c_obj_next_next_next_next_next_->dv_global
                            ->contents[3]) -
                   2.0 * c_obj_next_next_next_next_next_->I_zz->contents *
                       c_obj_next_next_next_next_next_->q->contents *
                       c_obj_next_next_next_next_next_->r->contents) +
                  2.0 * c_obj_next_next_next_next_next_->K_p_M->contents *
                      (x[0] * x[0]) * (a * a) * l_sigma_1_tmp) -
                 2.0 * c_obj_next_next_next_next_next_->K_p_M->contents *
                     (x[1] * x[1]) * (b_a * b_a) * m_sigma_1_tmp) +
                2.0 * c_obj_next_next_next_next_next_->K_p_M->contents *
                    (x[2] * x[2]) * (c_a * c_a) * n_sigma_1_tmp) -
               2.0 * c_obj_next_next_next_next_next_->K_p_M->contents *
                   (x[3] * x[3]) * (d_a * d_a) * o_sigma_1_tmp) +
              2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
                  (x[0] * x[0]) * (e_a * e_a) *
                  c_obj_next_next_next_next_next_->l_1->contents * sigma_6 *
                  t_sigma_1_tmp) -
             2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
                 (x[1] * x[1]) * (f_a * f_a) *
                 c_obj_next_next_next_next_next_->l_1->contents *
                 g_sigma_1_tmp * u_sigma_1_tmp) -
            2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
                (x[2] * x[2]) * (g_a * g_a) *
                c_obj_next_next_next_next_next_->l_2->contents * h_sigma_1_tmp *
                v_sigma_1_tmp) +
           2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
               (x[3] * x[3]) * (h_a * h_a) *
               c_obj_next_next_next_next_next_->l_2->contents * i_sigma_1_tmp *
               w_sigma_1_tmp) +
          2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
              (x[0] * x[0]) * (i_a * i_a) *
              c_obj_next_next_next_next_next_->l_z->contents * sigma_6 *
              p_sigma_1_tmp) +
         2.0 * c_obj_next_next_next_next_next_->K_p_T->contents *
             (x[1] * x[1]) * (j_a * j_a) *
             c_obj_next_next_next_next_next_->l_z->contents * g_sigma_1_tmp *
             q_sigma_1_tmp) +
        2.0 * c_obj_next_next_next_next_next_->K_p_T->contents * (x[2] * x[2]) *
            (k_a * k_a) * c_obj_next_next_next_next_next_->l_z->contents *
            h_sigma_1_tmp * r_sigma_1_tmp) +
       2.0 * c_obj_next_next_next_next_next_->K_p_T->contents * (x[3] * x[3]) *
           (l_a * l_a) * c_obj_next_next_next_next_next_->l_z->contents *
           i_sigma_1_tmp * s_sigma_1_tmp) +
      c_obj_next_next_next_next_next_->CL_aileron->contents *
          c_obj_next_next_next_next_next_->S->contents * (m_a * m_a) * x[14] *
          c_obj_next_next_next_next_next_->gain_ailerons->contents *
          c_obj_next_next_next_next_next_->rho->contents;
  a = c_obj_next_next_next_next_next_->gain_motor->contents;
  b_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  c_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  d_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  e_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  f_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  g_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  h_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  i_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  j_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  k_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  l_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  sigma_5 =
      (((((((((((((c_obj_next_next_next_next_next_->I_zz->contents *
                       c_obj_next_next_next_next_next_->dv_global->contents[5] -
                   c_obj_next_next_next_next_next_->I_xx->contents *
                       c_obj_next_next_next_next_next_->p->contents *
                       c_obj_next_next_next_next_next_->q->contents) +
                  c_obj_next_next_next_next_next_->I_yy->contents *
                      c_obj_next_next_next_next_next_->p->contents *
                      c_obj_next_next_next_next_next_->q->contents) +
                 c_obj_next_next_next_next_next_->K_p_T->contents *
                     y_sigma_1_tmp * (a * a) *
                     c_obj_next_next_next_next_next_->l_1->contents *
                     l_sigma_1_tmp) -
                c_obj_next_next_next_next_next_->K_p_T->contents * sigma_5 *
                    (b_a * b_a) *
                    c_obj_next_next_next_next_next_->l_1->contents *
                    m_sigma_1_tmp) -
               c_obj_next_next_next_next_next_->K_p_T->contents *
                   ab_sigma_1_tmp * (c_a * c_a) *
                   c_obj_next_next_next_next_next_->l_2->contents *
                   n_sigma_1_tmp) +
              c_obj_next_next_next_next_next_->K_p_T->contents *
                  bb_sigma_1_tmp * (d_a * d_a) *
                  c_obj_next_next_next_next_next_->l_2->contents *
                  o_sigma_1_tmp) -
             c_obj_next_next_next_next_next_->K_p_M->contents * y_sigma_1_tmp *
                 (e_a * e_a) * sigma_6 * t_sigma_1_tmp) +
            c_obj_next_next_next_next_next_->K_p_M->contents * sigma_5 *
                (f_a * f_a) * g_sigma_1_tmp * u_sigma_1_tmp) -
           c_obj_next_next_next_next_next_->K_p_M->contents * ab_sigma_1_tmp *
               (g_a * g_a) * h_sigma_1_tmp * v_sigma_1_tmp) +
          c_obj_next_next_next_next_next_->K_p_M->contents * bb_sigma_1_tmp *
              (h_a * h_a) * i_sigma_1_tmp * w_sigma_1_tmp) -
         c_obj_next_next_next_next_next_->K_p_T->contents * (x[0] * x[0]) *
             (i_a * i_a) * c_obj_next_next_next_next_next_->l_4->contents *
             sigma_6 * p_sigma_1_tmp) -
        c_obj_next_next_next_next_next_->K_p_T->contents * (x[1] * x[1]) *
            (j_a * j_a) * c_obj_next_next_next_next_next_->l_4->contents *
            g_sigma_1_tmp * q_sigma_1_tmp) +
       c_obj_next_next_next_next_next_->K_p_T->contents * (x[2] * x[2]) *
           (k_a * k_a) * c_obj_next_next_next_next_next_->l_3->contents *
           h_sigma_1_tmp * r_sigma_1_tmp) +
      c_obj_next_next_next_next_next_->K_p_T->contents * (x[3] * x[3]) *
          (l_a * l_a) * c_obj_next_next_next_next_next_->l_3->contents *
          i_sigma_1_tmp * s_sigma_1_tmp;
  a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  b_a = c_obj_next_next_next_next_next_->gain_theta->contents;
  c_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  d_a = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  e_a = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  sigma_6 = ((c_obj_next_next_next_next_next_->K_Cd->contents * (a * a) *
                  x_sigma_1_tmp * (b_a * b_a) -
              2.0 * c_obj_next_next_next_next_next_->K_Cd->contents *
                  (c_a * c_a) * x[12] *
                  c_obj_next_next_next_next_next_->flight_path_angle->contents *
                  c_obj_next_next_next_next_next_->gain_theta->contents) +
             c_obj_next_next_next_next_next_->K_Cd->contents * (d_a * d_a) *
                 (e_a * e_a)) +
            c_obj_next_next_next_next_next_->Cd_zero->contents;
  a = c_obj_next_next_next_next_next_->V->contents;
  b_a = c_obj_next_next_next_next_next_->V->contents;
  c_a = c_obj_next_next_next_next_next_->V->contents;
  d_a = c_obj_next_next_next_next_next_->V->contents;
  e_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  f_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  g_a = c_obj_next_next_next_next_next_->gain_motor->contents;
  h_a = c_obj_next_next_next_next_next_->V->contents;
  sigma_6 =
      (100.0 *
           (((((f_sigma_1_tmp *
                    (c_obj_next_next_next_next_next_->S->contents * (a * a) *
                         c_obj_next_next_next_next_next_->rho->contents *
                         sigma_6 * sigma_1_tmp * c_sigma_1_tmp / 2.0 -
                     c_obj_next_next_next_next_next_->Cl_alpha->contents *
                         c_obj_next_next_next_next_next_->S->contents *
                         (b_a * b_a) *
                         c_obj_next_next_next_next_next_->rho->contents *
                         b_sigma_1_tmp *
                         (c_obj_next_next_next_next_next_->flight_path_angle
                              ->contents -
                          x[12] * c_obj_next_next_next_next_next_->gain_theta
                                      ->contents) /
                         2.0) +
                e_sigma_1_tmp * d_sigma_1_tmp *
                    (c_obj_next_next_next_next_next_->Cl_alpha->contents *
                         c_obj_next_next_next_next_next_->S->contents *
                         (c_a * c_a) *
                         c_obj_next_next_next_next_next_->rho->contents *
                         sigma_1_tmp *
                         (c_obj_next_next_next_next_next_->flight_path_angle
                              ->contents -
                          x[12] * c_obj_next_next_next_next_next_->gain_theta
                                      ->contents) /
                         2.0 +
                     c_obj_next_next_next_next_next_->S->contents *
                         (d_a * d_a) *
                         c_obj_next_next_next_next_next_->rho->contents *
                         sigma_6 * b_sigma_1_tmp * c_sigma_1_tmp / 2.0)) +
               c_obj_next_next_next_next_next_->K_p_T->contents * (e_a * e_a) *
                   f_sigma_1_tmp * cb_sigma_1_tmp) -
              c_obj_next_next_next_next_next_->K_p_T->contents * (f_a * f_a) *
                  e_sigma_1_tmp * d_sigma_1_tmp *
                  (((cos(x[4] *
                         c_obj_next_next_next_next_next_->gain_el->contents) *
                         cos(x[8] * c_obj_next_next_next_next_next_->gain_az
                                        ->contents) *
                         (x[0] * x[0]) +
                     cos(x[5] *
                         c_obj_next_next_next_next_next_->gain_el->contents) *
                         cos(x[9] * c_obj_next_next_next_next_next_->gain_az
                                        ->contents) *
                         (x[1] * x[1])) +
                    cos(x[6] *
                        c_obj_next_next_next_next_next_->gain_el->contents) *
                        cos(x[10] * c_obj_next_next_next_next_next_->gain_az
                                        ->contents) *
                        (x[2] * x[2])) +
                   cos(x[7] *
                       c_obj_next_next_next_next_next_->gain_el->contents) *
                       cos(x[11] *
                           c_obj_next_next_next_next_next_->gain_az->contents) *
                       (x[3] * x[3]))) +
             c_obj_next_next_next_next_next_->K_p_T->contents * (g_a * g_a) *
                 d_sigma_1_tmp * j_sigma_1_tmp *
                 (((cos(x[4] *
                        c_obj_next_next_next_next_next_->gain_el->contents) *
                        sin(x[8] * c_obj_next_next_next_next_next_->gain_az
                                       ->contents) *
                        (x[0] * x[0]) +
                    cos(x[5] *
                        c_obj_next_next_next_next_next_->gain_el->contents) *
                        sin(x[9] * c_obj_next_next_next_next_next_->gain_az
                                       ->contents) *
                        (x[1] * x[1])) +
                   cos(x[6] *
                       c_obj_next_next_next_next_next_->gain_el->contents) *
                       sin(x[10] *
                           c_obj_next_next_next_next_next_->gain_az->contents) *
                       (x[2] * x[2])) +
                  cos(x[7] *
                      c_obj_next_next_next_next_next_->gain_el->contents) *
                      sin(x[11] *
                          c_obj_next_next_next_next_next_->gain_az->contents) *
                      (x[3] * x[3]))) -
            c_obj_next_next_next_next_next_->S->contents * (h_a * h_a) *
                c_obj_next_next_next_next_next_->rho->contents * sigma_6 *
                d_sigma_1_tmp * j_sigma_1_tmp * k_sigma_1_tmp / 2.0) /
           c_obj_next_next_next_next_next_->m->contents -
       100.0 * c_obj_next_next_next_next_next_->dv_global->contents[2]) +
      981.0;
  a = c_obj_next_next_next_next_next_->W_act_motor->contents;
  b_a = x[0] - c_obj_next_next_next_next_next_->desired_motor_value->contents /
                   c_obj_next_next_next_next_next_->gain_motor->contents;
  c_a = c_obj_next_next_next_next_next_->W_act_motor->contents;
  d_a = x[1] - c_obj_next_next_next_next_next_->desired_motor_value->contents /
                   c_obj_next_next_next_next_next_->gain_motor->contents;
  e_a = c_obj_next_next_next_next_next_->W_act_motor->contents;
  f_a = x[2] - c_obj_next_next_next_next_next_->desired_motor_value->contents /
                   c_obj_next_next_next_next_next_->gain_motor->contents;
  g_a = c_obj_next_next_next_next_next_->W_act_motor->contents;
  h_a = x[3] - c_obj_next_next_next_next_next_->desired_motor_value->contents /
                   c_obj_next_next_next_next_next_->gain_motor->contents;
  i_a = c_obj_next_next_next_next_next_->W_act_phi->contents;
  j_a = x[13] - c_obj_next_next_next_next_next_->desired_phi_value->contents /
                    c_obj_next_next_next_next_next_->gain_phi->contents;
  k_a = c_obj_next_next_next_next_next_->W_act_theta->contents;
  l_a = x[12] - c_obj_next_next_next_next_next_->desired_theta_value->contents /
                    c_obj_next_next_next_next_next_->gain_theta->contents;
  m_a = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  n_a = x[4] - c_obj_next_next_next_next_next_->desired_el_value->contents /
                   c_obj_next_next_next_next_next_->gain_el->contents;
  o_a = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  p_a = x[5] - c_obj_next_next_next_next_next_->desired_el_value->contents /
                   c_obj_next_next_next_next_next_->gain_el->contents;
  q_a = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  r_a = x[6] - c_obj_next_next_next_next_next_->desired_el_value->contents /
                   c_obj_next_next_next_next_next_->gain_el->contents;
  s_a = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  t_a = x[7] - c_obj_next_next_next_next_next_->desired_el_value->contents /
                   c_obj_next_next_next_next_next_->gain_el->contents;
  u_a = c_obj_next_next_next_next_next_->W_act_ailerons->contents;
  v_a = x[14] -
        c_obj_next_next_next_next_next_->desired_ailerons_value->contents /
            c_obj_next_next_next_next_next_->gain_ailerons->contents;
  w_a = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  g_sigma_1_tmp =
      x[8] - c_obj_next_next_next_next_next_->desired_az_value->contents /
                 c_obj_next_next_next_next_next_->gain_az->contents;
  h_sigma_1_tmp = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  i_sigma_1_tmp =
      x[9] - c_obj_next_next_next_next_next_->desired_az_value->contents /
                 c_obj_next_next_next_next_next_->gain_az->contents;
  l_sigma_1_tmp = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  m_sigma_1_tmp =
      x[10] - c_obj_next_next_next_next_next_->desired_az_value->contents /
                  c_obj_next_next_next_next_next_->gain_az->contents;
  n_sigma_1_tmp = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  o_sigma_1_tmp =
      x[11] - c_obj_next_next_next_next_next_->desired_az_value->contents /
                  c_obj_next_next_next_next_next_->gain_az->contents;
  p_sigma_1_tmp = c_obj_next_next_next_next_next_->W_dv_1->contents;
  q_sigma_1_tmp = c_obj_next_next_next_next_next_->W_dv_2->contents;
  r_sigma_1_tmp = c_obj_next_next_next_next_next_->W_dv_3->contents;
  s_sigma_1_tmp = c_obj_next_next_next_next_next_->W_dv_4->contents;
  t_sigma_1_tmp = c_obj_next_next_next_next_next_->I_xx->contents;
  u_sigma_1_tmp = c_obj_next_next_next_next_next_->W_dv_5->contents;
  v_sigma_1_tmp = c_obj_next_next_next_next_next_->I_yy->contents;
  w_sigma_1_tmp = c_obj_next_next_next_next_next_->W_dv_6->contents;
  y_sigma_1_tmp = c_obj_next_next_next_next_next_->I_zz->contents;
  fval = (((((c_obj_next_next_next_next_next_->gamma_quadratic_du->contents *
                  ((((((((((((((a * a * (b_a * b_a) + c_a * c_a * (d_a * d_a)) +
                               e_a * e_a * (f_a * f_a)) +
                              g_a * g_a * (h_a * h_a)) +
                             i_a * i_a * (j_a * j_a)) +
                            k_a * k_a * (l_a * l_a)) +
                           m_a * m_a * (n_a * n_a)) +
                          o_a * o_a * (p_a * p_a)) +
                         q_a * q_a * (r_a * r_a)) +
                        s_a * s_a * (t_a * t_a)) +
                       u_a * u_a * (v_a * v_a)) +
                      w_a * w_a * (g_sigma_1_tmp * g_sigma_1_tmp)) +
                     h_sigma_1_tmp * h_sigma_1_tmp *
                         (i_sigma_1_tmp * i_sigma_1_tmp)) +
                    l_sigma_1_tmp * l_sigma_1_tmp *
                        (m_sigma_1_tmp * m_sigma_1_tmp)) +
                   n_sigma_1_tmp * n_sigma_1_tmp *
                       (o_sigma_1_tmp * o_sigma_1_tmp)) +
              p_sigma_1_tmp * p_sigma_1_tmp * (sigma_1 * sigma_1)) +
             q_sigma_1_tmp * q_sigma_1_tmp * (sigma_2 * sigma_2)) +
            r_sigma_1_tmp * r_sigma_1_tmp * (sigma_6 * sigma_6) / 10000.0) +
           s_sigma_1_tmp * s_sigma_1_tmp * (sigma_4 * sigma_4) /
               (4.0 * (t_sigma_1_tmp * t_sigma_1_tmp))) +
          u_sigma_1_tmp * u_sigma_1_tmp * (sigma_3 * sigma_3) /
              (4.0 * (v_sigma_1_tmp * v_sigma_1_tmp))) +
         w_sigma_1_tmp * w_sigma_1_tmp * (sigma_5 * sigma_5) /
             (y_sigma_1_tmp * y_sigma_1_tmp);
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
