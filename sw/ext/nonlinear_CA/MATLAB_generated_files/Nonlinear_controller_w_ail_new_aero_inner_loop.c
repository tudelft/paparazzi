/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_controller_w_ail_new_aero_inner_loop.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-Oct-2024 12:47:26
 */

/* Include Files */
#include "Nonlinear_controller_w_ail_new_aero_inner_loop.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
  double grad[14];
  double Hx[13];
  bool hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  bool prev_hasLinear;
  double gammaScalar;
} struct_T;

#endif                                 /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T

typedef struct {
  double penaltyParam;
  double threshold;
  int nPenaltyDecreases;
  double linearizedConstrViol;
  double initFval;
  double initConstrViolationEq;
  double initConstrViolationIneq;
  double phi;
  double phiPrimePlus;
  double phiFullStep;
  double feasRelativeFactor;
  double nlpPrimalFeasError;
  double nlpDualFeasError;
  double nlpComplError;
  double firstOrderOpt;
  bool hasObjective;
} b_struct_T;

#endif                                 /* typedef_b_struct_T */

#ifndef typedef_captured_var
#define typedef_captured_var

typedef struct {
  double contents;
} captured_var;

#endif                                 /* typedef_captured_var */

#ifndef typedef_b_captured_var
#define typedef_b_captured_var

typedef struct {
  double contents[6];
} b_captured_var;

#endif                                 /* typedef_b_captured_var */

#ifndef typedef_c_struct_T
#define typedef_c_struct_T

typedef struct {
  b_captured_var *dv_global;
  captured_var *Beta;
  captured_var *CL_aileron;
  captured_var *Cd_zero;
  captured_var *Cl_alpha;
  captured_var *Cm_zero;
  captured_var *Cm_alpha;
  captured_var *I_xx;
  captured_var *I_yy;
  captured_var *I_zz;
  captured_var *K_Cd;
  captured_var *Phi;
  captured_var *S;
  captured_var *Theta;
  captured_var *V;
  captured_var *W_act_motor;
  captured_var *W_dv_1;
  captured_var *W_dv_2;
  captured_var *W_dv_3;
  captured_var *W_dv_4;
  captured_var *W_dv_5;
  captured_var *W_dv_6;
  captured_var *W_act_tilt_el;
  captured_var *W_act_tilt_az;
  captured_var *W_act_ailerons;
  captured_var *desired_el_value;
  captured_var *desired_az_value;
  captured_var *desired_motor_value;
  captured_var *desired_ailerons_value;
  captured_var *flight_path_angle;
  captured_var *gain_el;
  captured_var *gain_az;
  captured_var *gain_motor;
  captured_var *gain_ailerons;
  captured_var *gamma_quadratic_du;
  captured_var *l_1;
  captured_var *l_2;
  captured_var *l_3;
  captured_var *l_4;
  captured_var *l_z;
  captured_var *m;
  captured_var *p;
  captured_var *q;
  captured_var *r;
  captured_var *rho;
  captured_var *wing_chord;
  captured_var *V_scaled;
  captured_var *power_Cd_0;
  captured_var *power_Cd_a;
  captured_var *prop_R;
  captured_var *prop_Cd_0;
  captured_var *prop_Cl_0;
  captured_var *prop_Cd_a;
  captured_var *prop_Cl_a;
  captured_var *prop_delta;
  captured_var *prop_sigma;
  captured_var *prop_theta;
  captured_var *wing_span;
} c_struct_T;

#endif                                 /* typedef_c_struct_T */

#ifndef typedef_nested_function
#define typedef_nested_function

typedef struct {
  c_struct_T workspace;
} nested_function;

#endif                                 /* typedef_nested_function */

#ifndef typedef_coder_internal_stickyStruct
#define typedef_coder_internal_stickyStruct

typedef struct {
  nested_function value;
} coder_internal_stickyStruct;

#endif                                 /* typedef_coder_internal_stickyStruct */

#ifndef typedef_b_coder_internal_stickyStruct
#define typedef_b_coder_internal_stickyStruct

typedef struct {
  coder_internal_stickyStruct next;
} b_coder_internal_stickyStruct;

#endif                                 /* typedef_b_coder_internal_stickyStruct */

#ifndef typedef_c_coder_internal_stickyStruct
#define typedef_c_coder_internal_stickyStruct

typedef struct {
  b_coder_internal_stickyStruct next;
} c_coder_internal_stickyStruct;

#endif                                 /* typedef_c_coder_internal_stickyStruct */

#ifndef typedef_d_coder_internal_stickyStruct
#define typedef_d_coder_internal_stickyStruct

typedef struct {
  c_coder_internal_stickyStruct next;
} d_coder_internal_stickyStruct;

#endif                                 /* typedef_d_coder_internal_stickyStruct */

#ifndef typedef_e_coder_internal_stickyStruct
#define typedef_e_coder_internal_stickyStruct

typedef struct {
  d_coder_internal_stickyStruct next;
} e_coder_internal_stickyStruct;

#endif                                 /* typedef_e_coder_internal_stickyStruct */

#ifndef typedef_f_coder_internal_stickyStruct
#define typedef_f_coder_internal_stickyStruct

typedef struct {
  e_coder_internal_stickyStruct next;
} f_coder_internal_stickyStruct;

#endif                                 /* typedef_f_coder_internal_stickyStruct */

#ifndef typedef_g_coder_internal_stickyStruct
#define typedef_g_coder_internal_stickyStruct

typedef struct {
  f_coder_internal_stickyStruct next;
} g_coder_internal_stickyStruct;

#endif                                 /* typedef_g_coder_internal_stickyStruct */

#ifndef typedef_h_coder_internal_stickyStruct
#define typedef_h_coder_internal_stickyStruct

typedef struct {
  g_coder_internal_stickyStruct next;
} h_coder_internal_stickyStruct;

#endif                                 /* typedef_h_coder_internal_stickyStruct */

#ifndef typedef_i_coder_internal_stickyStruct
#define typedef_i_coder_internal_stickyStruct

typedef struct {
  h_coder_internal_stickyStruct next;
} i_coder_internal_stickyStruct;

#endif                                 /* typedef_i_coder_internal_stickyStruct */

#ifndef typedef_d_struct_T
#define typedef_d_struct_T

typedef struct {
  int ldq;
  double QR[729];
  double Q[729];
  int jpvt[27];
  int mrows;
  int ncols;
  double tau[27];
  int minRowCol;
  bool usedPivoting;
} d_struct_T;

#endif                                 /* typedef_d_struct_T */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T

typedef struct {
  double FMat[729];
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  bool ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
} e_struct_T;

#endif                                 /* typedef_e_struct_T */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T

typedef struct {
  double workspace_double[378];
  int workspace_int[27];
  int workspace_sort[27];
} f_struct_T;

#endif                                 /* typedef_f_struct_T */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T

typedef struct {
  nested_function objfun;
  double f_1;
  double f_2;
  int nVar;
  int mIneq;
  int mEq;
  int numEvals;
  bool SpecifyObjectiveGradient;
  bool SpecifyConstraintGradient;
  bool isEmptyNonlcon;
  bool hasLB[13];
  bool hasUB[13];
  bool hasBounds;
  int FiniteDifferenceType;
} g_struct_T;

#endif                                 /* typedef_g_struct_T */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T

typedef struct {
  int nVarMax;
  int mNonlinIneq;
  int mNonlinEq;
  int mIneq;
  int mEq;
  int iNonIneq0;
  int iNonEq0;
  double sqpFval;
  double sqpFval_old;
  double xstarsqp[13];
  double xstarsqp_old[13];
  double grad[14];
  double grad_old[14];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[27];
  double lambdaStopTest[27];
  double lambdaStopTestPrev[27];
  double steplength;
  double delta_x[14];
  double socDirection[14];
  int workingset_old[27];
  double gradLag[14];
  double delta_gradLag[14];
  double xstar[14];
  double fstar;
  double firstorderopt;
  double lambda[27];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[14];
} h_struct_T;

#endif                                 /* typedef_h_struct_T */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T

typedef struct {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double lb[14];
  double ub[14];
  int indexLB[14];
  int indexUB[14];
  int indexFixed[14];
  int mEqRemoved;
  double ATwset[378];
  double bwset[27];
  int nActiveConstr;
  double maxConstrWorkspace[27];
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  bool isActiveConstr[27];
  int Wid[27];
  int Wlocalidx[27];
  int nWConstr[5];
  int probType;
  double SLACK0;
} i_struct_T;

#endif                                 /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T

typedef struct {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ObjectiveLimit;
} j_struct_T;

#endif                                 /* typedef_j_struct_T */

#ifndef typedef_k_struct_T
#define typedef_k_struct_T

typedef struct {
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
} k_struct_T;

#endif                                 /* typedef_k_struct_T */

/* Variable Definitions */
static double freq;
static bool freq_not_empty;
static coderTimespec savedTime;
static bool savedTime_not_empty;
static bool isInitialized_Nonlinear_controller_w_ail_new_aero_inner_loop = false;

/* Function Declarations */
static bool BFGSUpdate(int nvar, double Bk[169], const double sk[14], double yk
  [14], double workspace[378]);
static void PresolveWorkingSet(h_struct_T *solution, f_struct_T *memspace,
  i_struct_T *workingset, d_struct_T *qrmanager);
static void RemoveDependentIneq_(i_struct_T *workingset, d_struct_T *qrmanager,
  f_struct_T *memspace, double tolfactor);
static void addBoundToActiveSetMatrix_(i_struct_T *obj, int TYPE, int idx_local);
static void b_computeGradLag(double workspace[378], int nVar, const double grad
  [14], const int finiteFixed[14], int mFixed, const int finiteLB[14], int mLB,
  const int finiteUB[14], int mUB, const double lambda[27]);
static void b_driver(const double lb[13], const double ub[13], h_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     i_coder_internal_stickyStruct *FcnEvaluator, f_struct_T
                     *memspace, i_struct_T *WorkingSet, double Hessian[169],
                     d_struct_T *QRManager, e_struct_T *CholManager, struct_T
                     *QPObjective);
static double b_maxConstraintViolation(const i_struct_T *obj, const double x[14]);
static double b_norm(const double x[6]);
static void b_test_exit(k_struct_T *Flags, f_struct_T *memspace, b_struct_T
  *MeritFunction, const i_struct_T *WorkingSet, h_struct_T *TrialState,
  d_struct_T *QRManager, const double lb[13], const double ub[13]);
static double b_timeKeeper(double *outTime_tv_nsec);
static void b_xgemm(int m, int n, int k, const double A[729], int ia0, const
                    double B[378], double C[729]);
static void b_xgemv(int m, int n, const double A[729], const double x[14],
                    double y[378]);
static double b_xnrm2(int n, const double x[14]);
static void c_CoderTimeAPI_callCoderClockGe(void);
static double c_computeObjectiveAndUserGradie(const c_struct_T
  *c_obj_next_next_next_next_next_, const double x[13], double grad_workspace[14],
  int *status);
static void c_compute_acc_nonlinear_CA_new_(const double u_in[15], double p,
  double q, double r, double m, double I_xx, double I_yy, double I_zz, double
  l_1, double l_2, double l_3, double l_4, double l_z, double Cl_alpha, double
  Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero, double CL_aileron,
  double rho, double V, double S, double wing_chord, double flight_path_angle,
  double Beta, double wing_span, double prop_R, double prop_Cd_0, double
  prop_Cl_0, double prop_Cd_a, double prop_Cl_a, double prop_delta, double
  prop_sigma, double prop_theta, double V_scaled, double Omega_1_scaled, double
  Omega_2_scaled, double accelerations_array[6]);
static double c_compute_cost_and_gradient_sec(const double in1[13], const double
  in2[6], double Beta, double CL_aileron, double Cd_zero, double Cl_alpha,
  double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz, double
  K_Cd, double Phi, double S, double Theta, double V, double W_act_motor, double
  W_dv_1, double W_dv_2, double W_dv_3, double W_dv_4, double W_dv_5, double
  W_dv_6, double W_act_tilt_el, double W_act_tilt_az, double W_act_ailerons,
  double desired_el_value, double desired_az_value, double desired_motor_value,
  double desired_ailerons_value, double flight_path_angle, double gain_el,
  double gain_az, double gain_motor, double gain_ailerons, double
  gamma_quadratic_du, double l_1, double l_2, double l_3, double l_4, double l_z,
  double m, double p, double q, double r, double rho, double wing_chord, double
  V_scaled, double power_Cd_0, double power_Cd_a, double prop_R, double
  prop_Cd_0, double prop_Cl_0, double prop_Cd_a, double prop_Cl_a, double
  prop_delta, double prop_sigma, double prop_theta, double wing_span, double
  gradient_data[], int *gradient_size);
static double computeComplError(const double xCurrent[13], const int finiteLB[14],
  int mLB, const double lb[13], const int finiteUB[14], int mUB, const double
  ub[13], const double lambda[27], int iL0);
static double computeFval(const struct_T *obj, double workspace[378], const
  double H[169], const double f[14], const double x[14]);
static double computeFval_ReuseHx(const struct_T *obj, double workspace[378],
  const double f[14], const double x[14]);
static void computeGradLag(double workspace[14], int nVar, const double grad[14],
  const int finiteFixed[14], int mFixed, const int finiteLB[14], int mLB, const
  int finiteUB[14], int mUB, const double lambda[27]);
static void computeGrad_StoreHx(struct_T *obj, const double H[169], const double
  f[14], const double x[14]);
static void computeQ_(d_struct_T *obj, int nrows);
static void compute_deltax(const double H[169], h_struct_T *solution, f_struct_T
  *memspace, const d_struct_T *qrmanager, e_struct_T *cholmanager, const
  struct_T *objective, bool alwaysPositiveDef);
static void countsort(int x[27], int xLen, int workspace[27], int xMin, int xMax);
static void deleteColMoveEnd(d_struct_T *obj, int idx);
static int div_nde_s32_floor(int numerator, int denominator);
static void driver(const double H[169], const double f[14], h_struct_T *solution,
                   f_struct_T *memspace, i_struct_T *workingset, d_struct_T
                   *qrmanager, e_struct_T *cholmanager, struct_T *objective,
                   j_struct_T *options, int runTimeOptions_MaxIterations);
static double evalObjAndConstr(const c_struct_T *c_obj_next_next_next_next_next_,
  const double x[13], int *status);
static void factorQR(d_struct_T *obj, const double A[378], int mrows, int ncols);
static void factoryConstruct(c_struct_T *objfun_workspace, const double lb[13],
  const double ub[13], g_struct_T *obj);
static bool feasibleX0ForWorkingSet(double workspace[378], double xCurrent[14],
  const i_struct_T *workingset, d_struct_T *qrmanager);
static double feasibleratiotest(const double solution_xstar[14], const double
  solution_searchDir[14], int workingset_nVar, const double workingset_lb[14],
  const double workingset_ub[14], const int workingset_indexLB[14], const int
  workingset_indexUB[14], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[27], const int
  workingset_nWConstr[5], bool isPhaseOne, bool *newBlocking, int *constrType,
  int *constrIdx);
static double fmincon(c_struct_T *fun_workspace, double x0[13], const double lb
                      [13], const double ub[13], double *exitflag, double
                      *output_iterations, double *output_funcCount, char
                      output_algorithm[3], double *output_constrviolation,
                      double *output_stepsize, double *output_lssteplength,
                      double *output_firstorderopt);
static double ft_1(const double ct[227], double gradient_data[], int
                   *gradient_size);
static void fullColLDL2_(e_struct_T *obj, int NColsRemain);
static void iterate(const double H[169], const double f[14], h_struct_T
                    *solution, f_struct_T *memspace, i_struct_T *workingset,
                    d_struct_T *qrmanager, e_struct_T *cholmanager, struct_T
                    *objective, const char options_SolverName[7], double
                    options_StepTolerance, double options_ObjectiveLimit, int
                    runTimeOptions_MaxIterations);
static void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[378],
  const double H[169], const double f[14], const double x[14]);
static double maxConstraintViolation(const i_struct_T *obj, const double x[378],
  int ix0);
static double maximum(const double x[2]);
static double minimum(const double x[2]);
static void qrf(double A[729], int m, int n, int nfxd, double tau[27]);
static void removeConstr(i_struct_T *obj, int idx_global);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static void setProblemType(i_struct_T *obj, int PROBLEM_TYPE);
static void solve(const e_struct_T *obj, double rhs[14]);
static void sortLambdaQP(double lambda[27], int WorkingSet_nActiveConstr, const
  int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[27], const int WorkingSet_Wlocalidx[27], double workspace[378]);
static bool step(int *STEP_TYPE, double Hessian[169], const double lb[13], const
                 double ub[13], h_struct_T *TrialState, b_struct_T
                 *MeritFunction, f_struct_T *memspace, i_struct_T *WorkingSet,
                 d_struct_T *QRManager, e_struct_T *CholManager, struct_T
                 *QPObjective, j_struct_T *qpoptions);
static bool test_exit(b_struct_T *MeritFunction, const i_struct_T *WorkingSet,
                      h_struct_T *TrialState, const double lb[13], const double
                      ub[13], bool *Flags_fevalOK, bool *Flags_done, bool
                      *Flags_stepAccepted, bool *Flags_failedLineSearch, int
                      *Flags_stepType);
static void tic(void);
static void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec);
static void timeKeeper_init(void);
static double toc(void);
static void xgemm(int m, int n, int k, const double A[169], int lda, const
                  double B[729], int ib0, double C[378]);
static void xgemv(int m, int n, const double A[169], int lda, const double x[14],
                  double y[13]);
static void xgeqp3(double A[729], int m, int n, int jpvt[27], double tau[27]);
static double xnrm2(int n, const double x[729], int ix0);
static int xpotrf(int n, double A[729]);
static double xrotg(double *a, double *b, double *s);
static void xzlarf(int m, int n, int iv0, double tau, double C[729], int ic0,
                   double work[27]);
static double xzlarfg(int n, double *alpha1, double x[729], int ix0);

/* Function Definitions */
/*
 * Arguments    : int nvar
 *                double Bk[169]
 *                const double sk[14]
 *                double yk[14]
 *                double workspace[378]
 * Return Type  : bool
 */
static bool BFGSUpdate(int nvar, double Bk[169], const double sk[14], double yk
  [14], double workspace[378])
{
  double curvatureS;
  double dotSY;
  double theta;
  int i;
  int i1;
  int i2;
  int ia;
  int iac;
  int ix;
  int k;
  bool success;
  dotSY = 0.0;
  i = (unsigned char)nvar;
  for (k = 0; k < i; k++) {
    dotSY += sk[k] * yk[k];
    workspace[k] = 0.0;
  }

  ix = 0;
  i1 = 13 * (nvar - 1) + 1;
  for (iac = 1; iac <= i1; iac += 13) {
    i2 = (iac + nvar) - 1;
    for (ia = iac; ia <= i2; ia++) {
      k = ia - iac;
      workspace[k] += Bk[ia - 1] * sk[ix];
    }

    ix++;
  }

  curvatureS = 0.0;
  if (nvar >= 1) {
    for (k = 0; k < i; k++) {
      curvatureS += sk[k] * workspace[k];
    }
  }

  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    for (k = 0; k < i; k++) {
      yk[k] *= theta;
    }

    if (!(1.0 - theta == 0.0)) {
      ix = nvar - 1;
      for (k = 0; k <= ix; k++) {
        yk[k] += (1.0 - theta) * workspace[k];
      }
    }

    dotSY = 0.0;
    for (k = 0; k < i; k++) {
      dotSY += sk[k] * yk[k];
    }
  }

  if ((curvatureS > 2.2204460492503131E-16) && (dotSY > 2.2204460492503131E-16))
  {
    success = true;
  } else {
    success = false;
  }

  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (!(curvatureS == 0.0)) {
      ix = 0;
      for (k = 0; k < i; k++) {
        theta = workspace[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          for (iac = i1; iac <= i2; iac++) {
            Bk[iac - 1] += workspace[(iac - ix) - 1] * theta;
          }
        }

        ix += 13;
      }
    }

    curvatureS = 1.0 / dotSY;
    if (!(curvatureS == 0.0)) {
      ix = 0;
      for (k = 0; k < i; k++) {
        theta = yk[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          for (iac = i1; iac <= i2; iac++) {
            Bk[iac - 1] += yk[(iac - ix) - 1] * theta;
          }
        }

        ix += 13;
      }
    }
  }

  return success;
}

/*
 * Arguments    : h_struct_T *solution
 *                f_struct_T *memspace
 *                i_struct_T *workingset
 *                d_struct_T *qrmanager
 * Return Type  : void
 */
static void PresolveWorkingSet(h_struct_T *solution, f_struct_T *memspace,
  i_struct_T *workingset, d_struct_T *qrmanager)
{
  double tol;
  int idxDiag;
  int idx_col;
  int ix;
  int ix0;
  int k;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  solution->state = 82;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    int i;
    int i1;
    int u0;
    i = (unsigned char)workingset->nVar;
    for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
      for (idx_col = 0; idx_col < i; idx_col++) {
        qrmanager->QR[ix + 27 * idx_col] = workingset->ATwset[idx_col + 14 * ix];
      }
    }

    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }

    memset(&qrmanager->jpvt[0], 0, (unsigned int)i * sizeof(int));
    i1 = mTotalWorkingEq_tmp_tmp * workingset->nVar;
    if (i1 == 0) {
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      idxDiag = workingset->nVar;
      if (mTotalWorkingEq_tmp_tmp <= idxDiag) {
        idxDiag = mTotalWorkingEq_tmp_tmp;
      }

      qrmanager->minRowCol = idxDiag;
      xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp_tmp, workingset->nVar,
             qrmanager->jpvt, qrmanager->tau);
    }

    tol = 100.0 * (double)workingset->nVar * 2.2204460492503131E-16;
    u0 = workingset->nVar;
    if (u0 > mTotalWorkingEq_tmp_tmp) {
      u0 = mTotalWorkingEq_tmp_tmp;
    }

    idxDiag = u0 + 27 * (u0 - 1);
    while ((idxDiag > 0) && (fabs(qrmanager->QR[idxDiag - 1]) < tol)) {
      idxDiag -= 28;
      nDepInd++;
    }

    if (nDepInd > 0) {
      bool exitg1;
      computeQ_(qrmanager, qrmanager->mrows);
      idxDiag = 0;
      exitg1 = false;
      while ((!exitg1) && (idxDiag <= nDepInd - 1)) {
        double qtb;
        ix = 27 * ((mTotalWorkingEq_tmp_tmp - idxDiag) - 1);
        qtb = 0.0;
        for (k = 0; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q[ix + k] * workingset->bwset[k];
        }

        if (fabs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idxDiag++;
        }
      }
    }

    if (nDepInd > 0) {
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idxDiag = 27 * idx_col;
        ix0 = 14 * idx_col;
        for (k = 0; k < i; k++) {
          qrmanager->QR[idxDiag + k] = workingset->ATwset[ix0 + k];
        }
      }

      for (idxDiag = 0; idxDiag < mWorkingFixed; idxDiag++) {
        qrmanager->jpvt[idxDiag] = 1;
      }

      i = workingset->nWConstr[0] + 1;
      if (i <= mTotalWorkingEq_tmp_tmp) {
        memset(&qrmanager->jpvt[i + -1], 0, (unsigned int)
               ((mTotalWorkingEq_tmp_tmp - i) + 1) * sizeof(int));
      }

      if (i1 == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = u0;
        xgeqp3(qrmanager->QR, workingset->nVar, mTotalWorkingEq_tmp_tmp,
               qrmanager->jpvt, qrmanager->tau);
      }

      for (idxDiag = 0; idxDiag < nDepInd; idxDiag++) {
        memspace->workspace_int[idxDiag] = qrmanager->jpvt
          [(mTotalWorkingEq_tmp_tmp - nDepInd) + idxDiag];
      }

      countsort(memspace->workspace_int, nDepInd, memspace->workspace_sort, 1,
                mTotalWorkingEq_tmp_tmp);
      for (idxDiag = nDepInd; idxDiag >= 1; idxDiag--) {
        i = memspace->workspace_int[idxDiag - 1];
        if (i <= mTotalWorkingEq_tmp_tmp) {
          if ((workingset->nActiveConstr == mTotalWorkingEq_tmp_tmp) || (i ==
               mTotalWorkingEq_tmp_tmp)) {
            workingset->mEqRemoved++;

            /* A check that is always false is detected at compile-time. Eliminating code that follows. */
          } else {
            workingset->mEqRemoved++;

            /* A check that is always false is detected at compile-time. Eliminating code that follows. */
          }
        }
      }
    }
  }

  if ((nDepInd != -1) && (workingset->nActiveConstr <= 27)) {
    bool guard1;
    bool okWorkingSet;
    RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
    okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
      solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
      okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
        solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      tol = b_maxConstraintViolation(workingset, solution->xstar);
      if (tol > 1.0E-6) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    idxDiag = mTotalWorkingEq_tmp_tmp + 1;
    ix = workingset->nActiveConstr;
    for (ix0 = idxDiag; ix0 <= ix; ix0++) {
      workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[ix0 -
        1] - 1] + workingset->Wlocalidx[ix0 - 1]) - 2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = mTotalWorkingEq_tmp_tmp;
  }
}

/*
 * Arguments    : i_struct_T *workingset
 *                d_struct_T *qrmanager
 *                f_struct_T *memspace
 *                double tolfactor
 * Return Type  : void
 */
static void RemoveDependentIneq_(i_struct_T *workingset, d_struct_T *qrmanager,
  f_struct_T *memspace, double tolfactor)
{
  int idx;
  int idx_col;
  int k;
  int nActiveConstr_tmp;
  int nFixedConstr;
  int nVar;
  nActiveConstr_tmp = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    double tol;
    int idxDiag;
    int nDepIneq;
    tol = tolfactor * (double)workingset->nVar * 2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt[idx] = 1;
    }

    idxDiag = nFixedConstr + 1;
    if (idxDiag <= nActiveConstr_tmp) {
      memset(&qrmanager->jpvt[idxDiag + -1], 0, (unsigned int)
             ((nActiveConstr_tmp - idxDiag) + 1) * sizeof(int));
    }

    for (idx_col = 0; idx_col < nActiveConstr_tmp; idx_col++) {
      nDepIneq = 27 * idx_col;
      idx = 14 * idx_col;
      idxDiag = (unsigned char)nVar;
      for (k = 0; k < idxDiag; k++) {
        qrmanager->QR[nDepIneq + k] = workingset->ATwset[idx + k];
      }
    }

    if (workingset->nVar * workingset->nActiveConstr == 0) {
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      idxDiag = workingset->nVar;
      nDepIneq = workingset->nActiveConstr;
      if (idxDiag <= nDepIneq) {
        nDepIneq = idxDiag;
      }

      qrmanager->minRowCol = nDepIneq;
      xgeqp3(qrmanager->QR, workingset->nVar, workingset->nActiveConstr,
             qrmanager->jpvt, qrmanager->tau);
    }

    nDepIneq = 0;
    for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar; idx--) {
      nDepIneq++;
      memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
    }

    if (idx + 1 <= workingset->nVar) {
      idxDiag = idx + 27 * idx;
      while ((idx + 1 > nFixedConstr) && (fabs(qrmanager->QR[idxDiag]) < tol)) {
        nDepIneq++;
        memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
        idx--;
        idxDiag -= 28;
      }
    }

    countsort(memspace->workspace_int, nDepIneq, memspace->workspace_sort,
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      removeConstr(workingset, memspace->workspace_int[idx - 1]);
    }
  }
}

/*
 * Arguments    : i_struct_T *obj
 *                int TYPE
 *                int idx_local
 * Return Type  : void
 */
static void addBoundToActiveSetMatrix_(i_struct_T *obj, int TYPE, int idx_local)
{
  int colOffset;
  int i;
  int idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  i = obj->nActiveConstr - 1;
  obj->Wid[i] = TYPE;
  obj->Wlocalidx[i] = idx_local;
  colOffset = 14 * i - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB[idx_local - 1];
    obj->bwset[i] = obj->ub[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB[idx_local - 1];
    obj->bwset[i] = obj->lb[idx_bnd_local - 1];
  }

  if (idx_bnd_local - 2 >= 0) {
    memset(&obj->ATwset[colOffset + 1], 0, (unsigned int)(((idx_bnd_local +
              colOffset) - colOffset) - 1) * sizeof(double));
  }

  obj->ATwset[idx_bnd_local + colOffset] = 2.0 * (double)(TYPE == 5) - 1.0;
  i = idx_bnd_local + 1;
  idx_bnd_local = obj->nVar;
  if (i <= idx_bnd_local) {
    memset(&obj->ATwset[i + colOffset], 0, (unsigned int)((((idx_bnd_local +
               colOffset) - i) - colOffset) + 1) * sizeof(double));
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    obj->ATwset[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/*
 * Arguments    : double workspace[378]
 *                int nVar
 *                const double grad[14]
 *                const int finiteFixed[14]
 *                int mFixed
 *                const int finiteLB[14]
 *                int mLB
 *                const int finiteUB[14]
 *                int mUB
 *                const double lambda[27]
 * Return Type  : void
 */
static void b_computeGradLag(double workspace[378], int nVar, const double grad
  [14], const int finiteFixed[14], int mFixed, const int finiteLB[14], int mLB,
  const int finiteUB[14], int mUB, const double lambda[27])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }

  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }

  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }

  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : const double lb[13]
 *                const double ub[13]
 *                h_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                const i_coder_internal_stickyStruct *FcnEvaluator
 *                f_struct_T *memspace
 *                i_struct_T *WorkingSet
 *                double Hessian[169]
 *                d_struct_T *QRManager
 *                e_struct_T *CholManager
 *                struct_T *QPObjective
 * Return Type  : void
 */
static void b_driver(const double lb[13], const double ub[13], h_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     i_coder_internal_stickyStruct *FcnEvaluator, f_struct_T
                     *memspace, i_struct_T *WorkingSet, double Hessian[169],
                     d_struct_T *QRManager, e_struct_T *CholManager, struct_T
                     *QPObjective)
{
  static const signed char iv[169] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const char qpoptions_SolverName[7] = { 'f', 'm', 'i', 'n', 'c', 'o',
    'n' };

  j_struct_T b_expl_temp;
  j_struct_T expl_temp;
  k_struct_T Flags;
  int i;
  int ineqStart;
  int ixlast;
  int k;
  int mConstr;
  int mFixed;
  int mLB;
  int mUB;
  int nVar_tmp_tmp;
  int qpoptions_MaxIterations;
  memset(&QPObjective->grad[0], 0, 14U * sizeof(double));
  memset(&QPObjective->Hx[0], 0, 13U * sizeof(double));
  QPObjective->hasLinear = true;
  QPObjective->nvar = 13;
  QPObjective->maxVar = 14;
  QPObjective->beta = 0.0;
  QPObjective->rho = 0.0;
  QPObjective->objtype = 3;
  QPObjective->prev_objtype = 3;
  QPObjective->prev_nvar = 0;
  QPObjective->prev_hasLinear = false;
  QPObjective->gammaScalar = 0.0;
  CholManager->ldm = 27;
  CholManager->ndims = 0;
  CholManager->info = 0;
  CholManager->scaleFactor = 0.0;
  CholManager->ConvexCheck = true;
  CholManager->regTol_ = rtInf;
  CholManager->workspace_ = rtInf;
  CholManager->workspace2_ = rtInf;
  QRManager->ldq = 27;
  memset(&CholManager->FMat[0], 0, 729U * sizeof(double));
  memset(&QRManager->QR[0], 0, 729U * sizeof(double));
  memset(&QRManager->Q[0], 0, 729U * sizeof(double));
  QRManager->mrows = 0;
  QRManager->ncols = 0;
  memset(&QRManager->jpvt[0], 0, 27U * sizeof(int));
  memset(&QRManager->tau[0], 0, 27U * sizeof(double));
  QRManager->minRowCol = 0;
  QRManager->usedPivoting = false;
  for (i = 0; i < 169; i++) {
    Hessian[i] = iv[i];
  }

  nVar_tmp_tmp = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = (WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4];
  ineqStart = WorkingSet->nVar;
  ixlast = (WorkingSet->sizes[3] + WorkingSet->sizes[4]) + (WorkingSet->sizes[0]
    << 1);
  if (ineqStart >= ixlast) {
    ixlast = ineqStart;
  }

  qpoptions_MaxIterations = 10 * ixlast;
  TrialState->steplength = 1.0;
  test_exit(MeritFunction, WorkingSet, TrialState, lb, ub, &Flags.fevalOK,
            &Flags.done, &Flags.stepAccepted, &Flags.failedLineSearch,
            &Flags.stepType);
  TrialState->sqpFval_old = TrialState->sqpFval;
  for (k = 0; k < 13; k++) {
    TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
    TrialState->grad_old[k] = TrialState->grad[k];
  }

  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  while (!Flags.done) {
    double phi_alpha;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        i = (unsigned char)mLB;
        for (ixlast = 0; ixlast < i; ixlast++) {
          WorkingSet->lb[WorkingSet->indexLB[ixlast] - 1] = -lb
            [WorkingSet->indexLB[ixlast] - 1] + TrialState->xstarsqp
            [WorkingSet->indexLB[ixlast] - 1];
        }

        i = (unsigned char)mUB;
        for (ixlast = 0; ixlast < i; ixlast++) {
          WorkingSet->ub[WorkingSet->indexUB[ixlast] - 1] = ub
            [WorkingSet->indexUB[ixlast] - 1] - TrialState->xstarsqp
            [WorkingSet->indexUB[ixlast] - 1];
        }

        i = (unsigned char)mFixed;
        for (ixlast = 0; ixlast < i; ixlast++) {
          phi_alpha = ub[WorkingSet->indexFixed[ixlast] - 1] -
            TrialState->xstarsqp[WorkingSet->indexFixed[ixlast] - 1];
          WorkingSet->ub[WorkingSet->indexFixed[ixlast] - 1] = phi_alpha;
          WorkingSet->bwset[ixlast] = phi_alpha;
        }

        if (WorkingSet->nActiveConstr > mFixed) {
          ineqStart = mFixed + 1;
          if (ineqStart < 1) {
            ineqStart = 1;
          }

          i = WorkingSet->nActiveConstr;
          for (ixlast = ineqStart; ixlast <= i; ixlast++) {
            switch (WorkingSet->Wid[ixlast - 1]) {
             case 4:
              WorkingSet->bwset[ixlast - 1] = WorkingSet->lb[WorkingSet->
                indexLB[WorkingSet->Wlocalidx[ixlast - 1] - 1] - 1];
              break;

             case 5:
              WorkingSet->bwset[ixlast - 1] = WorkingSet->ub[WorkingSet->
                indexUB[WorkingSet->Wlocalidx[ixlast - 1] - 1] - 1];
              break;

             default:
              /* A check that is always false is detected at compile-time. Eliminating code that follows. */
              break;
            }
          }
        }
      }

      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (i = 0; i < 7; i++) {
        expl_temp.SolverName[i] = qpoptions_SolverName[i];
      }

      b_expl_temp = expl_temp;
      Flags.stepAccepted = step(&Flags.stepType, Hessian, lb, ub, TrialState,
        MeritFunction, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        &b_expl_temp);
      if (Flags.stepAccepted) {
        i = (unsigned char)nVar_tmp_tmp;
        for (ineqStart = 0; ineqStart < i; ineqStart++) {
          TrialState->xstarsqp[ineqStart] += TrialState->delta_x[ineqStart];
        }

        TrialState->sqpFval = evalObjAndConstr
          (&FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace,
           TrialState->xstarsqp, &ineqStart);
        Flags.fevalOK = (ineqStart == 1);
        TrialState->FunctionEvaluations++;
        if (Flags.fevalOK) {
          MeritFunction->phiFullStep = TrialState->sqpFval;
        } else {
          MeritFunction->phiFullStep = rtInf;
        }
      }

      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction->phi < MeritFunction->phiFullStep) &&
          (TrialState->sqpFval < TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        double alpha;
        int exitflagLnSrch;
        bool evalWellDefined;
        bool socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }

        evalWellDefined = Flags.fevalOK;
        i = WorkingSet->nVar;
        alpha = 1.0;
        exitflagLnSrch = 1;
        phi_alpha = MeritFunction->phiFullStep;
        ineqStart = (unsigned char)WorkingSet->nVar;
        if (ineqStart - 1 >= 0) {
          memcpy(&TrialState->searchDir[0], &TrialState->delta_x[0], (unsigned
                  int)ineqStart * sizeof(double));
        }

        int exitg1;
        do {
          exitg1 = 0;
          if (TrialState->FunctionEvaluations < max_function_eval_inner_loop && toc() < max_time_inner_loop) {
            if (evalWellDefined && (phi_alpha <= MeritFunction->phi + alpha *
                                    0.0001 * MeritFunction->phiPrimePlus)) {
              exitg1 = 1;
            } else {
              bool exitg2;
              bool tooSmallX;
              alpha *= 0.7;
              ineqStart = (unsigned char)i;
              for (ixlast = 0; ixlast < ineqStart; ixlast++) {
                TrialState->delta_x[ixlast] = alpha * TrialState->xstar[ixlast];
              }

              if (socTaken) {
                phi_alpha = alpha * alpha;
                if ((i >= 1) && (!(phi_alpha == 0.0))) {
                  ixlast = i - 1;
                  for (k = 0; k <= ixlast; k++) {
                    TrialState->delta_x[k] += phi_alpha *
                      TrialState->socDirection[k];
                  }
                }
              }

              tooSmallX = true;
              ixlast = 0;
              exitg2 = false;
              while ((!exitg2) && (ixlast <= (unsigned char)i - 1)) {
                if (1.0E-12 * fmax(1.0, fabs(TrialState->xstarsqp[ixlast])) <=
                    fabs(TrialState->delta_x[ixlast])) {
                  tooSmallX = false;
                  exitg2 = true;
                } else {
                  ixlast++;
                }
              }

              if (tooSmallX) {
                exitflagLnSrch = -2;
                exitg1 = 1;
              } else {
                for (ixlast = 0; ixlast < ineqStart; ixlast++) {
                  TrialState->xstarsqp[ixlast] = TrialState->xstarsqp_old[ixlast]
                    + TrialState->delta_x[ixlast];
                }

                TrialState->sqpFval = evalObjAndConstr
                  (&FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace,
                   TrialState->xstarsqp, &ixlast);
                TrialState->FunctionEvaluations++;
                evalWellDefined = (ixlast == 1);
                if (evalWellDefined) {
                  phi_alpha = TrialState->sqpFval;
                } else {
                  phi_alpha = rtInf;
                }
              }
            }
          } else {
            exitflagLnSrch = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);

        Flags.fevalOK = evalWellDefined;
        TrialState->steplength = alpha;
        if (exitflagLnSrch > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      i = (unsigned char)nVar_tmp_tmp;
      for (ixlast = 0; ixlast < i; ixlast++) {
        TrialState->xstarsqp[ixlast] = TrialState->xstarsqp_old[ixlast] +
          TrialState->delta_x[ixlast];
      }

      i = (unsigned char)mConstr;
      for (ixlast = 0; ixlast < i; ixlast++) {
        phi_alpha = TrialState->lambdasqp[ixlast];
        phi_alpha += TrialState->steplength * (TrialState->lambda[ixlast] -
          phi_alpha);
        TrialState->lambdasqp[ixlast] = phi_alpha;
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      for (k = 0; k < 13; k++) {
        TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
        TrialState->grad_old[k] = TrialState->grad[k];
      }

      TrialState->sqpFval = c_computeObjectiveAndUserGradie
        (&FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace,
         TrialState->xstarsqp, TrialState->grad, &ineqStart);
      if (ineqStart == 1) {
        ineqStart = 1;
      }

      TrialState->FunctionEvaluations++;
      Flags.fevalOK = (ineqStart == 1);
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0], 13U *
             sizeof(double));
    }

    b_test_exit(&Flags, memspace, MeritFunction, WorkingSet, TrialState,
                QRManager, lb, ub);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      i = (unsigned char)nVar_tmp_tmp;
      memcpy(&TrialState->delta_gradLag[0], &TrialState->grad[0], (unsigned int)
             i * sizeof(double));
      if (nVar_tmp_tmp >= 1) {
        ixlast = nVar_tmp_tmp - 1;
        for (k = 0; k <= ixlast; k++) {
          TrialState->delta_gradLag[k] -= TrialState->grad_old[k];
        }
      }

      BFGSUpdate(nVar_tmp_tmp, Hessian, TrialState->delta_x,
                 TrialState->delta_gradLag, memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }
}

/*
 * Arguments    : const i_struct_T *obj
 *                const double x[14]
 * Return Type  : double
 */
static double b_maxConstraintViolation(const i_struct_T *obj, const double x[14])
{
  double v;
  int i;
  int idx;
  int idxLB;
  v = 0.0;
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB[idx] - 1;
      v = fmax(v, -x[idxLB] - obj->lb[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    i = (unsigned char)obj->sizes[4];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexUB[idx] - 1;
      v = fmax(v, x[idxLB] - obj->ub[idxLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    i = (unsigned char)obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      v = fmax(v, fabs(x[obj->indexFixed[idx] - 1] - obj->ub[obj->indexFixed[idx]
                       - 1]));
    }
  }

  return v;
}

/*
 * Arguments    : const double x[6]
 * Return Type  : double
 */
static double b_norm(const double x[6])
{
  double scale;
  double y;
  int k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 6; k++) {
    double absxk;
    absxk = fabs(x[k]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/*
 * Arguments    : k_struct_T *Flags
 *                f_struct_T *memspace
 *                b_struct_T *MeritFunction
 *                const i_struct_T *WorkingSet
 *                h_struct_T *TrialState
 *                d_struct_T *QRManager
 *                const double lb[13]
 *                const double ub[13]
 * Return Type  : void
 */
static void b_test_exit(k_struct_T *Flags, f_struct_T *memspace, b_struct_T
  *MeritFunction, const i_struct_T *WorkingSet, h_struct_T *TrialState,
  d_struct_T *QRManager, const double lb[13], const double ub[13])
{
  double optimRelativeFactor;
  double s;
  double smax;
  int b_i;
  int b_k;
  int i;
  int i1;
  int idx_max;
  int k;
  int mFixed;
  int nVar;
  bool dxTooSmall;
  bool exitg1;
  bool isFeasible;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  i = (unsigned char)((WorkingSet->sizes[0] + WorkingSet->sizes[3]) +
                      WorkingSet->sizes[4]);
  if (i - 1 >= 0) {
    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambdasqp[0], (unsigned
            int)i * sizeof(double));
  }

  computeGradLag(TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdaStopTest);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  optimRelativeFactor = fmax(1.0, fabs(TrialState->grad[idx_max - 1]));
  if (rtIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }

  smax = 0.0;
  i1 = (unsigned char)WorkingSet->sizes[3];
  for (idx_max = 0; idx_max < i1; idx_max++) {
    nVar = WorkingSet->indexLB[idx_max] - 1;
    smax = fmax(smax, lb[nVar] - TrialState->xstarsqp[nVar]);
  }

  i1 = (unsigned char)WorkingSet->sizes[4];
  for (idx_max = 0; idx_max < i1; idx_max++) {
    nVar = WorkingSet->indexUB[idx_max] - 1;
    smax = fmax(smax, TrialState->xstarsqp[nVar] - ub[nVar]);
  }

  MeritFunction->nlpPrimalFeasError = smax;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  }

  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  dxTooSmall = true;
  smax = 0.0;
  i1 = (unsigned char)WorkingSet->nVar;
  idx_max = 0;
  exitg1 = false;
  while ((!exitg1) && (idx_max <= i1 - 1)) {
    dxTooSmall = ((!rtIsInf(TrialState->gradLag[idx_max])) && (!rtIsNaN
      (TrialState->gradLag[idx_max])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = fmax(smax, fabs(TrialState->gradLag[idx_max]));
      idx_max++;
    }
  }

  MeritFunction->nlpDualFeasError = smax;
  if (!dxTooSmall) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = computeComplError(TrialState->xstarsqp,
      WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
      WorkingSet->sizes[4], ub, TrialState->lambdaStopTest, WorkingSet->sizes[0]
      + 1);
    smax = fmax(smax, MeritFunction->nlpComplError);
    MeritFunction->firstOrderOpt = smax;
    if (TrialState->sqpIterations > 1) {
      double d;
      double nlpComplErrorTmp;
      b_computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                       TrialState->grad, WorkingSet->indexFixed,
                       WorkingSet->sizes[0], WorkingSet->indexLB,
                       WorkingSet->sizes[3], WorkingSet->indexUB,
                       WorkingSet->sizes[4], TrialState->lambdaStopTestPrev);
      s = 0.0;
      idx_max = 0;
      while ((idx_max <= i1 - 1) && ((!rtIsInf(memspace->
                workspace_double[idx_max])) && (!rtIsNaN
               (memspace->workspace_double[idx_max])))) {
        s = fmax(s, fabs(memspace->workspace_double[idx_max]));
        idx_max++;
      }

      nlpComplErrorTmp = computeComplError(TrialState->xstarsqp,
        WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
        WorkingSet->sizes[4], ub, TrialState->lambdaStopTestPrev,
        WorkingSet->sizes[0] + 1);
      d = fmax(s, nlpComplErrorTmp);
      if (d < smax) {
        MeritFunction->nlpDualFeasError = s;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = d;
        if (i - 1 >= 0) {
          memcpy(&TrialState->lambdaStopTest[0], &TrialState->
                 lambdaStopTestPrev[0], (unsigned int)i * sizeof(double));
        }
      } else if (i - 1 >= 0) {
        memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
               (unsigned int)i * sizeof(double));
      }
    } else if (i - 1 >= 0) {
      memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
             (unsigned int)i * sizeof(double));
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-12 *
                       optimRelativeFactor) && (MeritFunction->nlpComplError <=
         1.0E-12 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        bool guard1;
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          idx_max = 0;
          exitg1 = false;
          while ((!exitg1) && (idx_max <= i1 - 1)) {
            if (1.0E-12 * fmax(1.0, fabs(TrialState->xstarsqp[idx_max])) <= fabs
                (TrialState->delta_x[idx_max])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx_max++;
            }
          }

          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType != 2) {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              }
            } else {
              nVar = WorkingSet->nActiveConstr - 1;
              if (WorkingSet->nActiveConstr > 0) {
                int rankR;
                for (k = 0; k <= nVar; k++) {
                  TrialState->lambda[k] = 0.0;
                  idx_max = 14 * k;
                  rankR = 27 * k;
                  for (b_k = 0; b_k < i1; b_k++) {
                    QRManager->QR[rankR + b_k] = WorkingSet->ATwset[idx_max +
                      b_k];
                  }
                }

                QRManager->usedPivoting = true;
                QRManager->mrows = WorkingSet->nVar;
                QRManager->ncols = WorkingSet->nActiveConstr;
                nVar = WorkingSet->nVar;
                b_k = WorkingSet->nActiveConstr;
                if (nVar <= b_k) {
                  b_k = nVar;
                }

                QRManager->minRowCol = b_k;
                xgeqp3(QRManager->QR, WorkingSet->nVar,
                       WorkingSet->nActiveConstr, QRManager->jpvt,
                       QRManager->tau);
                computeQ_(QRManager, WorkingSet->nVar);
                nVar = WorkingSet->nVar;
                idx_max = WorkingSet->nActiveConstr;
                if (nVar >= idx_max) {
                  idx_max = nVar;
                }

                smax = fabs(QRManager->QR[0]) * fmin(1.4901161193847656E-8,
                  (double)idx_max * 2.2204460492503131E-16);
                rankR = 0;
                nVar = 0;
                while ((rankR < b_k) && (fabs(QRManager->QR[nVar]) > smax)) {
                  rankR++;
                  nVar += 28;
                }

                b_xgemv(WorkingSet->nVar, WorkingSet->nVar, QRManager->Q,
                        TrialState->grad, memspace->workspace_double);
                if (rankR != 0) {
                  for (k = rankR; k >= 1; k--) {
                    nVar = (k + (k - 1) * 27) - 1;
                    memspace->workspace_double[k - 1] /= QRManager->QR[nVar];
                    for (b_i = 0; b_i <= k - 2; b_i++) {
                      idx_max = (k - b_i) - 2;
                      memspace->workspace_double[idx_max] -=
                        memspace->workspace_double[k - 1] * QRManager->QR[(nVar
                        - b_i) - 1];
                    }
                  }
                }

                nVar = WorkingSet->nActiveConstr;
                if (nVar <= b_k) {
                  b_k = nVar;
                }

                nVar = (unsigned char)b_k;
                for (idx_max = 0; idx_max < nVar; idx_max++) {
                  TrialState->lambda[QRManager->jpvt[idx_max] - 1] =
                    memspace->workspace_double[idx_max];
                }

                nVar = WorkingSet->sizes[0] + 1;
                for (idx_max = nVar; idx_max <= mFixed; idx_max++) {
                  TrialState->lambda[idx_max - 1] = -TrialState->lambda[idx_max
                    - 1];
                }

                sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                             WorkingSet->sizes, WorkingSet->isActiveIdx,
                             WorkingSet->Wid, WorkingSet->Wlocalidx,
                             memspace->workspace_double);
                b_computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                                 TrialState->grad, WorkingSet->indexFixed,
                                 WorkingSet->sizes[0], WorkingSet->indexLB,
                                 WorkingSet->sizes[3], WorkingSet->indexUB,
                                 WorkingSet->sizes[4], TrialState->lambda);
                smax = 0.0;
                idx_max = 0;
                while ((idx_max <= i1 - 1) && ((!rtIsInf
                         (memspace->workspace_double[idx_max])) && (!rtIsNaN
                         (memspace->workspace_double[idx_max])))) {
                  smax = fmax(smax, fabs(memspace->workspace_double[idx_max]));
                  idx_max++;
                }

                s = computeComplError(TrialState->xstarsqp, WorkingSet->indexLB,
                                      WorkingSet->sizes[3], lb,
                                      WorkingSet->indexUB, WorkingSet->sizes[4],
                                      ub, TrialState->lambda, WorkingSet->sizes
                                      [0] + 1);
                if ((smax <= 1.0E-12 * optimRelativeFactor) && (s <= 1.0E-12 *
                     optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = smax;
                  MeritFunction->nlpComplError = s;
                  MeritFunction->firstOrderOpt = fmax(smax, s);
                  if (i - 1 >= 0) {
                    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0],
                           (unsigned int)i * sizeof(double));
                  }

                  Flags->done = true;
                  TrialState->sqpExitFlag = 1;
                } else {
                  Flags->done = true;
                  TrialState->sqpExitFlag = 2;
                }
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if (TrialState->sqpIterations >= max_iterations_inner_loop || toc() >= max_time_inner_loop) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= max_function_eval_inner_loop || toc() >= max_time_inner_loop) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

/*
 * Arguments    : double *outTime_tv_nsec
 * Return Type  : double
 */
static double b_timeKeeper(double *outTime_tv_nsec)
{
  double outTime_tv_sec;
  outTime_tv_sec = savedTime.tv_sec;
  *outTime_tv_nsec = savedTime.tv_nsec;
  return outTime_tv_sec;
}

/*
 * Arguments    : int m
 *                int n
 *                int k
 *                const double A[729]
 *                int ia0
 *                const double B[378]
 *                double C[729]
 * Return Type  : void
 */
static void b_xgemm(int m, int n, int k, const double A[729], int ia0, const
                    double B[378], double C[729])
{
  int cr;
  int ic;
  int w;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    lastColC = 27 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 27) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        memset(&C[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof(double));
      }
    }

    br = -1;
    for (cr = 0; cr <= lastColC; cr += 27) {
      int ar;
      ar = ia0;
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        double temp;
        temp = 0.0;
        for (w = 0; w < k; w++) {
          temp += A[(w + ar) - 1] * B[(w + br) + 1];
        }

        C[ic - 1] += temp;
        ar += 27;
      }

      br += 27;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[729]
 *                const double x[14]
 *                double y[378]
 * Return Type  : void
 */
static void b_xgemv(int m, int n, const double A[729], const double x[14],
                    double y[378])
{
  int ia;
  int iac;
  if (m != 0) {
    int i;
    memset(&y[0], 0, (unsigned int)n * sizeof(double));
    i = 27 * (n - 1) + 1;
    for (iac = 1; iac <= i; iac += 27) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }

      i1 = div_nde_s32_floor(iac - 1, 27);
      y[i1] += c;
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[14]
 * Return Type  : double
 */
static double b_xnrm2(int n, const double x[14])
{
  double y;
  int k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[0]);
    } else {
      double scale;
      int i;
      scale = 3.3121686421112381E-170;
      i = (unsigned char)n;
      for (k = 0; k < i; k++) {
        double absxk;
        absxk = fabs(x[k]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void c_CoderTimeAPI_callCoderClockGe(void)
{
  freq_not_empty = false;
}

/*
 * Arguments    : const c_struct_T *c_obj_next_next_next_next_next_
 *                const double x[13]
 *                double grad_workspace[14]
 *                int *status
 * Return Type  : double
 */
static double c_computeObjectiveAndUserGradie(const c_struct_T
  *c_obj_next_next_next_next_next_, const double x[13], double grad_workspace[14],
  int *status)
{
  double gradient_data[13];
  double fval;
  int idx_current;
  bool allFinite;
  fval = c_compute_cost_and_gradient_sec(x,
    c_obj_next_next_next_next_next_->dv_global->contents,
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
    c_obj_next_next_next_next_next_->Phi->contents,
    c_obj_next_next_next_next_next_->S->contents,
    c_obj_next_next_next_next_next_->Theta->contents,
    c_obj_next_next_next_next_next_->V->contents,
    c_obj_next_next_next_next_next_->W_act_motor->contents,
    c_obj_next_next_next_next_next_->W_dv_1->contents,
    c_obj_next_next_next_next_next_->W_dv_2->contents,
    c_obj_next_next_next_next_next_->W_dv_3->contents,
    c_obj_next_next_next_next_next_->W_dv_4->contents,
    c_obj_next_next_next_next_next_->W_dv_5->contents,
    c_obj_next_next_next_next_next_->W_dv_6->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_el->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_az->contents,
    c_obj_next_next_next_next_next_->W_act_ailerons->contents,
    c_obj_next_next_next_next_next_->desired_el_value->contents,
    c_obj_next_next_next_next_next_->desired_az_value->contents,
    c_obj_next_next_next_next_next_->desired_motor_value->contents,
    c_obj_next_next_next_next_next_->desired_ailerons_value->contents,
    c_obj_next_next_next_next_next_->flight_path_angle->contents,
    c_obj_next_next_next_next_next_->gain_el->contents,
    c_obj_next_next_next_next_next_->gain_az->contents,
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
    c_obj_next_next_next_next_next_->q->contents,
    c_obj_next_next_next_next_next_->r->contents,
    c_obj_next_next_next_next_next_->rho->contents,
    c_obj_next_next_next_next_next_->wing_chord->contents,
    c_obj_next_next_next_next_next_->V_scaled->contents,
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
    c_obj_next_next_next_next_next_->wing_span->contents, gradient_data,
    &idx_current);
  memcpy(&grad_workspace[0], &gradient_data[0], 13U * sizeof(double));
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
    allFinite = true;
    idx_current = 0;
    while (allFinite && (idx_current + 1 <= 13)) {
      allFinite = ((!rtIsInf(grad_workspace[idx_current])) && (!rtIsNaN
        (grad_workspace[idx_current])));
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

  return fval;
}

/*
 * Arguments    : const double u_in[15]
 *                double p
 *                double q
 *                double r
 *                double m
 *                double I_xx
 *                double I_yy
 *                double I_zz
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
 *                double Cm_zero
 *                double CL_aileron
 *                double rho
 *                double V
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double Beta
 *                double wing_span
 *                double prop_R
 *                double prop_Cd_0
 *                double prop_Cl_0
 *                double prop_Cd_a
 *                double prop_Cl_a
 *                double prop_delta
 *                double prop_sigma
 *                double prop_theta
 *                double V_scaled
 *                double Omega_1_scaled
 *                double Omega_2_scaled
 *                double accelerations_array[6]
 * Return Type  : void
 */
static void c_compute_acc_nonlinear_CA_new_(const double u_in[15], double p,
  double q, double r, double m, double I_xx, double I_yy, double I_zz, double
  l_1, double l_2, double l_3, double l_4, double l_z, double Cl_alpha, double
  Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero, double CL_aileron,
  double rho, double V, double S, double wing_chord, double flight_path_angle,
  double Beta, double wing_span, double prop_R, double prop_Cd_0, double
  prop_Cl_0, double prop_Cd_a, double prop_Cl_a, double prop_delta, double
  prop_sigma, double prop_theta, double V_scaled, double Omega_1_scaled, double
  Omega_2_scaled, double accelerations_array[6])
{
  double t10;
  double t11;
  double t117_tmp;
  double t117_tmp_tmp;
  double t12;
  double t13;
  double t14;
  double t143;
  double t15;
  double t16;
  double t169;
  double t17;
  double t170;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t222;
  double t23;
  double t237;
  double t238;
  double t239;
  double t24;
  double t26;
  double t27;
  double t274;
  double t28;
  double t29;
  double t3;
  double t30;
  double t31;
  double t34;
  double t35;
  double t36;
  double t37;
  double t38;
  double t4;
  double t40;
  double t41;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t49;
  double t5;
  double t52;
  double t54;
  double t55;
  double t56;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t62;
  double t66;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t71;
  double t73;
  double t74;
  double t75;
  double t76;
  double t77;
  double t78;
  double t79;
  double t8;
  double t80;
  double t85;
  double t86;
  double t87;
  double t88;
  double t89;
  double t9;
  double t91_tmp;

  /* COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_AILERONS_V2_NEW_AERO */
  /*     ACCELERATIONS_ARRAY = COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_AILERONS_V2_NEW_AERO(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Omega_1,Omega_2,Omega_3,Omega_4,Omega_1_scaled,Omega_2_scaled,Phi,S,Theta,V,V_scaled,B_1,B_2,B_3,B_4,DELTA_AILERONS,FLIGHT_PATH_ANGLE,G_1,G_2,G_3,G_4,L_1,L_2,L_3,L_4,L_Z,M,P,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,Q,R,RHO,WING_SPAN,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     11-Oct-2024 01:04:12 */
  t3 = cos(u_in[13]);
  t4 = sin(Beta);
  t5 = cos(u_in[12]);
  t6 = sin(u_in[13]);
  t7 = cos(u_in[4]);
  t8 = cos(u_in[5]);
  t9 = cos(u_in[6]);
  t10 = cos(u_in[7]);
  t11 = sin(u_in[12]);
  t12 = cos(u_in[8]);
  t13 = cos(u_in[9]);
  t14 = cos(u_in[10]);
  t15 = cos(u_in[11]);
  t16 = sin(u_in[4]);
  t17 = sin(u_in[5]);
  t18 = sin(u_in[6]);
  t19 = sin(u_in[7]);
  t20 = sin(u_in[8]);
  t21 = sin(u_in[9]);
  t22 = sin(u_in[10]);
  t23 = sin(u_in[11]);
  t24 = log(prop_delta);
  t26 = u_in[0] * u_in[0];
  t27 = u_in[1] * u_in[1];
  t28 = u_in[2] * u_in[2];
  t29 = u_in[3] * u_in[3];
  t30 = V * V;
  t31 = V_scaled * V_scaled;
  t34 = rt_powd_snf(prop_R, 3.0);
  t35 = rt_powd_snf(prop_R, 4.0);
  t38 = prop_Cd_0 * prop_delta * 2.0;
  t40 = 1.0 / u_in[0];
  t42 = 1.0 / u_in[1];
  t44 = 1.0 / u_in[2];
  t46 = 1.0 / u_in[3];
  t49 = 1.0 / m;
  t52 = prop_delta * 16.0;
  t54 = 1.0 / prop_R;
  t56 = 1.0 / prop_delta;
  t36 = t9 * t9;
  t37 = t10 * t10;
  t41 = 1.0 / t26;
  t43 = 1.0 / t27;
  t45 = 1.0 / t28;
  t47 = 1.0 / t29;
  t55 = t54 * t54;
  t222 = prop_Cl_0 * prop_delta;
  t58 = t222 * (prop_delta + 1.0);
  t59 = u_in[12] - flight_path_angle;
  t66 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t67 = prop_Cl_a * prop_sigma * (prop_delta - 1.0) / 8.0;
  t60 = cos(t59);
  t61 = sin(t59);
  t62 = t58 * 8.0;
  t68 = (u_in[4] + 1.5707963267948966) + t59;
  t69 = (u_in[5] + 1.5707963267948966) + t59;
  t70 = (u_in[6] + 1.5707963267948966) + t59;
  t71 = (u_in[7] + 1.5707963267948966) + t59;
  t73 = cos(t68);
  t74 = cos(t69);
  t75 = cos(t70);
  t76 = cos(t71);
  t77 = sin(t68);
  t78 = sin(t69);
  t79 = sin(t70);
  t80 = sin(t71);
  t91_tmp = Cl_alpha * S * rho * t30 * t59;
  t85 = t77 * t77;
  t86 = t78 * t78;
  t87 = t79 * t79;
  t88 = t80 * t80;
  t89 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t59 * t59);
  t117_tmp_tmp = S * rho;
  t117_tmp = t117_tmp_tmp * cos(Beta) * t30;
  t143 = t91_tmp * t60 / 2.0 + t117_tmp * t61 * t89 / 2.0;
  t274 = t30 * t41 * t55;
  t68 = V * prop_Cl_a * prop_sigma;
  t69 = prop_Cl_0 * prop_sigma * t24 * t30;
  t70 = prop_sigma * (prop_delta - 1.0) * t56;
  t169 = (t67 + V * t40 * t54 * t73 / 2.0) + sqrt(((t274 * (t73 * t73) * 16.0 +
    t68 * t40 * (prop_delta - 1.0) * t54 * t73 * 8.0) + t69 * t41 * t55 * t85 *
    -8.0) - t70 * (t62 + prop_Cl_a * (t66 + prop_theta * (t52 + t274 * t85 * 8.0))))
    / 8.0;
  t274 = t30 * t43 * t55;
  t170 = (t67 + V * t42 * t54 * t74 / 2.0) + sqrt(((t274 * (t74 * t74) * 16.0 +
    t68 * t42 * (prop_delta - 1.0) * t54 * t74 * 8.0) + t69 * t43 * t55 * t86 *
    -8.0) - t70 * (t62 + prop_Cl_a * (t66 + prop_theta * (t52 + t274 * t86 * 8.0))))
    / 8.0;
  t274 = t30 * t45 * t55;
  t71 = (t67 + V * t44 * t54 * t75 / 2.0) + sqrt(((t274 * (t75 * t75) * 16.0 +
    t68 * t44 * (prop_delta - 1.0) * t54 * t75 * 8.0) + t69 * t45 * t55 * t87 *
    -8.0) - t70 * (t62 + prop_Cl_a * (t66 + prop_theta * (t52 + t274 * t87 * 8.0))))
    / 8.0;
  t274 = t30 * t47 * t55;
  t75 = (t67 + V * t46 * t54 * t76 / 2.0) + sqrt(((t274 * (t76 * t76) * 16.0 +
    t68 * t46 * (prop_delta - 1.0) * t54 * t76 * 8.0) + t69 * t47 * t55 * t88 *
    -8.0) - t70 * (t62 + prop_Cl_a * (t66 + prop_theta * (t52 + t274 * t88 * 8.0))))
    / 8.0;
  t74 = t222 * t24;
  t274 = prop_Cl_a - prop_Cd_a * 2.0;
  t40 = prop_Cd_a * prop_theta * 2.0;
  t237 = t74 * t169 + (prop_delta - 1.0) * (t38 + prop_theta * (t169 * t274 +
    t40));
  t238 = t74 * t170 + (prop_delta - 1.0) * (t38 + prop_theta * (t170 * t274 +
    t40));
  t239 = t74 * t71 + (prop_delta - 1.0) * (t38 + prop_theta * (t71 * t274 + t40));
  t38 = t74 * t75 + (prop_delta - 1.0) * (t38 + prop_theta * (t75 * t274 + t40));
  t274 = t74 * t30;
  t40 = prop_Cl_a * prop_theta * t30;
  t73 = prop_Cl_a * prop_delta;
  t42 = t274 * t41 * t55 * t85 + (prop_delta - 1.0) * ((t58 + t40 * t41 * t55 *
    t85) - t73 * (-prop_theta + t169) * 2.0);
  t222 = t274 * t43 * t55 * t86 + (prop_delta - 1.0) * ((t58 + t40 * t43 * t55 *
    t86) - t73 * (-prop_theta + t170) * 2.0);
  t66 = t274 * t45 * t55 * t87 + (prop_delta - 1.0) * ((t58 + t40 * t45 * t55 *
    t87) - t73 * (-prop_theta + t71) * 2.0);
  t71 = t274 * t47 * t55 * t88 + (prop_delta - 1.0) * ((t58 + t40 * t47 * t55 *
    t88) - t73 * (-prop_theta + t75) * 2.0);
  t274 = prop_sigma * rho;
  t73 = u_in[0] * V * prop_sigma * rho;
  t67 = t274 * t16 * t26 * t35 * t56 * t42 * 3.1415926535897931 / 4.0 + t73 * t7
    * t34 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t69 = u_in[1] * V * prop_sigma * rho;
  t62 = t274 * t17 * t27 * t35 * t56 * t222 * 3.1415926535897931 / 4.0 + t69 *
    t8 * t34 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t70 = u_in[2] * V * prop_sigma * rho;
  t76 = t274 * t18 * t28 * t35 * t56 * t66 * 3.1415926535897931 / 4.0 + t70 * t9
    * t34 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t74 = u_in[3] * V * prop_sigma * rho;
  t24 = t274 * t19 * t29 * t35 * t56 * t71 * 3.1415926535897931 / 4.0 + t74 *
    t10 * t34 * t56 * t80 * t38 * 3.1415926535897931 / 4.0;
  t75 = t274 * t7;
  t54 = t75 * t12 * t26 * t35 * t56 * t42 * 3.1415926535897931 / 4.0 - t73 * t12
    * t16 * t34 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t68 = t274 * t8;
  t52 = t68 * t13 * t27 * t35 * t56 * t222 * 3.1415926535897931 / 4.0 - t69 *
    t13 * t17 * t34 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t40 = t274 * t9;
  t46 = t40 * t14 * t28 * t35 * t56 * t66 * 3.1415926535897931 / 4.0 - t70 * t14
    * t18 * t34 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t274 *= t10;
  t44 = t274 * t15 * t29 * t35 * t56 * t71 * 3.1415926535897931 / 4.0 - t74 *
    t15 * t19 * t34 * t56 * t80 * t38 * 3.1415926535897931 / 4.0;
  t42 = t75 * t20 * t26 * t35 * t56 * t42 * 3.1415926535897931 / 4.0 - t73 * t16
    * t20 * t34 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t75 = t68 * t21 * t27 * t35 * t56 * t222 * 3.1415926535897931 / 4.0 - t69 *
    t17 * t21 * t34 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t73 = t40 * t22 * t28 * t35 * t56 * t66 * 3.1415926535897931 / 4.0 - t70 * t18
    * t22 * t34 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t70 = t274 * t23 * t29 * t35 * t56 * t71 * 3.1415926535897931 / 4.0 - t74 *
    t19 * t23 * t34 * t56 * t80 * t38 * 3.1415926535897931 / 4.0;
  t71 = ((t67 + t62) + t76) + t24;
  t274 = ((t42 + t75) + t73) + t70;
  t40 = ((t54 + t52) + t46) + t44;
  t74 = t3 * t11;
  t68 = t91_tmp * t61 / 2.0 - t117_tmp * t60 * t89 / 2.0;
  t69 = t117_tmp_tmp * t4;
  accelerations_array[0] = t49 * (((((t5 * t71 + t5 * t68) - t74 * t143) + t74 *
    t40) - t6 * t11 * t274) - t69 * t6 * t11 * t30 * t89 / 2.0);
  accelerations_array[1] = -t49 * (((-t6 * t143 + t3 * t274) + t6 * t40) +
    t117_tmp_tmp * t3 * t4 * t30 * t89 / 2.0);
  t74 = t3 * t5;
  accelerations_array[2] = -t49 * (((((t11 * t71 + t11 * t68) + t74 * t143) -
    t74 * t40) + t5 * t6 * t274) + t69 * t5 * t6 * t30 * t89 / 2.0) + 9.81;
  t74 = Omega_1_scaled * t7;
  t68 = Omega_2_scaled * t8;
  t69 = t117_tmp_tmp * t30;
  accelerations_array[3] = -(((((((((((l_1 * t54 - l_1 * t52) - l_2 * t46) + l_2
    * t44) + l_z * t42) + l_z * t75) + l_z * t73) + l_z * t70) - I_yy * q * r) +
    I_zz * q * r) - CL_aileron * S * u_in[14] * rho * t30 / 2.0) + t69 *
    wing_span * (((((t74 * 0.01228861457458129 - t68 * 0.01228861457458129) +
                    Omega_1_scaled * Omega_1_scaled * t7 * 0.0075450647873164947)
                   - Omega_2_scaled * Omega_2_scaled * t8 *
                   0.0075450647873164947) + ((((t74 * t31 * -0.02051939481844044
    + t68 * t31 * 0.02051939481844044) + Omega_1_scaled * t16 * t37 *
    0.0046438440894603552) - Omega_2_scaled * t17 * t36 * 0.0046438440894603552)
    + Omega_2_scaled * t18 * t36 * 0.0064371741699132748)) + ((Omega_1_scaled *
    t19 * t37 * -0.0064371741699132748 - Omega_1_scaled * (t7 * t7) * t19 *
    0.0039345373862287912) + Omega_2_scaled * (t8 * t8) * t18 *
    0.0039345373862287912)) / 2.0) / I_xx;
  t74 = I_xx * p;
  accelerations_array[4] = -((((((((((l_4 * t54 - l_3 * t46) + l_4 * t52) - l_3 *
    t44) + l_z * t67) + l_z * t62) + l_z * t76) + l_z * t24) + t74 * r) - I_zz *
    p * r) - t69 * wing_chord * (Cm_zero + Cm_alpha * t59) / 2.0) / I_yy;
  accelerations_array[5] = (((((((((l_1 * t67 - l_1 * t62) - l_2 * t76) + l_2 *
    t24) - l_4 * t42) + l_3 * t73) - l_4 * t75) + l_3 * t70) + t74 * q) - I_yy *
    p * q) / I_zz;
}

/*
 * COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2_NEW_AERO
 *     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2_NEW_AERO(IN1,IN2,Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Phi,S,Theta,V,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,FLIGHT_PATH_ANGLE,GAIN_EL,GAIN_AZ,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,Q,R,RHO,WING_CHORD,V_scaled,power_Cd_0,power_Cd_a,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,WING_SPAN)
 *
 * Arguments    : const double in1[13]
 *                const double in2[6]
 *                double Beta
 *                double CL_aileron
 *                double Cd_zero
 *                double Cl_alpha
 *                double Cm_zero
 *                double Cm_alpha
 *                double I_xx
 *                double I_yy
 *                double I_zz
 *                double K_Cd
 *                double Phi
 *                double S
 *                double Theta
 *                double V
 *                double W_act_motor
 *                double W_dv_1
 *                double W_dv_2
 *                double W_dv_3
 *                double W_dv_4
 *                double W_dv_5
 *                double W_dv_6
 *                double W_act_tilt_el
 *                double W_act_tilt_az
 *                double W_act_ailerons
 *                double desired_el_value
 *                double desired_az_value
 *                double desired_motor_value
 *                double desired_ailerons_value
 *                double flight_path_angle
 *                double gain_el
 *                double gain_az
 *                double gain_motor
 *                double gain_ailerons
 *                double gamma_quadratic_du
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double m
 *                double p
 *                double q
 *                double r
 *                double rho
 *                double wing_chord
 *                double V_scaled
 *                double power_Cd_0
 *                double power_Cd_a
 *                double prop_R
 *                double prop_Cd_0
 *                double prop_Cl_0
 *                double prop_Cd_a
 *                double prop_Cl_a
 *                double prop_delta
 *                double prop_sigma
 *                double prop_theta
 *                double wing_span
 *                double gradient_data[]
 *                int *gradient_size
 * Return Type  : double
 */
static double c_compute_cost_and_gradient_sec(const double in1[13], const double
  in2[6], double Beta, double CL_aileron, double Cd_zero, double Cl_alpha,
  double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz, double
  K_Cd, double Phi, double S, double Theta, double V, double W_act_motor, double
  W_dv_1, double W_dv_2, double W_dv_3, double W_dv_4, double W_dv_5, double
  W_dv_6, double W_act_tilt_el, double W_act_tilt_az, double W_act_ailerons,
  double desired_el_value, double desired_az_value, double desired_motor_value,
  double desired_ailerons_value, double flight_path_angle, double gain_el,
  double gain_az, double gain_motor, double gain_ailerons, double
  gamma_quadratic_du, double l_1, double l_2, double l_3, double l_4, double l_z,
  double m, double p, double q, double r, double rho, double wing_chord, double
  V_scaled, double power_Cd_0, double power_Cd_a, double prop_R, double
  prop_Cd_0, double prop_Cl_0, double prop_Cd_a, double prop_Cl_a, double
  prop_delta, double prop_sigma, double prop_theta, double wing_span, double
  gradient_data[], int *gradient_size)
{
  double b_CL_aileron[227];
  double b_gradient_data[13];
  double cost;
  double t10;
  double t103;
  double t103_tmp;
  double t104;
  double t106;
  double t107;
  double t11;
  double t112;
  double t113;
  double t114;
  double t115;
  double t118;
  double t119;
  double t12;
  double t120;
  double t129;
  double t13;
  double t132;
  double t134;
  double t14;
  double t142;
  double t143;
  double t144;
  double t145;
  double t146;
  double t147;
  double t148;
  double t149;
  double t15;
  double t150;
  double t154;
  double t155;
  double t156;
  double t157;
  double t158;
  double t16;
  double t162_tmp;
  double t175_tmp;
  double t175_tmp_tmp;
  double t19;
  double t197_tmp;
  double t20;
  double t21;
  double t217_tmp;
  double t217_tmp_tmp;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t3;
  double t307;
  double t340_tmp;
  double t341_tmp;
  double t342_tmp;
  double t343_tmp;
  double t347;
  double t356;
  double t357;
  double t358;
  double t359;
  double t364;
  double t365;
  double t366;
  double t367;
  double t39;
  double t4;
  double t40;
  double t42;
  double t43;
  double t47;
  double t476;
  double t477;
  double t478;
  double t479;
  double t48;
  double t5;
  double t504;
  double t505;
  double t506;
  double t55;
  double t56;
  double t57;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t62;
  double t63;
  double t64;
  double t65;
  double t66;
  double t67;
  double t679;
  double t68;
  double t69;
  double t7;
  double t70;
  double t74;
  double t75;
  double t76;
  double t77;
  double t78;
  double t79;
  double t8;
  double t80;
  double t81;
  double t82;
  double t83;
  double t84;
  double t85;
  double t89;
  double t9;
  double t90;
  double t95;
  double t97;
  double t98;
  double t99;

  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     11-Oct-2024 01:03:58 */
  t3 = cos(Phi);
  t4 = sin(Beta);
  t5 = cos(Theta);
  t6 = sin(Phi);
  t7 = sin(Theta);
  t8 = log(prop_delta);
  t9 = in1[4] * gain_el;
  t10 = in1[5] * gain_el;
  t11 = in1[6] * gain_el;
  t12 = in1[7] * gain_el;
  t13 = in1[8] * gain_az;
  t14 = in1[9] * gain_az;
  t15 = in1[10] * gain_az;
  t16 = in1[11] * gain_az;
  t19 = in1[0] * in1[0];
  t20 = rt_powd_snf(in1[0], 3.0);
  t21 = in1[1] * in1[1];
  t22 = rt_powd_snf(in1[1], 3.0);
  t23 = in1[2] * in1[2];
  t24 = rt_powd_snf(in1[2], 3.0);
  t25 = in1[3] * in1[3];
  t26 = rt_powd_snf(in1[3], 3.0);
  t27 = V * V;
  t28 = V_scaled * V_scaled;
  t39 = gain_motor * gain_motor;
  t40 = prop_Cd_a * 2.0;
  t42 = rt_powd_snf(prop_R, 3.0);
  t43 = rt_powd_snf(prop_R, 4.0);
  t47 = prop_Cd_0 * prop_delta * 2.0;
  t74 = 1.0 / in1[0];
  t76 = 1.0 / in1[1];
  t79 = 1.0 / in1[2];
  t82 = 1.0 / in1[3];
  t89 = 1.0 / gain_motor;
  t95 = prop_delta * 16.0;
  t97 = 1.0 / prop_R;
  t99 = 1.0 / prop_delta;
  t48 = prop_theta * t40;
  t55 = cos(t9);
  t56 = cos(t10);
  t57 = cos(t11);
  t58 = cos(t12);
  t59 = cos(t13);
  t60 = cos(t14);
  t61 = cos(t15);
  t62 = cos(t16);
  t63 = sin(t9);
  t64 = sin(t10);
  t65 = sin(t11);
  t66 = sin(t12);
  t67 = sin(t13);
  t68 = sin(t14);
  t69 = sin(t15);
  t70 = sin(t16);
  t75 = 1.0 / t19;
  t77 = 1.0 / t20;
  t78 = 1.0 / t21;
  t80 = 1.0 / t22;
  t81 = 1.0 / t23;
  t83 = 1.0 / t24;
  t84 = 1.0 / t25;
  t85 = 1.0 / t26;
  t90 = 1.0 / t39;
  t98 = t97 * t97;
  t103_tmp = prop_Cl_0 * prop_delta;
  t103 = t103_tmp * (prop_delta + 1.0);
  t104 = Theta - flight_path_angle;
  t106 = desired_el_value * (1.0 / gain_el);
  t107 = desired_az_value * (1.0 / gain_az);
  t132 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t679 = prop_Cl_a * prop_sigma;
  t134 = t679 * (prop_delta - 1.0) / 8.0;
  t112 = t55 * t55;
  t113 = t56 * t56;
  t114 = t57 * t57;
  t115 = t58 * t58;
  t118 = cos(t104);
  t119 = sin(t104);
  t120 = t103 * 8.0;
  t129 = power_Cd_0 * prop_delta + power_Cd_a * (prop_theta * prop_theta);
  t13 = (t9 + 1.5707963267948966) + t104;
  t14 = (t10 + 1.5707963267948966) + t104;
  t15 = (t11 + 1.5707963267948966) + t104;
  t16 = (t12 + 1.5707963267948966) + t104;
  t142 = cos(t13);
  t143 = cos(t14);
  t144 = cos(t15);
  t145 = cos(t16);
  t146 = sin(t13);
  t147 = sin(t14);
  t148 = sin(t15);
  t149 = sin(t16);
  t162_tmp = Cl_alpha * S * rho * t27 * t104;
  t150 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t104 * t104);
  t9 = t142 * t142;
  t10 = t143 * t143;
  t11 = t144 * t144;
  t154 = t145 * t145;
  t155 = t146 * t146;
  t156 = t147 * t147;
  t157 = t148 * t148;
  t158 = t149 * t149;
  t175_tmp_tmp = S * rho;
  t175_tmp = t175_tmp_tmp * cos(Beta) * t27;
  t197_tmp = prop_Cl_a * prop_theta * t27;
  t217_tmp_tmp = t103_tmp * t8;
  t217_tmp = t217_tmp_tmp * t27;
  t307 = t162_tmp * t118 / 2.0 + t175_tmp * t119 * t150 / 2.0;
  t340_tmp = t27 * t75 * t90 * t98;
  t12 = V * prop_Cl_a * prop_sigma;
  t103_tmp = prop_Cl_0 * prop_sigma * t8 * t27;
  t13 = prop_sigma * (prop_delta - 1.0) * t99;
  t14 = sqrt(((t340_tmp * t9 * 16.0 + t12 * t74 * t89 * (prop_delta - 1.0) * t97
               * t142 * 8.0) + t103_tmp * t75 * t90 * t98 * t155 * -8.0) - t13 *
             (t120 + prop_Cl_a * (t132 + prop_theta * (t95 + t340_tmp * t155 *
    8.0))));
  t341_tmp = t27 * t78 * t90 * t98;
  t15 = sqrt(((t341_tmp * t10 * 16.0 + t12 * t76 * t89 * (prop_delta - 1.0) *
               t97 * t143 * 8.0) + t103_tmp * t78 * t90 * t98 * t156 * -8.0) -
             t13 * (t120 + prop_Cl_a * (t132 + prop_theta * (t95 + t341_tmp *
    t156 * 8.0))));
  t342_tmp = t27 * t81 * t90 * t98;
  t16 = sqrt(((t342_tmp * t11 * 16.0 + t12 * t79 * t89 * (prop_delta - 1.0) *
               t97 * t144 * 8.0) + t103_tmp * t81 * t90 * t98 * t157 * -8.0) -
             t13 * (t120 + prop_Cl_a * (t132 + prop_theta * (t95 + t342_tmp *
    t157 * 8.0))));
  t343_tmp = t27 * t84 * t90 * t98;
  t13 = sqrt(((t343_tmp * t154 * 16.0 + t12 * t82 * t89 * (prop_delta - 1.0) *
               t97 * t145 * 8.0) + t103_tmp * t84 * t90 * t98 * t158 * -8.0) -
             t13 * (t120 + prop_Cl_a * (t132 + prop_theta * (t95 + t343_tmp *
    t158 * 8.0))));
  t132 = 1.0 / t14;
  t120 = 1.0 / t15;
  t95 = 1.0 / t16;
  t347 = 1.0 / t13;
  t356 = (t134 + V * t74 * t89 * t97 * t142 / 2.0) + t14 / 8.0;
  t357 = (t134 + V * t76 * t89 * t97 * t143 / 2.0) + t15 / 8.0;
  t358 = (t134 + V * t79 * t89 * t97 * t144 / 2.0) + t16 / 8.0;
  t359 = (t134 + V * t82 * t89 * t97 * t145 / 2.0) + t13 / 8.0;
  t364 = -prop_theta + t356;
  t365 = -prop_theta + t357;
  t366 = -prop_theta + t358;
  t367 = -prop_theta + t359;
  t13 = t679 * prop_theta * t27;
  t476 = V * t75 * t89 * t97 * t142 / 2.0 + (((t27 * t77 * t90 * t98 * t9 * 32.0
    + t12 * t75 * t89 * (prop_delta - 1.0) * t97 * t142 * 8.0) - t103_tmp * t77 *
    t90 * t98 * t155 * 16.0) - t13 * t77 * t90 * (prop_delta - 1.0) * t98 * t99 *
    t155 * 16.0) * t132 / 16.0;
  t477 = V * t78 * t89 * t97 * t143 / 2.0 + (((t27 * t80 * t90 * t98 * t10 *
    32.0 + t12 * t78 * t89 * (prop_delta - 1.0) * t97 * t143 * 8.0) - t103_tmp *
    t80 * t90 * t98 * t156 * 16.0) - t13 * t80 * t90 * (prop_delta - 1.0) * t98 *
    t99 * t156 * 16.0) * t120 / 16.0;
  t478 = V * t81 * t89 * t97 * t144 / 2.0 + (((t27 * t83 * t90 * t98 * t11 *
    32.0 + t12 * t81 * t89 * (prop_delta - 1.0) * t97 * t144 * 8.0) - t103_tmp *
    t83 * t90 * t98 * t157 * 16.0) - t13 * t83 * t90 * (prop_delta - 1.0) * t98 *
    t99 * t157 * 16.0) * t95 / 16.0;
  t479 = V * t84 * t89 * t97 * t145 / 2.0 + (((t27 * t85 * t90 * t98 * t154 *
    32.0 + t12 * t84 * t89 * (prop_delta - 1.0) * t97 * t145 * 8.0) - t103_tmp *
    t85 * t90 * t98 * t158 * 16.0) - t13 * t85 * t90 * (prop_delta - 1.0) * t98 *
    t99 * t158 * 16.0) * t347 / 16.0;
  t13 = V * gain_el;
  t14 = t13 * prop_Cl_a * prop_sigma;
  t15 = gain_el * t27;
  t10 = gain_el * prop_Cl_0;
  t16 = t10 * prop_sigma * t8 * t27;
  t11 = gain_el * prop_Cl_a;
  t9 = t11 * prop_sigma * prop_theta * t27;
  t504 = t13 * t74 * t89 * t97 * t146 / 2.0 + t132 * (((t14 * t74 * t89 *
    (prop_delta - 1.0) * t97 * t146 * 8.0 + t15 * t75 * t90 * t98 * t142 * t146 *
    32.0) + t16 * t75 * t90 * t98 * t142 * t146 * 16.0) + t9 * t75 * t90 *
    (prop_delta - 1.0) * t98 * t99 * t142 * t146 * 16.0) / 16.0;
  t505 = t13 * t76 * t89 * t97 * t147 / 2.0 + t120 * (((t14 * t76 * t89 *
    (prop_delta - 1.0) * t97 * t147 * 8.0 + t15 * t78 * t90 * t98 * t143 * t147 *
    32.0) + t16 * t78 * t90 * t98 * t143 * t147 * 16.0) + t9 * t78 * t90 *
    (prop_delta - 1.0) * t98 * t99 * t143 * t147 * 16.0) / 16.0;
  t506 = t13 * t79 * t89 * t97 * t148 / 2.0 + t95 * (((t14 * t79 * t89 *
    (prop_delta - 1.0) * t97 * t148 * 8.0 + t15 * t81 * t90 * t98 * t144 * t148 *
    32.0) + t16 * t81 * t90 * t98 * t144 * t148 * 16.0) + t9 * t81 * t90 *
    (prop_delta - 1.0) * t98 * t99 * t144 * t148 * 16.0) / 16.0;
  t79 = t13 * t82 * t89 * t97 * t149 / 2.0 + t347 * (((t14 * t82 * t89 *
    (prop_delta - 1.0) * t97 * t149 * 8.0 + t15 * t84 * t90 * t98 * t145 * t149 *
    32.0) + t16 * t84 * t90 * t98 * t145 * t149 * 16.0) + t9 * t84 * t90 *
    (prop_delta - 1.0) * t98 * t99 * t145 * t149 * 16.0) / 16.0;
  t13 = prop_Cl_a * prop_delta;
  t82 = prop_theta * (prop_delta - 1.0);
  t76 = prop_Cl_a - t40;
  t154 = t217_tmp_tmp * t476 + t82 * t476 * t76;
  t679 = t217_tmp_tmp * t477 + t82 * t477 * t76;
  t347 = t217_tmp_tmp * t478 + t82 * t478 * t76;
  t74 = t217_tmp_tmp * t479 + t82 * t479 * t76;
  b_CL_aileron[0] = CL_aileron;
  b_CL_aileron[1] = in1[0];
  b_CL_aileron[2] = in1[1];
  b_CL_aileron[3] = in1[2];
  b_CL_aileron[4] = in1[3];
  b_CL_aileron[5] = S;
  b_CL_aileron[6] = V;
  b_CL_aileron[7] = in1[4];
  b_CL_aileron[8] = in1[5];
  b_CL_aileron[9] = in1[6];
  b_CL_aileron[10] = in1[7];
  b_CL_aileron[11] = in1[12];
  b_CL_aileron[12] = desired_ailerons_value;
  b_CL_aileron[13] = in2[0];
  b_CL_aileron[14] = in2[1];
  b_CL_aileron[15] = in2[2];
  b_CL_aileron[16] = in2[3];
  b_CL_aileron[17] = in2[4];
  b_CL_aileron[18] = in2[5];
  b_CL_aileron[19] = in1[8];
  b_CL_aileron[20] = in1[9];
  b_CL_aileron[21] = in1[10];
  b_CL_aileron[22] = in1[11];
  b_CL_aileron[23] = gain_ailerons;
  b_CL_aileron[24] = gain_az;
  b_CL_aileron[25] = gain_el;
  b_CL_aileron[26] = gain_motor;
  b_CL_aileron[27] = gamma_quadratic_du;
  b_CL_aileron[28] = l_1;
  b_CL_aileron[29] = l_2;
  b_CL_aileron[30] = l_3;
  b_CL_aileron[31] = l_4;
  b_CL_aileron[32] = l_z;
  b_CL_aileron[33] = power_Cd_a;
  b_CL_aileron[34] = prop_Cl_0;
  b_CL_aileron[35] = prop_Cl_a;
  b_CL_aileron[36] = prop_sigma;
  b_CL_aileron[37] = rho;
  b_CL_aileron[38] = 1.5707963267948966;
  b_CL_aileron[39] = desired_motor_value * t89;
  b_CL_aileron[40] = -(I_zz * p * r);
  b_CL_aileron[41] = -(I_yy * q * r);
  b_CL_aileron[42] = t112;
  b_CL_aileron[43] = t113;
  b_CL_aileron[44] = t114;
  b_CL_aileron[45] = t115;
  b_CL_aileron[46] = -t106;
  b_CL_aileron[47] = -(t106 * 2.0);
  b_CL_aileron[48] = -t107;
  b_CL_aileron[49] = -(t107 * 2.0);
  b_CL_aileron[50] = t129;
  b_CL_aileron[51] = power_Cd_0 * ((prop_delta + 1.0) + prop_delta * prop_delta)
    * 2.0;
  b_CL_aileron[52] = prop_delta / 6.0 - 0.16666666666666666;
  b_CL_aileron[53] = -(CL_aileron * S * in1[12] * gain_ailerons * rho * t27 /
                       2.0);
  b_CL_aileron[54] = t142;
  b_CL_aileron[55] = t143;
  b_CL_aileron[56] = t144;
  b_CL_aileron[57] = t145;
  b_CL_aileron[58] = t146;
  b_CL_aileron[59] = t147;
  b_CL_aileron[60] = t148;
  b_CL_aileron[61] = t149;
  b_CL_aileron[62] = t155;
  b_CL_aileron[63] = t156;
  b_CL_aileron[64] = t157;
  b_CL_aileron[65] = t158;
  t134 = t175_tmp_tmp * t27;
  b_CL_aileron[66] = -(t134 * (Cm_zero + Cm_alpha * t104) * wing_chord / 2.0);
  b_CL_aileron[67] = t175_tmp_tmp * t3 * t4 * t27 * t150 / 2.0;
  t95 = t175_tmp_tmp * t4;
  b_CL_aileron[68] = t95 * t5 * t6 * t27 * t150 / 2.0;
  b_CL_aileron[69] = t95 * t6 * t7 * t27 * t150 / 2.0;
  b_CL_aileron[70] = t19;
  b_CL_aileron[71] = t20;
  b_CL_aileron[72] = t217_tmp * t75 * t90 * t98 * t155;
  b_CL_aileron[73] = t217_tmp * t78 * t90 * t98 * t156;
  b_CL_aileron[74] = t217_tmp * t81 * t90 * t98 * t157;
  b_CL_aileron[75] = t217_tmp * t84 * t90 * t98 * t158;
  b_CL_aileron[76] = t21;
  b_CL_aileron[77] = t22;
  b_CL_aileron[78] = t23;
  b_CL_aileron[79] = t24;
  b_CL_aileron[80] = t25;
  b_CL_aileron[81] = t26;
  b_CL_aileron[82] = t27;
  b_CL_aileron[83] = t340_tmp * t99 * t129 * t155 * 3.0;
  b_CL_aileron[84] = t341_tmp * t99 * t129 * t156 * 3.0;
  b_CL_aileron[85] = t342_tmp * t99 * t129 * t157 * 3.0;
  b_CL_aileron[86] = t343_tmp * t99 * t129 * t158 * 3.0;
  b_CL_aileron[87] = t28;
  b_CL_aileron[88] = W_act_motor * W_act_motor;
  t95 = t10 * prop_delta * t8 * t27;
  b_CL_aileron[89] = t95 * t75 * t90 * t98 * t142 * t146 * 2.0;
  b_CL_aileron[90] = t95 * t78 * t90 * t98 * t143 * t147 * 2.0;
  b_CL_aileron[91] = t95 * t81 * t90 * t98 * t144 * t148 * 2.0;
  b_CL_aileron[92] = t95 * t84 * t90 * t98 * t145 * t149 * 2.0;
  b_CL_aileron[93] = t3;
  b_CL_aileron[94] = W_dv_1 * W_dv_1;
  b_CL_aileron[95] = W_dv_2 * W_dv_2;
  t95 = t162_tmp * t119 / 2.0 - t175_tmp * t118 * t150 / 2.0;
  b_CL_aileron[96] = -t5 * t95;
  b_CL_aileron[97] = -(t6 * t307);
  b_CL_aileron[98] = t3 * t5 * t307;
  b_CL_aileron[99] = t3 * t7 * t307;
  b_CL_aileron[100] = t7 * t95;
  b_CL_aileron[101] = W_dv_3 * W_dv_3;
  b_CL_aileron[102] = W_dv_4 * W_dv_4;
  b_CL_aileron[103] = W_dv_5 * W_dv_5;
  b_CL_aileron[104] = W_dv_6 * W_dv_6;
  b_CL_aileron[105] = W_act_tilt_el * W_act_tilt_el;
  b_CL_aileron[106] = t364;
  b_CL_aileron[107] = t365;
  b_CL_aileron[108] = t366;
  b_CL_aileron[109] = t367;
  b_CL_aileron[110] = t217_tmp_tmp * t356;
  b_CL_aileron[111] = t217_tmp_tmp * t357;
  b_CL_aileron[112] = W_act_tilt_az * W_act_tilt_az;
  b_CL_aileron[113] = t217_tmp_tmp * t358;
  b_CL_aileron[114] = t217_tmp_tmp * t359;
  t95 = prop_Cl_0 * (prop_delta + 1.0);
  b_CL_aileron[115] = t95 * t356 * 3.0;
  b_CL_aileron[116] = t95 * t357 * 3.0;
  b_CL_aileron[117] = t95 * t358 * 3.0;
  b_CL_aileron[118] = t95 * t359 * 3.0;
  b_CL_aileron[119] = W_act_ailerons * W_act_ailerons;
  b_CL_aileron[120] = t39;
  b_CL_aileron[121] = t47 + prop_theta * (t48 + t356 * t76);
  b_CL_aileron[122] = t47 + prop_theta * (t48 + t357 * t76);
  b_CL_aileron[123] = prop_delta + 1.0;
  b_CL_aileron[124] = t47 + prop_theta * (t48 + t358 * t76);
  b_CL_aileron[125] = t47 + prop_theta * (t48 + t359 * t76);
  b_CL_aileron[126] = (t103 + t197_tmp * t75 * t90 * t98 * t155) - t13 * t364 *
    2.0;
  b_CL_aileron[127] = (t103 + t197_tmp * t78 * t90 * t98 * t156) - t13 * t365 *
    2.0;
  b_CL_aileron[128] = (t103 + t197_tmp * t81 * t90 * t98 * t157) - t13 * t366 *
    2.0;
  b_CL_aileron[129] = (t103 + t197_tmp * t84 * t90 * t98 * t158) - t13 * t367 *
    2.0;
  b_CL_aileron[130] = t42;
  b_CL_aileron[131] = t43;
  b_CL_aileron[132] = rt_powd_snf(prop_R, 5.0);
  b_CL_aileron[133] = t476;
  b_CL_aileron[134] = t477;
  b_CL_aileron[135] = t478;
  b_CL_aileron[136] = t479;
  t95 = I_xx * p;
  b_CL_aileron[137] = t95 * q;
  b_CL_aileron[138] = t5;
  b_CL_aileron[139] = I_yy * p * q;
  b_CL_aileron[140] = t504;
  b_CL_aileron[141] = t505;
  b_CL_aileron[142] = t506;
  b_CL_aileron[143] = t79;
  b_CL_aileron[144] = t95 * r;
  t95 = t11 * prop_theta * t27;
  b_CL_aileron[145] = (prop_delta - 1.0) * (t95 * t75 * t90 * t98 * t142 * t146 *
    2.0 + t13 * t504 * 2.0);
  b_CL_aileron[146] = (prop_delta - 1.0) * (t95 * t78 * t90 * t98 * t143 * t147 *
    2.0 + t13 * t505 * 2.0);
  b_CL_aileron[147] = (prop_delta - 1.0) * (t95 * t81 * t90 * t98 * t144 * t148 *
    2.0 + t13 * t506 * 2.0);
  b_CL_aileron[148] = (prop_delta - 1.0) * (t95 * t84 * t90 * t98 * t145 * t149 *
    2.0 + t13 * t79 * 2.0);
  b_CL_aileron[149] = I_zz * q * r;
  b_CL_aileron[150] = prop_Cl_a * t356 * 6.0 - power_Cd_a * t364 * 6.0;
  b_CL_aileron[151] = prop_Cl_a * t357 * 6.0 - power_Cd_a * t365 * 6.0;
  b_CL_aileron[152] = prop_Cl_a * t358 * 6.0 - power_Cd_a * t366 * 6.0;
  b_CL_aileron[153] = prop_Cl_a * t359 * 6.0 - power_Cd_a * t367 * 6.0;
  t95 = prop_sigma * rho;
  t120 = t95 * t19 * t39 * t43;
  t103_tmp = t217_tmp * t77 * t90 * t98 * t155 * 2.0 + (prop_delta - 1.0) *
    (t197_tmp * t77 * t90 * t98 * t155 * 2.0 - t13 * t476 * 2.0);
  b_CL_aileron[154] = t120 * t63 * t99 * 3.1415926535897931 * t103_tmp * -0.25;
  t132 = t95 * t21 * t39 * t43;
  t12 = t217_tmp * t80 * t90 * t98 * t156 * 2.0 + (prop_delta - 1.0) * (t197_tmp
    * t80 * t90 * t98 * t156 * 2.0 - t13 * t477 * 2.0);
  b_CL_aileron[155] = t132 * t64 * t99 * 3.1415926535897931 * t12 * -0.25;
  t11 = t95 * t23 * t39 * t43;
  t10 = t217_tmp * t83 * t90 * t98 * t157 * 2.0 + (prop_delta - 1.0) * (t197_tmp
    * t83 * t90 * t98 * t157 * 2.0 - t13 * t478 * 2.0);
  b_CL_aileron[156] = t11 * t65 * t99 * 3.1415926535897931 * t10 * -0.25;
  t95 = t95 * t25 * t39 * t43;
  t9 = t217_tmp * t85 * t90 * t98 * t158 * 2.0 + (prop_delta - 1.0) * (t197_tmp *
    t85 * t90 * t98 * t158 * 2.0 - t13 * t479 * 2.0);
  b_CL_aileron[157] = t95 * t66 * t99 * 3.1415926535897931 * t9 * -0.25;
  t120 *= t55;
  b_CL_aileron[158] = t120 * t59 * t99 * 3.1415926535897931 * t103_tmp * -0.25;
  t132 *= t56;
  b_CL_aileron[159] = t132 * t60 * t99 * 3.1415926535897931 * t12 * -0.25;
  b_CL_aileron[160] = t55;
  t11 *= t57;
  b_CL_aileron[161] = t11 * t61 * t99 * 3.1415926535897931 * t10 * -0.25;
  t95 *= t58;
  b_CL_aileron[162] = t95 * t62 * t99 * 3.1415926535897931 * t9 * -0.25;
  b_CL_aileron[163] = t120 * t67 * t99 * 3.1415926535897931 * t103_tmp * -0.25;
  b_CL_aileron[164] = t132 * t68 * t99 * 3.1415926535897931 * t12 * -0.25;
  b_CL_aileron[165] = t11 * t69 * t99 * 3.1415926535897931 * t10 * -0.25;
  b_CL_aileron[166] = t95 * t70 * t99 * 3.1415926535897931 * t9 * -0.25;
  b_CL_aileron[167] = t56;
  b_CL_aileron[168] = t57;
  b_CL_aileron[169] = t58;
  b_CL_aileron[170] = t59;
  b_CL_aileron[171] = t6;
  b_CL_aileron[172] = t60;
  b_CL_aileron[173] = t61;
  b_CL_aileron[174] = t62;
  b_CL_aileron[175] = t63;
  b_CL_aileron[176] = t64;
  t95 = in1[1] * t65;
  t120 = in1[0] * t66;
  b_CL_aileron[177] = t134 * (((((((((((in1[0] * t55 * 0.01228861457458129 -
    in1[1] * t56 * 0.01228861457458129) + in1[1] * t28 * t56 *
    0.02051939481844044) + t19 * t55 * 0.0075450647873164947) - in1[0] * t28 *
    t55 * 0.02051939481844044) - t21 * t56 * 0.0075450647873164947) + t95 * t114
    * 0.0064371741699132748) + in1[0] * t63 * t115 * 0.0046438440894603552) +
    t95 * t113 * 0.0039345373862287912) - t120 * t115 * 0.0064371741699132748) -
    in1[1] * t64 * t114 * 0.0046438440894603552) - t120 * t112 *
    0.0039345373862287912) * wing_span / 2.0;
  b_CL_aileron[178] = t65;
  b_CL_aileron[179] = t66;
  b_CL_aileron[180] = t67;
  b_CL_aileron[181] = t68;
  b_CL_aileron[182] = t74;
  t134 = in1[0] * V * gain_motor * prop_sigma * rho * t42;
  t95 = t134 * t55 * t99 * t146;
  b_CL_aileron[183] = t95 * t154 * 3.1415926535897931 / 4.0;
  t120 = in1[1] * V * gain_motor * prop_sigma * rho * t42;
  t103_tmp = t120 * t56 * t99 * t147;
  b_CL_aileron[184] = t103_tmp * t679 * 3.1415926535897931 / 4.0;
  t132 = in1[2] * V * gain_motor * prop_sigma * rho * t42;
  t12 = t132 * t57 * t99 * t148;
  b_CL_aileron[185] = t12 * t347 * 3.1415926535897931 / 4.0;
  t11 = in1[3] * V * gain_motor * prop_sigma * rho * t42;
  t10 = t11 * t58 * t99 * t149;
  b_CL_aileron[186] = t10 * t74 * 3.1415926535897931 / 4.0;
  b_CL_aileron[187] = t69;
  t9 = t134 * t59 * t63 * t99 * t146;
  b_CL_aileron[188] = t9 * t154 * 3.1415926535897931 / 4.0;
  t16 = t120 * t60 * t64 * t99 * t147;
  b_CL_aileron[189] = t16 * t679 * 3.1415926535897931 / 4.0;
  t15 = t132 * t61 * t65 * t99 * t148;
  b_CL_aileron[190] = t15 * t347 * 3.1415926535897931 / 4.0;
  t14 = t11 * t62 * t66 * t99 * t149;
  b_CL_aileron[191] = t14 * t74 * 3.1415926535897931 / 4.0;
  t134 = t134 * t63 * t67 * t99 * t146;
  b_CL_aileron[192] = t134 * t154 * 3.1415926535897931 / 4.0;
  t120 = t120 * t64 * t68 * t99 * t147;
  b_CL_aileron[193] = t120 * t679 * 3.1415926535897931 / 4.0;
  t132 = t132 * t65 * t69 * t99 * t148;
  b_CL_aileron[194] = t132 * t347 * 3.1415926535897931 / 4.0;
  b_CL_aileron[195] = t7;
  b_CL_aileron[196] = t70;
  b_CL_aileron[197] = 1.0 / I_xx;
  b_CL_aileron[198] = 1.0 / I_yy;
  b_CL_aileron[199] = 1.0 / I_zz;
  t13 = t217_tmp_tmp * t504 + t82 * t504 * t76;
  b_CL_aileron[200] = t95 * 3.1415926535897931 * t13 * -0.25;
  t95 = t217_tmp_tmp * t505 + t82 * t505 * t76;
  b_CL_aileron[201] = t103_tmp * 3.1415926535897931 * t95 * -0.25;
  b_CL_aileron[202] = t75;
  t103_tmp = t217_tmp_tmp * t506 + t82 * t506 * t76;
  b_CL_aileron[203] = t12 * 3.1415926535897931 * t103_tmp * -0.25;
  t12 = t217_tmp_tmp * t79 + t82 * t79 * t76;
  b_CL_aileron[204] = t10 * 3.1415926535897931 * t12 * -0.25;
  b_CL_aileron[205] = t9 * 3.1415926535897931 * t13 * -0.25;
  b_CL_aileron[206] = t16 * 3.1415926535897931 * t95 * -0.25;
  b_CL_aileron[207] = t15 * 3.1415926535897931 * t103_tmp * -0.25;
  b_CL_aileron[208] = t14 * 3.1415926535897931 * t12 * -0.25;
  b_CL_aileron[209] = t134 * 3.1415926535897931 * t13 * -0.25;
  b_CL_aileron[210] = t120 * 3.1415926535897931 * t95 * -0.25;
  b_CL_aileron[211] = t132 * 3.1415926535897931 * t103_tmp * -0.25;
  b_CL_aileron[212] = t11 * t66 * t70 * t99 * t149 * 3.1415926535897931 * t12 *
    -0.25;
  b_CL_aileron[213] = t77;
  b_CL_aileron[214] = t78;
  b_CL_aileron[215] = t80;
  b_CL_aileron[216] = t81;
  b_CL_aileron[217] = t83;
  b_CL_aileron[218] = t84;
  b_CL_aileron[219] = t85;
  b_CL_aileron[220] = t90;
  b_CL_aileron[221] = 1.0 / gain_ailerons;
  b_CL_aileron[222] = 1.0 / m;
  b_CL_aileron[223] = prop_delta - 1.0;
  b_CL_aileron[224] = t98;
  b_CL_aileron[225] = t99;
  b_CL_aileron[226] = wing_span;
  cost = ft_1(b_CL_aileron, b_gradient_data, gradient_size);
  if (*gradient_size - 1 >= 0) {
    memcpy(&gradient_data[0], &b_gradient_data[0], (unsigned int)*gradient_size *
           sizeof(double));
  }

  return cost;
}

/*
 * Arguments    : const double xCurrent[13]
 *                const int finiteLB[14]
 *                int mLB
 *                const double lb[13]
 *                const int finiteUB[14]
 *                int mUB
 *                const double ub[13]
 *                const double lambda[27]
 *                int iL0
 * Return Type  : double
 */
static double computeComplError(const double xCurrent[13], const int finiteLB[14],
  int mLB, const double lb[13], const int finiteUB[14], int mUB, const double
  ub[13], const double lambda[27], int iL0)
{
  double nlpComplError;
  int idx;
  nlpComplError = 0.0;
  if (mLB + mUB > 0) {
    double lbDelta;
    double lbLambda;
    int i;
    int i1;
    int ubOffset;
    ubOffset = (iL0 + mLB) - 1;
    i = (unsigned char)mLB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteLB[idx];
      lbDelta = xCurrent[i1 - 1] - lb[i1 - 1];
      lbLambda = lambda[(iL0 + idx) - 1];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }

    i = (unsigned char)mUB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteUB[idx];
      lbDelta = ub[i1 - 1] - xCurrent[i1 - 1];
      lbLambda = lambda[ubOffset + idx];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }
  }

  return nlpComplError;
}

/*
 * Arguments    : const struct_T *obj
 *                double workspace[378]
 *                const double H[169]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : double
 */
static double computeFval(const struct_T *obj, double workspace[378], const
  double H[169], const double f[14], const double x[14])
{
  double val;
  int idx;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;

   case 3:
    {
      linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
      val = 0.0;
      if (obj->nvar >= 1) {
        int i;
        i = (unsigned char)obj->nvar;
        for (idx = 0; idx < i; idx++) {
          val += x[idx] * workspace[idx];
        }
      }
    }
    break;

   default:
    {
      int i;
      linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
      i = obj->nvar + 1;
      for (idx = i; idx < 14; idx++) {
        workspace[idx - 1] = 0.5 * obj->beta * x[idx - 1] + obj->rho;
      }

      val = 0.0;
      for (idx = 0; idx < 13; idx++) {
        val += x[idx] * workspace[idx];
      }
    }
    break;
  }

  return val;
}

/*
 * Arguments    : const struct_T *obj
 *                double workspace[378]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : double
 */
static double computeFval_ReuseHx(const struct_T *obj, double workspace[378],
  const double f[14], const double x[14])
{
  double val;
  int k;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int i;
        i = (unsigned char)obj->nvar;
        for (k = 0; k < i; k++) {
          workspace[k] = 0.5 * obj->Hx[k] + f[k];
        }

        val = 0.0;
        if (obj->nvar >= 1) {
          for (k = 0; k < i; k++) {
            val += x[k] * workspace[k];
          }
        }
      } else {
        val = 0.0;
        if (obj->nvar >= 1) {
          int i;
          i = (unsigned char)obj->nvar;
          for (k = 0; k < i; k++) {
            val += x[k] * obj->Hx[k];
          }
        }

        val *= 0.5;
      }
    }
    break;

   default:
    {
      if (obj->hasLinear) {
        int i;
        i = (unsigned char)obj->nvar;
        if (i - 1 >= 0) {
          memcpy(&workspace[0], &f[0], (unsigned int)i * sizeof(double));
        }

        i = 12 - obj->nvar;
        for (k = 0; k <= i; k++) {
          workspace[obj->nvar + k] = obj->rho;
        }

        val = 0.0;
        for (k = 0; k < 13; k++) {
          double d;
          d = workspace[k] + 0.5 * obj->Hx[k];
          workspace[k] = d;
          val += x[k] * d;
        }
      } else {
        int i;
        val = 0.0;
        for (k = 0; k < 13; k++) {
          val += x[k] * obj->Hx[k];
        }

        val *= 0.5;
        i = obj->nvar + 1;
        for (k = i; k < 14; k++) {
          val += x[k - 1] * obj->rho;
        }
      }
    }
    break;
  }

  return val;
}

/*
 * Arguments    : double workspace[14]
 *                int nVar
 *                const double grad[14]
 *                const int finiteFixed[14]
 *                int mFixed
 *                const int finiteLB[14]
 *                int mLB
 *                const int finiteUB[14]
 *                int mUB
 *                const double lambda[27]
 * Return Type  : void
 */
static void computeGradLag(double workspace[14], int nVar, const double grad[14],
  const int finiteFixed[14], int mFixed, const int finiteLB[14], int mLB, const
  int finiteUB[14], int mUB, const double lambda[27])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }

  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }

  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }

  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : struct_T *obj
 *                const double H[169]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : void
 */
static void computeGrad_StoreHx(struct_T *obj, const double H[169], const double
  f[14], const double x[14])
{
  int ixlast;
  int k;
  switch (obj->objtype) {
   case 5:
    {
      int i;
      i = obj->nvar;
      if (i - 2 >= 0) {
        memset(&obj->grad[0], 0, (unsigned int)(i - 1) * sizeof(double));
      }

      obj->grad[obj->nvar - 1] = obj->gammaScalar;
    }
    break;

   case 3:
    {
      int i;
      xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
      i = (unsigned char)obj->nvar;
      if (i - 1 >= 0) {
        memcpy(&obj->grad[0], &obj->Hx[0], (unsigned int)i * sizeof(double));
      }

      if (obj->hasLinear && (obj->nvar >= 1)) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          obj->grad[k] += f[k];
        }
      }
    }
    break;

   default:
    {
      int i;
      xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
      i = obj->nvar + 1;
      for (ixlast = i; ixlast < 14; ixlast++) {
        obj->Hx[ixlast - 1] = obj->beta * x[ixlast - 1];
      }

      memcpy(&obj->grad[0], &obj->Hx[0], 13U * sizeof(double));
      if (obj->hasLinear && (obj->nvar >= 1)) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          obj->grad[k] += f[k];
        }
      }

      if (13 - obj->nvar >= 1) {
        ixlast = obj->nvar;
        i = 12 - obj->nvar;
        for (k = 0; k <= i; k++) {
          int i1;
          i1 = ixlast + k;
          obj->grad[i1] += obj->rho;
        }
      }
    }
    break;
  }
}

/*
 * Arguments    : d_struct_T *obj
 *                int nrows
 * Return Type  : void
 */
static void computeQ_(d_struct_T *obj, int nrows)
{
  double work[27];
  int b_i;
  int iQR0;
  int ia;
  int idx;
  int lastc;
  int m;
  int n;
  lastc = obj->minRowCol;
  for (idx = 0; idx < lastc; idx++) {
    iQR0 = 27 * idx + idx;
    n = obj->mrows - idx;
    if (n - 2 >= 0) {
      memcpy(&obj->Q[iQR0 + 1], &obj->QR[iQR0 + 1], (unsigned int)(((n + iQR0) -
               iQR0) - 1) * sizeof(double));
    }
  }

  m = obj->mrows;
  if (nrows >= 1) {
    int i;
    int i1;
    int itau;
    i = nrows - 1;
    for (idx = lastc; idx <= i; idx++) {
      ia = idx * 27;
      i1 = m - 1;
      memset(&obj->Q[ia], 0, (unsigned int)(((i1 + ia) - ia) + 1) * sizeof
             (double));
      obj->Q[ia + idx] = 1.0;
    }

    itau = obj->minRowCol - 1;
    memset(&work[0], 0, 27U * sizeof(double));
    for (b_i = obj->minRowCol; b_i >= 1; b_i--) {
      int iaii;
      iaii = b_i + (b_i - 1) * 27;
      if (b_i < nrows) {
        int lastv;
        obj->Q[iaii - 1] = 1.0;
        idx = iaii + 27;
        if (obj->tau[itau] != 0.0) {
          bool exitg2;
          lastv = m - b_i;
          iQR0 = (iaii + m) - b_i;
          while ((lastv + 1 > 0) && (obj->Q[iQR0 - 1] == 0.0)) {
            lastv--;
            iQR0--;
          }

          lastc = (nrows - b_i) - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int exitg1;
            iQR0 = (iaii + lastc * 27) + 27;
            ia = iQR0;
            do {
              exitg1 = 0;
              if (ia <= iQR0 + lastv) {
                if (obj->Q[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = -1;
        }

        if (lastv + 1 > 0) {
          double c;
          if (lastc + 1 != 0) {
            if (lastc >= 0) {
              memset(&work[0], 0, (unsigned int)(lastc + 1) * sizeof(double));
            }

            i = (iaii + 27 * lastc) + 27;
            for (n = idx; n <= i; n += 27) {
              c = 0.0;
              i1 = n + lastv;
              for (ia = n; ia <= i1; ia++) {
                c += obj->Q[ia - 1] * obj->Q[((iaii + ia) - n) - 1];
              }

              iQR0 = div_nde_s32_floor((n - iaii) - 27, 27);
              work[iQR0] += c;
            }
          }

          if (!(-obj->tau[itau] == 0.0)) {
            iQR0 = iaii;
            for (idx = 0; idx <= lastc; idx++) {
              c = work[idx];
              if (c != 0.0) {
                c *= -obj->tau[itau];
                i = iQR0 + 27;
                i1 = lastv + iQR0;
                for (n = i; n <= i1 + 27; n++) {
                  obj->Q[n - 1] += obj->Q[((iaii + n) - iQR0) - 28] * c;
                }
              }

              iQR0 += 27;
            }
          }
        }
      }

      if (b_i < m) {
        iQR0 = iaii + 1;
        i = (iaii + m) - b_i;
        for (lastc = iQR0; lastc <= i; lastc++) {
          obj->Q[lastc - 1] *= -obj->tau[itau];
        }
      }

      obj->Q[iaii - 1] = 1.0 - obj->tau[itau];
      i = (unsigned char)(b_i - 1);
      for (idx = 0; idx < i; idx++) {
        obj->Q[(iaii - idx) - 2] = 0.0;
      }

      itau--;
    }
  }
}

/*
 * Arguments    : const double H[169]
 *                h_struct_T *solution
 *                f_struct_T *memspace
 *                const d_struct_T *qrmanager
 *                e_struct_T *cholmanager
 *                const struct_T *objective
 *                bool alwaysPositiveDef
 * Return Type  : void
 */
static void compute_deltax(const double H[169], h_struct_T *solution, f_struct_T
  *memspace, const d_struct_T *qrmanager, e_struct_T *cholmanager, const
  struct_T *objective, bool alwaysPositiveDef)
{
  int b_i;
  int idx;
  int ix;
  int jA;
  int jjA;
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (nVar_tmp >= 0) {
      memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) * sizeof
             (double));
    }
  } else {
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution->searchDir[idx] = -objective->grad[idx];
    }

    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
       case 5:
        break;

       case 3:
        {
          double smax;
          int nVars;
          if (alwaysPositiveDef) {
            cholmanager->ndims = qrmanager->mrows;
            for (idx = 0; idx <= nVar_tmp; idx++) {
              jjA = (nVar_tmp + 1) * idx;
              jA = 27 * idx;
              for (ix = 0; ix <= nVar_tmp; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            cholmanager->info = xpotrf(qrmanager->mrows, cholmanager->FMat);
          } else {
            cholmanager->ndims = qrmanager->mrows;
            for (idx = 0; idx <= nVar_tmp; idx++) {
              jjA = qrmanager->mrows * idx;
              jA = 27 * idx;
              for (ix = 0; ix <= nVar_tmp; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            if (qrmanager->mrows < 1) {
              nVars = -1;
            } else {
              nVars = 0;
              if (qrmanager->mrows > 1) {
                smax = fabs(cholmanager->FMat[0]);
                for (ix = 2; ix <= nVar_tmp + 1; ix++) {
                  double s;
                  s = fabs(cholmanager->FMat[(ix - 1) * 28]);
                  if (s > smax) {
                    nVars = ix - 1;
                    smax = s;
                  }
                }
              }
            }

            cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[nVars + 27 *
              nVars]) * 2.2204460492503131E-16, 0.0);
            fullColLDL2_(cholmanager, qrmanager->mrows);
            if (cholmanager->ConvexCheck) {
              idx = 0;
              int exitg1;
              do {
                exitg1 = 0;
                if (idx <= nVar_tmp) {
                  if (cholmanager->FMat[idx + 27 * idx] <= 0.0) {
                    cholmanager->info = -idx - 1;
                    exitg1 = 1;
                  } else {
                    idx++;
                  }
                } else {
                  cholmanager->ConvexCheck = false;
                  exitg1 = 1;
                }
              } while (exitg1 == 0);
            }
          }

          if (cholmanager->info != 0) {
            solution->state = -6;
          } else if (alwaysPositiveDef) {
            solve(cholmanager, solution->searchDir);
          } else {
            int i;
            nVars = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= nVars + 1; idx++) {
                jjA = idx + idx * 27;
                i = nVars - idx;
                for (b_i = 0; b_i <= i; b_i++) {
                  ix = (idx + b_i) + 1;
                  solution->searchDir[ix] -= solution->searchDir[idx] *
                    cholmanager->FMat[(jjA + b_i) + 1];
                }
              }
            }

            nVars = cholmanager->ndims;
            for (idx = 0; idx < nVars; idx++) {
              solution->searchDir[idx] /= cholmanager->FMat[idx + 27 * idx];
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 27;
                smax = solution->searchDir[idx - 1];
                i = idx + 1;
                for (b_i = nVars; b_i >= i; b_i--) {
                  smax -= cholmanager->FMat[(jA + b_i) - 1] *
                    solution->searchDir[b_i - 1];
                }

                solution->searchDir[idx - 1] = smax;
              }
            }
          }
        }
        break;

       default:
        {
          if (alwaysPositiveDef) {
            int nVars;
            nVars = objective->nvar;
            cholmanager->ndims = objective->nvar;
            for (idx = 0; idx < nVars; idx++) {
              jjA = nVars * idx;
              jA = 27 * idx;
              for (ix = 0; ix < nVars; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            cholmanager->info = xpotrf(objective->nvar, cholmanager->FMat);
            if (cholmanager->info != 0) {
              solution->state = -6;
            } else {
              double smax;
              int i;
              solve(cholmanager, solution->searchDir);
              smax = 1.0 / objective->beta;
              jjA = objective->nvar + 1;
              i = qrmanager->mrows;
              for (ix = jjA; ix <= i; ix++) {
                solution->searchDir[ix - 1] *= smax;
              }
            }
          }
        }
        break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = 27 * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double[idx] = -qrmanager->Q[nVar_tmp + 27 *
            (qrmanager->ncols + idx)];
        }

        if (qrmanager->mrows != 0) {
          int i;
          memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) *
                 sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
          for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
            int nVars;
            nVars = jjA + nVar_tmp;
            for (idx = jjA; idx <= nVars; idx++) {
              jA = idx - jjA;
              solution->searchDir[jA] += qrmanager->Q[idx - 1] *
                memspace->workspace_double[ix];
            }

            ix++;
          }
        }
      } else {
        double smax;
        int i;
        int nVars;
        if (objective->objtype == 3) {
          xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                qrmanager->mrows, qrmanager->Q, nullStartIdx_tmp,
                memspace->workspace_double);
          b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager->mrows;
          xgemm(objective->nvar, mNull_tmp, objective->nvar, H, objective->nvar,
                qrmanager->Q, nullStartIdx_tmp, memspace->workspace_double);
          i = objective->nvar + 1;
          for (jA = 0; jA < mNull_tmp; jA++) {
            for (jjA = i; jjA <= nVars; jjA++) {
              memspace->workspace_double[(jjA + 27 * jA) - 1] = objective->beta *
                qrmanager->Q[(jjA + 27 * (jA + qrmanager->ncols)) - 1];
            }
          }

          b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        }

        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull_tmp;
          cholmanager->info = xpotrf(mNull_tmp, cholmanager->FMat);
        } else {
          cholmanager->ndims = mNull_tmp;
          nVars = 0;
          if (mNull_tmp > 1) {
            smax = fabs(cholmanager->FMat[0]);
            for (ix = 2; ix <= mNull_tmp; ix++) {
              double s;
              s = fabs(cholmanager->FMat[(ix - 1) * 28]);
              if (s > smax) {
                nVars = ix - 1;
                smax = s;
              }
            }
          }

          cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[nVars + 27 * nVars])
            * 2.2204460492503131E-16, 0.0);
          fullColLDL2_(cholmanager, mNull_tmp);
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager->FMat[idx + 27 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            memset(&memspace->workspace_double[0], 0, (unsigned int)mNull_tmp *
                   sizeof(double));
            i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
              smax = 0.0;
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                smax += qrmanager->Q[idx - 1] * objective->grad[idx - jjA];
              }

              nVars = div_nde_s32_floor(jjA - nullStartIdx_tmp, 27);
              memspace->workspace_double[nVars] -= smax;
            }
          }

          if (alwaysPositiveDef) {
            nVars = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx < nVars; idx++) {
                jA = idx * 27;
                smax = memspace->workspace_double[idx];
                for (b_i = 0; b_i < idx; b_i++) {
                  smax -= cholmanager->FMat[jA + b_i] *
                    memspace->workspace_double[b_i];
                }

                memspace->workspace_double[idx] = smax / cholmanager->FMat[jA +
                  idx];
              }
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jjA = (idx + (idx - 1) * 27) - 1;
                memspace->workspace_double[idx - 1] /= cholmanager->FMat[jjA];
                for (b_i = 0; b_i <= idx - 2; b_i++) {
                  ix = (idx - b_i) - 2;
                  memspace->workspace_double[ix] -= memspace->
                    workspace_double[idx - 1] * cholmanager->FMat[(jjA - b_i) -
                    1];
                }
              }
            }
          } else {
            nVars = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= nVars + 1; idx++) {
                jjA = idx + idx * 27;
                i = nVars - idx;
                for (b_i = 0; b_i <= i; b_i++) {
                  ix = (idx + b_i) + 1;
                  memspace->workspace_double[ix] -= memspace->
                    workspace_double[idx] * cholmanager->FMat[(jjA + b_i) + 1];
                }
              }
            }

            nVars = cholmanager->ndims;
            for (idx = 0; idx < nVars; idx++) {
              memspace->workspace_double[idx] /= cholmanager->FMat[idx + 27 *
                idx];
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 27;
                smax = memspace->workspace_double[idx - 1];
                i = idx + 1;
                for (b_i = nVars; b_i >= i; b_i--) {
                  smax -= cholmanager->FMat[(jA + b_i) - 1] *
                    memspace->workspace_double[b_i - 1];
                }

                memspace->workspace_double[idx - 1] = smax;
              }
            }
          }

          if (qrmanager->mrows != 0) {
            memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) *
                   sizeof(double));
            ix = 0;
            i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                jA = idx - jjA;
                solution->searchDir[jA] += qrmanager->Q[idx - 1] *
                  memspace->workspace_double[ix];
              }

              ix++;
            }
          }
        }
      }
    }
  }
}

/*
 * Arguments    : int x[27]
 *                int xLen
 *                int workspace[27]
 *                int xMin
 *                int xMax
 * Return Type  : void
 */
static void countsort(int x[27], int xLen, int workspace[27], int xMin, int xMax)
{
  int idx;
  int idxFill;
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    if (idxStart >= 0) {
      memset(&workspace[0], 0, (unsigned int)(idxStart + 1) * sizeof(int));
    }

    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart]++;
    }

    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] += workspace[idx - 2];
    }

    idxStart = 1;
    idxEnd = workspace[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }

      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }

    for (idx = idxStart; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

/*
 * Arguments    : d_struct_T *obj
 *                int idx
 * Return Type  : void
 */
static void deleteColMoveEnd(d_struct_T *obj, int idx)
{
  double s;
  double temp_tmp;
  int b_k;
  int i;
  int k;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    int b_i;
    int u0;
    b_i = obj->ncols - 1;
    obj->jpvt[idx - 1] = obj->jpvt[b_i];
    i = obj->minRowCol;
    for (k = 0; k < i; k++) {
      obj->QR[k + 27 * (idx - 1)] = obj->QR[k + 27 * b_i];
    }

    obj->ncols = b_i;
    u0 = obj->mrows;
    i = obj->ncols;
    if (u0 <= i) {
      i = u0;
    }

    obj->minRowCol = i;
    if (idx < obj->mrows) {
      double c;
      double temp;
      int QRk0;
      int b_temp_tmp;
      int endIdx;
      int n;
      u0 = obj->mrows - 1;
      endIdx = obj->ncols;
      if (u0 <= endIdx) {
        endIdx = u0;
      }

      k = endIdx;
      i = 27 * (idx - 1);
      while (k >= idx) {
        b_i = k + i;
        temp_tmp = obj->QR[b_i];
        c = xrotg(&obj->QR[b_i - 1], &temp_tmp, &s);
        obj->QR[b_i] = temp_tmp;
        b_i = 27 * (k - 1);
        obj->QR[k + b_i] = 0.0;
        QRk0 = k + 27 * idx;
        n = obj->ncols - idx;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 27;
            temp_tmp = obj->QR[b_temp_tmp - 1];
            temp = c * temp_tmp + s * obj->QR[b_temp_tmp];
            obj->QR[b_temp_tmp] = c * obj->QR[b_temp_tmp] - s * temp_tmp;
            obj->QR[b_temp_tmp - 1] = temp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = b_i + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 27];
          temp = c * obj->Q[b_temp_tmp] + s * temp_tmp;
          obj->Q[b_temp_tmp + 27] = c * temp_tmp - s * obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp] = temp;
        }

        k--;
      }

      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        u0 = 27 * (k - 1);
        i = k + u0;
        temp_tmp = obj->QR[i];
        c = xrotg(&obj->QR[i - 1], &temp_tmp, &s);
        obj->QR[i] = temp_tmp;
        QRk0 = k * 28;
        n = obj->ncols - k;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 27;
            temp_tmp = obj->QR[b_temp_tmp - 1];
            temp = c * temp_tmp + s * obj->QR[b_temp_tmp];
            obj->QR[b_temp_tmp] = c * obj->QR[b_temp_tmp] - s * temp_tmp;
            obj->QR[b_temp_tmp - 1] = temp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = u0 + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 27];
          temp = c * obj->Q[b_temp_tmp] + s * temp_tmp;
          obj->Q[b_temp_tmp + 27] = c * temp_tmp - s * obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp] = temp;
        }
      }
    }
  }
}

/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : int
 */
static int div_nde_s32_floor(int numerator, int denominator)
{
  int i;
  if (((numerator < 0) != (denominator < 0)) && (numerator % denominator != 0))
  {
    i = -1;
  } else {
    i = 0;
  }

  return numerator / denominator + i;
}

/*
 * Arguments    : const double H[169]
 *                const double f[14]
 *                h_struct_T *solution
 *                f_struct_T *memspace
 *                i_struct_T *workingset
 *                d_struct_T *qrmanager
 *                e_struct_T *cholmanager
 *                struct_T *objective
 *                j_struct_T *options
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
static void driver(const double H[169], const double f[14], h_struct_T *solution,
                   f_struct_T *memspace, i_struct_T *workingset, d_struct_T
                   *qrmanager, e_struct_T *cholmanager, struct_T *objective,
                   j_struct_T *options, int runTimeOptions_MaxIterations)
{
  int idxStartIneq;
  int idx_global;
  int mConstr;
  int nVar_tmp;
  bool guard1;
  solution->iterations = 0;
  nVar_tmp = workingset->nVar;
  guard1 = false;
  if (workingset->probType == 3) {
    mConstr = (unsigned char)workingset->sizes[0];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      solution->xstar[workingset->indexFixed[idxStartIneq] - 1] = workingset->
        ub[workingset->indexFixed[idxStartIneq] - 1];
    }

    mConstr = (unsigned char)workingset->sizes[3];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + idxStartIneq)
          - 1]) {
        solution->xstar[workingset->indexLB[idxStartIneq] - 1] = -workingset->
          lb[workingset->indexLB[idxStartIneq] - 1];
      }
    }

    mConstr = (unsigned char)workingset->sizes[4];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset->isActiveConstr[(workingset->isActiveIdx[4] + idxStartIneq)
          - 1]) {
        solution->xstar[workingset->indexUB[idxStartIneq] - 1] = workingset->
          ub[workingset->indexUB[idxStartIneq] - 1];
      }
    }

    PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution->state >= 0) {
      guard1 = true;
    }
  } else {
    solution->state = 82;
    guard1 = true;
  }

  if (guard1) {
    solution->iterations = 0;
    solution->maxConstr = b_maxConstraintViolation(workingset, solution->xstar);
    if (solution->maxConstr > 1.0E-6) {
      int PROBTYPE_ORIG;
      int idxEndIneq_tmp_tmp;
      PROBTYPE_ORIG = workingset->probType;
      solution->xstar[13] = solution->maxConstr + 1.0;
      if (workingset->probType == 3) {
        mConstr = 1;
      } else {
        mConstr = 4;
      }

      setProblemType(workingset, mConstr);
      mConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
      idxStartIneq = mConstr + 1;
      idxEndIneq_tmp_tmp = workingset->nActiveConstr;
      for (idx_global = idxStartIneq; idx_global <= idxEndIneq_tmp_tmp;
           idx_global++) {
        workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
          Wid[idx_global - 1] - 1] + workingset->Wlocalidx[idx_global - 1]) - 2]
          = false;
      }

      workingset->nWConstr[2] = 0;
      workingset->nWConstr[3] = 0;
      workingset->nWConstr[4] = 0;
      workingset->nActiveConstr = mConstr;
      objective->prev_objtype = objective->objtype;
      objective->prev_nvar = objective->nvar;
      objective->prev_hasLinear = objective->hasLinear;
      objective->objtype = 5;
      objective->nvar = 14;
      objective->gammaScalar = 1.0;
      objective->hasLinear = true;
      solution->fstar = computeFval(objective, memspace->workspace_double, H, f,
        solution->xstar);
      solution->state = 5;
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options->SolverName, 1.4901161193847657E-10, 1.0E-6,
              runTimeOptions_MaxIterations);
      if (workingset->isActiveConstr[(workingset->isActiveIdx[3] +
           workingset->sizes[3]) - 2]) {
        bool exitg1;
        idxStartIneq = workingset->sizes[0];
        exitg1 = false;
        while ((!exitg1) && (idxStartIneq + 1 <= workingset->nActiveConstr)) {
          if ((workingset->Wid[idxStartIneq] == 4) && (workingset->
               Wlocalidx[idxStartIneq] == workingset->sizes[3])) {
            removeConstr(workingset, idxStartIneq + 1);
            exitg1 = true;
          } else {
            idxStartIneq++;
          }
        }
      }

      mConstr = workingset->nActiveConstr;
      idxStartIneq = workingset->sizes[0];
      while ((mConstr > idxStartIneq) && (mConstr > nVar_tmp)) {
        removeConstr(workingset, mConstr);
        mConstr--;
      }

      solution->maxConstr = solution->xstar[13];
      setProblemType(workingset, PROBTYPE_ORIG);
      objective->objtype = objective->prev_objtype;
      objective->nvar = objective->prev_nvar;
      objective->hasLinear = objective->prev_hasLinear;
      options->ObjectiveLimit = rtMinusInf;
      options->StepTolerance = 1.0E-6;
      if (solution->state != 0) {
        solution->maxConstr = b_maxConstraintViolation(workingset,
          solution->xstar);
        if (solution->maxConstr > 1.0E-6) {
          memset(&solution->lambda[0], 0, 27U * sizeof(double));
          solution->fstar = computeFval(objective, memspace->workspace_double, H,
            f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            double maxConstr_new;
            mConstr = (unsigned char)nVar_tmp;
            if (mConstr - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (unsigned int)
                     mConstr * sizeof(double));
            }

            PresolveWorkingSet(solution, memspace, workingset, qrmanager);
            maxConstr_new = b_maxConstraintViolation(workingset, solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              if (mConstr - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (unsigned
                        int)mConstr * sizeof(double));
              }
            }
          }

          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, options->SolverName, options->StepTolerance,
                  options->ObjectiveLimit, runTimeOptions_MaxIterations);
        }
      }
    } else {
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options->SolverName, options->StepTolerance,
              options->ObjectiveLimit, runTimeOptions_MaxIterations);
    }
  }
}

/*
 * Arguments    : const c_struct_T *c_obj_next_next_next_next_next_
 *                const double x[13]
 *                int *status
 * Return Type  : double
 */
static double evalObjAndConstr(const c_struct_T *c_obj_next_next_next_next_next_,
  const double x[13], int *status)
{
  double gradient_data[13];
  double fval;
  int gradient_size;
  bool b;
  fval = c_compute_cost_and_gradient_sec(x,
    c_obj_next_next_next_next_next_->dv_global->contents,
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
    c_obj_next_next_next_next_next_->Phi->contents,
    c_obj_next_next_next_next_next_->S->contents,
    c_obj_next_next_next_next_next_->Theta->contents,
    c_obj_next_next_next_next_next_->V->contents,
    c_obj_next_next_next_next_next_->W_act_motor->contents,
    c_obj_next_next_next_next_next_->W_dv_1->contents,
    c_obj_next_next_next_next_next_->W_dv_2->contents,
    c_obj_next_next_next_next_next_->W_dv_3->contents,
    c_obj_next_next_next_next_next_->W_dv_4->contents,
    c_obj_next_next_next_next_next_->W_dv_5->contents,
    c_obj_next_next_next_next_next_->W_dv_6->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_el->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_az->contents,
    c_obj_next_next_next_next_next_->W_act_ailerons->contents,
    c_obj_next_next_next_next_next_->desired_el_value->contents,
    c_obj_next_next_next_next_next_->desired_az_value->contents,
    c_obj_next_next_next_next_next_->desired_motor_value->contents,
    c_obj_next_next_next_next_next_->desired_ailerons_value->contents,
    c_obj_next_next_next_next_next_->flight_path_angle->contents,
    c_obj_next_next_next_next_next_->gain_el->contents,
    c_obj_next_next_next_next_next_->gain_az->contents,
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
    c_obj_next_next_next_next_next_->q->contents,
    c_obj_next_next_next_next_next_->r->contents,
    c_obj_next_next_next_next_next_->rho->contents,
    c_obj_next_next_next_next_next_->wing_chord->contents,
    c_obj_next_next_next_next_next_->V_scaled->contents,
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
    c_obj_next_next_next_next_next_->wing_span->contents, gradient_data,
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
 * Arguments    : d_struct_T *obj
 *                const double A[378]
 *                int mrows
 *                int ncols
 * Return Type  : void
 */
static void factorQR(d_struct_T *obj, const double A[378], int mrows, int ncols)
{
  int i;
  int idx;
  int k;
  bool guard1;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    for (idx = 0; idx < ncols; idx++) {
      int ix0;
      int iy0;
      ix0 = 14 * idx;
      iy0 = 27 * idx;
      i = (unsigned char)mrows;
      for (k = 0; k < i; k++) {
        obj->QR[iy0 + k] = A[ix0 + k];
      }
    }

    guard1 = true;
  } else if (i == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (idx = 0; idx < ncols; idx++) {
      obj->jpvt[idx] = idx + 1;
    }

    if (mrows <= ncols) {
      i = mrows;
    } else {
      i = ncols;
    }

    obj->minRowCol = i;
    memset(&obj->tau[0], 0, 27U * sizeof(double));
    if (i >= 1) {
      qrf(obj->QR, mrows, ncols, i, obj->tau);
    }
  }
}

/*
 * Arguments    : c_struct_T *objfun_workspace
 *                const double lb[13]
 *                const double ub[13]
 *                g_struct_T *obj
 * Return Type  : void
 */
static void factoryConstruct(c_struct_T *objfun_workspace, const double lb[13],
  const double ub[13], g_struct_T *obj)
{
  int i;
  bool bv[13];
  bool b;
  obj->objfun.workspace = *objfun_workspace;
  obj->f_1 = 0.0;
  obj->f_2 = 0.0;
  obj->nVar = 13;
  obj->mIneq = 0;
  obj->mEq = 0;
  obj->numEvals = 0;
  obj->SpecifyObjectiveGradient = true;
  obj->SpecifyConstraintGradient = false;
  obj->isEmptyNonlcon = true;
  obj->FiniteDifferenceType = 0;
  for (i = 0; i < 13; i++) {
    bv[i] = obj->hasUB[i];
  }

  b = false;
  i = 0;
  while ((!b) && (i + 1 <= 13)) {
    obj->hasLB[i] = ((!rtIsInf(lb[i])) && (!rtIsNaN(lb[i])));
    bv[i] = ((!rtIsInf(ub[i])) && (!rtIsNaN(ub[i])));
    if (obj->hasLB[i] || bv[i]) {
      b = true;
    }

    i++;
  }

  while (i + 1 <= 13) {
    obj->hasLB[i] = ((!rtIsInf(lb[i])) && (!rtIsNaN(lb[i])));
    bv[i] = ((!rtIsInf(ub[i])) && (!rtIsNaN(ub[i])));
    i++;
  }

  for (i = 0; i < 13; i++) {
    obj->hasUB[i] = bv[i];
  }

  obj->hasBounds = b;
}

/*
 * Arguments    : double workspace[378]
 *                double xCurrent[14]
 *                const i_struct_T *workingset
 *                d_struct_T *qrmanager
 * Return Type  : bool
 */
static bool feasibleX0ForWorkingSet(double workspace[378], double xCurrent[14],
  const i_struct_T *workingset, d_struct_T *qrmanager)
{
  double B[378];
  int b_i;
  int br;
  int iAcol;
  int j;
  int jBcol;
  int k;
  int mWConstr;
  int nVar;
  bool nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int i;
    int i1;
    for (iAcol = 0; iAcol < mWConstr; iAcol++) {
      c = workingset->bwset[iAcol];
      workspace[iAcol] = c;
      workspace[iAcol + 27] = c;
    }

    if (mWConstr != 0) {
      i = 14 * (mWConstr - 1) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 14) {
        c = 0.0;
        i1 = (iAcol + nVar) - 1;
        for (br = iAcol; br <= i1; br++) {
          c += workingset->ATwset[br - 1] * xCurrent[br - iAcol];
        }

        i1 = div_nde_s32_floor(iAcol - 1, 14);
        workspace[i1] -= c;
      }
    }

    if (mWConstr >= nVar) {
      i = (unsigned char)nVar;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (br = 0; br < i; br++) {
        iAcol = 27 * br;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] = workingset->ATwset[br + 14 * jBcol];
        }

        qrmanager->jpvt[br] = br + 1;
      }

      if (mWConstr <= nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }

      qrmanager->minRowCol = i;
      memset(&qrmanager->tau[0], 0, 27U * sizeof(double));
      if (i >= 1) {
        qrf(qrmanager->QR, mWConstr, nVar, i, qrmanager->tau);
      }

      computeQ_(qrmanager, mWConstr);
      memcpy(&B[0], &workspace[0], 378U * sizeof(double));
      for (b_i = 0; b_i <= 27; b_i += 27) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof
                 (double));
        }
      }

      br = -1;
      for (b_i = 0; b_i <= 27; b_i += 27) {
        jBcol = -1;
        i = b_i + 1;
        i1 = b_i + nVar;
        for (k = i; k <= i1; k++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager->Q[(iAcol + jBcol) + 1] * B[(iAcol + br) + 1];
          }

          workspace[k - 1] += c;
          jBcol += 27;
        }

        br += 27;
      }

      for (j = 0; j < 2; j++) {
        jBcol = 27 * j - 1;
        for (k = nVar; k >= 1; k--) {
          iAcol = 27 * (k - 1) - 1;
          i = k + jBcol;
          c = workspace[i];
          if (c != 0.0) {
            workspace[i] = c / qrmanager->QR[k + iAcol];
            i1 = (unsigned char)(k - 1);
            for (b_i = 0; b_i < i1; b_i++) {
              int i2;
              i2 = (b_i + jBcol) + 1;
              workspace[i2] -= workspace[i] * qrmanager->QR[(b_i + iAcol) + 1];
            }
          }
        }
      }
    } else {
      factorQR(qrmanager, workingset->ATwset, nVar, mWConstr);
      computeQ_(qrmanager, qrmanager->minRowCol);
      for (j = 0; j < 2; j++) {
        jBcol = 27 * j;
        for (b_i = 0; b_i < mWConstr; b_i++) {
          iAcol = 27 * b_i;
          br = b_i + jBcol;
          c = workspace[br];
          i = (unsigned char)b_i;
          for (k = 0; k < i; k++) {
            c -= qrmanager->QR[k + iAcol] * workspace[k + jBcol];
          }

          workspace[br] = c / qrmanager->QR[b_i + iAcol];
        }
      }

      memcpy(&B[0], &workspace[0], 378U * sizeof(double));
      for (b_i = 0; b_i <= 27; b_i += 27) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof
                 (double));
        }
      }

      br = 0;
      for (b_i = 0; b_i <= 27; b_i += 27) {
        jBcol = -1;
        i = br + 1;
        i1 = br + mWConstr;
        for (j = i; j <= i1; j++) {
          int i2;
          i2 = b_i + 1;
          iAcol = b_i + nVar;
          for (k = i2; k <= iAcol; k++) {
            workspace[k - 1] += B[j - 1] * qrmanager->Q[(jBcol + k) - b_i];
          }

          jBcol += 27;
        }

        br += 27;
      }
    }

    iAcol = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (iAcol <= (unsigned char)nVar - 1) {
        if (rtIsInf(workspace[iAcol]) || rtIsNaN(workspace[iAcol])) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[iAcol + 27];
          if (rtIsInf(c) || rtIsNaN(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            iAcol++;
          }
        }
      } else {
        double constrViolation_basicX;
        iAcol = nVar - 1;
        for (k = 0; k <= iAcol; k++) {
          workspace[k] += xCurrent[k];
        }

        c = maxConstraintViolation(workingset, workspace, 1);
        constrViolation_basicX = maxConstraintViolation(workingset, workspace,
          28);
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          i = (unsigned char)nVar;
          memcpy(&xCurrent[0], &workspace[0], (unsigned int)i * sizeof(double));
        } else {
          i = (unsigned char)nVar;
          memcpy(&xCurrent[0], &workspace[27], (unsigned int)i * sizeof(double));
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

/*
 * Arguments    : const double solution_xstar[14]
 *                const double solution_searchDir[14]
 *                int workingset_nVar
 *                const double workingset_lb[14]
 *                const double workingset_ub[14]
 *                const int workingset_indexLB[14]
 *                const int workingset_indexUB[14]
 *                const int workingset_sizes[5]
 *                const int workingset_isActiveIdx[6]
 *                const bool workingset_isActiveConstr[27]
 *                const int workingset_nWConstr[5]
 *                bool isPhaseOne
 *                bool *newBlocking
 *                int *constrType
 *                int *constrIdx
 * Return Type  : double
 */
static double feasibleratiotest(const double solution_xstar[14], const double
  solution_searchDir[14], int workingset_nVar, const double workingset_lb[14],
  const double workingset_ub[14], const int workingset_indexLB[14], const int
  workingset_indexUB[14], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[27], const int
  workingset_nWConstr[5], bool isPhaseOne, bool *newBlocking, int *constrType,
  int *constrIdx)
{
  double alpha;
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio;
  int i;
  int i1;
  int idx;
  alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * b_xnrm2(workingset_nVar,
    solution_searchDir);
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX = (double)isPhaseOne * solution_xstar[workingset_nVar -
      1];
    phaseOneCorrectionP = (double)isPhaseOne *
      solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (idx = 0; idx <= i - 2; idx++) {
      i1 = workingset_indexLB[idx];
      pk_corrected = -solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar[i1 - 1] - workingset_lb[i1 - 1]) -
          phaseOneCorrectionX;
        pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
        if (pk_corrected < alpha) {
          alpha = pk_corrected;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }

    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    pk_corrected = -solution_searchDir[i];
    if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
         [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar[i] - workingset_lb[i];
      pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
      if (pk_corrected < alpha) {
        alpha = pk_corrected;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = (double)isPhaseOne * solution_xstar[workingset_nVar -
      1];
    phaseOneCorrectionP = (double)isPhaseOne *
      solution_searchDir[workingset_nVar - 1];
    i = (unsigned char)workingset_sizes[4];
    for (idx = 0; idx < i; idx++) {
      i1 = workingset_indexUB[idx];
      pk_corrected = solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar[i1 - 1] - workingset_ub[i1 - 1]) -
          phaseOneCorrectionX;
        pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
        if (pk_corrected < alpha) {
          alpha = pk_corrected;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (!isPhaseOne) {
    if ((*newBlocking) && (alpha > 1.0)) {
      *newBlocking = false;
    }

    alpha = fmin(alpha, 1.0);
  }

  return alpha;
}

/*
 * Arguments    : c_struct_T *fun_workspace
 *                double x0[13]
 *                const double lb[13]
 *                const double ub[13]
 *                double *exitflag
 *                double *output_iterations
 *                double *output_funcCount
 *                char output_algorithm[3]
 *                double *output_constrviolation
 *                double *output_stepsize
 *                double *output_lssteplength
 *                double *output_firstorderopt
 * Return Type  : double
 */
static double fmincon(c_struct_T *fun_workspace, double x0[13], const double lb
                      [13], const double ub[13], double *exitflag, double
                      *output_iterations, double *output_funcCount, char
                      output_algorithm[3], double *output_constrviolation,
                      double *output_stepsize, double *output_lssteplength,
                      double *output_firstorderopt)
{
  b_struct_T MeritFunction;
  c_struct_T b_fun_workspace;
  c_struct_T c_FcnEvaluator_next_next_next_n;
  d_struct_T QRManager;
  e_struct_T CholManager;
  f_struct_T memspace;
  g_struct_T unusedExpr;
  h_struct_T TrialState;
  i_coder_internal_stickyStruct r;
  i_struct_T WorkingSet;
  struct_T QPObjective;
  double fval;
  int b_i;
  int i;
  int idx;
  int mUB;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  bool exitg1;
  *exitflag = rtInf;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 13)) {
    if (lb[i] > ub[i]) {
      *exitflag = -2.0;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (*exitflag == -2.0) {
    fval = rtInf;
    *output_iterations = 0.0;
    *output_funcCount = 0.0;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = rtInf;
    *output_stepsize = rtInf;
    *output_lssteplength = rtInf;
    *output_firstorderopt = rtInf;
  } else {
    double scale;
    double y;
    int mFixed;
    int mLB;
    TrialState.nVarMax = 14;
    TrialState.mNonlinIneq = 0;
    TrialState.mNonlinEq = 0;
    TrialState.mIneq = 0;
    TrialState.mEq = 0;
    TrialState.iNonIneq0 = 1;
    TrialState.iNonEq0 = 1;
    TrialState.sqpFval_old = 0.0;
    TrialState.sqpIterations = 0;
    TrialState.sqpExitFlag = 0;
    memset(&TrialState.lambdasqp[0], 0, 27U * sizeof(double));
    TrialState.steplength = 1.0;
    memset(&TrialState.delta_x[0], 0, 14U * sizeof(double));
    TrialState.fstar = 0.0;
    TrialState.firstorderopt = 0.0;
    memset(&TrialState.lambda[0], 0, 27U * sizeof(double));
    TrialState.state = 0;
    TrialState.maxConstr = 0.0;
    TrialState.iterations = 0;
    memcpy(&TrialState.xstarsqp[0], &x0[0], 13U * sizeof(double));
    c_FcnEvaluator_next_next_next_n = *fun_workspace;
    b_fun_workspace = *fun_workspace;
    factoryConstruct(&b_fun_workspace, lb, ub, &unusedExpr);
    WorkingSet.nVar = 13;
    WorkingSet.nVarOrig = 13;
    WorkingSet.nVarMax = 14;
    WorkingSet.ldA = 14;
    memset(&WorkingSet.lb[0], 0, 14U * sizeof(double));
    memset(&WorkingSet.ub[0], 0, 14U * sizeof(double));
    WorkingSet.mEqRemoved = 0;
    memset(&WorkingSet.ATwset[0], 0, 378U * sizeof(double));
    WorkingSet.nActiveConstr = 0;
    memset(&WorkingSet.bwset[0], 0, 27U * sizeof(double));
    memset(&WorkingSet.maxConstrWorkspace[0], 0, 27U * sizeof(double));
    memset(&WorkingSet.Wid[0], 0, 27U * sizeof(int));
    memset(&WorkingSet.Wlocalidx[0], 0, 27U * sizeof(int));
    for (i = 0; i < 27; i++) {
      WorkingSet.isActiveConstr[i] = false;
    }

    for (i = 0; i < 5; i++) {
      WorkingSet.nWConstr[i] = 0;
    }

    WorkingSet.probType = 3;
    WorkingSet.SLACK0 = 1.0E-5;
    for (i = 0; i < 14; i++) {
      WorkingSet.indexLB[i] = 0;
      WorkingSet.indexUB[i] = 0;
      WorkingSet.indexFixed[i] = 0;
    }

    mLB = 0;
    mUB = 0;
    mFixed = 0;
    for (idx = 0; idx < 13; idx++) {
      bool guard1;
      y = lb[idx];
      guard1 = false;
      if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
        if (fabs(y - ub[idx]) < 1.0E-6) {
          mFixed++;
          WorkingSet.indexFixed[mFixed - 1] = idx + 1;
        } else {
          mLB++;
          WorkingSet.indexLB[mLB - 1] = idx + 1;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        y = ub[idx];
        if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
          mUB++;
          WorkingSet.indexUB[mUB - 1] = idx + 1;
        }
      }
    }

    i = (mLB + mUB) + mFixed;
    WorkingSet.mConstr = i;
    WorkingSet.mConstrOrig = i;
    WorkingSet.mConstrMax = 27;
    obj_tmp[0] = (signed char)mFixed;
    obj_tmp[1] = 0;
    obj_tmp[2] = 0;
    obj_tmp[3] = (signed char)mLB;
    obj_tmp[4] = (signed char)mUB;
    b_obj_tmp[0] = (signed char)mFixed;
    b_obj_tmp[1] = 0;
    b_obj_tmp[2] = 0;
    b_obj_tmp[3] = (signed char)(mLB + 1);
    b_obj_tmp[4] = (signed char)mUB;
    WorkingSet.isActiveIdx[0] = 1;
    WorkingSet.isActiveIdx[1] = mFixed;
    WorkingSet.isActiveIdx[2] = 0;
    WorkingSet.isActiveIdx[3] = 0;
    WorkingSet.isActiveIdx[4] = mLB;
    WorkingSet.isActiveIdx[5] = mUB;
    for (i = 0; i < 5; i++) {
      signed char i1;
      signed char i2;
      i1 = obj_tmp[i];
      WorkingSet.sizes[i] = i1;
      WorkingSet.sizesNormal[i] = i1;
      i2 = b_obj_tmp[i];
      WorkingSet.sizesPhaseOne[i] = i2;
      WorkingSet.sizesRegularized[i] = i1;
      WorkingSet.sizesRegPhaseOne[i] = i2;
      WorkingSet.isActiveIdx[i + 1] += WorkingSet.isActiveIdx[i];
    }

    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxNormal[b_i] = WorkingSet.isActiveIdx[b_i];
    }

    WorkingSet.isActiveIdxPhaseOne[0] = 1;
    WorkingSet.isActiveIdxPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxPhaseOne[2] = 0;
    WorkingSet.isActiveIdxPhaseOne[3] = 0;
    WorkingSet.isActiveIdxPhaseOne[4] = mLB + 1;
    WorkingSet.isActiveIdxPhaseOne[5] = mUB;
    for (i = 0; i < 5; i++) {
      WorkingSet.isActiveIdxPhaseOne[i + 1] += WorkingSet.isActiveIdxPhaseOne[i];
    }

    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxRegularized[b_i] = WorkingSet.isActiveIdx[b_i];
      WorkingSet.isActiveIdxRegPhaseOne[b_i] =
        WorkingSet.isActiveIdxPhaseOne[b_i];
    }

    for (idx = 0; idx < mLB; idx++) {
      b_i = WorkingSet.indexLB[idx];
      TrialState.xstarsqp[b_i - 1] = fmax(TrialState.xstarsqp[b_i - 1], lb[b_i -
        1]);
    }

    for (idx = 0; idx < mUB; idx++) {
      b_i = WorkingSet.indexUB[idx];
      TrialState.xstarsqp[b_i - 1] = fmin(TrialState.xstarsqp[b_i - 1], ub[b_i -
        1]);
    }

    for (idx = 0; idx < mFixed; idx++) {
      b_i = WorkingSet.indexFixed[idx];
      TrialState.xstarsqp[b_i - 1] = ub[b_i - 1];
    }

    TrialState.sqpFval = c_computeObjectiveAndUserGradie
      (&c_FcnEvaluator_next_next_next_n, TrialState.xstarsqp, TrialState.grad,
       &i);
    TrialState.FunctionEvaluations = 1;
    for (idx = 0; idx < mLB; idx++) {
      WorkingSet.lb[WorkingSet.indexLB[idx] - 1] = -lb[WorkingSet.indexLB[idx] -
        1] + x0[WorkingSet.indexLB[idx] - 1];
    }

    for (idx = 0; idx < mUB; idx++) {
      WorkingSet.ub[WorkingSet.indexUB[idx] - 1] = ub[WorkingSet.indexUB[idx] -
        1] - x0[WorkingSet.indexUB[idx] - 1];
    }

    for (idx = 0; idx < mFixed; idx++) {
      y = ub[WorkingSet.indexFixed[idx] - 1] - x0[WorkingSet.indexFixed[idx] - 1];
      WorkingSet.ub[WorkingSet.indexFixed[idx] - 1] = y;
      WorkingSet.bwset[idx] = y;
    }

    setProblemType(&WorkingSet, 3);
    i = WorkingSet.isActiveIdx[2];
    for (idx = i; idx < 28; idx++) {
      WorkingSet.isActiveConstr[idx - 1] = false;
    }

    WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
    WorkingSet.nWConstr[1] = 0;
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
    b_i = (unsigned char)WorkingSet.sizes[0];
    for (mUB = 0; mUB < b_i; mUB++) {
      WorkingSet.Wid[mUB] = 1;
      WorkingSet.Wlocalidx[mUB] = mUB + 1;
      WorkingSet.isActiveConstr[mUB] = true;
      i = 14 * mUB;
      idx = WorkingSet.indexFixed[mUB];
      if (idx - 2 >= 0) {
        memset(&WorkingSet.ATwset[i], 0, (unsigned int)(((idx + i) - i) - 1) *
               sizeof(double));
      }

      WorkingSet.ATwset[(WorkingSet.indexFixed[mUB] + i) - 1] = 1.0;
      idx = WorkingSet.indexFixed[mUB] + 1;
      mLB = WorkingSet.nVar;
      if (idx <= mLB) {
        memset(&WorkingSet.ATwset[(idx + i) + -1], 0, (unsigned int)((((mLB + i)
                  - idx) - i) + 1) * sizeof(double));
      }

      WorkingSet.bwset[mUB] = WorkingSet.ub[WorkingSet.indexFixed[mUB] - 1];
    }

    double Hessian[169];
    MeritFunction.initFval = TrialState.sqpFval;
    MeritFunction.penaltyParam = 1.0;
    MeritFunction.threshold = 0.0001;
    MeritFunction.nPenaltyDecreases = 0;
    MeritFunction.linearizedConstrViol = 0.0;
    MeritFunction.initConstrViolationEq = 0.0;
    MeritFunction.initConstrViolationIneq = 0.0;
    MeritFunction.phi = 0.0;
    MeritFunction.phiPrimePlus = 0.0;
    MeritFunction.phiFullStep = 0.0;
    MeritFunction.feasRelativeFactor = 0.0;
    MeritFunction.nlpPrimalFeasError = 0.0;
    MeritFunction.nlpDualFeasError = 0.0;
    MeritFunction.nlpComplError = 0.0;
    MeritFunction.firstOrderOpt = 0.0;
    MeritFunction.hasObjective = true;
    r.next.next.next.next.next.next.next.next.value.workspace =
      c_FcnEvaluator_next_next_next_n;
    b_driver(lb, ub, &TrialState, &MeritFunction, &r, &memspace, &WorkingSet,
             Hessian, &QRManager, &CholManager, &QPObjective);
    fval = TrialState.sqpFval;
    *exitflag = TrialState.sqpExitFlag;
    *output_iterations = TrialState.sqpIterations;
    *output_funcCount = TrialState.FunctionEvaluations;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = MeritFunction.nlpPrimalFeasError;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (i = 0; i < 13; i++) {
      double absxk;
      x0[i] = TrialState.xstarsqp[i];
      absxk = fabs(TrialState.delta_x[i]);
      if (absxk > scale) {
        double t;
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        double t;
        t = absxk / scale;
        y += t * t;
      }
    }

    *output_stepsize = scale * sqrt(y);
    *output_lssteplength = TrialState.steplength;
    *output_firstorderopt = MeritFunction.firstOrderOpt;
  }

  return fval;
}

/*
 * Arguments    : const double ct[227]
 *                double gradient_data[]
 *                int *gradient_size
 * Return Type  : double
 */
static double ft_1(const double ct[227], double gradient_data[], int
                   *gradient_size)
{
  double cost;
  double t424;
  double t425;
  double t426;
  double t427;
  double t436;
  double t436_tmp_tmp;
  double t437;
  double t437_tmp_tmp;
  double t438;
  double t438_tmp;
  double t438_tmp_tmp;
  double t439;
  double t439_tmp;
  double t439_tmp_tmp;
  double t452;
  double t452_tmp;
  double t453;
  double t453_tmp;
  double t454;
  double t454_tmp;
  double t455;
  double t455_tmp;
  double t456;
  double t456_tmp;
  double t457;
  double t457_tmp;
  double t458;
  double t458_tmp;
  double t459;
  double t459_tmp;
  double t556;
  double t557;
  double t558;
  double t559;
  double t560;
  double t561;
  double t562;
  double t563;
  double t564;
  double t564_tmp;
  double t565;
  double t566;
  double t567;
  double t569;
  double t570;
  double t571;
  double t572;
  double t609;
  double t610;
  double t611;
  double t612;
  double t613;
  double t614;
  double t615;
  double t616;
  double t702;
  double t703;
  double t704;
  double t705;
  double t716;
  double t717;
  double t718;
  double t719;
  double t720;
  double t721;
  double t722;
  double t723;
  double t732;
  double t733;
  double t734;
  double t735;
  double t744;
  double t745;
  double t746;
  double t747;
  double t766;
  double t767;
  double t768;
  double t769;
  double t774;
  double t774_tmp;
  double t774_tmp_tmp;
  double t775;
  double t775_tmp;
  double t775_tmp_tmp;
  double t776;
  double t776_tmp;
  double t776_tmp_tmp;
  double t777;
  double t777_tmp;
  double t777_tmp_tmp;
  double t778;
  double t779;
  double t780;
  double t781;
  double t782;
  double t783;
  double t784;
  double t785;
  double t786;
  double t787;
  double t788;
  double t789;
  double t790;
  double t791;
  double t793;
  double t795;
  double t797;
  double t798;
  double t799;
  double t800;
  double t801;
  double t818;
  double t821;
  t560 = ct[89] + ct[145];
  t561 = ct[90] + ct[146];
  t562 = ct[91] + ct[147];
  t563 = ct[92] + ct[148];
  t424 = ct[72] + ct[126] * ct[223];
  t425 = ct[73] + ct[127] * ct[223];
  t426 = ct[74] + ct[128] * ct[223];
  t427 = ct[75] + ct[129] * ct[223];
  t556 = ct[110] + ct[121] * ct[223];
  t557 = ct[111] + ct[122] * ct[223];
  t558 = ct[113] + ct[124] * ct[223];
  t559 = ct[114] + ct[125] * ct[223];
  t818 = ct[36] * ct[37];
  t436_tmp_tmp = t818 * ct[70] * ct[120];
  t613 = t436_tmp_tmp * ct[131];
  t795 = t613 * ct[175] * ct[225];
  t436 = t795 * t424 * 3.1415926535897931 / 4.0;
  t437_tmp_tmp = t818 * ct[76] * ct[120];
  t614 = t437_tmp_tmp * ct[131];
  t793 = t614 * ct[176] * ct[225];
  t437 = t793 * t425 * 3.1415926535897931 / 4.0;
  t438_tmp_tmp = t818 * ct[78] * ct[120];
  t615 = t438_tmp_tmp * ct[131];
  t438_tmp = t615 * ct[178] * ct[225];
  t438 = t438_tmp * t426 * 3.1415926535897931 / 4.0;
  t439_tmp_tmp = t818 * ct[80] * ct[120];
  t616 = t439_tmp_tmp * ct[131];
  t439_tmp = t616 * ct[179] * ct[225];
  t439 = t439_tmp * t427 * 3.1415926535897931 / 4.0;
  t613 *= ct[160];
  t452_tmp = t613 * ct[170] * ct[225];
  t452 = t452_tmp * t424 * 3.1415926535897931 / 4.0;
  t614 *= ct[167];
  t453_tmp = t614 * ct[172] * ct[225];
  t453 = t453_tmp * t425 * 3.1415926535897931 / 4.0;
  t615 *= ct[168];
  t454_tmp = t615 * ct[173] * ct[225];
  t454 = t454_tmp * t426 * 3.1415926535897931 / 4.0;
  t616 *= ct[169];
  t455_tmp = t616 * ct[174] * ct[225];
  t455 = t455_tmp * t427 * 3.1415926535897931 / 4.0;
  t456_tmp = t613 * ct[180] * ct[225];
  t456 = t456_tmp * t424 * 3.1415926535897931 / 4.0;
  t457_tmp = t614 * ct[181] * ct[225];
  t457 = t457_tmp * t425 * 3.1415926535897931 / 4.0;
  t458_tmp = t615 * ct[187] * ct[225];
  t458 = t458_tmp * t426 * 3.1415926535897931 / 4.0;
  t459_tmp = t616 * ct[196] * ct[225];
  t459 = t459_tmp * t427 * 3.1415926535897931 / 4.0;
  t564_tmp = ct[6] * ct[26] * ct[36] * ct[37] * ct[130];
  t564 = t564_tmp * ct[160] * ct[225] * ct[58] * t556 * 3.1415926535897931 / 4.0;
  t565 = t564_tmp * ct[167] * ct[225] * ct[59] * t557 * 3.1415926535897931 / 4.0;
  t566 = t564_tmp * ct[168] * ct[225] * ct[60] * t558 * 3.1415926535897931 / 4.0;
  t567 = t564_tmp * ct[169] * ct[225] * ct[61] * t559 * 3.1415926535897931 / 4.0;
  t800 = ct[1] * ct[6];
  t613 = t800 * ct[26] * ct[36] * ct[37] * ct[130];
  t609 = t613 * ct[170] * ct[175] * ct[225] * ct[58] * t556 * 3.1415926535897931
    * -0.25;
  t799 = ct[2] * ct[6];
  t614 = t799 * ct[26] * ct[36] * ct[37] * ct[130];
  t610 = t614 * ct[172] * ct[176] * ct[225] * ct[59] * t557 * 3.1415926535897931
    * -0.25;
  t798 = ct[3] * ct[6];
  t615 = t798 * ct[26] * ct[36] * ct[37] * ct[130];
  t611 = t615 * ct[173] * ct[178] * ct[225] * ct[60] * t558 * 3.1415926535897931
    * -0.25;
  t797 = ct[4] * ct[6];
  t616 = t797 * ct[26] * ct[36] * ct[37] * ct[130];
  t612 = t616 * ct[174] * ct[179] * ct[225] * ct[61] * t559 * 3.1415926535897931
    * -0.25;
  t613 = t613 * ct[175] * ct[180] * ct[225] * ct[58] * t556 * 3.1415926535897931
    * -0.25;
  t614 = t614 * ct[176] * ct[181] * ct[225] * ct[59] * t557 * 3.1415926535897931
    * -0.25;
  t615 = t615 * ct[178] * ct[187] * ct[225] * ct[60] * t558 * 3.1415926535897931
    * -0.25;
  t801 = t616 * ct[179] * ct[196] * ct[225] * ct[61];
  t616 = t801 * t559 * 3.1415926535897931 * -0.25;
  t766 = ((ct[51] + ct[83]) + ct[115]) - ct[106] * ct[150];
  t767 = ((ct[51] + ct[84]) + ct[116]) - ct[107] * ct[151];
  t768 = ((ct[51] + ct[85]) + ct[117]) - ct[108] * ct[152];
  t769 = ((ct[51] + ct[86]) + ct[118]) - ct[109] * ct[153];
  t569 = ct[1] * t564;
  t570 = ct[2] * t565;
  t571 = ct[3] * t566;
  t572 = ct[4] * t567;
  t716 = t452 + t609;
  t717 = t453 + t610;
  t718 = t454 + t611;
  t719 = t455 + t612;
  t720 = t456 + t613;
  t721 = t457 + t614;
  t722 = t458 + t615;
  t723 = t459 + t616;
  t821 = ct[1] * ct[36] * ct[37] * ct[120] * ct[131];
  t778 = ((t821 * ct[175] * ct[225] * ct[38] * t424 + ct[154]) + t564) - ct[183];
  t564 = ct[2] * ct[36] * ct[37] * ct[120] * ct[131];
  t779 = ((t564 * ct[176] * ct[225] * ct[38] * t425 + ct[155]) + t565) - ct[184];
  t565 = ct[3] * ct[36] * ct[37] * ct[120] * ct[131];
  t780 = ((t565 * ct[178] * ct[225] * ct[38] * t426 + ct[156]) + t566) - ct[185];
  t566 = ct[4] * ct[36] * ct[37] * ct[120] * ct[131];
  t781 = ((t566 * ct[179] * ct[225] * ct[38] * t427 + ct[157]) + t567) - ct[186];
  t702 = t436 + t569;
  t703 = t437 + t570;
  t704 = t438 + t571;
  t705 = t439 + t572;
  t732 = ct[24] * t452 + ct[24] * t609;
  t733 = ct[24] * t453 + ct[24] * t610;
  t734 = ct[24] * t454 + ct[24] * t611;
  t735 = ct[24] * t455 + ct[24] * t612;
  t744 = ct[24] * t456 + ct[24] * t613;
  t745 = ct[24] * t457 + ct[24] * t614;
  t746 = ct[24] * t458 + ct[24] * t615;
  t747 = ct[24] * t459 + ct[24] * t616;
  t774_tmp_tmp = t818 * ct[71];
  t774_tmp = t774_tmp_tmp * ct[120] * ct[132];
  t774 = ct[39] + t774_tmp * ct[38] * ct[52] * t766;
  t775_tmp_tmp = t818 * ct[77];
  t775_tmp = t775_tmp_tmp * ct[120] * ct[132];
  t775 = ct[39] + t775_tmp * ct[38] * ct[52] * t767;
  t776_tmp_tmp = t818 * ct[79];
  t776_tmp = t776_tmp_tmp * ct[120] * ct[132];
  t776 = ct[39] + t776_tmp * ct[38] * ct[52] * t768;
  t777_tmp_tmp = t818 * ct[81];
  t777_tmp = t777_tmp_tmp * ct[120] * ct[132];
  t777 = ct[39] + t777_tmp * ct[38] * ct[52] * t769;
  t614 = t821 * ct[160];
  t782 = ((t614 * ct[180] * ct[225] * ct[38] * t424 + ct[163]) - t564_tmp * ct
          [175] * ct[180] * ct[225] * ct[58] * t556 * 3.1415926535897931 / 4.0)
    + ct[192];
  t615 = t564 * ct[167];
  t783 = ((t615 * ct[181] * ct[225] * ct[38] * t425 + ct[164]) - t564_tmp * ct
          [176] * ct[181] * ct[225] * ct[59] * t557 * 3.1415926535897931 / 4.0)
    + ct[193];
  t616 = t565 * ct[168];
  t784 = ((t616 * ct[187] * ct[225] * ct[38] * t426 + ct[165]) - t564_tmp * ct
          [178] * ct[187] * ct[225] * ct[60] * t558 * 3.1415926535897931 / 4.0)
    + ct[194];
  t613 = t566 * ct[169];
  t785 = ((t613 * ct[196] * ct[225] * ct[38] * t427 + ct[166]) - t564_tmp * ct
          [179] * ct[196] * ct[225] * ct[61] * t559 * 3.1415926535897931 / 4.0)
    + t801 * ct[182] * 3.1415926535897931 / 4.0;
  t786 = ((t614 * ct[170] * ct[225] * ct[38] * t424 + ct[158]) - t564_tmp * ct
          [170] * ct[175] * ct[225] * ct[58] * t556 * 3.1415926535897931 / 4.0)
    + ct[188];
  t787 = ((t615 * ct[172] * ct[225] * ct[38] * t425 + ct[159]) - t564_tmp * ct
          [172] * ct[176] * ct[225] * ct[59] * t557 * 3.1415926535897931 / 4.0)
    + ct[189];
  t788 = ((t616 * ct[173] * ct[225] * ct[38] * t426 + ct[161]) - t564_tmp * ct
          [173] * ct[178] * ct[225] * ct[60] * t558 * 3.1415926535897931 / 4.0)
    + ct[190];
  t789 = ((t613 * ct[174] * ct[225] * ct[38] * t427 + ct[162]) - t564_tmp * ct
          [174] * ct[179] * ct[225] * ct[61] * t559 * 3.1415926535897931 / 4.0)
    + ct[191];
  t612 = t800 * ct[25] * ct[26] * ct[36] * ct[37] * ct[130];
  t564 = ct[25] * ct[36] * ct[37];
  t564_tmp = t612 * ct[175];
  t790 = (((t564 * ct[70] * ct[120] * ct[131] * ct[160] * ct[225] * t424 *
            3.1415926535897931 / 4.0 + t612 * ct[160] * ct[225] * ct[54] * t556 *
            3.1415926535897931 / 4.0) - t564_tmp * ct[225] * ct[58] * t556 *
           3.1415926535897931 / 4.0) + t795 * t560 * 3.1415926535897931 / 4.0) +
    ct[200];
  t611 = t799 * ct[25] * ct[26] * ct[36] * ct[37] * ct[130];
  t567 = t611 * ct[176];
  t791 = (((t564 * ct[76] * ct[120] * ct[131] * ct[167] * ct[225] * t425 *
            3.1415926535897931 / 4.0 + t611 * ct[167] * ct[225] * ct[55] * t557 *
            3.1415926535897931 / 4.0) - t567 * ct[225] * ct[59] * t557 *
           3.1415926535897931 / 4.0) + t793 * t561 * 3.1415926535897931 / 4.0) +
    ct[201];
  t610 = t798 * ct[25] * ct[26] * ct[36] * ct[37] * ct[130];
  t566 = t610 * ct[178];
  t426 = (((t564 * ct[78] * ct[120] * ct[131] * ct[168] * ct[225] * t426 *
            3.1415926535897931 / 4.0 + t610 * ct[168] * ct[225] * ct[56] * t558 *
            3.1415926535897931 / 4.0) - t566 * ct[225] * ct[60] * t558 *
           3.1415926535897931 / 4.0) + t438_tmp * t562 * 3.1415926535897931 /
          4.0) + ct[203];
  t609 = t797 * ct[25] * ct[26] * ct[36] * ct[37] * ct[130];
  t565 = t609 * ct[179];
  t793 = (((t564 * ct[80] * ct[120] * ct[131] * ct[169] * ct[225] * t427 *
            3.1415926535897931 / 4.0 + t609 * ct[169] * ct[225] * ct[57] * t559 *
            3.1415926535897931 / 4.0) - t565 * ct[225] * ct[61] * t559 *
           3.1415926535897931 / 4.0) + t439_tmp * t563 * 3.1415926535897931 /
          4.0) + ct[204];
  t564 = ((t720 + t721) + t722) + t723;
  t615 = ((t716 + t717) + t718) + t719;
  t614 = ct[25] * ct[180];
  t425 = (((t614 * t436 + t614 * t569) + t564_tmp * ct[180] * ct[225] * ct[54] *
           t556 * 3.1415926535897931 / 4.0) - t456_tmp * t560 *
          3.1415926535897931 / 4.0) + ct[209];
  t614 = ct[25] * ct[181];
  t795 = (((t614 * t437 + t614 * t570) + t567 * ct[181] * ct[225] * ct[55] *
           t557 * 3.1415926535897931 / 4.0) - t457_tmp * t561 *
          3.1415926535897931 / 4.0) + ct[210];
  t614 = ct[25] * ct[187];
  t424 = (((t614 * t438 + t614 * t571) + t566 * ct[187] * ct[225] * ct[56] *
           t558 * 3.1415926535897931 / 4.0) - t458_tmp * t562 *
          3.1415926535897931 / 4.0) + ct[211];
  t614 = ct[25] * ct[196];
  t797 = (((t614 * t439 + t614 * t572) + t565 * ct[196] * ct[225] * ct[57] *
           t559 * 3.1415926535897931 / 4.0) - t459_tmp * t563 *
          3.1415926535897931 / 4.0) + ct[212];
  t614 = ct[25] * ct[170];
  t798 = (((t614 * t436 + t614 * t569) + t612 * ct[170] * ct[175] * ct[225] *
           ct[54] * t556 * 3.1415926535897931 / 4.0) - t452_tmp * t560 *
          3.1415926535897931 / 4.0) + ct[205];
  t614 = ct[25] * ct[172];
  t799 = (((t614 * t437 + t614 * t570) + t611 * ct[172] * ct[176] * ct[225] *
           ct[55] * t557 * 3.1415926535897931 / 4.0) - t453_tmp * t561 *
          3.1415926535897931 / 4.0) + ct[206];
  t614 = ct[25] * ct[173];
  t800 = (((t614 * t438 + t614 * t571) + t610 * ct[173] * ct[178] * ct[225] *
           ct[56] * t558 * 3.1415926535897931 / 4.0) - t454_tmp * t562 *
          3.1415926535897931 / 4.0) + ct[207];
  t613 = ct[25] * ct[174];
  t801 = (((t613 * t439 + t613 * t572) + t609 * ct[174] * ct[179] * ct[225] *
           ct[57] * t559 * 3.1415926535897931 / 4.0) - t455_tmp * t563 *
          3.1415926535897931 / 4.0) + ct[208];
  t613 = ((t702 + t703) + t704) + t705;
  t818 = ct[18] + -ct[199] * (((((((((ct[137] - ct[139]) + ct[28] * t702) - ct
    [28] * t703) - ct[29] * t704) + ct[29] * t705) - ct[31] * t720) - ct[31] *
    t721) + ct[30] * t722) + ct[30] * t723);
  t821 = ct[17] + ct[198] * ((((((((((ct[40] + ct[144]) + ct[66]) + ct[32] *
    t702) + ct[32] * t703) + ct[32] * t704) + ct[32] * t705) + ct[31] * t716) +
    ct[31] * t717) - ct[30] * t718) - ct[30] * t719);
  t455 = ct[14] + ct[222] * (((ct[67] + ct[97]) + ct[93] * t564) + ct[171] *
    t615);
  t456 = ct[16] + ct[197] * (((((((((((ct[41] + ct[149]) + ct[53]) + ct[177]) +
    ct[28] * t716) + ct[29] * t719) + ct[32] * t720) + ct[32] * t721) + ct[32] *
    t722) + ct[32] * t723) - ct[28] * t717) - ct[29] * t718);
  t457 = ct[138] * ct[171];
  t458 = ct[93] * ct[138];
  t453 = (ct[15] + ct[222] * (((((ct[68] + ct[98]) + ct[100]) + ct[195] * t613)
            + t457 * t564) - t458 * t615)) - 9.81;
  t459 = ct[171] * ct[195];
  t564_tmp = ct[93] * ct[195];
  t452 = ct[13] + ct[222] * (((((ct[69] + ct[96]) + ct[99]) - ct[138] * t613) +
    t459 * t564) - t564_tmp * t615);
  t454 = ct[12] * ct[221];
  t613 = ct[11] - t454;
  t614 = ct[7] + ct[46];
  t615 = ct[8] + ct[46];
  t616 = ct[9] + ct[46];
  t564 = ct[10] + ct[46];
  t565 = ct[19] + ct[48];
  t566 = ct[20] + ct[48];
  t567 = ct[21] + ct[48];
  t609 = ct[22] + ct[48];
  cost = (((((ct[27] * ((((((((((((ct[119] * (t613 * t613) + ct[105] * (t614 *
    t614)) + ct[105] * (t615 * t615)) + ct[105] * (t616 * t616)) + ct[105] *
    (t564 * t564)) + ct[112] * (t565 * t565)) + ct[112] * (t566 * t566)) + ct
    [112] * (t567 * t567)) + ct[112] * (t609 * t609)) + ct[88] * (t774 * t774))
    + ct[88] * (t775 * t775)) + ct[88] * (t776 * t776)) + ct[88] * (t777 * t777))
              + ct[104] * (t818 * t818)) + ct[95] * (t455 * t455)) + ct[103] *
            (t821 * t821)) + ct[102] * (t456 * t456)) + ct[94] * (t452 * t452))
    + ct[101] * (t453 * t453);
  *gradient_size = 13;
  t609 = ct[102] * ct[197] * t456;
  t610 = ct[5] * ct[37] * ct[82] * ct[226];
  t611 = ct[104] * ct[199] * t818;
  t612 = ct[103] * ct[198] * t821;
  t567 = ct[95] * ct[222] * t455;
  t566 = ct[94] * ct[222] * t452;
  t616 = ct[101] * ct[222] * t453;
  t564 = ct[27] * ct[88];
  t565 = ct[34] * ct[123];
  gradient_data[0] = (((((t609 * ((ct[28] * t786 + ct[32] * t782) + t610 *
    (((((ct[160] * 0.01228861457458129 + ct[1] * ct[160] * 0.015090129574632989)
        - ct[87] * ct[160] * 0.02051939481844044) + ct[45] * ct[175] *
       0.0046438440894603552) - ct[42] * ct[179] * 0.0039345373862287912) + ct
     [45] * ct[179] * -0.0064371741699132748) / 2.0) * 2.0 - t611 * (ct[28] *
    t778 - ct[31] * t782) * 2.0) + t612 * (ct[31] * t786 + ct[32] * t778) * 2.0)
                        + t567 * (ct[93] * t782 + ct[171] * t786) * 2.0) - t566 *
                       ((ct[138] * t778 - t459 * t782) + t564_tmp * t786) * 2.0)
                      + t616 * ((ct[195] * t778 + t457 * t782) - t458 * t786) *
                      2.0) - t564 * t774 * (t774_tmp * ct[52] *
    3.1415926535897931 * (((-ct[133] * ct[150] + ct[106] * (ct[33] * ct[133] *
    6.0 - ct[35] * ct[133] * 6.0)) + t565 * ct[133] * 3.0) + ct[82] * ct[213] *
    ct[220] * ct[224] * ct[225] * ct[50] * ct[62] * 6.0) / 2.0 - t436_tmp_tmp *
    ct[132] * ct[52] * t766 * 3.1415926535897931 * 1.5) * 2.0;
  gradient_data[1] = (((((t609 * ((ct[28] * t787 - ct[32] * t783) + t610 *
    (((((ct[167] * 0.01228861457458129 + ct[2] * ct[167] * 0.015090129574632989)
        - ct[87] * ct[167] * 0.02051939481844044) + ct[44] * ct[176] *
       0.0046438440894603552) - ct[43] * ct[178] * 0.0039345373862287912) + ct
     [44] * ct[178] * -0.0064371741699132748) / 2.0) * -2.0 + t611 * (ct[28] *
    t779 + ct[31] * t783) * 2.0) + t612 * (ct[31] * t787 + ct[32] * t779) * 2.0)
                        + t567 * (ct[93] * t783 + ct[171] * t787) * 2.0) - t566 *
                       ((ct[138] * t779 - t459 * t783) + t564_tmp * t787) * 2.0)
                      + t616 * ((ct[195] * t779 + t457 * t783) - t458 * t787) *
                      2.0) - t564 * t775 * (t775_tmp * ct[52] *
    3.1415926535897931 * (((-ct[134] * ct[151] + ct[107] * (ct[33] * ct[134] *
    6.0 - ct[35] * ct[134] * 6.0)) + t565 * ct[134] * 3.0) + ct[82] * ct[215] *
    ct[220] * ct[224] * ct[225] * ct[50] * ct[63] * 6.0) / 2.0 - t437_tmp_tmp *
    ct[132] * ct[52] * t767 * 3.1415926535897931 * 1.5) * 2.0;
  gradient_data[2] = (((((t611 * (ct[29] * t780 - ct[30] * t784) * 2.0 - t612 *
    (ct[30] * t788 - ct[32] * t780) * 2.0) - t609 * (ct[29] * t788 - ct[32] *
    t784) * 2.0) + t567 * (ct[93] * t784 + ct[171] * t788) * 2.0) - t566 * ((ct
    [138] * t780 - t459 * t784) + t564_tmp * t788) * 2.0) + t616 * ((ct[195] *
    t780 + t457 * t784) - t458 * t788) * 2.0) - t564 * t776 * (t776_tmp * ct[52]
    * 3.1415926535897931 * (((-ct[135] * ct[152] + ct[108] * (ct[33] * ct[135] *
    6.0 - ct[35] * ct[135] * 6.0)) + t565 * ct[135] * 3.0) + ct[82] * ct[217] *
    ct[220] * ct[224] * ct[225] * ct[50] * ct[64] * 6.0) / 2.0 - t438_tmp_tmp *
    ct[132] * ct[52] * t768 * 3.1415926535897931 * 1.5) * 2.0;
  gradient_data[3] = (((((t611 * (ct[29] * t781 + ct[30] * t785) * -2.0 - t612 *
    (ct[30] * t789 - ct[32] * t781) * 2.0) + t609 * (ct[29] * t789 + ct[32] *
    t785) * 2.0) + t567 * (ct[93] * t785 + ct[171] * t789) * 2.0) - t566 * ((ct
    [138] * t781 - t459 * t785) + t564_tmp * t789) * 2.0) + t616 * ((ct[195] *
    t781 + t457 * t785) - t458 * t789) * 2.0) - t564 * t777 * (t777_tmp * ct[52]
    * 3.1415926535897931 * (((-ct[136] * ct[153] + ct[109] * (ct[33] * ct[136] *
    6.0 - ct[35] * ct[136] * 6.0)) + t565 * ct[136] * 3.0) + ct[82] * ct[219] *
    ct[220] * ct[224] * ct[225] * ct[50] * ct[65] * 6.0) / 2.0 - t439_tmp_tmp *
    ct[132] * ct[52] * t769 * 3.1415926535897931 * 1.5) * 2.0;
  t564 = ct[1] * ct[25];
  t614 = t564 * ct[160];
  t615 = ct[25] * ct[82];
  gradient_data[4] = (((((ct[27] * (ct[105] * (ct[7] * 2.0 + ct[47]) +
    t774_tmp_tmp * ct[88] * ct[120] * ct[132] * ct[52] * t774 *
    3.1415926535897931 * (((ct[140] * ct[150] - ct[106] * (ct[33] * ct[140] *
    6.0 - ct[35] * ct[140] * 6.0)) - t565 * ct[140] * 3.0) + t615 * ct[202] *
    ct[220] * ct[224] * ct[225] * ct[50] * ct[54] * ct[58] * 6.0)) - t609 *
    ((ct[28] * t798 + ct[32] * t425) - t610 * ((((t564 * ct[175] *
    -0.01228861457458129 - ct[25] * ct[70] * ct[175] * 0.0075450647873164947) +
    t564 * ct[87] * ct[175] * 0.02051939481844044) + t614 * ct[45] *
    0.0046438440894603552) + t614 * ct[175] * ct[179] * 0.0078690747724575825) /
     2.0) * 2.0) - t611 * (ct[28] * t790 + ct[31] * t425) * 2.0) - t612 * (ct[31]
    * t798 - ct[32] * t790) * 2.0) - t567 * (ct[93] * t425 + ct[171] * t798) *
                       2.0) - t566 * ((ct[138] * t790 + t459 * t425) - t564_tmp *
    t798) * 2.0) + t616 * ((ct[195] * t790 - t457 * t425) + t458 * t798) * 2.0;
  t614 = ct[2] * ct[25];
  t613 = t614 * ct[167];
  gradient_data[5] = (((((ct[27] * (ct[105] * (ct[8] * 2.0 + ct[47]) +
    t775_tmp_tmp * ct[88] * ct[120] * ct[132] * ct[52] * t775 *
    3.1415926535897931 * (((ct[141] * ct[151] - ct[107] * (ct[33] * ct[141] *
    6.0 - ct[35] * ct[141] * 6.0)) - t565 * ct[141] * 3.0) + t615 * ct[214] *
    ct[220] * ct[224] * ct[225] * ct[50] * ct[55] * ct[59] * 6.0)) - t609 *
    ((-ct[28] * t799 + ct[32] * t795) + t610 * ((((t614 * ct[176] *
    -0.01228861457458129 - ct[25] * ct[76] * ct[176] * 0.0075450647873164947) +
    t614 * ct[87] * ct[176] * 0.02051939481844044) + t613 * ct[44] *
    0.0046438440894603552) + t613 * ct[176] * ct[178] * 0.0078690747724575825) /
     2.0) * 2.0) + t611 * (ct[28] * t791 - ct[31] * t795) * 2.0) - t612 * (ct[31]
    * t799 - ct[32] * t791) * 2.0) - t567 * (ct[93] * t795 + ct[171] * t799) *
                       2.0) - t566 * ((ct[138] * t791 + t459 * t795) - t564_tmp *
    t799) * 2.0) + t616 * ((ct[195] * t791 - t457 * t795) + t458 * t799) * 2.0;
  t613 = t614 * ct[168];
  gradient_data[6] = ((((ct[27] * (ct[105] * (ct[9] * 2.0 + ct[47]) +
    t776_tmp_tmp * ct[88] * ct[120] * ct[132] * ct[52] * t776 *
    3.1415926535897931 * (((ct[142] * ct[152] - ct[108] * (ct[33] * ct[142] *
    6.0 - ct[35] * ct[142] * 6.0)) - t565 * ct[142] * 3.0) + t615 * ct[216] *
    ct[220] * ct[224] * ct[225] * ct[50] * ct[56] * ct[60] * 6.0)) + t611 * (ct
    [29] * t426 + ct[30] * t424) * 2.0) + t612 * (ct[30] * t800 + ct[32] * t426)
                        * 2.0) - t567 * (ct[93] * t424 + ct[171] * t800) * 2.0)
                      + t609 * ((ct[29] * t800 - ct[32] * t424) + t610 * (((t614
    * rt_powd_snf(ct[168], 3.0) * 0.0064371741699132748 - t613 * (ct[178] * ct
    [178]) * 0.01287434833982655) + t613 * ct[43] * 0.0039345373862287912) +
    t613 * ct[176] * ct[178] * 0.00928768817892071) / 2.0) * 2.0) + (t566 *
    ((ct[138] * t426 + t459 * t424) - t564_tmp * t800) * -2.0 + t616 * ((ct[195]
    * t426 - t457 * t424) + t458 * t800) * 2.0);
  t614 = t564 * ct[169];
  gradient_data[7] = ((((ct[27] * (ct[105] * (ct[10] * 2.0 + ct[47]) +
    t777_tmp_tmp * ct[88] * ct[120] * ct[132] * ct[52] * t777 *
    3.1415926535897931 * (((ct[143] * ct[153] - ct[109] * (ct[33] * ct[143] *
    6.0 - ct[35] * ct[143] * 6.0)) - t565 * ct[143] * 3.0) + t615 * ct[218] *
    ct[220] * ct[224] * ct[225] * ct[50] * ct[57] * ct[61] * 6.0)) - t611 * (ct
    [29] * t793 - ct[30] * t797) * 2.0) + t612 * (ct[30] * t801 + ct[32] * t793)
                        * 2.0) - t567 * (ct[93] * t797 + ct[171] * t801) * 2.0)
                      + t609 * ((ct[29] * t801 + ct[32] * t797) + t610 * (((t564
    * rt_powd_snf(ct[169], 3.0) * 0.0064371741699132748 - t614 * (ct[179] * ct
    [179]) * 0.01287434833982655) + t614 * ct[42] * 0.0039345373862287912) +
    t614 * ct[175] * ct[179] * 0.00928768817892071) / 2.0) * -2.0) + (t566 *
    ((ct[138] * t793 + t459 * t797) - t564_tmp * t801) * -2.0 + t616 * ((ct[195]
    * t793 - t457 * t797) + t458 * t801) * 2.0);
  t610 = ct[27] * ct[112];
  t611 = ct[31] * ct[104] * ct[199];
  t612 = ct[31] * ct[103] * ct[198];
  gradient_data[8] = (((((t610 * (ct[19] * 2.0 + ct[49]) + t616 * (t457 * t732 +
    t458 * t744) * 2.0) + t566 * (t459 * t732 + t564_tmp * t744) * 2.0) - t609 *
                        (ct[28] * t744 - ct[32] * t732) * 2.0) + t567 * (ct[93] *
    t732 - ct[171] * t744) * 2.0) + t611 * t732 * t818 * 2.0) - t612 * t744 *
    t821 * 2.0;
  gradient_data[9] = (((((t610 * (ct[20] * 2.0 + ct[49]) + t616 * (t457 * t733 +
    t458 * t745) * 2.0) + t566 * (t459 * t733 + t564_tmp * t745) * 2.0) + t609 *
                        (ct[28] * t745 + ct[32] * t733) * 2.0) + t567 * (ct[93] *
    t733 - ct[171] * t745) * 2.0) + t611 * t733 * t818 * 2.0) - t612 * t745 *
    t821 * 2.0;
  t611 = ct[30] * ct[104] * ct[199];
  t612 = ct[30] * ct[103] * ct[198];
  gradient_data[10] = (((((t610 * (ct[21] * 2.0 + ct[49]) + t616 * (t457 * t734
    + t458 * t746) * 2.0) + t566 * (t459 * t734 + t564_tmp * t746) * 2.0) + t609
    * (ct[29] * t746 + ct[32] * t734) * 2.0) + t567 * (ct[93] * t734 - ct[171] *
    t746) * 2.0) - t611 * t734 * t818 * 2.0) + t612 * t746 * t821 * 2.0;
  gradient_data[11] = (((((t610 * (ct[22] * 2.0 + ct[49]) + t616 * (t457 * t735
    + t458 * t747) * 2.0) + t566 * (t459 * t735 + t564_tmp * t747) * 2.0) - t609
    * (ct[29] * t747 - ct[32] * t735) * 2.0) + t567 * (ct[93] * t735 - ct[171] *
    t747) * 2.0) - t611 * t735 * t818 * 2.0) + t612 * t747 * t821 * 2.0;
  gradient_data[12] = ct[27] * ct[119] * (ct[11] * 2.0 - t454 * 2.0) - ct[0] *
    ct[5] * ct[23] * ct[37] * ct[82] * ct[102] * ct[197] * t456;
  return cost;
}

/*
 * Arguments    : e_struct_T *obj
 *                int NColsRemain
 * Return Type  : void
 */
static void fullColLDL2_(e_struct_T *obj, int NColsRemain)
{
  int ijA;
  int j;
  int jA;
  int k;
  for (k = 0; k < NColsRemain; k++) {
    double alpha1;
    double y;
    int LD_diagOffset;
    int i;
    int offset1;
    int subMatrixDim;
    LD_diagOffset = 28 * k;
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    offset1 = LD_diagOffset + 2;
    y = obj->workspace_;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      y = obj->FMat[(LD_diagOffset + jA) + 1];
    }

    obj->workspace_ = y;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 29;
          i1 = subMatrixDim + jA;
          for (ijA = i; ijA <= i1 + 29; ijA++) {
            obj->FMat[ijA - 1] += y * temp;
          }
        }

        jA += 27;
      }
    }

    alpha1 = 1.0 / obj->FMat[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    for (jA = offset1; jA <= i + 2; jA++) {
      obj->FMat[jA - 1] *= alpha1;
    }
  }
}

/*
 * Arguments    : const double H[169]
 *                const double f[14]
 *                h_struct_T *solution
 *                f_struct_T *memspace
 *                i_struct_T *workingset
 *                d_struct_T *qrmanager
 *                e_struct_T *cholmanager
 *                struct_T *objective
 *                const char options_SolverName[7]
 *                double options_StepTolerance
 *                double options_ObjectiveLimit
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
static void iterate(const double H[169], const double f[14], h_struct_T
                    *solution, f_struct_T *memspace, i_struct_T *workingset,
                    d_struct_T *qrmanager, e_struct_T *cholmanager, struct_T
                    *objective, const char options_SolverName[7], double
                    options_StepTolerance, double options_ObjectiveLimit, int
                    runTimeOptions_MaxIterations)
{
  static const char b[7] = { 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  double s;
  double temp;
  int TYPE;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int i;
  int idx;
  int iyend;
  int nActiveConstr;
  int nVar;
  bool subProblemChanged;
  bool updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double, f,
    solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  memset(&solution->lambda[0], 0, 27U * sizeof(double));
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      double minLambda;
      int idxQR;
      int ix0;
      bool guard1;
      bool guard2;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          {
            double c;
            ix0 = 14 * (workingset->nActiveConstr - 1);
            iyend = qrmanager->mrows;
            idxQR = qrmanager->ncols + 1;
            if (iyend <= idxQR) {
              idxQR = iyend;
            }

            qrmanager->minRowCol = idxQR;
            idxQR = 27 * qrmanager->ncols;
            if (qrmanager->mrows != 0) {
              iyend = idxQR + qrmanager->mrows;
              if (idxQR + 1 <= iyend) {
                memset(&qrmanager->QR[idxQR], 0, (unsigned int)(iyend - idxQR) *
                       sizeof(double));
              }

              i = 27 * (qrmanager->mrows - 1) + 1;
              for (iyend = 1; iyend <= i; iyend += 27) {
                c = 0.0;
                nActiveConstr = (iyend + qrmanager->mrows) - 1;
                for (idx = iyend; idx <= nActiveConstr; idx++) {
                  c += qrmanager->Q[idx - 1] * workingset->ATwset[(ix0 + idx) -
                    iyend];
                }

                nActiveConstr = idxQR + div_nde_s32_floor(iyend - 1, 27);
                qrmanager->QR[nActiveConstr] += c;
              }
            }

            qrmanager->ncols++;
            i = qrmanager->ncols - 1;
            qrmanager->jpvt[i] = qrmanager->ncols;
            for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--)
            {
              nActiveConstr = idx + 27 * i;
              temp = qrmanager->QR[nActiveConstr + 1];
              c = xrotg(&qrmanager->QR[nActiveConstr], &temp, &s);
              qrmanager->QR[nActiveConstr + 1] = temp;
              iyend = 27 * idx;
              ix0 = qrmanager->mrows;
              if (qrmanager->mrows >= 1) {
                for (nActiveConstr = 0; nActiveConstr < ix0; nActiveConstr++) {
                  idxQR = iyend + nActiveConstr;
                  minLambda = qrmanager->Q[idxQR + 27];
                  temp = c * qrmanager->Q[idxQR] + s * minLambda;
                  qrmanager->Q[idxQR + 27] = c * minLambda - s * qrmanager->
                    Q[idxQR];
                  qrmanager->Q[idxQR] = temp;
                }
              }
            }
          }
          break;

         case -1:
          deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          factorQR(qrmanager, workingset->ATwset, nVar,
                   workingset->nActiveConstr);
          computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        iyend = memcmp(&options_SolverName[0], &b[0], 7);
        compute_deltax(H, solution, memspace, qrmanager, cholmanager, objective,
                       iyend == 0);
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((b_xnrm2(nVar, solution->searchDir) < options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar)) {
          guard2 = true;
        } else {
          minLambda = feasibleratiotest(solution->xstar, solution->searchDir,
            workingset->nVar, workingset->lb, workingset->ub,
            workingset->indexLB, workingset->indexUB, workingset->sizes,
            workingset->isActiveIdx, workingset->isActiveConstr,
            workingset->nWConstr, TYPE == 5, &updateFval, &i, &iyend);
          if (updateFval) {
            switch (i) {
             case 3:
              workingset->nWConstr[2]++;
              workingset->isActiveConstr[(workingset->isActiveIdx[2] + iyend) -
                2] = true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] = iyend;

              /* A check that is always false is detected at compile-time. Eliminating code that follows. */
              break;

             case 4:
              addBoundToActiveSetMatrix_(workingset, 4, iyend);
              break;

             default:
              addBoundToActiveSetMatrix_(workingset, 5, iyend);
              break;
            }

            activeSetChangeID = 1;
          } else {
            if (objective->objtype == 5) {
              if (b_xnrm2(objective->nvar, solution->searchDir) > 100.0 *
                  (double)objective->nvar * 1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }

            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (!(minLambda == 0.0))) {
            iyend = nVar - 1;
            for (nActiveConstr = 0; nActiveConstr <= iyend; nActiveConstr++) {
              solution->xstar[nActiveConstr] += minLambda * solution->
                searchDir[nActiveConstr];
            }
          }

          computeGrad_StoreHx(objective, H, f, solution->xstar);
          updateFval = true;
          guard1 = true;
        }
      } else {
        i = (unsigned char)nVar;
        memset(&solution->searchDir[0], 0, (unsigned int)i * sizeof(double));
        guard2 = true;
      }

      if (guard2) {
        nActiveConstr = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          bool b_guard1;
          b_guard1 = false;
          if (objective->objtype != 4) {
            minLambda = 100.0 * (double)qrmanager->mrows *
              2.2204460492503131E-16;
            if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
              updateFval = true;
            } else {
              updateFval = false;
            }

            if (updateFval) {
              bool b_guard2;
              idx = qrmanager->ncols;
              b_guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                idxQR = qrmanager->mrows + 27 * (qrmanager->ncols - 1);
                while ((idx > qrmanager->mrows) && (fabs(qrmanager->QR[idxQR - 1])
                        >= minLambda)) {
                  idx--;
                  idxQR -= 27;
                }

                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  b_guard2 = true;
                }
              } else {
                b_guard2 = true;
              }

              if (b_guard2) {
                idxQR = idx + 27 * (idx - 1);
                while ((idx >= 1) && (fabs(qrmanager->QR[idxQR - 1]) >=
                                      minLambda)) {
                  idx--;
                  idxQR -= 28;
                }

                updateFval = (idx == 0);
              }
            }

            if (!updateFval) {
              solution->state = -7;
            } else {
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }

          if (b_guard1) {
            ix0 = qrmanager->ncols;
            b_xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q,
                    objective->grad, memspace->workspace_double);
            if (qrmanager->ncols != 0) {
              for (idx = ix0; idx >= 1; idx--) {
                iyend = (idx + (idx - 1) * 27) - 1;
                memspace->workspace_double[idx - 1] /= qrmanager->QR[iyend];
                for (i = 0; i <= idx - 2; i++) {
                  idxQR = (idx - i) - 2;
                  memspace->workspace_double[idxQR] -=
                    memspace->workspace_double[idx - 1] * qrmanager->QR[(iyend -
                    i) - 1];
                }
              }
            }

            for (idx = 0; idx < nActiveConstr; idx++) {
              solution->lambda[idx] = -memspace->workspace_double[idx];
            }
          }
        }

        if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
          iyend = 0;
          minLambda = 0.0;
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          nActiveConstr = workingset->nActiveConstr;
          for (idx = i; idx <= nActiveConstr; idx++) {
            temp = solution->lambda[idx - 1];
            if (temp < minLambda) {
              minLambda = temp;
              iyend = idx;
            }
          }

          if (iyend == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iyend;
            subProblemChanged = true;
            removeConstr(workingset, iyend);
            solution->lambda[iyend - 1] = 0.0;
          }
        } else {
          iyend = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda[iyend - 1] = 0.0;
        }

        updateFval = false;
        guard1 = true;
      }

      if (guard1) {
        solution->iterations++;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = b_maxConstraintViolation(workingset,
            solution->xstar);
          minLambda = solution->maxConstr;
          if (objective->objtype == 5) {
            minLambda = solution->maxConstr - solution->xstar[objective->nvar -
              1];
          }

          if (minLambda > 1.0E-6) {
            bool nonDegenerateWset;
            i = (unsigned char)objective->nvar;
            if (i - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (unsigned int)
                     i * sizeof(double));
            }

            nonDegenerateWset = feasibleX0ForWorkingSet
              (memspace->workspace_double, solution->searchDir, workingset,
               qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }

            activeSetChangeID = 0;
            minLambda = b_maxConstraintViolation(workingset, solution->searchDir);
            if (minLambda < solution->maxConstr) {
              if (i - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (unsigned
                        int)i * sizeof(double));
              }

              solution->maxConstr = minLambda;
            }
          }
        }

        if (updateFval && (options_ObjectiveLimit > rtMinusInf)) {
          solution->fstar = computeFval_ReuseHx(objective,
            memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) && ((solution->state !=
                0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = computeFval_ReuseHx(objective,
          memspace->workspace_double, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/*
 * Arguments    : bool obj_hasLinear
 *                int obj_nvar
 *                double workspace[378]
 *                const double H[169]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : void
 */
static void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[378],
  const double H[169], const double f[14], const double x[14])
{
  int beta1;
  int ia;
  int iac;
  beta1 = 0;
  if (obj_hasLinear) {
    beta1 = (unsigned char)obj_nvar;
    if (beta1 - 1 >= 0) {
      memcpy(&workspace[0], &f[0], (unsigned int)beta1 * sizeof(double));
    }

    beta1 = 1;
  }

  if (obj_nvar != 0) {
    int ix;
    if (beta1 != 1) {
      beta1 = (unsigned char)obj_nvar;
      memset(&workspace[0], 0, (unsigned int)beta1 * sizeof(double));
    }

    ix = 0;
    beta1 = obj_nvar * (obj_nvar - 1) + 1;
    for (iac = 1; obj_nvar < 0 ? iac >= beta1 : iac <= beta1; iac += obj_nvar) {
      double c;
      int i;
      c = 0.5 * x[ix];
      i = (iac + obj_nvar) - 1;
      for (ia = iac; ia <= i; ia++) {
        int i1;
        i1 = ia - iac;
        workspace[i1] += H[ia - 1] * c;
      }

      ix++;
    }
  }
}

/*
 * Arguments    : const i_struct_T *obj
 *                const double x[378]
 *                int ix0
 * Return Type  : double
 */
static double maxConstraintViolation(const i_struct_T *obj, const double x[378],
  int ix0)
{
  double v;
  int i;
  int idx;
  int idxLB;
  v = 0.0;
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB[idx] - 1;
      v = fmax(v, -x[(ix0 + idxLB) - 1] - obj->lb[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    i = (unsigned char)obj->sizes[4];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexUB[idx] - 1;
      v = fmax(v, x[(ix0 + idxLB) - 1] - obj->ub[idxLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    i = (unsigned char)obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      v = fmax(v, fabs(x[(ix0 + obj->indexFixed[idx]) - 2] - obj->ub
                       [obj->indexFixed[idx] - 1]));
    }
  }

  return v;
}

/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
static double maximum(const double x[2])
{
  double ex;
  if ((x[0] < x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
static double minimum(const double x[2])
{
  double ex;
  if ((x[0] > x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

/*
 * Arguments    : double A[729]
 *                int m
 *                int n
 *                int nfxd
 *                double tau[27]
 * Return Type  : void
 */
static void qrf(double A[729], int m, int n, int nfxd, double tau[27])
{
  double work[27];
  double atmp;
  int b_i;
  int i;
  memset(&tau[0], 0, 27U * sizeof(double));
  memset(&work[0], 0, 27U * sizeof(double));
  i = (unsigned char)nfxd;
  for (b_i = 0; b_i < i; b_i++) {
    double d;
    int ii;
    int mmi;
    ii = b_i * 27 + b_i;
    mmi = m - b_i;
    if (b_i + 1 < m) {
      atmp = A[ii];
      d = xzlarfg(mmi, &atmp, A, ii + 2);
      tau[b_i] = d;
      A[ii] = atmp;
    } else {
      d = 0.0;
      tau[b_i] = 0.0;
    }

    if (b_i + 1 < n) {
      atmp = A[ii];
      A[ii] = 1.0;
      xzlarf(mmi, (n - b_i) - 1, ii + 1, d, A, ii + 28, work);
      A[ii] = atmp;
    }
  }
}

/*
 * Arguments    : i_struct_T *obj
 *                int idx_global
 * Return Type  : void
 */
static void removeConstr(i_struct_T *obj, int idx_global)
{
  int TYPE_tmp;
  int i;
  int i1;
  int idx;
  TYPE_tmp = obj->Wid[idx_global - 1] - 1;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE_tmp] + obj->Wlocalidx[idx_global -
                       1]) - 2] = false;
  i = obj->nActiveConstr - 1;
  obj->Wid[idx_global - 1] = obj->Wid[i];
  obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[i];
  i1 = (unsigned char)obj->nVar;
  for (idx = 0; idx < i1; idx++) {
    obj->ATwset[idx + 14 * (idx_global - 1)] = obj->ATwset[idx + 14 * i];
  }

  obj->bwset[idx_global - 1] = obj->bwset[i];
  obj->nActiveConstr = i;
  obj->nWConstr[TYPE_tmp]--;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double b;
  double y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = rtNaN;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : i_struct_T *obj
 *                int PROBLEM_TYPE
 * Return Type  : void
 */
static void setProblemType(i_struct_T *obj, int PROBLEM_TYPE)
{
  int i;
  int idx;
  int idx_col;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 13;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = (unsigned char)obj->sizesNormal[4];
      for (idx = 0; idx < i; idx++) {
        obj->isActiveConstr[(obj->isActiveIdxNormal[4] + idx) - 1] =
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
      }
    }

    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }

    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
    break;

   case 1:
    {
      int idxStartIneq;
      obj->nVar = 14;
      obj->mConstr = obj->mConstrOrig + 1;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
      }

      i = (unsigned char)obj->sizes[0];
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[14 * idx + 13] = 0.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = 14;
      obj->lb[13] = 1.0E-5;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx = idxStartIneq; idx <= i; idx++) {
        obj->ATwset[14 * (idx - 1) + 13] = -1.0;
      }

      if (obj->nWConstr[4] > 0) {
        i = (unsigned char)(obj->sizesNormal[4] + 1);
        for (idx = 0; idx < i; idx++) {
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
        }
      }

      obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
    }
    break;

   case 2:
    {
      obj->nVar = 13;
      obj->mConstr = 26;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegularized[i];
      }

      if (obj->probType != 4) {
        int i1;
        int idxStartIneq;
        int idx_lb;
        idx_lb = 13;
        i = obj->sizesNormal[3] + 1;
        i1 = obj->sizesRegularized[3];
        for (idx = i; idx <= i1; idx++) {
          idx_lb++;
          obj->indexLB[idx - 1] = idx_lb;
        }

        if (obj->nWConstr[4] > 0) {
          i = (unsigned char)obj->sizesRegularized[4];
          for (idx = 0; idx < i; idx++) {
            obj->isActiveConstr[obj->isActiveIdxRegularized[4] + idx] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
          }
        }

        i = obj->isActiveIdx[4];
        i1 = obj->isActiveIdxRegularized[4] - 1;
        if (i <= i1) {
          memset(&obj->isActiveConstr[i + -1], 0, (unsigned int)((i1 - i) + 1) *
                 sizeof(bool));
        }

        idxStartIneq = obj->isActiveIdx[2];
        i = obj->nActiveConstr;
        for (idx_col = idxStartIneq; idx_col <= i; idx_col++) {
          idx_lb = 14 * (idx_col - 1) - 1;
          if (obj->Wid[idx_col - 1] == 3) {
            i1 = obj->Wlocalidx[idx_col - 1];
            idx = i1 + 12;
            if (idx >= 14) {
              memset(&obj->ATwset[idx_lb + 14], 0, (unsigned int)(((idx + idx_lb)
                       - idx_lb) - 13) * sizeof(double));
            }

            obj->ATwset[(i1 + idx_lb) + 13] = -1.0;
            i1 += 14;
            if (i1 <= 13) {
              memset(&obj->ATwset[i1 + idx_lb], 0, (unsigned int)(((idx_lb - i1)
                       - idx_lb) + 14) * sizeof(double));
            }
          }
        }
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
      }
    }
    break;

   default:
    {
      int idxStartIneq;
      obj->nVar = 14;
      obj->mConstr = 27;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
      }

      i = (unsigned char)obj->sizes[0];
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[14 * idx + 13] = 0.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = 14;
      obj->lb[13] = 1.0E-5;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx = idxStartIneq; idx <= i; idx++) {
        obj->ATwset[14 * (idx - 1) + 13] = -1.0;
      }

      if (obj->nWConstr[4] > 0) {
        i = (unsigned char)(obj->sizesNormal[4] + 1);
        for (idx = 0; idx < i; idx++) {
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
        }
      }

      obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

/*
 * Arguments    : const e_struct_T *obj
 *                double rhs[14]
 * Return Type  : void
 */
static void solve(const e_struct_T *obj, double rhs[14])
{
  int i;
  int j;
  int jA;
  int n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    for (j = 0; j < n_tmp; j++) {
      double temp;
      jA = j * 27;
      temp = rhs[j];
      for (i = 0; i < j; i++) {
        temp -= obj->FMat[jA + i] * rhs[i];
      }

      rhs[j] = temp / obj->FMat[jA + j];
    }
  }

  if (obj->ndims != 0) {
    for (j = n_tmp; j >= 1; j--) {
      jA = (j + (j - 1) * 27) - 1;
      rhs[j - 1] /= obj->FMat[jA];
      for (i = 0; i <= j - 2; i++) {
        int ix;
        ix = (j - i) - 2;
        rhs[ix] -= rhs[j - 1] * obj->FMat[(jA - i) - 1];
      }
    }
  }
}

/*
 * Arguments    : double lambda[27]
 *                int WorkingSet_nActiveConstr
 *                const int WorkingSet_sizes[5]
 *                const int WorkingSet_isActiveIdx[6]
 *                const int WorkingSet_Wid[27]
 *                const int WorkingSet_Wlocalidx[27]
 *                double workspace[378]
 * Return Type  : void
 */
static void sortLambdaQP(double lambda[27], int WorkingSet_nActiveConstr, const
  int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[27], const int WorkingSet_Wlocalidx[27], double workspace[378])
{
  if (WorkingSet_nActiveConstr != 0) {
    int idx;
    int idxOffset;
    int mAll;
    mAll = (WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4];
    idx = (unsigned char)mAll;
    if (idx - 1 >= 0) {
      memcpy(&workspace[0], &lambda[0], (unsigned int)idx * sizeof(double));
    }

    if (mAll - 1 >= 0) {
      memset(&lambda[0], 0, (unsigned int)mAll * sizeof(double));
    }

    mAll = 0;
    idx = 0;
    while ((idx + 1 <= WorkingSet_nActiveConstr) && (WorkingSet_Wid[idx] <= 2))
    {
      if (WorkingSet_Wid[idx] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }

      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }

    while (idx + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid[idx]) {
       case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;

       case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;

       default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }

      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }
  }
}

/*
 * Arguments    : int *STEP_TYPE
 *                double Hessian[169]
 *                const double lb[13]
 *                const double ub[13]
 *                h_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                f_struct_T *memspace
 *                i_struct_T *WorkingSet
 *                d_struct_T *QRManager
 *                e_struct_T *CholManager
 *                struct_T *QPObjective
 *                j_struct_T *qpoptions
 * Return Type  : bool
 */
static bool step(int *STEP_TYPE, double Hessian[169], const double lb[13], const
                 double ub[13], h_struct_T *TrialState, b_struct_T
                 *MeritFunction, f_struct_T *memspace, i_struct_T *WorkingSet,
                 d_struct_T *QRManager, e_struct_T *CholManager, struct_T
                 *QPObjective, j_struct_T *qpoptions)
{
  j_struct_T b_qpoptions;
  double dv[14];
  double oldDirIdx;
  double s;
  int idxEndIneq_tmp_tmp;
  int idxStartIneq;
  int k;
  int nVar_tmp_tmp;
  bool checkBoundViolation;
  bool stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet->nVar;
  if (*STEP_TYPE != 3) {
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->nVar;
    memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (unsigned int)
           idxEndIneq_tmp_tmp * sizeof(double));
  } else {
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->nVar;
    if (idxEndIneq_tmp_tmp - 1 >= 0) {
      memcpy(&TrialState->searchDir[0], &TrialState->xstar[0], (unsigned int)
             idxEndIneq_tmp_tmp * sizeof(double));
    }
  }

  int exitg1;
  bool guard1;
  do {
    int temp;
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
     case 1:
      b_qpoptions = *qpoptions;
      driver(Hessian, TrialState->grad, TrialState, memspace, WorkingSet,
             QRManager, CholManager, QPObjective, &b_qpoptions,
             qpoptions->MaxIterations);
      if (TrialState->state > 0) {
        MeritFunction->phi = TrialState->sqpFval;
        MeritFunction->linearizedConstrViol = 0.0;
        MeritFunction->penaltyParam = 1.0;
        MeritFunction->phiPrimePlus = fmin(TrialState->fstar, 0.0);
      }

      sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                   WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                   WorkingSet->Wlocalidx, memspace->workspace_double);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        if (idxEndIneq_tmp_tmp - 1 >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0], (unsigned int)
                 idxEndIneq_tmp_tmp * sizeof(double));
        }

        guard1 = true;
      }
      break;

     case 2:
      {
        double beta;
        temp = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
        idxStartIneq = temp + 1;
        idxEndIneq_tmp_tmp = WorkingSet->nActiveConstr;
        for (k = idxStartIneq; k <= idxEndIneq_tmp_tmp; k++) {
          WorkingSet->isActiveConstr[(WorkingSet->isActiveIdx[WorkingSet->Wid[k
            - 1] - 1] + WorkingSet->Wlocalidx[k - 1]) - 2] = false;
        }

        WorkingSet->nWConstr[2] = 0;
        WorkingSet->nWConstr[3] = 0;
        WorkingSet->nWConstr[4] = 0;
        WorkingSet->nActiveConstr = temp;
        memcpy(&dv[0], &TrialState->xstar[0], 14U * sizeof(double));
        idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[3];
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          s = WorkingSet->lb[WorkingSet->indexLB[k] - 1];
          if (-dv[WorkingSet->indexLB[k] - 1] > s) {
            if (rtIsInf(ub[WorkingSet->indexLB[k] - 1])) {
              dv[WorkingSet->indexLB[k] - 1] = -s + fabs(s);
            } else {
              dv[WorkingSet->indexLB[k] - 1] = (WorkingSet->ub
                [WorkingSet->indexLB[k] - 1] - s) / 2.0;
            }
          }
        }

        idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          s = WorkingSet->ub[WorkingSet->indexUB[k] - 1];
          if (dv[WorkingSet->indexUB[k] - 1] > s) {
            if (rtIsInf(lb[WorkingSet->indexUB[k] - 1])) {
              dv[WorkingSet->indexUB[k] - 1] = s - fabs(s);
            } else {
              dv[WorkingSet->indexUB[k] - 1] = (s - WorkingSet->lb
                [WorkingSet->indexUB[k] - 1]) / 2.0;
            }
          }
        }

        memcpy(&TrialState->xstar[0], &dv[0], 14U * sizeof(double));
        idxEndIneq_tmp_tmp = WorkingSet->nVar;
        beta = 0.0;
        idxStartIneq = (unsigned char)WorkingSet->nVar;
        for (k = 0; k < idxStartIneq; k++) {
          beta += Hessian[k + 13 * k];
        }

        beta /= (double)WorkingSet->nVar;
        if (TrialState->sqpIterations <= 1) {
          temp = QPObjective->nvar;
          if (QPObjective->nvar < 1) {
            idxStartIneq = 0;
          } else {
            idxStartIneq = 1;
            if (QPObjective->nvar > 1) {
              oldDirIdx = fabs(TrialState->grad[0]);
              for (k = 2; k <= temp; k++) {
                s = fabs(TrialState->grad[k - 1]);
                if (s > oldDirIdx) {
                  idxStartIneq = k;
                  oldDirIdx = s;
                }
              }
            }
          }

          s = 100.0 * fmax(1.0, fabs(TrialState->grad[idxStartIneq - 1]));
        } else {
          temp = WorkingSet->mConstr;
          if (WorkingSet->mConstr < 1) {
            idxStartIneq = 0;
          } else {
            idxStartIneq = 1;
            if (WorkingSet->mConstr > 1) {
              oldDirIdx = fabs(TrialState->lambdasqp[0]);
              for (k = 2; k <= temp; k++) {
                s = fabs(TrialState->lambdasqp[k - 1]);
                if (s > oldDirIdx) {
                  idxStartIneq = k;
                  oldDirIdx = s;
                }
              }
            }
          }

          s = fabs(TrialState->lambdasqp[idxStartIneq - 1]);
        }

        QPObjective->nvar = WorkingSet->nVar;
        QPObjective->beta = beta;
        QPObjective->rho = s;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 4;
        setProblemType(WorkingSet, 2);
        temp = qpoptions->MaxIterations;
        qpoptions->MaxIterations = (qpoptions->MaxIterations + WorkingSet->nVar)
          - idxEndIneq_tmp_tmp;
        memcpy(&dv[0], &TrialState->grad[0], 14U * sizeof(double));
        b_qpoptions = *qpoptions;
        driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
               CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
        qpoptions->MaxIterations = temp;
        if (TrialState->state != -6) {
          MeritFunction->phi = TrialState->sqpFval;
          MeritFunction->linearizedConstrViol = 0.0;
          MeritFunction->penaltyParam = 1.0;
          MeritFunction->phiPrimePlus = fmin((TrialState->fstar - s * 0.0) -
            beta / 2.0 * 0.0, 0.0);
          temp = WorkingSet->isActiveIdx[2];
          idxStartIneq = WorkingSet->nActiveConstr;
          for (k = temp; k <= idxStartIneq; k++) {
            if (WorkingSet->Wid[k - 1] == 3) {
              TrialState->lambda[k - 1] *= (double)memspace->
                workspace_int[WorkingSet->Wlocalidx[k - 1] - 1];
            }
          }
        }

        QPObjective->nvar = idxEndIneq_tmp_tmp;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 3;
        setProblemType(WorkingSet, 3);
        sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                     WorkingSet->Wlocalidx, memspace->workspace_double);
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        if (idxEndIneq_tmp_tmp - 1 >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0], (unsigned int)
                 idxEndIneq_tmp_tmp * sizeof(double));
        }

        guard1 = true;
      }
      break;

     default:
      idxEndIneq_tmp_tmp = WorkingSet->nVar;
      idxStartIneq = (unsigned char)WorkingSet->nVar;
      for (k = 0; k < idxStartIneq; k++) {
        TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k];
        TrialState->socDirection[k] = TrialState->xstar[k];
      }

      memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0], 27U *
             sizeof(double));
      memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (unsigned int)
             idxStartIneq * sizeof(double));
      memcpy(&dv[0], &TrialState->grad[0], 14U * sizeof(double));
      b_qpoptions = *qpoptions;
      driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
             CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
      idxStartIneq = (unsigned char)idxEndIneq_tmp_tmp;
      for (k = 0; k < idxStartIneq; k++) {
        s = TrialState->socDirection[k];
        oldDirIdx = s;
        s = TrialState->xstar[k] - s;
        TrialState->socDirection[k] = s;
        TrialState->xstar[k] = oldDirIdx;
      }

      stepSuccess = (b_xnrm2(idxEndIneq_tmp_tmp, TrialState->socDirection) <=
                     2.0 * b_xnrm2(idxEndIneq_tmp_tmp, TrialState->xstar));
      if (!stepSuccess) {
        memcpy(&TrialState->lambda[0], &TrialState->lambdaStopTest[0], 27U *
               sizeof(double));
      } else {
        sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                     WorkingSet->Wlocalidx, memspace->workspace_double);
      }

      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          TrialState->delta_x[k] = TrialState->xstar[k] +
            TrialState->socDirection[k];
        }
      }

      guard1 = true;
      break;
    }

    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        oldDirIdx = 0.0;
        s = 1.0;
        for (k = 0; k < 13; k++) {
          oldDirIdx = fmax(oldDirIdx, fabs(TrialState->grad[k]));
          s = fmax(s, fabs(TrialState->xstar[k]));
        }

        s = fmax(2.2204460492503131E-16, oldDirIdx / s);
        for (idxStartIneq = 0; idxStartIneq < 13; idxStartIneq++) {
          temp = 13 * idxStartIneq;
          for (k = 0; k < idxStartIneq; k++) {
            Hessian[temp + k] = 0.0;
          }

          temp = idxStartIneq + 13 * idxStartIneq;
          Hessian[temp] = s;
          idxEndIneq_tmp_tmp = 11 - idxStartIneq;
          if (idxEndIneq_tmp_tmp >= 0) {
            memset(&Hessian[temp + 1], 0, (unsigned int)(((idxEndIneq_tmp_tmp +
                      temp) - temp) + 1) * sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    memcpy(&dv[0], &TrialState->delta_x[0], 14U * sizeof(double));
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[3];
    for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet->indexLB[k] - 1];
      s = (TrialState->xstarsqp[WorkingSet->indexLB[k] - 1] + oldDirIdx) -
        lb[WorkingSet->indexLB[k] - 1];
      if (s < 0.0) {
        dv[WorkingSet->indexLB[k] - 1] = oldDirIdx - s;
        TrialState->xstar[WorkingSet->indexLB[k] - 1] -= s;
      }
    }

    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
    for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet->indexUB[k] - 1];
      s = (ub[WorkingSet->indexUB[k] - 1] - TrialState->xstarsqp
           [WorkingSet->indexUB[k] - 1]) - oldDirIdx;
      if (s < 0.0) {
        dv[WorkingSet->indexUB[k] - 1] = oldDirIdx + s;
        TrialState->xstar[WorkingSet->indexUB[k] - 1] += s;
      }
    }

    memcpy(&TrialState->delta_x[0], &dv[0], 14U * sizeof(double));
  }

  return stepSuccess;
}

/*
 * Arguments    : b_struct_T *MeritFunction
 *                const i_struct_T *WorkingSet
 *                h_struct_T *TrialState
 *                const double lb[13]
 *                const double ub[13]
 *                bool *Flags_fevalOK
 *                bool *Flags_done
 *                bool *Flags_stepAccepted
 *                bool *Flags_failedLineSearch
 *                int *Flags_stepType
 * Return Type  : bool
 */
static bool test_exit(b_struct_T *MeritFunction, const i_struct_T *WorkingSet,
                      h_struct_T *TrialState, const double lb[13], const double
                      ub[13], bool *Flags_fevalOK, bool *Flags_done, bool
                      *Flags_stepAccepted, bool *Flags_failedLineSearch, int
                      *Flags_stepType)
{
  double s;
  double smax;
  int i;
  int idx_max;
  int k;
  int nVar;
  bool Flags_gradOK;
  bool exitg1;
  bool isFeasible;
  *Flags_fevalOK = true;
  *Flags_done = false;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  i = (unsigned char)((WorkingSet->sizes[0] + WorkingSet->sizes[3]) +
                      WorkingSet->sizes[4]);
  if (i - 1 >= 0) {
    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambdasqp[0], (unsigned
            int)i * sizeof(double));
  }

  computeGradLag(TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdaStopTest);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  s = fmax(1.0, fabs(TrialState->grad[idx_max - 1]));
  if (rtIsInf(s)) {
    s = 1.0;
  }

  smax = 0.0;
  idx_max = (unsigned char)WorkingSet->sizes[3];
  for (k = 0; k < idx_max; k++) {
    nVar = WorkingSet->indexLB[k] - 1;
    smax = fmax(smax, lb[nVar] - TrialState->xstarsqp[nVar]);
  }

  idx_max = (unsigned char)WorkingSet->sizes[4];
  for (k = 0; k < idx_max; k++) {
    nVar = WorkingSet->indexUB[k] - 1;
    smax = fmax(smax, TrialState->xstarsqp[nVar] - ub[nVar]);
  }

  MeritFunction->nlpPrimalFeasError = smax;
  MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  Flags_gradOK = true;
  smax = 0.0;
  idx_max = (unsigned char)WorkingSet->nVar;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= idx_max - 1)) {
    Flags_gradOK = ((!rtIsInf(TrialState->gradLag[k])) && (!rtIsNaN
      (TrialState->gradLag[k])));
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = fmax(smax, fabs(TrialState->gradLag[k]));
      k++;
    }
  }

  MeritFunction->nlpDualFeasError = smax;
  if (!Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = smax;
    if (i - 1 >= 0) {
      memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
             (unsigned int)i * sizeof(double));
    }

    if (isFeasible && (smax <= 1.0E-12 * s)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }

  return Flags_gradOK;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void tic(void)
{
  coderTimespec b_timespec;
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  timeKeeper(b_timespec.tv_sec, b_timespec.tv_nsec);
}

/*
 * Arguments    : double newTime_tv_sec
 *                double newTime_tv_nsec
 * Return Type  : void
 */
static void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec)
{
  if (!savedTime_not_empty) {
    coderTimespec b_timespec;
    if (!freq_not_empty) {
      freq_not_empty = true;
      coderInitTimeFunctions(&freq);
    }

    coderTimeClockGettimeMonotonic(&b_timespec, freq);
    savedTime_not_empty = true;
  }

  savedTime.tv_sec = newTime_tv_sec;
  savedTime.tv_nsec = newTime_tv_nsec;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void timeKeeper_init(void)
{
  savedTime_not_empty = false;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double toc(void)
{
  coderTimespec b_timespec;
  double tstart_tv_nsec;
  double tstart_tv_sec;
  tstart_tv_sec = b_timeKeeper(&tstart_tv_nsec);
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  return (b_timespec.tv_sec - tstart_tv_sec) + (b_timespec.tv_nsec -
    tstart_tv_nsec) / 1.0E+9;
}

/*
 * Arguments    : int m
 *                int n
 *                int k
 *                const double A[169]
 *                int lda
 *                const double B[729]
 *                int ib0
 *                double C[378]
 * Return Type  : void
 */
static void xgemm(int m, int n, int k, const double A[169], int lda, const
                  double B[729], int ib0, double C[378])
{
  int cr;
  int ib;
  int ic;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    br = ib0;
    lastColC = 27 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 27) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        memset(&C[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof(double));
      }
    }

    for (cr = 0; cr <= lastColC; cr += 27) {
      int ar;
      ar = -1;
      i = br + k;
      for (ib = br; ib < i; ib++) {
        int i2;
        i1 = cr + 1;
        i2 = cr + m;
        for (ic = i1; ic <= i2; ic++) {
          C[ic - 1] += B[ib - 1] * A[(ar + ic) - cr];
        }

        ar += lda;
      }

      br += 27;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[169]
 *                int lda
 *                const double x[14]
 *                double y[13]
 * Return Type  : void
 */
static void xgemv(int m, int n, const double A[169], int lda, const double x[14],
                  double y[13])
{
  int ia;
  int iac;
  if ((m != 0) && (n != 0)) {
    int i;
    int ix;
    i = (unsigned char)m;
    memset(&y[0], 0, (unsigned int)i * sizeof(double));
    ix = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int i1;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        y[i2] += A[ia - 1] * x[ix];
      }

      ix++;
    }
  }
}

/*
 * Arguments    : double A[729]
 *                int m
 *                int n
 *                int jpvt[27]
 *                double tau[27]
 * Return Type  : void
 */
static void xgeqp3(double A[729], int m, int n, int jpvt[27], double tau[27])
{
  double vn1[27];
  double vn2[27];
  double work[27];
  double temp;
  int b_i;
  int k;
  int minmn_tmp;
  int pvt;
  if (m <= n) {
    minmn_tmp = m;
  } else {
    minmn_tmp = n;
  }

  memset(&tau[0], 0, 27U * sizeof(double));
  if (minmn_tmp < 1) {
    for (pvt = 0; pvt < n; pvt++) {
      jpvt[pvt] = pvt + 1;
    }
  } else {
    int i;
    int ix;
    int iy;
    int nfxd;
    int temp_tmp;
    nfxd = 0;
    for (pvt = 0; pvt < n; pvt++) {
      if (jpvt[pvt] != 0) {
        nfxd++;
        if (pvt + 1 != nfxd) {
          ix = pvt * 27;
          iy = (nfxd - 1) * 27;
          for (k = 0; k < m; k++) {
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            i = iy + k;
            A[temp_tmp] = A[i];
            A[i] = temp;
          }

          jpvt[pvt] = jpvt[nfxd - 1];
          jpvt[nfxd - 1] = pvt + 1;
        } else {
          jpvt[pvt] = pvt + 1;
        }
      } else {
        jpvt[pvt] = pvt + 1;
      }
    }

    if (nfxd > minmn_tmp) {
      nfxd = minmn_tmp;
    }

    qrf(A, m, n, nfxd, tau);
    if (nfxd < minmn_tmp) {
      double d;
      memset(&work[0], 0, 27U * sizeof(double));
      memset(&vn1[0], 0, 27U * sizeof(double));
      memset(&vn2[0], 0, 27U * sizeof(double));
      i = nfxd + 1;
      for (pvt = i; pvt <= n; pvt++) {
        d = xnrm2(m - nfxd, A, (nfxd + (pvt - 1) * 27) + 1);
        vn1[pvt - 1] = d;
        vn2[pvt - 1] = d;
      }

      for (b_i = i; b_i <= minmn_tmp; b_i++) {
        double s;
        int ii;
        int ip1;
        int mmi;
        int nmi;
        ip1 = b_i + 1;
        iy = (b_i - 1) * 27;
        ii = (iy + b_i) - 1;
        nmi = (n - b_i) + 1;
        mmi = m - b_i;
        if (nmi < 1) {
          nfxd = -2;
        } else {
          nfxd = -1;
          if (nmi > 1) {
            temp = fabs(vn1[b_i - 1]);
            for (k = 2; k <= nmi; k++) {
              s = fabs(vn1[(b_i + k) - 2]);
              if (s > temp) {
                nfxd = k - 2;
                temp = s;
              }
            }
          }
        }

        pvt = b_i + nfxd;
        if (pvt + 1 != b_i) {
          ix = pvt * 27;
          for (k = 0; k < m; k++) {
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            nfxd = iy + k;
            A[temp_tmp] = A[nfxd];
            A[nfxd] = temp;
          }

          nfxd = jpvt[pvt];
          jpvt[pvt] = jpvt[b_i - 1];
          jpvt[b_i - 1] = nfxd;
          vn1[pvt] = vn1[b_i - 1];
          vn2[pvt] = vn2[b_i - 1];
        }

        if (b_i < m) {
          temp = A[ii];
          d = xzlarfg(mmi + 1, &temp, A, ii + 2);
          tau[b_i - 1] = d;
          A[ii] = temp;
        } else {
          d = 0.0;
          tau[b_i - 1] = 0.0;
        }

        if (b_i < n) {
          temp = A[ii];
          A[ii] = 1.0;
          xzlarf(mmi + 1, nmi - 1, ii + 1, d, A, ii + 28, work);
          A[ii] = temp;
        }

        for (pvt = ip1; pvt <= n; pvt++) {
          nfxd = b_i + (pvt - 1) * 27;
          d = vn1[pvt - 1];
          if (d != 0.0) {
            temp = fabs(A[nfxd - 1]) / d;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }

            s = d / vn2[pvt - 1];
            s = temp * (s * s);
            if (s <= 1.4901161193847656E-8) {
              if (b_i < m) {
                d = xnrm2(mmi, A, nfxd + 1);
                vn1[pvt - 1] = d;
                vn2[pvt - 1] = d;
              } else {
                vn1[pvt - 1] = 0.0;
                vn2[pvt - 1] = 0.0;
              }
            } else {
              vn1[pvt - 1] = d * sqrt(temp);
            }
          }
        }
      }
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[729]
 *                int ix0
 * Return Type  : double
 */
static double xnrm2(int n, const double x[729], int ix0)
{
  double y;
  int k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        double absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/*
 * Arguments    : int n
 *                double A[729]
 * Return Type  : int
 */
static int xpotrf(int n, double A[729])
{
  int ia;
  int iac;
  int info;
  int j;
  int nmj;
  bool exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j <= n - 1)) {
    double c;
    double ssq;
    int idxA1j;
    int idxAjj;
    idxA1j = j * 27;
    idxAjj = idxA1j + j;
    ssq = 0.0;
    if (j >= 1) {
      for (nmj = 0; nmj < j; nmj++) {
        c = A[idxA1j + nmj];
        ssq += c * c;
      }
    }

    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < n) {
        int i;
        int ia0;
        int idxAjjp1;
        nmj = (n - j) - 2;
        ia0 = idxA1j + 28;
        idxAjjp1 = idxAjj + 28;
        if ((j != 0) && (nmj + 1 != 0)) {
          i = (idxA1j + 27 * nmj) + 28;
          for (iac = ia0; iac <= i; iac += 27) {
            int i1;
            c = 0.0;
            i1 = (iac + j) - 1;
            for (ia = iac; ia <= i1; ia++) {
              c += A[ia - 1] * A[(idxA1j + ia) - iac];
            }

            i1 = (idxAjj + div_nde_s32_floor((iac - idxA1j) - 28, 27) * 27) + 27;
            A[i1] -= c;
          }
        }

        ssq = 1.0 / ssq;
        i = (idxAjj + 27 * nmj) + 28;
        for (nmj = idxAjjp1; nmj <= i; nmj += 27) {
          A[nmj - 1] *= ssq;
        }
      }

      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/*
 * Arguments    : double *a
 *                double *b
 *                double *s
 * Return Type  : double
 */
static double xrotg(double *a, double *b, double *s)
{
  double absa;
  double absb;
  double c;
  double roe;
  double scale;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    double ads;
    double bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (c != 0.0) {
      *b = 1.0 / c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }

  return c;
}

/*
 * Arguments    : int m
 *                int n
 *                int iv0
 *                double tau
 *                double C[729]
 *                int ic0
 *                double work[27]
 * Return Type  : void
 */
static void xzlarf(int m, int n, int iv0, double tau, double C[729], int ic0,
                   double work[27])
{
  int i;
  int ia;
  int iac;
  int lastc;
  int lastv;
  if (tau != 0.0) {
    bool exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      int exitg1;
      i = ic0 + lastc * 27;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    double c;
    int b_i;
    if (lastc + 1 != 0) {
      if (lastc >= 0) {
        memset(&work[0], 0, (unsigned int)(lastc + 1) * sizeof(double));
      }

      b_i = ic0 + 27 * lastc;
      for (iac = ic0; iac <= b_i; iac += 27) {
        c = 0.0;
        i = (iac + lastv) - 1;
        for (ia = iac; ia <= i; ia++) {
          c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
        }

        i = div_nde_s32_floor(iac - ic0, 27);
        work[i] += c;
      }
    }

    if (!(-tau == 0.0)) {
      i = ic0;
      for (iac = 0; iac <= lastc; iac++) {
        c = work[iac];
        if (c != 0.0) {
          c *= -tau;
          b_i = lastv + i;
          for (ia = i; ia < b_i; ia++) {
            C[ia - 1] += C[((iv0 + ia) - i) - 1] * c;
          }
        }

        i += 27;
      }
    }
  }
}

/*
 * Arguments    : int n
 *                double *alpha1
 *                double x[729]
 *                int ix0
 * Return Type  : double
 */
static double xzlarfg(int n, double *alpha1, double x[729], int ix0)
{
  double tau;
  int k;
  tau = 0.0;
  if (n > 0) {
    double xnorm;
    xnorm = xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      double beta1;
      beta1 = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        beta1 = -beta1;
      }

      if (fabs(beta1) < 1.0020841800044864E-292) {
        int i;
        int knt;
        knt = 0;
        i = (ix0 + n) - 2;
        do {
          knt++;
          for (k = ix0; k <= i; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

        beta1 = rt_hypotd_snf(*alpha1, xnrm2(n - 1, x, ix0));
        if (*alpha1 >= 0.0) {
          beta1 = -beta1;
        }

        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        for (k = ix0; k <= i; k++) {
          x[k - 1] *= xnorm;
        }

        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }

        *alpha1 = beta1;
      } else {
        int i;
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        i = (ix0 + n) - 2;
        for (k = ix0; k <= i; k++) {
          x[k - 1] *= xnorm;
        }

        *alpha1 = beta1;
      }
    }
  }

  return tau;
}

/*
 * Create variables necessary for the optimization
 *
 * Arguments    : double m
 *                double I_xx
 *                double I_yy
 *                double I_zz
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double Phi
 *                double Theta
 *                double Omega_1
 *                double Omega_2
 *                double Omega_3
 *                double Omega_4
 *                double b_1
 *                double b_2
 *                double b_3
 *                double b_4
 *                double g_1
 *                double g_2
 *                double g_3
 *                double g_4
 *                double delta_ailerons
 *                double W_act_motor_const
 *                double W_act_motor_speed
 *                double W_act_tilt_el_const
 *                double W_act_tilt_el_speed
 *                double W_act_tilt_az_const
 *                double W_act_tilt_az_speed
 *                double W_act_ailerons_const
 *                double W_act_ailerons_speed
 *                double W_dv_1
 *                double W_dv_2
 *                double W_dv_3
 *                double W_dv_4
 *                double W_dv_5
 *                double W_dv_6
 *                double max_omega
 *                double min_omega
 *                double max_b
 *                double min_b
 *                double max_g
 *                double min_g
 *                double max_delta_ailerons
 *                double min_delta_ailerons
 *                double dv[6]
 *                double p
 *                double q
 *                double r
 *                double Cm_zero
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
 *                double CL_aileron
 *                double rho
 *                double V
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double Beta
 *                double gamma_quadratic_du
 *                double desired_motor_value
 *                double desired_el_value
 *                double desired_az_value
 *                double desired_ailerons_value
 *                double k_alt_tilt_constraint
 *                double min_alt_tilt_constraint
 *                double lidar_alt_corrected
 *                double approach_mode
 *                double verbose
 *                double vert_acc_margin
 *                const double current_accelerations[6]
 *                const double u_init[13]
 *                double use_u_init
 *                const double pqr_dot_outer_loop[3]
 *                double induced_failure
 *                double W_act_motor_failure
 *                double W_act_tilt_el_failure
 *                double W_act_tilt_az_failure
 *                double W_act_ailerons_failure
 *                double W_dv_1_failure
 *                double W_dv_2_failure
 *                double W_dv_3_failure
 *                double W_dv_4_failure
 *                double W_dv_5_failure
 *                double W_dv_6_failure
 *                double gamma_quadratic_du_failure
 *                double power_Cd_0
 *                double power_Cd_a
 *                double prop_R
 *                double prop_Cd_0
 *                double prop_Cl_0
 *                double prop_Cd_a
 *                double prop_Cl_a
 *                double prop_delta
 *                double prop_sigma
 *                double prop_theta
 *                double max_airspeed
 *                double wing_span
 *                double u_out[13]
 *                double residuals[6]
 *                double *elapsed_time
 *                double *N_iterations
 *                double *N_evaluation
 *                double *exitflag
 * Return Type  : void
 */
void Nonlinear_controller_w_ail_new_aero_inner_loop(double m, double I_xx,
  double I_yy, double I_zz, double l_1, double l_2, double l_3, double l_4,
  double l_z, double Phi, double Theta, double Omega_1, double Omega_2, double
  Omega_3, double Omega_4, double b_1, double b_2, double b_3, double b_4,
  double g_1, double g_2, double g_3, double g_4, double delta_ailerons, double
  W_act_motor_const, double W_act_motor_speed, double W_act_tilt_el_const,
  double W_act_tilt_el_speed, double W_act_tilt_az_const, double
  W_act_tilt_az_speed, double W_act_ailerons_const, double W_act_ailerons_speed,
  double W_dv_1, double W_dv_2, double W_dv_3, double W_dv_4, double W_dv_5,
  double W_dv_6, double max_omega, double min_omega, double max_b, double min_b,
  double max_g, double min_g, double max_delta_ailerons, double
  min_delta_ailerons, double dv[6], double p, double q, double r, double Cm_zero,
  double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha, double
  CL_aileron, double rho, double V, double S, double wing_chord, double
  flight_path_angle, double Beta, double gamma_quadratic_du, double
  desired_motor_value, double desired_el_value, double desired_az_value, double
  desired_ailerons_value, double k_alt_tilt_constraint, double
  min_alt_tilt_constraint, double lidar_alt_corrected, double approach_mode,
  double verbose, double vert_acc_margin, const double current_accelerations[6],
  const double u_init[13], double use_u_init, const double pqr_dot_outer_loop[3],
  double induced_failure, double W_act_motor_failure, double
  W_act_tilt_el_failure, double W_act_tilt_az_failure, double
  W_act_ailerons_failure, double W_dv_1_failure, double W_dv_2_failure, double
  W_dv_3_failure, double W_dv_4_failure, double W_dv_5_failure, double
  W_dv_6_failure, double gamma_quadratic_du_failure, double power_Cd_0, double
  power_Cd_a, double prop_R, double prop_Cd_0, double prop_Cl_0, double
  prop_Cd_a, double prop_Cl_a, double prop_delta, double prop_sigma, double
  prop_theta, double max_airspeed, double wing_span, double u_out[13], double
  residuals[6], double *elapsed_time, double *N_iterations, double *N_evaluation,
  double *exitflag)
{
  b_captured_var dv_global;
  c_struct_T b_expl_temp;
  c_struct_T expl_temp;
  captured_var V_scaled;
  captured_var W_act_ailerons;
  captured_var W_act_motor;
  captured_var W_act_tilt_az;
  captured_var W_act_tilt_el;
  captured_var b_Beta;
  captured_var b_CL_aileron;
  captured_var b_Cd_zero;
  captured_var b_Cl_alpha;
  captured_var b_Cm_alpha;
  captured_var b_Cm_zero;
  captured_var b_I_xx;
  captured_var b_I_yy;
  captured_var b_I_zz;
  captured_var b_K_Cd;
  captured_var b_Phi;
  captured_var b_S;
  captured_var b_Theta;
  captured_var b_V;
  captured_var b_W_dv_1;
  captured_var b_W_dv_2;
  captured_var b_W_dv_3;
  captured_var b_W_dv_4;
  captured_var b_W_dv_5;
  captured_var b_W_dv_6;
  captured_var b_desired_ailerons_value;
  captured_var b_desired_az_value;
  captured_var b_desired_el_value;
  captured_var b_desired_motor_value;
  captured_var b_flight_path_angle;
  captured_var b_gamma_quadratic_du;
  captured_var b_l_1;
  captured_var b_l_2;
  captured_var b_l_3;
  captured_var b_l_4;
  captured_var b_l_z;
  captured_var b_m;
  captured_var b_p;
  captured_var b_power_Cd_0;
  captured_var b_power_Cd_a;
  captured_var b_prop_Cd_0;
  captured_var b_prop_Cd_a;
  captured_var b_prop_Cl_0;
  captured_var b_prop_Cl_a;
  captured_var b_prop_R;
  captured_var b_prop_delta;
  captured_var b_prop_sigma;
  captured_var b_prop_theta;
  captured_var b_q;
  captured_var b_r;
  captured_var b_rho;
  captured_var b_wing_chord;
  captured_var b_wing_span;
  captured_var gain_ailerons;
  captured_var gain_az;
  captured_var gain_el;
  captured_var gain_motor;
  double u_out_local[15];
  double u_max[13];
  double u_min[13];
  double final_accelerations[6];
  double b_max_approach;
  double b_min_approach;
  double g_max_approach;
  double max_tilt_value_approach;
  int i;
  char c_expl_temp[3];
  if (!isInitialized_Nonlinear_controller_w_ail_new_aero_inner_loop) {
    Nonlinear_controller_w_ail_new_aero_inner_loop_initialize();
  }

  b_m.contents = m;
  b_I_xx.contents = I_xx;
  b_I_yy.contents = I_yy;
  b_I_zz.contents = I_zz;
  b_l_1.contents = l_1;
  b_l_2.contents = l_2;
  b_l_3.contents = l_3;
  b_l_4.contents = l_4;
  b_l_z.contents = l_z;
  b_Phi.contents = Phi;
  b_Theta.contents = Theta;
  b_W_dv_1.contents = W_dv_1;
  b_W_dv_2.contents = W_dv_2;
  b_W_dv_3.contents = W_dv_3;
  b_W_dv_4.contents = W_dv_4;
  b_W_dv_5.contents = W_dv_5;
  b_W_dv_6.contents = W_dv_6;
  b_p.contents = p;
  b_q.contents = q;
  b_r.contents = r;
  b_Cm_zero.contents = Cm_zero;
  b_Cl_alpha.contents = Cl_alpha;
  b_Cd_zero.contents = Cd_zero;
  b_K_Cd.contents = K_Cd;
  b_Cm_alpha.contents = Cm_alpha;
  b_CL_aileron.contents = CL_aileron;
  b_rho.contents = rho;
  b_V.contents = V;
  b_S.contents = S;
  b_wing_chord.contents = wing_chord;
  b_flight_path_angle.contents = flight_path_angle;
  b_Beta.contents = Beta;
  b_gamma_quadratic_du.contents = gamma_quadratic_du;
  b_desired_motor_value.contents = desired_motor_value;
  b_desired_el_value.contents = desired_el_value;
  b_desired_az_value.contents = desired_az_value;
  b_desired_ailerons_value.contents = desired_ailerons_value;
  b_power_Cd_0.contents = power_Cd_0;
  b_power_Cd_a.contents = power_Cd_a;
  b_prop_R.contents = prop_R;
  b_prop_Cd_0.contents = prop_Cd_0;
  b_prop_Cl_0.contents = prop_Cl_0;
  b_prop_Cd_a.contents = prop_Cd_a;
  b_prop_Cl_a.contents = prop_Cl_a;
  b_prop_delta.contents = prop_delta;
  b_prop_sigma.contents = prop_sigma;
  b_prop_theta.contents = prop_theta;
  b_wing_span.contents = wing_span;
  gain_motor.contents = max_omega / 2.0;
  gain_el.contents = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
  gain_az.contents = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;
  gain_ailerons.contents = (max_delta_ailerons - min_delta_ailerons) *
    3.1415926535897931 / 180.0 / 2.0;
  max_tilt_value_approach = b_V.contents / max_airspeed;
  V_scaled.contents = max_tilt_value_approach;
  u_max[0] = max_omega;
  u_max[1] = max_omega;
  u_max[2] = max_omega;
  u_max[3] = max_omega;
  u_max[4] = max_b;
  u_max[5] = max_b;
  u_max[6] = max_b;
  u_max[7] = max_b;
  u_max[8] = max_g;
  u_max[9] = max_g;
  u_max[10] = max_g;
  u_max[11] = max_g;
  u_max[12] = max_delta_ailerons;
  u_min[0] = min_omega;
  u_min[1] = min_omega;
  u_min[2] = min_omega;
  u_min[3] = min_omega;
  u_min[4] = min_b;
  u_min[5] = min_b;
  u_min[6] = min_b;
  u_min[7] = min_b;
  u_min[8] = min_g;
  u_min[9] = min_g;
  u_min[10] = min_g;
  u_min[11] = min_g;
  u_min[12] = min_delta_ailerons;

  /* Build the max and minimum actuator array: */
  if ((induced_failure > 0.5) && (induced_failure < 1.5)) {
    /* Motor 1 */
    u_max[0] = min_omega;
    u_max[1] = max_omega;
    u_max[2] = max_omega;
    u_max[3] = max_omega;
    u_max[4] = 0.0;
    u_max[5] = max_b;
    u_max[6] = max_b;
    u_max[7] = max_b;
    u_max[8] = 0.0;
    u_max[9] = max_g;
    u_max[10] = max_g;
    u_max[11] = max_g;
    u_max[12] = max_delta_ailerons;
    u_min[0] = min_omega;
    u_min[1] = min_omega;
    u_min[2] = min_omega;
    u_min[3] = min_omega;
    u_min[4] = 0.0;
    u_min[5] = min_b;
    u_min[6] = min_b;
    u_min[7] = min_b;
    u_min[8] = 0.0;
    u_min[9] = min_g;
    u_min[10] = min_g;
    u_min[11] = min_g;
    u_min[12] = min_delta_ailerons;
  } else if ((induced_failure > 1.5) && (induced_failure < 2.5)) {
    /* Motor 2 */
    u_max[0] = max_omega;
    u_max[1] = min_omega;
    u_max[2] = max_omega;
    u_max[3] = max_omega;
    u_max[4] = max_b;
    u_max[5] = 0.0;
    u_max[6] = max_b;
    u_max[7] = max_b;
    u_max[8] = max_g;
    u_max[9] = 0.0;
    u_max[10] = max_g;
    u_max[11] = max_g;
    u_max[12] = max_delta_ailerons;
    u_min[0] = min_omega;
    u_min[1] = min_omega;
    u_min[2] = min_omega;
    u_min[3] = min_omega;
    u_min[4] = min_b;
    u_min[5] = 0.0;
    u_min[6] = min_b;
    u_min[7] = min_b;
    u_min[8] = min_g;
    u_min[9] = 0.0;
    u_min[10] = min_g;
    u_min[11] = min_g;
    u_min[12] = min_delta_ailerons;
  } else if ((induced_failure > 2.5) && (induced_failure < 3.5)) {
    /* Motor 3 */
    u_max[0] = max_omega;
    u_max[1] = max_omega;
    u_max[2] = min_omega;
    u_max[3] = max_omega;
    u_max[4] = max_b;
    u_max[5] = max_b;
    u_max[6] = 0.0;
    u_max[7] = max_b;
    u_max[8] = max_g;
    u_max[9] = max_g;
    u_max[10] = 0.0;
    u_max[11] = max_g;
    u_max[12] = max_delta_ailerons;
    u_min[0] = min_omega;
    u_min[1] = min_omega;
    u_min[2] = min_omega;
    u_min[3] = min_omega;
    u_min[4] = min_b;
    u_min[5] = min_b;
    u_min[6] = 0.0;
    u_min[7] = min_b;
    u_min[8] = min_g;
    u_min[9] = min_g;
    u_min[10] = 0.0;
    u_min[11] = min_g;
    u_min[12] = min_delta_ailerons;
  } else if ((induced_failure > 3.5) && (induced_failure < 4.5)) {
    /* Motor 4 */
    u_max[0] = max_omega;
    u_max[1] = max_omega;
    u_max[2] = max_omega;
    u_max[3] = min_omega;
    u_max[4] = max_b;
    u_max[5] = max_b;
    u_max[6] = max_b;
    u_max[7] = 0.0;
    u_max[8] = max_g;
    u_max[9] = max_g;
    u_max[10] = max_g;
    u_max[11] = 0.0;
    u_max[12] = max_delta_ailerons;
    u_min[0] = min_omega;
    u_min[1] = min_omega;
    u_min[2] = min_omega;
    u_min[3] = min_omega;
    u_min[4] = min_b;
    u_min[5] = min_b;
    u_min[6] = min_b;
    u_min[7] = 0.0;
    u_min[8] = min_g;
    u_min[9] = min_g;
    u_min[10] = min_g;
    u_min[11] = 0.0;
    u_min[12] = min_delta_ailerons;
  }

  if (approach_mode != 0.0) {
    double b_max_tilt_value_approach[2];
    b_max_tilt_value_approach[0] = 0.0;
    b_max_tilt_value_approach[1] = k_alt_tilt_constraint * lidar_alt_corrected -
      min_alt_tilt_constraint * k_alt_tilt_constraint;
    max_tilt_value_approach = maximum(b_max_tilt_value_approach);

    /* Elevation angle */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_b;
    b_max_approach = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = min_b;
    b_min_approach = maximum(b_max_tilt_value_approach);

    /* Azimuth angle */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_g;
    g_max_approach = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = min_g;
    max_tilt_value_approach = maximum(b_max_tilt_value_approach);
    u_max[4] = b_max_approach;
    u_min[4] = b_min_approach;
    u_max[8] = g_max_approach;
    u_min[8] = max_tilt_value_approach;
    u_max[5] = b_max_approach;
    u_min[5] = b_min_approach;
    u_max[9] = g_max_approach;
    u_min[9] = max_tilt_value_approach;
    u_max[6] = b_max_approach;
    u_min[6] = b_min_approach;
    u_max[10] = g_max_approach;
    u_min[10] = max_tilt_value_approach;
    u_max[7] = b_max_approach;
    u_min[7] = b_min_approach;
    u_max[11] = g_max_approach;
    u_min[11] = max_tilt_value_approach;
  }

  for (i = 0; i < 9; i++) {
    u_max[i + 4] = u_max[i + 4] * 3.1415926535897931 / 180.0;
    u_min[i + 4] = u_min[i + 4] * 3.1415926535897931 / 180.0;
  }

  u_max[0] /= gain_motor.contents;
  u_min[0] /= gain_motor.contents;
  u_max[4] /= gain_el.contents;
  u_min[4] /= gain_el.contents;
  u_max[8] /= gain_az.contents;
  u_min[8] /= gain_az.contents;
  u_max[1] /= gain_motor.contents;
  u_min[1] /= gain_motor.contents;
  u_max[5] /= gain_el.contents;
  u_min[5] /= gain_el.contents;
  u_max[9] /= gain_az.contents;
  u_min[9] /= gain_az.contents;
  u_max[2] /= gain_motor.contents;
  u_min[2] /= gain_motor.contents;
  u_max[6] /= gain_el.contents;
  u_min[6] /= gain_el.contents;
  u_max[10] /= gain_az.contents;
  u_min[10] /= gain_az.contents;
  u_max[3] /= gain_motor.contents;
  u_min[3] /= gain_motor.contents;
  u_max[7] /= gain_el.contents;
  u_min[7] /= gain_el.contents;
  u_max[11] /= gain_az.contents;
  u_min[11] /= gain_az.contents;
  u_max[12] /= gain_ailerons.contents;
  u_min[12] /= gain_ailerons.contents;
  if (use_u_init > 0.8) {
    memcpy(&u_out[0], &u_init[0], 13U * sizeof(double));
    u_out[0] = u_init[0] / gain_motor.contents;
    u_out[4] /= gain_el.contents;
    u_out[8] /= gain_az.contents;
    u_out[1] = u_init[1] / gain_motor.contents;
    u_out[5] /= gain_el.contents;
    u_out[9] /= gain_az.contents;
    u_out[2] = u_init[2] / gain_motor.contents;
    u_out[6] /= gain_el.contents;
    u_out[10] /= gain_az.contents;
    u_out[3] = u_init[3] / gain_motor.contents;
    u_out[7] /= gain_el.contents;
    u_out[11] /= gain_az.contents;
    u_out[12] /= gain_ailerons.contents;
  } else {
    u_out[0] = Omega_1 / gain_motor.contents;
    u_out[4] = b_1 / gain_el.contents;
    u_out[8] = g_1 / gain_az.contents;
    u_out[1] = Omega_2 / gain_motor.contents;
    u_out[5] = b_2 / gain_el.contents;
    u_out[9] = g_2 / gain_az.contents;
    u_out[2] = Omega_3 / gain_motor.contents;
    u_out[6] = b_3 / gain_el.contents;
    u_out[10] = g_3 / gain_az.contents;
    u_out[3] = Omega_4 / gain_motor.contents;
    u_out[7] = b_4 / gain_el.contents;
    u_out[11] = g_4 / gain_az.contents;
    u_out[12] = delta_ailerons / gain_ailerons.contents;
  }

  /*  Apply Nonlinear optimization algorithm: */
  dv[3] = pqr_dot_outer_loop[0];
  dv[4] = pqr_dot_outer_loop[1];
  dv[5] = pqr_dot_outer_loop[2];
  for (i = 0; i < 6; i++) {
    dv_global.contents[i] = dv[i] + current_accelerations[i];
  }

  /* Pseudo-control hedging:  */
  max_tilt_value_approach = 0.0;
  if (dv_global.contents[2] >= 9.81 - vert_acc_margin) {
    max_tilt_value_approach = dv_global.contents[2] - (9.81 - vert_acc_margin);
  }

  dv_global.contents[2] -= max_tilt_value_approach;

  /* Compute weights for individual actuators and make sure they are always positive */
  max_tilt_value_approach = W_act_motor_const + W_act_motor_speed * b_V.contents;
  W_act_motor.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_tilt_el_const + W_act_tilt_el_speed *
    b_V.contents;
  W_act_tilt_el.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_tilt_az_const + W_act_tilt_az_speed *
    b_V.contents;
  W_act_tilt_az.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_ailerons_const + W_act_ailerons_speed *
    b_V.contents;
  W_act_ailerons.contents = fmax(0.0, max_tilt_value_approach);

  /* Default values for the optimizer: */
  /*  OPRIMIZATION OPTIONS */
  tic();

  /* Compute weights for individual actuators and make sure they are always positive */
  if (induced_failure > 0.5) {
    W_act_motor.contents = W_act_motor_failure;
    W_act_tilt_el.contents = W_act_tilt_el_failure;
    W_act_tilt_az.contents = W_act_tilt_az_failure;
    W_act_ailerons.contents = W_act_ailerons_failure;
    b_W_dv_1.contents = W_dv_1_failure;
    b_W_dv_2.contents = W_dv_2_failure;
    b_W_dv_3.contents = W_dv_3_failure;
    b_W_dv_4.contents = W_dv_4_failure;
    b_W_dv_5.contents = W_dv_5_failure;
    b_W_dv_6.contents = W_dv_6_failure;
    b_gamma_quadratic_du.contents = gamma_quadratic_du_failure;
  }

  /* Second optimization run to identify the actuator commands: */
  expl_temp.wing_span = &b_wing_span;
  expl_temp.prop_theta = &b_prop_theta;
  expl_temp.prop_sigma = &b_prop_sigma;
  expl_temp.prop_delta = &b_prop_delta;
  expl_temp.prop_Cl_a = &b_prop_Cl_a;
  expl_temp.prop_Cd_a = &b_prop_Cd_a;
  expl_temp.prop_Cl_0 = &b_prop_Cl_0;
  expl_temp.prop_Cd_0 = &b_prop_Cd_0;
  expl_temp.prop_R = &b_prop_R;
  expl_temp.power_Cd_a = &b_power_Cd_a;
  expl_temp.power_Cd_0 = &b_power_Cd_0;
  expl_temp.V_scaled = &V_scaled;
  expl_temp.wing_chord = &b_wing_chord;
  expl_temp.rho = &b_rho;
  expl_temp.r = &b_r;
  expl_temp.q = &b_q;
  expl_temp.p = &b_p;
  expl_temp.m = &b_m;
  expl_temp.l_z = &b_l_z;
  expl_temp.l_4 = &b_l_4;
  expl_temp.l_3 = &b_l_3;
  expl_temp.l_2 = &b_l_2;
  expl_temp.l_1 = &b_l_1;
  expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  expl_temp.gain_ailerons = &gain_ailerons;
  expl_temp.gain_motor = &gain_motor;
  expl_temp.gain_az = &gain_az;
  expl_temp.gain_el = &gain_el;
  expl_temp.flight_path_angle = &b_flight_path_angle;
  expl_temp.desired_ailerons_value = &b_desired_ailerons_value;
  expl_temp.desired_motor_value = &b_desired_motor_value;
  expl_temp.desired_az_value = &b_desired_az_value;
  expl_temp.desired_el_value = &b_desired_el_value;
  expl_temp.W_act_ailerons = &W_act_ailerons;
  expl_temp.W_act_tilt_az = &W_act_tilt_az;
  expl_temp.W_act_tilt_el = &W_act_tilt_el;
  expl_temp.W_dv_6 = &b_W_dv_6;
  expl_temp.W_dv_5 = &b_W_dv_5;
  expl_temp.W_dv_4 = &b_W_dv_4;
  expl_temp.W_dv_3 = &b_W_dv_3;
  expl_temp.W_dv_2 = &b_W_dv_2;
  expl_temp.W_dv_1 = &b_W_dv_1;
  expl_temp.W_act_motor = &W_act_motor;
  expl_temp.V = &b_V;
  expl_temp.Theta = &b_Theta;
  expl_temp.S = &b_S;
  expl_temp.Phi = &b_Phi;
  expl_temp.K_Cd = &b_K_Cd;
  expl_temp.I_zz = &b_I_zz;
  expl_temp.I_yy = &b_I_yy;
  expl_temp.I_xx = &b_I_xx;
  expl_temp.Cm_alpha = &b_Cm_alpha;
  expl_temp.Cm_zero = &b_Cm_zero;
  expl_temp.Cl_alpha = &b_Cl_alpha;
  expl_temp.Cd_zero = &b_Cd_zero;
  expl_temp.CL_aileron = &b_CL_aileron;
  expl_temp.Beta = &b_Beta;
  expl_temp.dv_global = &dv_global;
  b_expl_temp = expl_temp;
  fmincon(&b_expl_temp, u_out, u_min, u_max, exitflag, N_iterations,
          N_evaluation, c_expl_temp, &b_max_approach, &b_min_approach,
          &g_max_approach, &max_tilt_value_approach);
  *elapsed_time = toc();
  max_tilt_value_approach = gain_motor.contents;
  b_max_approach = gain_el.contents;
  b_min_approach = gain_az.contents;
  u_out_local[0] = u_out[0] * max_tilt_value_approach;
  u_out_local[4] = u_out[4] * b_max_approach;
  u_out_local[8] = u_out[8] * b_min_approach;
  u_out_local[1] = u_out[1] * max_tilt_value_approach;
  u_out_local[5] = u_out[5] * b_max_approach;
  u_out_local[9] = u_out[9] * b_min_approach;
  u_out_local[2] = u_out[2] * max_tilt_value_approach;
  u_out_local[6] = u_out[6] * b_max_approach;
  u_out_local[10] = u_out[10] * b_min_approach;
  u_out_local[3] = u_out[3] * max_tilt_value_approach;
  u_out_local[7] = u_out[7] * b_max_approach;
  u_out_local[11] = u_out[11] * b_min_approach;
  u_out_local[12] = b_Theta.contents;
  u_out_local[13] = b_Phi.contents;
  u_out_local[14] = u_out[12] * gain_ailerons.contents;
  c_compute_acc_nonlinear_CA_new_(u_out_local, b_p.contents, b_q.contents,
    b_r.contents, b_m.contents, b_I_xx.contents, b_I_yy.contents,
    b_I_zz.contents, b_l_1.contents, b_l_2.contents, b_l_3.contents,
    b_l_4.contents, b_l_z.contents, b_Cl_alpha.contents, b_Cd_zero.contents,
    b_K_Cd.contents, b_Cm_alpha.contents, b_Cm_zero.contents,
    b_CL_aileron.contents, b_rho.contents, b_V.contents, b_S.contents,
    b_wing_chord.contents, b_flight_path_angle.contents, b_Beta.contents,
    b_wing_span.contents, b_prop_R.contents, b_prop_Cd_0.contents,
    b_prop_Cl_0.contents, b_prop_Cd_a.contents, b_prop_Cl_a.contents,
    b_prop_delta.contents, b_prop_sigma.contents, b_prop_theta.contents,
    V_scaled.contents, u_out[0], u_out[1], final_accelerations);
  for (i = 0; i < 6; i++) {
    residuals[i] = dv_global.contents[i] - final_accelerations[i];
  }

  max_tilt_value_approach = gain_motor.contents;
  b_max_approach = gain_el.contents;
  b_min_approach = gain_az.contents;
  u_out[0] *= max_tilt_value_approach;
  u_out[4] *= b_max_approach;
  u_out[8] *= b_min_approach;
  u_out[1] *= max_tilt_value_approach;
  u_out[5] *= b_max_approach;
  u_out[9] *= b_min_approach;
  u_out[2] *= max_tilt_value_approach;
  u_out[6] *= b_max_approach;
  u_out[10] *= b_min_approach;
  u_out[3] *= max_tilt_value_approach;
  u_out[7] *= b_max_approach;
  u_out[11] *= b_min_approach;
  u_out[12] *= gain_ailerons.contents;

  /*  Print infos */
  if (verbose != 0.0) {
    printf("\n Solution: \n");
    fflush(stdout);
    printf(" Motors [rad/s] =  ");
    fflush(stdout);
    printf(" %f ", u_out[0]);
    fflush(stdout);
    printf(" %f ", u_out[1]);
    fflush(stdout);
    printf(" %f ", u_out[2]);
    fflush(stdout);
    printf(" %f ", u_out[3]);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Elevator angles [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[4] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[5] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[6] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[7] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Azimuth angles [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[8] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[9] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[10] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[11] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Ailerons deflection [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[12] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("\n Elapsed time = %f \n", *elapsed_time);
    fflush(stdout);
    printf("\n Number of iterations / evaluations = %f ", *N_iterations);
    fflush(stdout);
    printf("/ %f \n", *N_evaluation);
    fflush(stdout);
    printf("\n Exit flag optimizer = %f \n", *exitflag);
    fflush(stdout);
    printf("\n Modeled accelerations =   ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", current_accelerations[i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n desired acc increment =    ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", dv[i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n Requested accelerations =  ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      max_tilt_value_approach = dv_global.contents[i];
      printf(" %f ", max_tilt_value_approach);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n Achieved accelerations =   ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", final_accelerations[i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n Acc residuals / norm  =    ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", residuals[i]);
      fflush(stdout);
    }

    printf(" / ");
    fflush(stdout);
    printf(" %f \n", b_norm(residuals));
    fflush(stdout);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Nonlinear_controller_w_ail_new_aero_inner_loop_initialize(void)
{
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  isInitialized_Nonlinear_controller_w_ail_new_aero_inner_loop = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Nonlinear_controller_w_ail_new_aero_inner_loop_terminate(void)
{
  isInitialized_Nonlinear_controller_w_ail_new_aero_inner_loop = false;
}

/*
 * File trailer for Nonlinear_controller_w_ail_new_aero_inner_loop.c
 *
 * [EOF]
 */
