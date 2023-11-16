/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Nonlinear_controller_fcn_control_rf_aero_models.c
 *
 * Code generation for function 'Nonlinear_controller_fcn_control_rf_aero_models'
 *
 */

/* Include files */
#include "Nonlinear_controller_fcn_control_rf_aero_models.h"
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
  double grad[16];
  double Hx[15];
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

#ifndef typedef_cell_18
#define typedef_cell_18

typedef struct {
  double f1[15];
  double f2;
  double f3;
  double f4;
  double f5;
  double f6;
  double f7;
  double f8;
  double f9;
  double f10;
  double f11;
  double f12;
  double f13;
  double f14;
  double f15;
  double f16;
  double f17;
  double f18;
  double f19;
  double f20;
  double f21;
  double f22;
  double f23;
  double f24;
  double f25;
  double f26;
  double f27;
  double f28;
  double f29;
  double f30;
  double f31;
  double f32;
  double f33;
  double f34;
  double f35;
  double f36;
  double f37;
  double f38;
  double f39;
  double f40;
  double f41;
  double f42;
  double f43;
  double f44;
  double f45;
  double f46;
  double f47;
  double f48;
  double f49;
  double f50;
  double f51;
  double f52;
  double f53;
  double f54;
  double f55;
  double f56;
  double f57;
  double f58;
  double f59;
  double f60;
  double f61;
  double f62;
  double f63;
  double f64;
  double f65;
  double f66;
  double f67;
  double f68;
  double f69;
  double f70;
  double f71;
  double f72;
  double f73;
  double f74;
  double f75;
  double f76;
  double f77;
  double f78;
  double f79;
  double f80;
  double f81;
  double f82;
  double f83;
  double f84;
  double f85;
  double f86;
  double f87;
  double f88;
  double f89;
  double f90;
  double f91;
  double f92;
  double f93;
  double f94;
  double f95;
  double f96;
  double f97;
  double f98;
  double f99;
  double f100;
  double f101;
  double f102;
  double f103;
  double f104;
  double f105;
  double f106;
  double f107;
  double f108;
  double f109;
  double f110;
  double f111;
  double f112;
  double f113;
  double f114;
  double f115;
  double f116;
  double f117;
  double f118;
  double f119;
  double f120;
  double f121;
  double f122;
  double f123;
  double f124;
  double f125;
  double f126;
  double f127;
  double f128;
  double f129;
  double f130;
  double f131;
  double f132;
  double f133;
  double f134;
  double f135;
  double f136;
  double f137;
  double f138;
  double f139;
  double f140;
  double f141;
  double f142;
  double f143;
  double f144;
  double f145;
  double f146;
  double f147;
  double f148;
  double f149;
  double f150;
  double f151;
  double f152;
  double f153;
  double f154;
  double f155;
  double f156;
  double f157;
  double f158;
  double f159;
  double f160;
  double f161;
  double f162;
  double f163;
  double f164;
  double f165;
  double f166;
  double f167;
  double f168;
  double f169;
  double f170;
  double f171;
  double f172;
  double f173;
  double f174;
  double f175;
  double f176;
  double f177;
  double f178;
} cell_18;

#endif                                 /* typedef_cell_18 */

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
  captured_var *V;
  captured_var *gain_motor;
  captured_var *prop_sigma;
  captured_var *rho;
  captured_var *gain_el;
  captured_var *gain_az;
  captured_var *l_1;
  captured_var *l_4;
  captured_var *l_3;
  captured_var *l_z;
  captured_var *gamma_quadratic_du;
  captured_var *desired_phi_value;
  captured_var *desired_theta_value;
  captured_var *desired_ailerons_value;
  captured_var *K_p_M;
  captured_var *k_tilt;
  captured_var *gain_theta;
  captured_var *K_Cd;
  captured_var *S;
  captured_var *Cm_alpha;
  captured_var *wing_chord;
  captured_var *gain_phi;
  captured_var *CL_aileron;
  captured_var *gain_ailerons;
  captured_var *flight_path_angle;
  captured_var *Beta;
  captured_var *prop_delta;
  captured_var *Cl_alpha;
  captured_var *W_act_phi;
  captured_var *W_act_theta;
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
  captured_var *prop_Cd_a;
  captured_var *prop_R;
  captured_var *prop_Cd_0;
  captured_var *I_xx;
  captured_var *p;
  captured_var *q;
  captured_var *I_yy;
  captured_var *r;
  captured_var *I_zz;
  captured_var *gain_airspeed;
  captured_var *m;
  captured_var *prop_Cl_a;
  captured_var *prop_theta;
  captured_var *prop_Cl_0;
  captured_var *desired_el_value;
  captured_var *desired_az_value;
  captured_var *desired_motor_value;
  captured_var *Cm_zero;
  captured_var *Cd_zero;
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
  double workspace_double[496];
  int workspace_int[31];
  int workspace_sort[31];
} d_struct_T;

#endif                                 /* typedef_d_struct_T */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T

typedef struct {
  int ldq;
  double QR[961];
  double Q[961];
  int jpvt[31];
  int mrows;
  int ncols;
  double tau[31];
  int minRowCol;
  bool usedPivoting;
} e_struct_T;

#endif                                 /* typedef_e_struct_T */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T

typedef struct {
  double FMat[961];
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  bool ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
} f_struct_T;

#endif                                 /* typedef_f_struct_T */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T

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
  double xstarsqp[15];
  double xstarsqp_old[15];
  double grad[16];
  double grad_old[16];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[31];
  double lambdaStopTest[31];
  double lambdaStopTestPrev[31];
  double steplength;
  double delta_x[16];
  double socDirection[16];
  int workingset_old[31];
  double gradLag[16];
  double delta_gradLag[16];
  double xstar[16];
  double fstar;
  double firstorderopt;
  double lambda[31];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[16];
} g_struct_T;

#endif                                 /* typedef_g_struct_T */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T

typedef struct {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double lb[16];
  double ub[16];
  int indexLB[16];
  int indexUB[16];
  int indexFixed[16];
  int mEqRemoved;
  double ATwset[496];
  double bwset[31];
  int nActiveConstr;
  double maxConstrWorkspace[31];
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
  bool isActiveConstr[31];
  int Wid[31];
  int Wlocalidx[31];
  int nWConstr[5];
  int probType;
  double SLACK0;
} h_struct_T;

#endif                                 /* typedef_h_struct_T */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T

typedef struct {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ObjectiveLimit;
} i_struct_T;

#endif                                 /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T

typedef struct {
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
} j_struct_T;

#endif                                 /* typedef_j_struct_T */

/* Variable Definitions */
static double freq;
static bool freq_not_empty;
static coderTimespec savedTime;
static bool savedTime_not_empty;
static bool isInitialized_Nonlinear_controller_fcn_control_rf_aero_models =
  false;

/* Function Declarations */
static bool BFGSUpdate(int nvar, double Bk[225], const double sk[16], double yk
  [16], double workspace[496]);
static void PresolveWorkingSet(g_struct_T *solution, d_struct_T *memspace,
  h_struct_T *workingset, e_struct_T *qrmanager);
static void RemoveDependentIneq_(h_struct_T *workingset, e_struct_T *qrmanager,
  d_struct_T *memspace, double tolfactor);
static void addBoundToActiveSetMatrix_(h_struct_T *obj, int TYPE, int idx_local);
static void b_computeGradLag(double workspace[496], int nVar, const double grad
  [16], const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB,
  const int finiteUB[16], int mUB, const double lambda[31]);
static double b_compute_cost_and_gradient_fcn(const b_captured_var *dv_global,
  const captured_var *V, const captured_var *gain_motor, const captured_var
  *prop_sigma, const captured_var *rho, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *l_1, const captured_var *l_4, const
  captured_var *l_3, const captured_var *l_z, const captured_var
  *gamma_quadratic_du, const captured_var *desired_phi_value, const captured_var
  *desired_theta_value, const captured_var *desired_ailerons_value, const
  captured_var *K_p_M, const captured_var *k_tilt, const captured_var
  *gain_theta, const captured_var *K_Cd, const captured_var *S, const
  captured_var *Cm_alpha, const captured_var *wing_chord, const captured_var
  *gain_phi, const captured_var *CL_aileron, const captured_var *gain_ailerons,
  const captured_var *flight_path_angle, const captured_var *Beta, const
  captured_var *prop_delta, const captured_var *Cl_alpha, const captured_var
  *W_act_phi, const captured_var *W_act_theta, const captured_var *W_act_motor,
  const captured_var *W_dv_1, const captured_var *W_dv_2, const captured_var
  *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5, const
  captured_var *W_dv_6, const captured_var *W_act_tilt_el, const captured_var
  *W_act_tilt_az, const captured_var *W_act_ailerons, const captured_var
  *prop_Cd_a, const captured_var *prop_R, const captured_var *prop_Cd_0, const
  captured_var *I_xx, const captured_var *p, const captured_var *q, const
  captured_var *I_yy, const captured_var *r, const captured_var *I_zz, const
  captured_var *gain_airspeed, const captured_var *m, const captured_var
  *prop_Cl_a, const captured_var *prop_theta, const captured_var *prop_Cl_0,
  const captured_var *desired_el_value, const captured_var *desired_az_value,
  const captured_var *desired_motor_value, const captured_var *Cm_zero, const
  captured_var *Cd_zero, const double u_in[15]);
static void b_driver(const double lb[15], const double ub[15], g_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     i_coder_internal_stickyStruct *FcnEvaluator, d_struct_T
                     *memspace, h_struct_T *WorkingSet, double Hessian[225],
                     e_struct_T *QRManager, f_struct_T *CholManager, struct_T
                     *QPObjective);
static double b_ft_1(const b_captured_var *dv_global, const captured_var *V,
                     const captured_var *gain_motor, const captured_var
                     *prop_sigma, const captured_var *rho, const captured_var
                     *l_1, const captured_var *l_4, const captured_var *l_3,
                     const captured_var *l_z, const captured_var
                     *gamma_quadratic_du, const captured_var *desired_phi_value,
                     const captured_var *desired_theta_value, const captured_var
                     *desired_ailerons_value, const cell_18 *ct);
static double b_maxConstraintViolation(const h_struct_T *obj, const double x[16]);
static double b_norm(const double x[15]);
static void b_test_exit(j_struct_T *Flags, d_struct_T *memspace, b_struct_T
  *MeritFunction, const h_struct_T *WorkingSet, g_struct_T *TrialState,
  e_struct_T *QRManager, const double lb[15], const double ub[15]);
static double b_timeKeeper(double *outTime_tv_nsec);
static void b_xgemm(int m, int n, int k, const double A[961], int ia0, const
                    double B[496], double C[961]);
static void b_xgemv(int m, int n, const double A[961], const double x[16],
                    double y[496]);
static double b_xnrm2(int n, const double x[16]);
static double c_computeObjectiveAndUserGradie(const c_struct_T
  *c_obj_next_next_next_next_next_, const double x[15], double grad_workspace[16],
  int *status);
static void c_compute_acc_nonlinear_control(const double u_in[15], double p,
  double q, double r, double m, double I_xx, double I_yy, double I_zz, double
  l_1, double l_3, double l_4, double l_z, double Cl_alpha, double Cd_zero,
  double K_Cd, double Cm_alpha, double Cm_zero, double CL_aileron, double rho,
  double V, double S, double wing_chord, double flight_path_angle, double
  gain_motor, double gain_airspeed, double gain_el, double gain_theta, double
  gain_az, double gain_phi, double computed_acc[6]);
static double c_norm(const double x[6]);
static double computeComplError(const double xCurrent[15], const int finiteLB[16],
  int mLB, const double lb[15], const int finiteUB[16], int mUB, const double
  ub[15], const double lambda[31], int iL0);
static double computeFval(const struct_T *obj, double workspace[496], const
  double H[225], const double f[16], const double x[16]);
static double computeFval_ReuseHx(const struct_T *obj, double workspace[496],
  const double f[16], const double x[16]);
static void computeGradLag(double workspace[16], int nVar, const double grad[16],
  const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB, const
  int finiteUB[16], int mUB, const double lambda[31]);
static void computeGrad_StoreHx(struct_T *obj, const double H[225], const double
  f[16], const double x[16]);
static void computeQ_(e_struct_T *obj, int nrows);
static double compute_cost_and_gradient_fcn(const b_captured_var *dv_global,
  const captured_var *V, const captured_var *gain_motor, const captured_var
  *prop_sigma, const captured_var *rho, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *l_1, const captured_var *l_4, const
  captured_var *l_3, const captured_var *l_z, const captured_var
  *gamma_quadratic_du, const captured_var *desired_phi_value, const captured_var
  *desired_theta_value, const captured_var *desired_ailerons_value, const
  captured_var *K_p_M, const captured_var *k_tilt, const captured_var
  *gain_theta, const captured_var *K_Cd, const captured_var *S, const
  captured_var *Cm_alpha, const captured_var *wing_chord, const captured_var
  *gain_phi, const captured_var *CL_aileron, const captured_var *gain_ailerons,
  const captured_var *flight_path_angle, const captured_var *Beta, const
  captured_var *prop_delta, const captured_var *Cl_alpha, const captured_var
  *W_act_phi, const captured_var *W_act_theta, const captured_var *W_act_motor,
  const captured_var *W_dv_1, const captured_var *W_dv_2, const captured_var
  *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5, const
  captured_var *W_dv_6, const captured_var *W_act_tilt_el, const captured_var
  *W_act_tilt_az, const captured_var *W_act_ailerons, const captured_var
  *prop_Cd_a, const captured_var *prop_R, const captured_var *prop_Cd_0, const
  captured_var *I_xx, const captured_var *p, const captured_var *q, const
  captured_var *I_yy, const captured_var *r, const captured_var *I_zz, const
  captured_var *gain_airspeed, const captured_var *m, const captured_var
  *prop_Cl_a, const captured_var *prop_theta, const captured_var *prop_Cl_0,
  const captured_var *desired_el_value, const captured_var *desired_az_value,
  const captured_var *desired_motor_value, const captured_var *Cm_zero, const
  captured_var *Cd_zero, const double u_in[15], double gradient_data[], int
  *gradient_size);
static void compute_deltax(const double H[225], g_struct_T *solution, d_struct_T
  *memspace, const e_struct_T *qrmanager, f_struct_T *cholmanager, const
  struct_T *objective, bool alwaysPositiveDef);
static void countsort(int x[31], int xLen, int workspace[31], int xMin, int xMax);
static void deleteColMoveEnd(e_struct_T *obj, int idx);
static int div_nde_s32_floor(int numerator);
static void driver(const double H[225], const double f[16], g_struct_T *solution,
                   d_struct_T *memspace, h_struct_T *workingset, e_struct_T
                   *qrmanager, f_struct_T *cholmanager, struct_T *objective,
                   i_struct_T *options, int runTimeOptions_MaxIterations);
static double evalObjAndConstr(const c_struct_T *c_obj_next_next_next_next_next_,
  const double x[15], int *status);
static void factorQR(e_struct_T *obj, const double A[496], int mrows, int ncols);
static bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
  const h_struct_T *workingset, e_struct_T *qrmanager);
static double feasibleratiotest(const double solution_xstar[16], const double
  solution_searchDir[16], int workingset_nVar, const double workingset_lb[16],
  const double workingset_ub[16], const int workingset_indexLB[16], const int
  workingset_indexUB[16], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[31], const int
  workingset_nWConstr[5], bool isPhaseOne, bool *newBlocking, int *constrType,
  int *constrIdx);
static double fmincon(c_struct_T *fun_workspace, const double x0[15], const
                      double lb[15], const double ub[15], double x[15], double
                      *exitflag, double *output_iterations, double
                      *output_funcCount, char output_algorithm[3], double
                      *output_constrviolation, double *output_stepsize, double
                      *output_lssteplength, double *output_firstorderopt);
static double ft_1(const b_captured_var *dv_global, const captured_var *V, const
                   captured_var *gain_motor, const captured_var *prop_sigma,
                   const captured_var *rho, const captured_var *gain_el, const
                   captured_var *gain_az, const captured_var *l_1, const
                   captured_var *l_4, const captured_var *l_3, const
                   captured_var *l_z, const captured_var *gamma_quadratic_du,
                   const captured_var *desired_phi_value, const captured_var
                   *desired_theta_value, const captured_var
                   *desired_ailerons_value, const captured_var *K_p_M, const
                   captured_var *k_tilt, const captured_var *gain_theta, const
                   captured_var *K_Cd, const captured_var *S, const captured_var
                   *Cm_alpha, const captured_var *wing_chord, const captured_var
                   *gain_phi, const captured_var *CL_aileron, const captured_var
                   *gain_ailerons, const cell_18 *ct, double gradient_data[],
                   int *gradient_size);
static void fullColLDL2_(f_struct_T *obj, int NColsRemain);
static void iterate(const double H[225], const double f[16], g_struct_T
                    *solution, d_struct_T *memspace, h_struct_T *workingset,
                    e_struct_T *qrmanager, f_struct_T *cholmanager, struct_T
                    *objective, const char options_SolverName[7], double
                    options_StepTolerance, double options_ObjectiveLimit, int
                    runTimeOptions_MaxIterations);
static void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[496],
  const double H[225], const double f[16], const double x[16]);
static double maxConstraintViolation(const h_struct_T *obj, const double x[496],
  int ix0);
static double maximum(const double x[2]);
static double minimum(const double x[2]);
static void qrf(double A[961], int m, int n, int nfxd, double tau[31]);
static void removeConstr(h_struct_T *obj, int idx_global);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static void setProblemType(h_struct_T *obj, int PROBLEM_TYPE);
static void solve(const f_struct_T *obj, double rhs[16]);
static void sortLambdaQP(double lambda[31], int WorkingSet_nActiveConstr, const
  int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[31], const int WorkingSet_Wlocalidx[31], double workspace[496]);
static bool step(int *STEP_TYPE, double Hessian[225], const double lb[15], const
                 double ub[15], g_struct_T *TrialState, b_struct_T
                 *MeritFunction, d_struct_T *memspace, h_struct_T *WorkingSet,
                 e_struct_T *QRManager, f_struct_T *CholManager, struct_T
                 *QPObjective, i_struct_T *qpoptions);
static bool test_exit(b_struct_T *MeritFunction, const h_struct_T *WorkingSet,
                      g_struct_T *TrialState, const double lb[15], const double
                      ub[15], bool *Flags_fevalOK, bool *Flags_done, bool
                      *Flags_stepAccepted, bool *Flags_failedLineSearch, int
                      *Flags_stepType);
static void tic(void);
static void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec);
static double toc(void);
static void xgemm(int m, int n, int k, const double A[225], int lda, const
                  double B[961], int ib0, double C[496]);
static void xgemv(int m, int n, const double A[225], int lda, const double x[16],
                  double y[15]);
static void xgeqp3(double A[961], int m, int n, int jpvt[31], double tau[31]);
static double xnrm2(int n, const double x[961], int ix0);
static int xpotrf(int n, double A[961]);
static double xrotg(double *a, double *b, double *s);
static void xzlarf(int m, int n, int iv0, double tau, double C[961], int ic0,
                   double work[31]);
static double xzlarfg(int n, double *alpha1, double x[961], int ix0);

/* Function Definitions */
static bool BFGSUpdate(int nvar, double Bk[225], const double sk[16], double yk
  [16], double workspace[496])
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
  i1 = 15 * (nvar - 1) + 1;
  for (iac = 1; iac <= i1; iac += 15) {
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

        ix += 15;
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

        ix += 15;
      }
    }
  }

  return success;
}

static void PresolveWorkingSet(g_struct_T *solution, d_struct_T *memspace,
  h_struct_T *workingset, e_struct_T *qrmanager)
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
        qrmanager->QR[ix + 31 * idx_col] = workingset->ATwset[idx_col + (ix << 4)];
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

    idxDiag = u0 + 31 * (u0 - 1);
    while ((idxDiag > 0) && (fabs(qrmanager->QR[idxDiag - 1]) < tol)) {
      idxDiag -= 32;
      nDepInd++;
    }

    if (nDepInd > 0) {
      bool exitg1;
      computeQ_(qrmanager, qrmanager->mrows);
      idxDiag = 0;
      exitg1 = false;
      while ((!exitg1) && (idxDiag <= nDepInd - 1)) {
        double qtb;
        ix = 31 * ((mTotalWorkingEq_tmp_tmp - idxDiag) - 1);
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
        idxDiag = 31 * idx_col;
        ix0 = idx_col << 4;
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

  if ((nDepInd != -1) && (workingset->nActiveConstr <= 31)) {
    bool guard1 = false;
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

static void RemoveDependentIneq_(h_struct_T *workingset, e_struct_T *qrmanager,
  d_struct_T *memspace, double tolfactor)
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
      nDepIneq = 31 * idx_col;
      idx = idx_col << 4;
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
      idxDiag = idx + 31 * idx;
      while ((idx + 1 > nFixedConstr) && (fabs(qrmanager->QR[idxDiag]) < tol)) {
        nDepIneq++;
        memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
        idx--;
        idxDiag -= 32;
      }
    }

    countsort(memspace->workspace_int, nDepIneq, memspace->workspace_sort,
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      removeConstr(workingset, memspace->workspace_int[idx - 1]);
    }
  }
}

static void addBoundToActiveSetMatrix_(h_struct_T *obj, int TYPE, int idx_local)
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
  colOffset = (i << 4) - 1;
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

static void b_computeGradLag(double workspace[496], int nVar, const double grad
  [16], const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB,
  const int finiteUB[16], int mUB, const double lambda[31])
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

  iL0 = mFixed + (unsigned char)mLB;
  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

static double b_compute_cost_and_gradient_fcn(const b_captured_var *dv_global,
  const captured_var *V, const captured_var *gain_motor, const captured_var
  *prop_sigma, const captured_var *rho, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *l_1, const captured_var *l_4, const
  captured_var *l_3, const captured_var *l_z, const captured_var
  *gamma_quadratic_du, const captured_var *desired_phi_value, const captured_var
  *desired_theta_value, const captured_var *desired_ailerons_value, const
  captured_var *K_p_M, const captured_var *k_tilt, const captured_var
  *gain_theta, const captured_var *K_Cd, const captured_var *S, const
  captured_var *Cm_alpha, const captured_var *wing_chord, const captured_var
  *gain_phi, const captured_var *CL_aileron, const captured_var *gain_ailerons,
  const captured_var *flight_path_angle, const captured_var *Beta, const
  captured_var *prop_delta, const captured_var *Cl_alpha, const captured_var
  *W_act_phi, const captured_var *W_act_theta, const captured_var *W_act_motor,
  const captured_var *W_dv_1, const captured_var *W_dv_2, const captured_var
  *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5, const
  captured_var *W_dv_6, const captured_var *W_act_tilt_el, const captured_var
  *W_act_tilt_az, const captured_var *W_act_ailerons, const captured_var
  *prop_Cd_a, const captured_var *prop_R, const captured_var *prop_Cd_0, const
  captured_var *I_xx, const captured_var *p, const captured_var *q, const
  captured_var *I_yy, const captured_var *r, const captured_var *I_zz, const
  captured_var *gain_airspeed, const captured_var *m, const captured_var
  *prop_Cl_a, const captured_var *prop_theta, const captured_var *prop_Cl_0,
  const captured_var *desired_el_value, const captured_var *desired_az_value,
  const captured_var *desired_motor_value, const captured_var *Cm_zero, const
  captured_var *Cd_zero, const double u_in[15])
{
  cell_18 expl_temp;
  double Alpha;
  double a;
  double b_a;
  double b_t537_tmp_tmp;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double t10;
  double t100;
  double t101;
  double t102;
  double t103;
  double t104;
  double t105;
  double t108;
  double t11;
  double t110;
  double t111;
  double t112;
  double t117;
  double t119;
  double t12;
  double t120;
  double t13;
  double t133;
  double t134;
  double t135;
  double t136;
  double t137;
  double t138;
  double t14;
  double t140;
  double t141;
  double t142;
  double t143;
  double t144;
  double t145;
  double t146;
  double t147;
  double t15;
  double t152;
  double t153;
  double t154;
  double t155;
  double t16;
  double t165;
  double t166;
  double t167;
  double t168;
  double t169;
  double t17;
  double t170;
  double t171;
  double t18;
  double t189;
  double t19;
  double t193;
  double t197;
  double t198;
  double t199;
  double t2;
  double t20;
  double t200;
  double t202;
  double t207;
  double t207_tmp;
  double t21;
  double t22;
  double t23;
  double t24;
  double t259_tmp;
  double t283_tmp;
  double t283_tmp_tmp;
  double t287;
  double t287_tmp_tmp;
  double t3;
  double t302;
  double t367;
  double t368;
  double t4;
  double t404;
  double t409;
  double t41;
  double t410;
  double t411;
  double t412;
  double t42;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t517;
  double t518;
  double t519;
  double t52;
  double t520;
  double t537_tmp_tmp;
  double t57;
  double t58;
  double t59;
  double t60;
  double t61;
  double t62;
  double t63;
  double t64;
  double t65;
  double t66;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t71;
  double t72;
  double t73;
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
  double t86;
  double t87;
  double t88;
  double t89;
  double t9;
  double t94;
  double t95;
  double t96;

  /* { */
  /* function [cost, gradient] = compute_cost_and_gradient_fcn(u_in) */
  /*  */
  /*     Omega_1_scaled = u_in(1); */
  /*     Omega_2_scaled = u_in(2); */
  /*     Omega_3_scaled = u_in(3); */
  /*     Omega_4_scaled = u_in(4); */
  /*  */
  /*     b_1_scaled = u_in(5); */
  /*     b_2_scaled = u_in(6); */
  /*     b_3_scaled = u_in(7); */
  /*     b_4_scaled = u_in(8); */
  /*  */
  /*     g_1_scaled = u_in(9); */
  /*     g_2_scaled = u_in(10); */
  /*     g_3_scaled = u_in(11); */
  /*     g_4_scaled = u_in(12); */
  /*  */
  /*     Theta_scaled = u_in(13); */
  /*     Phi_scaled = u_in(14); */
  /*     delta_ailerons_scaled = u_in(15); */
  /*  */
  /*     Alpha = Theta_scaled * gain_theta - flight_path_angle; */
  /*  */
  /*     dv_global_1 = dv_global(1); */
  /*     dv_global_2 = dv_global(2); */
  /*     dv_global_3 = dv_global(3); */
  /*     dv_global_4 = dv_global(4); */
  /*     dv_global_5 = dv_global(5); */
  /*     dv_global_6 = dv_global(6); */
  /*   */
  /*     gamma1 = 0.1250*((16*V^2*sin(Alpha + b_4_scaled*gain_el)^2)/(Omega_4_scaled^2*gain_motor^2*prop_R^2) - (prop_sigma*(prop_delta - 1)*(8*prop_Cl_0*prop_delta + 8*prop_Cl_0*prop_delta^2 + prop_Cl_a^2*prop_delta*prop_sigma - prop_Cl_a^2*prop_delta^2*prop_sigma + 16*prop_Cl_a*prop_delta*prop_theta + (8*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_4_scaled*gain_el)^2)/(Omega_4_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (8*V^2*prop_Cl_0*prop_sigma*cos(Alpha + b_4_scaled*gain_el)^2*log(prop_delta))/(Omega_4_scaled^2*gain_motor^2*prop_R^2) - (8*V*prop_Cl_a*prop_sigma*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1))/(Omega_4_scaled*gain_motor*prop_R))^(1/2); */
  /*     gamma2 = 0.1250*((16*V^2*sin(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2) - (prop_sigma*(prop_delta - 1)*(8*prop_Cl_0*prop_delta + 8*prop_Cl_0*prop_delta^2 + prop_Cl_a^2*prop_delta*prop_sigma - prop_Cl_a^2*prop_delta^2*prop_sigma + 16*prop_Cl_a*prop_delta*prop_theta + (8*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (8*V^2*prop_Cl_0*prop_sigma*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2) - (8*V*prop_Cl_a*prop_sigma*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1))/(Omega_3_scaled*gain_motor*prop_R))^(1/2); */
  /*     gamma3 = 0.1250*((16*V^2*sin(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2) - (prop_sigma*(prop_delta - 1)*(8*prop_Cl_0*prop_delta + 8*prop_Cl_0*prop_delta^2 + prop_Cl_a^2*prop_delta*prop_sigma - prop_Cl_a^2*prop_delta^2*prop_sigma + 16*prop_Cl_a*prop_delta*prop_theta + (8*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (8*V^2*prop_Cl_0*prop_sigma*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2) - (8*V*prop_Cl_a*prop_sigma*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1))/(Omega_2_scaled*gain_motor*prop_R))^(1/2); */
  /*     gamma4 = 0.1250*((16*V^2*sin(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2) - (prop_sigma*(prop_delta - 1)*(8*prop_Cl_0*prop_delta + 8*prop_Cl_0*prop_delta^2 + prop_Cl_a^2*prop_delta*prop_sigma - prop_Cl_a^2*prop_delta^2*prop_sigma + 16*prop_Cl_a*prop_delta*prop_theta + (8*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (8*V^2*prop_Cl_0*prop_sigma*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2) - (8*V*prop_Cl_a*prop_sigma*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1))/(Omega_1_scaled*gain_motor*prop_R))^(1/2); */
  /*     gamma5 = 0.1250*prop_Cl_a*prop_sigma*(prop_delta - 1); */
  /*     gamma6 = (prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma1 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_4_scaled*gain_el)^2)/(Omega_4_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)^2*log(prop_delta))/(Omega_4_scaled^2*gain_motor^2*prop_R^2); */
  /*     gamma7 = (0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_3_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma2 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta; */
  /*     gamma8 = (0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_3_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma2 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta; */
  /*     gamma9 = (0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_3_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma2 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta; */
  /*     gamma10 = (prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma2 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2); */
  /*       */
  /*     sigma_1 = dv_global_4 + (0.5118*Omega_1_scaled - 0.5118*Omega_2_scaled - 0.2842*b_1_scaled + 0.2842*b_2_scaled + 0.2441*b_3_scaled - 0.2441*b_4_scaled + 0.3656*Omega_1_scaled*b_1_scaled - 0.3656*Omega_2_scaled*b_2_scaled - 0.1899*Omega_1_scaled*b_4_scaled + 0.1899*Omega_2_scaled*b_3_scaled + l_1*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_1*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - 0.1567*b_1_scaled^2 + 0.1567*b_2_scaled^2 + 0.1780*b_3_scaled^2 - 0.1780*b_4_scaled^2 - I_yy*q*r + I_zz*q*r + (1.8530*Omega_1_scaled*V)/gain_airspeed - (1.8530*Omega_2_scaled*V)/gain_airspeed + (0.5481*V*b_1_scaled)/gain_airspeed - (0.5481*V*b_2_scaled)/gain_airspeed - 0.5000*CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho)/I_xx; */
  /*     sigma_2 = dv_global_5 + (l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*r - I_zz*p*r)/I_yy; */
  /*     sigma_3 = dv_global_6 - (l_1*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_1*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*q - I_yy*p*q)/I_zz; */
  /*     sigma_4 = (2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)); */
  /*     sigma_5 = (2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)); */
  /*     sigma_6 = (2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)); */
  /*     sigma_7 = (2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)); */
  /*     sigma_8 = (prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2); */
  /*     sigma_9 = (prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2); */
  /*     sigma_10 = 0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta); */
  /*     sigma_11 = 1/prop_delta; */
  /*     sigma_12 = 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el); */
  /*     sigma_13 = 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el); */
  /*     sigma_14 = ((V*prop_Cl_a*prop_delta*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor*prop_R) + (2*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^3*gain_motor^2*prop_R^2))*(prop_delta - 1) + (2*V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^3*gain_motor^2*prop_R^2); */
  /*     sigma_15 = ((V*prop_Cl_a*prop_delta*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor*prop_R) + (2*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^3*gain_motor^2*prop_R^2))*(prop_delta - 1) + (2*V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^3*gain_motor^2*prop_R^2); */
  /*      */
  /*     compute_gradient_and_cost = [ */
  /*       */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (2*W_dv_6^2*sigma_3*(l_4*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) + l_1*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*sin(b_1_scaled*gain_el) - 1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R)))))/I_zz + (2*W_dv_4^2*sigma_1*(0.3656*b_1_scaled - 0.1899*b_4_scaled + l_z*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) + (1.8530*V)/gain_airspeed + l_1*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) + 0.5118))/I_xx - (2*W_dv_5^2*sigma_2*(l_z*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*sin(b_1_scaled*gain_el) - 1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) - l_4*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R)))))/I_yy - 6*K_p_M*Omega_1_scaled^2*W_act_motor^2*gain_motor^2*gamma_quadratic_du*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1)*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1)) + (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*sin(b_1_scaled*gain_el) - 1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) - l_4*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (2*W_dv_6^2*sigma_3*(l_4*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) - l_1*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*sin(b_2_scaled*gain_el) - 1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R)))))/I_zz - (2*W_dv_4^2*sigma_1*(0.3656*b_2_scaled - 0.1899*b_3_scaled - l_z*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) + (1.8530*V)/gain_airspeed + l_1*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) + 0.5118))/I_xx - (2*W_dv_5^2*sigma_2*(l_z*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*sin(b_2_scaled*gain_el) - 1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) - l_4*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R)))))/I_yy - 6*K_p_M*Omega_2_scaled^2*W_act_motor^2*gain_motor^2*gamma_quadratic_du*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1)*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1)) + (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*sin(b_2_scaled*gain_el) - 1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) - l_4*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     (2*W_dv_5^2*sigma_2*(l_z*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R)))))/I_yy - (2*W_dv_4^2*sigma_1*(l_1*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))) - l_z*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R)))))/I_xx + (2*W_dv_6^2*sigma_3*(l_1*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R)))))/I_zz - 6*K_p_M*Omega_3_scaled^2*W_act_motor^2*gain_motor^2*gamma_quadratic_du*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1)*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1)) - (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             (2*W_dv_4^2*sigma_1*(l_1*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))) + l_z*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R)))))/I_xx + (2*W_dv_5^2*sigma_2*(l_z*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R)))))/I_yy - (2*W_dv_6^2*sigma_3*(l_1*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))) + l_3*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R)))))/I_zz - 6*K_p_M*Omega_4_scaled^2*W_act_motor^2*gain_motor^2*gamma_quadratic_du*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1)*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1)) - (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                   W_act_tilt_el^2*gamma_quadratic_du*(2*b_1_scaled - (2*desired_el_value)/gain_el) + (2*W_dv_6^2*sigma_3*(l_1*(0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_1_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)) + l_4*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az))))/I_zz + (2*W_dv_4^2*sigma_1*(0.3656*Omega_1_scaled - 0.3134*b_1_scaled + l_1*sigma_13 + l_z*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)) + (0.5481*V)/gain_airspeed - 0.2842))/I_xx - (2*W_dv_5^2*sigma_2*(l_z*(0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_1_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)) - l_4*sigma_13))/I_yy + (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_1_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)) - l_4*sigma_13)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 + 4*K_p_M*Omega_1_scaled^3*V*W_act_motor^2*gain_el*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                   W_act_tilt_el^2*gamma_quadratic_du*(2*b_2_scaled - (2*desired_el_value)/gain_el) - (2*W_dv_6^2*sigma_3*(l_1*(0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_2_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)) - l_4*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az))))/I_zz - (2*W_dv_4^2*sigma_1*(0.3656*Omega_2_scaled - 0.3134*b_2_scaled + l_1*sigma_12 - l_z*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)) + (0.5481*V)/gain_airspeed - 0.2842))/I_xx - (2*W_dv_5^2*sigma_2*(l_z*(0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_2_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)) - l_4*sigma_12))/I_yy + (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_2_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)) - l_4*sigma_12)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 + 4*K_p_M*Omega_2_scaled^3*V*W_act_motor^2*gain_el*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                                                                       W_act_tilt_el^2*gamma_quadratic_du*(2*b_3_scaled - (2*desired_el_value)/gain_el) + (2*W_dv_5^2*sigma_2*(l_3*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)) - l_z*(0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) - 0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el))))/I_yy + (2*W_dv_6^2*sigma_3*(l_3*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)) - l_1*(0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) - 0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el))))/I_zz + (2*W_dv_4^2*sigma_1*(0.1899*Omega_2_scaled + 0.3559*b_3_scaled + l_1*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)) - l_z*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)) + 0.2441))/I_xx - (S*V^2*W_dv_5^2*rho*wing_chord*(l_3*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)) - l_z*(0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) - 0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 + 4*K_p_M*Omega_3_scaled^3*V*W_act_motor^2*gain_el*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                                                                               W_act_tilt_el^2*gamma_quadratic_du*(2*b_4_scaled - (2*desired_el_value)/gain_el) + (2*W_dv_5^2*sigma_2*(l_3*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)) - l_z*(0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) - 0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el))))/I_yy + (2*W_dv_6^2*sigma_3*(l_3*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)) + l_1*(0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) - 0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el))))/I_zz - (2*W_dv_4^2*sigma_1*(0.1899*Omega_1_scaled + 0.3559*b_4_scaled + l_1*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)) + l_z*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)) + 0.2441))/I_xx - (S*V^2*W_dv_5^2*rho*wing_chord*(l_3*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)) - l_z*(0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) - 0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 + 4*K_p_M*Omega_4_scaled^3*V*W_act_motor^2*gain_el*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          W_act_tilt_az^2*gamma_quadratic_du*(2*g_1_scaled - (2*desired_az_value)/gain_az) + (2*W_dv_4^2*sigma_1*(l_z*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*Omega_1_scaled*gain_motor*prop_R^3) - l_1*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled*gain_motor*prop_R^3)))/I_xx + (2*W_dv_6^2*l_4*sigma_3*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*Omega_1_scaled*gain_motor*prop_R^3))/I_zz - (2*W_dv_5^2*l_4*sigma_2*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled*gain_motor*prop_R^3))/I_yy + (S*V^2*W_dv_5^2*l_4*rho*wing_chord*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled*gain_motor*prop_R^3)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          W_act_tilt_az^2*gamma_quadratic_du*(2*g_2_scaled - (2*desired_az_value)/gain_az) + (2*W_dv_4^2*sigma_1*(l_z*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*Omega_2_scaled*gain_motor*prop_R^3) + l_1*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled*gain_motor*prop_R^3)))/I_xx + (2*W_dv_6^2*l_4*sigma_3*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*Omega_2_scaled*gain_motor*prop_R^3))/I_zz - (2*W_dv_5^2*l_4*sigma_2*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled*gain_motor*prop_R^3))/I_yy + (S*V^2*W_dv_5^2*l_4*rho*wing_chord*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled*gain_motor*prop_R^3)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          W_act_tilt_az^2*gamma_quadratic_du*(2*g_3_scaled - (2*desired_az_value)/gain_az) + (2*W_dv_4^2*sigma_1*(l_z*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*Omega_3_scaled*gain_motor*prop_R^3) + l_1*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled*gain_motor*prop_R^3)))/I_xx - (2*W_dv_6^2*l_3*sigma_3*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*Omega_3_scaled*gain_motor*prop_R^3))/I_zz + (2*W_dv_5^2*l_3*sigma_2*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled*gain_motor*prop_R^3))/I_yy - (S*V^2*W_dv_5^2*l_3*rho*wing_chord*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled*gain_motor*prop_R^3)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               W_act_tilt_az^2*gamma_quadratic_du*(2*g_4_scaled - (2*desired_az_value)/gain_az) + (2*W_dv_4^2*sigma_1*(l_z*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*Omega_4_scaled*gain_motor*prop_R^3) - l_1*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled*gain_motor*prop_R^3)))/I_xx - (2*W_dv_6^2*l_3*sigma_3*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*Omega_4_scaled*gain_motor*prop_R^3))/I_zz + (2*W_dv_5^2*l_3*sigma_2*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled*gain_motor*prop_R^3))/I_yy - (S*V^2*W_dv_5^2*l_3*rho*wing_chord*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled*gain_motor*prop_R^3)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*     W_act_theta^2*gamma_quadratic_du*(2*Theta_scaled - (2*desired_theta_value)/gain_theta) - (2*W_dv_3^2*((sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gamma9*sin(Theta_scaled*gain_theta) + gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))/m - dv_global_3 + 9.8100)*(gain_theta*gamma9*cos(Theta_scaled*gain_theta) - gain_theta*cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta) - 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gain_theta*gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + gain_theta*sigma_10*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gain_theta*gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta)))/m + (2*W_dv_1^2*(dv_global_1 - (gamma9*cos(Theta_scaled*gain_theta) - cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))/m)*(cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gain_theta*sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gain_theta*gamma9*sin(Theta_scaled*gain_theta) + cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta) - 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gain_theta*gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gain_theta*sigma_10*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + gain_theta*gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi)))/m - (2*W_dv_2^2*sin(Phi_scaled*gain_phi)*(dv_global_2 + (gamma8*cos(Phi_scaled*gain_phi) + gamma7*sin(Phi_scaled*gain_phi) + sigma_10*sin(Phi_scaled*gain_phi))/m)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta) - 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)))/m + (0.5000*Cm_alpha*S^2*V^4*W_dv_5^2*gain_theta*rho^2*wing_chord^2*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 - (Cm_alpha*S*V^2*W_dv_5^2*gain_theta*rho*sigma_2*wing_chord)/I_yy + 4*K_p_M*Omega_1_scaled^3*V*W_act_motor^2*gain_theta*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1)) + 4*K_p_M*Omega_2_scaled^3*V*W_act_motor^2*gain_theta*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1)) + 4*K_p_M*Omega_3_scaled^3*V*W_act_motor^2*gain_theta*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1)) + 4*K_p_M*Omega_4_scaled^3*V*W_act_motor^2*gain_theta*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  gamma_quadratic_du*(2*Phi_scaled - (2*desired_phi_value)/gain_phi)*W_act_phi^2 + (2*(dv_global_1 - (gamma9*cos(Theta_scaled*gain_theta) - cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))/m)*(gain_phi*gamma8*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + gain_phi*gamma7*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + gain_phi*sigma_10*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))*W_dv_1^2)/m + (2*(dv_global_2 + (gamma8*cos(Phi_scaled*gain_phi) + gamma7*sin(Phi_scaled*gain_phi) + sigma_10*sin(Phi_scaled*gain_phi))/m)*(gain_phi*gamma7*cos(Phi_scaled*gain_phi) + gain_phi*sigma_10*cos(Phi_scaled*gain_phi) - gain_phi*gamma8*sin(Phi_scaled*gain_phi))*W_dv_2^2)/m - (2*W_dv_3^2*(gain_phi*gamma8*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + gain_phi*gamma7*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi) + gain_phi*sigma_10*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))*((sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gamma9*sin(Theta_scaled*gain_theta) + gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))/m - dv_global_3 + 9.8100))/m; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          gamma_quadratic_du*(2*delta_ailerons_scaled - (2*desired_ailerons_value)/gain_ailerons)*W_act_ailerons^2 - (CL_aileron*S*V^2*W_dv_4^2*gain_ailerons*rho*sigma_1)/I_xx; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               W_dv_3^2*((sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gamma9*sin(Theta_scaled*gain_theta) + gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))/m - dv_global_3 + 9.8100)^2 + W_dv_1^2*(dv_global_1 - (gamma9*cos(Theta_scaled*gain_theta) - cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))/m)^2 + W_dv_2^2*(dv_global_2 + (gamma8*cos(Phi_scaled*gain_phi) + gamma7*sin(Phi_scaled*gain_phi) + sigma_10*sin(Phi_scaled*gain_phi))/m)^2 + W_dv_4^2*sigma_1^2 + W_dv_5^2*sigma_2^2 + W_dv_6^2*sigma_3^2 + W_act_phi^2*gamma_quadratic_du*(Phi_scaled - desired_phi_value/gain_phi)^2 + W_act_theta^2*gamma_quadratic_du*(Theta_scaled - desired_theta_value/gain_theta)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_1_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_2_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_3_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_4_scaled - desired_el_value/gain_el)^2 + W_act_ailerons^2*gamma_quadratic_du*(delta_ailerons_scaled - desired_ailerons_value/gain_ailerons)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_1_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_2_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_3_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_4_scaled - desired_az_value/gain_az)^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1))^2 + (0.2500*S^2*V^4*W_dv_5^2*rho^2*wing_chord^2*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta)^2)/I_yy^2 - (S*V^2*W_dv_5^2*rho*sigma_2*wing_chord*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy]; */
  /*       */
  /*     cost = compute_gradient_and_cost(16); */
  /*     gradient = compute_gradient_and_cost(1:15); */
  /*  */
  /*     cost_parametric = W_dv_3^2*((sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gamma9*sin(Theta_scaled*gain_theta) + cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))/m - dv_global_3 + 9.8100)^2 + W_dv_5^2*(dv_global_5 + (l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*r - I_zz*p*r)/I_yy)^2 + W_dv_4^2*(dv_global_4 + (0.5118*Omega_1_scaled - 0.5118*Omega_2_scaled - 0.2842*b_1_scaled + 0.2842*b_2_scaled + 0.2441*b_3_scaled - 0.2441*b_4_scaled + 0.3656*Omega_1_scaled*b_1_scaled - 0.3656*Omega_2_scaled*b_2_scaled - 0.1899*Omega_1_scaled*b_4_scaled + 0.1899*Omega_2_scaled*b_3_scaled + l_1*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_1*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - 0.1567*b_1_scaled^2 + 0.1567*b_2_scaled^2 + 0.1780*b_3_scaled^2 - 0.1780*b_4_scaled^2 - I_yy*q*r + I_zz*q*r + (1.8530*Omega_1_scaled*V)/gain_airspeed - (1.8530*Omega_2_scaled*V)/gain_airspeed + (0.5481*V*b_1_scaled)/gain_airspeed - (0.5481*V*b_2_scaled)/gain_airspeed - 0.5000*CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho)/I_xx)^2 + W_dv_1^2*(dv_global_1 - (gamma9*cos(Theta_scaled*gain_theta) - cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))/m)^2 + W_dv_2^2*(dv_global_2 + (sin(Phi_scaled*gain_phi)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma8*cos(Phi_scaled*gain_phi) + gamma7*sin(Phi_scaled*gain_phi))/m)^2 + W_dv_6^2*(dv_global_6 - (l_1*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_1*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*q - I_yy*p*q)/I_zz)^2 + W_act_phi^2*gamma_quadratic_du*(Phi_scaled - desired_phi_value/gain_phi)^2 + W_act_theta^2*gamma_quadratic_du*(Theta_scaled - desired_theta_value/gain_theta)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_1_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_2_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_3_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_4_scaled - desired_el_value/gain_el)^2 + W_act_ailerons^2*gamma_quadratic_du*(delta_ailerons_scaled - desired_ailerons_value/gain_ailerons)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_1_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_2_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_3_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_4_scaled - desired_az_value/gain_az)^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1))^2 + (0.2500*S^2*V^4*W_dv_5^2*rho^2*wing_chord^2*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta)^2)/I_yy^2 - (S*V^2*W_dv_5^2*rho*wing_chord*(dv_global_5 + (l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*r - I_zz*p*r)/I_yy)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy; */
  /*     disp(cost - cost_parametric) */
  /*  */
  /* end */
  /*     %} */
  /* COMPUTE_COST_AND_GRADIENT_FCN */
  /*     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT_FCN(Alpha,Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AIRSPEED,GAIN_AILERONS,GAMMA_QUADRATIC_DU,K_TILT,L_1,L_3,L_4,L_Z,M,P,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,Q,R,RHO,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 9.3. */
  /*     16-Nov-2023 16:59:28 */
  t165 = u_in[12] * gain_theta->contents;
  Alpha = t165 - flight_path_angle->contents;
  t2 = cos(Beta->contents);
  t3 = sin(Beta->contents);
  t4 = log(prop_delta->contents);
  t5 = u_in[13] * gain_phi->contents;
  t7 = u_in[4] * gain_el->contents;
  t8 = u_in[5] * gain_el->contents;
  t9 = u_in[6] * gain_el->contents;
  t10 = u_in[7] * gain_el->contents;
  t11 = u_in[8] * gain_az->contents;
  t12 = u_in[9] * gain_az->contents;
  t13 = u_in[10] * gain_az->contents;
  t14 = u_in[11] * gain_az->contents;
  a = Cl_alpha->contents;
  t15 = a * a;
  t16 = u_in[0] * u_in[0];
  t17 = rt_powd_snf(u_in[0], 3.0);
  t18 = u_in[1] * u_in[1];
  t19 = rt_powd_snf(u_in[1], 3.0);
  t20 = u_in[2] * u_in[2];
  t21 = rt_powd_snf(u_in[2], 3.0);
  t22 = u_in[3] * u_in[3];
  t23 = rt_powd_snf(u_in[3], 3.0);
  a = V->contents;
  t24 = a * a;
  a = W_act_phi->contents;
  b_a = W_act_theta->contents;
  c_a = W_act_motor->contents;
  d_a = W_dv_1->contents;
  e_a = W_dv_2->contents;
  f_a = W_dv_3->contents;
  g_a = W_dv_4->contents;
  h_a = W_dv_5->contents;
  i_a = W_dv_6->contents;
  j_a = W_act_tilt_el->contents;
  k_a = W_act_tilt_az->contents;
  l_a = W_act_ailerons->contents;
  t89 = gain_motor->contents;
  t41 = t89 * t89;
  t42 = prop_Cd_a->contents * 2.0;
  t44 = rt_powd_snf(prop_R->contents, 3.0);
  t45 = rt_powd_snf(prop_R->contents, 4.0);
  t46 = prop_Cd_0->contents * prop_delta->contents * 2.0;
  t77 = 1.0 / u_in[0];
  t79 = 1.0 / u_in[1];
  t82 = 1.0 / u_in[2];
  t85 = 1.0 / u_in[3];
  t89 = -flight_path_angle->contents;
  t94 = 1.0 / gain_motor->contents;
  t96 = 1.0 / gain_airspeed->contents;
  t100 = -prop_theta->contents;
  t101 = prop_delta->contents * 16.0;
  t102 = prop_delta->contents - 1.0;
  t103 = 1.0 / prop_R->contents;
  t105 = 1.0 / prop_delta->contents;
  t47 = prop_theta->contents * t42;
  t48 = cos(t5);
  t49 = cos(t165);
  t52 = sin(t5);
  t57 = cos(t7);
  t58 = cos(t8);
  t59 = cos(t9);
  t60 = cos(t10);
  t61 = sin(t165);
  t62 = cos(t11);
  t63 = cos(t12);
  t64 = cos(t13);
  t65 = cos(t14);
  t66 = sin(t7);
  t67 = sin(t8);
  t68 = sin(t9);
  t69 = sin(t10);
  t70 = sin(t11);
  t71 = sin(t12);
  t72 = sin(t13);
  t73 = sin(t14);
  t78 = 1.0 / t16;
  t80 = 1.0 / t17;
  t81 = 1.0 / t18;
  t83 = 1.0 / t19;
  t84 = 1.0 / t20;
  t86 = 1.0 / t21;
  t87 = 1.0 / t22;
  t88 = 1.0 / t23;
  t95 = 1.0 / t41;
  t104 = t103 * t103;
  t14 = prop_Cl_0->contents * prop_delta->contents;
  t108 = t14 * (prop_delta->contents + 1.0);
  t110 = desired_el_value->contents * (1.0 / gain_el->contents);
  t111 = desired_az_value->contents * (1.0 / gain_az->contents);
  t112 = desired_motor_value->contents * t94;
  t5 = (Alpha + t7) + 1.5707963267948966;
  t11 = (Alpha + t8) + 1.5707963267948966;
  t12 = (Alpha + t9) + 1.5707963267948966;
  t13 = (Alpha + t10) + 1.5707963267948966;
  t133 = prop_delta->contents * prop_sigma->contents * -prop_Cl_a->contents *
    t102;
  t520 = prop_Cl_a->contents * prop_sigma->contents;
  t134 = t520 * t102 / 8.0;
  t135 = (t165 + t7) + t89;
  t136 = (t165 + t8) + t89;
  t137 = (t165 + t9) + t89;
  t138 = (t165 + t10) + t89;
  t89 = V->contents * t96;
  t189 = t89 * 1.853;
  t202 = t89 * 0.54813;
  t117 = flight_path_angle->contents - t165;
  Alpha = t108 * 8.0;
  t140 = cos(t5);
  t141 = cos(t11);
  t142 = cos(t12);
  t143 = cos(t13);
  t144 = sin(t5);
  t145 = sin(t11);
  t146 = sin(t12);
  t147 = sin(t13);
  t152 = cos(t135);
  t153 = cos(t136);
  t154 = cos(t137);
  t155 = cos(t138);
  t119 = cos(t117);
  t120 = sin(t117);
  t89 = t140 * t140;
  t165 = t141 * t141;
  t166 = t142 * t142;
  t167 = t143 * t143;
  t168 = t144 * t144;
  t169 = t145 * t145;
  t170 = t146 * t146;
  t171 = t147 * t147;
  t5 = V->contents * k_tilt->contents;
  t197 = t5 * (t152 * t152);
  t198 = t5 * (t153 * t153);
  t199 = t5 * (t154 * t154);
  t200 = t5 * (t155 * t155);
  t11 = Cl_alpha->contents * S->contents;
  t207_tmp = t11 * gain_theta->contents * rho->contents * t24;
  t207 = t207_tmp * t119 / 2.0;
  t259_tmp = prop_Cl_a->contents * prop_theta->contents * t24;
  t283_tmp_tmp = t14 * t4;
  t283_tmp = t283_tmp_tmp * t24;
  t193 = Cd_zero->contents + K_Cd->contents * t15 * (t117 * t117);
  t287_tmp_tmp = S->contents * rho->contents;
  t5 = t287_tmp_tmp * t2 * t24;
  t287 = t5 * t120 * t193 / 2.0;
  t302 = t5 * t119 * t193 / 2.0;
  t5 = t11 * rho->contents * t24 * t117;
  t367 = t5 * t119 / 2.0 + t287;
  t368 = t5 * t120 / 2.0 - t302;
  t5 = t24 * t78 * t95 * t104;
  t7 = V->contents * prop_Cl_a->contents * prop_sigma->contents;
  t8 = prop_Cl_0->contents * prop_sigma->contents * t4 * t24;
  t11 = prop_sigma->contents * t102 * t105;
  t12 = sqrt(((t5 * t89 * 16.0 + t7 * t77 * t94 * t102 * t103 * t140 * 8.0) + t8
              * t78 * t95 * t104 * t168 * -8.0) - t11 * (Alpha +
              prop_Cl_a->contents * (t133 + prop_theta->contents * (t101 + t5 *
    t168 * 8.0))));
  t5 = t24 * t81 * t95 * t104;
  t13 = sqrt(((t5 * t165 * 16.0 + t7 * t79 * t94 * t102 * t103 * t141 * 8.0) +
              t8 * t81 * t95 * t104 * t169 * -8.0) - t11 * (Alpha +
              prop_Cl_a->contents * (t133 + prop_theta->contents * (t101 + t5 *
    t169 * 8.0))));
  t5 = t24 * t84 * t95 * t104;
  t14 = sqrt(((t5 * t166 * 16.0 + t7 * t82 * t94 * t102 * t103 * t142 * 8.0) +
              t8 * t84 * t95 * t104 * t170 * -8.0) - t11 * (Alpha +
              prop_Cl_a->contents * (t133 + prop_theta->contents * (t101 + t5 *
    t170 * 8.0))));
  t5 = t24 * t87 * t95 * t104;
  t5 = sqrt(((t5 * t167 * 16.0 + t7 * t85 * t94 * t102 * t103 * t143 * 8.0) + t8
             * t87 * t95 * t104 * t171 * -8.0) - t11 * (Alpha +
             prop_Cl_a->contents * (t133 + prop_theta->contents * (t101 + t5 *
    t171 * 8.0))));
  Alpha = 1.0 / t12;
  t9 = 1.0 / t13;
  t10 = 1.0 / t14;
  t404 = 1.0 / t5;
  t409 = (t134 + V->contents * t77 * t94 * t103 * t140 / 2.0) + t12 / 8.0;
  t410 = (t134 + V->contents * t79 * t94 * t103 * t141 / 2.0) + t13 / 8.0;
  t411 = (t134 + V->contents * t82 * t94 * t103 * t142 / 2.0) + t14 / 8.0;
  t412 = (t134 + V->contents * t85 * t94 * t103 * t143 / 2.0) + t5 / 8.0;
  t5 = t520 * prop_theta->contents * t24;
  t517 = V->contents * t78 * t94 * t103 * t140 / 2.0 + (((t24 * t80 * t95 * t104
    * t89 * 32.0 + t7 * t78 * t94 * t102 * t103 * t140 * 8.0) - t8 * t80 * t95 *
    t104 * t168 * 16.0) - t5 * t80 * t95 * t102 * t104 * t105 * t168 * 16.0) *
    Alpha / 16.0;
  t518 = V->contents * t81 * t94 * t103 * t141 / 2.0 + (((t24 * t83 * t95 * t104
    * t165 * 32.0 + t7 * t81 * t94 * t102 * t103 * t141 * 8.0) - t8 * t83 * t95 *
    t104 * t169 * 16.0) - t5 * t83 * t95 * t102 * t104 * t105 * t169 * 16.0) *
    t9 / 16.0;
  t519 = V->contents * t84 * t94 * t103 * t142 / 2.0 + (((t24 * t86 * t95 * t104
    * t166 * 32.0 + t7 * t84 * t94 * t102 * t103 * t142 * 8.0) - t8 * t86 * t95 *
    t104 * t170 * 16.0) - t5 * t86 * t95 * t102 * t104 * t105 * t170 * 16.0) *
    t10 / 16.0;
  t520 = V->contents * t87 * t94 * t103 * t143 / 2.0 + (((t24 * t88 * t95 * t104
    * t167 * 32.0 + t7 * t87 * t94 * t102 * t103 * t143 * 8.0) - t8 * t88 * t95 *
    t104 * t171 * 16.0) - t5 * t88 * t95 * t102 * t104 * t105 * t171 * 16.0) *
    t404 / 16.0;
  t5 = V->contents * gain_el->contents;
  t11 = t5 * prop_Cl_a->contents * prop_sigma->contents;
  t12 = gain_el->contents * t24;
  t537_tmp_tmp = gain_el->contents * prop_Cl_0->contents;
  t13 = t537_tmp_tmp * prop_sigma->contents * t4 * t24;
  b_t537_tmp_tmp = gain_el->contents * prop_Cl_a->contents;
  t14 = b_t537_tmp_tmp * prop_sigma->contents * prop_theta->contents * t24;
  t77 = t5 * t77 * t94 * t103 * t144 / 2.0 + (((t11 * t77 * t94 * t102 * t103 *
    t144 * 8.0 + t12 * t78 * t95 * t104 * t140 * t144 * 32.0) + t13 * t78 * t95 *
    t104 * t140 * t144 * 16.0) + t14 * t78 * t95 * t102 * t104 * t105 * t140 *
    t144 * 16.0) * Alpha / 16.0;
  t134 = t5 * t79 * t94 * t103 * t145 / 2.0 + (((t11 * t79 * t94 * t102 * t103 *
    t145 * 8.0 + t12 * t81 * t95 * t104 * t141 * t145 * 32.0) + t13 * t81 * t95 *
    t104 * t141 * t145 * 16.0) + t14 * t81 * t95 * t102 * t104 * t105 * t141 *
    t145 * 16.0) * t9 / 16.0;
  t101 = t5 * t82 * t94 * t103 * t146 / 2.0 + (((t11 * t82 * t94 * t102 * t103 *
    t146 * 8.0 + t12 * t84 * t95 * t104 * t142 * t146 * 32.0) + t13 * t84 * t95 *
    t104 * t142 * t146 * 16.0) + t14 * t84 * t95 * t102 * t104 * t105 * t142 *
    t146 * 16.0) * t10 / 16.0;
  t404 = t5 * t85 * t94 * t103 * t147 / 2.0 + (((t11 * t85 * t94 * t102 * t103 *
    t147 * 8.0 + t12 * t87 * t95 * t104 * t143 * t147 * 32.0) + t13 * t87 * t95 *
    t104 * t143 * t147 * 16.0) + t14 * t87 * t95 * t102 * t104 * t105 * t143 *
    t147 * 16.0) * t404 / 16.0;
  t166 = prop_Cl_a->contents * prop_delta->contents;
  expl_temp.f178 = 1.0 / m->contents;
  expl_temp.f177 = 1.0 / gain_ailerons->contents;
  expl_temp.f176 = 1.0 / gain_theta->contents;
  expl_temp.f175 = 1.0 / gain_phi->contents;
  expl_temp.f174 = 1.0 / I_zz->contents;
  expl_temp.f173 = 1.0 / I_yy->contents;
  expl_temp.f172 = 1.0 / I_xx->contents;
  expl_temp.f171 = t73;
  t167 = prop_theta->contents * t102;
  t13 = prop_Cl_a->contents - t42;
  expl_temp.f170 = t283_tmp_tmp * t404 + t167 * t404 * t13;
  expl_temp.f169 = t283_tmp_tmp * t101 + t167 * t101 * t13;
  expl_temp.f168 = t283_tmp_tmp * t134 + t167 * t134 * t13;
  expl_temp.f167 = t283_tmp_tmp * t77 + t167 * t77 * t13;
  t14 = u_in[3] * V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * t44;
  Alpha = t283_tmp_tmp * t520 + t167 * t520 * t13;
  expl_temp.f166 = t14 * t69 * t73 * t105 * t147 * 3.1415926535897931 * Alpha /
    4.0;
  t7 = u_in[2] * V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * t44;
  t8 = t283_tmp_tmp * t519 + t167 * t519 * t13;
  expl_temp.f165 = t7 * t68 * t72 * t105 * t146 * 3.1415926535897931 * t8 / 4.0;
  t9 = u_in[1] * V->contents;
  t89 = t9 * gain_motor->contents * prop_sigma->contents * rho->contents * t44;
  t10 = t283_tmp_tmp * t518 + t167 * t518 * t13;
  expl_temp.f164 = t89 * t67 * t71 * t105 * t145 * 3.1415926535897931 * t10 /
    4.0;
  t165 = u_in[0] * V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * t44;
  t167 = t283_tmp_tmp * t517 + t167 * t517 * t13;
  expl_temp.f163 = t165 * t66 * t70 * t105 * t144 * 3.1415926535897931 * t167 /
    4.0;
  expl_temp.f162 = t14 * t65 * t69 * t105 * t147 * 3.1415926535897931 * Alpha /
    4.0;
  expl_temp.f161 = t72;
  expl_temp.f160 = t7 * t64 * t68 * t105 * t146 * 3.1415926535897931 * t8 / 4.0;
  expl_temp.f159 = t89 * t63 * t67 * t105 * t145 * 3.1415926535897931 * t10 /
    4.0;
  expl_temp.f158 = t165 * t62 * t66 * t105 * t144 * 3.1415926535897931 * t167 /
    4.0;
  expl_temp.f157 = t71;
  expl_temp.f156 = t14 * t60 * t105 * t147 * 3.1415926535897931 * Alpha * -0.25;
  expl_temp.f155 = t7 * t59 * t105 * t146 * 3.1415926535897931 * t8 * -0.25;
  expl_temp.f154 = t89 * t58 * t105 * t145 * 3.1415926535897931 * t10 * -0.25;
  expl_temp.f153 = t165 * t57 * t105 * t144 * 3.1415926535897931 * t167 * -0.25;
  expl_temp.f152 = t70;
  expl_temp.f151 = t69;
  expl_temp.f150 = t68;
  expl_temp.f149 = t67;
  expl_temp.f148 = t66;
  expl_temp.f147 = t65;
  expl_temp.f146 = t64;
  expl_temp.f145 = t63;
  expl_temp.f144 = t62;
  expl_temp.f143 = t61;
  expl_temp.f142 = t60;
  expl_temp.f141 = t59;
  t167 = prop_sigma->contents * rho->contents;
  t133 = t167 * t22 * t41 * t45;
  t14 = t133 * t60;
  Alpha = t283_tmp * t88 * t95 * t104 * t171 * 2.0 + t102 * (t259_tmp * t88 *
    t95 * t104 * t171 * 2.0 - t166 * t520 * 2.0);
  expl_temp.f140 = t14 * t73 * t105 * 3.1415926535897931 * Alpha * -0.25;
  t12 = t167 * t20 * t41 * t45;
  t7 = t12 * t59;
  t8 = t283_tmp * t86 * t95 * t104 * t170 * 2.0 + t102 * (t259_tmp * t86 * t95 *
    t104 * t170 * 2.0 - t166 * t519 * 2.0);
  expl_temp.f139 = t7 * t72 * t105 * 3.1415926535897931 * t8 * -0.25;
  t11 = t167 * t18 * t41 * t45;
  t89 = t11 * t58;
  t10 = t283_tmp * t83 * t95 * t104 * t169 * 2.0 + t102 * (t259_tmp * t83 * t95 *
    t104 * t169 * 2.0 - t166 * t518 * 2.0);
  expl_temp.f138 = t89 * t71 * t105 * 3.1415926535897931 * t10 * -0.25;
  t5 = t167 * t16 * t41 * t45;
  t167 = t5 * t57;
  t165 = t283_tmp * t80 * t95 * t104 * t168 * 2.0 + t102 * (t259_tmp * t80 * t95
    * t104 * t168 * 2.0 - t166 * t517 * 2.0);
  expl_temp.f137 = t167 * t70 * t105 * 3.1415926535897931 * t165 * -0.25;
  expl_temp.f136 = t14 * t65 * t105 * 3.1415926535897931 * Alpha * -0.25;
  expl_temp.f135 = t7 * t64 * t105 * 3.1415926535897931 * t8 * -0.25;
  expl_temp.f134 = t89 * t63 * t105 * 3.1415926535897931 * t10 * -0.25;
  expl_temp.f133 = t167 * t62 * t105 * 3.1415926535897931 * t165 * -0.25;
  expl_temp.f132 = t133 * t69 * t105 * 3.1415926535897931 * Alpha * -0.25;
  expl_temp.f131 = t58;
  expl_temp.f130 = t12 * t68 * t105 * 3.1415926535897931 * t8 * -0.25;
  expl_temp.f129 = t11 * t67 * t105 * 3.1415926535897931 * t10 * -0.25;
  expl_temp.f128 = t5 * t66 * t105 * 3.1415926535897931 * t165 * -0.25;
  t167 = b_t537_tmp_tmp * prop_theta->contents * t24;
  expl_temp.f127 = t102 * (t167 * t81 * t95 * t104 * t141 * t145 * 2.0 + t166 *
    t134 * 2.0);
  expl_temp.f126 = t102 * (t167 * t78 * t95 * t104 * t140 * t144 * 2.0 + t166 *
    t77 * 2.0);
  expl_temp.f125 = t57;
  expl_temp.f124 = t167 * t87 * t95 * t104 * t143 * t147 * 2.0 + t166 * t404 *
    2.0;
  expl_temp.f123 = t167 * t84 * t95 * t104 * t142 * t146 * 2.0 + t166 * t101 *
    2.0;
  expl_temp.f122 = I_zz->contents * q->contents * r->contents;
  t167 = I_xx->contents * p->contents;
  expl_temp.f121 = t167 * r->contents;
  expl_temp.f120 = t52;
  expl_temp.f119 = t167 * q->contents;
  expl_temp.f118 = t49;
  expl_temp.f117 = t48;
  expl_temp.f116 = (t108 + t259_tmp * t87 * t95 * t104 * t171) - t166 * (t100 +
    t412) * 2.0;
  expl_temp.f115 = (t108 + t259_tmp * t84 * t95 * t104 * t170) - t166 * (t100 +
    t411) * 2.0;
  expl_temp.f114 = (t108 + t259_tmp * t81 * t95 * t104 * t169) - t166 * (t100 +
    t410) * 2.0;
  expl_temp.f113 = (t108 + t259_tmp * t78 * t95 * t104 * t168) - t166 * (t100 +
    t409) * 2.0;
  expl_temp.f112 = t45;
  expl_temp.f111 = t46 + prop_theta->contents * (t47 + t412 * t13);
  expl_temp.f110 = t46 + prop_theta->contents * (t47 + t411 * t13);
  expl_temp.f109 = t46 + prop_theta->contents * (t47 + t410 * t13);
  expl_temp.f108 = t46 + prop_theta->contents * (t47 + t409 * t13);
  expl_temp.f107 = t44;
  expl_temp.f106 = t283_tmp_tmp * t412;
  expl_temp.f105 = t283_tmp_tmp * t411;
  expl_temp.f104 = t283_tmp_tmp * t410;
  expl_temp.f103 = t283_tmp_tmp * t409;
  expl_temp.f102 = t41;
  t167 = K_Cd->contents * S->contents * gain_theta->contents * rho->contents *
    t2 * t15 * t24 * t117;
  expl_temp.f101 = ((t207_tmp * t120 / 2.0 + t117 * t207) - t167 * t119) +
    gain_theta->contents * t287;
  expl_temp.f100 = ((t207 + t207_tmp * t117 * t120 * -0.5) + t167 * t120) +
    gain_theta->contents * t302;
  expl_temp.f99 = -(t48 * t49 * t367);
  expl_temp.f98 = t48 * t61 * t367;
  expl_temp.f97 = t61 * t368;
  expl_temp.f96 = t49 * t368;
  expl_temp.f95 = t52 * t367;
  expl_temp.f94 = t367;
  expl_temp.f93 = l_a * l_a;
  expl_temp.f92 = k_a * k_a;
  t167 = t537_tmp_tmp * prop_delta->contents * t4 * t24;
  expl_temp.f91 = t167 * t87 * t95 * t104 * t143 * t147 * 2.0;
  expl_temp.f90 = t167 * t84 * t95 * t104 * t142 * t146 * 2.0;
  expl_temp.f89 = t167 * t81 * t95 * t104 * t141 * t145 * 2.0;
  expl_temp.f88 = t167 * t78 * t95 * t104 * t140 * t144 * 2.0;
  expl_temp.f87 = j_a * j_a;
  expl_temp.f86 = i_a * i_a;
  t167 = t287_tmp_tmp * t3 * t24;
  expl_temp.f85 = -(t167 * t52 * t61 * t193 / 2.0);
  expl_temp.f84 = h_a * h_a;
  expl_temp.f83 = t112 - K_p_M->contents * t23 * t41 * (t200 + 1.0);
  expl_temp.f82 = t112 - K_p_M->contents * t21 * t41 * (t199 + 1.0);
  expl_temp.f81 = t112 - K_p_M->contents * t19 * t41 * (t198 + 1.0);
  expl_temp.f80 = t112 - K_p_M->contents * t17 * t41 * (t197 + 1.0);
  expl_temp.f79 = g_a * g_a;
  expl_temp.f78 = t167 * t49 * t52 * t193 / 2.0;
  expl_temp.f77 = f_a * f_a;
  expl_temp.f76 = t3;
  expl_temp.f75 = e_a * e_a;
  expl_temp.f74 = d_a * d_a;
  expl_temp.f73 = c_a * c_a;
  expl_temp.f72 = t283_tmp * t87 * t95 * t104 * t171;
  expl_temp.f71 = t283_tmp * t84 * t95 * t104 * t170;
  expl_temp.f70 = t283_tmp * t81 * t95 * t104 * t169;
  expl_temp.f69 = t283_tmp * t78 * t95 * t104 * t168;
  expl_temp.f68 = b_a * b_a;
  expl_temp.f67 = a * a;
  expl_temp.f66 = t167 * t48 * t193 / 2.0;
  expl_temp.f65 = t24;
  expl_temp.f64 = t23;
  expl_temp.f63 = t22;
  expl_temp.f62 = -(t287_tmp_tmp * t24 * (Cm_zero->contents - Cm_alpha->contents
    * t117) * wing_chord->contents / 2.0);
  expl_temp.f61 = V->contents * u_in[5] * t96 * -0.54813;
  expl_temp.f60 = t21;
  expl_temp.f59 = u_in[4] * t202;
  expl_temp.f58 = t200 + 1.0;
  expl_temp.f57 = t199 + 1.0;
  expl_temp.f56 = t198 + 1.0;
  expl_temp.f55 = t197 + 1.0;
  expl_temp.f54 = t202;
  expl_temp.f53 = t9 * t96 * -1.853;
  expl_temp.f52 = t20;
  expl_temp.f51 = -(u_in[7] * u_in[7] * 0.17797);
  expl_temp.f50 = u_in[0] * t189;
  expl_temp.f49 = t193;
  expl_temp.f48 = u_in[6] * u_in[6] * 0.17797;
  expl_temp.f47 = -(u_in[1] * 0.51177);
  expl_temp.f46 = t19;
  expl_temp.f45 = t189;
  expl_temp.f44 = -(u_in[4] * u_in[4] * 0.15672);
  expl_temp.f43 = -(u_in[1] * u_in[5] * 0.36558);
  expl_temp.f42 = u_in[0] * 0.51177;
  expl_temp.f41 = t18;
  expl_temp.f40 = u_in[5] * u_in[5] * 0.15672;
  expl_temp.f39 = u_in[0] * u_in[4] * 0.36558;
  expl_temp.f38 = -(u_in[0] * u_in[7] * 0.18985);
  expl_temp.f37 = -(CL_aileron->contents * S->contents * u_in[14] *
                    gain_ailerons->contents * rho->contents * t24 / 2.0);
  expl_temp.f36 = -(u_in[4] * 0.28424);
  expl_temp.f35 = -(u_in[7] * 0.24408);
  expl_temp.f34 = t17;
  expl_temp.f33 = u_in[1] * u_in[6] * 0.18985;
  expl_temp.f32 = t16;
  expl_temp.f31 = sin(t138);
  expl_temp.f30 = sin(t137);
  expl_temp.f29 = sin(t136);
  expl_temp.f28 = sin(t135);
  expl_temp.f27 = t155;
  expl_temp.f26 = t154;
  expl_temp.f25 = t153;
  expl_temp.f24 = t152;
  expl_temp.f23 = u_in[5] * 0.28424;
  expl_temp.f22 = t15;
  expl_temp.f21 = u_in[6] * 0.24408;
  expl_temp.f20 = t147;
  expl_temp.f19 = t146;
  expl_temp.f18 = t145;
  expl_temp.f17 = t144;
  expl_temp.f16 = t143;
  expl_temp.f15 = t142;
  expl_temp.f14 = t141;
  expl_temp.f13 = t140;
  expl_temp.f12 = -(t111 * 2.0);
  expl_temp.f11 = -t111;
  expl_temp.f10 = -(t110 * 2.0);
  expl_temp.f9 = -t110;
  expl_temp.f8 = t117;
  expl_temp.f7 = -(I_yy->contents * q->contents * r->contents);
  expl_temp.f6 = -(I_zz->contents * p->contents * r->contents);
  expl_temp.f5 = -(I_yy->contents * p->contents * q->contents);
  expl_temp.f4 = 1.5707963267948966;
  expl_temp.f3 = t105;
  expl_temp.f2 = t102;
  memcpy(&expl_temp.f1[0], &u_in[0], 15U * sizeof(double));
  return b_ft_1(dv_global, V, gain_motor, prop_sigma, rho, l_1, l_4, l_3, l_z,
                gamma_quadratic_du, desired_phi_value, desired_theta_value,
                desired_ailerons_value, &expl_temp);
}

static void b_driver(const double lb[15], const double ub[15], g_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     i_coder_internal_stickyStruct *FcnEvaluator, d_struct_T
                     *memspace, h_struct_T *WorkingSet, double Hessian[225],
                     e_struct_T *QRManager, f_struct_T *CholManager, struct_T
                     *QPObjective)
{
  static const signed char iv[225] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1 };

  static const char qpoptions_SolverName[7] = { 'f', 'm', 'i', 'n', 'c', 'o',
    'n' };

  i_struct_T b_expl_temp;
  i_struct_T expl_temp;
  j_struct_T Flags;
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
  memset(&QPObjective->grad[0], 0, 16U * sizeof(double));
  memset(&QPObjective->Hx[0], 0, 15U * sizeof(double));
  QPObjective->hasLinear = true;
  QPObjective->nvar = 15;
  QPObjective->maxVar = 16;
  QPObjective->beta = 0.0;
  QPObjective->rho = 0.0;
  QPObjective->objtype = 3;
  QPObjective->prev_objtype = 3;
  QPObjective->prev_nvar = 0;
  QPObjective->prev_hasLinear = false;
  QPObjective->gammaScalar = 0.0;
  CholManager->ldm = 31;
  CholManager->ndims = 0;
  CholManager->info = 0;
  CholManager->scaleFactor = 0.0;
  CholManager->ConvexCheck = true;
  CholManager->regTol_ = rtInf;
  CholManager->workspace_ = rtInf;
  CholManager->workspace2_ = rtInf;
  QRManager->ldq = 31;
  memset(&CholManager->FMat[0], 0, 961U * sizeof(double));
  memset(&QRManager->QR[0], 0, 961U * sizeof(double));
  memset(&QRManager->Q[0], 0, 961U * sizeof(double));
  QRManager->mrows = 0;
  QRManager->ncols = 0;
  memset(&QRManager->jpvt[0], 0, 31U * sizeof(int));
  memset(&QRManager->tau[0], 0, 31U * sizeof(double));
  QRManager->minRowCol = 0;
  QRManager->usedPivoting = false;
  for (i = 0; i < 225; i++) {
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
  for (k = 0; k < 15; k++) {
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
          if (TrialState->FunctionEvaluations < 1000) {
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
                if (1.0E-9 * fmax(1.0, fabs(TrialState->xstarsqp[ixlast])) <=
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
      for (k = 0; k < 15; k++) {
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
      memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0], 15U *
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

static double b_ft_1(const b_captured_var *dv_global, const captured_var *V,
                     const captured_var *gain_motor, const captured_var
                     *prop_sigma, const captured_var *rho, const captured_var
                     *l_1, const captured_var *l_4, const captured_var *l_3,
                     const captured_var *l_z, const captured_var
                     *gamma_quadratic_du, const captured_var *desired_phi_value,
                     const captured_var *desired_theta_value, const captured_var
                     *desired_ailerons_value, const cell_18 *ct)
{
  double b_t769_tmp;
  double t461;
  double t462;
  double t463;
  double t464;
  double t589;
  double t590;
  double t591;
  double t592;
  double t753;
  double t754;
  double t756;
  double t767;
  double t767_tmp_tmp;
  double t768;
  double t769;
  double t769_tmp;
  double t769_tmp_tmp;
  double t770;
  double t770_tmp;
  double t770_tmp_tmp;
  double t771;
  double t772;
  double t773;
  double t774;
  double t829;
  double t832;
  double t833;
  t461 = ct->f69 + ct->f2 * ct->f113;
  t462 = ct->f70 + ct->f2 * ct->f114;
  t463 = ct->f71 + ct->f2 * ct->f115;
  t464 = ct->f72 + ct->f2 * ct->f116;
  t589 = ct->f103 + ct->f2 * ct->f108;
  t590 = ct->f104 + ct->f2 * ct->f109;
  t591 = ct->f105 + ct->f2 * ct->f110;
  t592 = ct->f106 + ct->f2 * ct->f111;
  t756 = prop_sigma->contents * rho->contents;
  t767_tmp_tmp = t756 * ct->f32 * ct->f102 * ct->f112;
  t832 = t767_tmp_tmp * ct->f125;
  t833 = ct->f1[0] * V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * ct->f107;
  t767 = t832 * ct->f152 * ct->f3 * t461 * 3.1415926535897931 / 4.0 + t833 *
    ct->f148 * ct->f152 * ct->f3 * ct->f17 * t589 * 3.1415926535897931 * -0.25;
  t754 = t756 * ct->f41 * ct->f102 * ct->f112;
  t829 = t754 * ct->f131;
  t773 = ct->f1[1] * V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * ct->f107;
  t768 = t829 * ct->f157 * ct->f3 * t462 * 3.1415926535897931 / 4.0 + t773 *
    ct->f149 * ct->f157 * ct->f3 * ct->f18 * t590 * 3.1415926535897931 * -0.25;
  t769_tmp_tmp = t756 * ct->f52 * ct->f102 * ct->f112;
  t769_tmp = t769_tmp_tmp * ct->f141;
  b_t769_tmp = ct->f1[2] * V->contents * gain_motor->contents *
    prop_sigma->contents * rho->contents * ct->f107;
  t769 = t769_tmp * ct->f161 * ct->f3 * t463 * 3.1415926535897931 / 4.0 +
    b_t769_tmp * ct->f150 * ct->f161 * ct->f3 * ct->f19 * t591 *
    3.1415926535897931 * -0.25;
  t770_tmp_tmp = t756 * ct->f63 * ct->f102 * ct->f112;
  t756 = t770_tmp_tmp * ct->f142;
  t770_tmp = ct->f1[3] * V->contents * gain_motor->contents *
    prop_sigma->contents * rho->contents * ct->f107;
  t770 = t756 * ct->f171 * ct->f3 * t464 * 3.1415926535897931 / 4.0 + t770_tmp *
    ct->f151 * ct->f171 * ct->f3 * ct->f20 * t592 * 3.1415926535897931 * -0.25;
  t771 = t832 * ct->f144 * ct->f3 * t461 * 3.1415926535897931 / 4.0 + t833 *
    ct->f144 * ct->f148 * ct->f3 * ct->f17 * t589 * 3.1415926535897931 * -0.25;
  t772 = t829 * ct->f145 * ct->f3 * t462 * 3.1415926535897931 / 4.0 + t773 *
    ct->f145 * ct->f149 * ct->f3 * ct->f18 * t590 * 3.1415926535897931 * -0.25;
  t773 = t769_tmp * ct->f146 * ct->f3 * t463 * 3.1415926535897931 / 4.0 +
    b_t769_tmp * ct->f146 * ct->f150 * ct->f3 * ct->f19 * t591 *
    3.1415926535897931 * -0.25;
  t774 = t756 * ct->f147 * ct->f3 * t464 * 3.1415926535897931 / 4.0 + t770_tmp *
    ct->f147 * ct->f151 * ct->f3 * ct->f20 * t592 * 3.1415926535897931 * -0.25;
  t756 = V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * ct->f107;
  t753 = t767_tmp_tmp * ct->f148 * ct->f3 * t461 * 3.1415926535897931 / 4.0 +
    ct->f1[0] * (t756 * ct->f125 * ct->f3 * ct->f17 * t589 * 3.1415926535897931 /
                 4.0);
  t754 = t754 * ct->f149 * ct->f3 * t462 * 3.1415926535897931 / 4.0 + ct->f1[1] *
    (t756 * ct->f131 * ct->f3 * ct->f18 * t590 * 3.1415926535897931 / 4.0);
  t462 = t769_tmp_tmp * ct->f150 * ct->f3 * t463 * 3.1415926535897931 / 4.0 +
    ct->f1[2] * (t756 * ct->f141 * ct->f3 * ct->f19 * t591 * 3.1415926535897931 /
                 4.0);
  t756 = t770_tmp_tmp * ct->f151 * ct->f3 * t464 * 3.1415926535897931 / 4.0 +
    ct->f1[3] * (t756 * ct->f142 * ct->f3 * ct->f20 * t592 * 3.1415926535897931 /
                 4.0);
  t832 = ((t767 + t768) + t769) + t770;
  t833 = ((t771 + t772) + t773) + t774;
  t829 = ((t753 + t754) + t462) + t756;
  t769_tmp_tmp = dv_global->contents[4] + ct->f173 * ((((((((((ct->f121 + ct->f6)
    + ct->f62) + l_z->contents * t753) + l_z->contents * t754) + l_z->contents *
    t462) + l_z->contents * t756) + l_4->contents * t771) + l_4->contents * t772)
    - l_3->contents * t773) - l_3->contents * t774);
  t770_tmp_tmp = dv_global->contents[1] + ct->f178 * (((ct->f66 + ct->f95) +
    ct->f117 * t832) + ct->f120 * t833);
  t463 = dv_global->contents[3] + ct->f172 *
    ((((((((((((((((((((((((((((ct->f122 + ct->f7) + ct->f21) + ct->f23) +
    ct->f33) + ct->f35) + ct->f36) + ct->f37) + ct->f38) + ct->f39) + ct->f40) +
                      ct->f42) + ct->f43) + ct->f44) + ct->f47) + ct->f48) +
                 ct->f50) + ct->f51) + ct->f53) + ct->f59) + ct->f61) +
            l_1->contents * t771) + l_1->contents * t774) + l_z->contents * t767)
         + l_z->contents * t768) + l_z->contents * t769) + l_z->contents * t770)
      - l_1->contents * t772) - l_1->contents * t773);
  t591 = (dv_global->contents[2] + ct->f178 * (((((ct->f78 + ct->f97) + ct->f99)
             + ct->f143 * t829) + ct->f118 * ct->f120 * t832) - ct->f117 *
           ct->f118 * t833)) - 9.81;
  t590 = dv_global->contents[5] - ct->f174 * (((((((((ct->f119 + ct->f5) +
    l_1->contents * t753) + l_1->contents * t756) - l_1->contents * t754) -
    l_1->contents * t462) + l_3->contents * t769) + l_3->contents * t770) -
    l_4->contents * t767) - l_4->contents * t768);
  t756 = dv_global->contents[0] - ct->f178 * (((((ct->f85 + ct->f96) + ct->f98)
    + ct->f118 * t829) + ct->f117 * ct->f143 * t833) - ct->f120 * ct->f143 *
    t832);
  t832 = ct->f1[13] - desired_phi_value->contents * ct->f175;
  t833 = ct->f1[12] - desired_theta_value->contents * ct->f176;
  t829 = ct->f1[14] - desired_ailerons_value->contents * ct->f177;
  t462 = ct->f1[4] + ct->f9;
  t773 = ct->f1[5] + ct->f9;
  t754 = ct->f1[6] + ct->f9;
  t769_tmp = ct->f1[7] + ct->f9;
  b_t769_tmp = ct->f1[8] + ct->f11;
  t770_tmp = ct->f1[9] + ct->f11;
  t753 = ct->f1[10] + ct->f11;
  t461 = ct->f1[11] + ct->f11;
  t589 = gamma_quadratic_du->contents * ct->f73;
  t767_tmp_tmp = gamma_quadratic_du->contents * ct->f87;
  t774 = gamma_quadratic_du->contents * ct->f92;
  return (((((((((((((((((((ct->f86 * (t590 * t590) + ct->f75 * (t770_tmp_tmp *
    t770_tmp_tmp)) + ct->f84 * (t769_tmp_tmp * t769_tmp_tmp)) + ct->f79 * (t463 *
    t463)) + ct->f74 * (t756 * t756)) + ct->f77 * (t591 * t591)) + t589 *
                       (ct->f80 * ct->f80)) + t589 * (ct->f81 * ct->f81)) + t589
                     * (ct->f82 * ct->f82)) + t589 * (ct->f83 * ct->f83)) +
                   gamma_quadratic_du->contents * ct->f67 * (t832 * t832)) +
                  gamma_quadratic_du->contents * ct->f68 * (t833 * t833)) +
                 gamma_quadratic_du->contents * ct->f93 * (t829 * t829)) +
                t767_tmp_tmp * (t462 * t462)) + t767_tmp_tmp * (t773 * t773)) +
              t767_tmp_tmp * (t754 * t754)) + t767_tmp_tmp * (t769_tmp *
              t769_tmp)) + t774 * (b_t769_tmp * b_t769_tmp)) + t774 * (t770_tmp *
            t770_tmp)) + t774 * (t753 * t753)) + t774 * (t461 * t461);
}

static double b_maxConstraintViolation(const h_struct_T *obj, const double x[16])
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

static double b_norm(const double x[15])
{
  double scale;
  double y;
  int k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 15; k++) {
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

static void b_test_exit(j_struct_T *Flags, d_struct_T *memspace, b_struct_T
  *MeritFunction, const h_struct_T *WorkingSet, g_struct_T *TrialState,
  e_struct_T *QRManager, const double lb[15], const double ub[15])
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

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-9 *
                       optimRelativeFactor) && (MeritFunction->nlpComplError <=
         1.0E-9 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        bool guard1 = false;
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          idx_max = 0;
          exitg1 = false;
          while ((!exitg1) && (idx_max <= i1 - 1)) {
            if (1.0E-9 * fmax(1.0, fabs(TrialState->xstarsqp[idx_max])) <= fabs
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
                  idx_max = k << 4;
                  rankR = 31 * k;
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
                  nVar += 32;
                }

                b_xgemv(WorkingSet->nVar, WorkingSet->nVar, QRManager->Q,
                        TrialState->grad, memspace->workspace_double);
                if (rankR != 0) {
                  for (k = rankR; k >= 1; k--) {
                    nVar = (k + (k - 1) * 31) - 1;
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
                if ((smax <= 1.0E-9 * optimRelativeFactor) && (s <= 1.0E-9 *
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
          if (TrialState->sqpIterations >= 150) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 1000) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

static double b_timeKeeper(double *outTime_tv_nsec)
{
  double outTime_tv_sec;
  outTime_tv_sec = savedTime.tv_sec;
  *outTime_tv_nsec = savedTime.tv_nsec;
  return outTime_tv_sec;
}

static void b_xgemm(int m, int n, int k, const double A[961], int ia0, const
                    double B[496], double C[961])
{
  int cr;
  int ic;
  int w;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    lastColC = 31 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 31) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        memset(&C[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof(double));
      }
    }

    br = -1;
    for (cr = 0; cr <= lastColC; cr += 31) {
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
        ar += 31;
      }

      br += 31;
    }
  }
}

static void b_xgemv(int m, int n, const double A[961], const double x[16],
                    double y[496])
{
  int ia;
  int iac;
  if (m != 0) {
    int i;
    memset(&y[0], 0, (unsigned int)n * sizeof(double));
    i = 31 * (n - 1) + 1;
    for (iac = 1; iac <= i; iac += 31) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }

      i1 = div_nde_s32_floor(iac - 1);
      y[i1] += c;
    }
  }
}

static double b_xnrm2(int n, const double x[16])
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

static double c_computeObjectiveAndUserGradie(const c_struct_T
  *c_obj_next_next_next_next_next_, const double x[15], double grad_workspace[16],
  int *status)
{
  double varargout_2_data[15];
  double fval;
  int idx_current;
  bool allFinite;
  fval = compute_cost_and_gradient_fcn
    (c_obj_next_next_next_next_next_->dv_global,
     c_obj_next_next_next_next_next_->V,
     c_obj_next_next_next_next_next_->gain_motor,
     c_obj_next_next_next_next_next_->prop_sigma,
     c_obj_next_next_next_next_next_->rho,
     c_obj_next_next_next_next_next_->gain_el,
     c_obj_next_next_next_next_next_->gain_az,
     c_obj_next_next_next_next_next_->l_1, c_obj_next_next_next_next_next_->l_4,
     c_obj_next_next_next_next_next_->l_3, c_obj_next_next_next_next_next_->l_z,
     c_obj_next_next_next_next_next_->gamma_quadratic_du,
     c_obj_next_next_next_next_next_->desired_phi_value,
     c_obj_next_next_next_next_next_->desired_theta_value,
     c_obj_next_next_next_next_next_->desired_ailerons_value,
     c_obj_next_next_next_next_next_->K_p_M,
     c_obj_next_next_next_next_next_->k_tilt,
     c_obj_next_next_next_next_next_->gain_theta,
     c_obj_next_next_next_next_next_->K_Cd, c_obj_next_next_next_next_next_->S,
     c_obj_next_next_next_next_next_->Cm_alpha,
     c_obj_next_next_next_next_next_->wing_chord,
     c_obj_next_next_next_next_next_->gain_phi,
     c_obj_next_next_next_next_next_->CL_aileron,
     c_obj_next_next_next_next_next_->gain_ailerons,
     c_obj_next_next_next_next_next_->flight_path_angle,
     c_obj_next_next_next_next_next_->Beta,
     c_obj_next_next_next_next_next_->prop_delta,
     c_obj_next_next_next_next_next_->Cl_alpha,
     c_obj_next_next_next_next_next_->W_act_phi,
     c_obj_next_next_next_next_next_->W_act_theta,
     c_obj_next_next_next_next_next_->W_act_motor,
     c_obj_next_next_next_next_next_->W_dv_1,
     c_obj_next_next_next_next_next_->W_dv_2,
     c_obj_next_next_next_next_next_->W_dv_3,
     c_obj_next_next_next_next_next_->W_dv_4,
     c_obj_next_next_next_next_next_->W_dv_5,
     c_obj_next_next_next_next_next_->W_dv_6,
     c_obj_next_next_next_next_next_->W_act_tilt_el,
     c_obj_next_next_next_next_next_->W_act_tilt_az,
     c_obj_next_next_next_next_next_->W_act_ailerons,
     c_obj_next_next_next_next_next_->prop_Cd_a,
     c_obj_next_next_next_next_next_->prop_R,
     c_obj_next_next_next_next_next_->prop_Cd_0,
     c_obj_next_next_next_next_next_->I_xx, c_obj_next_next_next_next_next_->p,
     c_obj_next_next_next_next_next_->q, c_obj_next_next_next_next_next_->I_yy,
     c_obj_next_next_next_next_next_->r, c_obj_next_next_next_next_next_->I_zz,
     c_obj_next_next_next_next_next_->gain_airspeed,
     c_obj_next_next_next_next_next_->m,
     c_obj_next_next_next_next_next_->prop_Cl_a,
     c_obj_next_next_next_next_next_->prop_theta,
     c_obj_next_next_next_next_next_->prop_Cl_0,
     c_obj_next_next_next_next_next_->desired_el_value,
     c_obj_next_next_next_next_next_->desired_az_value,
     c_obj_next_next_next_next_next_->desired_motor_value,
     c_obj_next_next_next_next_next_->Cm_zero,
     c_obj_next_next_next_next_next_->Cd_zero, x, varargout_2_data, &idx_current);
  memcpy(&grad_workspace[0], &varargout_2_data[0], 15U * sizeof(double));
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
    while (allFinite && (idx_current + 1 <= 15)) {
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

static void c_compute_acc_nonlinear_control(const double u_in[15], double p,
  double q, double r, double m, double I_xx, double I_yy, double I_zz, double
  l_1, double l_3, double l_4, double l_z, double Cl_alpha, double Cd_zero,
  double K_Cd, double Cm_alpha, double Cm_zero, double CL_aileron, double rho,
  double V, double S, double wing_chord, double flight_path_angle, double
  gain_motor, double gain_airspeed, double gain_el, double gain_theta, double
  gain_az, double gain_phi, double computed_acc[6])
{
  double Alpha;
  double Omega_1_scaled;
  double Omega_2_scaled;
  double Omega_3_scaled;
  double Omega_4_scaled;
  double Theta_scaled;
  double a_tmp;
  double a_tmp_tmp;
  double b_1_scaled;
  double b_2_scaled;
  double b_3_scaled;
  double b_4_scaled;
  double b_a_tmp;
  double b_a_tmp_tmp;
  double b_gamma1_tmp;
  double b_gamma1_tmp_tmp;
  double b_gamma2_tmp;
  double b_gamma3_tmp;
  double b_gamma4_tmp;
  double b_gamma7_tmp;
  double b_gamma7_tmp_tmp;
  double c_a_tmp;
  double c_a_tmp_tmp;
  double c_gamma1_tmp;
  double c_gamma2_tmp;
  double c_gamma4_tmp;
  double c_gamma7_tmp;
  double c_gamma7_tmp_tmp;
  double d_a_tmp;
  double d_a_tmp_tmp;
  double d_gamma1_tmp;
  double d_gamma7_tmp;
  double d_gamma7_tmp_tmp;
  double e_a_tmp;
  double e_gamma1_tmp;
  double e_gamma7_tmp;
  double f_a_tmp;
  double f_gamma1_tmp;
  double f_gamma7_tmp;
  double g_a_tmp;
  double g_gamma1_tmp;
  double g_gamma7_tmp;
  double gamma1;
  double gamma1_tmp;
  double gamma1_tmp_tmp;
  double gamma2;
  double gamma2_tmp;
  double gamma2_tmp_tmp;
  double gamma3;
  double gamma3_tmp;
  double gamma3_tmp_tmp;
  double gamma4;
  double gamma4_tmp;
  double gamma4_tmp_tmp;
  double gamma7_tmp;
  double gamma7_tmp_tmp;
  double gamma7_tmp_tmp_tmp;
  double gamma8;
  double h_gamma7_tmp;
  double i_gamma7_tmp;
  double j_gamma7_tmp;
  double k_gamma7_tmp;
  double l_gamma7_tmp;
  double m_gamma7_tmp;
  double n_gamma7_tmp;
  double o_gamma7_tmp;
  double p_gamma7_tmp;
  Omega_1_scaled = u_in[0] / gain_motor;
  Omega_2_scaled = u_in[1] / gain_motor;
  Omega_3_scaled = u_in[2] / gain_motor;
  Omega_4_scaled = u_in[3] / gain_motor;
  b_1_scaled = u_in[4] / gain_el;
  b_2_scaled = u_in[5] / gain_el;
  b_3_scaled = u_in[6] / gain_el;
  b_4_scaled = u_in[7] / gain_el;
  Theta_scaled = u_in[12] / gain_theta;
  Alpha = u_in[12] - flight_path_angle;

  /*  New prop model */
  /*  New aero models */
  a_tmp_tmp = b_4_scaled * gain_el;
  a_tmp = Alpha + a_tmp_tmp;
  b_a_tmp = sin(a_tmp);
  a_tmp = cos(a_tmp);
  gamma1_tmp = V * V;
  gamma1_tmp_tmp = gain_motor * gain_motor;
  b_gamma1_tmp_tmp = Omega_4_scaled * Omega_4_scaled;
  b_gamma1_tmp = b_gamma1_tmp_tmp * gamma1_tmp_tmp * 0.016129;
  gamma4 = 8.0 * gamma1_tmp;
  c_gamma1_tmp = a_tmp * a_tmp;
  d_gamma1_tmp = 16.0 * gamma1_tmp;
  e_gamma1_tmp = gamma4 * 3.46 * 0.2188;
  gamma4 = gamma4 * 0.12 * 0.0551;
  f_gamma1_tmp = 8.0 * V * 3.46 * 0.0551;
  g_gamma1_tmp = Omega_4_scaled * gain_motor * 0.127;
  gamma1 = 0.125 * sqrt(((d_gamma1_tmp * (b_a_tmp * b_a_tmp) / b_gamma1_tmp -
    -0.044080000000000008 * (e_gamma1_tmp * c_gamma1_tmp / b_gamma1_tmp +
    2.7584952256) / 0.2) - gamma4 * c_gamma1_tmp * -1.6094379124341003 /
    b_gamma1_tmp) - f_gamma1_tmp * b_a_tmp * -0.8 / g_gamma1_tmp);
  b_a_tmp_tmp = b_3_scaled * gain_el;
  c_a_tmp = Alpha + b_a_tmp_tmp;
  d_a_tmp = sin(c_a_tmp);
  c_a_tmp = cos(c_a_tmp);
  gamma2_tmp_tmp = Omega_3_scaled * Omega_3_scaled;
  gamma2_tmp = gamma2_tmp_tmp * gamma1_tmp_tmp * 0.016129;
  b_gamma2_tmp = c_a_tmp * c_a_tmp;
  c_gamma2_tmp = Omega_3_scaled * gain_motor * 0.127;
  gamma2 = 0.125 * sqrt(((d_gamma1_tmp * (d_a_tmp * d_a_tmp) / gamma2_tmp -
    -0.044080000000000008 * (e_gamma1_tmp * b_gamma2_tmp / gamma2_tmp +
    2.7584952256) / 0.2) - gamma4 * b_gamma2_tmp * -1.6094379124341003 /
    gamma2_tmp) - f_gamma1_tmp * d_a_tmp * -0.8 / c_gamma2_tmp);
  c_a_tmp_tmp = b_2_scaled * gain_el;
  e_a_tmp = Alpha + c_a_tmp_tmp;
  f_a_tmp = sin(e_a_tmp);
  e_a_tmp = cos(e_a_tmp);
  gamma3_tmp_tmp = Omega_2_scaled * Omega_2_scaled;
  gamma8 = gamma3_tmp_tmp * gamma1_tmp_tmp * 0.016129;
  gamma3_tmp = e_a_tmp * e_a_tmp;
  b_gamma3_tmp = Omega_2_scaled * gain_motor * 0.127;
  gamma3 = 0.125 * sqrt(((d_gamma1_tmp * (f_a_tmp * f_a_tmp) / gamma8 -
    -0.044080000000000008 * (e_gamma1_tmp * gamma3_tmp / gamma8 + 2.7584952256) /
    0.2) - gamma4 * gamma3_tmp * -1.6094379124341003 / gamma8) - f_gamma1_tmp *
                        f_a_tmp * -0.8 / b_gamma3_tmp);
  d_a_tmp_tmp = b_1_scaled * gain_el;
  g_a_tmp = Alpha + d_a_tmp_tmp;
  Alpha = sin(g_a_tmp);
  g_a_tmp = cos(g_a_tmp);
  gamma4_tmp_tmp = Omega_1_scaled * Omega_1_scaled;
  gamma4_tmp = gamma4_tmp_tmp * gamma1_tmp_tmp * 0.016129;
  b_gamma4_tmp = g_a_tmp * g_a_tmp;
  c_gamma4_tmp = Omega_1_scaled * gain_motor * 0.127;
  gamma4 = 0.125 * sqrt(((d_gamma1_tmp * (Alpha * Alpha) / gamma4_tmp -
    -0.044080000000000008 * (e_gamma1_tmp * b_gamma4_tmp / gamma4_tmp +
    2.7584952256) / 0.2) - gamma4 * b_gamma4_tmp * -1.6094379124341003 /
    gamma4_tmp) - f_gamma1_tmp * Alpha * -0.8 / c_gamma4_tmp);
  e_gamma1_tmp = gamma1_tmp * 3.46 * 0.2188;
  f_gamma1_tmp = gamma1_tmp * 0.12 * 0.2;
  d_gamma1_tmp = 0.5 * V * b_a_tmp / g_gamma1_tmp;
  gamma7_tmp_tmp = u_in[8] / gain_az * gain_az;
  gamma7_tmp = cos(gamma7_tmp_tmp);
  b_gamma7_tmp_tmp = u_in[9] / gain_az * gain_az;
  b_gamma7_tmp = cos(b_gamma7_tmp_tmp);
  c_gamma7_tmp_tmp = u_in[10] / gain_az * gain_az;
  c_gamma7_tmp = cos(c_gamma7_tmp_tmp);
  d_gamma7_tmp_tmp = u_in[11] / gain_az * gain_az;
  d_gamma7_tmp = cos(d_gamma7_tmp_tmp);
  e_gamma7_tmp = cos(d_a_tmp_tmp);
  f_gamma7_tmp = cos(c_a_tmp_tmp);
  g_gamma7_tmp = cos(b_a_tmp_tmp);
  h_gamma7_tmp = cos(a_tmp_tmp);
  i_gamma7_tmp = sin(d_a_tmp_tmp);
  j_gamma7_tmp = sin(c_a_tmp_tmp);
  k_gamma7_tmp = sin(b_a_tmp_tmp);
  l_gamma7_tmp = sin(a_tmp_tmp);
  m_gamma7_tmp = 0.5 * V * Alpha / c_gamma4_tmp;
  n_gamma7_tmp = (gamma4 - 0.0190646) - m_gamma7_tmp;
  f_a_tmp = 0.5 * V * f_a_tmp / b_gamma3_tmp;
  o_gamma7_tmp = (gamma3 - 0.0190646) - f_a_tmp;
  p_gamma7_tmp = 0.5 * V * d_a_tmp / c_gamma2_tmp;
  b_gamma3_tmp = (gamma2 - 0.0190646) - p_gamma7_tmp;
  d_a_tmp = (gamma1 - 0.0190646) - d_gamma1_tmp;
  c_gamma4_tmp = 0.7854 * gamma4_tmp_tmp * gamma1_tmp_tmp * 0.000260144641 *
    0.0551 * rho;
  c_a_tmp_tmp = c_gamma4_tmp * e_gamma7_tmp;
  m_gamma7_tmp = -0.8 * ((0.0288 - 1.3840000000000001 * (((gamma4 - 0.0190646) -
    0.2188) - m_gamma7_tmp)) + e_gamma1_tmp * b_gamma4_tmp / gamma4_tmp) +
    f_gamma1_tmp * b_gamma4_tmp * -1.6094379124341003 / gamma4_tmp;
  b_a_tmp_tmp = 0.7854 * gamma3_tmp_tmp * gamma1_tmp_tmp * 0.000260144641 *
    0.0551 * rho;
  d_a_tmp_tmp = b_a_tmp_tmp * f_gamma7_tmp;
  f_a_tmp = -0.8 * ((0.0288 - 1.3840000000000001 * (((gamma3 - 0.0190646) -
    0.2188) - f_a_tmp)) + e_gamma1_tmp * gamma3_tmp / gamma8) + f_gamma1_tmp *
    gamma3_tmp * -1.6094379124341003 / gamma8;
  gamma7_tmp_tmp_tmp = 0.7854 * gamma2_tmp_tmp * gamma1_tmp_tmp;
  a_tmp_tmp = gamma7_tmp_tmp_tmp * 0.000260144641 * 0.0551 * rho;
  g_gamma1_tmp = a_tmp_tmp * g_gamma7_tmp;
  p_gamma7_tmp = -0.8 * ((0.0288 - 1.3840000000000001 * (((gamma2 - 0.0190646) -
    0.2188) - p_gamma7_tmp)) + e_gamma1_tmp * b_gamma2_tmp / gamma2_tmp) +
    f_gamma1_tmp * b_gamma2_tmp * -1.6094379124341003 / gamma2_tmp;
  b_a_tmp = 0.7854 * b_gamma1_tmp_tmp * gamma1_tmp_tmp * (-0.8 * ((0.0288 -
    1.3840000000000001 * (((gamma1 - 0.0190646) - 0.2188) - d_gamma1_tmp)) +
    e_gamma1_tmp * c_gamma1_tmp / b_gamma1_tmp) + f_gamma1_tmp * c_gamma1_tmp *
    -1.6094379124341003 / b_gamma1_tmp) * 0.000260144641 * 0.0551 * rho;
  Alpha = b_a_tmp * h_gamma7_tmp;
  gamma4 = 0.7854 * Omega_1_scaled * V * gain_motor * 0.002048383 * 0.0551 * rho
    * g_a_tmp;
  n_gamma7_tmp = (0.020000000000000004 - 0.2188 * (-2.74 * n_gamma7_tmp -
    0.15753599999999998)) * -0.8 + -0.038626509898418405 * n_gamma7_tmp;
  d_gamma1_tmp = 0.7854 * Omega_2_scaled * V * gain_motor * 0.002048383 * 0.0551
    * rho * e_a_tmp;
  o_gamma7_tmp = (0.020000000000000004 - 0.2188 * (-2.74 * o_gamma7_tmp -
    0.15753599999999998)) * -0.8 + -0.038626509898418405 * o_gamma7_tmp;
  e_gamma1_tmp = 0.7854 * Omega_3_scaled * V * gain_motor * 0.002048383 * 0.0551
    * rho * c_a_tmp;
  b_gamma3_tmp = (0.020000000000000004 - 0.2188 * (-2.74 * b_gamma3_tmp -
    0.15753599999999998)) * -0.8 + -0.038626509898418405 * b_gamma3_tmp;
  f_gamma1_tmp = 0.7854 * Omega_4_scaled * V * gain_motor * 0.002048383 * 0.0551
    * rho * a_tmp;
  d_a_tmp = (0.020000000000000004 - 0.2188 * (-2.74 * d_a_tmp -
              0.15753599999999998)) * -0.8 + -0.038626509898418405 * d_a_tmp;
  b_gamma2_tmp = c_a_tmp_tmp * gamma7_tmp * m_gamma7_tmp / 0.2;
  gamma7_tmp = gamma4 * gamma7_tmp * i_gamma7_tmp * n_gamma7_tmp / 0.2;
  gamma2 = d_a_tmp_tmp * b_gamma7_tmp * f_a_tmp / 0.2;
  b_gamma7_tmp = d_gamma1_tmp * b_gamma7_tmp * j_gamma7_tmp * o_gamma7_tmp / 0.2;
  g_a_tmp = e_gamma1_tmp * c_gamma7_tmp * k_gamma7_tmp * b_gamma3_tmp / 0.2;
  e_a_tmp = Alpha * d_gamma7_tmp / 0.2;
  d_gamma7_tmp = f_gamma1_tmp * d_gamma7_tmp * l_gamma7_tmp * d_a_tmp / 0.2;
  Omega_3_scaled = ((((((b_gamma2_tmp + gamma2) + g_gamma1_tmp * c_gamma7_tmp *
                        p_gamma7_tmp / 0.2) + e_a_tmp) - gamma7_tmp) -
                     b_gamma7_tmp) - g_a_tmp) - d_gamma7_tmp;
  gamma2_tmp = sin(gamma7_tmp_tmp);
  gamma1 = sin(b_gamma7_tmp_tmp);
  c_gamma1_tmp = sin(c_gamma7_tmp_tmp);
  b_gamma1_tmp = sin(d_gamma7_tmp_tmp);
  b_gamma1_tmp_tmp = c_a_tmp_tmp * gamma2_tmp * m_gamma7_tmp / 0.2;
  gamma2_tmp = gamma4 * i_gamma7_tmp * gamma2_tmp * n_gamma7_tmp / 0.2;
  gamma3 = d_a_tmp_tmp * gamma1 * f_a_tmp / 0.2;
  gamma1 = d_gamma1_tmp * j_gamma7_tmp * gamma1 * o_gamma7_tmp / 0.2;
  gamma1_tmp_tmp = e_gamma1_tmp * k_gamma7_tmp * c_gamma1_tmp * b_gamma3_tmp /
    0.2;
  gamma3_tmp = Alpha * b_gamma1_tmp / 0.2;
  b_gamma1_tmp = f_gamma1_tmp * l_gamma7_tmp * b_gamma1_tmp * d_a_tmp / 0.2;
  gamma8 = ((((((b_gamma1_tmp_tmp + gamma3) + g_gamma1_tmp * c_gamma1_tmp *
                p_gamma7_tmp / 0.2) + gamma3_tmp) - gamma2_tmp) - gamma1) -
            gamma1_tmp_tmp) - b_gamma1_tmp;
  gamma2_tmp_tmp = c_gamma4_tmp * i_gamma7_tmp * m_gamma7_tmp / 0.2;
  gamma3_tmp_tmp = gamma4 * e_gamma7_tmp * n_gamma7_tmp / 0.2;
  b_gamma4_tmp = b_a_tmp_tmp * j_gamma7_tmp * f_a_tmp / 0.2;
  d_gamma1_tmp = d_gamma1_tmp * f_gamma7_tmp * o_gamma7_tmp / 0.2;
  gamma4_tmp = e_gamma1_tmp * g_gamma7_tmp * b_gamma3_tmp / 0.2;
  gamma4_tmp_tmp = b_a_tmp * l_gamma7_tmp / 0.2;
  b_a_tmp = f_gamma1_tmp * h_gamma7_tmp * d_a_tmp / 0.2;
  Alpha = ((((((gamma2_tmp_tmp + b_gamma4_tmp) + a_tmp_tmp * k_gamma7_tmp *
               p_gamma7_tmp / 0.2) + gamma4_tmp_tmp) + gamma3_tmp_tmp) +
            d_gamma1_tmp) + gamma4_tmp) + b_a_tmp;
  f_gamma1_tmp = Theta_scaled * gain_theta;
  c_gamma4_tmp = cos(f_gamma1_tmp);
  gamma4 = flight_path_angle - f_gamma1_tmp;
  f_a_tmp = sin(gamma4);
  b_gamma3_tmp = cos(gamma4);
  e_gamma1_tmp = u_in[13] / gain_phi * gain_phi;
  d_a_tmp = cos(e_gamma1_tmp);
  c_gamma2_tmp = sin(f_gamma1_tmp);
  g_gamma1_tmp = sin(e_gamma1_tmp);
  d_a_tmp_tmp = Cl_alpha * Cl_alpha;
  c_a_tmp_tmp = K_Cd * d_a_tmp_tmp;
  b_a_tmp_tmp = 0.5 * S * gamma1_tmp * rho;
  d_a_tmp_tmp = ((c_a_tmp_tmp * (Theta_scaled * Theta_scaled) * (gain_theta *
    gain_theta) - 2.0 * K_Cd * d_a_tmp_tmp * Theta_scaled * flight_path_angle *
                  gain_theta) + c_a_tmp_tmp * (flight_path_angle *
    flight_path_angle)) + Cd_zero;
  c_a_tmp_tmp = 0.5 * Cl_alpha * S * gamma1_tmp * rho;
  a_tmp_tmp = b_a_tmp_tmp * f_a_tmp * d_a_tmp_tmp + c_a_tmp_tmp * b_gamma3_tmp *
    gamma4;
  f_a_tmp = b_a_tmp_tmp * b_gamma3_tmp * d_a_tmp_tmp - c_a_tmp_tmp * f_a_tmp *
    gamma4;
  b_gamma3_tmp = Omega_3_scaled * d_a_tmp;
  computed_acc[0] = ((((Alpha * c_gamma4_tmp - c_gamma4_tmp * f_a_tmp) + d_a_tmp
                       * c_gamma2_tmp * a_tmp_tmp) + b_gamma3_tmp * c_gamma2_tmp)
                     - gamma8 * g_gamma1_tmp * c_gamma2_tmp) / m;
  computed_acc[1] = -((g_gamma1_tmp * a_tmp_tmp + gamma8 * d_a_tmp) +
                      Omega_3_scaled * g_gamma1_tmp) / m;
  computed_acc[2] = ((((c_gamma2_tmp * f_a_tmp - Alpha * c_gamma2_tmp) + d_a_tmp
                       * c_gamma4_tmp * a_tmp_tmp) + b_gamma3_tmp * c_gamma4_tmp)
                     - gamma8 * c_gamma4_tmp * g_gamma1_tmp) / m + 9.81;
  f_gamma1_tmp = gamma7_tmp_tmp_tmp * p_gamma7_tmp * 0.000260144641 * 0.0551 *
    rho;
  c_gamma4_tmp = f_gamma1_tmp * g_gamma7_tmp;
  f_a_tmp = b_gamma2_tmp - gamma7_tmp;
  b_gamma3_tmp = gamma2 - b_gamma7_tmp;
  d_a_tmp = c_gamma4_tmp * c_gamma7_tmp / 0.2 - g_a_tmp;
  c_gamma2_tmp = e_a_tmp - d_gamma7_tmp;
  g_gamma1_tmp = b_gamma1_tmp_tmp - gamma2_tmp;
  d_a_tmp_tmp = gamma3 - gamma1;
  c_gamma4_tmp = c_gamma4_tmp * c_gamma1_tmp / 0.2 - gamma1_tmp_tmp;
  c_a_tmp_tmp = gamma3_tmp - b_gamma1_tmp;
  computed_acc[3] = -((((((((((((((((((((((((((((0.5118 * Omega_1_scaled -
    0.5118 * Omega_2_scaled) - 0.2842 * b_1_scaled) + 0.2842 * b_2_scaled) +
    0.2441 * b_3_scaled) - 0.2441 * b_4_scaled) + 0.3656 * Omega_1_scaled *
    b_1_scaled) - 0.3656 * Omega_2_scaled * b_2_scaled) - 0.1899 *
    Omega_1_scaled * b_4_scaled) + 0.1899 * Omega_2_scaled * b_3_scaled) + l_1 *
    f_a_tmp) - l_1 * b_gamma3_tmp) + l_z * g_gamma1_tmp) + l_z * d_a_tmp_tmp) -
    l_1 * d_a_tmp) + l_1 * c_gamma2_tmp) + l_z * c_gamma4_tmp) + l_z *
    c_a_tmp_tmp) - 0.1567 * (b_1_scaled * b_1_scaled)) + 0.1567 * (b_2_scaled *
    b_2_scaled)) + 0.178 * (b_3_scaled * b_3_scaled)) - 0.178 * (b_4_scaled *
    b_4_scaled)) - I_yy * q * r) + I_zz * q * r) + 1.853 * Omega_1_scaled * V /
    gain_airspeed) - 1.853 * Omega_2_scaled * V / gain_airspeed) + 0.5481 * V *
                        b_1_scaled / gain_airspeed) - 0.5481 * V * b_2_scaled /
                       gain_airspeed) - 0.5 * CL_aileron * S * gamma1_tmp *
                      u_in[14] * rho) / I_xx;
  a_tmp_tmp = gamma2_tmp_tmp + gamma3_tmp_tmp;
  d_gamma1_tmp += b_gamma4_tmp;
  gamma4 = f_gamma1_tmp * k_gamma7_tmp / 0.2 + gamma4_tmp;
  Alpha = gamma4_tmp_tmp + b_a_tmp;
  e_gamma1_tmp = I_xx * p;
  computed_acc[4] = -((((((((((l_z * a_tmp_tmp + l_z * d_gamma1_tmp) + l_4 *
    f_a_tmp) + l_4 * b_gamma3_tmp) + l_z * gamma4) + l_z * Alpha) - l_3 *
    d_a_tmp) - l_3 * c_gamma2_tmp) + e_gamma1_tmp * r) - I_zz * p * r) -
                      b_a_tmp_tmp * wing_chord * ((Cm_zero - Cm_alpha *
    flight_path_angle) + Cm_alpha * Theta_scaled * gain_theta)) / I_yy;
  computed_acc[5] = (((((((((l_1 * a_tmp_tmp - l_1 * d_gamma1_tmp) - l_4 *
    g_gamma1_tmp) - l_4 * d_a_tmp_tmp) - l_1 * gamma4) + l_1 * Alpha) + l_3 *
                        c_gamma4_tmp) + l_3 * c_a_tmp_tmp) + e_gamma1_tmp * q) -
                     I_yy * p * q) / I_zz;
}

static double c_norm(const double x[6])
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

static double computeComplError(const double xCurrent[15], const int finiteLB[16],
  int mLB, const double lb[15], const int finiteUB[16], int mUB, const double
  ub[15], const double lambda[31], int iL0)
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

static double computeFval(const struct_T *obj, double workspace[496], const
  double H[225], const double f[16], const double x[16])
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
      for (idx = i; idx < 16; idx++) {
        workspace[idx - 1] = 0.5 * obj->beta * x[idx - 1] + obj->rho;
      }

      val = 0.0;
      for (idx = 0; idx < 15; idx++) {
        val += x[idx] * workspace[idx];
      }
    }
    break;
  }

  return val;
}

static double computeFval_ReuseHx(const struct_T *obj, double workspace[496],
  const double f[16], const double x[16])
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

        i = 14 - obj->nvar;
        for (k = 0; k <= i; k++) {
          workspace[obj->nvar + k] = obj->rho;
        }

        val = 0.0;
        for (k = 0; k < 15; k++) {
          double d;
          d = workspace[k] + 0.5 * obj->Hx[k];
          workspace[k] = d;
          val += x[k] * d;
        }
      } else {
        int i;
        val = 0.0;
        for (k = 0; k < 15; k++) {
          val += x[k] * obj->Hx[k];
        }

        val *= 0.5;
        i = obj->nvar + 1;
        for (k = i; k < 16; k++) {
          val += x[k - 1] * obj->rho;
        }
      }
    }
    break;
  }

  return val;
}

static void computeGradLag(double workspace[16], int nVar, const double grad[16],
  const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB, const
  int finiteUB[16], int mUB, const double lambda[31])
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

  iL0 = mFixed + (unsigned char)mLB;
  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

static void computeGrad_StoreHx(struct_T *obj, const double H[225], const double
  f[16], const double x[16])
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
      for (ixlast = i; ixlast < 16; ixlast++) {
        obj->Hx[ixlast - 1] = obj->beta * x[ixlast - 1];
      }

      memcpy(&obj->grad[0], &obj->Hx[0], 15U * sizeof(double));
      if (obj->hasLinear && (obj->nvar >= 1)) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          obj->grad[k] += f[k];
        }
      }

      if (15 - obj->nvar >= 1) {
        ixlast = obj->nvar;
        i = 14 - obj->nvar;
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

static void computeQ_(e_struct_T *obj, int nrows)
{
  double work[31];
  int b_i;
  int iQR0;
  int ia;
  int idx;
  int lastc;
  int m;
  int n;
  lastc = obj->minRowCol;
  for (idx = 0; idx < lastc; idx++) {
    iQR0 = 31 * idx + idx;
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
      ia = idx * 31;
      i1 = m - 1;
      memset(&obj->Q[ia], 0, (unsigned int)(((i1 + ia) - ia) + 1) * sizeof
             (double));
      obj->Q[ia + idx] = 1.0;
    }

    itau = obj->minRowCol - 1;
    memset(&work[0], 0, 31U * sizeof(double));
    for (b_i = obj->minRowCol; b_i >= 1; b_i--) {
      int iaii;
      iaii = b_i + (b_i - 1) * 31;
      if (b_i < nrows) {
        int lastv;
        obj->Q[iaii - 1] = 1.0;
        idx = iaii + 31;
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
            iQR0 = (iaii + lastc * 31) + 31;
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

            i = (iaii + 31 * lastc) + 31;
            for (n = idx; n <= i; n += 31) {
              c = 0.0;
              i1 = n + lastv;
              for (ia = n; ia <= i1; ia++) {
                c += obj->Q[ia - 1] * obj->Q[((iaii + ia) - n) - 1];
              }

              iQR0 = div_nde_s32_floor((n - iaii) - 31);
              work[iQR0] += c;
            }
          }

          if (!(-obj->tau[itau] == 0.0)) {
            iQR0 = iaii;
            for (idx = 0; idx <= lastc; idx++) {
              c = work[idx];
              if (c != 0.0) {
                c *= -obj->tau[itau];
                i = iQR0 + 31;
                i1 = lastv + iQR0;
                for (n = i; n <= i1 + 31; n++) {
                  obj->Q[n - 1] += obj->Q[((iaii + n) - iQR0) - 32] * c;
                }
              }

              iQR0 += 31;
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

static double compute_cost_and_gradient_fcn(const b_captured_var *dv_global,
  const captured_var *V, const captured_var *gain_motor, const captured_var
  *prop_sigma, const captured_var *rho, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *l_1, const captured_var *l_4, const
  captured_var *l_3, const captured_var *l_z, const captured_var
  *gamma_quadratic_du, const captured_var *desired_phi_value, const captured_var
  *desired_theta_value, const captured_var *desired_ailerons_value, const
  captured_var *K_p_M, const captured_var *k_tilt, const captured_var
  *gain_theta, const captured_var *K_Cd, const captured_var *S, const
  captured_var *Cm_alpha, const captured_var *wing_chord, const captured_var
  *gain_phi, const captured_var *CL_aileron, const captured_var *gain_ailerons,
  const captured_var *flight_path_angle, const captured_var *Beta, const
  captured_var *prop_delta, const captured_var *Cl_alpha, const captured_var
  *W_act_phi, const captured_var *W_act_theta, const captured_var *W_act_motor,
  const captured_var *W_dv_1, const captured_var *W_dv_2, const captured_var
  *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5, const
  captured_var *W_dv_6, const captured_var *W_act_tilt_el, const captured_var
  *W_act_tilt_az, const captured_var *W_act_ailerons, const captured_var
  *prop_Cd_a, const captured_var *prop_R, const captured_var *prop_Cd_0, const
  captured_var *I_xx, const captured_var *p, const captured_var *q, const
  captured_var *I_yy, const captured_var *r, const captured_var *I_zz, const
  captured_var *gain_airspeed, const captured_var *m, const captured_var
  *prop_Cl_a, const captured_var *prop_theta, const captured_var *prop_Cl_0,
  const captured_var *desired_el_value, const captured_var *desired_az_value,
  const captured_var *desired_motor_value, const captured_var *Cm_zero, const
  captured_var *Cd_zero, const double u_in[15], double gradient_data[], int
  *gradient_size)
{
  cell_18 expl_temp;
  double b_gradient_data[15];
  double Alpha;
  double a;
  double b_a;
  double b_t537_tmp_tmp;
  double c_a;
  double cost;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double t10;
  double t100;
  double t101;
  double t102;
  double t103;
  double t104;
  double t105;
  double t108;
  double t11;
  double t110;
  double t111;
  double t112;
  double t117;
  double t119;
  double t12;
  double t120;
  double t13;
  double t133;
  double t134;
  double t135;
  double t136;
  double t137;
  double t138;
  double t14;
  double t140;
  double t141;
  double t142;
  double t143;
  double t144;
  double t145;
  double t146;
  double t147;
  double t15;
  double t152;
  double t153;
  double t154;
  double t155;
  double t16;
  double t165;
  double t166;
  double t167;
  double t168;
  double t169;
  double t17;
  double t170;
  double t171;
  double t18;
  double t189;
  double t19;
  double t193;
  double t197;
  double t198;
  double t199;
  double t2;
  double t20;
  double t200;
  double t202;
  double t207;
  double t207_tmp;
  double t21;
  double t22;
  double t23;
  double t24;
  double t259_tmp;
  double t283_tmp;
  double t283_tmp_tmp;
  double t287;
  double t287_tmp_tmp;
  double t3;
  double t302;
  double t367;
  double t368;
  double t4;
  double t404;
  double t409;
  double t41;
  double t410;
  double t411;
  double t412;
  double t42;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t517;
  double t518;
  double t519;
  double t52;
  double t520;
  double t537_tmp_tmp;
  double t57;
  double t58;
  double t59;
  double t60;
  double t61;
  double t62;
  double t63;
  double t64;
  double t65;
  double t66;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t71;
  double t72;
  double t73;
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
  double t86;
  double t87;
  double t88;
  double t89;
  double t9;
  double t94;
  double t95;
  double t96;

  /* { */
  /* function [cost, gradient] = compute_cost_and_gradient_fcn(u_in) */
  /*  */
  /*     Omega_1_scaled = u_in(1); */
  /*     Omega_2_scaled = u_in(2); */
  /*     Omega_3_scaled = u_in(3); */
  /*     Omega_4_scaled = u_in(4); */
  /*  */
  /*     b_1_scaled = u_in(5); */
  /*     b_2_scaled = u_in(6); */
  /*     b_3_scaled = u_in(7); */
  /*     b_4_scaled = u_in(8); */
  /*  */
  /*     g_1_scaled = u_in(9); */
  /*     g_2_scaled = u_in(10); */
  /*     g_3_scaled = u_in(11); */
  /*     g_4_scaled = u_in(12); */
  /*  */
  /*     Theta_scaled = u_in(13); */
  /*     Phi_scaled = u_in(14); */
  /*     delta_ailerons_scaled = u_in(15); */
  /*  */
  /*     Alpha = Theta_scaled * gain_theta - flight_path_angle; */
  /*  */
  /*     dv_global_1 = dv_global(1); */
  /*     dv_global_2 = dv_global(2); */
  /*     dv_global_3 = dv_global(3); */
  /*     dv_global_4 = dv_global(4); */
  /*     dv_global_5 = dv_global(5); */
  /*     dv_global_6 = dv_global(6); */
  /*   */
  /*     gamma1 = 0.1250*((16*V^2*sin(Alpha + b_4_scaled*gain_el)^2)/(Omega_4_scaled^2*gain_motor^2*prop_R^2) - (prop_sigma*(prop_delta - 1)*(8*prop_Cl_0*prop_delta + 8*prop_Cl_0*prop_delta^2 + prop_Cl_a^2*prop_delta*prop_sigma - prop_Cl_a^2*prop_delta^2*prop_sigma + 16*prop_Cl_a*prop_delta*prop_theta + (8*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_4_scaled*gain_el)^2)/(Omega_4_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (8*V^2*prop_Cl_0*prop_sigma*cos(Alpha + b_4_scaled*gain_el)^2*log(prop_delta))/(Omega_4_scaled^2*gain_motor^2*prop_R^2) - (8*V*prop_Cl_a*prop_sigma*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1))/(Omega_4_scaled*gain_motor*prop_R))^(1/2); */
  /*     gamma2 = 0.1250*((16*V^2*sin(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2) - (prop_sigma*(prop_delta - 1)*(8*prop_Cl_0*prop_delta + 8*prop_Cl_0*prop_delta^2 + prop_Cl_a^2*prop_delta*prop_sigma - prop_Cl_a^2*prop_delta^2*prop_sigma + 16*prop_Cl_a*prop_delta*prop_theta + (8*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (8*V^2*prop_Cl_0*prop_sigma*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2) - (8*V*prop_Cl_a*prop_sigma*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1))/(Omega_3_scaled*gain_motor*prop_R))^(1/2); */
  /*     gamma3 = 0.1250*((16*V^2*sin(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2) - (prop_sigma*(prop_delta - 1)*(8*prop_Cl_0*prop_delta + 8*prop_Cl_0*prop_delta^2 + prop_Cl_a^2*prop_delta*prop_sigma - prop_Cl_a^2*prop_delta^2*prop_sigma + 16*prop_Cl_a*prop_delta*prop_theta + (8*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (8*V^2*prop_Cl_0*prop_sigma*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2) - (8*V*prop_Cl_a*prop_sigma*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1))/(Omega_2_scaled*gain_motor*prop_R))^(1/2); */
  /*     gamma4 = 0.1250*((16*V^2*sin(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2) - (prop_sigma*(prop_delta - 1)*(8*prop_Cl_0*prop_delta + 8*prop_Cl_0*prop_delta^2 + prop_Cl_a^2*prop_delta*prop_sigma - prop_Cl_a^2*prop_delta^2*prop_sigma + 16*prop_Cl_a*prop_delta*prop_theta + (8*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (8*V^2*prop_Cl_0*prop_sigma*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2) - (8*V*prop_Cl_a*prop_sigma*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1))/(Omega_1_scaled*gain_motor*prop_R))^(1/2); */
  /*     gamma5 = 0.1250*prop_Cl_a*prop_sigma*(prop_delta - 1); */
  /*     gamma6 = (prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma1 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_4_scaled*gain_el)^2)/(Omega_4_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)^2*log(prop_delta))/(Omega_4_scaled^2*gain_motor^2*prop_R^2); */
  /*     gamma7 = (0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_3_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma2 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta; */
  /*     gamma8 = (0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_3_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma2 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta; */
  /*     gamma9 = (0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_3_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma2 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta; */
  /*     gamma10 = (prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma2 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_3_scaled*gain_el)^2)/(Omega_3_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)^2*log(prop_delta))/(Omega_3_scaled^2*gain_motor^2*prop_R^2); */
  /*       */
  /*     sigma_1 = dv_global_4 + (0.5118*Omega_1_scaled - 0.5118*Omega_2_scaled - 0.2842*b_1_scaled + 0.2842*b_2_scaled + 0.2441*b_3_scaled - 0.2441*b_4_scaled + 0.3656*Omega_1_scaled*b_1_scaled - 0.3656*Omega_2_scaled*b_2_scaled - 0.1899*Omega_1_scaled*b_4_scaled + 0.1899*Omega_2_scaled*b_3_scaled + l_1*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_1*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - 0.1567*b_1_scaled^2 + 0.1567*b_2_scaled^2 + 0.1780*b_3_scaled^2 - 0.1780*b_4_scaled^2 - I_yy*q*r + I_zz*q*r + (1.8530*Omega_1_scaled*V)/gain_airspeed - (1.8530*Omega_2_scaled*V)/gain_airspeed + (0.5481*V*b_1_scaled)/gain_airspeed - (0.5481*V*b_2_scaled)/gain_airspeed - 0.5000*CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho)/I_xx; */
  /*     sigma_2 = dv_global_5 + (l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*r - I_zz*p*r)/I_yy; */
  /*     sigma_3 = dv_global_6 - (l_1*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_1*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*q - I_yy*p*q)/I_zz; */
  /*     sigma_4 = (2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)); */
  /*     sigma_5 = (2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)); */
  /*     sigma_6 = (2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)); */
  /*     sigma_7 = (2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)); */
  /*     sigma_8 = (prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2); */
  /*     sigma_9 = (prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2); */
  /*     sigma_10 = 0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta); */
  /*     sigma_11 = 1/prop_delta; */
  /*     sigma_12 = 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el); */
  /*     sigma_13 = 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el); */
  /*     sigma_14 = ((V*prop_Cl_a*prop_delta*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor*prop_R) + (2*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^3*gain_motor^2*prop_R^2))*(prop_delta - 1) + (2*V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^3*gain_motor^2*prop_R^2); */
  /*     sigma_15 = ((V*prop_Cl_a*prop_delta*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor*prop_R) + (2*V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^3*gain_motor^2*prop_R^2))*(prop_delta - 1) + (2*V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^3*gain_motor^2*prop_R^2); */
  /*      */
  /*     compute_gradient_and_cost = [ */
  /*       */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (2*W_dv_6^2*sigma_3*(l_4*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) + l_1*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*sin(b_1_scaled*gain_el) - 1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R)))))/I_zz + (2*W_dv_4^2*sigma_1*(0.3656*b_1_scaled - 0.1899*b_4_scaled + l_z*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) + (1.8530*V)/gain_airspeed + l_1*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) + 0.5118))/I_xx - (2*W_dv_5^2*sigma_2*(l_z*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*sin(b_1_scaled*gain_el) - 1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) - l_4*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R)))))/I_yy - 6*K_p_M*Omega_1_scaled^2*W_act_motor^2*gain_motor^2*gamma_quadratic_du*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1)*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1)) + (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*sin(b_1_scaled*gain_el) - 1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))) - l_4*(1.5708*Omega_1_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_15*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor*prop_R))))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (2*W_dv_6^2*sigma_3*(l_4*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) - l_1*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*sin(b_2_scaled*gain_el) - 1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R)))))/I_zz - (2*W_dv_4^2*sigma_1*(0.3656*b_2_scaled - 0.1899*b_3_scaled - l_z*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) + (1.8530*V)/gain_airspeed + l_1*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) + 0.5118))/I_xx - (2*W_dv_5^2*sigma_2*(l_z*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*sin(b_2_scaled*gain_el) - 1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) - l_4*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R)))))/I_yy - 6*K_p_M*Omega_2_scaled^2*W_act_motor^2*gain_motor^2*gamma_quadratic_du*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1)*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1)) + (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*sin(b_2_scaled*gain_el) - 1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))) - l_4*(1.5708*Omega_2_scaled*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sigma_14*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor*prop_R))))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     (2*W_dv_5^2*sigma_2*(l_z*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R)))))/I_yy - (2*W_dv_4^2*sigma_1*(l_1*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))) - l_z*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R)))))/I_xx + (2*W_dv_6^2*sigma_3*(l_1*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R)))))/I_zz - 6*K_p_M*Omega_3_scaled^2*W_act_motor^2*gain_motor^2*gamma_quadratic_du*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1)*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1)) - (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_3_scaled*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled^2*gain_motor*prop_R))))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             (2*W_dv_4^2*sigma_1*(l_1*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))) + l_z*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R)))))/I_xx + (2*W_dv_5^2*sigma_2*(l_z*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R)))))/I_yy - (2*W_dv_6^2*sigma_3*(l_1*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))) + l_3*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R)))))/I_zz - 6*K_p_M*Omega_4_scaled^2*W_act_motor^2*gain_motor^2*gamma_quadratic_du*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1)*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1)) - (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el) + 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))) - l_3*(1.5708*Omega_4_scaled*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*prop_theta*sin(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled^2*gain_motor*prop_R) - (0.5000*V*prop_Cl_0*prop_delta*sin(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled^2*gain_motor*prop_R))))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                   W_act_tilt_el^2*gamma_quadratic_du*(2*b_1_scaled - (2*desired_el_value)/gain_el) + (2*W_dv_6^2*sigma_3*(l_1*(0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_1_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)) + l_4*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az))))/I_zz + (2*W_dv_4^2*sigma_1*(0.3656*Omega_1_scaled - 0.3134*b_1_scaled + l_1*sigma_13 + l_z*(0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)) + (0.5481*V)/gain_airspeed - 0.2842))/I_xx - (2*W_dv_5^2*sigma_2*(l_z*(0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_1_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)) - l_4*sigma_13))/I_yy + (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_1_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_1_scaled*gain_motor*prop_R)) - 0.7854*Omega_1_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el) - 0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_1_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)*sin(Alpha + b_1_scaled*gain_el)*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el) + 0.7854*Omega_1_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_7*sigma_11*sin(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)) - l_4*sigma_13)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 + 4*K_p_M*Omega_1_scaled^3*V*W_act_motor^2*gain_el*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                   W_act_tilt_el^2*gamma_quadratic_du*(2*b_2_scaled - (2*desired_el_value)/gain_el) - (2*W_dv_6^2*sigma_3*(l_1*(0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_2_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)) - l_4*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az))))/I_zz - (2*W_dv_4^2*sigma_1*(0.3656*Omega_2_scaled - 0.3134*b_2_scaled + l_1*sigma_12 - l_z*(0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)) + (0.5481*V)/gain_airspeed - 0.2842))/I_xx - (2*W_dv_5^2*sigma_2*(l_z*(0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_2_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)) - l_4*sigma_12))/I_yy + (S*V^2*W_dv_5^2*rho*wing_chord*(l_z*(0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_2_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_2_scaled*gain_motor*prop_R)) - 0.7854*Omega_2_scaled^2*gain_el*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el) - 0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sigma_11*sin(b_2_scaled*gain_el)*((prop_delta - 1)*((V*gain_el*prop_Cl_a*prop_delta*cos(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R) - (2*V^2*gain_el*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) - (2*V^2*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)*sin(Alpha + b_2_scaled*gain_el)*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el) + 0.7854*Omega_2_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_6*sigma_11*sin(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)) - l_4*sigma_12)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 + 4*K_p_M*Omega_2_scaled^3*V*W_act_motor^2*gain_el*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                                                                       W_act_tilt_el^2*gamma_quadratic_du*(2*b_3_scaled - (2*desired_el_value)/gain_el) + (2*W_dv_5^2*sigma_2*(l_3*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)) - l_z*(0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) - 0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el))))/I_yy + (2*W_dv_6^2*sigma_3*(l_3*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)) - l_1*(0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) - 0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el))))/I_zz + (2*W_dv_4^2*sigma_1*(0.1899*Omega_2_scaled + 0.3559*b_3_scaled + l_1*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)) - l_z*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)) + 0.2441))/I_xx - (S*V^2*W_dv_5^2*rho*wing_chord*(l_3*(0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) - 0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)) - l_z*(0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_3_scaled*gain_el)*log(prop_delta))/(Omega_3_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_3_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_3_scaled*gain_motor*prop_R)) - 0.7854*Omega_3_scaled^2*gain_el*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el) + 0.7854*Omega_3_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_4*sigma_11*sin(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 + 4*K_p_M*Omega_3_scaled^3*V*W_act_motor^2*gain_el*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                                                                               W_act_tilt_el^2*gamma_quadratic_du*(2*b_4_scaled - (2*desired_el_value)/gain_el) + (2*W_dv_5^2*sigma_2*(l_3*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)) - l_z*(0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) - 0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el))))/I_yy + (2*W_dv_6^2*sigma_3*(l_3*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)) + l_1*(0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) - 0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el))))/I_zz - (2*W_dv_4^2*sigma_1*(0.1899*Omega_1_scaled + 0.3559*b_4_scaled + l_1*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)) + l_z*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)) + 0.2441))/I_xx - (S*V^2*W_dv_5^2*rho*wing_chord*(l_3*(0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) - 0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)) - l_z*(0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((0.5000*V*gain_el*prop_Cl_0*prop_delta*cos(Alpha + b_4_scaled*gain_el)*log(prop_delta))/(Omega_4_scaled*gain_motor*prop_R) - (0.5000*V*gain_el*prop_theta*cos(Alpha + b_4_scaled*gain_el)*(prop_delta - 1)*(2*prop_Cd_a - prop_Cl_a))/(Omega_4_scaled*gain_motor*prop_R)) - 0.7854*Omega_4_scaled^2*gain_el*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el) + 0.7854*Omega_4_scaled*V*gain_el*gain_motor*prop_R^3*prop_sigma*rho*sigma_5*sigma_11*sin(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)))*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 + 4*K_p_M*Omega_4_scaled^3*V*W_act_motor^2*gain_el*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          W_act_tilt_az^2*gamma_quadratic_du*(2*g_1_scaled - (2*desired_az_value)/gain_az) + (2*W_dv_4^2*sigma_1*(l_z*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*Omega_1_scaled*gain_motor*prop_R^3) - l_1*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled*gain_motor*prop_R^3)))/I_xx + (2*W_dv_6^2*l_4*sigma_3*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*Omega_1_scaled*gain_motor*prop_R^3))/I_zz - (2*W_dv_5^2*l_4*sigma_2*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled*gain_motor*prop_R^3))/I_yy + (S*V^2*W_dv_5^2*l_4*rho*wing_chord*(0.7854*gain_az*prop_sigma*rho*sigma_8*sigma_11*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_7*sigma_11*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled*gain_motor*prop_R^3)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          W_act_tilt_az^2*gamma_quadratic_du*(2*g_2_scaled - (2*desired_az_value)/gain_az) + (2*W_dv_4^2*sigma_1*(l_z*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*Omega_2_scaled*gain_motor*prop_R^3) + l_1*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled*gain_motor*prop_R^3)))/I_xx + (2*W_dv_6^2*l_4*sigma_3*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*Omega_2_scaled*gain_motor*prop_R^3))/I_zz - (2*W_dv_5^2*l_4*sigma_2*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled*gain_motor*prop_R^3))/I_yy + (S*V^2*W_dv_5^2*l_4*rho*wing_chord*(0.7854*gain_az*prop_sigma*rho*sigma_9*sigma_11*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_6*sigma_11*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled*gain_motor*prop_R^3)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          W_act_tilt_az^2*gamma_quadratic_du*(2*g_3_scaled - (2*desired_az_value)/gain_az) + (2*W_dv_4^2*sigma_1*(l_z*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*Omega_3_scaled*gain_motor*prop_R^3) + l_1*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled*gain_motor*prop_R^3)))/I_xx - (2*W_dv_6^2*l_3*sigma_3*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*Omega_3_scaled*gain_motor*prop_R^3))/I_zz + (2*W_dv_5^2*l_3*sigma_2*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled*gain_motor*prop_R^3))/I_yy - (S*V^2*W_dv_5^2*l_3*rho*wing_chord*(0.7854*gain_az*gamma10*prop_sigma*rho*sigma_11*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_4*sigma_11*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled*gain_motor*prop_R^3)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               W_act_tilt_az^2*gamma_quadratic_du*(2*g_4_scaled - (2*desired_az_value)/gain_az) + (2*W_dv_4^2*sigma_1*(l_z*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*Omega_4_scaled*gain_motor*prop_R^3) - l_1*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled*gain_motor*prop_R^3)))/I_xx - (2*W_dv_6^2*l_3*sigma_3*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*Omega_4_scaled*gain_motor*prop_R^3))/I_zz + (2*W_dv_5^2*l_3*sigma_2*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled*gain_motor*prop_R^3))/I_yy - (S*V^2*W_dv_5^2*l_3*rho*wing_chord*(0.7854*gain_az*gamma6*prop_sigma*rho*sigma_11*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2*gain_motor^2*prop_R^4 - 0.7854*V*gain_az*prop_sigma*rho*sigma_5*sigma_11*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled*gain_motor*prop_R^3)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2; */
  /*     W_act_theta^2*gamma_quadratic_du*(2*Theta_scaled - (2*desired_theta_value)/gain_theta) - (2*W_dv_3^2*((sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gamma9*sin(Theta_scaled*gain_theta) + gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))/m - dv_global_3 + 9.8100)*(gain_theta*gamma9*cos(Theta_scaled*gain_theta) - gain_theta*cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta) - 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gain_theta*gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + gain_theta*sigma_10*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gain_theta*gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta)))/m + (2*W_dv_1^2*(dv_global_1 - (gamma9*cos(Theta_scaled*gain_theta) - cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))/m)*(cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gain_theta*sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gain_theta*gamma9*sin(Theta_scaled*gain_theta) + cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta) - 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gain_theta*gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gain_theta*sigma_10*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + gain_theta*gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi)))/m - (2*W_dv_2^2*sin(Phi_scaled*gain_phi)*(dv_global_2 + (gamma8*cos(Phi_scaled*gain_phi) + gamma7*sin(Phi_scaled*gain_phi) + sigma_10*sin(Phi_scaled*gain_phi))/m)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(- 2*K_Cd*Theta_scaled*Cl_alpha^2*gain_theta^2 + 2*K_Cd*flight_path_angle*Cl_alpha^2*gain_theta) + 0.5000*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*gain_theta*rho*cos(flight_path_angle - Theta_scaled*gain_theta) - 0.5000*Cl_alpha*S*V^2*gain_theta*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)))/m + (0.5000*Cm_alpha*S^2*V^4*W_dv_5^2*gain_theta*rho^2*wing_chord^2*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy^2 - (Cm_alpha*S*V^2*W_dv_5^2*gain_theta*rho*sigma_2*wing_chord)/I_yy + 4*K_p_M*Omega_1_scaled^3*V*W_act_motor^2*gain_theta*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1)) + 4*K_p_M*Omega_2_scaled^3*V*W_act_motor^2*gain_theta*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1)) + 4*K_p_M*Omega_3_scaled^3*V*W_act_motor^2*gain_theta*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1)) + 4*K_p_M*Omega_4_scaled^3*V*W_act_motor^2*gain_theta*gain_motor^2*gamma_quadratic_du*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)*sin(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1)); */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  gamma_quadratic_du*(2*Phi_scaled - (2*desired_phi_value)/gain_phi)*W_act_phi^2 + (2*(dv_global_1 - (gamma9*cos(Theta_scaled*gain_theta) - cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))/m)*(gain_phi*gamma8*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + gain_phi*gamma7*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + gain_phi*sigma_10*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))*W_dv_1^2)/m + (2*(dv_global_2 + (gamma8*cos(Phi_scaled*gain_phi) + gamma7*sin(Phi_scaled*gain_phi) + sigma_10*sin(Phi_scaled*gain_phi))/m)*(gain_phi*gamma7*cos(Phi_scaled*gain_phi) + gain_phi*sigma_10*cos(Phi_scaled*gain_phi) - gain_phi*gamma8*sin(Phi_scaled*gain_phi))*W_dv_2^2)/m - (2*W_dv_3^2*(gain_phi*gamma8*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + gain_phi*gamma7*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi) + gain_phi*sigma_10*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))*((sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gamma9*sin(Theta_scaled*gain_theta) + gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))/m - dv_global_3 + 9.8100))/m; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          gamma_quadratic_du*(2*delta_ailerons_scaled - (2*desired_ailerons_value)/gain_ailerons)*W_act_ailerons^2 - (CL_aileron*S*V^2*W_dv_4^2*gain_ailerons*rho*sigma_1)/I_xx; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               W_dv_3^2*((sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gamma9*sin(Theta_scaled*gain_theta) + gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))/m - dv_global_3 + 9.8100)^2 + W_dv_1^2*(dv_global_1 - (gamma9*cos(Theta_scaled*gain_theta) - cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) + sigma_10*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))/m)^2 + W_dv_2^2*(dv_global_2 + (gamma8*cos(Phi_scaled*gain_phi) + gamma7*sin(Phi_scaled*gain_phi) + sigma_10*sin(Phi_scaled*gain_phi))/m)^2 + W_dv_4^2*sigma_1^2 + W_dv_5^2*sigma_2^2 + W_dv_6^2*sigma_3^2 + W_act_phi^2*gamma_quadratic_du*(Phi_scaled - desired_phi_value/gain_phi)^2 + W_act_theta^2*gamma_quadratic_du*(Theta_scaled - desired_theta_value/gain_theta)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_1_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_2_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_3_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_4_scaled - desired_el_value/gain_el)^2 + W_act_ailerons^2*gamma_quadratic_du*(delta_ailerons_scaled - desired_ailerons_value/gain_ailerons)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_1_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_2_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_3_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_4_scaled - desired_az_value/gain_az)^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1))^2 + (0.2500*S^2*V^4*W_dv_5^2*rho^2*wing_chord^2*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta)^2)/I_yy^2 - (S*V^2*W_dv_5^2*rho*sigma_2*wing_chord*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy]; */
  /*       */
  /*     cost = compute_gradient_and_cost(16); */
  /*     gradient = compute_gradient_and_cost(1:15); */
  /*  */
  /*     cost_parametric = W_dv_3^2*((sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) - gamma9*sin(Theta_scaled*gain_theta) + cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*cos(Theta_scaled*gain_theta) - gamma8*cos(Theta_scaled*gain_theta)*sin(Phi_scaled*gain_phi))/m - dv_global_3 + 9.8100)^2 + W_dv_5^2*(dv_global_5 + (l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*r - I_zz*p*r)/I_yy)^2 + W_dv_4^2*(dv_global_4 + (0.5118*Omega_1_scaled - 0.5118*Omega_2_scaled - 0.2842*b_1_scaled + 0.2842*b_2_scaled + 0.2441*b_3_scaled - 0.2441*b_4_scaled + 0.3656*Omega_1_scaled*b_1_scaled - 0.3656*Omega_2_scaled*b_2_scaled - 0.1899*Omega_1_scaled*b_4_scaled + 0.1899*Omega_2_scaled*b_3_scaled + l_1*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_1*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - 0.1567*b_1_scaled^2 + 0.1567*b_2_scaled^2 + 0.1780*b_3_scaled^2 - 0.1780*b_4_scaled^2 - I_yy*q*r + I_zz*q*r + (1.8530*Omega_1_scaled*V)/gain_airspeed - (1.8530*Omega_2_scaled*V)/gain_airspeed + (0.5481*V*b_1_scaled)/gain_airspeed - (0.5481*V*b_2_scaled)/gain_airspeed - 0.5000*CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho)/I_xx)^2 + W_dv_1^2*(dv_global_1 - (gamma9*cos(Theta_scaled*gain_theta) - cos(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) - 0.5000*Cl_alpha*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma7*cos(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta) - gamma8*sin(Phi_scaled*gain_phi)*sin(Theta_scaled*gain_theta))/m)^2 + W_dv_2^2*(dv_global_2 + (sin(Phi_scaled*gain_phi)*(0.5000*S*V^2*rho*sin(flight_path_angle - Theta_scaled*gain_theta)*(K_Cd*Cl_alpha^2*Theta_scaled^2*gain_theta^2 - 2*K_Cd*Cl_alpha^2*Theta_scaled*flight_path_angle*gain_theta + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero) + 0.5000*Cl_alpha*S*V^2*rho*cos(flight_path_angle - Theta_scaled*gain_theta)*(flight_path_angle - Theta_scaled*gain_theta)) + gamma8*cos(Phi_scaled*gain_phi) + gamma7*sin(Phi_scaled*gain_phi))/m)^2 + W_dv_6^2*(dv_global_6 - (l_1*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) - l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) - l_1*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_1*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*q - I_yy*p*q)/I_zz)^2 + W_act_phi^2*gamma_quadratic_du*(Phi_scaled - desired_phi_value/gain_phi)^2 + W_act_theta^2*gamma_quadratic_du*(Theta_scaled - desired_theta_value/gain_theta)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_1_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_2_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_3_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el^2*gamma_quadratic_du*(b_4_scaled - desired_el_value/gain_el)^2 + W_act_ailerons^2*gamma_quadratic_du*(delta_ailerons_scaled - desired_ailerons_value/gain_ailerons)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_1_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_2_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_3_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az^2*gamma_quadratic_du*(g_4_scaled - desired_az_value/gain_az)^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_1_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_1_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_2_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_2_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_3_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_3_scaled*gain_el)^2 + 1))^2 + W_act_motor^2*gamma_quadratic_du*(desired_motor_value/gain_motor - K_p_M*Omega_4_scaled^3*gain_motor^2*(V*k_tilt*cos(Theta_scaled*gain_theta - flight_path_angle + b_4_scaled*gain_el)^2 + 1))^2 + (0.2500*S^2*V^4*W_dv_5^2*rho^2*wing_chord^2*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta)^2)/I_yy^2 - (S*V^2*W_dv_5^2*rho*wing_chord*(dv_global_5 + (l_z*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_1_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*sin(b_2_scaled*gain_el)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta + (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_1_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma4 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_1_scaled*gain_el)^2)/(Omega_1_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_1_scaled*gain_el)^2*log(prop_delta))/(Omega_1_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_1_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma4 + gamma5 - (0.5000*V*sin(Alpha + b_1_scaled*gain_el))/(Omega_1_scaled*gain_motor*prop_R))))/prop_delta) + l_4*((0.7854*Omega_2_scaled^2*gain_motor^2*prop_R^4*prop_sigma*rho*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*((prop_delta - 1)*(prop_Cl_0*prop_delta*(prop_delta + 1) - 2*prop_Cl_a*prop_delta*(gamma3 + gamma5 - prop_theta - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) + (V^2*prop_Cl_a*prop_theta*cos(Alpha + b_2_scaled*gain_el)^2)/(Omega_2_scaled^2*gain_motor^2*prop_R^2)) + (V^2*prop_Cl_0*prop_delta*cos(Alpha + b_2_scaled*gain_el)^2*log(prop_delta))/(Omega_2_scaled^2*gain_motor^2*prop_R^2)))/prop_delta - (0.7854*Omega_2_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma3 + gamma5 - (0.5000*V*sin(Alpha + b_2_scaled*gain_el))/(Omega_2_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*sin(b_3_scaled*gain_el))/prop_delta + (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) + l_z*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*sin(b_4_scaled*gain_el))/prop_delta + (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_3_scaled^2*gain_motor^2*gamma10*prop_R^4*prop_sigma*rho*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))/prop_delta - (0.7854*Omega_3_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma2 + gamma5 - (0.5000*V*sin(Alpha + b_3_scaled*gain_el))/(Omega_3_scaled*gain_motor*prop_R))))/prop_delta) - l_3*((0.7854*Omega_4_scaled^2*gain_motor^2*gamma6*prop_R^4*prop_sigma*rho*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))/prop_delta - (0.7854*Omega_4_scaled*V*gain_motor*prop_R^3*prop_sigma*rho*cos(Alpha + b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*((2*prop_Cd_0*prop_delta - prop_theta*((2*prop_Cd_a - prop_Cl_a)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R)) - 2*prop_Cd_a*prop_theta))*(prop_delta - 1) + prop_Cl_0*prop_delta*log(prop_delta)*(gamma1 + gamma5 - (0.5000*V*sin(Alpha + b_4_scaled*gain_el))/(Omega_4_scaled*gain_motor*prop_R))))/prop_delta) + I_xx*p*r - I_zz*p*r)/I_yy)*(Cm_zero - Cm_alpha*flight_path_angle + Cm_alpha*Theta_scaled*gain_theta))/I_yy; */
  /*     disp(cost - cost_parametric) */
  /*  */
  /* end */
  /*     %} */
  /* COMPUTE_COST_AND_GRADIENT_FCN */
  /*     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT_FCN(Alpha,Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AIRSPEED,GAIN_AILERONS,GAMMA_QUADRATIC_DU,K_TILT,L_1,L_3,L_4,L_Z,M,P,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,Q,R,RHO,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 9.3. */
  /*     16-Nov-2023 16:59:28 */
  t165 = u_in[12] * gain_theta->contents;
  Alpha = t165 - flight_path_angle->contents;
  t2 = cos(Beta->contents);
  t3 = sin(Beta->contents);
  t4 = log(prop_delta->contents);
  t5 = u_in[13] * gain_phi->contents;
  t7 = u_in[4] * gain_el->contents;
  t8 = u_in[5] * gain_el->contents;
  t9 = u_in[6] * gain_el->contents;
  t10 = u_in[7] * gain_el->contents;
  t11 = u_in[8] * gain_az->contents;
  t12 = u_in[9] * gain_az->contents;
  t13 = u_in[10] * gain_az->contents;
  t14 = u_in[11] * gain_az->contents;
  a = Cl_alpha->contents;
  t15 = a * a;
  t16 = u_in[0] * u_in[0];
  t17 = rt_powd_snf(u_in[0], 3.0);
  t18 = u_in[1] * u_in[1];
  t19 = rt_powd_snf(u_in[1], 3.0);
  t20 = u_in[2] * u_in[2];
  t21 = rt_powd_snf(u_in[2], 3.0);
  t22 = u_in[3] * u_in[3];
  t23 = rt_powd_snf(u_in[3], 3.0);
  a = V->contents;
  t24 = a * a;
  a = W_act_phi->contents;
  b_a = W_act_theta->contents;
  c_a = W_act_motor->contents;
  d_a = W_dv_1->contents;
  e_a = W_dv_2->contents;
  f_a = W_dv_3->contents;
  g_a = W_dv_4->contents;
  h_a = W_dv_5->contents;
  i_a = W_dv_6->contents;
  j_a = W_act_tilt_el->contents;
  k_a = W_act_tilt_az->contents;
  l_a = W_act_ailerons->contents;
  t89 = gain_motor->contents;
  t41 = t89 * t89;
  t42 = prop_Cd_a->contents * 2.0;
  t44 = rt_powd_snf(prop_R->contents, 3.0);
  t45 = rt_powd_snf(prop_R->contents, 4.0);
  t46 = prop_Cd_0->contents * prop_delta->contents * 2.0;
  t77 = 1.0 / u_in[0];
  t79 = 1.0 / u_in[1];
  t82 = 1.0 / u_in[2];
  t85 = 1.0 / u_in[3];
  t89 = -flight_path_angle->contents;
  t94 = 1.0 / gain_motor->contents;
  t96 = 1.0 / gain_airspeed->contents;
  t100 = -prop_theta->contents;
  t101 = prop_delta->contents * 16.0;
  t102 = prop_delta->contents - 1.0;
  t103 = 1.0 / prop_R->contents;
  t105 = 1.0 / prop_delta->contents;
  t47 = prop_theta->contents * t42;
  t48 = cos(t5);
  t49 = cos(t165);
  t52 = sin(t5);
  t57 = cos(t7);
  t58 = cos(t8);
  t59 = cos(t9);
  t60 = cos(t10);
  t61 = sin(t165);
  t62 = cos(t11);
  t63 = cos(t12);
  t64 = cos(t13);
  t65 = cos(t14);
  t66 = sin(t7);
  t67 = sin(t8);
  t68 = sin(t9);
  t69 = sin(t10);
  t70 = sin(t11);
  t71 = sin(t12);
  t72 = sin(t13);
  t73 = sin(t14);
  t78 = 1.0 / t16;
  t80 = 1.0 / t17;
  t81 = 1.0 / t18;
  t83 = 1.0 / t19;
  t84 = 1.0 / t20;
  t86 = 1.0 / t21;
  t87 = 1.0 / t22;
  t88 = 1.0 / t23;
  t95 = 1.0 / t41;
  t104 = t103 * t103;
  t14 = prop_Cl_0->contents * prop_delta->contents;
  t108 = t14 * (prop_delta->contents + 1.0);
  t110 = desired_el_value->contents * (1.0 / gain_el->contents);
  t111 = desired_az_value->contents * (1.0 / gain_az->contents);
  t112 = desired_motor_value->contents * t94;
  t5 = (Alpha + t7) + 1.5707963267948966;
  t11 = (Alpha + t8) + 1.5707963267948966;
  t12 = (Alpha + t9) + 1.5707963267948966;
  t13 = (Alpha + t10) + 1.5707963267948966;
  t133 = prop_delta->contents * prop_sigma->contents * -prop_Cl_a->contents *
    t102;
  t520 = prop_Cl_a->contents * prop_sigma->contents;
  t134 = t520 * t102 / 8.0;
  t135 = (t165 + t7) + t89;
  t136 = (t165 + t8) + t89;
  t137 = (t165 + t9) + t89;
  t138 = (t165 + t10) + t89;
  t89 = V->contents * t96;
  t189 = t89 * 1.853;
  t202 = t89 * 0.54813;
  t117 = flight_path_angle->contents - t165;
  Alpha = t108 * 8.0;
  t140 = cos(t5);
  t141 = cos(t11);
  t142 = cos(t12);
  t143 = cos(t13);
  t144 = sin(t5);
  t145 = sin(t11);
  t146 = sin(t12);
  t147 = sin(t13);
  t152 = cos(t135);
  t153 = cos(t136);
  t154 = cos(t137);
  t155 = cos(t138);
  t119 = cos(t117);
  t120 = sin(t117);
  t89 = t140 * t140;
  t165 = t141 * t141;
  t166 = t142 * t142;
  t167 = t143 * t143;
  t168 = t144 * t144;
  t169 = t145 * t145;
  t170 = t146 * t146;
  t171 = t147 * t147;
  t5 = V->contents * k_tilt->contents;
  t197 = t5 * (t152 * t152);
  t198 = t5 * (t153 * t153);
  t199 = t5 * (t154 * t154);
  t200 = t5 * (t155 * t155);
  t11 = Cl_alpha->contents * S->contents;
  t207_tmp = t11 * gain_theta->contents * rho->contents * t24;
  t207 = t207_tmp * t119 / 2.0;
  t259_tmp = prop_Cl_a->contents * prop_theta->contents * t24;
  t283_tmp_tmp = t14 * t4;
  t283_tmp = t283_tmp_tmp * t24;
  t193 = Cd_zero->contents + K_Cd->contents * t15 * (t117 * t117);
  t287_tmp_tmp = S->contents * rho->contents;
  t5 = t287_tmp_tmp * t2 * t24;
  t287 = t5 * t120 * t193 / 2.0;
  t302 = t5 * t119 * t193 / 2.0;
  t5 = t11 * rho->contents * t24 * t117;
  t367 = t5 * t119 / 2.0 + t287;
  t368 = t5 * t120 / 2.0 - t302;
  t5 = t24 * t78 * t95 * t104;
  t7 = V->contents * prop_Cl_a->contents * prop_sigma->contents;
  t8 = prop_Cl_0->contents * prop_sigma->contents * t4 * t24;
  t11 = prop_sigma->contents * t102 * t105;
  t12 = sqrt(((t5 * t89 * 16.0 + t7 * t77 * t94 * t102 * t103 * t140 * 8.0) + t8
              * t78 * t95 * t104 * t168 * -8.0) - t11 * (Alpha +
              prop_Cl_a->contents * (t133 + prop_theta->contents * (t101 + t5 *
    t168 * 8.0))));
  t5 = t24 * t81 * t95 * t104;
  t13 = sqrt(((t5 * t165 * 16.0 + t7 * t79 * t94 * t102 * t103 * t141 * 8.0) +
              t8 * t81 * t95 * t104 * t169 * -8.0) - t11 * (Alpha +
              prop_Cl_a->contents * (t133 + prop_theta->contents * (t101 + t5 *
    t169 * 8.0))));
  t5 = t24 * t84 * t95 * t104;
  t14 = sqrt(((t5 * t166 * 16.0 + t7 * t82 * t94 * t102 * t103 * t142 * 8.0) +
              t8 * t84 * t95 * t104 * t170 * -8.0) - t11 * (Alpha +
              prop_Cl_a->contents * (t133 + prop_theta->contents * (t101 + t5 *
    t170 * 8.0))));
  t5 = t24 * t87 * t95 * t104;
  t5 = sqrt(((t5 * t167 * 16.0 + t7 * t85 * t94 * t102 * t103 * t143 * 8.0) + t8
             * t87 * t95 * t104 * t171 * -8.0) - t11 * (Alpha +
             prop_Cl_a->contents * (t133 + prop_theta->contents * (t101 + t5 *
    t171 * 8.0))));
  Alpha = 1.0 / t12;
  t9 = 1.0 / t13;
  t10 = 1.0 / t14;
  t404 = 1.0 / t5;
  t409 = (t134 + V->contents * t77 * t94 * t103 * t140 / 2.0) + t12 / 8.0;
  t410 = (t134 + V->contents * t79 * t94 * t103 * t141 / 2.0) + t13 / 8.0;
  t411 = (t134 + V->contents * t82 * t94 * t103 * t142 / 2.0) + t14 / 8.0;
  t412 = (t134 + V->contents * t85 * t94 * t103 * t143 / 2.0) + t5 / 8.0;
  t5 = t520 * prop_theta->contents * t24;
  t517 = V->contents * t78 * t94 * t103 * t140 / 2.0 + (((t24 * t80 * t95 * t104
    * t89 * 32.0 + t7 * t78 * t94 * t102 * t103 * t140 * 8.0) - t8 * t80 * t95 *
    t104 * t168 * 16.0) - t5 * t80 * t95 * t102 * t104 * t105 * t168 * 16.0) *
    Alpha / 16.0;
  t518 = V->contents * t81 * t94 * t103 * t141 / 2.0 + (((t24 * t83 * t95 * t104
    * t165 * 32.0 + t7 * t81 * t94 * t102 * t103 * t141 * 8.0) - t8 * t83 * t95 *
    t104 * t169 * 16.0) - t5 * t83 * t95 * t102 * t104 * t105 * t169 * 16.0) *
    t9 / 16.0;
  t519 = V->contents * t84 * t94 * t103 * t142 / 2.0 + (((t24 * t86 * t95 * t104
    * t166 * 32.0 + t7 * t84 * t94 * t102 * t103 * t142 * 8.0) - t8 * t86 * t95 *
    t104 * t170 * 16.0) - t5 * t86 * t95 * t102 * t104 * t105 * t170 * 16.0) *
    t10 / 16.0;
  t520 = V->contents * t87 * t94 * t103 * t143 / 2.0 + (((t24 * t88 * t95 * t104
    * t167 * 32.0 + t7 * t87 * t94 * t102 * t103 * t143 * 8.0) - t8 * t88 * t95 *
    t104 * t171 * 16.0) - t5 * t88 * t95 * t102 * t104 * t105 * t171 * 16.0) *
    t404 / 16.0;
  t5 = V->contents * gain_el->contents;
  t11 = t5 * prop_Cl_a->contents * prop_sigma->contents;
  t12 = gain_el->contents * t24;
  t537_tmp_tmp = gain_el->contents * prop_Cl_0->contents;
  t13 = t537_tmp_tmp * prop_sigma->contents * t4 * t24;
  b_t537_tmp_tmp = gain_el->contents * prop_Cl_a->contents;
  t14 = b_t537_tmp_tmp * prop_sigma->contents * prop_theta->contents * t24;
  t77 = t5 * t77 * t94 * t103 * t144 / 2.0 + (((t11 * t77 * t94 * t102 * t103 *
    t144 * 8.0 + t12 * t78 * t95 * t104 * t140 * t144 * 32.0) + t13 * t78 * t95 *
    t104 * t140 * t144 * 16.0) + t14 * t78 * t95 * t102 * t104 * t105 * t140 *
    t144 * 16.0) * Alpha / 16.0;
  t134 = t5 * t79 * t94 * t103 * t145 / 2.0 + (((t11 * t79 * t94 * t102 * t103 *
    t145 * 8.0 + t12 * t81 * t95 * t104 * t141 * t145 * 32.0) + t13 * t81 * t95 *
    t104 * t141 * t145 * 16.0) + t14 * t81 * t95 * t102 * t104 * t105 * t141 *
    t145 * 16.0) * t9 / 16.0;
  t101 = t5 * t82 * t94 * t103 * t146 / 2.0 + (((t11 * t82 * t94 * t102 * t103 *
    t146 * 8.0 + t12 * t84 * t95 * t104 * t142 * t146 * 32.0) + t13 * t84 * t95 *
    t104 * t142 * t146 * 16.0) + t14 * t84 * t95 * t102 * t104 * t105 * t142 *
    t146 * 16.0) * t10 / 16.0;
  t404 = t5 * t85 * t94 * t103 * t147 / 2.0 + (((t11 * t85 * t94 * t102 * t103 *
    t147 * 8.0 + t12 * t87 * t95 * t104 * t143 * t147 * 32.0) + t13 * t87 * t95 *
    t104 * t143 * t147 * 16.0) + t14 * t87 * t95 * t102 * t104 * t105 * t143 *
    t147 * 16.0) * t404 / 16.0;
  t166 = prop_Cl_a->contents * prop_delta->contents;
  expl_temp.f178 = 1.0 / m->contents;
  expl_temp.f177 = 1.0 / gain_ailerons->contents;
  expl_temp.f176 = 1.0 / gain_theta->contents;
  expl_temp.f175 = 1.0 / gain_phi->contents;
  expl_temp.f174 = 1.0 / I_zz->contents;
  expl_temp.f173 = 1.0 / I_yy->contents;
  expl_temp.f172 = 1.0 / I_xx->contents;
  expl_temp.f171 = t73;
  t167 = prop_theta->contents * t102;
  t13 = prop_Cl_a->contents - t42;
  expl_temp.f170 = t283_tmp_tmp * t404 + t167 * t404 * t13;
  expl_temp.f169 = t283_tmp_tmp * t101 + t167 * t101 * t13;
  expl_temp.f168 = t283_tmp_tmp * t134 + t167 * t134 * t13;
  expl_temp.f167 = t283_tmp_tmp * t77 + t167 * t77 * t13;
  t14 = u_in[3] * V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * t44;
  Alpha = t283_tmp_tmp * t520 + t167 * t520 * t13;
  expl_temp.f166 = t14 * t69 * t73 * t105 * t147 * 3.1415926535897931 * Alpha /
    4.0;
  t7 = u_in[2] * V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * t44;
  t8 = t283_tmp_tmp * t519 + t167 * t519 * t13;
  expl_temp.f165 = t7 * t68 * t72 * t105 * t146 * 3.1415926535897931 * t8 / 4.0;
  t9 = u_in[1] * V->contents;
  t89 = t9 * gain_motor->contents * prop_sigma->contents * rho->contents * t44;
  t10 = t283_tmp_tmp * t518 + t167 * t518 * t13;
  expl_temp.f164 = t89 * t67 * t71 * t105 * t145 * 3.1415926535897931 * t10 /
    4.0;
  t165 = u_in[0] * V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * t44;
  t167 = t283_tmp_tmp * t517 + t167 * t517 * t13;
  expl_temp.f163 = t165 * t66 * t70 * t105 * t144 * 3.1415926535897931 * t167 /
    4.0;
  expl_temp.f162 = t14 * t65 * t69 * t105 * t147 * 3.1415926535897931 * Alpha /
    4.0;
  expl_temp.f161 = t72;
  expl_temp.f160 = t7 * t64 * t68 * t105 * t146 * 3.1415926535897931 * t8 / 4.0;
  expl_temp.f159 = t89 * t63 * t67 * t105 * t145 * 3.1415926535897931 * t10 /
    4.0;
  expl_temp.f158 = t165 * t62 * t66 * t105 * t144 * 3.1415926535897931 * t167 /
    4.0;
  expl_temp.f157 = t71;
  expl_temp.f156 = t14 * t60 * t105 * t147 * 3.1415926535897931 * Alpha * -0.25;
  expl_temp.f155 = t7 * t59 * t105 * t146 * 3.1415926535897931 * t8 * -0.25;
  expl_temp.f154 = t89 * t58 * t105 * t145 * 3.1415926535897931 * t10 * -0.25;
  expl_temp.f153 = t165 * t57 * t105 * t144 * 3.1415926535897931 * t167 * -0.25;
  expl_temp.f152 = t70;
  expl_temp.f151 = t69;
  expl_temp.f150 = t68;
  expl_temp.f149 = t67;
  expl_temp.f148 = t66;
  expl_temp.f147 = t65;
  expl_temp.f146 = t64;
  expl_temp.f145 = t63;
  expl_temp.f144 = t62;
  expl_temp.f143 = t61;
  expl_temp.f142 = t60;
  expl_temp.f141 = t59;
  t167 = prop_sigma->contents * rho->contents;
  t133 = t167 * t22 * t41 * t45;
  t14 = t133 * t60;
  Alpha = t283_tmp * t88 * t95 * t104 * t171 * 2.0 + t102 * (t259_tmp * t88 *
    t95 * t104 * t171 * 2.0 - t166 * t520 * 2.0);
  expl_temp.f140 = t14 * t73 * t105 * 3.1415926535897931 * Alpha * -0.25;
  t12 = t167 * t20 * t41 * t45;
  t7 = t12 * t59;
  t8 = t283_tmp * t86 * t95 * t104 * t170 * 2.0 + t102 * (t259_tmp * t86 * t95 *
    t104 * t170 * 2.0 - t166 * t519 * 2.0);
  expl_temp.f139 = t7 * t72 * t105 * 3.1415926535897931 * t8 * -0.25;
  t11 = t167 * t18 * t41 * t45;
  t89 = t11 * t58;
  t10 = t283_tmp * t83 * t95 * t104 * t169 * 2.0 + t102 * (t259_tmp * t83 * t95 *
    t104 * t169 * 2.0 - t166 * t518 * 2.0);
  expl_temp.f138 = t89 * t71 * t105 * 3.1415926535897931 * t10 * -0.25;
  t5 = t167 * t16 * t41 * t45;
  t167 = t5 * t57;
  t165 = t283_tmp * t80 * t95 * t104 * t168 * 2.0 + t102 * (t259_tmp * t80 * t95
    * t104 * t168 * 2.0 - t166 * t517 * 2.0);
  expl_temp.f137 = t167 * t70 * t105 * 3.1415926535897931 * t165 * -0.25;
  expl_temp.f136 = t14 * t65 * t105 * 3.1415926535897931 * Alpha * -0.25;
  expl_temp.f135 = t7 * t64 * t105 * 3.1415926535897931 * t8 * -0.25;
  expl_temp.f134 = t89 * t63 * t105 * 3.1415926535897931 * t10 * -0.25;
  expl_temp.f133 = t167 * t62 * t105 * 3.1415926535897931 * t165 * -0.25;
  expl_temp.f132 = t133 * t69 * t105 * 3.1415926535897931 * Alpha * -0.25;
  expl_temp.f131 = t58;
  expl_temp.f130 = t12 * t68 * t105 * 3.1415926535897931 * t8 * -0.25;
  expl_temp.f129 = t11 * t67 * t105 * 3.1415926535897931 * t10 * -0.25;
  expl_temp.f128 = t5 * t66 * t105 * 3.1415926535897931 * t165 * -0.25;
  t167 = b_t537_tmp_tmp * prop_theta->contents * t24;
  expl_temp.f127 = t102 * (t167 * t81 * t95 * t104 * t141 * t145 * 2.0 + t166 *
    t134 * 2.0);
  expl_temp.f126 = t102 * (t167 * t78 * t95 * t104 * t140 * t144 * 2.0 + t166 *
    t77 * 2.0);
  expl_temp.f125 = t57;
  expl_temp.f124 = t167 * t87 * t95 * t104 * t143 * t147 * 2.0 + t166 * t404 *
    2.0;
  expl_temp.f123 = t167 * t84 * t95 * t104 * t142 * t146 * 2.0 + t166 * t101 *
    2.0;
  expl_temp.f122 = I_zz->contents * q->contents * r->contents;
  t167 = I_xx->contents * p->contents;
  expl_temp.f121 = t167 * r->contents;
  expl_temp.f120 = t52;
  expl_temp.f119 = t167 * q->contents;
  expl_temp.f118 = t49;
  expl_temp.f117 = t48;
  expl_temp.f116 = (t108 + t259_tmp * t87 * t95 * t104 * t171) - t166 * (t100 +
    t412) * 2.0;
  expl_temp.f115 = (t108 + t259_tmp * t84 * t95 * t104 * t170) - t166 * (t100 +
    t411) * 2.0;
  expl_temp.f114 = (t108 + t259_tmp * t81 * t95 * t104 * t169) - t166 * (t100 +
    t410) * 2.0;
  expl_temp.f113 = (t108 + t259_tmp * t78 * t95 * t104 * t168) - t166 * (t100 +
    t409) * 2.0;
  expl_temp.f112 = t45;
  expl_temp.f111 = t46 + prop_theta->contents * (t47 + t412 * t13);
  expl_temp.f110 = t46 + prop_theta->contents * (t47 + t411 * t13);
  expl_temp.f109 = t46 + prop_theta->contents * (t47 + t410 * t13);
  expl_temp.f108 = t46 + prop_theta->contents * (t47 + t409 * t13);
  expl_temp.f107 = t44;
  expl_temp.f106 = t283_tmp_tmp * t412;
  expl_temp.f105 = t283_tmp_tmp * t411;
  expl_temp.f104 = t283_tmp_tmp * t410;
  expl_temp.f103 = t283_tmp_tmp * t409;
  expl_temp.f102 = t41;
  t167 = K_Cd->contents * S->contents * gain_theta->contents * rho->contents *
    t2 * t15 * t24 * t117;
  expl_temp.f101 = ((t207_tmp * t120 / 2.0 + t117 * t207) - t167 * t119) +
    gain_theta->contents * t287;
  expl_temp.f100 = ((t207 + t207_tmp * t117 * t120 * -0.5) + t167 * t120) +
    gain_theta->contents * t302;
  expl_temp.f99 = -(t48 * t49 * t367);
  expl_temp.f98 = t48 * t61 * t367;
  expl_temp.f97 = t61 * t368;
  expl_temp.f96 = t49 * t368;
  expl_temp.f95 = t52 * t367;
  expl_temp.f94 = t367;
  expl_temp.f93 = l_a * l_a;
  expl_temp.f92 = k_a * k_a;
  t167 = t537_tmp_tmp * prop_delta->contents * t4 * t24;
  expl_temp.f91 = t167 * t87 * t95 * t104 * t143 * t147 * 2.0;
  expl_temp.f90 = t167 * t84 * t95 * t104 * t142 * t146 * 2.0;
  expl_temp.f89 = t167 * t81 * t95 * t104 * t141 * t145 * 2.0;
  expl_temp.f88 = t167 * t78 * t95 * t104 * t140 * t144 * 2.0;
  expl_temp.f87 = j_a * j_a;
  expl_temp.f86 = i_a * i_a;
  t167 = t287_tmp_tmp * t3 * t24;
  expl_temp.f85 = -(t167 * t52 * t61 * t193 / 2.0);
  expl_temp.f84 = h_a * h_a;
  expl_temp.f83 = t112 - K_p_M->contents * t23 * t41 * (t200 + 1.0);
  expl_temp.f82 = t112 - K_p_M->contents * t21 * t41 * (t199 + 1.0);
  expl_temp.f81 = t112 - K_p_M->contents * t19 * t41 * (t198 + 1.0);
  expl_temp.f80 = t112 - K_p_M->contents * t17 * t41 * (t197 + 1.0);
  expl_temp.f79 = g_a * g_a;
  expl_temp.f78 = t167 * t49 * t52 * t193 / 2.0;
  expl_temp.f77 = f_a * f_a;
  expl_temp.f76 = t3;
  expl_temp.f75 = e_a * e_a;
  expl_temp.f74 = d_a * d_a;
  expl_temp.f73 = c_a * c_a;
  expl_temp.f72 = t283_tmp * t87 * t95 * t104 * t171;
  expl_temp.f71 = t283_tmp * t84 * t95 * t104 * t170;
  expl_temp.f70 = t283_tmp * t81 * t95 * t104 * t169;
  expl_temp.f69 = t283_tmp * t78 * t95 * t104 * t168;
  expl_temp.f68 = b_a * b_a;
  expl_temp.f67 = a * a;
  expl_temp.f66 = t167 * t48 * t193 / 2.0;
  expl_temp.f65 = t24;
  expl_temp.f64 = t23;
  expl_temp.f63 = t22;
  expl_temp.f62 = -(t287_tmp_tmp * t24 * (Cm_zero->contents - Cm_alpha->contents
    * t117) * wing_chord->contents / 2.0);
  expl_temp.f61 = V->contents * u_in[5] * t96 * -0.54813;
  expl_temp.f60 = t21;
  expl_temp.f59 = u_in[4] * t202;
  expl_temp.f58 = t200 + 1.0;
  expl_temp.f57 = t199 + 1.0;
  expl_temp.f56 = t198 + 1.0;
  expl_temp.f55 = t197 + 1.0;
  expl_temp.f54 = t202;
  expl_temp.f53 = t9 * t96 * -1.853;
  expl_temp.f52 = t20;
  expl_temp.f51 = -(u_in[7] * u_in[7] * 0.17797);
  expl_temp.f50 = u_in[0] * t189;
  expl_temp.f49 = t193;
  expl_temp.f48 = u_in[6] * u_in[6] * 0.17797;
  expl_temp.f47 = -(u_in[1] * 0.51177);
  expl_temp.f46 = t19;
  expl_temp.f45 = t189;
  expl_temp.f44 = -(u_in[4] * u_in[4] * 0.15672);
  expl_temp.f43 = -(u_in[1] * u_in[5] * 0.36558);
  expl_temp.f42 = u_in[0] * 0.51177;
  expl_temp.f41 = t18;
  expl_temp.f40 = u_in[5] * u_in[5] * 0.15672;
  expl_temp.f39 = u_in[0] * u_in[4] * 0.36558;
  expl_temp.f38 = -(u_in[0] * u_in[7] * 0.18985);
  expl_temp.f37 = -(CL_aileron->contents * S->contents * u_in[14] *
                    gain_ailerons->contents * rho->contents * t24 / 2.0);
  expl_temp.f36 = -(u_in[4] * 0.28424);
  expl_temp.f35 = -(u_in[7] * 0.24408);
  expl_temp.f34 = t17;
  expl_temp.f33 = u_in[1] * u_in[6] * 0.18985;
  expl_temp.f32 = t16;
  expl_temp.f31 = sin(t138);
  expl_temp.f30 = sin(t137);
  expl_temp.f29 = sin(t136);
  expl_temp.f28 = sin(t135);
  expl_temp.f27 = t155;
  expl_temp.f26 = t154;
  expl_temp.f25 = t153;
  expl_temp.f24 = t152;
  expl_temp.f23 = u_in[5] * 0.28424;
  expl_temp.f22 = t15;
  expl_temp.f21 = u_in[6] * 0.24408;
  expl_temp.f20 = t147;
  expl_temp.f19 = t146;
  expl_temp.f18 = t145;
  expl_temp.f17 = t144;
  expl_temp.f16 = t143;
  expl_temp.f15 = t142;
  expl_temp.f14 = t141;
  expl_temp.f13 = t140;
  expl_temp.f12 = -(t111 * 2.0);
  expl_temp.f11 = -t111;
  expl_temp.f10 = -(t110 * 2.0);
  expl_temp.f9 = -t110;
  expl_temp.f8 = t117;
  expl_temp.f7 = -(I_yy->contents * q->contents * r->contents);
  expl_temp.f6 = -(I_zz->contents * p->contents * r->contents);
  expl_temp.f5 = -(I_yy->contents * p->contents * q->contents);
  expl_temp.f4 = 1.5707963267948966;
  expl_temp.f3 = t105;
  expl_temp.f2 = t102;
  memcpy(&expl_temp.f1[0], &u_in[0], 15U * sizeof(double));
  cost = ft_1(dv_global, V, gain_motor, prop_sigma, rho, gain_el, gain_az, l_1,
              l_4, l_3, l_z, gamma_quadratic_du, desired_phi_value,
              desired_theta_value, desired_ailerons_value, K_p_M, k_tilt,
              gain_theta, K_Cd, S, Cm_alpha, wing_chord, gain_phi, CL_aileron,
              gain_ailerons, &expl_temp, b_gradient_data, gradient_size);
  if (*gradient_size - 1 >= 0) {
    memcpy(&gradient_data[0], &b_gradient_data[0], (unsigned int)*gradient_size *
           sizeof(double));
  }

  return cost;
}

static void compute_deltax(const double H[225], g_struct_T *solution, d_struct_T
  *memspace, const e_struct_T *qrmanager, f_struct_T *cholmanager, const
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
              jA = 31 * idx;
              for (ix = 0; ix <= nVar_tmp; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            cholmanager->info = xpotrf(qrmanager->mrows, cholmanager->FMat);
          } else {
            cholmanager->ndims = qrmanager->mrows;
            for (idx = 0; idx <= nVar_tmp; idx++) {
              jjA = qrmanager->mrows * idx;
              jA = 31 * idx;
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
                  s = fabs(cholmanager->FMat[(ix - 1) << 5]);
                  if (s > smax) {
                    nVars = ix - 1;
                    smax = s;
                  }
                }
              }
            }

            cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[nVars + 31 *
              nVars]) * 2.2204460492503131E-16, 0.0);
            fullColLDL2_(cholmanager, qrmanager->mrows);
            if (cholmanager->ConvexCheck) {
              idx = 0;
              int exitg1;
              do {
                exitg1 = 0;
                if (idx <= nVar_tmp) {
                  if (cholmanager->FMat[idx + 31 * idx] <= 0.0) {
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
                jjA = idx + idx * 31;
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
              solution->searchDir[idx] /= cholmanager->FMat[idx + 31 * idx];
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 31;
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
              jA = 31 * idx;
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
      nullStartIdx_tmp = 31 * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double[idx] = -qrmanager->Q[nVar_tmp + 31 *
            (qrmanager->ncols + idx)];
        }

        if (qrmanager->mrows != 0) {
          int i;
          memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) *
                 sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
          for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
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
              memspace->workspace_double[(jjA + 31 * jA) - 1] = objective->beta *
                qrmanager->Q[(jjA + 31 * (jA + qrmanager->ncols)) - 1];
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
              s = fabs(cholmanager->FMat[(ix - 1) << 5]);
              if (s > smax) {
                nVars = ix - 1;
                smax = s;
              }
            }
          }

          cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[nVars + 31 * nVars])
            * 2.2204460492503131E-16, 0.0);
          fullColLDL2_(cholmanager, mNull_tmp);
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager->FMat[idx + 31 * idx] <= 0.0) {
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
            i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
              smax = 0.0;
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                smax += qrmanager->Q[idx - 1] * objective->grad[idx - jjA];
              }

              nVars = div_nde_s32_floor(jjA - nullStartIdx_tmp);
              memspace->workspace_double[nVars] -= smax;
            }
          }

          if (alwaysPositiveDef) {
            nVars = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx < nVars; idx++) {
                jA = idx * 31;
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
                jjA = (idx + (idx - 1) * 31) - 1;
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
                jjA = idx + idx * 31;
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
              memspace->workspace_double[idx] /= cholmanager->FMat[idx + 31 *
                idx];
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 31;
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
            i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
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

static void countsort(int x[31], int xLen, int workspace[31], int xMin, int xMax)
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

static void deleteColMoveEnd(e_struct_T *obj, int idx)
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
      obj->QR[k + 31 * (idx - 1)] = obj->QR[k + 31 * b_i];
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
      i = 31 * (idx - 1);
      while (k >= idx) {
        b_i = k + i;
        temp_tmp = obj->QR[b_i];
        c = xrotg(&obj->QR[b_i - 1], &temp_tmp, &s);
        obj->QR[b_i] = temp_tmp;
        b_i = 31 * (k - 1);
        obj->QR[k + b_i] = 0.0;
        QRk0 = k + 31 * idx;
        n = obj->ncols - idx;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 31;
            temp_tmp = obj->QR[b_temp_tmp - 1];
            temp = c * temp_tmp + s * obj->QR[b_temp_tmp];
            obj->QR[b_temp_tmp] = c * obj->QR[b_temp_tmp] - s * temp_tmp;
            obj->QR[b_temp_tmp - 1] = temp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = b_i + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 31];
          temp = c * obj->Q[b_temp_tmp] + s * temp_tmp;
          obj->Q[b_temp_tmp + 31] = c * temp_tmp - s * obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp] = temp;
        }

        k--;
      }

      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        u0 = 31 * (k - 1);
        i = k + u0;
        temp_tmp = obj->QR[i];
        c = xrotg(&obj->QR[i - 1], &temp_tmp, &s);
        obj->QR[i] = temp_tmp;
        QRk0 = k << 5;
        n = obj->ncols - k;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 31;
            temp_tmp = obj->QR[b_temp_tmp - 1];
            temp = c * temp_tmp + s * obj->QR[b_temp_tmp];
            obj->QR[b_temp_tmp] = c * obj->QR[b_temp_tmp] - s * temp_tmp;
            obj->QR[b_temp_tmp - 1] = temp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = u0 + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 31];
          temp = c * obj->Q[b_temp_tmp] + s * temp_tmp;
          obj->Q[b_temp_tmp + 31] = c * temp_tmp - s * obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp] = temp;
        }
      }
    }
  }
}

static int div_nde_s32_floor(int numerator)
{
  int i;
  if ((numerator < 0) && (numerator % 31 != 0)) {
    i = -1;
  } else {
    i = 0;
  }

  return numerator / 31 + i;
}

static void driver(const double H[225], const double f[16], g_struct_T *solution,
                   d_struct_T *memspace, h_struct_T *workingset, e_struct_T
                   *qrmanager, f_struct_T *cholmanager, struct_T *objective,
                   i_struct_T *options, int runTimeOptions_MaxIterations)
{
  int idxStartIneq;
  int idx_global;
  int mConstr;
  int nVar_tmp;
  bool guard1 = false;
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
      solution->xstar[15] = solution->maxConstr + 1.0;
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
      objective->nvar = 16;
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

      solution->maxConstr = solution->xstar[15];
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
          memset(&solution->lambda[0], 0, 31U * sizeof(double));
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

static double evalObjAndConstr(const c_struct_T *c_obj_next_next_next_next_next_,
  const double x[15], int *status)
{
  double fval;
  bool b;
  fval = b_compute_cost_and_gradient_fcn
    (c_obj_next_next_next_next_next_->dv_global,
     c_obj_next_next_next_next_next_->V,
     c_obj_next_next_next_next_next_->gain_motor,
     c_obj_next_next_next_next_next_->prop_sigma,
     c_obj_next_next_next_next_next_->rho,
     c_obj_next_next_next_next_next_->gain_el,
     c_obj_next_next_next_next_next_->gain_az,
     c_obj_next_next_next_next_next_->l_1, c_obj_next_next_next_next_next_->l_4,
     c_obj_next_next_next_next_next_->l_3, c_obj_next_next_next_next_next_->l_z,
     c_obj_next_next_next_next_next_->gamma_quadratic_du,
     c_obj_next_next_next_next_next_->desired_phi_value,
     c_obj_next_next_next_next_next_->desired_theta_value,
     c_obj_next_next_next_next_next_->desired_ailerons_value,
     c_obj_next_next_next_next_next_->K_p_M,
     c_obj_next_next_next_next_next_->k_tilt,
     c_obj_next_next_next_next_next_->gain_theta,
     c_obj_next_next_next_next_next_->K_Cd, c_obj_next_next_next_next_next_->S,
     c_obj_next_next_next_next_next_->Cm_alpha,
     c_obj_next_next_next_next_next_->wing_chord,
     c_obj_next_next_next_next_next_->gain_phi,
     c_obj_next_next_next_next_next_->CL_aileron,
     c_obj_next_next_next_next_next_->gain_ailerons,
     c_obj_next_next_next_next_next_->flight_path_angle,
     c_obj_next_next_next_next_next_->Beta,
     c_obj_next_next_next_next_next_->prop_delta,
     c_obj_next_next_next_next_next_->Cl_alpha,
     c_obj_next_next_next_next_next_->W_act_phi,
     c_obj_next_next_next_next_next_->W_act_theta,
     c_obj_next_next_next_next_next_->W_act_motor,
     c_obj_next_next_next_next_next_->W_dv_1,
     c_obj_next_next_next_next_next_->W_dv_2,
     c_obj_next_next_next_next_next_->W_dv_3,
     c_obj_next_next_next_next_next_->W_dv_4,
     c_obj_next_next_next_next_next_->W_dv_5,
     c_obj_next_next_next_next_next_->W_dv_6,
     c_obj_next_next_next_next_next_->W_act_tilt_el,
     c_obj_next_next_next_next_next_->W_act_tilt_az,
     c_obj_next_next_next_next_next_->W_act_ailerons,
     c_obj_next_next_next_next_next_->prop_Cd_a,
     c_obj_next_next_next_next_next_->prop_R,
     c_obj_next_next_next_next_next_->prop_Cd_0,
     c_obj_next_next_next_next_next_->I_xx, c_obj_next_next_next_next_next_->p,
     c_obj_next_next_next_next_next_->q, c_obj_next_next_next_next_next_->I_yy,
     c_obj_next_next_next_next_next_->r, c_obj_next_next_next_next_next_->I_zz,
     c_obj_next_next_next_next_next_->gain_airspeed,
     c_obj_next_next_next_next_next_->m,
     c_obj_next_next_next_next_next_->prop_Cl_a,
     c_obj_next_next_next_next_next_->prop_theta,
     c_obj_next_next_next_next_next_->prop_Cl_0,
     c_obj_next_next_next_next_next_->desired_el_value,
     c_obj_next_next_next_next_next_->desired_az_value,
     c_obj_next_next_next_next_next_->desired_motor_value,
     c_obj_next_next_next_next_next_->Cm_zero,
     c_obj_next_next_next_next_next_->Cd_zero, x);
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

static void factorQR(e_struct_T *obj, const double A[496], int mrows, int ncols)
{
  int i;
  int idx;
  int k;
  bool guard1 = false;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    for (idx = 0; idx < ncols; idx++) {
      int ix0;
      int iy0;
      ix0 = idx << 4;
      iy0 = 31 * idx;
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
    memset(&obj->tau[0], 0, 31U * sizeof(double));
    if (i >= 1) {
      qrf(obj->QR, mrows, ncols, i, obj->tau);
    }
  }
}

static bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
  const h_struct_T *workingset, e_struct_T *qrmanager)
{
  double B[496];
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
      workspace[iAcol + 31] = c;
    }

    if (mWConstr != 0) {
      i = ((mWConstr - 1) << 4) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 16) {
        c = 0.0;
        i1 = (iAcol + nVar) - 1;
        for (br = iAcol; br <= i1; br++) {
          c += workingset->ATwset[br - 1] * xCurrent[br - iAcol];
        }

        i1 = (iAcol - 1) >> 4;
        workspace[i1] -= c;
      }
    }

    if (mWConstr >= nVar) {
      i = (unsigned char)nVar;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (br = 0; br < i; br++) {
        iAcol = 31 * br;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] = workingset->ATwset[br + (jBcol << 4)];
        }

        qrmanager->jpvt[br] = br + 1;
      }

      if (mWConstr <= nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }

      qrmanager->minRowCol = i;
      memset(&qrmanager->tau[0], 0, 31U * sizeof(double));
      if (i >= 1) {
        qrf(qrmanager->QR, mWConstr, nVar, i, qrmanager->tau);
      }

      computeQ_(qrmanager, mWConstr);
      memcpy(&B[0], &workspace[0], 496U * sizeof(double));
      for (b_i = 0; b_i <= 31; b_i += 31) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof
                 (double));
        }
      }

      br = -1;
      for (b_i = 0; b_i <= 31; b_i += 31) {
        jBcol = -1;
        i = b_i + 1;
        i1 = b_i + nVar;
        for (k = i; k <= i1; k++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager->Q[(iAcol + jBcol) + 1] * B[(iAcol + br) + 1];
          }

          workspace[k - 1] += c;
          jBcol += 31;
        }

        br += 31;
      }

      for (j = 0; j < 2; j++) {
        jBcol = 31 * j - 1;
        for (k = nVar; k >= 1; k--) {
          iAcol = 31 * (k - 1) - 1;
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
        jBcol = 31 * j;
        for (b_i = 0; b_i < mWConstr; b_i++) {
          iAcol = 31 * b_i;
          br = b_i + jBcol;
          c = workspace[br];
          i = (unsigned char)b_i;
          for (k = 0; k < i; k++) {
            c -= qrmanager->QR[k + iAcol] * workspace[k + jBcol];
          }

          workspace[br] = c / qrmanager->QR[b_i + iAcol];
        }
      }

      memcpy(&B[0], &workspace[0], 496U * sizeof(double));
      for (b_i = 0; b_i <= 31; b_i += 31) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof
                 (double));
        }
      }

      br = 0;
      for (b_i = 0; b_i <= 31; b_i += 31) {
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

          jBcol += 31;
        }

        br += 31;
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
          c = workspace[iAcol + 31];
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
          32);
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          i = (unsigned char)nVar;
          memcpy(&xCurrent[0], &workspace[0], (unsigned int)i * sizeof(double));
        } else {
          i = (unsigned char)nVar;
          memcpy(&xCurrent[0], &workspace[31], (unsigned int)i * sizeof(double));
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

static double feasibleratiotest(const double solution_xstar[16], const double
  solution_searchDir[16], int workingset_nVar, const double workingset_lb[16],
  const double workingset_ub[16], const int workingset_indexLB[16], const int
  workingset_indexUB[16], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[31], const int
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

static double fmincon(c_struct_T *fun_workspace, const double x0[15], const
                      double lb[15], const double ub[15], double x[15], double
                      *exitflag, double *output_iterations, double
                      *output_funcCount, char output_algorithm[3], double
                      *output_constrviolation, double *output_stepsize, double
                      *output_lssteplength, double *output_firstorderopt)
{
  b_struct_T MeritFunction;
  d_struct_T memspace;
  e_struct_T QRManager;
  f_struct_T CholManager;
  g_struct_T TrialState;
  h_struct_T WorkingSet;
  i_coder_internal_stickyStruct r;
  struct_T QPObjective;
  double fval;
  double scale;
  double y;
  int b_i;
  int colOffsetATw;
  int i;
  int mFixed;
  int mLB;
  int mUB;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  output_algorithm[0] = 's';
  output_algorithm[1] = 'q';
  output_algorithm[2] = 'p';
  TrialState.nVarMax = 16;
  TrialState.mNonlinIneq = 0;
  TrialState.mNonlinEq = 0;
  TrialState.mIneq = 0;
  TrialState.mEq = 0;
  TrialState.iNonIneq0 = 1;
  TrialState.iNonEq0 = 1;
  TrialState.sqpFval_old = 0.0;
  TrialState.sqpIterations = 0;
  TrialState.sqpExitFlag = 0;
  memset(&TrialState.lambdasqp[0], 0, 31U * sizeof(double));
  TrialState.steplength = 1.0;
  memset(&TrialState.delta_x[0], 0, 16U * sizeof(double));
  TrialState.fstar = 0.0;
  TrialState.firstorderopt = 0.0;
  memset(&TrialState.lambda[0], 0, 31U * sizeof(double));
  TrialState.state = 0;
  TrialState.maxConstr = 0.0;
  TrialState.iterations = 0;
  memcpy(&TrialState.xstarsqp[0], &x0[0], 15U * sizeof(double));
  WorkingSet.nVar = 15;
  WorkingSet.nVarOrig = 15;
  WorkingSet.nVarMax = 16;
  WorkingSet.ldA = 16;
  memset(&WorkingSet.lb[0], 0, 16U * sizeof(double));
  memset(&WorkingSet.ub[0], 0, 16U * sizeof(double));
  WorkingSet.mEqRemoved = 0;
  memset(&WorkingSet.ATwset[0], 0, 496U * sizeof(double));
  WorkingSet.nActiveConstr = 0;
  memset(&WorkingSet.bwset[0], 0, 31U * sizeof(double));
  memset(&WorkingSet.maxConstrWorkspace[0], 0, 31U * sizeof(double));
  memset(&WorkingSet.Wid[0], 0, 31U * sizeof(int));
  memset(&WorkingSet.Wlocalidx[0], 0, 31U * sizeof(int));
  for (i = 0; i < 31; i++) {
    WorkingSet.isActiveConstr[i] = false;
  }

  for (i = 0; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }

  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  memset(&WorkingSet.indexLB[0], 0, 16U * sizeof(int));
  memset(&WorkingSet.indexUB[0], 0, 16U * sizeof(int));
  memset(&WorkingSet.indexFixed[0], 0, 16U * sizeof(int));
  mLB = 0;
  mUB = 0;
  mFixed = 0;
  for (colOffsetATw = 0; colOffsetATw < 15; colOffsetATw++) {
    bool guard1 = false;
    y = lb[colOffsetATw];
    guard1 = false;
    if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
      if (fabs(y - ub[colOffsetATw]) < 1.0E-6) {
        mFixed++;
        WorkingSet.indexFixed[mFixed - 1] = colOffsetATw + 1;
      } else {
        mLB++;
        WorkingSet.indexLB[mLB - 1] = colOffsetATw + 1;
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      y = ub[colOffsetATw];
      if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
        mUB++;
        WorkingSet.indexUB[mUB - 1] = colOffsetATw + 1;
      }
    }
  }

  i = (mLB + mUB) + mFixed;
  WorkingSet.mConstr = i;
  WorkingSet.mConstrOrig = i;
  WorkingSet.mConstrMax = 31;
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
    WorkingSet.isActiveIdxRegPhaseOne[b_i] = WorkingSet.isActiveIdxPhaseOne[b_i];
  }

  for (colOffsetATw = 0; colOffsetATw < mLB; colOffsetATw++) {
    b_i = WorkingSet.indexLB[colOffsetATw];
    TrialState.xstarsqp[b_i - 1] = fmax(TrialState.xstarsqp[b_i - 1], lb[b_i - 1]);
  }

  for (colOffsetATw = 0; colOffsetATw < mUB; colOffsetATw++) {
    b_i = WorkingSet.indexUB[colOffsetATw];
    TrialState.xstarsqp[b_i - 1] = fmin(TrialState.xstarsqp[b_i - 1], ub[b_i - 1]);
  }

  for (colOffsetATw = 0; colOffsetATw < mFixed; colOffsetATw++) {
    b_i = WorkingSet.indexFixed[colOffsetATw];
    TrialState.xstarsqp[b_i - 1] = ub[b_i - 1];
  }

  TrialState.sqpFval = c_computeObjectiveAndUserGradie(fun_workspace,
    TrialState.xstarsqp, TrialState.grad, &i);
  TrialState.FunctionEvaluations = 1;
  for (colOffsetATw = 0; colOffsetATw < mLB; colOffsetATw++) {
    WorkingSet.lb[WorkingSet.indexLB[colOffsetATw] - 1] =
      -lb[WorkingSet.indexLB[colOffsetATw] - 1] +
      x0[WorkingSet.indexLB[colOffsetATw] - 1];
  }

  for (colOffsetATw = 0; colOffsetATw < mUB; colOffsetATw++) {
    WorkingSet.ub[WorkingSet.indexUB[colOffsetATw] - 1] =
      ub[WorkingSet.indexUB[colOffsetATw] - 1] -
      x0[WorkingSet.indexUB[colOffsetATw] - 1];
  }

  for (colOffsetATw = 0; colOffsetATw < mFixed; colOffsetATw++) {
    y = ub[WorkingSet.indexFixed[colOffsetATw] - 1] -
      x0[WorkingSet.indexFixed[colOffsetATw] - 1];
    WorkingSet.ub[WorkingSet.indexFixed[colOffsetATw] - 1] = y;
    WorkingSet.bwset[colOffsetATw] = y;
  }

  setProblemType(&WorkingSet, 3);
  i = WorkingSet.isActiveIdx[2];
  for (colOffsetATw = i; colOffsetATw < 32; colOffsetATw++) {
    WorkingSet.isActiveConstr[colOffsetATw - 1] = false;
  }

  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
  b_i = (unsigned char)WorkingSet.sizes[0];
  for (i = 0; i < b_i; i++) {
    WorkingSet.Wid[i] = 1;
    WorkingSet.Wlocalidx[i] = i + 1;
    WorkingSet.isActiveConstr[i] = true;
    colOffsetATw = i << 4;
    mLB = WorkingSet.indexFixed[i];
    if (mLB - 2 >= 0) {
      memset(&WorkingSet.ATwset[colOffsetATw], 0, (unsigned int)(((mLB +
                colOffsetATw) - colOffsetATw) - 1) * sizeof(double));
    }

    WorkingSet.ATwset[(WorkingSet.indexFixed[i] + colOffsetATw) - 1] = 1.0;
    mLB = WorkingSet.indexFixed[i] + 1;
    mUB = WorkingSet.nVar;
    if (mLB <= mUB) {
      memset(&WorkingSet.ATwset[(mLB + colOffsetATw) + -1], 0, (unsigned int)
             ((((mUB + colOffsetATw) - mLB) - colOffsetATw) + 1) * sizeof(double));
    }

    WorkingSet.bwset[i] = WorkingSet.ub[WorkingSet.indexFixed[i] - 1];
  }

  double Hessian[225];
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
  r.next.next.next.next.next.next.next.next.value.workspace = *fun_workspace;
  b_driver(lb, ub, &TrialState, &MeritFunction, &r, &memspace, &WorkingSet,
           Hessian, &QRManager, &CholManager, &QPObjective);
  fval = TrialState.sqpFval;
  *exitflag = TrialState.sqpExitFlag;
  *output_iterations = TrialState.sqpIterations;
  *output_funcCount = TrialState.FunctionEvaluations;
  *output_constrviolation = MeritFunction.nlpPrimalFeasError;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (i = 0; i < 15; i++) {
    double absxk;
    x[i] = TrialState.xstarsqp[i];
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
  return fval;
}

static double ft_1(const b_captured_var *dv_global, const captured_var *V, const
                   captured_var *gain_motor, const captured_var *prop_sigma,
                   const captured_var *rho, const captured_var *gain_el, const
                   captured_var *gain_az, const captured_var *l_1, const
                   captured_var *l_4, const captured_var *l_3, const
                   captured_var *l_z, const captured_var *gamma_quadratic_du,
                   const captured_var *desired_phi_value, const captured_var
                   *desired_theta_value, const captured_var
                   *desired_ailerons_value, const captured_var *K_p_M, const
                   captured_var *k_tilt, const captured_var *gain_theta, const
                   captured_var *K_Cd, const captured_var *S, const captured_var
                   *Cm_alpha, const captured_var *wing_chord, const captured_var
                   *gain_phi, const captured_var *CL_aileron, const captured_var
                   *gain_ailerons, const cell_18 *ct, double gradient_data[],
                   int *gradient_size)
{
  double cost;
  double t461;
  double t462;
  double t463;
  double t464;
  double t473;
  double t474;
  double t474_tmp;
  double t475;
  double t476;
  double t489;
  double t490;
  double t491;
  double t491_tmp;
  double t492;
  double t492_tmp;
  double t493;
  double t493_tmp;
  double t494;
  double t494_tmp;
  double t495;
  double t495_tmp;
  double t496;
  double t496_tmp;
  double t589;
  double t590;
  double t591;
  double t592;
  double t593;
  double t594;
  double t595;
  double t596;
  double t601;
  double t602;
  double t603;
  double t604;
  double t605;
  double t606;
  double t607;
  double t608;
  double t685;
  double t685_tmp;
  double t686;
  double t687;
  double t688;
  double t688_tmp;
  double t689;
  double t689_tmp;
  double t690;
  double t690_tmp;
  double t691;
  double t691_tmp;
  double t692;
  double t692_tmp;
  double t753;
  double t754;
  double t755;
  double t756;
  double t767;
  double t768;
  double t769;
  double t770;
  double t771;
  double t772;
  double t773;
  double t774;
  double t783;
  double t784;
  double t785;
  double t786;
  double t795;
  double t796;
  double t797;
  double t798;
  double t805;
  double t805_tmp;
  double t806;
  double t807;
  double t808;
  double t809;
  double t810;
  double t811;
  double t812;
  double t813;
  double t814;
  double t815;
  double t816;
  double t817;
  double t817_tmp;
  double t818;
  double t819;
  double t820;
  double t821;
  double t822;
  double t823;
  double t824;
  double t826;
  double t827;
  double t828;
  double t830;
  double t831;
  double t832;
  double t833;
  double t834;
  double t835;
  double t837;
  double t838;
  double t848;
  double t851;
  double t854;
  double t861;
  t593 = ct->f88 + ct->f126;
  t594 = ct->f89 + ct->f127;
  t595 = ct->f90 + ct->f2 * ct->f123;
  t596 = ct->f91 + ct->f2 * ct->f124;
  t461 = ct->f69 + ct->f2 * ct->f113;
  t462 = ct->f70 + ct->f2 * ct->f114;
  t463 = ct->f71 + ct->f2 * ct->f115;
  t464 = ct->f72 + ct->f2 * ct->f116;
  t589 = ct->f103 + ct->f2 * ct->f108;
  t590 = ct->f104 + ct->f2 * ct->f109;
  t591 = ct->f105 + ct->f2 * ct->f110;
  t592 = ct->f106 + ct->f2 * ct->f111;
  t604 = prop_sigma->contents * rho->contents;
  t601 = t604 * ct->f32 * ct->f102 * ct->f112;
  t835 = t601 * ct->f148 * ct->f3;
  t473 = t835 * t461 * 3.1415926535897931 / 4.0;
  t602 = t604 * ct->f41 * ct->f102 * ct->f112;
  t474_tmp = t602 * ct->f149 * ct->f3;
  t474 = t474_tmp * t462 * 3.1415926535897931 / 4.0;
  t603 = t604 * ct->f52 * ct->f102 * ct->f112;
  t838 = t603 * ct->f150 * ct->f3;
  t475 = t838 * t463 * 3.1415926535897931 / 4.0;
  t604 = t604 * ct->f63 * ct->f102 * ct->f112;
  t827 = t604 * ct->f151 * ct->f3;
  t476 = t827 * t464 * 3.1415926535897931 / 4.0;
  t601 *= ct->f125;
  t823 = t601 * ct->f144 * ct->f3;
  t489 = t823 * t461 * 3.1415926535897931 / 4.0;
  t602 *= ct->f131;
  t822 = t602 * ct->f145 * ct->f3;
  t490 = t822 * t462 * 3.1415926535897931 / 4.0;
  t603 *= ct->f141;
  t491_tmp = t603 * ct->f146 * ct->f3;
  t491 = t491_tmp * t463 * 3.1415926535897931 / 4.0;
  t604 *= ct->f142;
  t492_tmp = t604 * ct->f147 * ct->f3;
  t492 = t492_tmp * t464 * 3.1415926535897931 / 4.0;
  t493_tmp = t601 * ct->f152 * ct->f3;
  t493 = t493_tmp * t461 * 3.1415926535897931 / 4.0;
  t494_tmp = t602 * ct->f157 * ct->f3;
  t494 = t494_tmp * t462 * 3.1415926535897931 / 4.0;
  t495_tmp = t603 * ct->f161 * ct->f3;
  t495 = t495_tmp * t463 * 3.1415926535897931 / 4.0;
  t496_tmp = t604 * ct->f171 * ct->f3;
  t496 = t496_tmp * t464 * 3.1415926535897931 / 4.0;
  t861 = V->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * ct->f107;
  t601 = t861 * ct->f125 * ct->f3 * ct->f17 * t589 * 3.1415926535897931 / 4.0;
  t602 = t861 * ct->f131 * ct->f3 * ct->f18 * t590 * 3.1415926535897931 / 4.0;
  t603 = t861 * ct->f141 * ct->f3 * ct->f19 * t591 * 3.1415926535897931 / 4.0;
  t604 = t861 * ct->f142 * ct->f3 * ct->f20 * t592 * 3.1415926535897931 / 4.0;
  t851 = ct->f1[0] * V->contents;
  t685_tmp = t851 * gain_motor->contents * prop_sigma->contents * rho->contents *
    ct->f107;
  t837 = t685_tmp * ct->f144 * ct->f148 * ct->f3 * ct->f17;
  t685 = t837 * t589 * 3.1415926535897931 * -0.25;
  t854 = ct->f1[1] * V->contents;
  t831 = t854 * gain_motor->contents * prop_sigma->contents * rho->contents *
    ct->f107;
  t830 = t831 * ct->f145 * ct->f149 * ct->f3 * ct->f18;
  t686 = t830 * t590 * 3.1415926535897931 * -0.25;
  t848 = ct->f1[2] * V->contents;
  t828 = t848 * gain_motor->contents * prop_sigma->contents * rho->contents *
    ct->f107;
  t826 = t828 * ct->f146 * ct->f150 * ct->f3 * ct->f19;
  t687 = t826 * t591 * 3.1415926535897931 * -0.25;
  t834 = ct->f1[3] * V->contents;
  t688_tmp = t834 * gain_motor->contents * prop_sigma->contents * rho->contents *
    ct->f107;
  t824 = t688_tmp * ct->f147 * ct->f151 * ct->f3 * ct->f20;
  t688 = t824 * t592 * 3.1415926535897931 * -0.25;
  t689_tmp = t685_tmp * ct->f148 * ct->f152 * ct->f3 * ct->f17;
  t689 = t689_tmp * t589 * 3.1415926535897931 * -0.25;
  t690_tmp = t831 * ct->f149 * ct->f157 * ct->f3 * ct->f18;
  t690 = t690_tmp * t590 * 3.1415926535897931 * -0.25;
  t691_tmp = t828 * ct->f150 * ct->f161 * ct->f3 * ct->f19;
  t691 = t691_tmp * t591 * 3.1415926535897931 * -0.25;
  t692_tmp = t688_tmp * ct->f151 * ct->f171 * ct->f3 * ct->f20;
  t692 = t692_tmp * t592 * 3.1415926535897931 * -0.25;
  t605 = ct->f1[0] * t601;
  t606 = ct->f1[1] * t602;
  t607 = ct->f1[2] * t603;
  t608 = ct->f1[3] * t604;
  t767 = t493 + t689;
  t768 = t494 + t690;
  t769 = t495 + t691;
  t770 = t496 + t692;
  t771 = t489 + t685;
  t772 = t490 + t686;
  t773 = t491 + t687;
  t774 = t492 + t688;
  t805_tmp = ct->f1[0] * prop_sigma->contents * rho->contents * ct->f102 *
    ct->f112;
  t805 = ((t805_tmp * ct->f148 * ct->f3 * 1.5707963267948966 * t461 + ct->f128)
          + t601) + ct->f153;
  t601 = ct->f1[1] * prop_sigma->contents * rho->contents * ct->f102 * ct->f112;
  t806 = ((t601 * ct->f149 * ct->f3 * 1.5707963267948966 * t462 + ct->f129) +
          t602) + ct->f154;
  t602 = ct->f1[2] * prop_sigma->contents * rho->contents * ct->f102 * ct->f112;
  t807 = ((t602 * ct->f150 * ct->f3 * 1.5707963267948966 * t463 + ct->f130) +
          t603) + ct->f155;
  t603 = ct->f1[3] * prop_sigma->contents * rho->contents * ct->f102 * ct->f112;
  t808 = ((t603 * ct->f151 * ct->f3 * 1.5707963267948966 * t464 + ct->f132) +
          t604) + ct->f156;
  t753 = t473 + t605;
  t754 = t474 + t606;
  t755 = t475 + t607;
  t756 = t476 + t608;
  t783 = gain_az->contents * t489 + gain_az->contents * t685;
  t784 = gain_az->contents * t490 + gain_az->contents * t686;
  t785 = gain_az->contents * t491 + gain_az->contents * t687;
  t786 = gain_az->contents * t492 + gain_az->contents * t688;
  t795 = gain_az->contents * t493 + gain_az->contents * t689;
  t796 = gain_az->contents * t494 + gain_az->contents * t690;
  t797 = gain_az->contents * t495 + gain_az->contents * t691;
  t798 = gain_az->contents * t496 + gain_az->contents * t692;
  t495 = t805_tmp * ct->f125;
  t809 = ((t495 * ct->f144 * ct->f3 * 1.5707963267948966 * t461 + ct->f133) -
          t861 * ct->f144 * ct->f148 * ct->f3 * ct->f17 * t589 *
          3.1415926535897931 / 4.0) + ct->f158;
  t496 = t601 * ct->f131;
  t810 = ((t496 * ct->f145 * ct->f3 * 1.5707963267948966 * t462 + ct->f134) -
          t861 * ct->f145 * ct->f149 * ct->f3 * ct->f18 * t590 *
          3.1415926535897931 / 4.0) + ct->f159;
  t692 = t602 * ct->f141;
  t811 = ((t692 * ct->f146 * ct->f3 * 1.5707963267948966 * t463 + ct->f135) -
          t861 * ct->f146 * ct->f150 * ct->f3 * ct->f19 * t591 *
          3.1415926535897931 / 4.0) + ct->f160;
  t805_tmp = t603 * ct->f142;
  t812 = ((t805_tmp * ct->f147 * ct->f3 * 1.5707963267948966 * t464 + ct->f136)
          - t861 * ct->f147 * ct->f151 * ct->f3 * ct->f20 * t592 *
          3.1415926535897931 / 4.0) + ct->f162;
  t813 = ((t495 * ct->f152 * ct->f3 * 1.5707963267948966 * t461 + ct->f137) -
          t861 * ct->f148 * ct->f152 * ct->f3 * ct->f17 * t589 *
          3.1415926535897931 / 4.0) + ct->f163;
  t814 = ((t496 * ct->f157 * ct->f3 * 1.5707963267948966 * t462 + ct->f138) -
          t861 * ct->f149 * ct->f157 * ct->f3 * ct->f18 * t590 *
          3.1415926535897931 / 4.0) + ct->f164;
  t815 = ((t692 * ct->f161 * ct->f3 * 1.5707963267948966 * t463 + ct->f139) -
          t861 * ct->f150 * ct->f161 * ct->f3 * ct->f19 * t591 *
          3.1415926535897931 / 4.0) + ct->f165;
  t816 = ((t805_tmp * ct->f171 * ct->f3 * 1.5707963267948966 * t464 + ct->f140)
          - t861 * ct->f151 * ct->f171 * ct->f3 * ct->f20 * t592 *
          3.1415926535897931 / 4.0) + ct->f166;
  t817_tmp = t851 * gain_el->contents * gain_motor->contents *
    prop_sigma->contents * rho->contents * ct->f107;
  t601 = gain_el->contents * prop_sigma->contents * rho->contents;
  t851 = t817_tmp * ct->f148;
  t817 = (((t601 * ct->f32 * ct->f102 * ct->f112 * ct->f125 * ct->f3 * t461 *
            3.1415926535897931 / 4.0 + t835 * t593 * 3.1415926535897931 / 4.0) +
           t817_tmp * ct->f125 * ct->f3 * ct->f13 * t589 * 3.1415926535897931 /
           4.0) - t851 * ct->f3 * ct->f17 * t589 * 3.1415926535897931 / 4.0) -
    t685_tmp * ct->f125 * ct->f3 * ct->f17 * ct->f167 * 3.1415926535897931 / 4.0;
  t461 = t854 * gain_el->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * ct->f107;
  t861 = t461 * ct->f149;
  t818 = (((t601 * ct->f41 * ct->f102 * ct->f112 * ct->f131 * ct->f3 * t462 *
            3.1415926535897931 / 4.0 + t474_tmp * t594 * 3.1415926535897931 /
            4.0) + t461 * ct->f131 * ct->f3 * ct->f14 * t590 *
           3.1415926535897931 / 4.0) - t861 * ct->f3 * ct->f18 * t590 *
          3.1415926535897931 / 4.0) - t831 * ct->f131 * ct->f3 * ct->f18 *
    ct->f168 * 3.1415926535897931 / 4.0;
  t688 = t848 * gain_el->contents * gain_motor->contents * prop_sigma->contents *
    rho->contents * ct->f107;
  t604 = t688 * ct->f150;
  t819 = (((t601 * ct->f52 * ct->f102 * ct->f112 * ct->f141 * ct->f3 * t463 *
            3.1415926535897931 / 4.0 + t838 * t595 * 3.1415926535897931 / 4.0) +
           t688 * ct->f141 * ct->f3 * ct->f15 * t591 * 3.1415926535897931 / 4.0)
          - t604 * ct->f3 * ct->f19 * t591 * 3.1415926535897931 / 4.0) - t828 *
    ct->f141 * ct->f3 * ct->f19 * ct->f169 * 3.1415926535897931 / 4.0;
  t685_tmp = t834 * gain_el->contents * gain_motor->contents *
    prop_sigma->contents * rho->contents * ct->f107;
  t603 = t685_tmp * ct->f151;
  t820 = (((t601 * ct->f63 * ct->f102 * ct->f112 * ct->f142 * ct->f3 * t464 *
            3.1415926535897931 / 4.0 + t827 * t596 * 3.1415926535897931 / 4.0) +
           t685_tmp * ct->f142 * ct->f3 * ct->f16 * t592 * 3.1415926535897931 /
           4.0) - t603 * ct->f3 * ct->f20 * t592 * 3.1415926535897931 / 4.0) -
    t688_tmp * ct->f142 * ct->f3 * ct->f20 * ct->f170 * 3.1415926535897931 / 4.0;
  t832 = ((t767 + t768) + t769) + t770;
  t833 = ((t771 + t772) + t773) + t774;
  t602 = gain_el->contents * ct->f144;
  t821 = (((t602 * t473 - t823 * t593 * 3.1415926535897931 / 4.0) + t602 * t605)
          + t817_tmp * ct->f144 * ct->f148 * ct->f3 * ct->f13 * t589 *
          3.1415926535897931 / 4.0) - t837 * ct->f167 * 3.1415926535897931 / 4.0;
  t601 = gain_el->contents * ct->f145;
  t822 = (((t601 * t474 - t822 * t594 * 3.1415926535897931 / 4.0) + t601 * t606)
          + t461 * ct->f145 * ct->f149 * ct->f3 * ct->f14 * t590 *
          3.1415926535897931 / 4.0) - t830 * ct->f168 * 3.1415926535897931 / 4.0;
  t601 = gain_el->contents * ct->f146;
  t823 = (((t601 * t475 - t491_tmp * t595 * 3.1415926535897931 / 4.0) + t601 *
           t607) + t688 * ct->f146 * ct->f150 * ct->f3 * ct->f15 * t591 *
          3.1415926535897931 / 4.0) - t826 * ct->f169 * 3.1415926535897931 / 4.0;
  t601 = gain_el->contents * ct->f147;
  t824 = (((t601 * t476 - t492_tmp * t596 * 3.1415926535897931 / 4.0) + t601 *
           t608) + t685_tmp * ct->f147 * ct->f151 * ct->f3 * ct->f16 * t592 *
          3.1415926535897931 / 4.0) - t824 * ct->f170 * 3.1415926535897931 / 4.0;
  t601 = gain_el->contents * ct->f152;
  t464 = (((t601 * t473 - t493_tmp * t593 * 3.1415926535897931 / 4.0) + t601 *
           t605) + t851 * ct->f152 * ct->f3 * ct->f13 * t589 *
          3.1415926535897931 / 4.0) - t689_tmp * ct->f167 * 3.1415926535897931 /
    4.0;
  t601 = gain_el->contents * ct->f157;
  t826 = (((t601 * t474 - t494_tmp * t594 * 3.1415926535897931 / 4.0) + t601 *
           t606) + t861 * ct->f157 * ct->f3 * ct->f14 * t590 *
          3.1415926535897931 / 4.0) - t690_tmp * ct->f168 * 3.1415926535897931 /
    4.0;
  t601 = gain_el->contents * ct->f161;
  t827 = (((t601 * t475 - t495_tmp * t595 * 3.1415926535897931 / 4.0) + t601 *
           t607) + t604 * ct->f161 * ct->f3 * ct->f15 * t591 *
          3.1415926535897931 / 4.0) - t691_tmp * ct->f169 * 3.1415926535897931 /
    4.0;
  t604 = gain_el->contents * ct->f171;
  t828 = (((t604 * t476 - t496_tmp * t596 * 3.1415926535897931 / 4.0) + t604 *
           t608) + t603 * ct->f171 * ct->f3 * ct->f16 * t592 *
          3.1415926535897931 / 4.0) - t692_tmp * ct->f170 * 3.1415926535897931 /
    4.0;
  t604 = ((t753 + t754) + t755) + t756;
  t834 = ct->f117 * t832;
  t835 = ct->f120 * t833;
  t688_tmp = ct->f117 * ct->f118;
  t462 = t688_tmp * t833;
  t474_tmp = ct->f117 * ct->f143;
  t837 = t474_tmp * t833;
  t463 = ct->f118 * ct->f120;
  t838 = t463 * t832;
  t830 = ct->f118 * t604;
  t831 = ct->f143 * t604;
  t848 = dv_global->contents[4] + ct->f173 * ((((((((((ct->f121 + ct->f6) +
    ct->f62) + l_z->contents * t753) + l_z->contents * t754) + l_z->contents *
    t755) + l_z->contents * t756) + l_4->contents * t771) + l_4->contents * t772)
    - l_3->contents * t773) - l_3->contents * t774);
  t851 = dv_global->contents[1] + ct->f178 * (((ct->f66 + ct->f95) + t834) +
    t835);
  t854 = dv_global->contents[3] + ct->f172 *
    ((((((((((((((((((((((((((((ct->f122 + ct->f7) + ct->f21) + ct->f23) +
    ct->f33) + ct->f35) + ct->f36) + ct->f37) + ct->f38) + ct->f39) + ct->f40) +
                      ct->f42) + ct->f43) + ct->f44) + ct->f47) + ct->f48) +
                 ct->f50) + ct->f51) + ct->f53) + ct->f59) + ct->f61) +
            l_1->contents * t771) + l_1->contents * t774) + l_z->contents * t767)
         + l_z->contents * t768) + l_z->contents * t769) + l_z->contents * t770)
      - l_1->contents * t772) - l_1->contents * t773);
  t861 = (dv_global->contents[2] + ct->f178 * (((((ct->f78 + ct->f97) + ct->f99)
             + t831) + t838) - t462)) - 9.81;
  t689 = dv_global->contents[5] - ct->f174 * (((((((((ct->f119 + ct->f5) +
    l_1->contents * t753) + l_1->contents * t756) - l_1->contents * t754) -
    l_1->contents * t755) + l_3->contents * t769) + l_3->contents * t770) -
    l_4->contents * t767) - l_4->contents * t768);
  t461 = ct->f120 * ct->f143;
  t690 = t461 * t832;
  t688 = dv_global->contents[0] - ct->f178 * (((((ct->f85 + ct->f96) + ct->f98)
    + t830) + t837) - t690);
  t691 = desired_phi_value->contents * ct->f175;
  t601 = ct->f1[13] - t691;
  t817_tmp = desired_theta_value->contents * ct->f176;
  t602 = ct->f1[12] - t817_tmp;
  t489 = desired_ailerons_value->contents * ct->f177;
  t603 = ct->f1[14] - t489;
  t604 = ct->f1[4] + ct->f9;
  t495 = ct->f1[5] + ct->f9;
  t496 = ct->f1[6] + ct->f9;
  t692 = ct->f1[7] + ct->f9;
  t805_tmp = ct->f1[8] + ct->f11;
  t685 = ct->f1[9] + ct->f11;
  t685_tmp = ct->f1[10] + ct->f11;
  t686 = ct->f1[11] + ct->f11;
  t687 = gamma_quadratic_du->contents * ct->f73;
  t490 = gamma_quadratic_du->contents * ct->f87;
  t491 = gamma_quadratic_du->contents * ct->f92;
  t492 = gamma_quadratic_du->contents * ct->f68;
  t493 = gamma_quadratic_du->contents * ct->f67;
  t494 = gamma_quadratic_du->contents * ct->f93;
  cost = (((((((((((((((((((ct->f86 * (t689 * t689) + ct->f75 * (t851 * t851)) +
    ct->f84 * (t848 * t848)) + ct->f79 * (t854 * t854)) + ct->f74 * (t688 * t688))
                        + ct->f77 * (t861 * t861)) + t687 * (ct->f80 * ct->f80))
                      + t687 * (ct->f81 * ct->f81)) + t687 * (ct->f82 * ct->f82))
                    + t687 * (ct->f83 * ct->f83)) + t493 * (t601 * t601)) + t492
                  * (t602 * t602)) + t494 * (t603 * t603)) + t490 * (t604 * t604))
               + t490 * (t495 * t495)) + t490 * (t496 * t496)) + t490 * (t692 *
              t692)) + t491 * (t805_tmp * t805_tmp)) + t491 * (t685 * t685)) +
          t491 * (t685_tmp * t685_tmp)) + t491 * (t686 * t686);
  *gradient_size = 15;
  t496 = ct->f79 * ct->f172 * t854;
  t692 = ct->f86 * ct->f174 * t689;
  t805_tmp = ct->f84 * ct->f173 * t848;
  t495 = ct->f75 * ct->f178 * t851;
  t603 = ct->f77 * ct->f178 * t861;
  t601 = ct->f74 * ct->f178 * t688;
  t602 = K_p_M->contents * gamma_quadratic_du->contents;
  gradient_data[0] = (((((t496 * (((((ct->f1[4] * 0.36558 - ct->f1[7] * 0.18985)
    + ct->f45) + l_1->contents * t809) + l_z->contents * t813) + 0.51177) * 2.0
    - t692 * (l_1->contents * t805 - l_4->contents * t813) * 2.0) + t805_tmp *
    (l_4->contents * t809 + l_z->contents * t805) * 2.0) + t495 * (ct->f117 *
    t813 + ct->f120 * t809) * 2.0) + t603 * ((ct->f143 * t805 - t688_tmp * t809)
    + t463 * t813) * 2.0) - t601 * ((ct->f118 * t805 + t474_tmp * t809) - t461 *
    t813) * 2.0) - t602 * ct->f32 * ct->f73 * ct->f102 * ct->f55 * ct->f80 * 6.0;
  gradient_data[1] = (((((t496 * (((((ct->f1[5] * 0.36558 - ct->f1[6] * 0.18985)
    + ct->f45) + l_1->contents * t810) - l_z->contents * t814) + 0.51177) * -2.0
    + t692 * (l_1->contents * t806 + l_4->contents * t814) * 2.0) + t805_tmp *
    (l_4->contents * t810 + l_z->contents * t806) * 2.0) + t495 * (ct->f117 *
    t814 + ct->f120 * t810) * 2.0) + t603 * ((ct->f143 * t806 - t688_tmp * t810)
    + t463 * t814) * 2.0) - t601 * ((ct->f118 * t806 + t474_tmp * t810) - t461 *
    t814) * 2.0) - t602 * ct->f41 * ct->f73 * ct->f102 * ct->f56 * ct->f81 * 6.0;
  gradient_data[2] = (((((t692 * (l_1->contents * t807 - l_3->contents * t815) *
    2.0 - t805_tmp * (l_3->contents * t811 - l_z->contents * t807) * 2.0) - t496
    * (l_1->contents * t811 - l_z->contents * t815) * 2.0) + t495 * (ct->f117 *
    t815 + ct->f120 * t811) * 2.0) + t603 * ((ct->f143 * t807 - t688_tmp * t811)
    + t463 * t815) * 2.0) - t601 * ((ct->f118 * t807 + t474_tmp * t811) - t461 *
    t815) * 2.0) - t602 * ct->f52 * ct->f73 * ct->f102 * ct->f57 * ct->f82 * 6.0;
  gradient_data[3] = (((((t692 * (l_1->contents * t808 + l_3->contents * t816) *
    -2.0 - t805_tmp * (l_3->contents * t812 - l_z->contents * t808) * 2.0) +
    t496 * (l_1->contents * t812 + l_z->contents * t816) * 2.0) + t495 *
                        (ct->f117 * t816 + ct->f120 * t812) * 2.0) + t603 *
                       ((ct->f143 * t808 - t688_tmp * t812) + t463 * t816) * 2.0)
                      - t601 * ((ct->f118 * t808 + t474_tmp * t812) - t461 *
    t816) * 2.0) - t602 * ct->f63 * ct->f73 * ct->f102 * ct->f58 * ct->f83 * 6.0;
  t604 = K_p_M->contents * V->contents;
  t602 = t604 * gain_el->contents * gamma_quadratic_du->contents *
    k_tilt->contents;
  gradient_data[4] = ((((((t490 * (ct->f1[4] * 2.0 + ct->f10) - t692 *
    (l_1->contents * t817 + l_4->contents * t464) * 2.0) - t805_tmp *
    (l_4->contents * t821 - l_z->contents * t817) * 2.0) - t495 * (ct->f117 *
    t464 + ct->f120 * t821) * 2.0) + t603 * ((ct->f143 * t817 + t688_tmp * t821)
    - t463 * t464) * 2.0) - t601 * ((ct->f118 * t817 - t474_tmp * t821) + t461 *
    t464) * 2.0) - t496 * (((((ct->f1[0] * -0.36558 + ct->f1[4] * 0.31344) -
    ct->f54) + l_1->contents * t821) + l_z->contents * t464) + 0.28424) * 2.0) +
    t602 * ct->f34 * ct->f73 * ct->f102 * ct->f24 * ct->f28 * ct->f80 * 4.0;
  gradient_data[5] = ((((((t490 * (ct->f1[5] * 2.0 + ct->f10) - t496 *
    (((((ct->f1[1] * 0.36558 - ct->f1[5] * 0.31344) + ct->f54) - l_1->contents *
       t822) + l_z->contents * t826) - 0.28424) * 2.0) + t692 * (l_1->contents *
    t818 - l_4->contents * t826) * 2.0) - t805_tmp * (l_4->contents * t822 -
    l_z->contents * t818) * 2.0) - t495 * (ct->f117 * t826 + ct->f120 * t822) *
                        2.0) + t603 * ((ct->f143 * t818 + t688_tmp * t822) -
    t463 * t826) * 2.0) - t601 * ((ct->f118 * t818 - t474_tmp * t822) + t461 *
    t826) * 2.0) + t602 * ct->f46 * ct->f73 * ct->f102 * ct->f25 * ct->f29 *
    ct->f81 * 4.0;
  gradient_data[6] = ((((((t490 * (ct->f1[6] * 2.0 + ct->f10) + t496 *
    ((((ct->f1[1] * 0.18985 + ct->f1[6] * 0.35594) + l_1->contents * t823) -
      l_z->contents * t827) + 0.24408) * 2.0) + t692 * (l_1->contents * t819 +
    l_3->contents * t827) * 2.0) + t805_tmp * (l_3->contents * t823 +
    l_z->contents * t819) * 2.0) - t495 * (ct->f117 * t827 + ct->f120 * t823) *
                        2.0) + t603 * ((ct->f143 * t819 + t688_tmp * t823) -
    t463 * t827) * 2.0) - t601 * ((ct->f118 * t819 - t474_tmp * t823) + t461 *
    t827) * 2.0) + t602 * ct->f60 * ct->f73 * ct->f102 * ct->f26 * ct->f30 *
    ct->f82 * 4.0;
  gradient_data[7] = ((((((t490 * (ct->f1[7] * 2.0 + ct->f10) - t496 *
    ((((ct->f1[0] * 0.18985 + ct->f1[7] * 0.35594) + l_1->contents * t824) +
      l_z->contents * t828) + 0.24408) * 2.0) - t692 * (l_1->contents * t820 -
    l_3->contents * t828) * 2.0) + t805_tmp * (l_3->contents * t824 +
    l_z->contents * t820) * 2.0) - t495 * (ct->f117 * t828 + ct->f120 * t824) *
                        2.0) + t603 * ((ct->f143 * t820 + t688_tmp * t824) -
    t463 * t828) * 2.0) - t601 * ((ct->f118 * t820 - t474_tmp * t824) + t461 *
    t828) * 2.0) + t602 * ct->f64 * ct->f73 * ct->f102 * ct->f27 * ct->f31 *
    ct->f83 * 4.0;
  t692 = l_4->contents * ct->f86 * ct->f174;
  t805_tmp = l_4->contents * ct->f84 * ct->f173;
  gradient_data[8] = (((((t491 * (ct->f1[8] * 2.0 + ct->f12) + t603 * (t463 *
    t783 + t688_tmp * t795) * 2.0) + t601 * (t461 * t783 + t474_tmp * t795) *
    2.0) - t496 * (l_1->contents * t795 - l_z->contents * t783) * 2.0) + t495 *
                       (ct->f117 * t783 - ct->f120 * t795) * 2.0) + t692 * t783 *
                      t689 * 2.0) - t805_tmp * t795 * t848 * 2.0;
  gradient_data[9] = (((((t491 * (ct->f1[9] * 2.0 + ct->f12) + t603 * (t463 *
    t784 + t688_tmp * t796) * 2.0) + t601 * (t461 * t784 + t474_tmp * t796) *
    2.0) + t496 * (l_1->contents * t796 + l_z->contents * t784) * 2.0) + t495 *
                       (ct->f117 * t784 - ct->f120 * t796) * 2.0) + t692 * t784 *
                      t689 * 2.0) - t805_tmp * t796 * t848 * 2.0;
  t692 = l_3->contents * ct->f86 * ct->f174;
  t805_tmp = l_3->contents * ct->f84 * ct->f173;
  gradient_data[10] = (((((t491 * (ct->f1[10] * 2.0 + ct->f12) + t603 * (t463 *
    t785 + t688_tmp * t797) * 2.0) + t601 * (t461 * t785 + t474_tmp * t797) *
    2.0) + t496 * (l_1->contents * t797 + l_z->contents * t785) * 2.0) + t495 *
                        (ct->f117 * t785 - ct->f120 * t797) * 2.0) - t692 * t785
                       * t689 * 2.0) + t805_tmp * t797 * t848 * 2.0;
  gradient_data[11] = (((((t491 * (ct->f1[11] * 2.0 + ct->f12) + t603 * (t463 *
    t786 + t688_tmp * t798) * 2.0) + t601 * (t461 * t786 + t474_tmp * t798) *
    2.0) - t496 * (l_1->contents * t798 - l_z->contents * t786) * 2.0) + t495 *
                        (ct->f117 * t786 - ct->f120 * t798) * 2.0) - t692 * t786
                       * t689 * 2.0) + t805_tmp * t798 * t848 * 2.0;
  t496 = K_Cd->contents * S->contents * gain_theta->contents * rho->contents *
    ct->f76 * ct->f22 * ct->f65;
  t692 = t604 * gain_theta->contents * gamma_quadratic_du->contents *
    k_tilt->contents;
  gradient_data[12] = (((((((t492 * (ct->f1[12] * 2.0 - t817_tmp * 2.0) + t601 *
    ((((((((gain_theta->contents * ct->f78 + gain_theta->contents * ct->f97) +
           gain_theta->contents * ct->f99) + gain_theta->contents * t831) +
         gain_theta->contents * t838) + gain_theta->contents * -t462) + ct->f118
       * ct->f101) + t474_tmp * ct->f100) - t496 * ct->f120 * ct->f143 * ct->f8)
    * 2.0) - t495 * (ct->f120 * ct->f100 + t496 * ct->f117 * ct->f8) * 2.0) +
    t603 * ((((((((gain_theta->contents * ct->f96 + gain_theta->contents *
                   ct->f98) + gain_theta->contents * t830) +
                 gain_theta->contents * t837) + gain_theta->contents * -t690) -
               ct->f143 * ct->f101) + t688_tmp * ct->f100) - S->contents *
             gain_theta->contents * rho->contents * ct->f76 * ct->f65 * ct->f120
             * ct->f143 * ct->f49 / 2.0) - t496 * ct->f118 * ct->f120 * ct->f8) *
    2.0) - Cm_alpha->contents * S->contents * gain_theta->contents *
    rho->contents * ct->f65 * ct->f84 * ct->f173 * t848 * wing_chord->contents)
    + t692 * ct->f34 * ct->f73 * ct->f102 * ct->f24 * ct->f28 * ct->f80 * 4.0) +
                        t692 * ct->f46 * ct->f73 * ct->f102 * ct->f25 * ct->f29 *
                        ct->f81 * 4.0) + t692 * ct->f60 * ct->f73 * ct->f102 *
                       ct->f26 * ct->f30 * ct->f82 * 4.0) + t692 * ct->f64 *
    ct->f73 * ct->f102 * ct->f27 * ct->f31 * ct->f83 * 4.0;
  t496 = gain_phi->contents * ct->f117;
  t692 = gain_phi->contents * ct->f118;
  t805_tmp = gain_phi->contents * ct->f143;
  gradient_data[13] = ((t493 * (ct->f1[13] * 2.0 - t691 * 2.0) + t495 * (((t496 *
    ct->f94 + t496 * t833) - gain_phi->contents * ct->f120 * t832) - S->contents
    * gain_phi->contents * rho->contents * ct->f76 * ct->f65 * ct->f120 *
    ct->f49 / 2.0) * 2.0) + t603 * (((t692 * ct->f66 + t692 * ct->f95) + t692 *
    t834) + t692 * t835) * 2.0) + t601 * (((t805_tmp * ct->f66 + t805_tmp *
    ct->f95) + t805_tmp * t834) + t805_tmp * t835) * 2.0;
  gradient_data[14] = t494 * (ct->f1[14] * 2.0 - t489 * 2.0) -
    CL_aileron->contents * S->contents * gain_ailerons->contents * rho->contents
    * ct->f65 * ct->f79 * ct->f172 * t854;
  return cost;
}

static void fullColLDL2_(f_struct_T *obj, int NColsRemain)
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
    LD_diagOffset = k << 5;
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
          i = jA + 33;
          i1 = subMatrixDim + jA;
          for (ijA = i; ijA <= i1 + 33; ijA++) {
            obj->FMat[ijA - 1] += y * temp;
          }
        }

        jA += 31;
      }
    }

    alpha1 = 1.0 / obj->FMat[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    for (jA = offset1; jA <= i + 2; jA++) {
      obj->FMat[jA - 1] *= alpha1;
    }
  }
}

static void iterate(const double H[225], const double f[16], g_struct_T
                    *solution, d_struct_T *memspace, h_struct_T *workingset,
                    e_struct_T *qrmanager, f_struct_T *cholmanager, struct_T
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

  memset(&solution->lambda[0], 0, 31U * sizeof(double));
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      double minLambda;
      int idxQR;
      int ix0;
      bool guard1 = false;
      bool guard2 = false;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          {
            double c;
            ix0 = (workingset->nActiveConstr - 1) << 4;
            iyend = qrmanager->mrows;
            idxQR = qrmanager->ncols + 1;
            if (iyend <= idxQR) {
              idxQR = iyend;
            }

            qrmanager->minRowCol = idxQR;
            idxQR = 31 * qrmanager->ncols;
            if (qrmanager->mrows != 0) {
              iyend = idxQR + qrmanager->mrows;
              if (idxQR + 1 <= iyend) {
                memset(&qrmanager->QR[idxQR], 0, (unsigned int)(iyend - idxQR) *
                       sizeof(double));
              }

              i = 31 * (qrmanager->mrows - 1) + 1;
              for (iyend = 1; iyend <= i; iyend += 31) {
                c = 0.0;
                nActiveConstr = (iyend + qrmanager->mrows) - 1;
                for (idx = iyend; idx <= nActiveConstr; idx++) {
                  c += qrmanager->Q[idx - 1] * workingset->ATwset[(ix0 + idx) -
                    iyend];
                }

                nActiveConstr = idxQR + div_nde_s32_floor(iyend - 1);
                qrmanager->QR[nActiveConstr] += c;
              }
            }

            qrmanager->ncols++;
            i = qrmanager->ncols - 1;
            qrmanager->jpvt[i] = qrmanager->ncols;
            for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--)
            {
              nActiveConstr = idx + 31 * i;
              temp = qrmanager->QR[nActiveConstr + 1];
              c = xrotg(&qrmanager->QR[nActiveConstr], &temp, &s);
              qrmanager->QR[nActiveConstr + 1] = temp;
              iyend = 31 * idx;
              ix0 = qrmanager->mrows;
              if (qrmanager->mrows >= 1) {
                for (nActiveConstr = 0; nActiveConstr < ix0; nActiveConstr++) {
                  idxQR = iyend + nActiveConstr;
                  minLambda = qrmanager->Q[idxQR + 31];
                  temp = c * qrmanager->Q[idxQR] + s * minLambda;
                  qrmanager->Q[idxQR + 31] = c * minLambda - s * qrmanager->
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
          bool b_guard1 = false;
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
              bool b_guard2 = false;
              idx = qrmanager->ncols;
              b_guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                idxQR = qrmanager->mrows + 31 * (qrmanager->ncols - 1);
                while ((idx > qrmanager->mrows) && (fabs(qrmanager->QR[idxQR - 1])
                        >= minLambda)) {
                  idx--;
                  idxQR -= 31;
                }

                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  b_guard2 = true;
                }
              } else {
                b_guard2 = true;
              }

              if (b_guard2) {
                idxQR = idx + 31 * (idx - 1);
                while ((idx >= 1) && (fabs(qrmanager->QR[idxQR - 1]) >=
                                      minLambda)) {
                  idx--;
                  idxQR -= 32;
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
                iyend = (idx + (idx - 1) * 31) - 1;
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

static void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[496],
  const double H[225], const double f[16], const double x[16])
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

static double maxConstraintViolation(const h_struct_T *obj, const double x[496],
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

static void qrf(double A[961], int m, int n, int nfxd, double tau[31])
{
  double work[31];
  double atmp;
  int b_i;
  int i;
  memset(&tau[0], 0, 31U * sizeof(double));
  memset(&work[0], 0, 31U * sizeof(double));
  i = (unsigned char)nfxd;
  for (b_i = 0; b_i < i; b_i++) {
    double d;
    int ii;
    int mmi;
    ii = b_i * 31 + b_i;
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
      xzlarf(mmi, (n - b_i) - 1, ii + 1, d, A, ii + 32, work);
      A[ii] = atmp;
    }
  }
}

static void removeConstr(h_struct_T *obj, int idx_global)
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
    obj->ATwset[idx + ((idx_global - 1) << 4)] = obj->ATwset[idx + (i << 4)];
  }

  obj->bwset[idx_global - 1] = obj->bwset[i];
  obj->nActiveConstr = i;
  obj->nWConstr[TYPE_tmp]--;
}

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

static void setProblemType(h_struct_T *obj, int PROBLEM_TYPE)
{
  int i;
  int idx;
  int idx_col;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 15;
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
      obj->nVar = 16;
      obj->mConstr = obj->mConstrOrig + 1;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
      }

      i = (unsigned char)obj->sizes[0];
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[(idx << 4) + 15] = 0.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = 16;
      obj->lb[15] = 1.0E-5;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx = idxStartIneq; idx <= i; idx++) {
        obj->ATwset[((idx - 1) << 4) + 15] = -1.0;
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
      obj->nVar = 15;
      obj->mConstr = 30;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegularized[i];
      }

      if (obj->probType != 4) {
        int i1;
        int idxStartIneq;
        int idx_lb;
        idx_lb = 15;
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
          idx_lb = ((idx_col - 1) << 4) - 1;
          if (obj->Wid[idx_col - 1] == 3) {
            i1 = obj->Wlocalidx[idx_col - 1];
            idx = i1 + 14;
            if (idx >= 16) {
              memset(&obj->ATwset[idx_lb + 16], 0, (unsigned int)(((idx + idx_lb)
                       - idx_lb) - 15) * sizeof(double));
            }

            obj->ATwset[(i1 + idx_lb) + 15] = -1.0;
            i1 += 16;
            if (i1 <= 15) {
              memset(&obj->ATwset[i1 + idx_lb], 0, (unsigned int)(((idx_lb - i1)
                       - idx_lb) + 16) * sizeof(double));
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
      obj->nVar = 16;
      obj->mConstr = 31;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
      }

      i = (unsigned char)obj->sizes[0];
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[(idx << 4) + 15] = 0.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = 16;
      obj->lb[15] = 1.0E-5;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx = idxStartIneq; idx <= i; idx++) {
        obj->ATwset[((idx - 1) << 4) + 15] = -1.0;
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

static void solve(const f_struct_T *obj, double rhs[16])
{
  int i;
  int j;
  int n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    int jA;
    for (j = 0; j < n_tmp; j++) {
      double temp;
      jA = j * 31;
      temp = rhs[j];
      for (i = 0; i < j; i++) {
        temp -= obj->FMat[jA + i] * rhs[i];
      }

      rhs[j] = temp / obj->FMat[jA + j];
    }

    for (j = n_tmp; j >= 1; j--) {
      jA = (j + (j - 1) * 31) - 1;
      rhs[j - 1] /= obj->FMat[jA];
      for (i = 0; i <= j - 2; i++) {
        int ix;
        ix = (j - i) - 2;
        rhs[ix] -= rhs[j - 1] * obj->FMat[(jA - i) - 1];
      }
    }
  }
}

static void sortLambdaQP(double lambda[31], int WorkingSet_nActiveConstr, const
  int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[31], const int WorkingSet_Wlocalidx[31], double workspace[496])
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

static bool step(int *STEP_TYPE, double Hessian[225], const double lb[15], const
                 double ub[15], g_struct_T *TrialState, b_struct_T
                 *MeritFunction, d_struct_T *memspace, h_struct_T *WorkingSet,
                 e_struct_T *QRManager, f_struct_T *CholManager, struct_T
                 *QPObjective, i_struct_T *qpoptions)
{
  i_struct_T b_qpoptions;
  double dv[16];
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
  bool guard1 = false;
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
        memcpy(&dv[0], &TrialState->xstar[0], 16U * sizeof(double));
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

        memcpy(&TrialState->xstar[0], &dv[0], 16U * sizeof(double));
        idxEndIneq_tmp_tmp = WorkingSet->nVar;
        beta = 0.0;
        idxStartIneq = (unsigned char)WorkingSet->nVar;
        for (k = 0; k < idxStartIneq; k++) {
          beta += Hessian[k + 15 * k];
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
        memcpy(&dv[0], &TrialState->grad[0], 16U * sizeof(double));
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

      memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0], 31U *
             sizeof(double));
      memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (unsigned int)
             idxStartIneq * sizeof(double));
      memcpy(&dv[0], &TrialState->grad[0], 16U * sizeof(double));
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
        memcpy(&TrialState->lambda[0], &TrialState->lambdaStopTest[0], 31U *
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
        for (k = 0; k < 15; k++) {
          oldDirIdx = fmax(oldDirIdx, fabs(TrialState->grad[k]));
          s = fmax(s, fabs(TrialState->xstar[k]));
        }

        s = fmax(2.2204460492503131E-16, oldDirIdx / s);
        for (idxStartIneq = 0; idxStartIneq < 15; idxStartIneq++) {
          temp = 15 * idxStartIneq;
          for (k = 0; k < idxStartIneq; k++) {
            Hessian[temp + k] = 0.0;
          }

          temp = idxStartIneq + 15 * idxStartIneq;
          Hessian[temp] = s;
          idxEndIneq_tmp_tmp = 13 - idxStartIneq;
          if (idxEndIneq_tmp_tmp >= 0) {
            memset(&Hessian[temp + 1], 0, (unsigned int)(((idxEndIneq_tmp_tmp +
                      temp) - temp) + 1) * sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    memcpy(&dv[0], &TrialState->delta_x[0], 16U * sizeof(double));
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

    memcpy(&TrialState->delta_x[0], &dv[0], 16U * sizeof(double));
  }

  return stepSuccess;
}

static bool test_exit(b_struct_T *MeritFunction, const h_struct_T *WorkingSet,
                      g_struct_T *TrialState, const double lb[15], const double
                      ub[15], bool *Flags_fevalOK, bool *Flags_done, bool
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

    if (isFeasible && (smax <= 1.0E-9 * s)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }

  return Flags_gradOK;
}

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

static void xgemm(int m, int n, int k, const double A[225], int lda, const
                  double B[961], int ib0, double C[496])
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
    lastColC = 31 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 31) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        memset(&C[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof(double));
      }
    }

    for (cr = 0; cr <= lastColC; cr += 31) {
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

      br += 31;
    }
  }
}

static void xgemv(int m, int n, const double A[225], int lda, const double x[16],
                  double y[15])
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

static void xgeqp3(double A[961], int m, int n, int jpvt[31], double tau[31])
{
  double vn1[31];
  double vn2[31];
  double work[31];
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

  memset(&tau[0], 0, 31U * sizeof(double));
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
          ix = pvt * 31;
          iy = (nfxd - 1) * 31;
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
      memset(&work[0], 0, 31U * sizeof(double));
      memset(&vn1[0], 0, 31U * sizeof(double));
      memset(&vn2[0], 0, 31U * sizeof(double));
      i = nfxd + 1;
      for (pvt = i; pvt <= n; pvt++) {
        d = xnrm2(m - nfxd, A, (nfxd + (pvt - 1) * 31) + 1);
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
        iy = (b_i - 1) * 31;
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
          ix = pvt * 31;
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
          xzlarf(mmi + 1, nmi - 1, ii + 1, d, A, ii + 32, work);
          A[ii] = temp;
        }

        for (pvt = ip1; pvt <= n; pvt++) {
          nfxd = b_i + (pvt - 1) * 31;
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

static double xnrm2(int n, const double x[961], int ix0)
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

static int xpotrf(int n, double A[961])
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
    idxA1j = j * 31;
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
        ia0 = idxA1j + 32;
        idxAjjp1 = idxAjj + 32;
        if ((j != 0) && (nmj + 1 != 0)) {
          i = (idxA1j + 31 * nmj) + 32;
          for (iac = ia0; iac <= i; iac += 31) {
            int i1;
            c = 0.0;
            i1 = (iac + j) - 1;
            for (ia = iac; ia <= i1; ia++) {
              c += A[ia - 1] * A[(idxA1j + ia) - iac];
            }

            i1 = (idxAjj + div_nde_s32_floor((iac - idxA1j) - 32) * 31) + 31;
            A[i1] -= c;
          }
        }

        ssq = 1.0 / ssq;
        i = (idxAjj + 31 * nmj) + 32;
        for (nmj = idxAjjp1; nmj <= i; nmj += 31) {
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

static void xzlarf(int m, int n, int iv0, double tau, double C[961], int ic0,
                   double work[31])
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
      i = ic0 + lastc * 31;
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

      b_i = ic0 + 31 * lastc;
      for (iac = ic0; iac <= b_i; iac += 31) {
        c = 0.0;
        i = (iac + lastv) - 1;
        for (ia = iac; ia <= i; ia++) {
          c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
        }

        i = div_nde_s32_floor(iac - ic0);
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

        i += 31;
      }
    }
  }
}

static double xzlarfg(int n, double *alpha1, double x[961], int ix0)
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

void Nonlinear_controller_fcn_control_rf_aero_models(double m, double I_xx,
  double I_yy, double I_zz, double l_1, double l_2, double l_3, double l_4,
  double l_z, double Phi, double Theta, double Omega_1, double Omega_2, double
  Omega_3, double Omega_4, double b_1, double b_2, double b_3, double b_4,
  double g_1, double g_2, double g_3, double g_4, double delta_ailerons, double
  W_act_motor_const, double W_act_motor_speed, double W_act_tilt_el_const,
  double W_act_tilt_el_speed, double W_act_tilt_az_const, double
  W_act_tilt_az_speed, double W_act_theta_const, double W_act_theta_speed,
  double W_act_phi_const, double W_act_phi_speed, double W_act_ailerons_const,
  double W_act_ailerons_speed, double W_dv_1, double W_dv_2, double W_dv_3,
  double W_dv_4, double W_dv_5, double W_dv_6, double max_omega, double
  min_omega, double max_b, double min_b, double max_g, double min_g, double
  max_theta, double min_theta, double max_phi, double max_delta_ailerons, double
  min_delta_ailerons, const double dv[6], double p, double q, double r, double
  Cm_zero, double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha, double
  CL_aileron, double rho, double V, double S, double wing_chord, double
  flight_path_angle, double max_alpha, double min_alpha, double Beta, double
  gamma_quadratic_du, double desired_motor_value, double desired_el_value,
  double desired_az_value, double desired_theta_value, double desired_phi_value,
  double desired_ailerons_value, double k_alt_tilt_constraint, double
  min_alt_tilt_constraint, double lidar_alt_corrected, double approach_mode,
  double verbose, double aoa_protection_speed, double transition_speed, double
  u_out[15], double residuals[6], double *elapsed_time, double *N_iterations,
  double *N_evaluation, double *exitflag)
{
  b_captured_var dv_global;
  c_struct_T b_expl_temp;
  c_struct_T expl_temp;
  captured_var K_p_M;
  captured_var W_act_ailerons;
  captured_var W_act_motor;
  captured_var W_act_phi;
  captured_var W_act_theta;
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
  captured_var b_S;
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
  captured_var b_desired_phi_value;
  captured_var b_desired_theta_value;
  captured_var b_flight_path_angle;
  captured_var b_gamma_quadratic_du;
  captured_var b_l_1;
  captured_var b_l_3;
  captured_var b_l_4;
  captured_var b_l_z;
  captured_var b_m;
  captured_var b_p;
  captured_var b_q;
  captured_var b_r;
  captured_var b_rho;
  captured_var b_wing_chord;
  captured_var gain_ailerons;
  captured_var gain_airspeed;
  captured_var gain_az;
  captured_var gain_el;
  captured_var gain_motor;
  captured_var gain_phi;
  captured_var gain_theta;
  captured_var k_tilt;
  captured_var prop_Cd_0;
  captured_var prop_Cd_a;
  captured_var prop_Cl_0;
  captured_var prop_Cl_a;
  captured_var prop_R;
  captured_var prop_delta;
  captured_var prop_sigma;
  captured_var prop_theta;
  double actual_u[15];
  double u_max[15];
  double u_max_scaled[15];
  double u_min[15];
  double current_accelerations[6];
  double final_accelerations[6];
  double b_max_approach;
  double b_max_tilt_value_approach;
  double b_min_approach;
  double g_max_approach;
  double g_min_approach;
  double max_theta_protection;
  double min_theta_protection;
  int i;
  char c_expl_temp[3];
  (void)l_2;
  if (!isInitialized_Nonlinear_controller_fcn_control_rf_aero_models) {
    Nonlinear_controller_fcn_control_rf_aero_models_initialize();
  }

  b_m.contents = m;
  b_I_xx.contents = I_xx;
  b_I_yy.contents = I_yy;
  b_I_zz.contents = I_zz;
  b_l_1.contents = l_1;
  b_l_3.contents = l_3;
  b_l_4.contents = l_4;
  b_l_z.contents = l_z;
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
  b_desired_theta_value.contents = desired_theta_value;
  b_desired_phi_value.contents = desired_phi_value;
  b_desired_ailerons_value.contents = desired_ailerons_value;

  /*  Create variables necessary for the optimization */
  if (b_V.contents > aoa_protection_speed) {
    b_min_approach = (max_alpha + b_flight_path_angle.contents) * 180.0 /
      3.1415926535897931;
    max_theta_protection = fmin(max_theta, b_min_approach);
    b_min_approach = (min_alpha + b_flight_path_angle.contents) * 180.0 /
      3.1415926535897931;
    min_theta_protection = fmax(min_theta, b_min_approach);
  } else {
    max_theta_protection = max_theta;
    min_theta_protection = min_theta;
  }

  /*  max_theta_protection = max_theta; */
  /*  min_theta_protection = min_theta; */
  /* 15 m/s is the maximum airspeed at which the model provides valid results. */
  gain_motor.contents = max_omega / 2.0;
  gain_el.contents = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
  gain_az.contents = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;
  gain_theta.contents = (max_theta_protection - min_theta_protection) *
    3.1415926535897931 / 180.0 / 2.0;
  gain_phi.contents = max_phi * 3.1415926535897931 / 180.0;
  gain_ailerons.contents = (max_delta_ailerons - min_delta_ailerons) *
    3.1415926535897931 / 180.0 / 2.0;
  gain_airspeed.contents = 15.0;
  actual_u[0] = Omega_1;
  actual_u[1] = Omega_2;
  actual_u[2] = Omega_3;
  actual_u[3] = Omega_4;
  actual_u[4] = b_1;
  actual_u[5] = b_2;
  actual_u[6] = b_3;
  actual_u[7] = b_4;
  actual_u[8] = g_1;
  actual_u[9] = g_2;
  actual_u[10] = g_3;
  actual_u[11] = g_4;
  actual_u[12] = Theta;
  actual_u[13] = Phi;
  actual_u[14] = delta_ailerons;

  /* Build the max and minimum actuator array:  */
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
  u_max[12] = max_theta_protection;
  u_max[13] = max_phi;
  u_max[14] = max_delta_ailerons;
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
  u_min[12] = min_theta_protection;
  u_min[13] = -max_phi;
  u_min[14] = min_delta_ailerons;
  if (approach_mode != 0.0) {
    double max_tilt_value_approach[2];
    max_tilt_value_approach[0] = 0.0;
    max_tilt_value_approach[1] = k_alt_tilt_constraint * lidar_alt_corrected -
      min_alt_tilt_constraint * k_alt_tilt_constraint;
    b_max_tilt_value_approach = maximum(max_tilt_value_approach);

    /* Elevation angle */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_b;
    b_max_approach = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_b;
    b_min_approach = maximum(max_tilt_value_approach);

    /* Azimuth angle */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_g;
    g_max_approach = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_g;
    g_min_approach = maximum(max_tilt_value_approach);
    u_max[4] = b_max_approach;
    u_min[4] = b_min_approach;
    u_max[8] = g_max_approach;
    u_min[8] = g_min_approach;
    u_max[5] = b_max_approach;
    u_min[5] = b_min_approach;
    u_max[9] = g_max_approach;
    u_min[9] = g_min_approach;
    u_max[6] = b_max_approach;
    u_min[6] = b_min_approach;
    u_max[10] = g_max_approach;
    u_min[10] = g_min_approach;
    u_max[7] = b_max_approach;
    u_min[7] = b_min_approach;
    u_max[11] = g_max_approach;
    u_min[11] = g_min_approach;

    /* Pitch angle  */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_theta_protection;
    u_max[12] = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_theta_protection;
    u_min[12] = maximum(max_tilt_value_approach);

    /* Roll angle  */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_phi;
    u_max[13] = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = -max_phi;
    u_min[13] = maximum(max_tilt_value_approach);
  }

  for (i = 0; i < 11; i++) {
    u_max[i + 4] = u_max[i + 4] * 3.1415926535897931 / 180.0;
    u_min[i + 4] = u_min[i + 4] * 3.1415926535897931 / 180.0;
  }

  memcpy(&u_max_scaled[0], &u_max[0], 15U * sizeof(double));
  u_max_scaled[0] = u_max[0] / gain_motor.contents;
  u_max_scaled[1] = u_max[1] / gain_motor.contents;
  u_max_scaled[2] = u_max[2] / gain_motor.contents;
  u_max_scaled[3] = u_max[3] / gain_motor.contents;
  b_max_tilt_value_approach = u_min[0] / gain_motor.contents;
  b_max_approach = u_min[1] / gain_motor.contents;
  b_min_approach = u_min[2] / gain_motor.contents;
  g_max_approach = u_min[3] / gain_motor.contents;
  u_min[0] = b_max_tilt_value_approach;
  u_min[1] = b_max_approach;
  u_min[2] = b_min_approach;
  u_min[3] = g_max_approach;
  b_max_tilt_value_approach = u_max_scaled[4] / gain_el.contents;
  b_max_approach = u_max_scaled[5] / gain_el.contents;
  b_min_approach = u_max_scaled[6] / gain_el.contents;
  g_max_approach = u_max_scaled[7] / gain_el.contents;
  u_max_scaled[4] = b_max_tilt_value_approach;
  u_max_scaled[5] = b_max_approach;
  u_max_scaled[6] = b_min_approach;
  u_max_scaled[7] = g_max_approach;
  b_max_tilt_value_approach = u_min[4] / gain_el.contents;
  b_max_approach = u_min[5] / gain_el.contents;
  b_min_approach = u_min[6] / gain_el.contents;
  g_max_approach = u_min[7] / gain_el.contents;
  u_min[4] = b_max_tilt_value_approach;
  u_min[5] = b_max_approach;
  u_min[6] = b_min_approach;
  u_min[7] = g_max_approach;
  b_max_tilt_value_approach = u_max_scaled[8] / gain_az.contents;
  b_max_approach = u_max_scaled[9] / gain_az.contents;
  b_min_approach = u_max_scaled[10] / gain_az.contents;
  g_max_approach = u_max_scaled[11] / gain_az.contents;
  u_max_scaled[8] = b_max_tilt_value_approach;
  u_max_scaled[9] = b_max_approach;
  u_max_scaled[10] = b_min_approach;
  u_max_scaled[11] = g_max_approach;
  b_max_tilt_value_approach = u_min[8] / gain_az.contents;
  b_max_approach = u_min[9] / gain_az.contents;
  b_min_approach = u_min[10] / gain_az.contents;
  g_max_approach = u_min[11] / gain_az.contents;
  u_min[8] = b_max_tilt_value_approach;
  u_min[9] = b_max_approach;
  u_min[10] = b_min_approach;
  u_min[11] = g_max_approach;
  u_max_scaled[12] /= gain_theta.contents;
  u_min[12] /= gain_theta.contents;
  u_max_scaled[13] /= gain_phi.contents;
  u_min[13] /= gain_phi.contents;
  u_max_scaled[14] /= gain_ailerons.contents;
  u_min[14] /= gain_ailerons.contents;
  memcpy(&u_max[0], &actual_u[0], 15U * sizeof(double));
  u_max[0] = Omega_1 / gain_motor.contents;
  u_max[1] = Omega_2 / gain_motor.contents;
  u_max[2] = Omega_3 / gain_motor.contents;
  u_max[3] = Omega_4 / gain_motor.contents;
  b_max_tilt_value_approach = u_max[4] / gain_el.contents;
  b_max_approach = u_max[5] / gain_el.contents;
  b_min_approach = u_max[6] / gain_el.contents;
  g_max_approach = u_max[7] / gain_el.contents;
  u_max[4] = b_max_tilt_value_approach;
  u_max[5] = b_max_approach;
  u_max[6] = b_min_approach;
  u_max[7] = g_max_approach;
  b_max_tilt_value_approach = u_max[8] / gain_az.contents;
  b_max_approach = u_max[9] / gain_az.contents;
  b_min_approach = u_max[10] / gain_az.contents;
  g_max_approach = u_max[11] / gain_az.contents;
  u_max[8] = b_max_tilt_value_approach;
  u_max[9] = b_max_approach;
  u_max[10] = b_min_approach;
  u_max[11] = g_max_approach;
  u_max[12] /= gain_theta.contents;
  u_max[13] /= gain_phi.contents;
  u_max[14] /= gain_ailerons.contents;

  /*  Apply Nonlinear optimization algorithm: */
  c_compute_acc_nonlinear_control(actual_u, b_p.contents, b_q.contents,
    b_r.contents, b_m.contents, b_I_xx.contents, b_I_yy.contents,
    b_I_zz.contents, b_l_1.contents, b_l_3.contents, b_l_4.contents,
    b_l_z.contents, b_Cl_alpha.contents, b_Cd_zero.contents, b_K_Cd.contents,
    b_Cm_alpha.contents, b_Cm_zero.contents, b_CL_aileron.contents,
    b_rho.contents, b_V.contents, b_S.contents, b_wing_chord.contents,
    b_flight_path_angle.contents, gain_motor.contents, gain_airspeed.contents,
    gain_el.contents, gain_theta.contents, gain_az.contents, gain_phi.contents,
    current_accelerations);
  for (i = 0; i < 6; i++) {
    dv_global.contents[i] = dv[i] + current_accelerations[i];
  }

  /* Pseudo-control hedging:  */
  if (b_V.contents > transition_speed) {
    b_max_approach = b_Cl_alpha.contents * max_alpha;
    b_min_approach = b_Cl_alpha.contents * min_alpha;
    b_max_tilt_value_approach = b_V.contents;
    g_max_approach = b_max_approach * 0.5 * b_rho.contents * b_S.contents *
      (b_max_tilt_value_approach * b_max_tilt_value_approach) * cos
      (max_theta_protection * 3.1415926535897931 / 180.0);
    b_max_tilt_value_approach = b_V.contents;
    b_max_approach = b_min_approach * 0.5 * b_rho.contents * b_S.contents *
      (b_max_tilt_value_approach * b_max_tilt_value_approach) * cos
      (min_theta_protection * 3.1415926535897931 / 180.0);
    b_max_tilt_value_approach = 9.81 - b_max_approach / b_m.contents;
    b_max_approach = 9.81 - g_max_approach / b_m.contents;
    b_min_approach = 0.0;
    if (dv_global.contents[2] >= b_max_tilt_value_approach) {
      b_min_approach = dv_global.contents[2] - b_max_tilt_value_approach;
    } else if (dv_global.contents[2] <= b_max_approach) {
      b_min_approach = dv_global.contents[2] - b_max_approach;
    }

    dv_global.contents[2] -= b_min_approach;
  }

  /* Compute weights for actuators and make sure they are always positive */
  b_min_approach = W_act_motor_const + W_act_motor_speed * b_V.contents;
  W_act_motor.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_tilt_el_const + W_act_tilt_el_speed * b_V.contents;
  W_act_tilt_el.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_tilt_az_const + W_act_tilt_az_speed * b_V.contents;
  W_act_tilt_az.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_theta_const + W_act_theta_speed * b_V.contents;
  W_act_theta.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_phi_const + W_act_phi_speed * b_V.contents;
  W_act_phi.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_ailerons_const + W_act_ailerons_speed * b_V.contents;
  W_act_ailerons.contents = fmax(0.0, b_min_approach);

  /*  New prop model */
  prop_Cl_0.contents = 0.12;
  prop_Cl_a.contents = 3.46;
  prop_Cd_0.contents = 0.05;
  prop_Cd_a.contents = 0.36;
  prop_sigma.contents = 0.0551;
  prop_delta.contents = 0.2;
  prop_theta.contents = 0.2188;
  prop_R.contents = 0.127;
  K_p_M.contents = 3.670182E-7;
  k_tilt.contents = 0.13333333333333333;

  /* Default values for the optimizer: */
  tic();
  expl_temp.Cd_zero = &b_Cd_zero;
  expl_temp.Cm_zero = &b_Cm_zero;
  expl_temp.desired_motor_value = &b_desired_motor_value;
  expl_temp.desired_az_value = &b_desired_az_value;
  expl_temp.desired_el_value = &b_desired_el_value;
  expl_temp.prop_Cl_0 = &prop_Cl_0;
  expl_temp.prop_theta = &prop_theta;
  expl_temp.prop_Cl_a = &prop_Cl_a;
  expl_temp.m = &b_m;
  expl_temp.gain_airspeed = &gain_airspeed;
  expl_temp.I_zz = &b_I_zz;
  expl_temp.r = &b_r;
  expl_temp.I_yy = &b_I_yy;
  expl_temp.q = &b_q;
  expl_temp.p = &b_p;
  expl_temp.I_xx = &b_I_xx;
  expl_temp.prop_Cd_0 = &prop_Cd_0;
  expl_temp.prop_R = &prop_R;
  expl_temp.prop_Cd_a = &prop_Cd_a;
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
  expl_temp.W_act_theta = &W_act_theta;
  expl_temp.W_act_phi = &W_act_phi;
  expl_temp.Cl_alpha = &b_Cl_alpha;
  expl_temp.prop_delta = &prop_delta;
  expl_temp.Beta = &b_Beta;
  expl_temp.flight_path_angle = &b_flight_path_angle;
  expl_temp.gain_ailerons = &gain_ailerons;
  expl_temp.CL_aileron = &b_CL_aileron;
  expl_temp.gain_phi = &gain_phi;
  expl_temp.wing_chord = &b_wing_chord;
  expl_temp.Cm_alpha = &b_Cm_alpha;
  expl_temp.S = &b_S;
  expl_temp.K_Cd = &b_K_Cd;
  expl_temp.gain_theta = &gain_theta;
  expl_temp.k_tilt = &k_tilt;
  expl_temp.K_p_M = &K_p_M;
  expl_temp.desired_ailerons_value = &b_desired_ailerons_value;
  expl_temp.desired_theta_value = &b_desired_theta_value;
  expl_temp.desired_phi_value = &b_desired_phi_value;
  expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  expl_temp.l_z = &b_l_z;
  expl_temp.l_3 = &b_l_3;
  expl_temp.l_4 = &b_l_4;
  expl_temp.l_1 = &b_l_1;
  expl_temp.gain_az = &gain_az;
  expl_temp.gain_el = &gain_el;
  expl_temp.rho = &b_rho;
  expl_temp.prop_sigma = &prop_sigma;
  expl_temp.gain_motor = &gain_motor;
  expl_temp.V = &b_V;
  expl_temp.dv_global = &dv_global;
  b_expl_temp = expl_temp;
  fmincon(&b_expl_temp, u_max, u_min, u_max_scaled, u_out, exitflag,
          N_iterations, N_evaluation, c_expl_temp, &b_max_approach,
          &b_min_approach, &g_max_approach, &g_min_approach);
  *elapsed_time = toc();
  b_min_approach = gain_motor.contents;
  u_out[0] *= b_min_approach;
  u_out[1] *= b_min_approach;
  u_out[2] *= b_min_approach;
  u_out[3] *= b_min_approach;
  b_min_approach = gain_el.contents;
  u_out[4] *= b_min_approach;
  u_out[5] *= b_min_approach;
  u_out[6] *= b_min_approach;
  u_out[7] *= b_min_approach;
  b_min_approach = gain_az.contents;
  u_out[8] *= b_min_approach;
  u_out[9] *= b_min_approach;
  u_out[10] *= b_min_approach;
  u_out[11] *= b_min_approach;
  u_out[12] *= gain_theta.contents;
  u_out[13] *= gain_phi.contents;
  u_out[14] *= gain_ailerons.contents;
  c_compute_acc_nonlinear_control(u_out, b_p.contents, b_q.contents,
    b_r.contents, b_m.contents, b_I_xx.contents, b_I_yy.contents,
    b_I_zz.contents, b_l_1.contents, b_l_3.contents, b_l_4.contents,
    b_l_z.contents, b_Cl_alpha.contents, b_Cd_zero.contents, b_K_Cd.contents,
    b_Cm_alpha.contents, b_Cm_zero.contents, b_CL_aileron.contents,
    b_rho.contents, b_V.contents, b_S.contents, b_wing_chord.contents,
    b_flight_path_angle.contents, gain_motor.contents, gain_airspeed.contents,
    gain_el.contents, gain_theta.contents, gain_az.contents, gain_phi.contents,
    final_accelerations);
  for (i = 0; i < 6; i++) {
    residuals[i] = dv_global.contents[i] - final_accelerations[i];
  }

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
    printf(" Theta [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[12] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Phi [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[13] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Ailerons deflection [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[14] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("\n Elapsed time = %f \n", *elapsed_time);
    fflush(stdout);
    printf("\n Number of iterations / evaluations = %f ", *N_iterations);
    fflush(stdout);
    printf("/ %f \n", *N_evaluation);
    fflush(stdout);
    memcpy(&u_max[0], &u_out[0], 15U * sizeof(double));
    u_max[0] = u_out[0] / gain_motor.contents;
    u_max[1] = u_out[1] / gain_motor.contents;
    u_max[2] = u_out[2] / gain_motor.contents;
    u_max[3] = u_out[3] / gain_motor.contents;
    b_max_tilt_value_approach = u_max[4] / gain_el.contents;
    b_max_approach = u_max[5] / gain_el.contents;
    b_min_approach = u_max[6] / gain_el.contents;
    g_max_approach = u_max[7] / gain_el.contents;
    u_max[4] = b_max_tilt_value_approach;
    u_max[5] = b_max_approach;
    u_max[6] = b_min_approach;
    u_max[7] = g_max_approach;
    b_max_tilt_value_approach = u_max[8] / gain_az.contents;
    b_max_approach = u_max[9] / gain_az.contents;
    b_min_approach = u_max[10] / gain_az.contents;
    g_max_approach = u_max[11] / gain_az.contents;
    u_max[8] = b_max_tilt_value_approach;
    u_max[9] = b_max_approach;
    u_max[10] = b_min_approach;
    u_max[11] = g_max_approach;
    u_max[12] /= gain_theta.contents;
    u_max[13] /= gain_phi.contents;
    u_max[14] /= gain_ailerons.contents;
    printf("\n Solution scaled norm = %f \n", b_norm(u_max));
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
      b_min_approach = dv_global.contents[i];
      printf(" %f ", b_min_approach);
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
    printf(" %f \n", c_norm(residuals));
    fflush(stdout);
  }
}

void Nonlinear_controller_fcn_control_rf_aero_models_initialize(void)
{
  savedTime_not_empty = false;
  freq_not_empty = false;
  isInitialized_Nonlinear_controller_fcn_control_rf_aero_models = true;
}

void Nonlinear_controller_fcn_control_rf_aero_models_terminate(void)
{
  isInitialized_Nonlinear_controller_fcn_control_rf_aero_models = false;
}

/* End of code generation (Nonlinear_controller_fcn_control_rf_aero_models.c) */
