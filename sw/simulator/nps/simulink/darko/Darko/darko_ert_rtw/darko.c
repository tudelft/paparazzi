/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: darko.c
 *
 * Code generated for Simulink model 'darko'.
 *
 * Model version                  : 2.24
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Fri Mar 31 10:46:22 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "darko.h"
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include <emmintrin.h>
#include <stddef.h>
#define NumBitsPerChar                 8U

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Forward declaration for local functions */
static real_T norm(const real_T x[3]);
static void mldivide(const real_T A[9], const real_T B_0[3], real_T Y[3]);

#define NOT_USING_NONFINITE_LITERALS   1

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T norm(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void mldivide(const real_T A[9], const real_T B_0[3], real_T Y[3])
{
  real_T b_A[9];
  real_T a21;
  real_T maxval;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  memcpy(&b_A[0], &A[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(A[0]);
  a21 = fabs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r1 + 3] * b_A[r2];
  b_A[r3 + 3] -= b_A[r1 + 3] * b_A[r3];
  b_A[r2 + 6] -= b_A[r1 + 6] * b_A[r2];
  b_A[r3 + 6] -= b_A[r1 + 6] * b_A[r3];
  if (fabs(b_A[r3 + 3]) > fabs(b_A[r2 + 3])) {
    int32_T rtemp;
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  Y[1] = B_0[r2] - B_0[r1] * b_A[r2];
  Y[2] = (B_0[r3] - B_0[r1] * b_A[r3]) - b_A[r3 + 3] * Y[1];
  Y[2] /= b_A[r3 + 6];
  Y[0] = B_0[r1] - b_A[r1 + 6] * Y[2];
  Y[1] -= b_A[r2 + 6] * Y[2];
  Y[1] /= b_A[r2 + 3];
  Y[0] -= b_A[r1 + 3] * Y[1];
  Y[0] /= b_A[r1];
}

/* Model step function */
void darko_step(void)
{
  real_T B_1[9];
  real_T Rq[9];
  real_T dpdt[3];
  real_T dvdt[3];
  real_T p[3];
  real_T wb[3];
  real_T absxk;
  real_T n;
  real_T scale;
  real_T t;
  int32_T k;
  static const real_T f[9] = { 0.0, 0.0, -0.021887388060695588, 0.0, 0.0, 0.0,
    0.00066629525077479006, 0.0, 0.0 };

  static const real_T g_a[9] = { 0.007018, 0.0, 0.0, 0.0, 0.002785, 0.0, 0.0,
    0.0, 0.00606 };

  static const real_T h_a[12] = { 0.009805652452480957, 0.0, -0.1685451207947917,
    -0.009805652452480957, 0.0, 0.1685451207947917, -0.21862117501751782,
    -0.06635321469639531, 0.0, 0.21862117501751782, -0.06635321469639531, 0.0 };

  __m128d tmp_5;
  __m128d tmp_6;
  __m128d tmp_7;
  __m128d tmp_8;
  __m128d tmp_9;
  __m128d tmp_a;
  __m128d tmp_b;
  __m128d tmp_c;
  real_T tmp[13];
  real_T tmp_0[12];
  real_T tmp_1[12];
  real_T B_2[9];
  real_T g[9];
  real_T rtb_TmpSignalConversionAtSFunct[4];
  real_T d[3];
  real_T h_a_0[3];
  real_T n_0[3];
  real_T tmp_2[3];
  real_T tmp_3[3];
  real_T rtb_Saturation2;
  real_T rtb_u_idx_0;
  real_T rtb_u_idx_1;
  real_T rtb_u_idx_2;
  real_T rtb_u_idx_3;
  real_T scale_tmp;
  real_T scale_tmp_0;
  real_T scale_tmp_1;
  real_T scale_tmp_2;
  real_T scale_tmp_3;
  real_T scale_tmp_4;
  real_T tmp_4;
  int32_T d_tmp;
  int32_T i;
  int8_T Rq_tmp[9];
  static const int8_T e_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const real_T f_b[12] = { 0.91261212390456969, 0.0, 0.0,
    0.91261212390456969, 0.0, 0.0, 0.0, 0.0, -1.4104591936614053, 0.0, 0.0,
    -1.4104591936614053 };

  static const real_T e[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    0.0099372556331065141, 0.0 };

  static const real_T d_0[9] = { -0.0027121555370000004, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.089092636710610115 };

  static const real_T g_0[9] = { 0.0, -0.0024412855913852772, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0 };

  static const real_T f_0[9] = { 0.0, 0.0, -0.021887388060695588, 0.0, 0.0, 0.0,
    0.00066629525077479006, 0.0, 0.0 };

  static const real_T c[3] = { 0.0, 0.0, 9.81 };

  static const real_T g_a_0[9] = { 0.007018, 0.0, 0.0, 0.0, 0.002785, 0.0, 0.0,
    0.0, 0.00606 };

  static const real_T h[9] = { -0.007018, -0.0, -0.0, -0.0, -0.002785, -0.0,
    -0.0, -0.0, -0.00606 };

  static const real_T h_a_1[12] = { 0.009805652452480957, 0.0,
    -0.1685451207947917, -0.009805652452480957, 0.0, 0.1685451207947917,
    -0.21862117501751782, -0.06635321469639531, 0.0, 0.21862117501751782,
    -0.06635321469639531, 0.0 };

  static const real_T i_0[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    0.60231997436745077, 0.0 };

  static const real_T l[9] = { 0.1396, 0.0, 0.0405, 0.0, 0.63575, 0.0, 0.05725,
    0.0, 0.00195 };

  static const real_T n_1[9] = { -0.0033925451494078163, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.00010327576387009247 };

  static const real_T m[9] = { 0.0033925451494078163, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.00010327576387009247 };

  static const real_T o[9] = { 0.0, 0.0, -1.5941, -0.0, 0.0, 0.0, 1.5941, -0.0,
    0.0 };

  /* Outport: '<Root>/p' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.p[0] = rtDW.DiscreteTimeIntegrator_DSTATE[0];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.v[0] = rtDW.DiscreteTimeIntegrator_DSTATE[3];

  /* Outport: '<Root>/p' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.p[1] = rtDW.DiscreteTimeIntegrator_DSTATE[1];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.v[1] = rtDW.DiscreteTimeIntegrator_DSTATE[4];

  /* Outport: '<Root>/p' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.p[2] = rtDW.DiscreteTimeIntegrator_DSTATE[2];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.v[2] = rtDW.DiscreteTimeIntegrator_DSTATE[5];

  /* Outport: '<Root>/q' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.q[0] = rtDW.DiscreteTimeIntegrator_DSTATE[6];
  rtY.q[1] = rtDW.DiscreteTimeIntegrator_DSTATE[7];
  rtY.q[2] = rtDW.DiscreteTimeIntegrator_DSTATE[8];
  rtY.q[3] = rtDW.DiscreteTimeIntegrator_DSTATE[9];

  /* Outport: '<Root>/omega' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.omega[0] = rtDW.DiscreteTimeIntegrator_DSTATE[10];
  rtY.omega[1] = rtDW.DiscreteTimeIntegrator_DSTATE[11];
  rtY.omega[2] = rtDW.DiscreteTimeIntegrator_DSTATE[12];

  /* MATLAB Function: '<Root>/MATLAB Function2' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  scale = 3.3121686421112381E-170;
  absxk = fabs(rtDW.DiscreteTimeIntegrator_DSTATE[6]);
  if (absxk > 3.3121686421112381E-170) {
    n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n = t * t;
  }

  absxk = fabs(rtDW.DiscreteTimeIntegrator_DSTATE[7]);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }

  absxk = fabs(rtDW.DiscreteTimeIntegrator_DSTATE[8]);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }

  absxk = fabs(rtDW.DiscreteTimeIntegrator_DSTATE[9]);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }

  n = scale * sqrt(n);

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/u'
   */
  rtb_u_idx_3 = rtU.u[2] * 15769.0;
  if (rtb_u_idx_3 < 10.0) {
    rtb_u_idx_2 = 10.0;
  } else if (rtIsNaN(rtb_u_idx_3)) {
    rtb_u_idx_2 = 10.0;
  } else {
    rtb_u_idx_2 = rtb_u_idx_3;
  }

  rtb_u_idx_3 = rtU.u[3] * 15769.0;
  rtb_u_idx_0 = rtU.u[0] * 0.52359877559829882;
  rtb_u_idx_1 = rtU.u[1] * 0.52359877559829882;
  if (rtb_u_idx_3 < 10.0) {
    rtb_u_idx_3 = 10.0;
  } else if (rtIsNaN(rtb_u_idx_3)) {
    rtb_u_idx_3 = 10.0;
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */

  /* Saturate: '<S4>/Saturation3' */
  if (rtb_u_idx_0 > 0.52359877559829882) {
    rtb_Saturation2 = 0.52359877559829882;
  } else if (rtb_u_idx_0 < -0.52359877559829882) {
    rtb_Saturation2 = -0.52359877559829882;
  } else {
    rtb_Saturation2 = rtb_u_idx_0;
  }

  /* End of Saturate: '<S4>/Saturation3' */

  /* RateLimiter: '<S4>/Rate Limiter' */
  rtb_u_idx_0 = rtb_Saturation2 - rtDW.PrevY;
  if (rtb_u_idx_0 > 0.00511556003759538) {
    scale = rtDW.PrevY + 0.00511556003759538;
  } else if (rtb_u_idx_0 < -0.00511556003759538) {
    scale = rtDW.PrevY - 0.00511556003759538;
  } else {
    scale = rtb_Saturation2;
  }

  rtDW.PrevY = scale;

  /* End of RateLimiter: '<S4>/Rate Limiter' */

  /* Saturate: '<S4>/Saturation1' */
  if (rtb_u_idx_1 > 0.52359877559829882) {
    rtb_Saturation2 = 0.52359877559829882;
  } else if (rtb_u_idx_1 < -0.52359877559829882) {
    rtb_Saturation2 = -0.52359877559829882;
  } else {
    rtb_Saturation2 = rtb_u_idx_1;
  }

  /* End of Saturate: '<S4>/Saturation1' */

  /* RateLimiter: '<S4>/Rate Limiter1' */
  rtb_u_idx_0 = rtb_Saturation2 - rtDW.PrevY_f;
  if (rtb_u_idx_0 > 0.00511556003759538) {
    absxk = rtDW.PrevY_f + 0.00511556003759538;
  } else if (rtb_u_idx_0 < -0.00511556003759538) {
    absxk = rtDW.PrevY_f - 0.00511556003759538;
  } else {
    absxk = rtb_Saturation2;
  }

  rtDW.PrevY_f = absxk;

  /* End of RateLimiter: '<S4>/Rate Limiter1' */

  /* Saturate: '<S4>/Saturation4' */
  if (rtb_u_idx_2 > 15769.0) {
    rtb_Saturation2 = 15769.0;
  } else {
    rtb_Saturation2 = rtb_u_idx_2;
  }

  /* End of Saturate: '<S4>/Saturation4' */

  /* RateLimiter: '<S4>/Rate Limiter2' */
  rtb_u_idx_0 = rtb_Saturation2 - rtDW.PrevY_a;
  if (rtb_u_idx_0 > 307.755) {
    t = rtDW.PrevY_a + 307.755;
  } else if (rtb_u_idx_0 < -307.755) {
    t = rtDW.PrevY_a - 307.755;
  } else {
    t = rtb_Saturation2;
  }

  rtDW.PrevY_a = t;

  /* End of RateLimiter: '<S4>/Rate Limiter2' */

  /* Saturate: '<S4>/Saturation2' */
  if (rtb_u_idx_3 > 15769.0) {
    rtb_Saturation2 = 15769.0;
  } else {
    rtb_Saturation2 = rtb_u_idx_3;
  }

  /* End of Saturate: '<S4>/Saturation2' */

  /* RateLimiter: '<S4>/Rate Limiter3' */
  rtb_u_idx_0 = rtb_Saturation2 - rtDW.PrevY_d;
  if (rtb_u_idx_0 > 307.755) {
    rtb_Saturation2 = rtDW.PrevY_d + 307.755;
  } else if (rtb_u_idx_0 < -307.755) {
    rtb_Saturation2 = rtDW.PrevY_d - 307.755;
  }

  rtDW.PrevY_d = rtb_Saturation2;

  /* End of RateLimiter: '<S4>/Rate Limiter3' */

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  Inport: '<Root>/w'
   *  SignalConversion generated from: '<S1>/ SFunction '
   */
  rtb_u_idx_2 = t * t;
  rtb_u_idx_0 = rtb_u_idx_2 * 1.4047E-8;
  t = rtb_Saturation2 * rtb_Saturation2;
  rtb_u_idx_1 = t * 1.4047E-8;
  rtb_u_idx_2 *= -absxk * 1.4047E-8;
  rtb_u_idx_3 = scale * 1.4047E-8 * t;
  if (((rtb_u_idx_0 > 2.46654967216334) && (rtb_u_idx_1 > 2.46654967216334) &&
       (rtDW.DiscreteTimeIntegrator_DSTATE[2] == 0.0)) ||
      (rtDW.DiscreteTimeIntegrator_DSTATE[2] != 0.0)) {
    t = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6];
    rtb_Saturation2 = (rtDW.DiscreteTimeIntegrator_DSTATE[7] *
                       rtDW.DiscreteTimeIntegrator_DSTATE[7] +
                       rtDW.DiscreteTimeIntegrator_DSTATE[8] *
                       rtDW.DiscreteTimeIntegrator_DSTATE[8]) +
      rtDW.DiscreteTimeIntegrator_DSTATE[9] *
      rtDW.DiscreteTimeIntegrator_DSTATE[9];
    memset(&B_1[0], 0, 9U * sizeof(real_T));
    B_1[0] = 1.0;
    B_1[4] = 1.0;
    B_1[8] = 1.0;
    for (k = 0; k < 9; k++) {
      Rq_tmp[k] = e_b[k];
    }

    dpdt[0] = t * 0.0;
    scale_tmp_1 = t * -rtDW.DiscreteTimeIntegrator_DSTATE[9];
    dpdt[1] = scale_tmp_1;
    scale_tmp_3 = t * rtDW.DiscreteTimeIntegrator_DSTATE[8];
    dpdt[2] = scale_tmp_3;
    scale_tmp = t * rtDW.DiscreteTimeIntegrator_DSTATE[9];
    wb[0] = scale_tmp;
    wb[1] = t * 0.0;
    scale_tmp_4 = t * -rtDW.DiscreteTimeIntegrator_DSTATE[7];
    wb[2] = scale_tmp_4;
    scale_tmp_0 = t * -rtDW.DiscreteTimeIntegrator_DSTATE[8];
    dvdt[0] = scale_tmp_0;
    scale_tmp_2 = t * rtDW.DiscreteTimeIntegrator_DSTATE[7];
    dvdt[1] = scale_tmp_2;
    dvdt[2] = t * 0.0;
    for (k = 0; k < 3; k++) {
      tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[k + 7];
      g[3 * k] = tmp_4 * rtDW.DiscreteTimeIntegrator_DSTATE[7] - (real_T)Rq_tmp
        [3 * k] * rtb_Saturation2;
      i = 3 * k + 1;
      g[i] = tmp_4 * rtDW.DiscreteTimeIntegrator_DSTATE[8] - (real_T)Rq_tmp[i] *
        rtb_Saturation2;
      d_tmp = 3 * k + 2;
      g[d_tmp] = tmp_4 * rtDW.DiscreteTimeIntegrator_DSTATE[9] - (real_T)
        Rq_tmp[d_tmp] * rtb_Saturation2;
      B_2[3 * k] = B_1[3 * k] + dpdt[k];
      B_2[i] = B_1[i] + wb[k];
      B_2[d_tmp] = B_1[d_tmp] + dvdt[k];
    }

    scale = rtb_u_idx_2 / rtb_u_idx_0;
    absxk = rtb_u_idx_3 / rtb_u_idx_1;
    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&g[k]);
      tmp_c = _mm_loadu_pd(&B_2[k]);
      _mm_storeu_pd(&Rq[k], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(2.0), tmp_b),
        tmp_c));
      _mm_storeu_pd(&B_1[k], _mm_set1_pd(0.0));
    }

    for (k = 8; k < 9; k++) {
      Rq[k] = 2.0 * g[k] + B_2[k];
      B_1[k] = 0.0;
    }

    for (k = 0; k < 3; k++) {
      B_1[k + 3 * k] = 1.0;
      tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[k + 7];
      g[3 * k] = tmp_4 * rtDW.DiscreteTimeIntegrator_DSTATE[7] - rtb_Saturation2
        * (real_T)Rq_tmp[k];
      g[3 * k + 1] = tmp_4 * rtDW.DiscreteTimeIntegrator_DSTATE[8] - (real_T)
        Rq_tmp[k + 3] * rtb_Saturation2;
      g[3 * k + 2] = tmp_4 * rtDW.DiscreteTimeIntegrator_DSTATE[9] - (real_T)
        Rq_tmp[k + 6] * rtb_Saturation2;
    }

    B_2[0] = t * 0.0 + B_1[0];
    B_2[3] = scale_tmp + B_1[1];
    B_2[6] = scale_tmp_0 + B_1[2];
    B_2[1] = scale_tmp_1 + B_1[3];
    B_2[4] = t * 0.0 + B_1[4];
    B_2[7] = scale_tmp_2 + B_1[5];
    B_2[2] = scale_tmp_3 + B_1[6];
    B_2[5] = scale_tmp_4 + B_1[7];
    B_2[8] = t * 0.0 + B_1[8];
    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&g[k]);
      tmp_c = _mm_loadu_pd(&B_2[k]);
      _mm_storeu_pd(&B_1[k], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(2.0), tmp_b),
        tmp_c));
    }

    for (k = 8; k < 9; k++) {
      B_1[k] = 2.0 * g[k] + B_2[k];
    }

    t = rtDW.DiscreteTimeIntegrator_DSTATE[3] - rtU.w[0];
    scale_tmp_1 = rtDW.DiscreteTimeIntegrator_DSTATE[4] - rtU.w[1];
    tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[5] - rtU.w[2];
    for (k = 0; k <= 0; k += 2) {
      tmp_b = _mm_loadu_pd(&B_1[k + 3]);
      tmp_c = _mm_loadu_pd(&B_1[k]);
      tmp_a = _mm_loadu_pd(&B_1[k + 6]);
      _mm_storeu_pd(&wb[k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_b, _mm_set1_pd
        (scale_tmp_1)), _mm_mul_pd(tmp_c, _mm_set1_pd(t))), _mm_mul_pd(tmp_a,
        _mm_set1_pd(tmp_4))));
    }

    for (k = 2; k < 3; k++) {
      wb[k] = (B_1[k + 3] * scale_tmp_1 + B_1[k] * t) + B_1[k + 6] * tmp_4;
    }

    memset(&B_1[0], 0, 9U * sizeof(real_T));
    B_1[0] = 0.542;
    B_1[4] = 0.13;
    B_1[8] = 0.542;
    t = norm(wb);
    scale_tmp_1 = scale + absxk;
    for (k = 0; k <= 0; k += 2) {
      tmp_b = _mm_loadu_pd(&B_1[k + 3]);
      tmp_c = _mm_loadu_pd(&B_1[k]);
      tmp_a = _mm_loadu_pd(&B_1[k + 6]);
      _mm_storeu_pd(&dvdt[k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_b,
        _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[11])), _mm_mul_pd(tmp_c,
        _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[10]))), _mm_mul_pd(tmp_a,
        _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[12]))));
      tmp_b = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[k + 3]);
      _mm_storeu_pd(&dpdt[k], tmp_b);
    }

    for (k = 2; k < 3; k++) {
      dvdt[k] = (B_1[k + 3] * rtDW.DiscreteTimeIntegrator_DSTATE[11] + B_1[k] *
                 rtDW.DiscreteTimeIntegrator_DSTATE[10]) + B_1[k + 6] *
        rtDW.DiscreteTimeIntegrator_DSTATE[12];
      dpdt[k] = rtDW.DiscreteTimeIntegrator_DSTATE[k + 3];
    }

    rtb_Saturation2 = norm(dvdt);
    t = sqrt(rtb_Saturation2 * rtb_Saturation2 * 0.0 + t * t);
    for (k = 0; k < 3; k++) {
      tmp_4 = Rq[k + 3];
      rtb_Saturation2 = Rq[k];
      scale_tmp_3 = Rq[k + 6];
      for (i = 0; i < 4; i++) {
        tmp_0[k + 3 * i] = (f_b[3 * i + 1] * (tmp_4 * 2.0325203252032522) +
                            2.0325203252032522 * rtb_Saturation2 * f_b[3 * i]) +
          f_b[3 * i + 2] * (scale_tmp_3 * 2.0325203252032522);
      }

      rtb_Saturation2 = 0.0;
      scale_tmp_3 = e[k + 3];
      scale_tmp = e[k + 6];
      scale_tmp_4 = 0.0;
      for (i = 0; i < 3; i++) {
        d_tmp = 3 * i + k;
        rtb_Saturation2 += d_0[d_tmp] * t * wb[i];
        tmp_4 = B_1[3 * i];
        scale_tmp_0 = 0.0 * t * tmp_4;
        scale_tmp_2 = g_0[k] * scale_tmp_1 * tmp_4;
        tmp_4 = B_1[3 * i + 1];
        scale_tmp_0 += scale_tmp_3 * t * tmp_4;
        scale_tmp_2 += 0.0 * scale_tmp_1 * tmp_4;
        tmp_4 = B_1[3 * i + 2];
        g[d_tmp] = 0.0 * scale_tmp_1 * tmp_4 + scale_tmp_2;
        scale_tmp_4 += (scale_tmp * t * tmp_4 + scale_tmp_0) *
          rtDW.DiscreteTimeIntegrator_DSTATE[i + 10];
      }

      p[k] = scale_tmp_4;
      dvdt[k] = rtb_Saturation2;
    }

    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&g[k]);
      _mm_storeu_pd(&B_2[k], _mm_mul_pd(tmp_b, _mm_set1_pd(t)));
    }

    for (k = 8; k < 9; k++) {
      B_2[k] = g[k] * t;
    }

    tmp_4 = wb[0];
    rtb_Saturation2 = wb[1];
    scale_tmp_3 = wb[2];

    /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
     *  Inport: '<Root>/w'
     */
    scale_tmp = rtDW.DiscreteTimeIntegrator_DSTATE[11];
    scale_tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[10];
    scale_tmp_0 = rtDW.DiscreteTimeIntegrator_DSTATE[12];
    for (k = 0; k <= 0; k += 2) {
      tmp_b = _mm_set1_pd(scale_tmp_1);
      tmp_c = _mm_set1_pd(t);
      tmp_a = _mm_loadu_pd(&dvdt[k]);
      tmp_6 = _mm_loadu_pd(&p[k]);
      tmp_7 = _mm_loadu_pd(&B_2[k + 3]);
      tmp_8 = _mm_loadu_pd(&B_2[k]);
      tmp_9 = _mm_loadu_pd(&B_2[k + 6]);
      _mm_storeu_pd(&d[k], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_mul_pd(_mm_mul_pd(_mm_loadu_pd(&f[k]), tmp_b), tmp_c),
                    _mm_set1_pd(tmp_4)), _mm_set1_pd(0.0 * scale_tmp_1 * t *
        rtb_Saturation2)), _mm_mul_pd(_mm_mul_pd(_mm_mul_pd(_mm_loadu_pd(&f[k +
        6]), tmp_b), tmp_c), _mm_set1_pd(scale_tmp_3))), _mm_sub_pd(tmp_a, tmp_6)),
        _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_7, _mm_set1_pd(scale_tmp)),
        _mm_mul_pd(tmp_8, _mm_set1_pd(scale_tmp_4))), _mm_mul_pd(tmp_9,
        _mm_set1_pd(scale_tmp_0)))));
    }

    for (k = 2; k < 3; k++) {
      d[k] = (((f_0[k] * scale_tmp_1 * t * tmp_4 + 0.0 * scale_tmp_1 * t *
                rtb_Saturation2) + f_0[k + 6] * scale_tmp_1 * t * scale_tmp_3) +
              (dvdt[k] - p[k])) + ((B_2[k + 3] * scale_tmp + B_2[k] *
        scale_tmp_4) + B_2[k + 6] * scale_tmp_0);
    }

    tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[6];
    g[0] = 0.0;
    g[3] = -rtDW.DiscreteTimeIntegrator_DSTATE[9];
    g[6] = rtDW.DiscreteTimeIntegrator_DSTATE[8];
    g[1] = rtDW.DiscreteTimeIntegrator_DSTATE[9];
    g[4] = 0.0;
    g[7] = -rtDW.DiscreteTimeIntegrator_DSTATE[7];
    g[2] = -rtDW.DiscreteTimeIntegrator_DSTATE[8];
    g[5] = rtDW.DiscreteTimeIntegrator_DSTATE[7];
    g[8] = 0.0;
    rtb_Saturation2 = d[1];
    scale_tmp_3 = d[0];
    scale_tmp = d[2];
    for (k = 0; k < 3; k++) {
      i = k << 2;
      tmp_1[i] = -rtDW.DiscreteTimeIntegrator_DSTATE[k + 7] * 0.5;
      tmp_1[i + 1] = ((real_T)Rq_tmp[3 * k] * tmp_4 + g[3 * k]) * 0.5;
      d_tmp = 3 * k + 1;
      tmp_1[i + 2] = ((real_T)Rq_tmp[d_tmp] * tmp_4 + g[d_tmp]) * 0.5;
      d_tmp = 3 * k + 2;
      tmp_1[i + 3] = ((real_T)Rq_tmp[d_tmp] * tmp_4 + g[d_tmp]) * 0.5;
      dvdt[k] = ((((tmp_0[k + 3] * rtb_u_idx_1 + tmp_0[k] * rtb_u_idx_0) +
                   tmp_0[k + 6] * rtb_u_idx_2) + tmp_0[k + 9] * rtb_u_idx_3) +
                 c[k]) + ((Rq[k + 3] * 2.0325203252032522 * rtb_Saturation2 +
                           2.0325203252032522 * Rq[k] * scale_tmp_3) + Rq[k + 6]
                          * 2.0325203252032522 * scale_tmp);
    }

    /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
    scale_tmp = rtDW.DiscreteTimeIntegrator_DSTATE[11];
    scale_tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[10];
    scale_tmp_0 = rtDW.DiscreteTimeIntegrator_DSTATE[12];
    for (k = 0; k <= 2; k += 2) {
      tmp_b = _mm_loadu_pd(&tmp_1[k + 4]);
      tmp_c = _mm_loadu_pd(&tmp_1[k]);
      tmp_a = _mm_loadu_pd(&tmp_1[k + 8]);
      _mm_storeu_pd(&rtb_TmpSignalConversionAtSFunct[k], _mm_add_pd(_mm_add_pd
        (_mm_mul_pd(tmp_b, _mm_set1_pd(scale_tmp)), _mm_mul_pd(tmp_c,
        _mm_set1_pd(scale_tmp_4))), _mm_mul_pd(tmp_a, _mm_set1_pd(scale_tmp_0))));
    }

    /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
    scale_tmp = rtDW.DiscreteTimeIntegrator_DSTATE[11];
    scale_tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[10];
    scale_tmp_0 = rtDW.DiscreteTimeIntegrator_DSTATE[12];
    for (k = 0; k <= 0; k += 2) {
      _mm_storeu_pd(&p[k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&g_a[k
        + 3]), _mm_set1_pd(scale_tmp)), _mm_mul_pd(_mm_loadu_pd(&g_a[k]),
        _mm_set1_pd(scale_tmp_4))), _mm_mul_pd(_mm_loadu_pd(&g_a[k + 6]),
        _mm_set1_pd(scale_tmp_0))));
    }

    for (k = 2; k < 3; k++) {
      p[k] = (g_a_0[k + 3] * scale_tmp + g_a_0[k] * scale_tmp_4) + g_a_0[k + 6] *
        scale_tmp_0;
    }

    d[0] = p[2] * rtDW.DiscreteTimeIntegrator_DSTATE[11] - p[1] *
      rtDW.DiscreteTimeIntegrator_DSTATE[12];
    d[1] = p[0] * rtDW.DiscreteTimeIntegrator_DSTATE[12] - p[2] *
      rtDW.DiscreteTimeIntegrator_DSTATE[10];
    d[2] = p[1] * rtDW.DiscreteTimeIntegrator_DSTATE[10] - p[0] *
      rtDW.DiscreteTimeIntegrator_DSTATE[11];
    mldivide(h, d, p);
    for (k = 0; k <= 0; k += 2) {
      _mm_storeu_pd(&h_a_0[k], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_loadu_pd(&h_a[k + 3]), _mm_set1_pd(rtb_u_idx_1)), _mm_mul_pd
        (_mm_loadu_pd(&h_a[k]), _mm_set1_pd(rtb_u_idx_0))), _mm_mul_pd
        (_mm_loadu_pd(&h_a[k + 6]), _mm_set1_pd(rtb_u_idx_2))), _mm_mul_pd
        (_mm_loadu_pd(&h_a[k + 9]), _mm_set1_pd(rtb_u_idx_3))));
    }

    for (k = 2; k < 3; k++) {
      h_a_0[k] = ((h_a_1[k + 3] * rtb_u_idx_1 + h_a_1[k] * rtb_u_idx_0) +
                  h_a_1[k + 6] * rtb_u_idx_2) + h_a_1[k + 9] * rtb_u_idx_3;
    }

    mldivide(g_a_0, h_a_0, d);
    for (k = 0; k < 3; k++) {
      rtb_u_idx_1 = i_0[3 * k + 1];
      rtb_u_idx_2 = i_0[3 * k];
      rtb_u_idx_0 = i_0[3 * k + 2];
      for (i = 0; i <= 0; i += 2) {
        tmp_b = _mm_loadu_pd(&B_1[i + 3]);
        tmp_c = _mm_set1_pd(-0.0164983);
        tmp_a = _mm_loadu_pd(&B_1[i]);
        tmp_6 = _mm_loadu_pd(&B_1[i + 6]);
        _mm_storeu_pd(&g[i + 3 * k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_mul_pd
          (tmp_b, tmp_c), _mm_set1_pd(rtb_u_idx_1)), _mm_mul_pd(_mm_mul_pd(tmp_c,
          tmp_a), _mm_set1_pd(rtb_u_idx_2))), _mm_mul_pd(_mm_mul_pd(tmp_6, tmp_c),
          _mm_set1_pd(rtb_u_idx_0))));
      }

      for (i = 2; i < 3; i++) {
        g[i + 3 * k] = (B_1[i + 3] * -0.0164983 * rtb_u_idx_1 + -0.0164983 *
                        B_1[i] * rtb_u_idx_2) + B_1[i + 6] * -0.0164983 *
          rtb_u_idx_0;
      }
    }

    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&g[k]);
      _mm_storeu_pd(&Rq[k], _mm_mul_pd(tmp_b, _mm_set1_pd(t)));
    }

    for (k = 8; k < 9; k++) {
      Rq[k] = g[k] * t;
    }

    for (k = 0; k < 3; k++) {
      rtb_u_idx_1 = l[3 * k + 1];
      rtb_u_idx_2 = l[3 * k];
      rtb_u_idx_0 = l[3 * k + 2];
      for (i = 0; i <= 0; i += 2) {
        tmp_b = _mm_loadu_pd(&B_1[i + 3]);
        tmp_c = _mm_set1_pd(0.0164983);
        tmp_a = _mm_loadu_pd(&B_1[i]);
        tmp_6 = _mm_loadu_pd(&B_1[i + 6]);
        _mm_storeu_pd(&g[i + 3 * k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_mul_pd
          (tmp_b, tmp_c), _mm_set1_pd(rtb_u_idx_1)), _mm_mul_pd(_mm_mul_pd(tmp_c,
          tmp_a), _mm_set1_pd(rtb_u_idx_2))), _mm_mul_pd(_mm_mul_pd(tmp_6, tmp_c),
          _mm_set1_pd(rtb_u_idx_0))));
      }

      for (i = 2; i < 3; i++) {
        g[i + 3 * k] = (B_1[i + 3] * 0.0164983 * rtb_u_idx_1 + 0.0164983 * B_1[i]
                        * rtb_u_idx_2) + B_1[i + 6] * 0.0164983 * rtb_u_idx_0;
      }
    }

    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&g[k]);
      _mm_storeu_pd(&B_2[k], _mm_mul_pd(tmp_b, _mm_set1_pd(t)));
    }

    for (k = 8; k < 9; k++) {
      B_2[k] = g[k] * t;
    }

    for (k = 0; k < 3; k++) {
      n_0[k] = (n_1[k] * absxk * t * wb[0] + 0.0 * absxk * t * wb[1]) + n_1[k +
        6] * absxk * t * wb[2];
      tmp_4 = 0.0 * scale * t;
      h_a_0[k] = (((Rq[k + 3] * wb[1] + Rq[k] * wb[0]) + Rq[k + 6] * wb[2]) -
                  ((B_2[k + 3] * rtDW.DiscreteTimeIntegrator_DSTATE[11] + B_2[k]
                    * rtDW.DiscreteTimeIntegrator_DSTATE[10]) + B_2[k + 6] *
                   rtDW.DiscreteTimeIntegrator_DSTATE[12])) + ((m[k] * scale * t
        * wb[0] + tmp_4 * wb[1]) + m[k + 6] * scale * t * wb[2]);
      for (i = 0; i < 3; i++) {
        g[k + 3 * i] = (B_1[3 * i + 1] * tmp_4 + B_1[3 * i] * tmp_4) + B_1[3 * i
          + 2] * tmp_4;
      }
    }

    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&g[k]);
      _mm_storeu_pd(&Rq[k], _mm_mul_pd(tmp_b, _mm_set1_pd(t)));
    }

    for (k = 8; k < 9; k++) {
      Rq[k] = g[k] * t;
    }

    for (k = 0; k < 3; k++) {
      for (i = 0; i < 3; i++) {
        tmp_4 = 0.0 * absxk * t;
        g[k + 3 * i] = (B_1[3 * i + 1] * tmp_4 + B_1[3 * i] * tmp_4) + B_1[3 * i
          + 2] * tmp_4;
      }
    }

    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&g[k]);
      _mm_storeu_pd(&B_2[k], _mm_mul_pd(tmp_b, _mm_set1_pd(t)));
    }

    for (k = 8; k < 9; k++) {
      B_2[k] = g[k] * t;
    }

    /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
    scale_tmp = rtDW.DiscreteTimeIntegrator_DSTATE[11];
    scale_tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[10];
    scale_tmp_0 = rtDW.DiscreteTimeIntegrator_DSTATE[12];
    for (k = 0; k <= 0; k += 2) {
      tmp_b = _mm_loadu_pd(&B_2[k + 3]);
      tmp_c = _mm_set1_pd(scale_tmp);
      tmp_a = _mm_loadu_pd(&B_2[k]);
      tmp_6 = _mm_set1_pd(scale_tmp_4);
      tmp_7 = _mm_loadu_pd(&B_2[k + 6]);
      tmp_8 = _mm_set1_pd(scale_tmp_0);
      _mm_storeu_pd(&tmp_3[k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_b, tmp_c),
        _mm_mul_pd(tmp_a, tmp_6)), _mm_mul_pd(tmp_7, tmp_8)));
      tmp_b = _mm_loadu_pd(&Rq[k + 3]);
      tmp_a = _mm_loadu_pd(&Rq[k]);
      tmp_7 = _mm_loadu_pd(&Rq[k + 6]);
      tmp_9 = _mm_loadu_pd(&h_a_0[k]);
      tmp_5 = _mm_loadu_pd(&n_0[k]);
      _mm_storeu_pd(&tmp_2[k], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_b,
        tmp_c), _mm_mul_pd(tmp_a, tmp_6)), _mm_mul_pd(tmp_7, tmp_8)), _mm_add_pd
        (tmp_9, tmp_5)));
    }

    for (k = 2; k < 3; k++) {
      tmp_3[k] = (B_2[k + 3] * scale_tmp + B_2[k] * scale_tmp_4) + B_2[k + 6] *
        scale_tmp_0;
      tmp_2[k] = ((Rq[k + 3] * scale_tmp + Rq[k] * scale_tmp_4) + Rq[k + 6] *
                  scale_tmp_0) + (h_a_0[k] + n_0[k]);
    }

    for (k = 0; k < 3; k++) {
      tmp_4 = B_1[k + 3];
      scale = B_1[k];
      absxk = B_1[k + 6];
      for (i = 0; i < 3; i++) {
        g[k + 3 * i] = (i_0[3 * i + 1] * (tmp_4 * 0.00824915) + 0.00824915 *
                        scale * i_0[3 * i]) + i_0[3 * i + 2] * (absxk *
          0.00824915);
      }

      tmp_4 = g[k + 3];
      scale = g[k];
      absxk = g[k + 6];
      for (i = 0; i < 3; i++) {
        Rq[k + 3 * i] = (o[3 * i + 1] * tmp_4 + o[3 * i] * scale) + o[3 * i + 2]
          * absxk;
      }
    }

    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&Rq[k]);
      _mm_storeu_pd(&g[k], _mm_mul_pd(_mm_mul_pd(tmp_b, _mm_set1_pd(scale_tmp_1)),
        _mm_set1_pd(t)));
    }

    for (k = 8; k < 9; k++) {
      g[k] = Rq[k] * scale_tmp_1 * t;
    }

    for (k = 0; k < 3; k++) {
      tmp_4 = B_1[k + 3];
      scale = B_1[k];
      absxk = B_1[k + 6];
      for (i = 0; i < 3; i++) {
        Rq[k + 3 * i] = (l[3 * i + 1] * (tmp_4 * 0.00824915) + 0.00824915 *
                         scale * l[3 * i]) + l[3 * i + 2] * (absxk * 0.00824915);
      }

      tmp_4 = Rq[k + 3];
      scale = Rq[k];
      absxk = Rq[k + 6];
      for (i = 0; i < 3; i++) {
        B_2[k + 3 * i] = (o[3 * i + 1] * tmp_4 + o[3 * i] * scale) + o[3 * i + 2]
          * absxk;
      }
    }

    for (k = 0; k <= 6; k += 2) {
      tmp_b = _mm_loadu_pd(&B_2[k]);
      _mm_storeu_pd(&Rq[k], _mm_mul_pd(_mm_mul_pd(tmp_b, _mm_set1_pd(scale_tmp_1)),
        _mm_set1_pd(t)));
    }

    for (k = 8; k < 9; k++) {
      Rq[k] = B_2[k] * scale_tmp_1 * t;
    }

    for (k = 0; k < 3; k++) {
      tmp_4 = 0.0;
      scale = 0.0;
      absxk = Rq[k + 3];
      rtb_u_idx_1 = Rq[k];
      rtb_u_idx_2 = Rq[k + 6];
      for (i = 0; i < 3; i++) {
        tmp_4 += g[3 * i + k] * wb[i];
        scale += ((B_1[3 * i + 1] * absxk + B_1[3 * i] * rtb_u_idx_1) + B_1[3 *
                  i + 2] * rtb_u_idx_2) * rtDW.DiscreteTimeIntegrator_DSTATE[i +
          10];
      }

      h_a_0[k] = ((tmp_2[k] + tmp_3[k]) + tmp_4) + scale;
    }

    mldivide(g_a_0, h_a_0, tmp_2);
    wb[0] = (p[0] + d[0]) + tmp_2[0];
    wb[1] = (p[1] + d[1]) + tmp_2[1];
    wb[2] = (p[2] + d[2]) + tmp_2[2];
  } else {
    dpdt[0] = 0.0;
    dvdt[0] = 0.0;
    dpdt[1] = 0.0;
    dvdt[1] = 0.0;
    dpdt[2] = 0.0;
    dvdt[2] = 0.0;
    rtb_TmpSignalConversionAtSFunct[0] = 0.0;
    rtb_TmpSignalConversionAtSFunct[1] = 0.0;
    rtb_TmpSignalConversionAtSFunct[2] = 0.0;
    rtb_TmpSignalConversionAtSFunct[3] = 0.0;
    wb[0] = 0.0;
    wb[1] = 0.0;
    wb[2] = 0.0;
  }

  /* Outport: '<Root>/accel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.accel[0] = dvdt[0];

  /* Outport: '<Root>/rotaccel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.rotaccel[0] = wb[0];

  /* Outport: '<Root>/accel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.accel[1] = dvdt[1];

  /* Outport: '<Root>/rotaccel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.rotaccel[1] = wb[1];

  /* Outport: '<Root>/accel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.accel[2] = dvdt[2];

  /* Outport: '<Root>/rotaccel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.rotaccel[2] = wb[2];

  /* Sum: '<Root>/Sum5' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  MATLAB Function: '<Root>/MATLAB Function'
   *  MATLAB Function: '<Root>/MATLAB Function2'
   */
  tmp[0] = 0.000977 * dpdt[0] + rtDW.DiscreteTimeIntegrator_DSTATE[0];
  tmp[1] = 0.000977 * dpdt[1] + rtDW.DiscreteTimeIntegrator_DSTATE[1];
  tmp[2] = 0.000977 * dpdt[2] + rtDW.DiscreteTimeIntegrator_DSTATE[2];
  tmp[3] = 0.000977 * dvdt[0] + rtDW.DiscreteTimeIntegrator_DSTATE[3];
  tmp[4] = 0.000977 * dvdt[1] + rtDW.DiscreteTimeIntegrator_DSTATE[4];
  tmp[5] = 0.000977 * dvdt[2] + rtDW.DiscreteTimeIntegrator_DSTATE[5];
  tmp[6] = ((1.0 - n) * rtDW.DiscreteTimeIntegrator_DSTATE[6] +
            rtb_TmpSignalConversionAtSFunct[0]) * 0.000977 +
    rtDW.DiscreteTimeIntegrator_DSTATE[6];
  tmp[7] = ((1.0 - n) * rtDW.DiscreteTimeIntegrator_DSTATE[7] +
            rtb_TmpSignalConversionAtSFunct[1]) * 0.000977 +
    rtDW.DiscreteTimeIntegrator_DSTATE[7];
  tmp[8] = ((1.0 - n) * rtDW.DiscreteTimeIntegrator_DSTATE[8] +
            rtb_TmpSignalConversionAtSFunct[2]) * 0.000977 +
    rtDW.DiscreteTimeIntegrator_DSTATE[8];
  tmp[9] = ((1.0 - n) * rtDW.DiscreteTimeIntegrator_DSTATE[9] +
            rtb_TmpSignalConversionAtSFunct[3]) * 0.000977 +
    rtDW.DiscreteTimeIntegrator_DSTATE[9];
  tmp[10] = 0.000977 * wb[0] + rtDW.DiscreteTimeIntegrator_DSTATE[10];
  tmp[11] = 0.000977 * wb[1] + rtDW.DiscreteTimeIntegrator_DSTATE[11];
  tmp[12] = 0.000977 * wb[2] + rtDW.DiscreteTimeIntegrator_DSTATE[12];

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  memcpy(&rtDW.DiscreteTimeIntegrator_DSTATE[0], &tmp[0], 13U * sizeof(real_T));
}

/* Model initialize function */
void darko_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  memcpy(&rtDW.DiscreteTimeIntegrator_DSTATE[0],
         &rtConstP.DiscreteTimeIntegrator_IC[0], 13U * sizeof(real_T));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
