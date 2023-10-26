//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: CLIK.cpp
//
// Code generated for Simulink model 'CLIK'.
//
// Model version                  : 3.74
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Sat Oct 14 10:34:16 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "CLIK.h"
#include "rtwtypes.h"
#include <cmath>
#include <emmintrin.h>
#include <cstring>
#include <stddef.h>
#define NumBitsPerChar                 8U

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern "C"
{
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}

extern "C"
{
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetInf(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T inf{ 0.0 };

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

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T minf{ 0.0 };

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

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}

extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetNaN(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T nan{ 0.0 };

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

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF{ { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

extern "C"
{
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
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

  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    boolean_T result{ (boolean_T) 0 };

    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.fltVal = value;
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}

// Function for MATLAB Function: '<S1>/q_l_dot'
void CLIK::eye(real_T b_I[16])
{
  std::memset(&b_I[0], 0, sizeof(real_T) << 4U);
  b_I[0] = 1.0;
  b_I[5] = 1.0;
  b_I[10] = 1.0;
  b_I[15] = 1.0;
}

// Function for MATLAB Function: '<S1>/q_l_dot'
void CLIK::mrdiv(const real_T A[12], const real_T B_0[9], real_T Y[12])
{
  real_T b_A[9];
  real_T Y_tmp_1;
  real_T Y_tmp_2;
  real_T Y_tmp_3;
  real_T Y_tmp_4;
  real_T a21;
  real_T maxval;
  int32_T Y_tmp;
  int32_T Y_tmp_0;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
  std::memcpy(&b_A[0], &B_0[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(B_0[0]);
  a21 = std::abs(B_0[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(B_0[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = B_0[r2] / B_0[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r1 + 3] * b_A[r2];
  b_A[r3 + 3] -= b_A[r1 + 3] * b_A[r3];
  b_A[r2 + 6] -= b_A[r1 + 6] * b_A[r2];
  b_A[r3 + 6] -= b_A[r1 + 6] * b_A[r3];
  if (std::abs(b_A[r3 + 3]) > std::abs(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  rtemp = r1 << 2;
  Y[rtemp] = A[0] / b_A[r1];
  Y_tmp = r2 << 2;
  maxval = b_A[r1 + 3];
  Y[Y_tmp] = A[4] - Y[rtemp] * maxval;
  Y_tmp_0 = r3 << 2;
  a21 = b_A[r1 + 6];
  Y[Y_tmp_0] = A[8] - Y[rtemp] * a21;
  Y_tmp_1 = b_A[r2 + 3];
  Y[Y_tmp] /= Y_tmp_1;
  Y_tmp_2 = b_A[r2 + 6];
  Y[Y_tmp_0] -= Y[Y_tmp] * Y_tmp_2;
  Y_tmp_3 = b_A[r3 + 6];
  Y[Y_tmp_0] /= Y_tmp_3;
  Y_tmp_4 = b_A[r3 + 3];
  Y[Y_tmp] -= Y[Y_tmp_0] * Y_tmp_4;
  Y[rtemp] -= Y[Y_tmp_0] * b_A[r3];
  Y[rtemp] -= Y[Y_tmp] * b_A[r2];
  Y[rtemp + 1] = A[1] / b_A[r1];
  Y[Y_tmp + 1] = A[5] - Y[rtemp + 1] * maxval;
  Y[Y_tmp_0 + 1] = A[9] - Y[rtemp + 1] * a21;
  Y[Y_tmp + 1] /= Y_tmp_1;
  Y[Y_tmp_0 + 1] -= Y[Y_tmp + 1] * Y_tmp_2;
  Y[Y_tmp_0 + 1] /= Y_tmp_3;
  Y[Y_tmp + 1] -= Y[Y_tmp_0 + 1] * Y_tmp_4;
  Y[rtemp + 1] -= Y[Y_tmp_0 + 1] * b_A[r3];
  Y[rtemp + 1] -= Y[Y_tmp + 1] * b_A[r2];
  Y[rtemp + 2] = A[2] / b_A[r1];
  Y[Y_tmp + 2] = A[6] - Y[rtemp + 2] * maxval;
  Y[Y_tmp_0 + 2] = A[10] - Y[rtemp + 2] * a21;
  Y[Y_tmp + 2] /= Y_tmp_1;
  Y[Y_tmp_0 + 2] -= Y[Y_tmp + 2] * Y_tmp_2;
  Y[Y_tmp_0 + 2] /= Y_tmp_3;
  Y[Y_tmp + 2] -= Y[Y_tmp_0 + 2] * Y_tmp_4;
  Y[rtemp + 2] -= Y[Y_tmp_0 + 2] * b_A[r3];
  Y[rtemp + 2] -= Y[Y_tmp + 2] * b_A[r2];
  Y[rtemp + 3] = A[3] / b_A[r1];
  Y[Y_tmp + 3] = A[7] - Y[rtemp + 3] * maxval;
  Y[Y_tmp_0 + 3] = A[11] - Y[rtemp + 3] * a21;
  Y[Y_tmp + 3] /= Y_tmp_1;
  Y[Y_tmp_0 + 3] -= Y[Y_tmp + 3] * Y_tmp_2;
  Y[Y_tmp_0 + 3] /= Y_tmp_3;
  Y[Y_tmp + 3] -= Y[Y_tmp_0 + 3] * Y_tmp_4;
  Y[rtemp + 3] -= Y[Y_tmp_0 + 3] * b_A[r3];
  Y[rtemp + 3] -= Y[Y_tmp + 3] * b_A[r2];
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (std::isinf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

// Model step function
void CLIK::step()
{
  static const int8_T g[9]{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  real_T N_a[16];
  real_T N_a_0[16];
  real_T N_a_1[16];
  real_T N_a_2[16];
  real_T N_a_7[16];
  real_T N_b[16];
  real_T N_c[16];
  real_T N_d[16];
  real_T N_e[16];
  real_T N_e_tmp[16];
  real_T Je[12];
  real_T N_a_3[12];
  real_T N_e_tmp_0[12];
  real_T tmp[12];
  real_T N_e_tmp_1[9];
  real_T N_e_tmp_2[9];
  real_T J_a[4];
  real_T J_b[4];
  real_T J_c[4];
  real_T J_d[4];
  real_T J_l1M[4];
  real_T J_l1m[4];
  real_T J_l2M[4];
  real_T J_l2m[4];
  real_T J_l3M[4];
  real_T J_l3m[4];
  real_T J_l4M[4];
  real_T J_l4m[4];
  real_T N_a_4[4];
  real_T N_a_5[4];
  real_T N_a_6[4];
  real_T q_l_dot_opt[4];
  real_T rtb_q_l_dot[4];
  real_T rtb_q_r_dot[4];
  real_T rtb_step5[4];
  real_T rtb_Sum2[3];
  real_T rtb_Sum4[3];
  real_T rtb_uT[3];
  real_T Je_tmp;
  real_T Je_tmp_0;
  real_T Je_tmp_1;
  real_T Je_tmp_2;
  real_T N_a_tmp_0;
  real_T N_a_tmp_1;
  real_T N_a_tmp_2;
  real_T i;
  real_T j;
  real_T k;
  real_T l;
  real_T q_l_dot_opt_tmp;
  real_T q_l_dot_opt_tmp_0;
  real_T q_l_dot_opt_tmp_1;
  real_T q_l_dot_opt_tmp_2;
  real_T q_l_dot_opt_tmp_3;
  real_T q_l_dot_opt_tmp_4;
  real_T q_l_dot_opt_tmp_5;
  real_T q_l_dot_opt_tmp_6;
  real_T q_l_dot_opt_tmp_7;
  real_T q_l_dot_opt_tmp_8;
  real_T q_l_dot_opt_tmp_9;
  real_T q_l_dot_opt_tmp_tmp;
  real_T q_l_dot_opt_tmp_tmp_0;
  real_T q_l_dot_opt_tmp_tmp_1;
  real_T q_l_dot_opt_tmp_tmp_2;
  real_T q_l_dot_opt_tmp_tmp_3;
  real_T q_l_dot_opt_tmp_tmp_4;
  real_T q_l_dot_opt_tmp_tmp_5;
  real_T q_l_dot_opt_tmp_tmp_6;
  real_T q_l_dot_opt_tmp_tmp_7;
  real_T q_l_dot_opt_tmp_tmp_tmp;
  real_T q_l_dot_opt_tmp_tmp_tmp_0;
  real_T rtb_Add_idx_0;
  real_T rtb_Add_idx_1;
  real_T rtb_Add_idx_2;
  real_T rtb_AvoidDividebyZero;
  real_T rtb_MinMax;
  real_T rtb_Sum4_tmp;
  real_T rtb_Sum4_tmp_0;
  real_T rtb_Sum4_tmp_1;
  real_T rtb_Sum4_tmp_tmp;
  real_T w_l1m;
  real_T w_l2m;
  real_T w_l3m;
  real_T w_l4m;
  int32_T N_a_tmp;
  int32_T N_a_tmp_tmp;
  int32_T N_b_tmp;
  int32_T act_l1M;
  int32_T act_l1m;
  int32_T act_l2M;
  int32_T act_l2m;
  int32_T act_l3M;
  int32_T act_l3m;
  int32_T act_l4M;
  int32_T act_l4m;
  int32_T converted;
  int32_T exitg1;
  int32_T i_0;
  int32_T tot_task;
  int8_T temp_act[128];

  // Outputs for Atomic SubSystem: '<Root>/CLIK'
  // MinMax: '<S13>/MinMax' incorporates:
  //   Constant: '<S13>/Time constant'
  //   Gain: '<S13>/Minimum sampling to time constant ratio'

  rtb_MinMax = std::fmax(10.0 * rtDW.Probe[0], 0.1);

  // Sum: '<S1>/Add' incorporates:
  //   Inport: '<Root>/L_half'
  //   Inport: '<Root>/xd'

  rtb_Add_idx_0 = 0.0 + rtU.xd[0];
  rtb_Add_idx_1 = rtU.L_half + rtU.xd[1];
  rtb_Add_idx_2 = 0.0 + rtU.xd[2];

  // DiscreteIntegrator: '<S17>/Integrator'
  if (rtDW.Integrator_IC_LOADING != 0) {
    rtDW.Integrator_DSTATE[0] = rtb_Add_idx_0;
    rtDW.Integrator_DSTATE[1] = rtb_Add_idx_1;
    rtDW.Integrator_DSTATE[2] = rtb_Add_idx_2;
  }

  if (rtDW.Integrator_PrevResetState != 0) {
    rtDW.Integrator_DSTATE[0] = rtb_Add_idx_0;
    rtDW.Integrator_DSTATE[1] = rtb_Add_idx_1;
    rtDW.Integrator_DSTATE[2] = rtb_Add_idx_2;
  }

  // MATLAB Function: '<S1>/dirkin_l' incorporates:
  //   Inport: '<Root>/q1l'
  //   Inport: '<Root>/q2l'
  //   Inport: '<Root>/q3l'
  //   Inport: '<Root>/q4l'
  //   MATLAB Function: '<S1>/q_l_dot'

  i = std::sin(rtU.q1l);
  j = std::cos(rtU.q2l);
  k = std::sin(rtU.q2l);
  l = std::cos(rtU.q4l);
  rtb_Sum4_tmp = std::sin(rtU.q3l);
  rtb_Sum4_tmp_tmp = std::sin(rtU.q4l);
  rtb_Sum4_tmp_0 = std::cos(rtU.q3l);
  rtb_Sum4_tmp_1 = std::cos(rtU.q1l);

  // Product: '<S3>/1//T' incorporates:
  //   DiscreteIntegrator: '<S17>/Integrator'
  //   Fcn: '<S13>/Avoid Divide by Zero'
  //   Sum: '<S3>/Sum1'

  rtb_uT[0] = 1.0 / rtb_MinMax * (rtb_Add_idx_0 - rtDW.Integrator_DSTATE[0]);

  // Sum: '<S1>/Sum3' incorporates:
  //   MATLAB Function: '<S1>/dirkin_l'
  //   Sum: '<S3>/Sum1'

  rtb_Add_idx_0 -= (-(11.0 * j * i) / 40.0 - (i * k * rtb_Sum4_tmp +
    rtb_Sum4_tmp_1 * rtb_Sum4_tmp_0) * rtb_Sum4_tmp_tmp / 4.0) - j * l * i / 4.0;

  // Product: '<S3>/1//T' incorporates:
  //   DiscreteIntegrator: '<S17>/Integrator'
  //   Fcn: '<S13>/Avoid Divide by Zero'
  //   Sum: '<S3>/Sum1'

  rtb_uT[1] = 1.0 / rtb_MinMax * (rtb_Add_idx_1 - rtDW.Integrator_DSTATE[1]);

  // Sum: '<S1>/Sum3' incorporates:
  //   MATLAB Function: '<S1>/dirkin_l'
  //   Sum: '<S3>/Sum1'

  rtb_Add_idx_1 -= ((11.0 * k / 40.0 + l * k / 4.0) - j * rtb_Sum4_tmp *
                    rtb_Sum4_tmp_tmp / 4.0) + 0.18;

  // Product: '<S3>/1//T' incorporates:
  //   DiscreteIntegrator: '<S17>/Integrator'
  //   Fcn: '<S13>/Avoid Divide by Zero'
  //   Sum: '<S3>/Sum1'

  rtb_uT[2] = 1.0 / rtb_MinMax * (rtb_Add_idx_2 - rtDW.Integrator_DSTATE[2]);

  // Sum: '<S1>/Sum3' incorporates:
  //   MATLAB Function: '<S1>/dirkin_l'
  //   Sum: '<S3>/Sum1'

  rtb_Add_idx_2 -= ((rtb_Sum4_tmp_0 * i - rtb_Sum4_tmp_1 * k * rtb_Sum4_tmp) *
                    rtb_Sum4_tmp_tmp / 4.0 - 11.0 * rtb_Sum4_tmp_1 * j / 40.0) -
    rtb_Sum4_tmp_1 * j * l / 4.0;

  // End of Outputs for SubSystem: '<Root>/CLIK'
  for (i_0 = 0; i_0 <= 0; i_0 += 2) {
    // Outputs for Atomic SubSystem: '<Root>/CLIK'
    // Sum: '<S1>/Sum4' incorporates:
    //   Gain: '<S1>/Gain1'

    tmp_2 = _mm_loadu_pd(&rtb_uT[i_0]);
    _mm_storeu_pd(&rtb_Sum4[i_0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&rtConstP.pooled6[i_0 + 3]), _mm_set1_pd(rtb_Add_idx_1)),
      _mm_mul_pd(_mm_loadu_pd(&rtConstP.pooled6[i_0]), _mm_set1_pd(rtb_Add_idx_0))),
      _mm_mul_pd(_mm_loadu_pd(&rtConstP.pooled6[i_0 + 6]), _mm_set1_pd
                 (rtb_Add_idx_2))), tmp_2));

    // End of Outputs for SubSystem: '<Root>/CLIK'
  }

  // Outputs for Atomic SubSystem: '<Root>/CLIK'
  // Sum: '<S1>/Sum4' incorporates:
  //   Gain: '<S1>/Gain1'

  for (i_0 = 2; i_0 < 3; i_0++) {
    rtb_Sum4[i_0] = ((rtConstP.pooled6[i_0 + 3] * rtb_Add_idx_1 +
                      rtConstP.pooled6[i_0] * rtb_Add_idx_0) +
                     rtConstP.pooled6[i_0 + 6] * rtb_Add_idx_2) + rtb_uT[i_0];
  }

  // MATLAB Function: '<S1>/q_l_dot' incorporates:
  //   Inport: '<Root>/K_l1M'
  //   Inport: '<Root>/K_l1m'
  //   Inport: '<Root>/K_l2M'
  //   Inport: '<Root>/K_l2m'
  //   Inport: '<Root>/K_l3M'
  //   Inport: '<Root>/K_l3m'
  //   Inport: '<Root>/K_l4M'
  //   Inport: '<Root>/K_l4m'
  //   Inport: '<Root>/K_second'
  //   Inport: '<Root>/load'
  //   Inport: '<Root>/q1l'
  //   Inport: '<Root>/q1r'
  //   Inport: '<Root>/q2l'
  //   Inport: '<Root>/q2r'
  //   Inport: '<Root>/q3l'
  //   Inport: '<Root>/q3r'
  //   Inport: '<Root>/q4l'
  //   Inport: '<Root>/q4r'
  //   MATLAB Function: '<S1>/dirkin_l'
  //   MATLAB Function: '<S1>/q_r_dot'

  rtb_MinMax = (rtU.q1l - 1.5207963267948965) / 3.1415926535897931;
  rtb_MinMax = rtb_MinMax * rtb_MinMax * 0.5;
  rtb_Add_idx_0 = (rtU.q2l - 1.5207963267948965) / 1.9198621771937625;
  rtb_Add_idx_0 = rtb_Add_idx_0 * rtb_Add_idx_0 * 0.5;
  rtb_Add_idx_1 = (rtU.q3l - 1.5207963267948965) / 3.1415926535897931;
  rtb_Add_idx_1 = rtb_Add_idx_1 * rtb_Add_idx_1 * 0.5;
  rtb_Add_idx_2 = (rtU.q4l - 2.5679938779914946) / 5.2359877559829888;
  rtb_Add_idx_2 = rtb_Add_idx_2 * rtb_Add_idx_2 * 0.5;
  w_l1m = (rtU.q1l - -1.5207963267948965) / 3.1415926535897931;
  w_l1m = w_l1m * w_l1m * 0.5;
  w_l2m = (rtU.q2l - -0.29906585039886591) / 1.9198621771937625;
  w_l2m = w_l2m * w_l2m * 0.5;
  w_l3m = (rtU.q3l - -1.5207963267948965) / 3.1415926535897931;
  w_l3m = w_l3m * w_l3m * 0.5;
  w_l4m = (rtU.q4l - -2.5679938779914946) / 5.2359877559829888;
  w_l4m = w_l4m * w_l4m * 0.5;
  J_l1M[0] = (2.0 * rtU.q1l - 3.041592653589793) / 19.739208802178716;
  J_l1M[1] = 0.0;
  J_l1M[2] = 0.0;
  J_l1M[3] = 0.0;
  J_l1m[0] = (2.0 * rtU.q1l + 3.041592653589793) / 19.739208802178716;
  J_l1m[1] = 0.0;
  J_l1m[2] = 0.0;
  J_l1m[3] = 0.0;
  J_l2M[0] = 0.0;
  J_l2M[1] = (2.0 * rtU.q2l - 3.041592653589793) * 162.0 / 1194.2221325318123;
  J_l2M[2] = 0.0;
  J_l2M[3] = 0.0;
  J_l2m[0] = 0.0;
  J_l2m[1] = (2.0 * rtU.q2l + 0.59813170079773181) * 162.0 / 1194.2221325318123;
  J_l2m[2] = 0.0;
  J_l2m[3] = 0.0;
  J_l3M[0] = 0.0;
  J_l3M[1] = 0.0;
  J_l3M[2] = (2.0 * rtU.q3l - 3.041592653589793) / 19.739208802178716;
  J_l3M[3] = 0.0;
  J_l3m[0] = 0.0;
  J_l3m[1] = 0.0;
  J_l3m[2] = (2.0 * rtU.q3l + 3.041592653589793) / 19.739208802178716;
  J_l3m[3] = 0.0;
  J_l4M[0] = 0.0;
  J_l4M[1] = 0.0;
  J_l4M[2] = 0.0;
  J_l4M[3] = (2.0 * rtU.q4l - 5.1359877559829892) * 9.0 / 493.48022005446791;
  J_l4m[0] = 0.0;
  J_l4m[1] = 0.0;
  J_l4m[2] = 0.0;
  J_l4m[3] = (2.0 * rtU.q4l + 5.1359877559829892) * 9.0 / 493.48022005446791;
  act_l1M = 0;
  act_l2M = 0;
  act_l3M = 0;
  act_l4M = 0;
  act_l1m = 0;
  act_l2m = 0;
  act_l3m = 0;
  act_l4m = 0;
  if (rtU.q1l >= 1.4707963267948965) {
    act_l1M = 1;
  } else if (rtU.q1l <= -1.4707963267948965) {
    act_l1m = 1;
  }

  if (rtU.q2l >= 1.4707963267948965) {
    act_l2M = 1;
  } else if (rtU.q2l <= -0.24906585039886589) {
    act_l2m = 1;
  }

  if (rtU.q3l >= 1.4707963267948965) {
    act_l3M = 1;
  } else if (rtU.q3l <= -1.4707963267948965) {
    act_l3m = 1;
  }

  if (rtU.q4l >= 2.5179938779914943) {
    act_l4M = 1;
  } else if (rtU.q4l <= -2.5179938779914943) {
    act_l4m = 1;
  }

  Je[0] = ((std::cos(rtU.q3l) * std::sin(rtU.q1l) - std::cos(rtU.q1l) * std::sin
            (rtU.q2l) * std::sin(rtU.q3l)) * std::sin(rtU.q4l) / 4.0 - 11.0 *
           std::cos(rtU.q1l) * std::cos(rtU.q2l) / 40.0) - std::cos(rtU.q1l) *
    std::cos(rtU.q2l) * std::cos(rtU.q4l) / 4.0;
  Je[3] = (11.0 * i * k / 40.0 + l * i * k / 4.0) - j * i * rtb_Sum4_tmp *
    rtb_Sum4_tmp_tmp / 4.0;
  Je[6] = (rtb_Sum4_tmp_1 * rtb_Sum4_tmp - std::cos(rtU.q3l) * std::sin(rtU.q1l)
           * k) * rtb_Sum4_tmp_tmp / 4.0;
  Je_tmp_1 = std::sin(rtU.q1l) * std::sin(rtU.q2l) * std::sin(rtU.q3l) + std::
    cos(rtU.q1l) * std::cos(rtU.q3l);
  Je[9] = std::cos(rtU.q2l) * std::sin(rtU.q1l) * rtb_Sum4_tmp_tmp / 4.0 -
    Je_tmp_1 * l / 4.0;
  Je[1] = 0.0;
  Je_tmp = std::cos(rtU.q2l) * std::cos(rtU.q4l);
  Je[4] = (11.0 * std::cos(rtU.q2l) / 40.0 + Je_tmp / 4.0) + k * rtb_Sum4_tmp *
    rtb_Sum4_tmp_tmp / 4.0;
  Je[7] = -(j * rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp) / 4.0;
  Je[10] = -(k * rtb_Sum4_tmp_tmp) / 4.0 - Je_tmp * rtb_Sum4_tmp / 4.0;
  Je_tmp_0 = 11.0 * std::cos(rtU.q2l) * std::sin(rtU.q1l);
  Je[2] = ((std::sin(rtU.q1l) * std::sin(rtU.q2l) * std::sin(rtU.q3l) + std::cos
            (rtU.q1l) * std::cos(rtU.q3l)) * std::sin(rtU.q4l) / 4.0 + Je_tmp_0 /
           40.0) + std::cos(rtU.q2l) * std::cos(rtU.q4l) * std::sin(rtU.q1l) /
    4.0;
  rtb_AvoidDividebyZero = std::cos(rtU.q1l) * std::cos(rtU.q2l);
  Je[5] = (11.0 * std::cos(rtU.q1l) * k / 40.0 + rtb_Sum4_tmp_1 * l * k / 4.0) -
    rtb_AvoidDividebyZero * rtb_Sum4_tmp * rtb_Sum4_tmp_tmp / 4.0;
  Je[8] = -((std::cos(rtU.q1l) * std::cos(rtU.q3l) * k + i * rtb_Sum4_tmp) *
            rtb_Sum4_tmp_tmp) / 4.0;
  Je_tmp_2 = std::cos(rtU.q3l) * std::sin(rtU.q1l) - std::cos(rtU.q1l) * std::
    sin(rtU.q2l) * std::sin(rtU.q3l);
  Je[11] = Je_tmp_2 * l / 4.0 + rtb_AvoidDividebyZero * rtb_Sum4_tmp_tmp / 4.0;
  eye(N_e_tmp);
  for (i_0 = 0; i_0 < 3; i_0++) {
    tot_task = i_0 << 2;
    N_e_tmp_0[tot_task] = Je[i_0];
    N_e_tmp_0[tot_task + 1] = Je[i_0 + 3];
    N_e_tmp_0[tot_task + 2] = Je[i_0 + 6];
    N_e_tmp_0[tot_task + 3] = Je[i_0 + 9];
    for (N_b_tmp = 0; N_b_tmp <= 0; N_b_tmp += 2) {
      converted = 3 * i_0 + N_b_tmp;
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_set1_pd(0.0));
      tmp_2 = _mm_loadu_pd(&Je[N_b_tmp]);
      tmp_1 = _mm_loadu_pd(&N_e_tmp_1[converted]);
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_add_pd(tmp_1, _mm_mul_pd
        (_mm_set1_pd(N_e_tmp_0[tot_task]), tmp_2)));
      tmp_2 = _mm_loadu_pd(&Je[N_b_tmp + 3]);
      tmp_1 = _mm_loadu_pd(&N_e_tmp_1[converted]);
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (N_e_tmp_0[tot_task + 1]), tmp_2), tmp_1));
      tmp_2 = _mm_loadu_pd(&Je[N_b_tmp + 6]);
      tmp_1 = _mm_loadu_pd(&N_e_tmp_1[converted]);
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (N_e_tmp_0[tot_task + 2]), tmp_2), tmp_1));
      tmp_2 = _mm_loadu_pd(&Je[N_b_tmp + 9]);
      tmp_1 = _mm_loadu_pd(&N_e_tmp_1[converted]);
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (N_e_tmp_0[tot_task + 3]), tmp_2), tmp_1));
    }

    for (N_b_tmp = 2; N_b_tmp < 3; N_b_tmp++) {
      converted = 3 * i_0 + N_b_tmp;
      N_e_tmp_1[converted] = 0.0;
      N_e_tmp_1[converted] += N_e_tmp_0[tot_task] * Je[N_b_tmp];
      N_e_tmp_1[converted] += N_e_tmp_0[tot_task + 1] * Je[N_b_tmp + 3];
      N_e_tmp_1[converted] += N_e_tmp_0[tot_task + 2] * Je[N_b_tmp + 6];
      N_e_tmp_1[converted] += N_e_tmp_0[tot_task + 3] * Je[N_b_tmp + 9];
    }
  }

  mrdiv(N_e_tmp_0, N_e_tmp_1, tmp);
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
      tot_task = (N_b_tmp << 2) + i_0;
      N_e[tot_task] = N_e_tmp[tot_task] - ((Je[3 * N_b_tmp + 1] * tmp[i_0 + 4] +
        Je[3 * N_b_tmp] * tmp[i_0]) + Je[3 * N_b_tmp + 2] * tmp[i_0 + 8]);
    }
  }

  rtb_AvoidDividebyZero = std::sin(rtU.q1r);
  q_l_dot_opt_tmp = std::cos(rtU.q2r);
  q_l_dot_opt_tmp_tmp_1 = std::cos(rtU.q1r);
  q_l_dot_opt_tmp_tmp_3 = std::sin(rtU.q4r);
  q_l_dot_opt_tmp_0 = std::cos(rtU.q4r);
  q_l_dot_opt_tmp_tmp_2 = std::cos(rtU.q3r);
  q_l_dot_opt_tmp_tmp = std::sin(rtU.q2r);
  q_l_dot_opt_tmp_tmp_0 = std::sin(rtU.q3r);
  q_l_dot_opt_tmp_1 = 11.0 * std::cos(rtU.q1l) * std::cos(rtU.q2l) / 80.0;
  q_l_dot_opt_tmp_2 = (std::cos(rtU.q3l) * std::sin(rtU.q1l) - std::cos(rtU.q1l)
                       * std::sin(rtU.q2l) * std::sin(rtU.q3l)) * std::sin
    (rtU.q4l) / 8.0;
  q_l_dot_opt_tmp_3 = std::cos(rtU.q1l) * std::cos(rtU.q2l) * std::cos(rtU.q4l) /
    8.0;
  q_l_dot_opt_tmp_tmp_6 = 7.409300489671729E+15 * rtb_Sum4_tmp_tmp;
  q_l_dot_opt_tmp_4 = Je_tmp_2 * q_l_dot_opt_tmp_tmp_6 / 7.2057594037927936E+17;
  q_l_dot_opt_tmp_5 = (std::sin(rtU.q1l) * std::sin(rtU.q2l) * std::sin(rtU.q3l)
                       + std::cos(rtU.q1l) * std::cos(rtU.q3l)) * std::sin
    (rtU.q4l) / 8.0;
  q_l_dot_opt_tmp_6 = Je_tmp_0 / 80.0;
  q_l_dot_opt_tmp_7 = std::cos(rtU.q2l) * std::cos(rtU.q4l) * std::sin(rtU.q1l) /
    8.0;
  q_l_dot_opt_tmp_8 = (rtU.load + 1.6) * 3.0;
  q_l_dot_opt_tmp_tmp_tmp_0 = q_l_dot_opt_tmp_tmp_1 * q_l_dot_opt_tmp_tmp_2;
  q_l_dot_opt_tmp_tmp_4 = rtb_AvoidDividebyZero * q_l_dot_opt_tmp_tmp *
    q_l_dot_opt_tmp_tmp_0 + q_l_dot_opt_tmp_tmp_tmp_0;
  q_l_dot_opt_tmp_tmp_tmp = q_l_dot_opt_tmp_tmp_2 * rtb_AvoidDividebyZero;
  q_l_dot_opt_tmp_tmp_5 = q_l_dot_opt_tmp_tmp_tmp - q_l_dot_opt_tmp_tmp_1 *
    q_l_dot_opt_tmp_tmp * q_l_dot_opt_tmp_tmp_0;
  q_l_dot_opt_tmp_tmp_7 = 7.409300489671729E+15 * q_l_dot_opt_tmp_tmp_3;
  q_l_dot_opt_tmp_9 = q_l_dot_opt_tmp_tmp_5 * q_l_dot_opt_tmp_tmp_7 /
    7.2057594037927936E+17;
  q_l_dot_opt[0] = ((((rtU.load * 0.99749498660405445 / 4.0 +
                       0.020513424549813183) + 0.0075) / (rtU.load + 1.6) -
                     ((((((((((11.0 * q_l_dot_opt_tmp * rtb_AvoidDividebyZero /
    80.0 + q_l_dot_opt_tmp_6) + q_l_dot_opt_tmp_5) + q_l_dot_opt_tmp_tmp_4 *
    q_l_dot_opt_tmp_tmp_3 / 8.0) + q_l_dot_opt_tmp_7) + q_l_dot_opt_tmp *
    q_l_dot_opt_tmp_0 * rtb_AvoidDividebyZero / 8.0) * rtU.load + (((((3.0 *
    rtb_Sum4_tmp_1 / 800.0 + 3.0 * q_l_dot_opt_tmp_tmp_1 / 800.0) + 10879.0 * i /
    5.0E+6) + 10879.0 * rtb_AvoidDividebyZero / 5.0E+6) + 10481.0 * j * i /
    100000.0) + 10481.0 * q_l_dot_opt_tmp * rtb_AvoidDividebyZero / 100000.0)) +
    7.409300489671729E+15 * rtb_Sum4_tmp_tmp * Je_tmp_1 / 7.2057594037927936E+17)
                        + 7.409300489671729E+15 * q_l_dot_opt_tmp_tmp_3 *
                        q_l_dot_opt_tmp_tmp_4 / 7.2057594037927936E+17) +
                       7.409300489671729E+15 * j * l * i /
                       7.2057594037927936E+17) + 7.409300489671729E+15 *
                      q_l_dot_opt_tmp * q_l_dot_opt_tmp_0 *
                      rtb_AvoidDividebyZero / 7.2057594037927936E+17) /
                     (rtU.load + 1.6)) * (((((10879.0 * rtb_Sum4_tmp_1 / 5.0E+6
    - 3.0 * i / 800.0) + 10481.0 * rtb_Sum4_tmp_1 * j / 100000.0) -
    q_l_dot_opt_tmp_4) + ((q_l_dot_opt_tmp_1 - q_l_dot_opt_tmp_2) +
    q_l_dot_opt_tmp_3) * rtU.load) + 7.409300489671729E+15 * rtb_Sum4_tmp_1 * j *
    l / 7.2057594037927936E+17) / q_l_dot_opt_tmp_8 + (((((((((((11.0 *
    q_l_dot_opt_tmp_tmp_1 * q_l_dot_opt_tmp / 80.0 + q_l_dot_opt_tmp_1) -
    q_l_dot_opt_tmp_2) - q_l_dot_opt_tmp_tmp_5 * q_l_dot_opt_tmp_tmp_3 / 8.0) +
    q_l_dot_opt_tmp_3) + q_l_dot_opt_tmp_tmp_1 * q_l_dot_opt_tmp *
    q_l_dot_opt_tmp_0 / 8.0) * rtU.load + (((((10879.0 * std::cos(rtU.q1l) /
    5.0E+6 + 10879.0 * q_l_dot_opt_tmp_tmp_1 / 5.0E+6) - 3.0 * std::sin(rtU.q1l)
    / 800.0) - 3.0 * rtb_AvoidDividebyZero / 800.0) + 10481.0 * std::cos(rtU.q1l)
    * std::cos(rtU.q2l) / 100000.0) + 10481.0 * q_l_dot_opt_tmp_tmp_1 *
    q_l_dot_opt_tmp / 100000.0)) - q_l_dot_opt_tmp_4) - q_l_dot_opt_tmp_9) +
    7.409300489671729E+15 * std::cos(rtU.q1l) * std::cos(rtU.q2l) * std::cos
    (rtU.q4l) / 7.2057594037927936E+17) + 7.409300489671729E+15 *
    q_l_dot_opt_tmp_tmp_1 * q_l_dot_opt_tmp * q_l_dot_opt_tmp_0 /
    7.2057594037927936E+17) / (rtU.load + 1.6) - ((rtU.load *
    0.29268430041692572 + 0.0014547063080642103) + 0.2139716) / (rtU.load + 1.6))
                    * ((((std::sin(rtU.q1l) * std::sin(rtU.q2l) * std::sin
    (rtU.q3l) + std::cos(rtU.q1l) * std::cos(rtU.q3l)) * (7.409300489671729E+15 *
    std::sin(rtU.q4l)) / 7.2057594037927936E+17 + ((3.0 * std::cos(rtU.q1l) /
    800.0 + 10879.0 * std::sin(rtU.q1l) / 5.0E+6) + 10481.0 * std::cos(rtU.q2l) *
    std::sin(rtU.q1l) / 100000.0)) + ((q_l_dot_opt_tmp_5 + q_l_dot_opt_tmp_6) +
    q_l_dot_opt_tmp_7) * rtU.load) + 7.409300489671729E+15 * std::cos(rtU.q2l) *
                       std::cos(rtU.q4l) * std::sin(rtU.q1l) /
                       7.2057594037927936E+17) / q_l_dot_opt_tmp_8) *
    rtU.K_second;
  q_l_dot_opt_tmp_1 = ((rtU.load * 0.99749498660405445 / 4.0 +
                        0.020513424549813183) + 0.0075) / (rtU.load + 1.6) -
    ((((((((((std::sin(rtU.q1l) * std::sin(rtU.q2l) * std::sin(rtU.q3l) + std::
              cos(rtU.q1l) * std::cos(rtU.q3l)) * std::sin(rtU.q4l) / 8.0 +
             (11.0 * std::cos(rtU.q2l) * std::sin(rtU.q1l) / 80.0 + 11.0 * std::
              cos(rtU.q2r) * std::sin(rtU.q1r) / 80.0)) + (std::sin(rtU.q1r) *
             std::sin(rtU.q2r) * std::sin(rtU.q3r) + std::cos(rtU.q1r) * std::
             cos(rtU.q3r)) * std::sin(rtU.q4r) / 8.0) + std::cos(rtU.q2l) * std::
           cos(rtU.q4l) * std::sin(rtU.q1l) / 8.0) + std::cos(rtU.q2r) * std::
          cos(rtU.q4r) * std::sin(rtU.q1r) / 8.0) * rtU.load + (((((3.0 * std::
              cos(rtU.q1l) / 800.0 + 3.0 * std::cos(rtU.q1r) / 800.0) + 10879.0 *
             std::sin(rtU.q1l) / 5.0E+6) + 10879.0 * std::sin(rtU.q1r) / 5.0E+6)
           + 10481.0 * std::cos(rtU.q2l) * std::sin(rtU.q1l) / 100000.0) +
          10481.0 * std::cos(rtU.q2r) * std::sin(rtU.q1r) / 100000.0)) + (std::
         sin(rtU.q1l) * std::sin(rtU.q2l) * std::sin(rtU.q3l) + std::cos(rtU.q1l)
         * std::cos(rtU.q3l)) * (7.409300489671729E+15 * std::sin(rtU.q4l)) /
        7.2057594037927936E+17) + (std::sin(rtU.q1r) * std::sin(rtU.q2r) * std::
        sin(rtU.q3r) + std::cos(rtU.q1r) * std::cos(rtU.q3r)) *
       (7.409300489671729E+15 * std::sin(rtU.q4r)) / 7.2057594037927936E+17) +
      7.409300489671729E+15 * std::cos(rtU.q2l) * std::cos(rtU.q4l) * std::sin
      (rtU.q1l) / 7.2057594037927936E+17) + 7.409300489671729E+15 * std::cos
     (rtU.q2r) * std::cos(rtU.q4r) * std::sin(rtU.q1r) / 7.2057594037927936E+17)
    / (rtU.load + 1.6);
  q_l_dot_opt_tmp_2 = (rtU.load + 1.6) * (rtU.load + 1.6) * 3.0;
  q_l_dot_opt_tmp_3 = ((((((((((11.0 * std::cos(rtU.q1l) * std::cos(rtU.q2l) /
    80.0 + 11.0 * std::cos(rtU.q1r) * std::cos(rtU.q2r) / 80.0) - (std::cos
    (rtU.q3l) * std::sin(rtU.q1l) - std::cos(rtU.q1l) * std::sin(rtU.q2l) * std::
    sin(rtU.q3l)) * std::sin(rtU.q4l) / 8.0) - (std::cos(rtU.q3r) * std::sin
    (rtU.q1r) - std::cos(rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r)) * std::
    sin(rtU.q4r) / 8.0) + std::cos(rtU.q1l) * std::cos(rtU.q2l) * std::cos
    (rtU.q4l) / 8.0) + std::cos(rtU.q1r) * std::cos(rtU.q2r) * std::cos(rtU.q4r)
    / 8.0) * rtU.load + (((((10879.0 * std::cos(rtU.q1l) / 5.0E+6 + 10879.0 *
    std::cos(rtU.q1r) / 5.0E+6) - 3.0 * std::sin(rtU.q1l) / 800.0) - 3.0 * std::
    sin(rtU.q1r) / 800.0) + 10481.0 * std::cos(rtU.q1l) * std::cos(rtU.q2l) /
    100000.0) + 10481.0 * std::cos(rtU.q1r) * std::cos(rtU.q2r) / 100000.0)) -
    (std::cos(rtU.q3l) * std::sin(rtU.q1l) - std::cos(rtU.q1l) * std::sin
     (rtU.q2l) * std::sin(rtU.q3l)) * (7.409300489671729E+15 * std::sin(rtU.q4l))
    / 7.2057594037927936E+17) - (std::cos(rtU.q3r) * std::sin(rtU.q1r) - std::
    cos(rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r)) *
    (7.409300489671729E+15 * std::sin(rtU.q4r)) / 7.2057594037927936E+17) +
                        7.409300489671729E+15 * std::cos(rtU.q1l) * std::cos
                        (rtU.q2l) * std::cos(rtU.q4l) / 7.2057594037927936E+17)
                       + 7.409300489671729E+15 * std::cos(rtU.q1r) * std::cos
                       (rtU.q2r) * std::cos(rtU.q4r) / 7.2057594037927936E+17) /
    (rtU.load + 1.6) - ((rtU.load * 0.29268430041692572 + 0.0014547063080642103)
                        + 0.2139716) / (rtU.load + 1.6);
  q_l_dot_opt_tmp_4 = 7.409300489671729E+15 * std::cos(rtU.q1l) * std::cos
    (rtU.q2l);
  q_l_dot_opt_tmp_5 = 7.409300489671729E+15 * std::cos(rtU.q2l) * std::cos
    (rtU.q4l);
  q_l_dot_opt[1] = ((((((((((11.0 * std::sin(rtU.q2l) / 80.0 + 11.0 *
    q_l_dot_opt_tmp_tmp / 80.0) + std::cos(rtU.q4l) * std::sin(rtU.q2l) / 8.0) +
    q_l_dot_opt_tmp_0 * q_l_dot_opt_tmp_tmp / 8.0) - std::cos(rtU.q2l) * std::
    sin(rtU.q3l) * std::sin(rtU.q4l) / 8.0) - q_l_dot_opt_tmp *
    q_l_dot_opt_tmp_tmp_0 * q_l_dot_opt_tmp_tmp_3 / 8.0) * rtU.load + (((10481.0
    * k / 100000.0 + 10481.0 * q_l_dot_opt_tmp_tmp / 100000.0) +
    7.409300489671729E+15 * l * k / 7.2057594037927936E+17) +
    7.409300489671729E+15 * q_l_dot_opt_tmp_0 * q_l_dot_opt_tmp_tmp /
    7.2057594037927936E+17)) - 7.409300489671729E+15 * std::cos(rtU.q2l) *
                       rtb_Sum4_tmp * rtb_Sum4_tmp_tmp / 7.2057594037927936E+17)
                      - 7.409300489671729E+15 * std::cos(rtU.q2r) *
                      q_l_dot_opt_tmp_tmp_0 * q_l_dot_opt_tmp_tmp_3 /
                      7.2057594037927936E+17) * ((((11.0 * std::cos(rtU.q2l) /
    80.0 + Je_tmp / 8.0) + std::sin(rtU.q2l) * std::sin(rtU.q3l) * std::sin
    (rtU.q4l) / 8.0) * rtU.load + (10481.0 * std::cos(rtU.q2l) / 100000.0 +
    q_l_dot_opt_tmp_5 / 7.2057594037927936E+17)) + 7.409300489671729E+15 * k *
    rtb_Sum4_tmp * rtb_Sum4_tmp_tmp / 7.2057594037927936E+17) /
                     q_l_dot_opt_tmp_2 - (((((11.0 * std::cos(rtU.q1l) * std::
    sin(rtU.q2l) / 80.0 + std::cos(rtU.q1l) * std::cos(rtU.q4l) * std::sin
    (rtU.q2l) / 8.0) - std::cos(rtU.q1l) * std::cos(rtU.q2l) * std::sin(rtU.q3l)
    * std::sin(rtU.q4l) / 8.0) * rtU.load + 10481.0 * std::cos(rtU.q1l) * k /
    100000.0) + 7.409300489671729E+15 * std::cos(rtU.q1l) * l * k /
    7.2057594037927936E+17) - q_l_dot_opt_tmp_4 * rtb_Sum4_tmp *
    rtb_Sum4_tmp_tmp / 7.2057594037927936E+17) * q_l_dot_opt_tmp_3 /
                     q_l_dot_opt_tmp_8) + (((((11.0 * std::sin(rtU.q1l) * std::
    sin(rtU.q2l) / 80.0 + std::cos(rtU.q4l) * std::sin(rtU.q1l) * std::sin
    (rtU.q2l) / 8.0) - std::cos(rtU.q2l) * std::sin(rtU.q1l) * std::sin(rtU.q3l)
    * std::sin(rtU.q4l) / 8.0) * rtU.load + 10481.0 * i * k / 100000.0) +
    7.409300489671729E+15 * std::cos(rtU.q4l) * i * k / 7.2057594037927936E+17)
    - 7.409300489671729E+15 * std::cos(rtU.q2l) * i * rtb_Sum4_tmp *
    rtb_Sum4_tmp_tmp / 7.2057594037927936E+17) * q_l_dot_opt_tmp_1 /
                    q_l_dot_opt_tmp_8) * -rtU.K_second;
  q_l_dot_opt_tmp_6 = std::cos(rtU.q1l) * std::sin(rtU.q3l) - std::cos(rtU.q3l) *
    std::sin(rtU.q1l) * std::sin(rtU.q2l);
  q_l_dot_opt_tmp_7 = std::cos(rtU.q1l) * std::cos(rtU.q3l) * std::sin(rtU.q2l)
    + std::sin(rtU.q1l) * std::sin(rtU.q3l);
  Je_tmp_0 = (((((((11.0 * std::sin(rtU.q2l) / 80.0 + 11.0 * std::sin(rtU.q2r) /
                    80.0) + std::cos(rtU.q4l) * std::sin(rtU.q2l) / 8.0) + std::
                  cos(rtU.q4r) * std::sin(rtU.q2r) / 8.0) - std::cos(rtU.q2l) *
                 std::sin(rtU.q3l) * std::sin(rtU.q4l) / 8.0) - std::cos(rtU.q2r)
                * std::sin(rtU.q3r) * std::sin(rtU.q4r) / 8.0) * rtU.load +
               (((10481.0 * std::sin(rtU.q2l) / 100000.0 + 10481.0 * std::sin
                  (rtU.q2r) / 100000.0) + 7.409300489671729E+15 * std::cos
                 (rtU.q4l) * std::sin(rtU.q2l) / 7.2057594037927936E+17) +
                7.409300489671729E+15 * std::cos(rtU.q4r) * std::sin(rtU.q2r) /
                7.2057594037927936E+17)) - 7.409300489671729E+15 * std::cos
              (rtU.q2l) * std::sin(rtU.q3l) * std::sin(rtU.q4l) /
              7.2057594037927936E+17) - 7.409300489671729E+15 * std::cos(rtU.q2r)
    * std::sin(rtU.q3r) * std::sin(rtU.q4r) / 7.2057594037927936E+17;
  q_l_dot_opt[2] = (((rtU.load * rtb_Sum4_tmp_tmp * q_l_dot_opt_tmp_6 / 8.0 +
                      q_l_dot_opt_tmp_6 * q_l_dot_opt_tmp_tmp_6 /
                      7.2057594037927936E+17) * q_l_dot_opt_tmp_1 /
                     q_l_dot_opt_tmp_8 - (7.409300489671729E+15 * std::cos
    (rtU.q2l) * rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp / 7.2057594037927936E+17 +
    rtU.load * j * rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp / 8.0) * Je_tmp_0 /
                     q_l_dot_opt_tmp_2) + (rtU.load * std::sin(rtU.q4l) *
    q_l_dot_opt_tmp_7 / 8.0 + q_l_dot_opt_tmp_7 * q_l_dot_opt_tmp_tmp_6 /
    7.2057594037927936E+17) * q_l_dot_opt_tmp_3 / q_l_dot_opt_tmp_8) *
    -rtU.K_second;
  q_l_dot_opt[3] = ((((((std::cos(rtU.q3l) * std::sin(rtU.q1l) - std::cos
    (rtU.q1l) * std::sin(rtU.q2l) * std::sin(rtU.q3l)) * std::cos(rtU.q4l) / 8.0
                        + std::cos(rtU.q1l) * std::cos(rtU.q2l) * std::sin
                        (rtU.q4l) / 8.0) * rtU.load + 7.409300489671729E+15 *
                       std::cos(rtU.q4l) * Je_tmp_2 / 7.2057594037927936E+17) +
                      q_l_dot_opt_tmp_4 * rtb_Sum4_tmp_tmp /
                      7.2057594037927936E+17) * q_l_dot_opt_tmp_3 /
                     q_l_dot_opt_tmp_8 + ((((std::sin(rtU.q1l) * std::sin
    (rtU.q2l) * std::sin(rtU.q3l) + std::cos(rtU.q1l) * std::cos(rtU.q3l)) * std::
    cos(rtU.q4l) / 8.0 - std::cos(rtU.q2l) * std::sin(rtU.q1l) * std::sin
    (rtU.q4l) / 8.0) * rtU.load + 7.409300489671729E+15 * std::cos(rtU.q4l) *
    Je_tmp_1 / 7.2057594037927936E+17) - 7.409300489671729E+15 * std::cos
    (rtU.q2l) * std::sin(rtU.q1l) * rtb_Sum4_tmp_tmp / 7.2057594037927936E+17) *
                     q_l_dot_opt_tmp_1 / q_l_dot_opt_tmp_8) + (((std::cos
    (rtU.q2l) * std::cos(rtU.q4l) * std::sin(rtU.q3l) / 8.0 + std::sin(rtU.q2l) *
    std::sin(rtU.q4l) / 8.0) * rtU.load + 7.409300489671729E+15 * std::sin
    (rtU.q2l) * rtb_Sum4_tmp_tmp / 7.2057594037927936E+17) + q_l_dot_opt_tmp_5 *
    rtb_Sum4_tmp / 7.2057594037927936E+17) * Je_tmp_0 / q_l_dot_opt_tmp_2) *
    rtU.K_second;
  tot_task = ((((((act_l1M + act_l1m) + act_l2M) + act_l2m) + act_l3M) + act_l3m)
              + act_l4M) + act_l4m;
  switch (tot_task) {
   case 0:
    temp_act[0] = static_cast<int8_T>(act_l1M);
    temp_act[16] = static_cast<int8_T>(act_l1m);
    temp_act[32] = static_cast<int8_T>(act_l2M);
    temp_act[48] = static_cast<int8_T>(act_l2m);
    temp_act[64] = static_cast<int8_T>(act_l3M);
    temp_act[80] = static_cast<int8_T>(act_l3m);
    temp_act[96] = static_cast<int8_T>(act_l4M);
    temp_act[112] = static_cast<int8_T>(act_l4m);
    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    break;

   case 1:
    for (i_0 = 0; i_0 < 8; i_0++) {
      temp_act[i_0 << 4] = 0;
    }

    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    break;

   case 2:
    for (i_0 = 0; i_0 < 8; i_0++) {
      temp_act[i_0 << 4] = 0;
    }

    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    i = 1.0;
    do {
      exitg1 = 0;
      i_0 = ((static_cast<int32_T>(i) - 1) << 4) + 1;
      if (temp_act[i_0] == 1) {
        exitg1 = 1;
      } else {
        i++;
      }
    } while (exitg1 == 0);

    temp_act[i_0] = 0;
    j = 1.0;
    do {
      exitg1 = 0;
      i_0 = ((static_cast<int32_T>(j) - 1) << 4) + 2;
      if ((temp_act[i_0] == 1) && (j != i)) {
        exitg1 = 1;
      } else {
        j++;
      }
    } while (exitg1 == 0);

    temp_act[i_0] = 0;
    break;

   case 3:
    for (i_0 = 0; i_0 < 8; i_0++) {
      temp_act[i_0 << 4] = 0;
    }

    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    i = 1.0;
    do {
      exitg1 = 0;
      i_0 = (static_cast<int32_T>(i) - 1) << 4;
      if (temp_act[i_0 + 6] == 1) {
        exitg1 = 1;
      } else {
        i++;
      }
    } while (exitg1 == 0);

    temp_act[i_0 + 6] = 0;
    j = 1.0;
    do {
      exitg1 = 0;
      N_b_tmp = (static_cast<int32_T>(j) - 1) << 4;
      if ((temp_act[N_b_tmp + 5] == 1) && (j != i)) {
        exitg1 = 1;
      } else {
        j++;
      }
    } while (exitg1 == 0);

    temp_act[N_b_tmp + 5] = 0;
    k = 1.0;
    do {
      exitg1 = 0;
      converted = (static_cast<int32_T>(k) - 1) << 4;
      if ((temp_act[converted + 4] == 1) && ((k != i) && (k != j))) {
        exitg1 = 1;
      } else {
        k++;
      }
    } while (exitg1 == 0);

    temp_act[converted + 4] = 0;
    temp_act[i_0 + 3] = 0;
    temp_act[N_b_tmp + 3] = 0;
    temp_act[i_0 + 2] = 0;
    temp_act[converted + 2] = 0;
    temp_act[N_b_tmp + 1] = 0;
    temp_act[converted + 1] = 0;
    break;

   default:
    for (i_0 = 0; i_0 < 8; i_0++) {
      temp_act[i_0 << 4] = 0;
    }

    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    i = 1.0;
    do {
      exitg1 = 0;
      i_0 = (static_cast<int32_T>(i) - 1) << 4;
      if (temp_act[i_0 + 14] == 1) {
        exitg1 = 1;
      } else {
        i++;
      }
    } while (exitg1 == 0);

    temp_act[i_0 + 14] = 0;
    j = 1.0;
    do {
      exitg1 = 0;
      N_b_tmp = (static_cast<int32_T>(j) - 1) << 4;
      if ((temp_act[N_b_tmp + 13] == 1) && (j != i)) {
        exitg1 = 1;
      } else {
        j++;
      }
    } while (exitg1 == 0);

    temp_act[N_b_tmp + 13] = 0;
    k = 1.0;
    do {
      exitg1 = 0;
      converted = (static_cast<int32_T>(k) - 1) << 4;
      if ((temp_act[converted + 12] == 1) && ((k != i) && (k != j))) {
        exitg1 = 1;
      } else {
        k++;
      }
    } while (exitg1 == 0);

    temp_act[converted + 12] = 0;
    l = 1.0;
    do {
      exitg1 = 0;
      N_a_tmp = (static_cast<int32_T>(l) - 1) << 4;
      if ((temp_act[N_a_tmp + 11] == 1) && ((l != i) && (l != j) && (l != k))) {
        exitg1 = 1;
      } else {
        l++;
      }
    } while (exitg1 == 0);

    temp_act[N_a_tmp + 11] = 0;
    temp_act[i_0 + 10] = 0;
    temp_act[N_b_tmp + 10] = 0;
    temp_act[i_0 + 9] = 0;
    temp_act[converted + 9] = 0;
    temp_act[i_0 + 8] = 0;
    temp_act[N_a_tmp + 8] = 0;
    temp_act[N_b_tmp + 7] = 0;
    temp_act[converted + 7] = 0;
    temp_act[N_b_tmp + 6] = 0;
    temp_act[N_a_tmp + 6] = 0;
    temp_act[converted + 5] = 0;
    temp_act[N_a_tmp + 5] = 0;
    temp_act[i_0 + 4] = 0;
    temp_act[N_b_tmp + 4] = 0;
    temp_act[converted + 4] = 0;
    temp_act[N_b_tmp + 3] = 0;
    temp_act[converted + 3] = 0;
    temp_act[N_a_tmp + 3] = 0;
    temp_act[converted + 2] = 0;
    temp_act[N_a_tmp + 2] = 0;
    temp_act[i_0 + 2] = 0;
    temp_act[N_a_tmp + 1] = 0;
    temp_act[i_0 + 1] = 0;
    temp_act[N_b_tmp + 1] = 0;
    break;
  }

  converted = 0;
  rtb_q_l_dot[0] = 0.0;
  rtb_q_l_dot[1] = 0.0;
  rtb_q_l_dot[2] = 0.0;
  rtb_q_l_dot[3] = 0.0;
  i = 1.0;
  while (converted == 0) {
    if (temp_act[static_cast<int32_T>(i) - 1] == 1) {
      j = rtU.K_l1M;
      k = -rtb_MinMax;
      J_a[0] = J_l1M[0];
      q_l_dot_opt_tmp_4 = J_l1M[0] * J_l1M[0];
      J_a[1] = 0.0;
      J_a[2] = 0.0;
      J_a[3] = 0.0;
      l = J_l1M[0] / q_l_dot_opt_tmp_4;
      rtb_Sum4_tmp = 0.0 / q_l_dot_opt_tmp_4;
      for (i_0 = 0; i_0 < 4; i_0++) {
        q_l_dot_opt_tmp_4 = J_l1M[i_0];
        N_a_tmp = i_0 << 2;
        N_a[N_a_tmp] = N_e_tmp[N_a_tmp] - l * q_l_dot_opt_tmp_4;
        q_l_dot_opt_tmp_tmp_6 = rtb_Sum4_tmp * q_l_dot_opt_tmp_4;
        N_a[N_a_tmp + 1] = N_e_tmp[N_a_tmp + 1] - q_l_dot_opt_tmp_tmp_6;
        N_a[N_a_tmp + 2] = N_e_tmp[N_a_tmp + 2] - q_l_dot_opt_tmp_tmp_6;
        N_a[N_a_tmp + 3] = N_e_tmp[N_a_tmp + 3] - q_l_dot_opt_tmp_tmp_6;
      }
    } else if (temp_act[static_cast<int32_T>(i) + 15] == 1) {
      j = rtU.K_l1m;
      k = -w_l1m;
      J_a[0] = J_l1m[0];
      l = J_l1m[0] * J_l1m[0];
      J_a[1] = 0.0;
      J_a[2] = 0.0;
      J_a[3] = 0.0;
      rtb_Sum4_tmp = J_l1m[0] / l;
      rtb_Sum4_tmp_tmp = 0.0 / l;
      for (i_0 = 0; i_0 < 4; i_0++) {
        l = J_l1m[i_0];
        N_a_tmp = i_0 << 2;
        N_a[N_a_tmp] = N_e_tmp[N_a_tmp] - rtb_Sum4_tmp * l;
        q_l_dot_opt_tmp_tmp_6 = rtb_Sum4_tmp_tmp * l;
        N_a[N_a_tmp + 1] = N_e_tmp[N_a_tmp + 1] - q_l_dot_opt_tmp_tmp_6;
        N_a[N_a_tmp + 2] = N_e_tmp[N_a_tmp + 2] - q_l_dot_opt_tmp_tmp_6;
        N_a[N_a_tmp + 3] = N_e_tmp[N_a_tmp + 3] - rtb_Sum4_tmp_tmp * l;
      }
    } else {
      j = 0.0;
      k = 0.0;
      J_a[0] = 1.0;
      J_a[1] = 0.0;
      J_a[2] = 0.0;
      J_a[3] = 0.0;
      std::memcpy(&N_a[0], &N_e_tmp[0], sizeof(real_T) << 4U);
    }

    if (temp_act[static_cast<int32_T>(i) + 31] == 1) {
      l = rtU.K_l2M;
      rtb_Sum4_tmp = -rtb_Add_idx_0;
      J_b[0] = 0.0;
      J_b[1] = J_l2M[1];
      rtb_Sum4_tmp_tmp = J_l2M[1] * J_l2M[1];
      J_b[2] = 0.0;
      J_b[3] = 0.0;
      rtb_Sum4_tmp_0 = 0.0 / rtb_Sum4_tmp_tmp;
      rtb_Sum4_tmp_1 = J_l2M[1] / rtb_Sum4_tmp_tmp;
      for (i_0 = 0; i_0 < 4; i_0++) {
        rtb_Sum4_tmp_tmp = J_l2M[i_0];
        N_b_tmp = i_0 << 2;
        N_b[N_b_tmp] = N_e_tmp[N_b_tmp] - rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - rtb_Sum4_tmp_1 *
          rtb_Sum4_tmp_tmp;
        Je_tmp_1 = rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - Je_tmp_1;
        N_b[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - Je_tmp_1;
      }
    } else if (temp_act[static_cast<int32_T>(i) + 47] == 1) {
      l = rtU.K_l2m;
      rtb_Sum4_tmp = -w_l2m;
      J_b[0] = 0.0;
      J_b[1] = J_l2m[1];
      rtb_Sum4_tmp_tmp = J_l2m[1] * J_l2m[1];
      J_b[2] = 0.0;
      J_b[3] = 0.0;
      rtb_Sum4_tmp_0 = 0.0 / rtb_Sum4_tmp_tmp;
      rtb_Sum4_tmp_1 = J_l2m[1] / rtb_Sum4_tmp_tmp;
      for (i_0 = 0; i_0 < 4; i_0++) {
        rtb_Sum4_tmp_tmp = J_l2m[i_0];
        N_b_tmp = i_0 << 2;
        N_b[N_b_tmp] = N_e_tmp[N_b_tmp] - rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - rtb_Sum4_tmp_1 *
          rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - rtb_Sum4_tmp_0 *
          rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - rtb_Sum4_tmp_0 *
          rtb_Sum4_tmp_tmp;
      }
    } else {
      l = 0.0;
      rtb_Sum4_tmp = 0.0;
      J_b[0] = 0.0;
      J_b[1] = 1.0;
      J_b[2] = 0.0;
      J_b[3] = 0.0;
      std::memcpy(&N_b[0], &N_e_tmp[0], sizeof(real_T) << 4U);
    }

    if (temp_act[static_cast<int32_T>(i) + 63] == 1) {
      rtb_Sum4_tmp_tmp = rtU.K_l3M;
      rtb_Sum4_tmp_0 = -rtb_Add_idx_1;
      J_c[0] = 0.0;
      J_c[1] = 0.0;
      J_c[2] = J_l3M[2];
      rtb_Sum4_tmp_1 = J_l3M[2] * J_l3M[2];
      J_c[3] = 0.0;
      Je_tmp_1 = 0.0 / rtb_Sum4_tmp_1;
      Je_tmp = J_l3M[2] / rtb_Sum4_tmp_1;
      for (i_0 = 0; i_0 < 4; i_0++) {
        rtb_Sum4_tmp_1 = J_l3M[i_0];
        N_b_tmp = i_0 << 2;
        N_c[N_b_tmp] = N_e_tmp[N_b_tmp] - Je_tmp_1 * rtb_Sum4_tmp_1;
        Je_tmp_2 = Je_tmp_1 * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - Je_tmp_2;
        N_c[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - Je_tmp * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - Je_tmp_2;
      }
    } else if (temp_act[static_cast<int32_T>(i) + 79] == 1) {
      rtb_Sum4_tmp_tmp = rtU.K_l3m;
      rtb_Sum4_tmp_0 = -w_l3m;
      J_c[0] = 0.0;
      J_c[1] = 0.0;
      J_c[2] = J_l3m[2];
      rtb_Sum4_tmp_1 = J_l3m[2] * J_l3m[2];
      J_c[3] = 0.0;
      Je_tmp_1 = 0.0 / rtb_Sum4_tmp_1;
      Je_tmp = J_l3m[2] / rtb_Sum4_tmp_1;
      for (i_0 = 0; i_0 < 4; i_0++) {
        rtb_Sum4_tmp_1 = J_l3m[i_0];
        N_b_tmp = i_0 << 2;
        N_c[N_b_tmp] = N_e_tmp[N_b_tmp] - Je_tmp_1 * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - Je_tmp_1 * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - Je_tmp * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - Je_tmp_1 * rtb_Sum4_tmp_1;
      }
    } else {
      rtb_Sum4_tmp_tmp = 0.0;
      rtb_Sum4_tmp_0 = 0.0;
      J_c[0] = 0.0;
      J_c[1] = 0.0;
      J_c[2] = 1.0;
      J_c[3] = 0.0;
      std::memcpy(&N_c[0], &N_e_tmp[0], sizeof(real_T) << 4U);
    }

    if (temp_act[static_cast<int32_T>(i) + 95] == 1) {
      rtb_Sum4_tmp_1 = rtU.K_l4M;
      Je_tmp_1 = -rtb_Add_idx_2;
      J_d[0] = 0.0;
      J_d[1] = 0.0;
      J_d[2] = 0.0;
      J_d[3] = J_l4M[3];
      Je_tmp = J_l4M[3] * J_l4M[3];
      Je_tmp_2 = 0.0 / Je_tmp;
      q_l_dot_opt_tmp_4 = J_l4M[3] / Je_tmp;
      for (i_0 = 0; i_0 < 4; i_0++) {
        Je_tmp = J_l4M[i_0];
        N_b_tmp = i_0 << 2;
        q_l_dot_opt_tmp_5 = Je_tmp_2 * Je_tmp;
        N_d[N_b_tmp] = N_e_tmp[N_b_tmp] - q_l_dot_opt_tmp_5;
        N_d[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - q_l_dot_opt_tmp_5;
        N_d[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - q_l_dot_opt_tmp_5;
        N_d[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - q_l_dot_opt_tmp_4 * Je_tmp;
      }
    } else if (temp_act[static_cast<int32_T>(i) + 111] == 1) {
      rtb_Sum4_tmp_1 = rtU.K_l4m;
      Je_tmp_1 = -w_l4m;
      J_d[0] = 0.0;
      J_d[1] = 0.0;
      J_d[2] = 0.0;
      J_d[3] = J_l4m[3];
      Je_tmp = J_l4m[3] * J_l4m[3];
      Je_tmp_2 = 0.0 / Je_tmp;
      q_l_dot_opt_tmp_4 = J_l4m[3] / Je_tmp;
      for (i_0 = 0; i_0 < 4; i_0++) {
        Je_tmp = J_l4m[i_0];
        N_b_tmp = i_0 << 2;
        N_d[N_b_tmp] = N_e_tmp[N_b_tmp] - Je_tmp_2 * Je_tmp;
        N_d[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - Je_tmp_2 * Je_tmp;
        N_d[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - Je_tmp_2 * Je_tmp;
        N_d[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - q_l_dot_opt_tmp_4 * Je_tmp;
      }
    } else {
      rtb_Sum4_tmp_1 = 0.0;
      Je_tmp_1 = 0.0;
      J_d[0] = 0.0;
      J_d[1] = 0.0;
      J_d[2] = 0.0;
      J_d[3] = 1.0;
      std::memcpy(&N_d[0], &N_e_tmp[0], sizeof(real_T) << 4U);
    }

    Je_tmp = J_a[0] * J_a[0];
    q_l_dot_opt_tmp_4 = J_b[1] * J_b[1];
    q_l_dot_opt_tmp_5 = J_c[2] * J_c[2];
    q_l_dot_opt_tmp_6 = J_d[3] * J_d[3];
    if (std::abs(rtU.q4l) < 0.1) {
      for (i_0 = 0; i_0 < 4; i_0++) {
        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_0[N_a_tmp] = 0.0;
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp] * N_a[i_0];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 1] * N_a[i_0 + 4];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 2] * N_a[i_0 + 8];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 3] * N_a[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_1[N_a_tmp] = 0.0;
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp] * N_a_0[i_0];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 1] * N_a_0[i_0 + 4];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 2] * N_a_0[i_0 + 8];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 3] * N_a_0[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_2[N_a_tmp] = 0.0;
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp] * N_a_1[i_0];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 1] * N_a_1[i_0 + 4];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 2] * N_a_1[i_0 + 8];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 3] * N_a_1[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 3; N_b_tmp++) {
          N_a_tmp = (N_b_tmp << 2) + i_0;
          N_a_3[N_a_tmp] = 0.0;
          N_a_3[N_a_tmp] += N_a_2[i_0] * Je[N_b_tmp];
          N_a_3[N_a_tmp] += N_a_2[i_0 + 4] * Je[N_b_tmp + 3];
          N_a_3[N_a_tmp] += N_a_2[i_0 + 8] * Je[N_b_tmp + 6];
          N_a_3[N_a_tmp] += N_a_2[i_0 + 12] * Je[N_b_tmp + 9];
        }
      }

      for (i_0 = 0; i_0 < 9; i_0++) {
        N_e_tmp_2[i_0] = N_e_tmp_1[i_0] + static_cast<real_T>(g[i_0]);
      }

      mrdiv(N_a_3, N_e_tmp_2, tmp);
      for (i_0 = 0; i_0 <= 2; i_0 += 2) {
        tmp_2 = _mm_loadu_pd(&tmp[i_0]);
        tmp_1 = _mm_loadu_pd(&tmp[i_0 + 4]);
        tmp_0 = _mm_loadu_pd(&tmp[i_0 + 8]);
        _mm_storeu_pd(&rtb_q_r_dot[i_0], _mm_add_pd(_mm_mul_pd(tmp_0,
          _mm_set1_pd(rtb_Sum4[2])), _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd
          (rtb_Sum4[1])), _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(rtb_Sum4[0])),
          _mm_set1_pd(0.0)))));
      }
    } else {
      for (i_0 = 0; i_0 < 4; i_0++) {
        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_0[N_a_tmp] = 0.0;
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp] * N_a[i_0];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 1] * N_a[i_0 + 4];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 2] * N_a[i_0 + 8];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 3] * N_a[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_1[N_a_tmp] = 0.0;
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp] * N_a_0[i_0];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 1] * N_a_0[i_0 + 4];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 2] * N_a_0[i_0 + 8];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 3] * N_a_0[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_2[N_a_tmp] = 0.0;
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp] * N_a_1[i_0];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 1] * N_a_1[i_0 + 4];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 2] * N_a_1[i_0 + 8];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 3] * N_a_1[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 3; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_3[N_a_tmp] = 0.0;
          N_a_3[N_a_tmp] += N_e_tmp_0[N_a_tmp_tmp] * N_a_2[i_0];
          N_a_3[N_a_tmp] += N_e_tmp_0[N_a_tmp_tmp + 1] * N_a_2[i_0 + 4];
          N_a_3[N_a_tmp] += N_e_tmp_0[N_a_tmp_tmp + 2] * N_a_2[i_0 + 8];
          N_a_3[N_a_tmp] += N_e_tmp_0[N_a_tmp_tmp + 3] * N_a_2[i_0 + 12];
        }
      }

      mrdiv(N_a_3, N_e_tmp_1, tmp);
      for (i_0 = 0; i_0 <= 2; i_0 += 2) {
        tmp_2 = _mm_loadu_pd(&tmp[i_0]);
        tmp_1 = _mm_loadu_pd(&tmp[i_0 + 4]);
        tmp_0 = _mm_loadu_pd(&tmp[i_0 + 8]);
        _mm_storeu_pd(&rtb_q_r_dot[i_0], _mm_add_pd(_mm_mul_pd(tmp_0,
          _mm_set1_pd(rtb_Sum4[2])), _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd
          (rtb_Sum4[1])), _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(rtb_Sum4[0])),
          _mm_set1_pd(0.0)))));
      }
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      Je_tmp_2 = 0.0;
      q_l_dot_opt_tmp_7 = 0.0;
      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        converted = N_b_tmp << 2;
        N_a_tmp = converted + i_0;
        Je_tmp_2 += N_a[N_a_tmp] * J_b[N_b_tmp];
        N_a_0[N_a_tmp] = 0.0;
        q_l_dot_opt_tmp_tmp_6 = N_b[converted] * N_a[i_0];
        N_a_0[N_a_tmp] += q_l_dot_opt_tmp_tmp_6;
        N_a_tmp_0 = N_b[converted + 1] * N_a[i_0 + 4];
        N_a_0[N_a_tmp] += N_a_tmp_0;
        N_a_tmp_1 = N_b[converted + 2] * N_a[i_0 + 8];
        N_a_0[N_a_tmp] += N_a_tmp_1;
        N_a_tmp_2 = N_b[converted + 3] * N_a[i_0 + 12];
        N_a_0[N_a_tmp] += N_a_tmp_2;
        q_l_dot_opt_tmp_7 += N_a_0[N_a_tmp] * J_c[N_b_tmp];
        N_a_1[N_a_tmp] = 0.0;
        N_a_1[N_a_tmp] += q_l_dot_opt_tmp_tmp_6;
        N_a_1[N_a_tmp] += N_a_tmp_0;
        N_a_1[N_a_tmp] += N_a_tmp_1;
        N_a_1[N_a_tmp] += N_a_tmp_2;
      }

      N_a_4[i_0] = Je_tmp_2 / q_l_dot_opt_tmp_4;
      N_a_5[i_0] = q_l_dot_opt_tmp_7 / q_l_dot_opt_tmp_5;
      Je_tmp_2 = 0.0;
      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_2[N_a_tmp] = 0.0;
        N_a_2[N_a_tmp] += N_c[N_a_tmp_tmp] * N_a_1[i_0];
        N_a_2[N_a_tmp] += N_c[N_a_tmp_tmp + 1] * N_a_1[i_0 + 4];
        N_a_2[N_a_tmp] += N_c[N_a_tmp_tmp + 2] * N_a_1[i_0 + 8];
        N_a_2[N_a_tmp] += N_c[N_a_tmp_tmp + 3] * N_a_1[i_0 + 12];
        Je_tmp_2 += N_a_2[N_a_tmp] * J_d[N_b_tmp];
      }

      N_a_6[i_0] = Je_tmp_2 / q_l_dot_opt_tmp_6;
    }

    converted = 1;
    q_l_dot_opt_tmp_4 = 0.0;
    for (i_0 = 0; i_0 < 4; i_0++) {
      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_0[N_a_tmp] = 0.0;
        N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp] * N_a[i_0];
        N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 1] * N_a[i_0 + 4];
        N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 2] * N_a[i_0 + 8];
        N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 3] * N_a[i_0 + 12];
      }

      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_1[N_a_tmp] = 0.0;
        N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp] * N_a_0[i_0];
        N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 1] * N_a_0[i_0 + 4];
        N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 2] * N_a_0[i_0 + 8];
        N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 3] * N_a_0[i_0 + 12];
      }

      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_2[N_a_tmp] = 0.0;
        N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp] * N_a_1[i_0];
        N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 1] * N_a_1[i_0 + 4];
        N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 2] * N_a_1[i_0 + 8];
        N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 3] * N_a_1[i_0 + 12];
      }

      Je_tmp_2 = 0.0;
      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_7[N_a_tmp] = 0.0;
        N_a_7[N_a_tmp] += N_e[N_a_tmp_tmp] * N_a_2[i_0];
        N_a_7[N_a_tmp] += N_e[N_a_tmp_tmp + 1] * N_a_2[i_0 + 4];
        N_a_7[N_a_tmp] += N_e[N_a_tmp_tmp + 2] * N_a_2[i_0 + 8];
        N_a_7[N_a_tmp] += N_e[N_a_tmp_tmp + 3] * N_a_2[i_0 + 12];
        Je_tmp_2 += N_a_7[N_a_tmp] * q_l_dot_opt[N_b_tmp];
      }

      rtb_q_l_dot[i_0] = ((((J_a[i_0] / Je_tmp * j * k + N_a_4[i_0] * l *
        rtb_Sum4_tmp) + N_a_5[i_0] * rtb_Sum4_tmp_tmp * rtb_Sum4_tmp_0) +
                           N_a_6[i_0] * rtb_Sum4_tmp_1 * Je_tmp_1) +
                          rtb_q_r_dot[i_0]) + Je_tmp_2;
      q_l_dot_opt_tmp_4 += J_l1M[i_0] * rtb_q_l_dot[i_0];
    }

    if ((act_l1M == 1) && (q_l_dot_opt_tmp_4 >= 0.0)) {
      converted = 0;
    }

    if ((act_l2M == 1) && (((0.0 * rtb_q_l_dot[0] + J_l2M[1] * rtb_q_l_dot[1]) +
          0.0 * rtb_q_l_dot[2]) + 0.0 * rtb_q_l_dot[3] >= 0.0)) {
      converted = 0;
    }

    Je_tmp_2 = 0.0 * rtb_q_l_dot[0] + 0.0 * rtb_q_l_dot[1];
    if ((act_l3M == 1) && ((J_l3M[2] * rtb_q_l_dot[2] + Je_tmp_2) + 0.0 *
                           rtb_q_l_dot[3] >= 0.0)) {
      converted = 0;
    }

    q_l_dot_opt_tmp_7 = 0.0 * rtb_q_l_dot[2] + Je_tmp_2;
    if ((act_l4M == 1) && (J_l4M[3] * rtb_q_l_dot[3] + q_l_dot_opt_tmp_7 >= 0.0))
    {
      converted = 0;
    }

    if ((act_l1m == 1) && (((J_l1m[0] * rtb_q_l_dot[0] + 0.0 * rtb_q_l_dot[1]) +
          0.0 * rtb_q_l_dot[2]) + 0.0 * rtb_q_l_dot[3] <= 0.0)) {
      converted = 0;
    }

    if ((act_l2m == 1) && (((0.0 * rtb_q_l_dot[0] + J_l2m[1] * rtb_q_l_dot[1]) +
          0.0 * rtb_q_l_dot[2]) + 0.0 * rtb_q_l_dot[3] <= 0.0)) {
      converted = 0;
    }

    if ((act_l3m == 1) && ((J_l3m[2] * rtb_q_l_dot[2] + Je_tmp_2) + 0.0 *
                           rtb_q_l_dot[3] <= 0.0)) {
      converted = 0;
    }

    if ((act_l4m == 1) && (J_l4m[3] * rtb_q_l_dot[3] + q_l_dot_opt_tmp_7 <= 0.0))
    {
      converted = 0;
    }

    if (i == rt_powd_snf(2.0, static_cast<real_T>(tot_task))) {
      converted = 1;
    }

    i++;
  }

  // MinMax: '<S8>/MinMax' incorporates:
  //   Constant: '<S8>/Time constant'
  //   Gain: '<S8>/Minimum sampling to time constant ratio'

  rtb_MinMax = std::fmax(10.0 * rtDW.Probe_n[0], 0.1);

  // Sum: '<S1>/Add1' incorporates:
  //   Inport: '<Root>/L_half'
  //   Inport: '<Root>/xd'

  rtb_Add_idx_0 = rtU.xd[0] - 0.0;
  rtb_Add_idx_1 = rtU.xd[1] - rtU.L_half;
  rtb_Add_idx_2 = rtU.xd[2] - 0.0;

  // DiscreteIntegrator: '<S12>/Integrator'
  if (rtDW.Integrator_IC_LOADING_d != 0) {
    rtDW.Integrator_DSTATE_a[0] = rtb_Add_idx_0;
    rtDW.Integrator_DSTATE_a[1] = rtb_Add_idx_1;
    rtDW.Integrator_DSTATE_a[2] = rtb_Add_idx_2;
  }

  if (rtDW.Integrator_PrevResetState_f != 0) {
    rtDW.Integrator_DSTATE_a[0] = rtb_Add_idx_0;
    rtDW.Integrator_DSTATE_a[1] = rtb_Add_idx_1;
    rtDW.Integrator_DSTATE_a[2] = rtb_Add_idx_2;
  }

  // Product: '<S2>/1//T' incorporates:
  //   DiscreteIntegrator: '<S12>/Integrator'
  //   Sum: '<S2>/Sum1'

  rtb_Sum4[0] = 1.0 / rtb_MinMax * (rtb_Add_idx_0 - rtDW.Integrator_DSTATE_a[0]);

  // MATLAB Function: '<S1>/dirkin_r' incorporates:
  //   Inport: '<Root>/q1r'
  //   Inport: '<Root>/q2r'
  //   Inport: '<Root>/q3r'
  //   Inport: '<Root>/q4r'
  //   MATLAB Function: '<S1>/q_r_dot'

  i = 11.0 * std::cos(rtU.q2r) * std::sin(rtU.q1r);
  j = (std::sin(rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r) + std::cos
       (rtU.q1r) * std::cos(rtU.q3r)) * std::sin(rtU.q4r) / 4.0;
  k = std::cos(rtU.q2r) * std::cos(rtU.q4r) * std::sin(rtU.q1r) / 4.0;

  // Sum: '<S1>/Sum1' incorporates:
  //   MATLAB Function: '<S1>/dirkin_r'
  //   Sum: '<S2>/Sum1'

  rtb_Add_idx_0 -= (-i / 40.0 - j) - k;

  // Product: '<S2>/1//T' incorporates:
  //   DiscreteIntegrator: '<S12>/Integrator'
  //   Sum: '<S2>/Sum1'

  rtb_Sum4[1] = 1.0 / rtb_MinMax * (rtb_Add_idx_1 - rtDW.Integrator_DSTATE_a[1]);

  // Sum: '<S1>/Sum1' incorporates:
  //   Inport: '<Root>/q2r'
  //   Inport: '<Root>/q3r'
  //   Inport: '<Root>/q4r'
  //   MATLAB Function: '<S1>/dirkin_r'
  //   Sum: '<S2>/Sum1'

  rtb_Add_idx_1 -= ((11.0 * std::sin(rtU.q2r) / 40.0 + std::cos(rtU.q4r) * std::
                     sin(rtU.q2r) / 4.0) - std::cos(rtU.q2r) * std::sin(rtU.q3r)
                    * std::sin(rtU.q4r) / 4.0) - 0.18;

  // Product: '<S2>/1//T' incorporates:
  //   DiscreteIntegrator: '<S12>/Integrator'
  //   Sum: '<S2>/Sum1'

  rtb_Sum4[2] = 1.0 / rtb_MinMax * (rtb_Add_idx_2 - rtDW.Integrator_DSTATE_a[2]);

  // MATLAB Function: '<S1>/dirkin_r' incorporates:
  //   Inport: '<Root>/q1r'
  //   Inport: '<Root>/q2r'
  //   Inport: '<Root>/q3r'
  //   Inport: '<Root>/q4r'
  //   MATLAB Function: '<S1>/q_r_dot'

  l = ((std::cos(rtU.q3r) * std::sin(rtU.q1r) - std::cos(rtU.q1r) * std::sin
        (rtU.q2r) * std::sin(rtU.q3r)) * std::sin(rtU.q4r) / 4.0 - 11.0 * std::
       cos(rtU.q1r) * std::cos(rtU.q2r) / 40.0) - std::cos(rtU.q1r) * std::cos
    (rtU.q2r) * std::cos(rtU.q4r) / 4.0;

  // Sum: '<S1>/Sum1' incorporates:
  //   MATLAB Function: '<S1>/dirkin_r'
  //   Sum: '<S2>/Sum1'

  rtb_Add_idx_2 -= l;

  // End of Outputs for SubSystem: '<Root>/CLIK'
  for (i_0 = 0; i_0 <= 0; i_0 += 2) {
    // Outputs for Atomic SubSystem: '<Root>/CLIK'
    // Sum: '<S1>/Sum2' incorporates:
    //   Gain: '<S1>/Gain2'

    tmp_2 = _mm_loadu_pd(&rtb_Sum4[i_0]);
    _mm_storeu_pd(&rtb_Sum2[i_0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&rtConstP.pooled6[i_0 + 3]), _mm_set1_pd(rtb_Add_idx_1)),
      _mm_mul_pd(_mm_loadu_pd(&rtConstP.pooled6[i_0]), _mm_set1_pd(rtb_Add_idx_0))),
      _mm_mul_pd(_mm_loadu_pd(&rtConstP.pooled6[i_0 + 6]), _mm_set1_pd
                 (rtb_Add_idx_2))), tmp_2));

    // End of Outputs for SubSystem: '<Root>/CLIK'
  }

  // Outputs for Atomic SubSystem: '<Root>/CLIK'
  // Sum: '<S1>/Sum2' incorporates:
  //   Gain: '<S1>/Gain2'

  for (i_0 = 2; i_0 < 3; i_0++) {
    rtb_Sum2[i_0] = ((rtConstP.pooled6[i_0 + 3] * rtb_Add_idx_1 +
                      rtConstP.pooled6[i_0] * rtb_Add_idx_0) +
                     rtConstP.pooled6[i_0 + 6] * rtb_Add_idx_2) + rtb_Sum4[i_0];
  }

  // MATLAB Function: '<S1>/q_r_dot' incorporates:
  //   Inport: '<Root>/K_l1M'
  //   Inport: '<Root>/K_l1m'
  //   Inport: '<Root>/K_l2M'
  //   Inport: '<Root>/K_l2m'
  //   Inport: '<Root>/K_l3M'
  //   Inport: '<Root>/K_l3m'
  //   Inport: '<Root>/K_l4M'
  //   Inport: '<Root>/K_l4m'
  //   Inport: '<Root>/K_second'
  //   Inport: '<Root>/load'
  //   Inport: '<Root>/q1r'
  //   Inport: '<Root>/q2r'
  //   Inport: '<Root>/q3r'
  //   Inport: '<Root>/q4r'
  //   MATLAB Function: '<S1>/q_l_dot'

  rtb_MinMax = (rtU.q1r - 1.5207963267948965) / 3.1415926535897931;
  rtb_MinMax = rtb_MinMax * rtb_MinMax * 0.5;
  rtb_Add_idx_0 = (rtU.q2r - 0.29906585039886591) / 1.9198621771937625;
  rtb_Add_idx_0 = rtb_Add_idx_0 * rtb_Add_idx_0 * 0.5;
  rtb_Add_idx_1 = (rtU.q3r - 1.5207963267948965) / 3.1415926535897931;
  rtb_Add_idx_1 = rtb_Add_idx_1 * rtb_Add_idx_1 * 0.5;
  rtb_Add_idx_2 = (rtU.q4r - 2.5679938779914946) / 5.2359877559829888;
  rtb_Add_idx_2 = rtb_Add_idx_2 * rtb_Add_idx_2 * 0.5;
  w_l1m = (rtU.q1r - -1.5207963267948965) / 3.1415926535897931;
  w_l1m = w_l1m * w_l1m * 0.5;
  w_l2m = (rtU.q2r - -1.5207963267948965) / 1.9198621771937625;
  w_l2m = w_l2m * w_l2m * 0.5;
  w_l3m = (rtU.q3r - -1.5207963267948965) / 3.1415926535897931;
  w_l3m = w_l3m * w_l3m * 0.5;
  w_l4m = (rtU.q4r - -2.5679938779914946) / 5.2359877559829888;
  w_l4m = w_l4m * w_l4m * 0.5;
  J_l1M[0] = (2.0 * rtU.q1r - 3.041592653589793) / 19.739208802178716;
  J_l1M[1] = 0.0;
  J_l1M[2] = 0.0;
  J_l1M[3] = 0.0;
  J_l1m[0] = (2.0 * rtU.q1r + 3.041592653589793) / 19.739208802178716;
  J_l1m[1] = 0.0;
  J_l1m[2] = 0.0;
  J_l1m[3] = 0.0;
  J_l2M[0] = 0.0;
  J_l2M[1] = (2.0 * rtU.q2r - 0.59813170079773181) * 162.0 / 1194.2221325318123;
  J_l2M[2] = 0.0;
  J_l2M[3] = 0.0;
  J_l2m[0] = 0.0;
  J_l2m[1] = (2.0 * rtU.q2r + 3.041592653589793) * 162.0 / 1194.2221325318123;
  J_l2m[2] = 0.0;
  J_l2m[3] = 0.0;
  J_l3M[0] = 0.0;
  J_l3M[1] = 0.0;
  J_l3M[2] = (2.0 * rtU.q3r - 3.041592653589793) / 19.739208802178716;
  J_l3M[3] = 0.0;
  J_l3m[0] = 0.0;
  J_l3m[1] = 0.0;
  J_l3m[2] = (2.0 * rtU.q3r + 3.041592653589793) / 19.739208802178716;
  J_l3m[3] = 0.0;
  J_l4M[0] = 0.0;
  J_l4M[1] = 0.0;
  J_l4M[2] = 0.0;
  J_l4M[3] = (2.0 * rtU.q4r - 5.1359877559829892) * 9.0 / 493.48022005446791;
  J_l4m[0] = 0.0;
  J_l4m[1] = 0.0;
  J_l4m[2] = 0.0;
  J_l4m[3] = (2.0 * rtU.q4r + 5.1359877559829892) * 9.0 / 493.48022005446791;
  act_l1M = 0;
  act_l2M = 0;
  act_l3M = 0;
  act_l4M = 0;
  act_l1m = 0;
  act_l2m = 0;
  act_l3m = 0;
  act_l4m = 0;
  if (rtU.q1r >= 1.4707963267948965) {
    act_l1M = 1;
  } else if (rtU.q1r <= -1.4707963267948965) {
    act_l1m = 1;
  }

  if (rtU.q2r >= 0.24906585039886589) {
    act_l2M = 1;
  } else if (rtU.q2r <= -1.4707963267948965) {
    act_l2m = 1;
  }

  if (rtU.q3r >= 1.4707963267948965) {
    act_l3M = 1;
  } else if (rtU.q3r <= -1.4707963267948965) {
    act_l3m = 1;
  }

  if (rtU.q4r >= 2.5179938779914943) {
    act_l4M = 1;
  } else if (rtU.q4r <= -2.5179938779914943) {
    act_l4m = 1;
  }

  Je[0] = l;
  Je[3] = (11.0 * rtb_AvoidDividebyZero * q_l_dot_opt_tmp_tmp / 40.0 +
           q_l_dot_opt_tmp_0 * rtb_AvoidDividebyZero * q_l_dot_opt_tmp_tmp / 4.0)
    - q_l_dot_opt_tmp * rtb_AvoidDividebyZero * q_l_dot_opt_tmp_tmp_0 *
    q_l_dot_opt_tmp_tmp_3 / 4.0;
  Je[6] = (q_l_dot_opt_tmp_tmp_1 * q_l_dot_opt_tmp_tmp_0 -
           q_l_dot_opt_tmp_tmp_tmp * q_l_dot_opt_tmp_tmp) *
    q_l_dot_opt_tmp_tmp_3 / 4.0;
  Je[9] = std::cos(rtU.q2r) * std::sin(rtU.q1r) * q_l_dot_opt_tmp_tmp_3 / 4.0 -
    q_l_dot_opt_tmp_tmp_4 * q_l_dot_opt_tmp_0 / 4.0;
  Je[1] = 0.0;
  Je_tmp_1 = std::cos(rtU.q2r) * std::cos(rtU.q4r);
  Je[4] = (11.0 * std::cos(rtU.q2r) / 40.0 + Je_tmp_1 / 4.0) +
    q_l_dot_opt_tmp_tmp * q_l_dot_opt_tmp_tmp_0 * q_l_dot_opt_tmp_tmp_3 / 4.0;
  Je[7] = -(q_l_dot_opt_tmp * q_l_dot_opt_tmp_tmp_2 * q_l_dot_opt_tmp_tmp_3) /
    4.0;
  Je[10] = -(q_l_dot_opt_tmp_tmp * q_l_dot_opt_tmp_tmp_3) / 4.0 - Je_tmp_1 *
    q_l_dot_opt_tmp_tmp_0 / 4.0;
  Je[2] = (i / 40.0 + j) + k;
  Je_tmp = std::cos(rtU.q1r) * std::cos(rtU.q2r);
  Je[5] = (11.0 * std::cos(rtU.q1r) * q_l_dot_opt_tmp_tmp / 40.0 +
           q_l_dot_opt_tmp_tmp_1 * q_l_dot_opt_tmp_0 * q_l_dot_opt_tmp_tmp / 4.0)
    - Je_tmp * q_l_dot_opt_tmp_tmp_0 * q_l_dot_opt_tmp_tmp_3 / 4.0;
  Je[8] = -((q_l_dot_opt_tmp_tmp_tmp_0 * q_l_dot_opt_tmp_tmp +
             rtb_AvoidDividebyZero * q_l_dot_opt_tmp_tmp_0) *
            q_l_dot_opt_tmp_tmp_3) / 4.0;
  Je[11] = q_l_dot_opt_tmp_tmp_5 * q_l_dot_opt_tmp_0 / 4.0 + Je_tmp *
    q_l_dot_opt_tmp_tmp_3 / 4.0;
  eye(N_e_tmp);
  for (i_0 = 0; i_0 < 3; i_0++) {
    tot_task = i_0 << 2;
    N_e_tmp_0[tot_task] = Je[i_0];
    N_e_tmp_0[tot_task + 1] = Je[i_0 + 3];
    N_e_tmp_0[tot_task + 2] = Je[i_0 + 6];
    N_e_tmp_0[tot_task + 3] = Je[i_0 + 9];
    for (N_b_tmp = 0; N_b_tmp <= 0; N_b_tmp += 2) {
      converted = 3 * i_0 + N_b_tmp;
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_set1_pd(0.0));
      tmp_2 = _mm_loadu_pd(&Je[N_b_tmp]);
      tmp_1 = _mm_loadu_pd(&N_e_tmp_1[converted]);
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_add_pd(tmp_1, _mm_mul_pd
        (_mm_set1_pd(N_e_tmp_0[tot_task]), tmp_2)));
      tmp_2 = _mm_loadu_pd(&Je[N_b_tmp + 3]);
      tmp_1 = _mm_loadu_pd(&N_e_tmp_1[converted]);
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (N_e_tmp_0[tot_task + 1]), tmp_2), tmp_1));
      tmp_2 = _mm_loadu_pd(&Je[N_b_tmp + 6]);
      tmp_1 = _mm_loadu_pd(&N_e_tmp_1[converted]);
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (N_e_tmp_0[tot_task + 2]), tmp_2), tmp_1));
      tmp_2 = _mm_loadu_pd(&Je[N_b_tmp + 9]);
      tmp_1 = _mm_loadu_pd(&N_e_tmp_1[converted]);
      _mm_storeu_pd(&N_e_tmp_1[converted], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (N_e_tmp_0[tot_task + 3]), tmp_2), tmp_1));
    }

    for (N_b_tmp = 2; N_b_tmp < 3; N_b_tmp++) {
      converted = 3 * i_0 + N_b_tmp;
      N_e_tmp_1[converted] = 0.0;
      N_e_tmp_1[converted] += N_e_tmp_0[tot_task] * Je[N_b_tmp];
      N_e_tmp_1[converted] += N_e_tmp_0[tot_task + 1] * Je[N_b_tmp + 3];
      N_e_tmp_1[converted] += N_e_tmp_0[tot_task + 2] * Je[N_b_tmp + 6];
      N_e_tmp_1[converted] += N_e_tmp_0[tot_task + 3] * Je[N_b_tmp + 9];
    }
  }

  mrdiv(N_e_tmp_0, N_e_tmp_1, tmp);
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
      tot_task = (N_b_tmp << 2) + i_0;
      N_e[tot_task] = N_e_tmp[tot_task] - ((Je[3 * N_b_tmp + 1] * tmp[i_0 + 4] +
        Je[3 * N_b_tmp] * tmp[i_0]) + Je[3 * N_b_tmp + 2] * tmp[i_0 + 8]);
    }
  }

  q_l_dot_opt[0] = (((((11.0 * std::cos(rtU.q1r) * std::cos(rtU.q2r) / 80.0 -
                        (std::cos(rtU.q3r) * std::sin(rtU.q1r) - std::cos
    (rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r)) * std::sin(rtU.q4r) / 8.0)
                       + std::cos(rtU.q1r) * std::cos(rtU.q2r) * std::cos
                       (rtU.q4r) / 8.0) * rtU.load + (((10879.0 * std::cos
    (rtU.q1r) / 5.0E+6 - 3.0 * std::sin(rtU.q1r) / 800.0) + 10481.0 * std::cos
    (rtU.q1r) * std::cos(rtU.q2r) / 100000.0) - q_l_dot_opt_tmp_9)) +
                     7.409300489671729E+15 * std::cos(rtU.q1r) * std::cos
                     (rtU.q2r) * std::cos(rtU.q4r) / 7.2057594037927936E+17) *
                    q_l_dot_opt_tmp_1 / q_l_dot_opt_tmp_8 + (((((std::sin
    (rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r) + std::cos(rtU.q1r) * std::
    cos(rtU.q3r)) * std::sin(rtU.q4r) / 8.0 + 11.0 * std::cos(rtU.q2r) * std::
    sin(rtU.q1r) / 80.0) + std::cos(rtU.q2r) * std::cos(rtU.q4r) * std::sin
    (rtU.q1r) / 8.0) * rtU.load + ((std::sin(rtU.q1r) * std::sin(rtU.q2r) * std::
    sin(rtU.q3r) + std::cos(rtU.q1r) * std::cos(rtU.q3r)) *
    (7.409300489671729E+15 * std::sin(rtU.q4r)) / 7.2057594037927936E+17 + ((3.0
    * std::cos(rtU.q1r) / 800.0 + 10879.0 * std::sin(rtU.q1r) / 5.0E+6) +
    10481.0 * std::cos(rtU.q2r) * std::sin(rtU.q1r) / 100000.0))) +
    7.409300489671729E+15 * std::cos(rtU.q2r) * std::cos(rtU.q4r) * std::sin
    (rtU.q1r) / 7.2057594037927936E+17) * q_l_dot_opt_tmp_3 / q_l_dot_opt_tmp_8)
    * rtU.K_second;
  q_l_dot_opt_tmp_5 = 7.409300489671729E+15 * std::cos(rtU.q1r) * std::cos
    (rtU.q2r);
  q_l_dot_opt_tmp_4 = 7.409300489671729E+15 * std::cos(rtU.q2r) * std::cos
    (rtU.q4r);
  q_l_dot_opt[1] = ((((((11.0 * std::cos(rtU.q2r) / 80.0 + Je_tmp_1 / 8.0) + std::
                        sin(rtU.q2r) * std::sin(rtU.q3r) * std::sin(rtU.q4r) /
                        8.0) * rtU.load + (10481.0 * std::cos(rtU.q2r) /
    100000.0 + q_l_dot_opt_tmp_4 / 7.2057594037927936E+17)) +
                      7.409300489671729E+15 * q_l_dot_opt_tmp_tmp *
                      q_l_dot_opt_tmp_tmp_0 * q_l_dot_opt_tmp_tmp_3 /
                      7.2057594037927936E+17) * Je_tmp_0 / q_l_dot_opt_tmp_2 -
                     (((((11.0 * std::cos(rtU.q1r) * std::sin(rtU.q2r) / 80.0 +
    std::cos(rtU.q1r) * std::cos(rtU.q4r) * std::sin(rtU.q2r) / 8.0) - std::cos
    (rtU.q1r) * std::cos(rtU.q2r) * std::sin(rtU.q3r) * std::sin(rtU.q4r) / 8.0)
                        * rtU.load + 10481.0 * std::cos(rtU.q1r) *
                        q_l_dot_opt_tmp_tmp / 100000.0) + 7.409300489671729E+15 *
                       std::cos(rtU.q1r) * q_l_dot_opt_tmp_0 *
                       q_l_dot_opt_tmp_tmp / 7.2057594037927936E+17) -
                      q_l_dot_opt_tmp_5 * q_l_dot_opt_tmp_tmp_0 *
                      q_l_dot_opt_tmp_tmp_3 / 7.2057594037927936E+17) *
                     q_l_dot_opt_tmp_3 / q_l_dot_opt_tmp_8) + (((((11.0 * std::
    sin(rtU.q1r) * std::sin(rtU.q2r) / 80.0 + std::cos(rtU.q4r) * std::sin
    (rtU.q1r) * std::sin(rtU.q2r) / 8.0) - std::cos(rtU.q2r) * std::sin(rtU.q1r)
    * std::sin(rtU.q3r) * std::sin(rtU.q4r) / 8.0) * rtU.load + 10481.0 *
    rtb_AvoidDividebyZero * q_l_dot_opt_tmp_tmp / 100000.0) +
    7.409300489671729E+15 * std::cos(rtU.q4r) * rtb_AvoidDividebyZero *
    q_l_dot_opt_tmp_tmp / 7.2057594037927936E+17) - 7.409300489671729E+15 * std::
    cos(rtU.q2r) * rtb_AvoidDividebyZero * q_l_dot_opt_tmp_tmp_0 *
    q_l_dot_opt_tmp_tmp_3 / 7.2057594037927936E+17) * q_l_dot_opt_tmp_1 /
                    q_l_dot_opt_tmp_8) * -rtU.K_second;
  q_l_dot_opt_tmp_0 = std::cos(rtU.q1r) * std::sin(rtU.q3r) - std::cos(rtU.q3r) *
    std::sin(rtU.q1r) * std::sin(rtU.q2r);
  q_l_dot_opt_tmp_6 = std::cos(rtU.q1r) * std::cos(rtU.q3r) * std::sin(rtU.q2r)
    + std::sin(rtU.q1r) * std::sin(rtU.q3r);
  q_l_dot_opt[2] = (((rtU.load * q_l_dot_opt_tmp_tmp_3 * q_l_dot_opt_tmp_0 / 8.0
                      + q_l_dot_opt_tmp_0 * q_l_dot_opt_tmp_tmp_7 /
                      7.2057594037927936E+17) * q_l_dot_opt_tmp_1 /
                     q_l_dot_opt_tmp_8 - (7.409300489671729E+15 * std::cos
    (rtU.q2r) * q_l_dot_opt_tmp_tmp_2 * q_l_dot_opt_tmp_tmp_3 /
    7.2057594037927936E+17 + rtU.load * q_l_dot_opt_tmp * q_l_dot_opt_tmp_tmp_2 *
    q_l_dot_opt_tmp_tmp_3 / 8.0) * Je_tmp_0 / q_l_dot_opt_tmp_2) + (rtU.load *
    std::sin(rtU.q4r) * q_l_dot_opt_tmp_6 / 8.0 + q_l_dot_opt_tmp_6 *
    q_l_dot_opt_tmp_tmp_7 / 7.2057594037927936E+17) * q_l_dot_opt_tmp_3 /
                    q_l_dot_opt_tmp_8) * -rtU.K_second;
  q_l_dot_opt[3] = ((((((std::cos(rtU.q3r) * std::sin(rtU.q1r) - std::cos
    (rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r)) * std::cos(rtU.q4r) / 8.0
                        + std::cos(rtU.q1r) * std::cos(rtU.q2r) * std::sin
                        (rtU.q4r) / 8.0) * rtU.load + 7.409300489671729E+15 *
                       std::cos(rtU.q4r) * q_l_dot_opt_tmp_tmp_5 /
                       7.2057594037927936E+17) + q_l_dot_opt_tmp_5 *
                      q_l_dot_opt_tmp_tmp_3 / 7.2057594037927936E+17) *
                     q_l_dot_opt_tmp_3 / q_l_dot_opt_tmp_8 + ((((std::sin
    (rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r) + std::cos(rtU.q1r) * std::
    cos(rtU.q3r)) * std::cos(rtU.q4r) / 8.0 - std::cos(rtU.q2r) * std::sin
    (rtU.q1r) * std::sin(rtU.q4r) / 8.0) * rtU.load + 7.409300489671729E+15 *
    std::cos(rtU.q4r) * q_l_dot_opt_tmp_tmp_4 / 7.2057594037927936E+17) -
    7.409300489671729E+15 * std::cos(rtU.q2r) * std::sin(rtU.q1r) *
    q_l_dot_opt_tmp_tmp_3 / 7.2057594037927936E+17) * q_l_dot_opt_tmp_1 /
                     q_l_dot_opt_tmp_8) + (((std::cos(rtU.q2r) * std::cos
    (rtU.q4r) * std::sin(rtU.q3r) / 8.0 + std::sin(rtU.q2r) * std::sin(rtU.q4r) /
    8.0) * rtU.load + 7.409300489671729E+15 * std::sin(rtU.q2r) *
    q_l_dot_opt_tmp_tmp_3 / 7.2057594037927936E+17) + q_l_dot_opt_tmp_4 *
    q_l_dot_opt_tmp_tmp_0 / 7.2057594037927936E+17) * Je_tmp_0 /
                    q_l_dot_opt_tmp_2) * rtU.K_second;
  tot_task = ((((((act_l1M + act_l1m) + act_l2M) + act_l2m) + act_l3M) + act_l3m)
              + act_l4M) + act_l4m;
  switch (tot_task) {
   case 0:
    temp_act[0] = static_cast<int8_T>(act_l1M);
    temp_act[16] = static_cast<int8_T>(act_l1m);
    temp_act[32] = static_cast<int8_T>(act_l2M);
    temp_act[48] = static_cast<int8_T>(act_l2m);
    temp_act[64] = static_cast<int8_T>(act_l3M);
    temp_act[80] = static_cast<int8_T>(act_l3m);
    temp_act[96] = static_cast<int8_T>(act_l4M);
    temp_act[112] = static_cast<int8_T>(act_l4m);
    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    break;

   case 1:
    for (i_0 = 0; i_0 < 8; i_0++) {
      temp_act[i_0 << 4] = 0;
    }

    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    break;

   case 2:
    for (i_0 = 0; i_0 < 8; i_0++) {
      temp_act[i_0 << 4] = 0;
    }

    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    i = 1.0;
    do {
      exitg1 = 0;
      i_0 = ((static_cast<int32_T>(i) - 1) << 4) + 1;
      if (temp_act[i_0] == 1) {
        exitg1 = 1;
      } else {
        i++;
      }
    } while (exitg1 == 0);

    temp_act[i_0] = 0;
    j = 1.0;
    do {
      exitg1 = 0;
      i_0 = ((static_cast<int32_T>(j) - 1) << 4) + 2;
      if ((temp_act[i_0] == 1) && (j != i)) {
        exitg1 = 1;
      } else {
        j++;
      }
    } while (exitg1 == 0);

    temp_act[i_0] = 0;
    break;

   case 3:
    for (i_0 = 0; i_0 < 8; i_0++) {
      temp_act[i_0 << 4] = 0;
    }

    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    i = 1.0;
    do {
      exitg1 = 0;
      i_0 = (static_cast<int32_T>(i) - 1) << 4;
      if (temp_act[i_0 + 6] == 1) {
        exitg1 = 1;
      } else {
        i++;
      }
    } while (exitg1 == 0);

    temp_act[i_0 + 6] = 0;
    j = 1.0;
    do {
      exitg1 = 0;
      N_b_tmp = (static_cast<int32_T>(j) - 1) << 4;
      if ((temp_act[N_b_tmp + 5] == 1) && (j != i)) {
        exitg1 = 1;
      } else {
        j++;
      }
    } while (exitg1 == 0);

    temp_act[N_b_tmp + 5] = 0;
    k = 1.0;
    do {
      exitg1 = 0;
      converted = (static_cast<int32_T>(k) - 1) << 4;
      if ((temp_act[converted + 4] == 1) && ((k != i) && (k != j))) {
        exitg1 = 1;
      } else {
        k++;
      }
    } while (exitg1 == 0);

    temp_act[converted + 4] = 0;
    temp_act[i_0 + 3] = 0;
    temp_act[N_b_tmp + 3] = 0;
    temp_act[i_0 + 2] = 0;
    temp_act[converted + 2] = 0;
    temp_act[N_b_tmp + 1] = 0;
    temp_act[converted + 1] = 0;
    break;

   default:
    for (i_0 = 0; i_0 < 8; i_0++) {
      temp_act[i_0 << 4] = 0;
    }

    temp_act[1] = static_cast<int8_T>(act_l1M);
    temp_act[17] = static_cast<int8_T>(act_l1m);
    temp_act[33] = static_cast<int8_T>(act_l2M);
    temp_act[49] = static_cast<int8_T>(act_l2m);
    temp_act[65] = static_cast<int8_T>(act_l3M);
    temp_act[81] = static_cast<int8_T>(act_l3m);
    temp_act[97] = static_cast<int8_T>(act_l4M);
    temp_act[113] = static_cast<int8_T>(act_l4m);
    temp_act[2] = static_cast<int8_T>(act_l1M);
    temp_act[18] = static_cast<int8_T>(act_l1m);
    temp_act[34] = static_cast<int8_T>(act_l2M);
    temp_act[50] = static_cast<int8_T>(act_l2m);
    temp_act[66] = static_cast<int8_T>(act_l3M);
    temp_act[82] = static_cast<int8_T>(act_l3m);
    temp_act[98] = static_cast<int8_T>(act_l4M);
    temp_act[114] = static_cast<int8_T>(act_l4m);
    temp_act[3] = static_cast<int8_T>(act_l1M);
    temp_act[19] = static_cast<int8_T>(act_l1m);
    temp_act[35] = static_cast<int8_T>(act_l2M);
    temp_act[51] = static_cast<int8_T>(act_l2m);
    temp_act[67] = static_cast<int8_T>(act_l3M);
    temp_act[83] = static_cast<int8_T>(act_l3m);
    temp_act[99] = static_cast<int8_T>(act_l4M);
    temp_act[115] = static_cast<int8_T>(act_l4m);
    temp_act[4] = static_cast<int8_T>(act_l1M);
    temp_act[20] = static_cast<int8_T>(act_l1m);
    temp_act[36] = static_cast<int8_T>(act_l2M);
    temp_act[52] = static_cast<int8_T>(act_l2m);
    temp_act[68] = static_cast<int8_T>(act_l3M);
    temp_act[84] = static_cast<int8_T>(act_l3m);
    temp_act[100] = static_cast<int8_T>(act_l4M);
    temp_act[116] = static_cast<int8_T>(act_l4m);
    temp_act[5] = static_cast<int8_T>(act_l1M);
    temp_act[21] = static_cast<int8_T>(act_l1m);
    temp_act[37] = static_cast<int8_T>(act_l2M);
    temp_act[53] = static_cast<int8_T>(act_l2m);
    temp_act[69] = static_cast<int8_T>(act_l3M);
    temp_act[85] = static_cast<int8_T>(act_l3m);
    temp_act[101] = static_cast<int8_T>(act_l4M);
    temp_act[117] = static_cast<int8_T>(act_l4m);
    temp_act[6] = static_cast<int8_T>(act_l1M);
    temp_act[22] = static_cast<int8_T>(act_l1m);
    temp_act[38] = static_cast<int8_T>(act_l2M);
    temp_act[54] = static_cast<int8_T>(act_l2m);
    temp_act[70] = static_cast<int8_T>(act_l3M);
    temp_act[86] = static_cast<int8_T>(act_l3m);
    temp_act[102] = static_cast<int8_T>(act_l4M);
    temp_act[118] = static_cast<int8_T>(act_l4m);
    temp_act[7] = static_cast<int8_T>(act_l1M);
    temp_act[23] = static_cast<int8_T>(act_l1m);
    temp_act[39] = static_cast<int8_T>(act_l2M);
    temp_act[55] = static_cast<int8_T>(act_l2m);
    temp_act[71] = static_cast<int8_T>(act_l3M);
    temp_act[87] = static_cast<int8_T>(act_l3m);
    temp_act[103] = static_cast<int8_T>(act_l4M);
    temp_act[119] = static_cast<int8_T>(act_l4m);
    temp_act[8] = static_cast<int8_T>(act_l1M);
    temp_act[24] = static_cast<int8_T>(act_l1m);
    temp_act[40] = static_cast<int8_T>(act_l2M);
    temp_act[56] = static_cast<int8_T>(act_l2m);
    temp_act[72] = static_cast<int8_T>(act_l3M);
    temp_act[88] = static_cast<int8_T>(act_l3m);
    temp_act[104] = static_cast<int8_T>(act_l4M);
    temp_act[120] = static_cast<int8_T>(act_l4m);
    temp_act[9] = static_cast<int8_T>(act_l1M);
    temp_act[25] = static_cast<int8_T>(act_l1m);
    temp_act[41] = static_cast<int8_T>(act_l2M);
    temp_act[57] = static_cast<int8_T>(act_l2m);
    temp_act[73] = static_cast<int8_T>(act_l3M);
    temp_act[89] = static_cast<int8_T>(act_l3m);
    temp_act[105] = static_cast<int8_T>(act_l4M);
    temp_act[121] = static_cast<int8_T>(act_l4m);
    temp_act[10] = static_cast<int8_T>(act_l1M);
    temp_act[26] = static_cast<int8_T>(act_l1m);
    temp_act[42] = static_cast<int8_T>(act_l2M);
    temp_act[58] = static_cast<int8_T>(act_l2m);
    temp_act[74] = static_cast<int8_T>(act_l3M);
    temp_act[90] = static_cast<int8_T>(act_l3m);
    temp_act[106] = static_cast<int8_T>(act_l4M);
    temp_act[122] = static_cast<int8_T>(act_l4m);
    temp_act[11] = static_cast<int8_T>(act_l1M);
    temp_act[27] = static_cast<int8_T>(act_l1m);
    temp_act[43] = static_cast<int8_T>(act_l2M);
    temp_act[59] = static_cast<int8_T>(act_l2m);
    temp_act[75] = static_cast<int8_T>(act_l3M);
    temp_act[91] = static_cast<int8_T>(act_l3m);
    temp_act[107] = static_cast<int8_T>(act_l4M);
    temp_act[123] = static_cast<int8_T>(act_l4m);
    temp_act[12] = static_cast<int8_T>(act_l1M);
    temp_act[28] = static_cast<int8_T>(act_l1m);
    temp_act[44] = static_cast<int8_T>(act_l2M);
    temp_act[60] = static_cast<int8_T>(act_l2m);
    temp_act[76] = static_cast<int8_T>(act_l3M);
    temp_act[92] = static_cast<int8_T>(act_l3m);
    temp_act[108] = static_cast<int8_T>(act_l4M);
    temp_act[124] = static_cast<int8_T>(act_l4m);
    temp_act[13] = static_cast<int8_T>(act_l1M);
    temp_act[29] = static_cast<int8_T>(act_l1m);
    temp_act[45] = static_cast<int8_T>(act_l2M);
    temp_act[61] = static_cast<int8_T>(act_l2m);
    temp_act[77] = static_cast<int8_T>(act_l3M);
    temp_act[93] = static_cast<int8_T>(act_l3m);
    temp_act[109] = static_cast<int8_T>(act_l4M);
    temp_act[125] = static_cast<int8_T>(act_l4m);
    temp_act[14] = static_cast<int8_T>(act_l1M);
    temp_act[30] = static_cast<int8_T>(act_l1m);
    temp_act[46] = static_cast<int8_T>(act_l2M);
    temp_act[62] = static_cast<int8_T>(act_l2m);
    temp_act[78] = static_cast<int8_T>(act_l3M);
    temp_act[94] = static_cast<int8_T>(act_l3m);
    temp_act[110] = static_cast<int8_T>(act_l4M);
    temp_act[126] = static_cast<int8_T>(act_l4m);
    temp_act[15] = static_cast<int8_T>(act_l1M);
    temp_act[31] = static_cast<int8_T>(act_l1m);
    temp_act[47] = static_cast<int8_T>(act_l2M);
    temp_act[63] = static_cast<int8_T>(act_l2m);
    temp_act[79] = static_cast<int8_T>(act_l3M);
    temp_act[95] = static_cast<int8_T>(act_l3m);
    temp_act[111] = static_cast<int8_T>(act_l4M);
    temp_act[127] = static_cast<int8_T>(act_l4m);
    i = 1.0;
    do {
      exitg1 = 0;
      i_0 = (static_cast<int32_T>(i) - 1) << 4;
      if (temp_act[i_0 + 14] == 1) {
        exitg1 = 1;
      } else {
        i++;
      }
    } while (exitg1 == 0);

    temp_act[i_0 + 14] = 0;
    j = 1.0;
    do {
      exitg1 = 0;
      N_b_tmp = (static_cast<int32_T>(j) - 1) << 4;
      if ((temp_act[N_b_tmp + 13] == 1) && (j != i)) {
        exitg1 = 1;
      } else {
        j++;
      }
    } while (exitg1 == 0);

    temp_act[N_b_tmp + 13] = 0;
    k = 1.0;
    do {
      exitg1 = 0;
      converted = (static_cast<int32_T>(k) - 1) << 4;
      if ((temp_act[converted + 12] == 1) && ((k != i) && (k != j))) {
        exitg1 = 1;
      } else {
        k++;
      }
    } while (exitg1 == 0);

    temp_act[converted + 12] = 0;
    l = 1.0;
    do {
      exitg1 = 0;
      N_a_tmp = (static_cast<int32_T>(l) - 1) << 4;
      if ((temp_act[N_a_tmp + 11] == 1) && ((l != i) && (l != j) && (l != k))) {
        exitg1 = 1;
      } else {
        l++;
      }
    } while (exitg1 == 0);

    temp_act[N_a_tmp + 11] = 0;
    temp_act[i_0 + 10] = 0;
    temp_act[N_b_tmp + 10] = 0;
    temp_act[i_0 + 9] = 0;
    temp_act[converted + 9] = 0;
    temp_act[i_0 + 8] = 0;
    temp_act[N_a_tmp + 8] = 0;
    temp_act[N_b_tmp + 7] = 0;
    temp_act[converted + 7] = 0;
    temp_act[N_b_tmp + 6] = 0;
    temp_act[N_a_tmp + 6] = 0;
    temp_act[converted + 5] = 0;
    temp_act[N_a_tmp + 5] = 0;
    temp_act[i_0 + 4] = 0;
    temp_act[N_b_tmp + 4] = 0;
    temp_act[converted + 4] = 0;
    temp_act[N_b_tmp + 3] = 0;
    temp_act[converted + 3] = 0;
    temp_act[N_a_tmp + 3] = 0;
    temp_act[converted + 2] = 0;
    temp_act[N_a_tmp + 2] = 0;
    temp_act[i_0 + 2] = 0;
    temp_act[N_a_tmp + 1] = 0;
    temp_act[i_0 + 1] = 0;
    temp_act[N_b_tmp + 1] = 0;
    break;
  }

  converted = 0;
  rtb_q_r_dot[0] = 0.0;
  rtb_q_r_dot[1] = 0.0;
  rtb_q_r_dot[2] = 0.0;
  rtb_q_r_dot[3] = 0.0;
  i = 1.0;
  while (converted == 0) {
    if (temp_act[static_cast<int32_T>(i) - 1] == 1) {
      j = rtU.K_l1M;
      k = -rtb_MinMax;
      J_a[0] = J_l1M[0];
      q_l_dot_opt_tmp_4 = J_l1M[0] * J_l1M[0];
      J_a[1] = 0.0;
      J_a[2] = 0.0;
      J_a[3] = 0.0;
      l = J_l1M[0] / q_l_dot_opt_tmp_4;
      rtb_Sum4_tmp = 0.0 / q_l_dot_opt_tmp_4;
      for (i_0 = 0; i_0 < 4; i_0++) {
        q_l_dot_opt_tmp_4 = J_l1M[i_0];
        N_a_tmp = i_0 << 2;
        N_a[N_a_tmp] = N_e_tmp[N_a_tmp] - l * q_l_dot_opt_tmp_4;
        q_l_dot_opt_tmp_tmp_6 = rtb_Sum4_tmp * q_l_dot_opt_tmp_4;
        N_a[N_a_tmp + 1] = N_e_tmp[N_a_tmp + 1] - q_l_dot_opt_tmp_tmp_6;
        N_a[N_a_tmp + 2] = N_e_tmp[N_a_tmp + 2] - q_l_dot_opt_tmp_tmp_6;
        N_a[N_a_tmp + 3] = N_e_tmp[N_a_tmp + 3] - q_l_dot_opt_tmp_tmp_6;
      }
    } else if (temp_act[static_cast<int32_T>(i) + 15] == 1) {
      j = rtU.K_l1m;
      k = -w_l1m;
      J_a[0] = J_l1m[0];
      l = J_l1m[0] * J_l1m[0];
      J_a[1] = 0.0;
      J_a[2] = 0.0;
      J_a[3] = 0.0;
      rtb_Sum4_tmp = J_l1m[0] / l;
      rtb_Sum4_tmp_tmp = 0.0 / l;
      for (i_0 = 0; i_0 < 4; i_0++) {
        l = J_l1m[i_0];
        N_a_tmp = i_0 << 2;
        N_a[N_a_tmp] = N_e_tmp[N_a_tmp] - rtb_Sum4_tmp * l;
        q_l_dot_opt_tmp_tmp_6 = rtb_Sum4_tmp_tmp * l;
        N_a[N_a_tmp + 1] = N_e_tmp[N_a_tmp + 1] - q_l_dot_opt_tmp_tmp_6;
        N_a[N_a_tmp + 2] = N_e_tmp[N_a_tmp + 2] - q_l_dot_opt_tmp_tmp_6;
        N_a[N_a_tmp + 3] = N_e_tmp[N_a_tmp + 3] - rtb_Sum4_tmp_tmp * l;
      }
    } else {
      j = 0.0;
      k = 0.0;
      J_a[0] = 1.0;
      J_a[1] = 0.0;
      J_a[2] = 0.0;
      J_a[3] = 0.0;
      std::memcpy(&N_a[0], &N_e_tmp[0], sizeof(real_T) << 4U);
    }

    if (temp_act[static_cast<int32_T>(i) + 31] == 1) {
      l = rtU.K_l2M;
      rtb_Sum4_tmp = -rtb_Add_idx_0;
      J_b[0] = 0.0;
      J_b[1] = J_l2M[1];
      rtb_Sum4_tmp_tmp = J_l2M[1] * J_l2M[1];
      J_b[2] = 0.0;
      J_b[3] = 0.0;
      rtb_Sum4_tmp_0 = 0.0 / rtb_Sum4_tmp_tmp;
      rtb_Sum4_tmp_1 = J_l2M[1] / rtb_Sum4_tmp_tmp;
      for (i_0 = 0; i_0 < 4; i_0++) {
        rtb_Sum4_tmp_tmp = J_l2M[i_0];
        N_b_tmp = i_0 << 2;
        N_b[N_b_tmp] = N_e_tmp[N_b_tmp] - rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - rtb_Sum4_tmp_1 *
          rtb_Sum4_tmp_tmp;
        Je_tmp_1 = rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - Je_tmp_1;
        N_b[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - Je_tmp_1;
      }
    } else if (temp_act[static_cast<int32_T>(i) + 47] == 1) {
      l = rtU.K_l2m;
      rtb_Sum4_tmp = -w_l2m;
      J_b[0] = 0.0;
      J_b[1] = J_l2m[1];
      rtb_Sum4_tmp_tmp = J_l2m[1] * J_l2m[1];
      J_b[2] = 0.0;
      J_b[3] = 0.0;
      rtb_Sum4_tmp_0 = 0.0 / rtb_Sum4_tmp_tmp;
      rtb_Sum4_tmp_1 = J_l2m[1] / rtb_Sum4_tmp_tmp;
      for (i_0 = 0; i_0 < 4; i_0++) {
        rtb_Sum4_tmp_tmp = J_l2m[i_0];
        N_b_tmp = i_0 << 2;
        N_b[N_b_tmp] = N_e_tmp[N_b_tmp] - rtb_Sum4_tmp_0 * rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - rtb_Sum4_tmp_1 *
          rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - rtb_Sum4_tmp_0 *
          rtb_Sum4_tmp_tmp;
        N_b[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - rtb_Sum4_tmp_0 *
          rtb_Sum4_tmp_tmp;
      }
    } else {
      l = 0.0;
      rtb_Sum4_tmp = 0.0;
      J_b[0] = 0.0;
      J_b[1] = 1.0;
      J_b[2] = 0.0;
      J_b[3] = 0.0;
      std::memcpy(&N_b[0], &N_e_tmp[0], sizeof(real_T) << 4U);
    }

    if (temp_act[static_cast<int32_T>(i) + 63] == 1) {
      rtb_Sum4_tmp_tmp = rtU.K_l3M;
      rtb_Sum4_tmp_0 = -rtb_Add_idx_1;
      J_c[0] = 0.0;
      J_c[1] = 0.0;
      J_c[2] = J_l3M[2];
      rtb_Sum4_tmp_1 = J_l3M[2] * J_l3M[2];
      J_c[3] = 0.0;
      Je_tmp_1 = 0.0 / rtb_Sum4_tmp_1;
      Je_tmp = J_l3M[2] / rtb_Sum4_tmp_1;
      for (i_0 = 0; i_0 < 4; i_0++) {
        rtb_Sum4_tmp_1 = J_l3M[i_0];
        N_b_tmp = i_0 << 2;
        N_c[N_b_tmp] = N_e_tmp[N_b_tmp] - Je_tmp_1 * rtb_Sum4_tmp_1;
        Je_tmp_2 = Je_tmp_1 * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - Je_tmp_2;
        N_c[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - Je_tmp * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - Je_tmp_2;
      }
    } else if (temp_act[static_cast<int32_T>(i) + 79] == 1) {
      rtb_Sum4_tmp_tmp = rtU.K_l3m;
      rtb_Sum4_tmp_0 = -w_l3m;
      J_c[0] = 0.0;
      J_c[1] = 0.0;
      J_c[2] = J_l3m[2];
      rtb_Sum4_tmp_1 = J_l3m[2] * J_l3m[2];
      J_c[3] = 0.0;
      Je_tmp_1 = 0.0 / rtb_Sum4_tmp_1;
      Je_tmp = J_l3m[2] / rtb_Sum4_tmp_1;
      for (i_0 = 0; i_0 < 4; i_0++) {
        rtb_Sum4_tmp_1 = J_l3m[i_0];
        N_b_tmp = i_0 << 2;
        N_c[N_b_tmp] = N_e_tmp[N_b_tmp] - Je_tmp_1 * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - Je_tmp_1 * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - Je_tmp * rtb_Sum4_tmp_1;
        N_c[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - Je_tmp_1 * rtb_Sum4_tmp_1;
      }
    } else {
      rtb_Sum4_tmp_tmp = 0.0;
      rtb_Sum4_tmp_0 = 0.0;
      J_c[0] = 0.0;
      J_c[1] = 0.0;
      J_c[2] = 1.0;
      J_c[3] = 0.0;
      std::memcpy(&N_c[0], &N_e_tmp[0], sizeof(real_T) << 4U);
    }

    if (temp_act[static_cast<int32_T>(i) + 95] == 1) {
      rtb_Sum4_tmp_1 = rtU.K_l4M;
      Je_tmp_1 = -rtb_Add_idx_2;
      J_d[0] = 0.0;
      J_d[1] = 0.0;
      J_d[2] = 0.0;
      J_d[3] = J_l4M[3];
      Je_tmp = J_l4M[3] * J_l4M[3];
      Je_tmp_2 = 0.0 / Je_tmp;
      q_l_dot_opt_tmp_4 = J_l4M[3] / Je_tmp;
      for (i_0 = 0; i_0 < 4; i_0++) {
        Je_tmp = J_l4M[i_0];
        N_b_tmp = i_0 << 2;
        q_l_dot_opt_tmp_5 = Je_tmp_2 * Je_tmp;
        N_d[N_b_tmp] = N_e_tmp[N_b_tmp] - q_l_dot_opt_tmp_5;
        N_d[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - q_l_dot_opt_tmp_5;
        N_d[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - q_l_dot_opt_tmp_5;
        N_d[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - q_l_dot_opt_tmp_4 * Je_tmp;
      }
    } else if (temp_act[static_cast<int32_T>(i) + 111] == 1) {
      rtb_Sum4_tmp_1 = rtU.K_l4m;
      Je_tmp_1 = -w_l4m;
      J_d[0] = 0.0;
      J_d[1] = 0.0;
      J_d[2] = 0.0;
      J_d[3] = J_l4m[3];
      Je_tmp = J_l4m[3] * J_l4m[3];
      Je_tmp_2 = 0.0 / Je_tmp;
      q_l_dot_opt_tmp_4 = J_l4m[3] / Je_tmp;
      for (i_0 = 0; i_0 < 4; i_0++) {
        Je_tmp = J_l4m[i_0];
        N_b_tmp = i_0 << 2;
        N_d[N_b_tmp] = N_e_tmp[N_b_tmp] - Je_tmp_2 * Je_tmp;
        N_d[N_b_tmp + 1] = N_e_tmp[N_b_tmp + 1] - Je_tmp_2 * Je_tmp;
        N_d[N_b_tmp + 2] = N_e_tmp[N_b_tmp + 2] - Je_tmp_2 * Je_tmp;
        N_d[N_b_tmp + 3] = N_e_tmp[N_b_tmp + 3] - q_l_dot_opt_tmp_4 * Je_tmp;
      }
    } else {
      rtb_Sum4_tmp_1 = 0.0;
      Je_tmp_1 = 0.0;
      J_d[0] = 0.0;
      J_d[1] = 0.0;
      J_d[2] = 0.0;
      J_d[3] = 1.0;
      std::memcpy(&N_d[0], &N_e_tmp[0], sizeof(real_T) << 4U);
    }

    Je_tmp = J_a[0] * J_a[0];
    q_l_dot_opt_tmp_4 = J_b[1] * J_b[1];
    q_l_dot_opt_tmp_5 = J_c[2] * J_c[2];
    q_l_dot_opt_tmp_6 = J_d[3] * J_d[3];
    if (std::abs(rtU.q4r) < 0.1) {
      for (i_0 = 0; i_0 < 4; i_0++) {
        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_0[N_a_tmp] = 0.0;
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp] * N_a[i_0];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 1] * N_a[i_0 + 4];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 2] * N_a[i_0 + 8];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 3] * N_a[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_1[N_a_tmp] = 0.0;
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp] * N_a_0[i_0];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 1] * N_a_0[i_0 + 4];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 2] * N_a_0[i_0 + 8];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 3] * N_a_0[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_2[N_a_tmp] = 0.0;
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp] * N_a_1[i_0];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 1] * N_a_1[i_0 + 4];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 2] * N_a_1[i_0 + 8];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 3] * N_a_1[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 3; N_b_tmp++) {
          N_a_tmp = (N_b_tmp << 2) + i_0;
          N_a_3[N_a_tmp] = 0.0;
          N_a_3[N_a_tmp] += N_a_2[i_0] * Je[N_b_tmp];
          N_a_3[N_a_tmp] += N_a_2[i_0 + 4] * Je[N_b_tmp + 3];
          N_a_3[N_a_tmp] += N_a_2[i_0 + 8] * Je[N_b_tmp + 6];
          N_a_3[N_a_tmp] += N_a_2[i_0 + 12] * Je[N_b_tmp + 9];
        }
      }

      for (i_0 = 0; i_0 < 9; i_0++) {
        N_e_tmp_2[i_0] = N_e_tmp_1[i_0] + static_cast<real_T>(g[i_0]);
      }

      mrdiv(N_a_3, N_e_tmp_2, tmp);
      for (i_0 = 0; i_0 <= 2; i_0 += 2) {
        tmp_2 = _mm_loadu_pd(&tmp[i_0]);
        tmp_1 = _mm_loadu_pd(&tmp[i_0 + 4]);
        tmp_0 = _mm_loadu_pd(&tmp[i_0 + 8]);
        _mm_storeu_pd(&rtb_step5[i_0], _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd
          (rtb_Sum2[2])), _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(rtb_Sum2[1])),
          _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(rtb_Sum2[0])), _mm_set1_pd
                     (0.0)))));
      }
    } else {
      for (i_0 = 0; i_0 < 4; i_0++) {
        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_0[N_a_tmp] = 0.0;
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp] * N_a[i_0];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 1] * N_a[i_0 + 4];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 2] * N_a[i_0 + 8];
          N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 3] * N_a[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_1[N_a_tmp] = 0.0;
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp] * N_a_0[i_0];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 1] * N_a_0[i_0 + 4];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 2] * N_a_0[i_0 + 8];
          N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 3] * N_a_0[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_2[N_a_tmp] = 0.0;
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp] * N_a_1[i_0];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 1] * N_a_1[i_0 + 4];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 2] * N_a_1[i_0 + 8];
          N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 3] * N_a_1[i_0 + 12];
        }

        for (N_b_tmp = 0; N_b_tmp < 3; N_b_tmp++) {
          N_a_tmp_tmp = N_b_tmp << 2;
          N_a_tmp = i_0 + N_a_tmp_tmp;
          N_a_3[N_a_tmp] = 0.0;
          N_a_3[N_a_tmp] += N_e_tmp_0[N_a_tmp_tmp] * N_a_2[i_0];
          N_a_3[N_a_tmp] += N_e_tmp_0[N_a_tmp_tmp + 1] * N_a_2[i_0 + 4];
          N_a_3[N_a_tmp] += N_e_tmp_0[N_a_tmp_tmp + 2] * N_a_2[i_0 + 8];
          N_a_3[N_a_tmp] += N_e_tmp_0[N_a_tmp_tmp + 3] * N_a_2[i_0 + 12];
        }
      }

      mrdiv(N_a_3, N_e_tmp_1, tmp);
      for (i_0 = 0; i_0 <= 2; i_0 += 2) {
        tmp_2 = _mm_loadu_pd(&tmp[i_0]);
        tmp_1 = _mm_loadu_pd(&tmp[i_0 + 4]);
        tmp_0 = _mm_loadu_pd(&tmp[i_0 + 8]);
        _mm_storeu_pd(&rtb_step5[i_0], _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd
          (rtb_Sum2[2])), _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(rtb_Sum2[1])),
          _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(rtb_Sum2[0])), _mm_set1_pd
                     (0.0)))));
      }
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      Je_tmp_2 = 0.0;
      q_l_dot_opt_tmp_7 = 0.0;
      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        converted = N_b_tmp << 2;
        N_a_tmp = converted + i_0;
        Je_tmp_2 += N_a[N_a_tmp] * J_b[N_b_tmp];
        N_a_0[N_a_tmp] = 0.0;
        q_l_dot_opt_tmp_tmp_6 = N_b[converted] * N_a[i_0];
        N_a_0[N_a_tmp] += q_l_dot_opt_tmp_tmp_6;
        N_a_tmp_0 = N_b[converted + 1] * N_a[i_0 + 4];
        N_a_0[N_a_tmp] += N_a_tmp_0;
        N_a_tmp_1 = N_b[converted + 2] * N_a[i_0 + 8];
        N_a_0[N_a_tmp] += N_a_tmp_1;
        N_a_tmp_2 = N_b[converted + 3] * N_a[i_0 + 12];
        N_a_0[N_a_tmp] += N_a_tmp_2;
        q_l_dot_opt_tmp_7 += N_a_0[N_a_tmp] * J_c[N_b_tmp];
        N_a_1[N_a_tmp] = 0.0;
        N_a_1[N_a_tmp] += q_l_dot_opt_tmp_tmp_6;
        N_a_1[N_a_tmp] += N_a_tmp_0;
        N_a_1[N_a_tmp] += N_a_tmp_1;
        N_a_1[N_a_tmp] += N_a_tmp_2;
      }

      N_a_4[i_0] = Je_tmp_2 / q_l_dot_opt_tmp_4;
      N_a_5[i_0] = q_l_dot_opt_tmp_7 / q_l_dot_opt_tmp_5;
      Je_tmp_2 = 0.0;
      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_2[N_a_tmp] = 0.0;
        N_a_2[N_a_tmp] += N_c[N_a_tmp_tmp] * N_a_1[i_0];
        N_a_2[N_a_tmp] += N_c[N_a_tmp_tmp + 1] * N_a_1[i_0 + 4];
        N_a_2[N_a_tmp] += N_c[N_a_tmp_tmp + 2] * N_a_1[i_0 + 8];
        N_a_2[N_a_tmp] += N_c[N_a_tmp_tmp + 3] * N_a_1[i_0 + 12];
        Je_tmp_2 += N_a_2[N_a_tmp] * J_d[N_b_tmp];
      }

      N_a_6[i_0] = Je_tmp_2 / q_l_dot_opt_tmp_6;
    }

    converted = 1;
    q_l_dot_opt_tmp_4 = 0.0;
    for (i_0 = 0; i_0 < 4; i_0++) {
      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_0[N_a_tmp] = 0.0;
        N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp] * N_a[i_0];
        N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 1] * N_a[i_0 + 4];
        N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 2] * N_a[i_0 + 8];
        N_a_0[N_a_tmp] += N_b[N_a_tmp_tmp + 3] * N_a[i_0 + 12];
      }

      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_1[N_a_tmp] = 0.0;
        N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp] * N_a_0[i_0];
        N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 1] * N_a_0[i_0 + 4];
        N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 2] * N_a_0[i_0 + 8];
        N_a_1[N_a_tmp] += N_c[N_a_tmp_tmp + 3] * N_a_0[i_0 + 12];
      }

      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_2[N_a_tmp] = 0.0;
        N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp] * N_a_1[i_0];
        N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 1] * N_a_1[i_0 + 4];
        N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 2] * N_a_1[i_0 + 8];
        N_a_2[N_a_tmp] += N_d[N_a_tmp_tmp + 3] * N_a_1[i_0 + 12];
      }

      Je_tmp_2 = 0.0;
      for (N_b_tmp = 0; N_b_tmp < 4; N_b_tmp++) {
        N_a_tmp_tmp = N_b_tmp << 2;
        N_a_tmp = i_0 + N_a_tmp_tmp;
        N_a_7[N_a_tmp] = 0.0;
        N_a_7[N_a_tmp] += N_e[N_a_tmp_tmp] * N_a_2[i_0];
        N_a_7[N_a_tmp] += N_e[N_a_tmp_tmp + 1] * N_a_2[i_0 + 4];
        N_a_7[N_a_tmp] += N_e[N_a_tmp_tmp + 2] * N_a_2[i_0 + 8];
        N_a_7[N_a_tmp] += N_e[N_a_tmp_tmp + 3] * N_a_2[i_0 + 12];
        Je_tmp_2 += N_a_7[N_a_tmp] * q_l_dot_opt[N_b_tmp];
      }

      rtb_q_r_dot[i_0] = ((((J_a[i_0] / Je_tmp * j * k + N_a_4[i_0] * l *
        rtb_Sum4_tmp) + N_a_5[i_0] * rtb_Sum4_tmp_tmp * rtb_Sum4_tmp_0) +
                           N_a_6[i_0] * rtb_Sum4_tmp_1 * Je_tmp_1) +
                          rtb_step5[i_0]) + Je_tmp_2;
      q_l_dot_opt_tmp_4 += J_l1M[i_0] * rtb_q_r_dot[i_0];
    }

    if ((act_l1M == 1) && (q_l_dot_opt_tmp_4 >= 0.0)) {
      converted = 0;
    }

    if ((act_l2M == 1) && (((0.0 * rtb_q_r_dot[0] + J_l2M[1] * rtb_q_r_dot[1]) +
          0.0 * rtb_q_r_dot[2]) + 0.0 * rtb_q_r_dot[3] >= 0.0)) {
      converted = 0;
    }

    Je_tmp_2 = 0.0 * rtb_q_r_dot[0] + 0.0 * rtb_q_r_dot[1];
    if ((act_l3M == 1) && ((J_l3M[2] * rtb_q_r_dot[2] + Je_tmp_2) + 0.0 *
                           rtb_q_r_dot[3] >= 0.0)) {
      converted = 0;
    }

    q_l_dot_opt_tmp_7 = 0.0 * rtb_q_r_dot[2] + Je_tmp_2;
    if ((act_l4M == 1) && (J_l4M[3] * rtb_q_r_dot[3] + q_l_dot_opt_tmp_7 >= 0.0))
    {
      converted = 0;
    }

    if ((act_l1m == 1) && (((J_l1m[0] * rtb_q_r_dot[0] + 0.0 * rtb_q_r_dot[1]) +
          0.0 * rtb_q_r_dot[2]) + 0.0 * rtb_q_r_dot[3] <= 0.0)) {
      converted = 0;
    }

    if ((act_l2m == 1) && (((0.0 * rtb_q_r_dot[0] + J_l2m[1] * rtb_q_r_dot[1]) +
          0.0 * rtb_q_r_dot[2]) + 0.0 * rtb_q_r_dot[3] <= 0.0)) {
      converted = 0;
    }

    if ((act_l3m == 1) && ((J_l3m[2] * rtb_q_r_dot[2] + Je_tmp_2) + 0.0 *
                           rtb_q_r_dot[3] <= 0.0)) {
      converted = 0;
    }

    if ((act_l4m == 1) && (J_l4m[3] * rtb_q_r_dot[3] + q_l_dot_opt_tmp_7 <= 0.0))
    {
      converted = 0;
    }

    if (i == rt_powd_snf(2.0, static_cast<real_T>(tot_task))) {
      converted = 1;
    }

    i++;
  }

  // Outport: '<Root>/q1r_d' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'

  rtY.q1r_d = rtDW.DiscreteTimeIntegrator1_DSTATE;

  // Outport: '<Root>/q3r_d' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

  rtY.q3r_d = rtDW.DiscreteTimeIntegrator2_DSTATE;

  // Outport: '<Root>/q2r_d' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator3'

  rtY.q2r_d = rtDW.DiscreteTimeIntegrator3_DSTATE;

  // Outport: '<Root>/q1l_d' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator4'

  rtY.q1l_d = rtDW.DiscreteTimeIntegrator4_DSTATE;

  // Outport: '<Root>/q3l_d' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator5'

  rtY.q3l_d = rtDW.DiscreteTimeIntegrator5_DSTATE;

  // Outport: '<Root>/q2l_d' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator6'

  rtY.q2l_d = rtDW.DiscreteTimeIntegrator6_DSTATE;

  // Outport: '<Root>/q4l_d' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator7'

  rtY.q4l_d = rtDW.DiscreteTimeIntegrator7_DSTATE;

  // Outport: '<Root>/q4r_d' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator8'

  rtY.q4r_d = rtDW.DiscreteTimeIntegrator8_DSTATE;

  // Update for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_IC_LOADING = 0U;
  rtDW.Integrator_PrevResetState = 0;

  // Update for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_IC_LOADING_d = 0U;

  // Update for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_DSTATE[0] += 0.01 * rtb_uT[0];

  // Update for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_DSTATE_a[0] += 0.01 * rtb_Sum4[0];

  // Update for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_DSTATE[1] += 0.01 * rtb_uT[1];

  // Update for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_DSTATE_a[1] += 0.01 * rtb_Sum4[1];

  // Update for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_DSTATE[2] += 0.01 * rtb_uT[2];

  // Update for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_DSTATE_a[2] += 0.01 * rtb_Sum4[2];
  rtDW.Integrator_PrevResetState_f = 0;

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  rtDW.DiscreteTimeIntegrator1_DSTATE += 0.01 * rtb_q_r_dot[0];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.01 * rtb_q_r_dot[2];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator3'
  rtDW.DiscreteTimeIntegrator3_DSTATE += 0.01 * rtb_q_r_dot[1];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator4'
  rtDW.DiscreteTimeIntegrator4_DSTATE += 0.01 * rtb_q_l_dot[0];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator5'
  rtDW.DiscreteTimeIntegrator5_DSTATE += 0.01 * rtb_q_l_dot[2];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator6'
  rtDW.DiscreteTimeIntegrator6_DSTATE += 0.01 * rtb_q_l_dot[1];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator7'
  rtDW.DiscreteTimeIntegrator7_DSTATE += 0.01 * rtb_q_l_dot[3];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator8'
  rtDW.DiscreteTimeIntegrator8_DSTATE += 0.01 * rtb_q_r_dot[3];

  // MATLAB Function: '<S1>/q_l_dot' incorporates:
  //   Inport: '<Root>/q1l_n'
  //   Inport: '<Root>/q1r_n'
  //   Inport: '<Root>/q2l_n'
  //   Inport: '<Root>/q2r_n'
  //   Inport: '<Root>/q3l_n'
  //   Inport: '<Root>/q3r_n'
  //   Inport: '<Root>/q4l_n'
  //   Inport: '<Root>/q4r_n'

  rtb_AvoidDividebyZero = std::sin(rtU.q1l_n);
  q_l_dot_opt_tmp = std::sin(rtU.q1r_n);
  q_l_dot_opt_tmp_5 = std::cos(rtU.q2l_n);
  q_l_dot_opt_tmp_1 = std::cos(rtU.q2r_n);
  q_l_dot_opt_tmp_0 = std::cos(rtU.q1l_n);
  q_l_dot_opt_tmp_2 = std::cos(rtU.q1r_n);
  q_l_dot_opt_tmp_6 = std::sin(rtU.q4l_n);
  q_l_dot_opt_tmp_3 = std::sin(rtU.q4r_n);
  rtb_MinMax = std::cos(rtU.q4l_n);
  rtb_Add_idx_0 = std::cos(rtU.q4r_n);
  rtb_Add_idx_1 = std::sin(rtU.q2l_n);
  rtb_Add_idx_2 = std::sin(rtU.q2r_n);
  w_l1m = std::sin(rtU.q3l_n);
  w_l2m = std::sin(rtU.q3r_n);
  w_l3m = std::cos(rtU.q3l_n);
  w_l4m = std::cos(rtU.q3r_n);

  // Outport: '<Root>/CoM_bar' incorporates:
  //   Inport: '<Root>/load'
  //   Inport: '<Root>/q1l_n'
  //   Inport: '<Root>/q1r_n'
  //   Inport: '<Root>/q2l_n'
  //   Inport: '<Root>/q2r_n'
  //   Inport: '<Root>/q3l_n'
  //   Inport: '<Root>/q3r_n'
  //   Inport: '<Root>/q4l_n'
  //   Inport: '<Root>/q4r_n'
  //   MATLAB Function: '<S1>/q_l_dot'

  rtY.CoM_bar[0] = -((((((((((rtb_AvoidDividebyZero * rtb_Add_idx_1 * w_l1m +
    q_l_dot_opt_tmp_0 * w_l3m) * q_l_dot_opt_tmp_6 / 8.0 + (11.0 *
    q_l_dot_opt_tmp_5 * rtb_AvoidDividebyZero / 80.0 + 11.0 * q_l_dot_opt_tmp_1 *
    q_l_dot_opt_tmp / 80.0)) + (q_l_dot_opt_tmp * rtb_Add_idx_2 * w_l2m +
    q_l_dot_opt_tmp_2 * w_l4m) * q_l_dot_opt_tmp_3 / 8.0) + q_l_dot_opt_tmp_5 *
    rtb_MinMax * rtb_AvoidDividebyZero / 8.0) + q_l_dot_opt_tmp_1 *
    rtb_Add_idx_0 * q_l_dot_opt_tmp / 8.0) * rtU.load + (((((3.0 *
    q_l_dot_opt_tmp_0 / 800.0 + 3.0 * q_l_dot_opt_tmp_2 / 800.0) + 10879.0 *
    rtb_AvoidDividebyZero / 5.0E+6) + 10879.0 * q_l_dot_opt_tmp / 5.0E+6) +
    10481.0 * q_l_dot_opt_tmp_5 * rtb_AvoidDividebyZero / 100000.0) + 10481.0 *
    q_l_dot_opt_tmp_1 * q_l_dot_opt_tmp / 100000.0)) + (std::sin(rtU.q1l_n) *
    std::sin(rtU.q2l_n) * std::sin(rtU.q3l_n) + std::cos(rtU.q1l_n) * std::cos
    (rtU.q3l_n)) * (7.409300489671729E+15 * q_l_dot_opt_tmp_6) /
                        7.2057594037927936E+17) + (std::sin(rtU.q1r_n) * std::
    sin(rtU.q2r_n) * std::sin(rtU.q3r_n) + std::cos(rtU.q1r_n) * std::cos
    (rtU.q3r_n)) * (7.409300489671729E+15 * q_l_dot_opt_tmp_3) /
                       7.2057594037927936E+17) + 7.409300489671729E+15 *
                      q_l_dot_opt_tmp_5 * rtb_MinMax * rtb_AvoidDividebyZero /
                      7.2057594037927936E+17) + 7.409300489671729E+15 *
                     q_l_dot_opt_tmp_1 * rtb_Add_idx_0 * q_l_dot_opt_tmp /
                     7.2057594037927936E+17) / (rtU.load + 1.6);
  rtY.CoM_bar[1] = ((((((((11.0 * rtb_Add_idx_1 / 80.0 + 11.0 * rtb_Add_idx_2 /
    80.0) + rtb_MinMax * rtb_Add_idx_1 / 8.0) + rtb_Add_idx_0 * rtb_Add_idx_2 /
    8.0) - q_l_dot_opt_tmp_5 * w_l1m * q_l_dot_opt_tmp_6 / 8.0) -
                       q_l_dot_opt_tmp_1 * w_l2m * q_l_dot_opt_tmp_3 / 8.0) *
                      rtU.load + (((10481.0 * rtb_Add_idx_1 / 100000.0 + 10481.0
    * rtb_Add_idx_2 / 100000.0) + 7.409300489671729E+15 * rtb_MinMax *
    rtb_Add_idx_1 / 7.2057594037927936E+17) + 7.409300489671729E+15 *
    rtb_Add_idx_0 * rtb_Add_idx_2 / 7.2057594037927936E+17)) -
                     7.409300489671729E+15 * std::cos(rtU.q2l_n) * w_l1m *
                     q_l_dot_opt_tmp_6 / 7.2057594037927936E+17) -
                    7.409300489671729E+15 * std::cos(rtU.q2r_n) * w_l2m *
                    q_l_dot_opt_tmp_3 / 7.2057594037927936E+17) / (rtU.load +
    1.6);
  rtY.CoM_bar[2] = -((((((((((11.0 * q_l_dot_opt_tmp_0 * q_l_dot_opt_tmp_5 /
    80.0 + 11.0 * q_l_dot_opt_tmp_2 * q_l_dot_opt_tmp_1 / 80.0) - (w_l3m *
    rtb_AvoidDividebyZero - q_l_dot_opt_tmp_0 * rtb_Add_idx_1 * w_l1m) *
    q_l_dot_opt_tmp_6 / 8.0) - (w_l4m * q_l_dot_opt_tmp - q_l_dot_opt_tmp_2 *
    rtb_Add_idx_2 * w_l2m) * q_l_dot_opt_tmp_3 / 8.0) + q_l_dot_opt_tmp_0 *
    q_l_dot_opt_tmp_5 * rtb_MinMax / 8.0) + q_l_dot_opt_tmp_2 *
    q_l_dot_opt_tmp_1 * rtb_Add_idx_0 / 8.0) * rtU.load + (((((10879.0 *
    q_l_dot_opt_tmp_0 / 5.0E+6 + 10879.0 * q_l_dot_opt_tmp_2 / 5.0E+6) - 3.0 *
    rtb_AvoidDividebyZero / 800.0) - 3.0 * q_l_dot_opt_tmp / 800.0) + 10481.0 *
    q_l_dot_opt_tmp_0 * q_l_dot_opt_tmp_5 / 100000.0) + 10481.0 *
    q_l_dot_opt_tmp_2 * q_l_dot_opt_tmp_1 / 100000.0)) - (std::cos(rtU.q3l_n) *
    std::sin(rtU.q1l_n) - std::cos(rtU.q1l_n) * std::sin(rtU.q2l_n) * std::sin
    (rtU.q3l_n)) * (7.409300489671729E+15 * std::sin(rtU.q4l_n)) /
                        7.2057594037927936E+17) - (std::cos(rtU.q3r_n) * std::
    sin(rtU.q1r_n) - std::cos(rtU.q1r_n) * std::sin(rtU.q2r_n) * std::sin
    (rtU.q3r_n)) * (7.409300489671729E+15 * std::sin(rtU.q4r_n)) /
                       7.2057594037927936E+17) + 7.409300489671729E+15 *
                      q_l_dot_opt_tmp_0 * q_l_dot_opt_tmp_5 * rtb_MinMax /
                      7.2057594037927936E+17) + 7.409300489671729E+15 *
                     q_l_dot_opt_tmp_2 * q_l_dot_opt_tmp_1 * rtb_Add_idx_0 /
                     7.2057594037927936E+17) / (rtU.load + 1.6);

  // MATLAB Function: '<S1>/q_l_dot' incorporates:
  //   Inport: '<Root>/load'
  //   Inport: '<Root>/q1l'
  //   Inport: '<Root>/q1r'
  //   Inport: '<Root>/q2l'
  //   Inport: '<Root>/q2r'
  //   Inport: '<Root>/q3l'
  //   Inport: '<Root>/q3r'
  //   Inport: '<Root>/q4l'
  //   Inport: '<Root>/q4r'
  //   MATLAB Function: '<S1>/q_r_dot'

  rtb_MinMax = -((((((((((std::sin(rtU.q1l) * std::sin(rtU.q2l) * std::sin
    (rtU.q3l) + std::cos(rtU.q1l) * std::cos(rtU.q3l)) * std::sin(rtU.q4l) / 8.0
    + (11.0 * std::cos(rtU.q2l) * std::sin(rtU.q1l) / 80.0 + 11.0 * std::cos
       (rtU.q2r) * std::sin(rtU.q1r) / 80.0)) + (std::sin(rtU.q1r) * std::sin
    (rtU.q2r) * std::sin(rtU.q3r) + std::cos(rtU.q1r) * std::cos(rtU.q3r)) * std::
                        sin(rtU.q4r) / 8.0) + std::cos(rtU.q2l) * std::cos
                       (rtU.q4l) * std::sin(rtU.q1l) / 8.0) + std::cos(rtU.q2r) *
                      std::cos(rtU.q4r) * std::sin(rtU.q1r) / 8.0) * rtU.load +
                     (((((3.0 * std::cos(rtU.q1l) / 800.0 + 3.0 * std::cos
    (rtU.q1r) / 800.0) + 10879.0 * std::sin(rtU.q1l) / 5.0E+6) + 10879.0 * std::
                        sin(rtU.q1r) / 5.0E+6) + 10481.0 * std::cos(rtU.q2l) *
                       std::sin(rtU.q1l) / 100000.0) + 10481.0 * std::cos
                      (rtU.q2r) * std::sin(rtU.q1r) / 100000.0)) + (std::sin
    (rtU.q1l) * std::sin(rtU.q2l) * std::sin(rtU.q3l) + std::cos(rtU.q1l) * std::
    cos(rtU.q3l)) * (7.409300489671729E+15 * std::sin(rtU.q4l)) /
                    7.2057594037927936E+17) + (std::sin(rtU.q1r) * std::sin
    (rtU.q2r) * std::sin(rtU.q3r) + std::cos(rtU.q1r) * std::cos(rtU.q3r)) *
                   (7.409300489671729E+15 * std::sin(rtU.q4r)) /
                   7.2057594037927936E+17) + 7.409300489671729E+15 * std::cos
                  (rtU.q2l) * std::cos(rtU.q4l) * std::sin(rtU.q1l) /
                  7.2057594037927936E+17) + 7.409300489671729E+15 * std::cos
                 (rtU.q2r) * std::cos(rtU.q4r) * std::sin(rtU.q1r) /
                 7.2057594037927936E+17) / (rtU.load + 1.6);

  // Outport: '<Root>/CoM' incorporates:
  //   MATLAB Function: '<S1>/q_l_dot'

  rtY.CoM[0] = rtb_MinMax;

  // MATLAB Function: '<S1>/q_l_dot' incorporates:
  //   Inport: '<Root>/load'
  //   MATLAB Function: '<S1>/q_r_dot'

  rtb_AvoidDividebyZero = Je_tmp_0 / (rtU.load + 1.6);

  // Outport: '<Root>/CoM' incorporates:
  //   MATLAB Function: '<S1>/q_l_dot'

  rtY.CoM[1] = rtb_AvoidDividebyZero;

  // MATLAB Function: '<S1>/q_l_dot' incorporates:
  //   Inport: '<Root>/load'
  //   Inport: '<Root>/q1l'
  //   Inport: '<Root>/q1r'
  //   Inport: '<Root>/q2l'
  //   Inport: '<Root>/q2r'
  //   Inport: '<Root>/q3l'
  //   Inport: '<Root>/q3r'
  //   Inport: '<Root>/q4l'
  //   Inport: '<Root>/q4r'
  //   MATLAB Function: '<S1>/q_r_dot'

  q_l_dot_opt_tmp = -((((((((((11.0 * std::cos(rtU.q1l) * std::cos(rtU.q2l) /
    80.0 + 11.0 * std::cos(rtU.q1r) * std::cos(rtU.q2r) / 80.0) - (std::cos
    (rtU.q3l) * std::sin(rtU.q1l) - std::cos(rtU.q1l) * std::sin(rtU.q2l) * std::
    sin(rtU.q3l)) * std::sin(rtU.q4l) / 8.0) - (std::cos(rtU.q3r) * std::sin
    (rtU.q1r) - std::cos(rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r)) * std::
    sin(rtU.q4r) / 8.0) + std::cos(rtU.q1l) * std::cos(rtU.q2l) * std::cos
    (rtU.q4l) / 8.0) + std::cos(rtU.q1r) * std::cos(rtU.q2r) * std::cos(rtU.q4r)
    / 8.0) * rtU.load + (((((10879.0 * std::cos(rtU.q1l) / 5.0E+6 + 10879.0 *
    std::cos(rtU.q1r) / 5.0E+6) - 3.0 * std::sin(rtU.q1l) / 800.0) - 3.0 * std::
    sin(rtU.q1r) / 800.0) + 10481.0 * std::cos(rtU.q1l) * std::cos(rtU.q2l) /
    100000.0) + 10481.0 * std::cos(rtU.q1r) * std::cos(rtU.q2r) / 100000.0)) -
    (std::cos(rtU.q3l) * std::sin(rtU.q1l) - std::cos(rtU.q1l) * std::sin
     (rtU.q2l) * std::sin(rtU.q3l)) * (7.409300489671729E+15 * std::sin(rtU.q4l))
    / 7.2057594037927936E+17) - (std::cos(rtU.q3r) * std::sin(rtU.q1r) - std::
    cos(rtU.q1r) * std::sin(rtU.q2r) * std::sin(rtU.q3r)) *
                        (7.409300489671729E+15 * std::sin(rtU.q4r)) /
                        7.2057594037927936E+17) + 7.409300489671729E+15 * std::
                       cos(rtU.q1l) * std::cos(rtU.q2l) * std::cos(rtU.q4l) /
                       7.2057594037927936E+17) + 7.409300489671729E+15 * std::
                      cos(rtU.q1r) * std::cos(rtU.q2r) * std::cos(rtU.q4r) /
                      7.2057594037927936E+17) / (rtU.load + 1.6);

  // Outport: '<Root>/CoM' incorporates:
  //   MATLAB Function: '<S1>/q_l_dot'

  rtY.CoM[2] = q_l_dot_opt_tmp;

  // Outport: '<Root>/CoM_bar1' incorporates:
  //   Inport: '<Root>/load'
  //   Inport: '<Root>/q1l_n'
  //   Inport: '<Root>/q1r_n'
  //   Inport: '<Root>/q2l_n'
  //   Inport: '<Root>/q2r_n'
  //   Inport: '<Root>/q3l_n'
  //   Inport: '<Root>/q3r_n'
  //   Inport: '<Root>/q4l_n'
  //   Inport: '<Root>/q4r_n'
  //   MATLAB Function: '<S1>/q_r_dot'

  rtY.CoM_bar1[0] = -((((((((((std::sin(rtU.q1l_n) * std::sin(rtU.q2l_n) * std::
    sin(rtU.q3l_n) + std::cos(rtU.q1l_n) * std::cos(rtU.q3l_n)) * std::sin
    (rtU.q4l_n) / 8.0 + (11.0 * std::cos(rtU.q2l_n) * std::sin(rtU.q1l_n) / 80.0
    + 11.0 * std::cos(rtU.q2r_n) * std::sin(rtU.q1r_n) / 80.0)) + (std::sin
    (rtU.q1r_n) * std::sin(rtU.q2r_n) * std::sin(rtU.q3r_n) + std::cos(rtU.q1r_n)
    * std::cos(rtU.q3r_n)) * std::sin(rtU.q4r_n) / 8.0) + std::cos(rtU.q2l_n) *
    std::cos(rtU.q4l_n) * std::sin(rtU.q1l_n) / 8.0) + std::cos(rtU.q2r_n) * std::
    cos(rtU.q4r_n) * std::sin(rtU.q1r_n) / 8.0) * rtU.load + (((((3.0 * std::cos
    (rtU.q1l_n) / 800.0 + 3.0 * std::cos(rtU.q1r_n) / 800.0) + 10879.0 * std::
    sin(rtU.q1l_n) / 5.0E+6) + 10879.0 * std::sin(rtU.q1r_n) / 5.0E+6) + 10481.0
    * std::cos(rtU.q2l_n) * std::sin(rtU.q1l_n) / 100000.0) + 10481.0 * std::cos
    (rtU.q2r_n) * std::sin(rtU.q1r_n) / 100000.0)) + (std::sin(rtU.q1l_n) * std::
    sin(rtU.q2l_n) * std::sin(rtU.q3l_n) + std::cos(rtU.q1l_n) * std::cos
    (rtU.q3l_n)) * (7.409300489671729E+15 * std::sin(rtU.q4l_n)) /
    7.2057594037927936E+17) + (std::sin(rtU.q1r_n) * std::sin(rtU.q2r_n) * std::
    sin(rtU.q3r_n) + std::cos(rtU.q1r_n) * std::cos(rtU.q3r_n)) *
                        (7.409300489671729E+15 * std::sin(rtU.q4r_n)) /
                        7.2057594037927936E+17) + 7.409300489671729E+15 * std::
                       cos(rtU.q2l_n) * std::cos(rtU.q4l_n) * std::sin(rtU.q1l_n)
                       / 7.2057594037927936E+17) + 7.409300489671729E+15 * std::
                      cos(rtU.q2r_n) * std::cos(rtU.q4r_n) * std::sin(rtU.q1r_n)
                      / 7.2057594037927936E+17) / (rtU.load + 1.6);
  rtY.CoM_bar1[1] = ((((((((11.0 * std::sin(rtU.q2l_n) / 80.0 + 11.0 * std::sin
    (rtU.q2r_n) / 80.0) + std::cos(rtU.q4l_n) * std::sin(rtU.q2l_n) / 8.0) + std::
    cos(rtU.q4r_n) * std::sin(rtU.q2r_n) / 8.0) - std::cos(rtU.q2l_n) * std::sin
    (rtU.q3l_n) * std::sin(rtU.q4l_n) / 8.0) - std::cos(rtU.q2r_n) * std::sin
                        (rtU.q3r_n) * std::sin(rtU.q4r_n) / 8.0) * rtU.load +
                       (((10481.0 * std::sin(rtU.q2l_n) / 100000.0 + 10481.0 *
    std::sin(rtU.q2r_n) / 100000.0) + 7.409300489671729E+15 * std::cos(rtU.q4l_n)
    * std::sin(rtU.q2l_n) / 7.2057594037927936E+17) + 7.409300489671729E+15 *
                        std::cos(rtU.q4r_n) * std::sin(rtU.q2r_n) /
                        7.2057594037927936E+17)) - 7.409300489671729E+15 * std::
                      cos(rtU.q2l_n) * std::sin(rtU.q3l_n) * std::sin(rtU.q4l_n)
                      / 7.2057594037927936E+17) - 7.409300489671729E+15 * std::
                     cos(rtU.q2r_n) * std::sin(rtU.q3r_n) * std::sin(rtU.q4r_n) /
                     7.2057594037927936E+17) / (rtU.load + 1.6);
  rtY.CoM_bar1[2] = -((((((((((11.0 * std::cos(rtU.q1l_n) * std::cos(rtU.q2l_n) /
    80.0 + 11.0 * std::cos(rtU.q1r_n) * std::cos(rtU.q2r_n) / 80.0) - (std::cos
    (rtU.q3l_n) * std::sin(rtU.q1l_n) - std::cos(rtU.q1l_n) * std::sin(rtU.q2l_n)
    * std::sin(rtU.q3l_n)) * std::sin(rtU.q4l_n) / 8.0) - (std::cos(rtU.q3r_n) *
    std::sin(rtU.q1r_n) - std::cos(rtU.q1r_n) * std::sin(rtU.q2r_n) * std::sin
    (rtU.q3r_n)) * std::sin(rtU.q4r_n) / 8.0) + std::cos(rtU.q1l_n) * std::cos
    (rtU.q2l_n) * std::cos(rtU.q4l_n) / 8.0) + std::cos(rtU.q1r_n) * std::cos
    (rtU.q2r_n) * std::cos(rtU.q4r_n) / 8.0) * rtU.load + (((((10879.0 * std::
    cos(rtU.q1l_n) / 5.0E+6 + 10879.0 * std::cos(rtU.q1r_n) / 5.0E+6) - 3.0 *
    std::sin(rtU.q1l_n) / 800.0) - 3.0 * std::sin(rtU.q1r_n) / 800.0) + 10481.0 *
    std::cos(rtU.q1l_n) * std::cos(rtU.q2l_n) / 100000.0) + 10481.0 * std::cos
    (rtU.q1r_n) * std::cos(rtU.q2r_n) / 100000.0)) - (std::cos(rtU.q3l_n) * std::
    sin(rtU.q1l_n) - std::cos(rtU.q1l_n) * std::sin(rtU.q2l_n) * std::sin
    (rtU.q3l_n)) * (7.409300489671729E+15 * std::sin(rtU.q4l_n)) /
    7.2057594037927936E+17) - (std::cos(rtU.q3r_n) * std::sin(rtU.q1r_n) - std::
    cos(rtU.q1r_n) * std::sin(rtU.q2r_n) * std::sin(rtU.q3r_n)) *
                        (7.409300489671729E+15 * std::sin(rtU.q4r_n)) /
                        7.2057594037927936E+17) + 7.409300489671729E+15 * std::
                       cos(rtU.q1l_n) * std::cos(rtU.q2l_n) * std::cos(rtU.q4l_n)
                       / 7.2057594037927936E+17) + 7.409300489671729E+15 * std::
                      cos(rtU.q1r_n) * std::cos(rtU.q2r_n) * std::cos(rtU.q4r_n)
                      / 7.2057594037927936E+17) / (rtU.load + 1.6);

  // Outport: '<Root>/CoM1'
  rtY.CoM1[0] = rtb_MinMax;
  rtY.CoM1[1] = rtb_AvoidDividebyZero;
  rtY.CoM1[2] = q_l_dot_opt_tmp;

  // End of Outputs for SubSystem: '<Root>/CLIK'
}

// Model initialize function
void CLIK::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // SystemInitialize for Atomic SubSystem: '<Root>/CLIK'
  // Start for Probe: '<S13>/Probe'
  rtDW.Probe[0] = 0.01;
  rtDW.Probe[1] = 0.0;

  // Start for Probe: '<S8>/Probe'
  rtDW.Probe_n[0] = 0.01;
  rtDW.Probe_n[1] = 0.0;

  // InitializeConditions for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_IC_LOADING = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_IC_LOADING_d = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator7' 
  rtDW.DiscreteTimeIntegrator7_DSTATE = -1.5;

  // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator8' 
  rtDW.DiscreteTimeIntegrator8_DSTATE = -1.5;

  // End of SystemInitialize for SubSystem: '<Root>/CLIK'
}

// Constructor
CLIK::CLIK() :
  rtU(),
  rtY(),
  rtDW(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
CLIK::~CLIK()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
CLIK::RT_MODEL * CLIK::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
