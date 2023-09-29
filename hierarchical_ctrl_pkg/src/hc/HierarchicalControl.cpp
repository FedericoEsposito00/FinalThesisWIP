//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: HierarchicalControl.cpp
//
// Code generated for Simulink model 'HierarchicalControl'.
//
// Model version                  : 4.35
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Tue Sep 26 09:08:47 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "HierarchicalControl.h"
#include <cmath>
#include <emmintrin.h>
#include <cstring>
#include "rtwtypes.h"
#include <stddef.h>
#define NumBitsPerChar                 8U

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

// Model step function
void HierarchicalControl::step()
{
  static const real_T e[16]{ 8.54858E-6, 0.0, 5.5138341E-6, 1.3677728E-7,
    8.54858E-6, -5.5138341E-6, 0.0, -1.3677728E-7, 8.54858E-6, 0.0,
    -5.5138341E-6, 1.3677728E-7, 8.54858E-6, 5.5138341E-6, 0.0, -1.3677728E-7 };

  __m128d tmp_7;
  __m128d tmp_8;
  real_T tmp[18];
  real_T A_0[16];
  real_T A[9];
  real_T A_1[9];
  real_T A_2[9];
  real_T A_3[9];
  real_T A_4[9];
  real_T B_0[9];
  real_T rtb_Q[9];
  real_T rtb_Subtract_0[6];
  real_T B_1[4];
  real_T rtb_mu_d[3];
  real_T rtb_tau[3];
  real_T rtb_uT_e_0[3];
  real_T tmp_0[3];
  real_T v[3];
  real_T rtb_AvoidDividebyZero;
  real_T rtb_AvoidDividebyZero_k;
  real_T rtb_MinMax;
  real_T rtb_Q_b;
  real_T rtb_Subtract_idx_0;
  real_T rtb_Subtract_idx_1;
  real_T rtb_Subtract_idx_2;
  real_T rtb_phi_d;
  real_T rtb_theta_d;
  real_T rtb_uT_e_idx_0;
  real_T rtb_uT_e_idx_1;
  real_T rtb_uT_e_idx_2;
  real_T rtb_uT_idx_0;
  real_T rtb_uT_idx_1;
  real_T rtb_uT_idx_2;
  real_T tmp_9;
  real_T tmp_a;
  real_T tmp_b;
  real_T v_0;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
  int8_T ipiv[4];

  // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
  // Fcn: '<S13>/Avoid Divide by Zero' incorporates:
  //   Constant: '<S13>/Time constant'
  //   Gain: '<S13>/Minimum sampling to time constant ratio'
  //   MinMax: '<S13>/MinMax'

  rtb_AvoidDividebyZero = std::fmax(10.0 * rtDW.Probe[0], 0.1);

  // Fcn: '<S8>/Avoid Divide by Zero' incorporates:
  //   Constant: '<S8>/Time constant'
  //   Gain: '<S8>/Minimum sampling to time constant ratio'
  //   MinMax: '<S8>/MinMax'

  rtb_AvoidDividebyZero_k = std::fmax(10.0 * rtDW.Probe_e[0], 0.1);

  // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  for (r1 = 0; r1 <= 16; r1 += 2) {
    // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
    // MATLAB Function: '<S1>/Outer-loop control'
    _mm_storeu_pd(&tmp[r1], _mm_mul_pd(_mm_loadu_pd
      (&rtConstP.Outerloopcontrol_Kp[r1]), _mm_set1_pd(-1.0)));

    // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  }

  // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
  // MATLAB Function: '<S1>/Outer-loop control' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator3'
  //   Inport: '<Root>/acc_linear_des'
  //   Inport: '<Root>/estimate'
  //   Inport: '<Root>/linear_vel'
  //   Inport: '<Root>/position'
  //   Inport: '<Root>/position_des'
  //   Inport: '<Root>/psi_d'
  //   Inport: '<Root>/vel_linear_des'
  //   Sum: '<S1>/Subtract3'

  tmp_9 = rtU.position[0] - rtU.position_des[0];
  rtb_Subtract_0[0] = tmp_9;
  rtb_Subtract_0[3] = rtU.linear_vel[0] - rtU.vel_linear_des[0];
  tmp_a = rtU.position[1] - rtU.position_des[1];
  rtb_Subtract_0[1] = tmp_a;
  rtb_Subtract_0[4] = rtU.linear_vel[1] - rtU.vel_linear_des[1];
  tmp_b = rtU.position[2] - rtU.position_des[2];
  rtb_Subtract_0[2] = tmp_b;
  rtb_Subtract_0[5] = rtU.linear_vel[2] - rtU.vel_linear_des[2];
  for (r1 = 0; r1 < 3; r1++) {
    rtb_tau[r1] = 0.0;
    for (rtemp = 0; rtemp < 6; rtemp++) {
      rtb_tau[r1] += tmp[3 * rtemp + r1] * rtb_Subtract_0[rtemp];
    }

    rtb_mu_d[r1] = ((rtb_tau[r1] - ((rtConstP.Outerloopcontrol_Ki[r1 + 3] *
      rtDW.DiscreteTimeIntegrator3_DSTATE[1] + rtConstP.Outerloopcontrol_Ki[r1] *
      rtDW.DiscreteTimeIntegrator3_DSTATE[0]) + rtConstP.Outerloopcontrol_Ki[r1
      + 6] * rtDW.DiscreteTimeIntegrator3_DSTATE[2])) + rtU.acc_linear_des[r1])
      - 0.049336425082638517 * rtU.estimate[r1];
  }

  if (rtb_mu_d[2] > 9.81) {
    rtb_mu_d[2] = 9.8;
  }

  rtb_MinMax = std::sqrt((rtb_mu_d[0] * rtb_mu_d[0] + rtb_mu_d[1] * rtb_mu_d[1])
    + (rtb_mu_d[2] - 9.81) * (rtb_mu_d[2] - 9.81)) * 20.269;
  rtb_theta_d = std::cos(rtU.psi_d);
  rtb_uT_idx_0 = std::sin(rtU.psi_d);
  rtb_phi_d = std::asin((rtb_mu_d[1] * rtb_theta_d - rtb_mu_d[0] * rtb_uT_idx_0)
                        * (20.269 / rtb_MinMax));
  rtb_theta_d = std::atan((rtb_mu_d[0] * rtb_theta_d + rtb_mu_d[1] *
    rtb_uT_idx_0) / (rtb_mu_d[2] - 9.81));

  // DiscreteIntegrator: '<S12>/Integrator' incorporates:
  //   Inport: '<Root>/psi_d'

  if (rtDW.Integrator_IC_LOADING != 0) {
    rtDW.Integrator_DSTATE[0] = rtb_phi_d;
    rtDW.Integrator_DSTATE[1] = rtb_theta_d;
    rtDW.Integrator_DSTATE[2] = rtU.psi_d;
  }

  if (rtDW.Integrator_PrevResetState != 0) {
    rtDW.Integrator_DSTATE[0] = rtb_phi_d;
    rtDW.Integrator_DSTATE[1] = rtb_theta_d;
    rtDW.Integrator_DSTATE[2] = rtU.psi_d;
  }

  // Product: '<S5>/1//T' incorporates:
  //   DiscreteIntegrator: '<S12>/Integrator'
  //   Inport: '<Root>/psi_d'
  //   Sum: '<S5>/Sum1'

  rtb_uT_idx_0 = 1.0 / rtb_AvoidDividebyZero_k * (rtb_phi_d -
    rtDW.Integrator_DSTATE[0]);
  rtb_uT_idx_1 = 1.0 / rtb_AvoidDividebyZero_k * (rtb_theta_d -
    rtDW.Integrator_DSTATE[1]);
  rtb_uT_idx_2 = 1.0 / rtb_AvoidDividebyZero_k * (rtU.psi_d -
    rtDW.Integrator_DSTATE[2]);

  // DiscreteIntegrator: '<S17>/Integrator' incorporates:
  //   Inport: '<Root>/dot_psi_d'

  if (rtDW.Integrator_IC_LOADING_e != 0) {
    rtDW.Integrator_DSTATE_k[0] = rtb_uT_idx_0;
    rtDW.Integrator_DSTATE_k[1] = rtb_uT_idx_1;
    rtDW.Integrator_DSTATE_k[2] = rtU.dot_psi_d;
  }

  if (rtDW.Integrator_PrevResetState_b != 0) {
    rtDW.Integrator_DSTATE_k[0] = rtb_uT_idx_0;
    rtDW.Integrator_DSTATE_k[1] = rtb_uT_idx_1;
    rtDW.Integrator_DSTATE_k[2] = rtU.dot_psi_d;
  }

  // Product: '<S6>/1//T' incorporates:
  //   DiscreteIntegrator: '<S17>/Integrator'
  //   Inport: '<Root>/dot_psi_d'
  //   Sum: '<S6>/Sum1'

  rtb_uT_e_idx_0 = 1.0 / rtb_AvoidDividebyZero * (rtb_uT_idx_0 -
    rtDW.Integrator_DSTATE_k[0]);
  rtb_uT_e_idx_1 = 1.0 / rtb_AvoidDividebyZero * (rtb_uT_idx_1 -
    rtDW.Integrator_DSTATE_k[1]);
  rtb_uT_e_idx_2 = 1.0 / rtb_AvoidDividebyZero * (rtU.dot_psi_d -
    rtDW.Integrator_DSTATE_k[2]);

  // Sum: '<S2>/Subtract' incorporates:
  //   Inport: '<Root>/eta'
  //   Inport: '<Root>/psi_d'

  rtb_Subtract_idx_0 = rtU.eta[0] - rtb_phi_d;
  rtb_Subtract_idx_1 = rtU.eta[1] - rtb_theta_d;
  rtb_Subtract_idx_2 = rtU.eta[2] - rtU.psi_d;

  // MATLAB Function: '<S2>/Inner-loop control' incorporates:
  //   Inport: '<Root>/eta'
  //   Inport: '<Root>/eta_dot'

  rtb_AvoidDividebyZero = std::sin(rtU.eta[0]);
  rtb_AvoidDividebyZero_k = std::cos(rtU.eta[1]);
  rtb_phi_d = std::cos(rtU.eta[0]);
  rtb_theta_d = std::sin(rtU.eta[1]);
  rtb_Q[0] = 1.0;
  rtb_Q[3] = 0.0;
  rtb_Q[6] = -rtb_theta_d;
  rtb_Q[1] = 0.0;
  rtb_Q[4] = rtb_phi_d;
  rtb_Q[7] = rtb_AvoidDividebyZero_k * rtb_AvoidDividebyZero;
  rtb_Q[2] = 0.0;
  rtb_Q[5] = -rtb_AvoidDividebyZero;
  rtb_Q[8] = rtb_AvoidDividebyZero_k * rtb_phi_d;
  for (r1 = 0; r1 < 3; r1++) {
    rtb_Q_b = rtb_Q[r1];
    A[3 * r1] = rtb_Q_b;
    v_0 = rtb_Q_b * rtU.eta_dot[0];
    rtb_Q_b = rtb_Q[r1 + 3];
    A[3 * r1 + 1] = rtb_Q_b;
    v_0 += rtb_Q_b * rtU.eta_dot[1];
    rtb_Q_b = rtb_Q[r1 + 6];
    A[3 * r1 + 2] = rtb_Q_b;
    v[r1] = rtb_Q_b * rtU.eta_dot[2] + v_0;
  }

  B_0[0] = 0.0;
  B_0[3] = -v[2];
  B_0[6] = v[1];
  B_0[1] = v[2];
  B_0[4] = 0.0;
  B_0[7] = -v[0];
  B_0[2] = -v[1];
  B_0[5] = v[0];
  B_0[8] = 0.0;
  for (r1 = 0; r1 < 3; r1++) {
    for (rtemp = 0; rtemp < 3; rtemp++) {
      r2 = 3 * rtemp + r1;
      A_1[r2] = 0.0;
      A_1[r2] += B_0[3 * rtemp] * A[r1];
      A_1[r2] += B_0[3 * rtemp + 1] * A[r1 + 3];
      A_1[r2] += B_0[3 * rtemp + 2] * A[r1 + 6];
    }

    for (rtemp = 0; rtemp < 3; rtemp++) {
      r2 = 3 * rtemp + r1;
      A_2[r2] = 0.0;
      A_2[r2] += rtConstP.Innerloopcontrol_Ib[3 * rtemp] * A_1[r1];
      A_2[r2] += rtConstP.Innerloopcontrol_Ib[3 * rtemp + 1] * A_1[r1 + 3];
      A_2[r2] += rtConstP.Innerloopcontrol_Ib[3 * rtemp + 2] * A_1[r1 + 6];
    }
  }

  B_0[0] = 0.0;
  B_0[3] = 0.0;
  B_0[6] = -rtU.eta_dot[1] * rtb_AvoidDividebyZero_k;
  B_0[1] = 0.0;
  B_0[4] = -rtU.eta_dot[0] * rtb_AvoidDividebyZero;
  B_0[7] = -rtU.eta_dot[1] * rtb_theta_d * rtb_AvoidDividebyZero + rtU.eta_dot[0]
    * rtb_AvoidDividebyZero_k * rtb_phi_d;
  B_0[2] = 0.0;
  B_0[5] = -rtU.eta_dot[0] * rtb_phi_d;
  B_0[8] = -rtU.eta_dot[1] * std::sin(rtU.eta[1]) * rtb_phi_d - rtU.eta_dot[0] *
    std::cos(rtU.eta[1]) * rtb_AvoidDividebyZero;
  for (r1 = 0; r1 < 3; r1++) {
    for (rtemp = 0; rtemp < 3; rtemp++) {
      r2 = 3 * rtemp + r1;
      A_1[r2] = 0.0;
      A_3[r2] = 0.0;
      A_1[r2] += rtConstP.Innerloopcontrol_Ib[3 * rtemp] * A[r1];
      A_3[r2] += rtb_Q[3 * rtemp] * A_2[r1];
      r3 = 3 * rtemp + 1;
      A_1[r2] += A[r1 + 3] * rtConstP.Innerloopcontrol_Ib[r3];
      A_3[r2] += A_2[r1 + 3] * rtb_Q[r3];
      r3 = 3 * rtemp + 2;
      A_1[r2] += A[r1 + 6] * rtConstP.Innerloopcontrol_Ib[r3];
      A_3[r2] += A_2[r1 + 6] * rtb_Q[r3];
    }

    for (rtemp = 0; rtemp < 3; rtemp++) {
      r2 = 3 * rtemp + r1;
      A_4[r2] = 0.0;
      A_4[r2] += B_0[3 * rtemp] * A_1[r1];
      A_4[r2] += B_0[3 * rtemp + 1] * A_1[r1 + 3];
      A_4[r2] += B_0[3 * rtemp + 2] * A_1[r1 + 6];
    }
  }

  // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  for (r1 = 0; r1 <= 6; r1 += 2) {
    // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
    // MATLAB Function: '<S2>/Inner-loop control'
    tmp_7 = _mm_loadu_pd(&A_3[r1]);
    tmp_8 = _mm_loadu_pd(&A_4[r1]);
    _mm_storeu_pd(&B_0[r1], _mm_add_pd(tmp_7, tmp_8));

    // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  }

  // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
  // MATLAB Function: '<S2>/Inner-loop control' incorporates:
  //   Inport: '<Root>/estimate'

  for (r1 = 8; r1 < 9; r1++) {
    B_0[r1] = A_3[r1] + A_4[r1];
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  rtb_AvoidDividebyZero = A[0];
  rtb_AvoidDividebyZero_k = std::abs(A[1]);
  if (rtb_AvoidDividebyZero_k > A[0]) {
    rtb_AvoidDividebyZero = rtb_AvoidDividebyZero_k;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(A[2]) > rtb_AvoidDividebyZero) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] /= A[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  A_3[0] = B_0[r1];
  A_3[1] = B_0[r2] - A_3[0] * A[r2];
  rtb_AvoidDividebyZero = A[r3 + 3];
  A_3[2] = (B_0[r3] - A_3[0] * A[r3]) - rtb_AvoidDividebyZero * A_3[1];
  rtb_AvoidDividebyZero_k = A[r3 + 6];
  A_3[2] /= rtb_AvoidDividebyZero_k;
  rtb_theta_d = A[r1 + 6];
  A_3[0] -= rtb_theta_d * A_3[2];
  rtb_phi_d = A[r2 + 6];
  A_3[1] -= rtb_phi_d * A_3[2];
  rtb_Q_b = A[r2 + 3];
  A_3[1] /= rtb_Q_b;
  v_0 = A[r1 + 3];
  A_3[0] -= v_0 * A_3[1];
  A_3[0] /= A[r1];
  A_3[3] = B_0[r1 + 3];
  A_3[4] = B_0[r2 + 3] - A_3[3] * A[r2];
  A_3[5] = (B_0[r3 + 3] - A_3[3] * A[r3]) - rtb_AvoidDividebyZero * A_3[4];
  A_3[5] /= rtb_AvoidDividebyZero_k;
  A_3[3] -= rtb_theta_d * A_3[5];
  A_3[4] -= rtb_phi_d * A_3[5];
  A_3[4] /= rtb_Q_b;
  A_3[3] -= v_0 * A_3[4];
  A_3[3] /= A[r1];
  A_3[6] = B_0[r1 + 6];
  A_3[7] = B_0[r2 + 6] - A_3[6] * A[r2];
  A_3[8] = (B_0[r3 + 6] - A_3[6] * A[r3]) - rtb_AvoidDividebyZero * A_3[7];
  A_3[8] /= rtb_AvoidDividebyZero_k;
  A_3[6] -= rtb_theta_d * A_3[8];
  A_3[7] -= rtb_phi_d * A_3[8];
  A_3[7] /= rtb_Q_b;
  A_3[6] -= v_0 * A_3[7];
  A_3[6] /= A[r1];
  for (r1 = 0; r1 < 3; r1++) {
    r2 = r1 * 3;
    v[r1] = (rtb_Q[r2 + 1] * rtU.estimate[4] + rtb_Q[r2] * rtU.estimate[3]) +
      rtb_Q[r2 + 2] * rtU.estimate[5];
  }

  // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  for (r1 = 0; r1 <= 16; r1 += 2) {
    // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
    // MATLAB Function: '<S2>/Inner-loop control'
    _mm_storeu_pd(&tmp[r1], _mm_mul_pd(_mm_loadu_pd
      (&rtConstP.Innerloopcontrol_Ke[r1]), _mm_set1_pd(-1.0)));

    // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  }

  // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
  // MATLAB Function: '<S2>/Inner-loop control' incorporates:
  //   Inport: '<Root>/dot_psi_d'
  //   Inport: '<Root>/eta_dot'
  //   Sum: '<S2>/Subtract1'

  rtb_Subtract_0[0] = rtb_Subtract_idx_0;
  rtb_Subtract_0[1] = rtb_Subtract_idx_1;
  rtb_Subtract_0[2] = rtb_Subtract_idx_2;
  rtb_Subtract_0[3] = rtU.eta_dot[0] - rtb_uT_idx_0;
  rtb_Subtract_0[4] = rtU.eta_dot[1] - rtb_uT_idx_1;
  rtb_Subtract_0[5] = rtU.eta_dot[2] - rtU.dot_psi_d;

  // SignalConversion generated from: '<S7>/ SFunction ' incorporates:
  //   Inport: '<Root>/ddot_psi_d'
  //   MATLAB Function: '<S2>/Inner-loop control'

  rtb_uT_e_0[0] = rtb_uT_e_idx_0;
  rtb_uT_e_0[1] = rtb_uT_e_idx_1;
  rtb_uT_e_0[2] = rtU.ddot_psi_d;

  // MATLAB Function: '<S2>/Inner-loop control' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'

  for (r1 = 0; r1 < 3; r1++) {
    rtb_tau[r1] = 0.0;
    for (rtemp = 0; rtemp < 6; rtemp++) {
      rtb_tau[r1] += tmp[3 * rtemp + r1] * rtb_Subtract_0[rtemp];
    }

    for (rtemp = 0; rtemp < 3; rtemp++) {
      r2 = 3 * rtemp + r1;
      B_0[r2] = 0.0;
      B_0[r2] += rtb_Q[3 * rtemp] * rtConstP.Innerloopcontrol_Ib[r1];
      B_0[r2] += rtb_Q[3 * rtemp + 1] * rtConstP.Innerloopcontrol_Ib[r1 + 3];
      B_0[r2] += rtb_Q[3 * rtemp + 2] * rtConstP.Innerloopcontrol_Ib[r1 + 6];
    }

    tmp_0[r1] = (rtb_tau[r1] - ((rtConstP.Innerloopcontrol_Ki_ang[r1 + 3] *
      rtDW.DiscreteTimeIntegrator_DSTATE[1] +
      rtConstP.Innerloopcontrol_Ki_ang[r1] * rtDW.DiscreteTimeIntegrator_DSTATE
      [0]) + rtConstP.Innerloopcontrol_Ki_ang[r1 + 6] *
      rtDW.DiscreteTimeIntegrator_DSTATE[2])) + rtb_uT_e_0[r1];
  }

  // MATLAB Function: '<S1>/MATLAB Function2' incorporates:
  //   MATLAB Function: '<S1>/Outer-loop control'
  //   SignalConversion generated from: '<S3>/ SFunction '

  B_1[0] = rtb_MinMax;

  // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  for (r1 = 0; r1 <= 0; r1 += 2) {
    __m128d tmp_1;
    __m128d tmp_2;
    __m128d tmp_3;
    __m128d tmp_4;
    __m128d tmp_5;
    __m128d tmp_6;

    // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
    // MATLAB Function: '<S2>/Inner-loop control' incorporates:
    //   Inport: '<Root>/eta_dot'

    tmp_7 = _mm_loadu_pd(&B_0[r1]);
    tmp_8 = _mm_set1_pd(0.0);
    tmp_1 = _mm_loadu_pd(&A_3[r1]);
    tmp_2 = _mm_loadu_pd(&B_0[r1 + 3]);
    tmp_3 = _mm_loadu_pd(&A_3[r1 + 3]);
    tmp_4 = _mm_loadu_pd(&B_0[r1 + 6]);
    tmp_5 = _mm_loadu_pd(&A_3[r1 + 6]);
    tmp_6 = _mm_loadu_pd(&v[r1]);
    tmp_7 = _mm_sub_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd
      (tmp_0[2])), _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(tmp_0[1])),
      _mm_add_pd(_mm_mul_pd(tmp_7, _mm_set1_pd(tmp_0[0])), tmp_8))), _mm_add_pd
      (_mm_mul_pd(tmp_5, _mm_set1_pd(rtU.eta_dot[2])), _mm_add_pd(_mm_mul_pd
      (tmp_3, _mm_set1_pd(rtU.eta_dot[1])), _mm_add_pd(_mm_mul_pd(tmp_1,
      _mm_set1_pd(rtU.eta_dot[0])), tmp_8)))), tmp_6);

    // MATLAB Function: '<S1>/MATLAB Function2' incorporates:
    //   MATLAB Function: '<S2>/Inner-loop control'

    _mm_storeu_pd(&B_1[r1 + 1], tmp_7);

    // MATLAB Function: '<S2>/Inner-loop control'
    _mm_storeu_pd(&rtb_tau[r1], tmp_7);

    // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  }

  // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
  for (r1 = 2; r1 < 3; r1++) {
    // MATLAB Function: '<S2>/Inner-loop control' incorporates:
    //   Inport: '<Root>/eta_dot'

    rtb_AvoidDividebyZero = (((B_0[r1 + 3] * tmp_0[1] + B_0[r1] * tmp_0[0]) +
      B_0[r1 + 6] * tmp_0[2]) + ((A_3[r1 + 3] * rtU.eta_dot[1] + A_3[r1] *
      rtU.eta_dot[0]) + A_3[r1 + 6] * rtU.eta_dot[2])) - v[r1];

    // MATLAB Function: '<S1>/MATLAB Function2'
    B_1[r1 + 1] = rtb_AvoidDividebyZero;

    // MATLAB Function: '<S2>/Inner-loop control'
    rtb_tau[r1] = rtb_AvoidDividebyZero;
  }

  // MATLAB Function: '<S1>/MATLAB Function2'
  std::memcpy(&A_0[0], &e[0], sizeof(real_T) << 4U);
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  for (rtemp = 0; rtemp < 3; rtemp++) {
    int32_T c_k;
    int32_T vectorUB;
    r1 = rtemp * 5;
    r2 = 4 - rtemp;
    r3 = 0;
    rtb_AvoidDividebyZero_k = std::abs(A_0[r1]);
    for (c_k = 2; c_k <= r2; c_k++) {
      rtb_AvoidDividebyZero = std::abs(A_0[(r1 + c_k) - 1]);
      if (rtb_AvoidDividebyZero > rtb_AvoidDividebyZero_k) {
        r3 = c_k - 1;
        rtb_AvoidDividebyZero_k = rtb_AvoidDividebyZero;
      }
    }

    if (A_0[r1 + r3] != 0.0) {
      if (r3 != 0) {
        r2 = rtemp + r3;
        ipiv[rtemp] = static_cast<int8_T>(r2 + 1);
        rtb_AvoidDividebyZero = A_0[rtemp];
        A_0[rtemp] = A_0[r2];
        A_0[r2] = rtb_AvoidDividebyZero;
        rtb_AvoidDividebyZero = A_0[rtemp + 4];
        A_0[rtemp + 4] = A_0[r2 + 4];
        A_0[r2 + 4] = rtb_AvoidDividebyZero;
        rtb_AvoidDividebyZero = A_0[rtemp + 8];
        A_0[rtemp + 8] = A_0[r2 + 8];
        A_0[r2 + 8] = rtb_AvoidDividebyZero;
        rtb_AvoidDividebyZero = A_0[rtemp + 12];
        A_0[rtemp + 12] = A_0[r2 + 12];
        A_0[r2 + 12] = rtb_AvoidDividebyZero;
      }

      r2 = (r1 - rtemp) + 4;
      c_k = (((((r2 - r1) - 1) / 2) << 1) + r1) + 2;
      vectorUB = c_k - 2;
      for (r3 = r1 + 2; r3 <= vectorUB; r3 += 2) {
        tmp_7 = _mm_loadu_pd(&A_0[r3 - 1]);
        _mm_storeu_pd(&A_0[r3 - 1], _mm_div_pd(tmp_7, _mm_set1_pd(A_0[r1])));
      }

      for (r3 = c_k; r3 <= r2; r3++) {
        A_0[r3 - 1] /= A_0[r1];
      }
    }

    r2 = 2 - rtemp;
    r3 = r1 + 6;
    for (c_k = 0; c_k <= r2; c_k++) {
      rtb_AvoidDividebyZero = A_0[((c_k << 2) + r1) + 4];
      if (rtb_AvoidDividebyZero != 0.0) {
        vectorUB = (r3 - rtemp) + 2;
        for (int32_T ijA{r3}; ijA <= vectorUB; ijA++) {
          A_0[ijA - 1] += A_0[((r1 + ijA) - r3) + 1] * -rtb_AvoidDividebyZero;
        }
      }

      r3 += 4;
    }
  }

  if (ipiv[0] != 1) {
    rtb_AvoidDividebyZero = B_1[0];
    B_1[0] = B_1[ipiv[0] - 1];
    B_1[ipiv[0] - 1] = rtb_AvoidDividebyZero;
  }

  if (ipiv[1] != 2) {
    rtb_AvoidDividebyZero = B_1[1];
    B_1[1] = B_1[ipiv[1] - 1];
    B_1[ipiv[1] - 1] = rtb_AvoidDividebyZero;
  }

  if (ipiv[2] != 3) {
    rtb_AvoidDividebyZero = B_1[2];
    B_1[2] = B_1[ipiv[2] - 1];
    B_1[ipiv[2] - 1] = rtb_AvoidDividebyZero;
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    r1 = rtemp << 2;
    if (B_1[rtemp] != 0.0) {
      for (r2 = rtemp + 2; r2 < 5; r2++) {
        B_1[r2 - 1] -= A_0[(r2 + r1) - 1] * B_1[rtemp];
      }
    }
  }

  for (rtemp = 3; rtemp >= 0; rtemp--) {
    r1 = rtemp << 2;
    if (B_1[rtemp] != 0.0) {
      B_1[rtemp] /= A_0[rtemp + r1];
      for (r3 = 0; r3 < rtemp; r3++) {
        B_1[r3] -= A_0[r3 + r1] * B_1[rtemp];
      }
    }
  }

  if (std::isnan(B_1[0])) {
    rtb_AvoidDividebyZero = (rtNaN);
  } else if (B_1[0] < 0.0) {
    rtb_AvoidDividebyZero = -1.0;
  } else {
    rtb_AvoidDividebyZero = (B_1[0] > 0.0);
  }

  // Outport: '<Root>/velocities' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function2'

  rtY.velocities[0] = std::sqrt(std::abs(B_1[0])) * rtb_AvoidDividebyZero;

  // MATLAB Function: '<S1>/MATLAB Function2'
  if (std::isnan(B_1[1])) {
    rtb_AvoidDividebyZero = (rtNaN);
  } else if (B_1[1] < 0.0) {
    rtb_AvoidDividebyZero = -1.0;
  } else {
    rtb_AvoidDividebyZero = (B_1[1] > 0.0);
  }

  // Outport: '<Root>/velocities' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function2'

  rtY.velocities[1] = std::sqrt(std::abs(B_1[1])) * rtb_AvoidDividebyZero;

  // MATLAB Function: '<S1>/MATLAB Function2'
  if (std::isnan(B_1[2])) {
    rtb_AvoidDividebyZero = (rtNaN);
  } else if (B_1[2] < 0.0) {
    rtb_AvoidDividebyZero = -1.0;
  } else {
    rtb_AvoidDividebyZero = (B_1[2] > 0.0);
  }

  // Outport: '<Root>/velocities' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function2'

  rtY.velocities[2] = std::sqrt(std::abs(B_1[2])) * rtb_AvoidDividebyZero;

  // MATLAB Function: '<S1>/MATLAB Function2'
  if (std::isnan(B_1[3])) {
    rtb_AvoidDividebyZero = (rtNaN);
  } else if (B_1[3] < 0.0) {
    rtb_AvoidDividebyZero = -1.0;
  } else {
    rtb_AvoidDividebyZero = (B_1[3] > 0.0);
  }

  // Outport: '<Root>/velocities' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function2'

  rtY.velocities[3] = std::sqrt(std::abs(B_1[3])) * rtb_AvoidDividebyZero;

  // Update for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_IC_LOADING = 0U;
  rtDW.Integrator_PrevResetState = 0;

  // Update for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_IC_LOADING_e = 0U;

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE[0] += 0.01 * rtb_Subtract_idx_0;

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator3'
  rtDW.DiscreteTimeIntegrator3_DSTATE[0] += tmp_9 * 0.01;

  // Update for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_DSTATE[0] += 0.01 * rtb_uT_idx_0;

  // Update for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_DSTATE_k[0] += 0.01 * rtb_uT_e_idx_0;

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE[1] += 0.01 * rtb_Subtract_idx_1;

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator3'
  rtDW.DiscreteTimeIntegrator3_DSTATE[1] += tmp_a * 0.01;

  // Update for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_DSTATE[1] += 0.01 * rtb_uT_idx_1;

  // Update for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_DSTATE_k[1] += 0.01 * rtb_uT_e_idx_1;

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE[2] += 0.01 * rtb_Subtract_idx_2;

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator3'
  rtDW.DiscreteTimeIntegrator3_DSTATE[2] += tmp_b * 0.01;

  // Update for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_DSTATE[2] += 0.01 * rtb_uT_idx_2;

  // Update for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_DSTATE_k[2] += 0.01 * rtb_uT_e_idx_2;
  rtDW.Integrator_PrevResetState_b = 0;

  // End of Outputs for SubSystem: '<Root>/HierarchicalControl'

  // Outport: '<Root>/Q'
  std::memcpy(&rtY.Q[0], &rtb_Q[0], 9U * sizeof(real_T));

  // Outport: '<Root>/u' incorporates:
  //   Constant: '<S1>/Constant'
  //   MATLAB Function: '<S1>/Outer-loop control'

  rtY.u[0] = 0.0;
  rtY.u[1] = 0.0;

  // Outputs for Atomic SubSystem: '<Root>/HierarchicalControl'
  rtY.u[2] = rtb_MinMax;

  // End of Outputs for SubSystem: '<Root>/HierarchicalControl'
  rtY.u[3] = rtb_tau[0];

  // Outport: '<Root>/mu_d'
  rtY.mu_d[0] = rtb_mu_d[0];

  // Outport: '<Root>/u'
  rtY.u[4] = rtb_tau[1];

  // Outport: '<Root>/mu_d'
  rtY.mu_d[1] = rtb_mu_d[1];

  // Outport: '<Root>/u'
  rtY.u[5] = rtb_tau[2];

  // Outport: '<Root>/mu_d'
  rtY.mu_d[2] = rtb_mu_d[2];

  // Outport: '<Root>/eta_d_dot' incorporates:
  //   Inport: '<Root>/dot_psi_d'

  rtY.eta_d_dot[2] = rtU.dot_psi_d;
  rtY.eta_d_dot[0] = rtb_uT_idx_0;

  // Outport: '<Root>/eta_d_ddot'
  rtY.eta_d_ddot[0] = rtb_uT_e_idx_0;

  // Outport: '<Root>/eta_d_dot'
  rtY.eta_d_dot[1] = rtb_uT_idx_1;

  // Outport: '<Root>/eta_d_ddot' incorporates:
  //   Inport: '<Root>/ddot_psi_d'

  rtY.eta_d_ddot[1] = rtb_uT_e_idx_1;
  rtY.eta_d_ddot[2] = rtU.ddot_psi_d;
}

// Model initialize function
void HierarchicalControl::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // SystemInitialize for Atomic SubSystem: '<Root>/HierarchicalControl'
  // Start for Probe: '<S13>/Probe'
  rtDW.Probe[0] = 0.01;
  rtDW.Probe[1] = 0.0;

  // Start for Probe: '<S8>/Probe'
  rtDW.Probe_e[0] = 0.01;
  rtDW.Probe_e[1] = 0.0;

  // InitializeConditions for DiscreteIntegrator: '<S12>/Integrator'
  rtDW.Integrator_IC_LOADING = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S17>/Integrator'
  rtDW.Integrator_IC_LOADING_e = 1U;

  // End of SystemInitialize for SubSystem: '<Root>/HierarchicalControl'
}

// Constructor
HierarchicalControl::HierarchicalControl() :
  rtU(),
  rtY(),
  rtDW(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
HierarchicalControl::~HierarchicalControl()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
HierarchicalControl::RT_MODEL * HierarchicalControl::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
