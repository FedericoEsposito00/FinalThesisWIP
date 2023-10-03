//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Estimator.cpp
//
// Code generated for Simulink model 'Estimator'.
//
// Model version                  : 4.37
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Mon Oct  2 15:20:27 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "Estimator.h"
#include <cmath>
#include <emmintrin.h>
#include "rtwtypes.h"

// Model step function
void Estimator::step()
{
  static const real_T b_0[3]{ 0.0, 0.0, 9.81 };

  static const int8_T b[9]{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  __m128d tmp_9;
  __m128d tmp_a;
  real_T tmp[36];
  real_T DiscreteTimeIntegrator_DSTATE_0[9];
  real_T DiscreteTimeIntegrator_DSTATE_t[9];
  real_T Q_tmp[9];
  real_T Q_tmp_0[9];
  real_T Q_tmp_1[9];
  real_T a[9];
  real_T rtb_Subtract[6];
  real_T tmp_0[6];
  real_T ct[3];
  real_T tmp_1[3];
  real_T Q_tmp_tmp_0;
  real_T Q_tmp_tmp_1;
  real_T Q_tmp_tmp_2;
  real_T Q_tmp_tmp_3;
  real_T Q_tmp_tmp_tmp;
  real_T ct_0;
  real_T st;
  int32_T Q_tmp_tmp;
  int32_T i;

  // Outputs for Atomic SubSystem: '<Root>/Estimator'
  // MATLAB Function: '<S1>/MATLAB Function' incorporates:
  //   Inport: '<Root>/eta'
  //   Inport: '<Root>/eta_dot'
  //   Inport: '<Root>/p_dot'
  //   MATLAB Function: '<S1>/MATLAB Function1'

  Q_tmp_tmp_1 = std::sin(rtU.eta[0]);
  Q_tmp_tmp_2 = std::cos(rtU.eta[1]);
  Q_tmp_tmp_0 = std::cos(rtU.eta[0]);
  Q_tmp_tmp_tmp = std::sin(rtU.eta[1]);
  Q_tmp[0] = 1.0;
  Q_tmp[3] = 0.0;
  Q_tmp[6] = -Q_tmp_tmp_tmp;
  Q_tmp[1] = 0.0;
  Q_tmp[4] = Q_tmp_tmp_0;
  Q_tmp[7] = Q_tmp_tmp_2 * Q_tmp_tmp_1;
  Q_tmp[2] = 0.0;
  Q_tmp[5] = -Q_tmp_tmp_1;
  Q_tmp_tmp_3 = Q_tmp_tmp_2 * Q_tmp_tmp_0;
  Q_tmp[8] = Q_tmp_tmp_3;
  for (int32_T i_0{0}; i_0 < 3; i_0++) {
    for (i = 0; i < 3; i++) {
      Q_tmp_tmp = 3 * i + i_0;
      Q_tmp_0[Q_tmp_tmp] = 0.0;
      Q_tmp_0[Q_tmp_tmp] += Q_tmp[3 * i_0] * rtConstP1.pooled1[3 * i];
      Q_tmp_0[Q_tmp_tmp] += Q_tmp[3 * i_0 + 1] * rtConstP1.pooled1[3 * i + 1];
      Q_tmp_0[Q_tmp_tmp] += Q_tmp[3 * i_0 + 2] * rtConstP1.pooled1[3 * i + 2];
    }

    for (i = 0; i < 3; i++) {
      Q_tmp_tmp = 3 * i + i_0;
      Q_tmp_1[Q_tmp_tmp] = 0.0;
      Q_tmp_1[Q_tmp_tmp] += Q_tmp[3 * i] * Q_tmp_0[i_0];
      Q_tmp_1[Q_tmp_tmp] += Q_tmp[3 * i + 1] * Q_tmp_0[i_0 + 3];
      Q_tmp_1[Q_tmp_tmp] += Q_tmp[3 * i + 2] * Q_tmp_0[i_0 + 6];
      Q_tmp_tmp = 6 * i_0 + i;
      tmp[Q_tmp_tmp] = static_cast<real_T>(b[3 * i_0 + i]) * 20.269;
      tmp[i + 6 * (i_0 + 3)] = 0.0;
      tmp[Q_tmp_tmp + 3] = 0.0;
    }
  }

  for (int32_T i_0{0}; i_0 < 3; i_0++) {
    i = (i_0 + 3) * 6;
    tmp[i + 3] = Q_tmp_1[3 * i_0];
    tmp[i + 4] = Q_tmp_1[3 * i_0 + 1];
    tmp[i + 5] = Q_tmp_1[3 * i_0 + 2];
    tmp_0[i_0] = rtU.p_dot[i_0];
    tmp_0[i_0 + 3] = rtU.eta_dot[i_0];
  }

  // Sum: '<S1>/Subtract' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   MATLAB Function: '<S1>/MATLAB Function'

  for (int32_T i_0{0}; i_0 < 6; i_0++) {
    ct_0 = 0.0;
    for (i = 0; i < 6; i++) {
      ct_0 += tmp[6 * i + i_0] * tmp_0[i];
    }

    rtb_Subtract[i_0] = ct_0 - rtDW.DiscreteTimeIntegrator_DSTATE[i_0];
  }

  real_T a_tmp;

  // End of Sum: '<S1>/Subtract'

  // MATLAB Function: '<S1>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/eta'
  //   Inport: '<Root>/eta_dot'
  //   MATLAB Function: '<S1>/MATLAB Function'

  ct_0 = std::cos(rtU.eta[2]);
  st = std::sin(rtU.eta[2]);
  a[0] = Q_tmp_tmp_2 * ct_0;
  a[3] = -Q_tmp_tmp_2 * st;
  a[6] = Q_tmp_tmp_tmp;
  a_tmp = Q_tmp_tmp_1 * ct_0;
  a[1] = a_tmp * Q_tmp_tmp_tmp + Q_tmp_tmp_0 * st;
  ct_0 *= Q_tmp_tmp_0;
  a[4] = ct_0 - Q_tmp_tmp_1 * Q_tmp_tmp_tmp * st;
  a[7] = -Q_tmp_tmp_2 * Q_tmp_tmp_1;
  a[2] = Q_tmp_tmp_1 * st - ct_0 * Q_tmp_tmp_tmp;
  a[5] = Q_tmp_tmp_0 * Q_tmp_tmp_tmp * st + a_tmp;
  a[8] = Q_tmp_tmp_3;
  for (int32_T i_0{0}; i_0 < 3; i_0++) {
    st = Q_tmp[i_0];
    ct_0 = st * rtU.eta_dot[0];
    Q_tmp_0[3 * i_0] = st;
    st = Q_tmp[i_0 + 3];
    ct_0 += st * rtU.eta_dot[1];
    Q_tmp_0[3 * i_0 + 1] = st;
    st = Q_tmp[i_0 + 6];
    Q_tmp_0[3 * i_0 + 2] = st;

    // Sum: '<S1>/Sum' incorporates:
    //   Inport: '<Root>/u'

    tmp_1[i_0] = (20.269 * b_0[i_0] - ((a[i_0 + 3] * rtU.u[1] + a[i_0] * rtU.u[0])
      + a[i_0 + 6] * rtU.u[2])) + rtb_Subtract[i_0];
    ct[i_0] = st * rtU.eta_dot[2] + ct_0;
  }

  a[0] = 0.0;
  a[3] = -ct[2];
  a[6] = ct[1];
  a[1] = ct[2];
  a[4] = 0.0;
  a[7] = -ct[0];
  a[2] = -ct[1];
  a[5] = ct[0];
  a[8] = 0.0;
  for (int32_T i_0{0}; i_0 < 3; i_0++) {
    for (i = 0; i < 3; i++) {
      Q_tmp_tmp = 3 * i + i_0;
      Q_tmp_1[Q_tmp_tmp] = 0.0;
      Q_tmp_1[Q_tmp_tmp] += a[3 * i] * Q_tmp_0[i_0];
      Q_tmp_1[Q_tmp_tmp] += a[3 * i + 1] * Q_tmp_0[i_0 + 3];
      Q_tmp_1[Q_tmp_tmp] += a[3 * i + 2] * Q_tmp_0[i_0 + 6];
    }

    for (i = 0; i < 3; i++) {
      Q_tmp_tmp = 3 * i + i_0;
      DiscreteTimeIntegrator_DSTATE_t[Q_tmp_tmp] = 0.0;
      DiscreteTimeIntegrator_DSTATE_t[Q_tmp_tmp] += rtConstP1.pooled1[3 * i] *
        Q_tmp_1[i_0];
      DiscreteTimeIntegrator_DSTATE_t[Q_tmp_tmp] += rtConstP1.pooled1[3 * i + 1] *
        Q_tmp_1[i_0 + 3];
      DiscreteTimeIntegrator_DSTATE_t[Q_tmp_tmp] += rtConstP1.pooled1[3 * i + 2] *
        Q_tmp_1[i_0 + 6];
    }
  }

  a[0] = 0.0;
  a[3] = 0.0;
  a[6] = -rtU.eta_dot[1] * Q_tmp_tmp_2;
  a[1] = 0.0;
  a[4] = -rtU.eta_dot[0] * Q_tmp_tmp_1;
  a[7] = -rtU.eta_dot[1] * Q_tmp_tmp_tmp * Q_tmp_tmp_1 + rtU.eta_dot[0] *
    Q_tmp_tmp_2 * Q_tmp_tmp_0;
  a[2] = 0.0;
  a[5] = -rtU.eta_dot[0] * Q_tmp_tmp_0;
  a[8] = -rtU.eta_dot[1] * std::sin(rtU.eta[1]) * Q_tmp_tmp_0 - rtU.eta_dot[0] *
    std::cos(rtU.eta[1]) * Q_tmp_tmp_1;
  for (int32_T i_0{0}; i_0 < 3; i_0++) {
    for (i = 0; i < 3; i++) {
      Q_tmp_tmp = 3 * i + i_0;
      Q_tmp_1[Q_tmp_tmp] = 0.0;
      DiscreteTimeIntegrator_DSTATE_0[Q_tmp_tmp] = 0.0;
      Q_tmp_1[Q_tmp_tmp] += rtConstP1.pooled1[3 * i] * Q_tmp_0[i_0];
      DiscreteTimeIntegrator_DSTATE_0[Q_tmp_tmp] += Q_tmp[3 * i_0] *
        DiscreteTimeIntegrator_DSTATE_t[i];
      Q_tmp_1[Q_tmp_tmp] += rtConstP1.pooled1[3 * i + 1] * Q_tmp_0[i_0 + 3];
      DiscreteTimeIntegrator_DSTATE_0[Q_tmp_tmp] += Q_tmp[3 * i_0 + 1] *
        DiscreteTimeIntegrator_DSTATE_t[i + 3];
      Q_tmp_1[Q_tmp_tmp] += rtConstP1.pooled1[3 * i + 2] * Q_tmp_0[i_0 + 6];
      DiscreteTimeIntegrator_DSTATE_0[Q_tmp_tmp] += Q_tmp[3 * i_0 + 2] *
        DiscreteTimeIntegrator_DSTATE_t[i + 6];
    }
  }

  for (int32_T i_0{0}; i_0 < 3; i_0++) {
    for (i = 0; i < 3; i++) {
      Q_tmp_tmp = 3 * i + i_0;
      DiscreteTimeIntegrator_DSTATE_t[Q_tmp_tmp] = 0.0;
      DiscreteTimeIntegrator_DSTATE_t[Q_tmp_tmp] += a[3 * i_0] * Q_tmp_1[i];
      DiscreteTimeIntegrator_DSTATE_t[Q_tmp_tmp] += a[3 * i_0 + 1] * Q_tmp_1[i +
        3];
      DiscreteTimeIntegrator_DSTATE_t[Q_tmp_tmp] += a[3 * i_0 + 2] * Q_tmp_1[i +
        6];
    }
  }

  // End of Outputs for SubSystem: '<Root>/Estimator'
  for (int32_T i_0{0}; i_0 <= 6; i_0 += 2) {
    // Outputs for Atomic SubSystem: '<Root>/Estimator'
    // MATLAB Function: '<S1>/MATLAB Function1'
    tmp_9 = _mm_loadu_pd(&DiscreteTimeIntegrator_DSTATE_0[i_0]);
    tmp_a = _mm_loadu_pd(&DiscreteTimeIntegrator_DSTATE_t[i_0]);
    _mm_storeu_pd(&Q_tmp_1[i_0], _mm_add_pd(tmp_9, tmp_a));

    // End of Outputs for SubSystem: '<Root>/Estimator'
  }

  // Outputs for Atomic SubSystem: '<Root>/Estimator'
  // MATLAB Function: '<S1>/MATLAB Function1'
  for (int32_T i_0{8}; i_0 < 9; i_0++) {
    Q_tmp_1[i_0] = DiscreteTimeIntegrator_DSTATE_0[i_0] +
      DiscreteTimeIntegrator_DSTATE_t[i_0];
  }

  // End of Outputs for SubSystem: '<Root>/Estimator'
  for (int32_T i_0{0}; i_0 <= 0; i_0 += 2) {
    __m128d tmp_2;
    __m128d tmp_3;
    __m128d tmp_4;
    __m128d tmp_5;
    __m128d tmp_6;
    __m128d tmp_7;
    __m128d tmp_8;
    __m128d tmp_b;

    // Outputs for Atomic SubSystem: '<Root>/Estimator'
    // MATLAB Function: '<S1>/MATLAB Function1'
    tmp_9 = _mm_loadu_pd(&Q_tmp_1[i_0]);
    tmp_a = _mm_set1_pd(0.0);
    tmp_2 = _mm_loadu_pd(&Q_tmp_0[i_0]);
    tmp_3 = _mm_loadu_pd(&Q_tmp_1[i_0 + 3]);
    tmp_4 = _mm_loadu_pd(&Q_tmp_0[i_0 + 3]);
    tmp_5 = _mm_loadu_pd(&Q_tmp_1[i_0 + 6]);
    tmp_6 = _mm_loadu_pd(&Q_tmp_0[i_0 + 6]);

    // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
    //   MATLAB Function: '<S1>/MATLAB Function1'

    tmp_7 = _mm_loadu_pd(&tmp_1[i_0]);
    tmp_b = _mm_set1_pd(0.01);
    tmp_8 = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i_0]);

    // MATLAB Function: '<S1>/MATLAB Function1'
    _mm_storeu_pd(&tmp_0[i_0], _mm_add_pd(_mm_mul_pd(tmp_b, tmp_7), tmp_8));

    // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
    //   MATLAB Function: '<S1>/MATLAB Function1'
    //   Sum: '<S1>/Sum'

    tmp_7 = _mm_loadu_pd(&rtb_Subtract[i_0 + 3]);
    tmp_8 = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i_0 + 3]);

    // MATLAB Function: '<S1>/MATLAB Function1' incorporates:
    //   Inport: '<Root>/eta_dot'
    //   Inport: '<Root>/tau'

    _mm_storeu_pd(&tmp_0[i_0 + 3], _mm_add_pd(_mm_mul_pd(_mm_add_pd(_mm_add_pd
      (_mm_add_pd(_mm_mul_pd(tmp_5, _mm_set1_pd(rtU.eta_dot[2])), _mm_add_pd
                  (_mm_mul_pd(tmp_3, _mm_set1_pd(rtU.eta_dot[1])), _mm_add_pd
                   (_mm_mul_pd(tmp_9, _mm_set1_pd(rtU.eta_dot[0])), tmp_a))),
       _mm_add_pd(_mm_mul_pd(tmp_6, _mm_set1_pd(rtU.tau[2])), _mm_add_pd
                  (_mm_mul_pd(tmp_4, _mm_set1_pd(rtU.tau[1])), _mm_add_pd
                   (_mm_mul_pd(tmp_2, _mm_set1_pd(rtU.tau[0])), tmp_a)))), tmp_7),
      tmp_b), tmp_8));

    // End of Outputs for SubSystem: '<Root>/Estimator'
  }

  // Outputs for Atomic SubSystem: '<Root>/Estimator'
  // MATLAB Function: '<S1>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   Inport: '<Root>/eta_dot'
  //   Inport: '<Root>/tau'
  //   Sum: '<S1>/Sum'

  for (int32_T i_0{2}; i_0 < 3; i_0++) {
    tmp_0[i_0] = 0.01 * tmp_1[i_0] + rtDW.DiscreteTimeIntegrator_DSTATE[i_0];
    tmp_0[i_0 + 3] = ((((Q_tmp_1[i_0 + 3] * rtU.eta_dot[1] + Q_tmp_1[i_0] *
                         rtU.eta_dot[0]) + Q_tmp_1[i_0 + 6] * rtU.eta_dot[2]) +
                       ((Q_tmp_0[i_0 + 3] * rtU.tau[1] + Q_tmp_0[i_0] * rtU.tau
                         [0]) + Q_tmp_0[i_0 + 6] * rtU.tau[2])) +
                      rtb_Subtract[i_0 + 3]) * 0.01 +
      rtDW.DiscreteTimeIntegrator_DSTATE[i_0 + 3];
  }

  for (int32_T i_0{0}; i_0 < 6; i_0++) {
    // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE[i_0] = tmp_0[i_0];

    // Outport: '<Root>/estimate'
    rtY.estimate[i_0] = rtb_Subtract[i_0];
  }

  // End of Outputs for SubSystem: '<Root>/Estimator'
}

// Model initialize function
void Estimator::initialize()
{
  // (no initialization code required)
}

// Constructor
Estimator::Estimator() :
  rtU(),
  rtY(),
  rtDW(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
Estimator::~Estimator()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
Estimator::RT_MODEL * Estimator::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
