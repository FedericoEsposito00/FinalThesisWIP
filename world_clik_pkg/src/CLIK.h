//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: CLIK.h
//
// Code generated for Simulink model 'CLIK'.
//
// Model version                  : 3.85
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Sun Oct  8 12:06:07 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_CLIK_h_
#define RTW_HEADER_CLIK_h_
#include "rtwtypes.h"
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

extern "C"
{
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"

extern "C"
{
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"

#define NOT_USING_NONFINITE_LITERALS   1

extern "C"
{
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
  struct BigEndianIEEEDouble {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  };

  struct LittleEndianIEEEDouble {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  };

  struct IEEESingle {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  };
}                                      // extern "C"

// Class declaration for model CLIK
class CLIK final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    real_T Probe[2];                   // '<S13>/Probe'
    real_T Probe_n[2];                 // '<S8>/Probe'
    real_T Integrator_DSTATE[3];       // '<S17>/Integrator'
    real_T Integrator_DSTATE_a[3];     // '<S12>/Integrator'
    real_T DiscreteTimeIntegrator1_DSTATE;// '<S1>/Discrete-Time Integrator1'
    real_T DiscreteTimeIntegrator2_DSTATE;// '<S1>/Discrete-Time Integrator2'
    real_T DiscreteTimeIntegrator3_DSTATE;// '<S1>/Discrete-Time Integrator3'
    real_T DiscreteTimeIntegrator4_DSTATE;// '<S1>/Discrete-Time Integrator4'
    real_T DiscreteTimeIntegrator5_DSTATE;// '<S1>/Discrete-Time Integrator5'
    real_T DiscreteTimeIntegrator6_DSTATE;// '<S1>/Discrete-Time Integrator6'
    real_T DiscreteTimeIntegrator7_DSTATE;// '<S1>/Discrete-Time Integrator7'
    real_T DiscreteTimeIntegrator8_DSTATE;// '<S1>/Discrete-Time Integrator8'
    int8_T Integrator_PrevResetState;  // '<S17>/Integrator'
    int8_T Integrator_PrevResetState_f;// '<S12>/Integrator'
    uint8_T Integrator_IC_LOADING;     // '<S17>/Integrator'
    uint8_T Integrator_IC_LOADING_d;   // '<S12>/Integrator'
  };

  // Constant parameters (default storage)
  struct ConstP {
    // Pooled Parameter (Expression: eye(3)*10)
    //  Referenced by:
    //    '<S1>/Gain1'
    //    '<S1>/Gain2'

    real_T pooled6[9];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    real_T xd[3];                      // '<Root>/xd'
    real_T q1l;                        // '<Root>/q1l'
    real_T q2l;                        // '<Root>/q2l'
    real_T q3l;                        // '<Root>/q3l'
    real_T q4l;                        // '<Root>/q4l'
    real_T q1r;                        // '<Root>/q1r'
    real_T q2r;                        // '<Root>/q2r'
    real_T q3r;                        // '<Root>/q3r'
    real_T q4r;                        // '<Root>/q4r'
    real_T load;                       // '<Root>/load'
    real_T K_second;                   // '<Root>/K_second'
    real_T q1l_n;                      // '<Root>/q1l_n'
    real_T q2l_n;                      // '<Root>/q2l_n'
    real_T q3l_n;                      // '<Root>/q3l_n'
    real_T q4l_n;                      // '<Root>/q4l_n'
    real_T q1r_n;                      // '<Root>/q1r_n'
    real_T q2r_n;                      // '<Root>/q2r_n'
    real_T q3r_n;                      // '<Root>/q3r_n'
    real_T q4r_n;                      // '<Root>/q4r_n'
    real_T K_l1M;                      // '<Root>/K_l1M'
    real_T K_l1m;                      // '<Root>/K_l1m'
    real_T K_l2M;                      // '<Root>/K_l2M'
    real_T K_l2m;                      // '<Root>/K_l2m'
    real_T K_l3M;                      // '<Root>/K_l3M'
    real_T K_l3m;                      // '<Root>/K_l3m'
    real_T K_l4M;                      // '<Root>/K_l4M'
    real_T K_l4m;                      // '<Root>/K_l4m'
    real_T x_shoulders;                // '<Root>/x_shoulders'
    real_T y_shoulders;                // '<Root>/y_shoulders'
    real_T z_shoulders;                // '<Root>/z_shoulders'
    real_T roll_shoulders;             // '<Root>/roll_shoulders'
    real_T pitch_shoulders;            // '<Root>/pitch_shoulders'
    real_T yaw_shoulders;              // '<Root>/yaw_shoulders'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real_T q1l_d;                      // '<Root>/q1l_d'
    real_T q2l_d;                      // '<Root>/q2l_d'
    real_T q3l_d;                      // '<Root>/q3l_d'
    real_T q4l_d;                      // '<Root>/q4l_d'
    real_T q1r_d;                      // '<Root>/q1r_d'
    real_T q2r_d;                      // '<Root>/q2r_d'
    real_T q3r_d;                      // '<Root>/q3r_d'
    real_T q4r_d;                      // '<Root>/q4r_d'
    real_T CoM_bar[3];                 // '<Root>/CoM_bar'
    real_T CoM[3];                     // '<Root>/CoM'
    real_T CoM_bar1[3];                // '<Root>/CoM_bar1'
    real_T CoM1[3];                    // '<Root>/CoM1'
  };

  // Real-time Model Data Structure
  struct RT_MODEL {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  CLIK(CLIK const&) = delete;

  // Assignment Operator
  CLIK& operator= (CLIK const&) & = delete;

  // Move Constructor
  CLIK(CLIK &&) = delete;

  // Move Assignment Operator
  CLIK& operator= (CLIK &&) = delete;

  // Real-Time Model get method
  CLIK::RT_MODEL * getRTM();

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  CLIK();

  // Destructor
  ~CLIK();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // private member function(s) for subsystem '<Root>'
  void eye(real_T b_I[16]);
  void mrdiv(const real_T A[12], const real_T B_0[9], real_T Y[12]);

  // Real-Time Model
  RT_MODEL rtM;
};

// Constant parameters (default storage)
extern const CLIK::ConstP rtConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Scope' : Unused code path elimination
//  Block '<S1>/Scope1' : Unused code path elimination
//  Block '<S1>/Scope10' : Unused code path elimination
//  Block '<S1>/Scope11' : Unused code path elimination
//  Block '<S1>/Scope12' : Unused code path elimination
//  Block '<S1>/Scope13' : Unused code path elimination
//  Block '<S1>/Scope14' : Unused code path elimination
//  Block '<S1>/Scope16' : Unused code path elimination
//  Block '<S1>/Scope17' : Unused code path elimination
//  Block '<S1>/Scope18' : Unused code path elimination
//  Block '<S1>/Scope19' : Unused code path elimination
//  Block '<S1>/Scope20' : Unused code path elimination
//  Block '<S1>/Scope21' : Unused code path elimination
//  Block '<S1>/Scope22' : Unused code path elimination
//  Block '<S1>/Scope23' : Unused code path elimination
//  Block '<S1>/Scope24' : Unused code path elimination
//  Block '<S1>/Scope25' : Unused code path elimination
//  Block '<S1>/Scope26' : Unused code path elimination
//  Block '<S1>/Scope27' : Unused code path elimination
//  Block '<S1>/Scope28' : Unused code path elimination
//  Block '<S1>/Scope29' : Unused code path elimination
//  Block '<S1>/Scope3' : Unused code path elimination
//  Block '<S1>/Scope30' : Unused code path elimination
//  Block '<S1>/Scope31' : Unused code path elimination
//  Block '<S1>/Scope32' : Unused code path elimination
//  Block '<S1>/Scope33' : Unused code path elimination
//  Block '<S1>/Scope34' : Unused code path elimination
//  Block '<S1>/Scope35' : Unused code path elimination
//  Block '<S1>/Scope36' : Unused code path elimination
//  Block '<S1>/Scope37' : Unused code path elimination
//  Block '<S1>/Scope38' : Unused code path elimination
//  Block '<S1>/Scope39' : Unused code path elimination
//  Block '<S1>/Scope4' : Unused code path elimination
//  Block '<S1>/Scope40' : Unused code path elimination
//  Block '<S1>/Scope5' : Unused code path elimination
//  Block '<S1>/Scope6' : Unused code path elimination
//  Block '<S1>/Scope7' : Unused code path elimination
//  Block '<S1>/Scope8' : Unused code path elimination
//  Block '<S1>/Scope9' : Unused code path elimination
//  Block '<S2>/Gain' : Eliminated nontunable gain of 1
//  Block '<S12>/Saturation' : Eliminated Saturate block
//  Block '<S2>/[A,B]' : Eliminated Saturate block
//  Block '<S3>/Gain' : Eliminated nontunable gain of 1
//  Block '<S17>/Saturation' : Eliminated Saturate block
//  Block '<S3>/[A,B]' : Eliminated Saturate block


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('WorldBased_CLIK_pinv_sendtoros/CLIK')    - opens subsystem WorldBased_CLIK_pinv_sendtoros/CLIK
//  hilite_system('WorldBased_CLIK_pinv_sendtoros/CLIK/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'WorldBased_CLIK_pinv_sendtoros'
//  '<S1>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK'
//  '<S2>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)'
//  '<S3>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)1'
//  '<S4>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK/dirkin_l'
//  '<S5>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK/dirkin_r'
//  '<S6>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK/q_l_dot'
//  '<S7>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK/q_r_dot'
//  '<S8>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)/Enable//disable time constant'
//  '<S9>'   : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)/Initialization'
//  '<S10>'  : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)/Integrator (Discrete or Continuous)'
//  '<S11>'  : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)/Initialization/Init_u'
//  '<S12>'  : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)/Integrator (Discrete or Continuous)/Discrete'
//  '<S13>'  : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)1/Enable//disable time constant'
//  '<S14>'  : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)1/Initialization'
//  '<S15>'  : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)1/Integrator (Discrete or Continuous)'
//  '<S16>'  : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)1/Initialization/Init_u'
//  '<S17>'  : 'WorldBased_CLIK_pinv_sendtoros/CLIK/Filtered Derivative (Discrete or Continuous)1/Integrator (Discrete or Continuous)/Discrete'

#endif                                 // RTW_HEADER_CLIK_h_

//
// File trailer for generated code.
//
// [EOF]
//
