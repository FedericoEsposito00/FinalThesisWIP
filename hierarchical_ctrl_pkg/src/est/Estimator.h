//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Estimator.h
//
// Code generated for Simulink model 'Estimator'.
//
// Model version                  : 4.40
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Fri Oct 13 11:46:31 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_Estimator_h_
#define RTW_HEADER_Estimator_h_
#include "rtwtypes.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model Estimator
class Estimator final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    real_T DiscreteTimeIntegrator_DSTATE[6];// '<S1>/Discrete-Time Integrator'
    real_T DiscreteTimeIntegrator1_DSTATE[6];// '<S1>/Discrete-Time Integrator1' 
  };

  // Constant parameters (default storage)
  struct ConstP {
    // Pooled Parameter (Expression: Ib)
    //  Referenced by:
    //    '<S1>/MATLAB Function'
    //    '<S1>/MATLAB Function1'

    real_T pooled1[9];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    real_T eta[3];                     // '<Root>/eta'
    real_T eta_dot[3];                 // '<Root>/eta_dot'
    real_T u[3];                       // '<Root>/u'
    real_T tau[3];                     // '<Root>/tau'
    real_T p_dot[3];                   // '<Root>/p_dot'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real_T estimate[6];                // '<Root>/estimate'
  };

  // Real-time Model Data Structure
  struct RT_MODEL {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  Estimator(Estimator const&) = delete;

  // Assignment Operator
  Estimator& operator= (Estimator const&) & = delete;

  // Move Constructor
  Estimator(Estimator &&) = delete;

  // Move Assignment Operator
  Estimator& operator= (Estimator &&) = delete;

  // Real-Time Model get method
  Estimator::RT_MODEL * getRTM();

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  static void initialize();

  // model step function
  void step();

  // Constructor
  Estimator();

  // Destructor
  ~Estimator();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // Real-Time Model
  RT_MODEL rtM;
};

// Constant parameters (default storage)
extern const Estimator::ConstP rtConstP1;

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
//  hilite_system('AerRob5_HierarchicalController/HierarchicalControl/Estimator')    - opens subsystem AerRob5_HierarchicalController/HierarchicalControl/Estimator
//  hilite_system('AerRob5_HierarchicalController/HierarchicalControl/Estimator/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'AerRob5_HierarchicalController/HierarchicalControl'
//  '<S1>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Estimator'
//  '<S2>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Estimator/MATLAB Function'
//  '<S3>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Estimator/MATLAB Function1'

#endif                                 // RTW_HEADER_Estimator_h_

//
// File trailer for generated code.
//
// [EOF]
//
