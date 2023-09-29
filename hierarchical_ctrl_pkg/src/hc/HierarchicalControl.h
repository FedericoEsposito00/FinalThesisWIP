//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: HierarchicalControl.h
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
#ifndef RTW_HEADER_HierarchicalControl_h_
#define RTW_HEADER_HierarchicalControl_h_
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

extern "C"
{
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"

// Class declaration for model HierarchicalControl
class HierarchicalControl final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    real_T Probe[2];                   // '<S13>/Probe'
    real_T Probe_e[2];                 // '<S8>/Probe'
    real_T DiscreteTimeIntegrator_DSTATE[3];// '<S2>/Discrete-Time Integrator'
    real_T DiscreteTimeIntegrator3_DSTATE[3];// '<S1>/Discrete-Time Integrator3' 
    real_T Integrator_DSTATE[3];       // '<S12>/Integrator'
    real_T Integrator_DSTATE_k[3];     // '<S17>/Integrator'
    int8_T Integrator_PrevResetState;  // '<S12>/Integrator'
    int8_T Integrator_PrevResetState_b;// '<S17>/Integrator'
    uint8_T Integrator_IC_LOADING;     // '<S12>/Integrator'
    uint8_T Integrator_IC_LOADING_e;   // '<S17>/Integrator'
  };

  // Constant parameters (default storage)
  struct ConstP {
    // Expression: Ib
    //  Referenced by: '<S2>/Inner-loop control'

    real_T Innerloopcontrol_Ib[9];

    // Expression: Ke
    //  Referenced by: '<S2>/Inner-loop control'

    real_T Innerloopcontrol_Ke[18];

    // Expression: Ki_ang
    //  Referenced by: '<S2>/Inner-loop control'

    real_T Innerloopcontrol_Ki_ang[9];

    // Expression: Ki
    //  Referenced by: '<S1>/Outer-loop control'

    real_T Outerloopcontrol_Ki[9];

    // Expression: Kp
    //  Referenced by: '<S1>/Outer-loop control'

    real_T Outerloopcontrol_Kp[18];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    real_T position[3];                // '<Root>/position'
    real_T linear_vel[3];              // '<Root>/linear_vel'
    real_T eta[3];                     // '<Root>/eta'
    real_T eta_dot[3];                 // '<Root>/eta_dot'
    real_T position_des[3];            // '<Root>/position_des'
    real_T vel_linear_des[3];          // '<Root>/vel_linear_des'
    real_T acc_linear_des[3];          // '<Root>/acc_linear_des'
    real_T psi_d;                      // '<Root>/psi_d'
    real_T dot_psi_d;                  // '<Root>/dot_psi_d'
    real_T ddot_psi_d;                 // '<Root>/ddot_psi_d'
    real_T estimate[6];                // '<Root>/estimate'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real_T u[6];                       // '<Root>/u'
    real_T velocities[4];              // '<Root>/velocities'
    real_T mu_d[3];                    // '<Root>/mu_d'
    real_T Q[9];                       // '<Root>/Q'
    real_T eta_d_dot[3];               // '<Root>/eta_d_dot'
    real_T eta_d_ddot[3];              // '<Root>/eta_d_ddot'
  };

  // Real-time Model Data Structure
  struct RT_MODEL {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  HierarchicalControl(HierarchicalControl const&) = delete;

  // Assignment Operator
  HierarchicalControl& operator= (HierarchicalControl const&) & = delete;

  // Move Constructor
  HierarchicalControl(HierarchicalControl &&) = delete;

  // Move Assignment Operator
  HierarchicalControl& operator= (HierarchicalControl &&) = delete;

  // Real-Time Model get method
  HierarchicalControl::RT_MODEL * getRTM();

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  HierarchicalControl();

  // Destructor
  ~HierarchicalControl();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // Real-Time Model
  RT_MODEL rtM;
};

// Constant parameters (default storage)
extern const HierarchicalControl::ConstP rtConstP;

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
//  Block '<S1>/Scope15' : Unused code path elimination
//  Block '<S1>/Scope16' : Unused code path elimination
//  Block '<S1>/Scope17' : Unused code path elimination
//  Block '<S1>/Scope2' : Unused code path elimination
//  Block '<S1>/Scope3' : Unused code path elimination
//  Block '<S1>/Scope4' : Unused code path elimination
//  Block '<S1>/Scope5' : Unused code path elimination
//  Block '<S1>/Scope6' : Unused code path elimination
//  Block '<S1>/Scope7' : Unused code path elimination
//  Block '<S1>/Scope8' : Unused code path elimination
//  Block '<S1>/Scope9' : Unused code path elimination
//  Block '<S1>/To Workspace1' : Unused code path elimination
//  Block '<S1>/To Workspace10' : Unused code path elimination
//  Block '<S1>/To Workspace11' : Unused code path elimination
//  Block '<S1>/To Workspace12' : Unused code path elimination
//  Block '<S1>/To Workspace13' : Unused code path elimination
//  Block '<S1>/To Workspace2' : Unused code path elimination
//  Block '<S1>/To Workspace3' : Unused code path elimination
//  Block '<S1>/To Workspace7' : Unused code path elimination
//  Block '<S1>/To Workspace8' : Unused code path elimination
//  Block '<S1>/To Workspace9' : Unused code path elimination
//  Block '<S5>/Gain' : Eliminated nontunable gain of 1
//  Block '<S12>/Saturation' : Eliminated Saturate block
//  Block '<S5>/[A,B]' : Eliminated Saturate block
//  Block '<S6>/Gain' : Eliminated nontunable gain of 1
//  Block '<S17>/Saturation' : Eliminated Saturate block
//  Block '<S6>/[A,B]' : Eliminated Saturate block


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
//  hilite_system('AerRob5_HierarchicalController/HierarchicalControl')    - opens subsystem AerRob5_HierarchicalController/HierarchicalControl
//  hilite_system('AerRob5_HierarchicalController/HierarchicalControl/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'AerRob5_HierarchicalController'
//  '<S1>'   : 'AerRob5_HierarchicalController/HierarchicalControl'
//  '<S2>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control'
//  '<S3>'   : 'AerRob5_HierarchicalController/HierarchicalControl/MATLAB Function2'
//  '<S4>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Outer-loop control'
//  '<S5>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)'
//  '<S6>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)1'
//  '<S7>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Inner-loop control'
//  '<S8>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)/Enable//disable time constant'
//  '<S9>'   : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)/Initialization'
//  '<S10>'  : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)/Integrator (Discrete or Continuous)'
//  '<S11>'  : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)/Initialization/Init_u'
//  '<S12>'  : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)/Integrator (Discrete or Continuous)/Discrete'
//  '<S13>'  : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)1/Enable//disable time constant'
//  '<S14>'  : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)1/Initialization'
//  '<S15>'  : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)1/Integrator (Discrete or Continuous)'
//  '<S16>'  : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)1/Initialization/Init_u'
//  '<S17>'  : 'AerRob5_HierarchicalController/HierarchicalControl/Inner-loop control/Filtered Derivative (Discrete or Continuous)1/Integrator (Discrete or Continuous)/Discrete'

#endif                                 // RTW_HEADER_HierarchicalControl_h_

//
// File trailer for generated code.
//
// [EOF]
//
