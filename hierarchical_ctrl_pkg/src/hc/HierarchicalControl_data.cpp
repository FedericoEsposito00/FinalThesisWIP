//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: HierarchicalControl_data.cpp
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

// Constant parameters (default storage)
const HierarchicalControl::ConstP rtConstP{
  // Expression: Ib
  //  Referenced by: '<S2>/Inner-loop control'

  { 0.845, 0.0, 0.0, 0.0, 0.845, 0.0, 0.0, 0.0, 0.912 },

  // Expression: Ke
  //  Referenced by: '<S2>/Inner-loop control'

  { 100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 50.0, 10.0, 0.0, 0.0, 0.0, 10.0,
    0.0, 0.0, 0.0, 5.0 },

  // Expression: Ki_ang
  //  Referenced by: '<S2>/Inner-loop control'

  { 1, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 1 },

  // Expression: Ki
  //  Referenced by: '<S1>/Outer-loop control'

  { 1, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 1 },

  // Expression: Kp
  //  Referenced by: '<S1>/Outer-loop control'

  { 5.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 5.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0 }
};

//
// File trailer for generated code.
//
// [EOF]
//
