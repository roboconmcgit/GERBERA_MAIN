/*
 *      モーターバーツクラスのヘッダファイル
 */

 #ifndef STEPRUN_H_
 #define STEPRUN_H_
 
 #include "ev3api.h"
 #include "parameter.h"
 #include "Clock.h"
 #include "util.h"
 #include "Brain_Calc_Library.h"
 
 #include "CruiseCtrl.h"
 
 using ev3api::Clock;
 /*
  *  関数のプロトタイプ宣言
  */
  
 class StepRun{
 private:
 
     Clock*       gClock;
     CruiseCtrl  *gCruiseCtrl;
 
     BrainCalcLibrary *gStep = new BrainCalcLibrary();       //段差走行オブジェクト（計算ライブラリ）
     PID *gForward = new PID();
   
     enum enumStep_Mode{
        Step_Start,
        Approach_to_Step,
        Change_Right_Edge_Trace,
        Right_Edge_On,
        First_Dansa,
        First_Dansa_On,
        First_Dansa_Tail_On,
        Fst_Turn_Pos_Adj,
        First_Turn,
        First_Pre_Stand_Up,
        First_Dansa_Stand_Up,
        Approach_to_2nd_Step,
        Pre_Second_Dansa,
        Second_Dansa,
        Second_Dansa_On,
        Second_Dansa_Tail_On,
        Second_Turn,
        Second_Pre_Stand_Up,
        Second_Dansa_Stand_Up,
        Approach_to_Exit,
  
        Change_Left_Edge_Trace,
        Left_edge_On,
        End_of_Step
    };
 
     enumStep_Mode   Step_Mode;
     float LineTracerYawrate(int line_value);
  
 public:    
     StepRun(CruiseCtrl* Cruise);     //コンストラクタ
     ~StepRun();                      //デストラクタ
 
     void init ();
     
     int StepRunner(
         int line_value, 
         float mOdo, 
         float angle, 
         bool dansa, 
         bool mRobo_balance_mode,
         int &forward,
         float &yawratecmd,
         float &anglecommand,
         bool &tail_stand_mode,
         float &ref_x,
         float mXvalue
     );//段差走行
 };
 
 #endif // !STEPRUN_H_
 