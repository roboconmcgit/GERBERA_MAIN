/*
 *      モーターバーツクラスのヘッダファイル
 */

#ifndef DIFFICULT_CTRL_H_
#define DIFFICULT_CTRL_H_

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
 
class DifficultCtrl{
private:

    Clock*       gClock;
    CruiseCtrl  *gCruiseCtrl;

    BrainCalcLibrary *gStep = new BrainCalcLibrary();       //段差走行オブジェクト（計算ライブラリ）
  
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

    enum enumLUG_Mode{
	  LUG_Start,
	  Approach_to_LUG,
      LUG_Tail_On,
	  LUG_Tailangle,
	  LUG_Stop0,
	  LUG_forward,
	  LUG_Stop1,
	  LUG_Trun1,
	  LUG_Stop2,
	  LUG_forward2,
	  LUG_Stop3,
	  LUG_Trun2,
	  LUG_Stop4,
	  LUG_forward3,
	  LUG_Stop5,
	  LUG_Stand_Up,
	  LUG_Stop6,
	  Garage_Turn,
	  Garage_Stop0,
	  END,
    };

    enumLUG_Mode LUG_Mode;

public:    
    DifficultCtrl(CruiseCtrl* Cruise);     //コンストラクタ
    ~DifficultCtrl();                      //デストラクタ

    void init ();
    float LineTracerYawrate(int line_value); //Dummy
    float LineTracer(int line_value, float traceforward);
    
    int StartDashRunner();                              //スタートダッシュ
    int StepRunner(
        int line_value, 
        float mOdo, 
        float angle, 
        bool dansa, 
        bool mRobo_balance_mode,
        int &forward,
        float &yawratecmd,
        float &anglecommand,
        bool &tail_mode_lflag,
        float &ref_x,
        float mXvalue
    );//段差走行
    int LookUpGateRunner(
        int line_value_lug, 
        float odo, 
        float angle,
        int line_value, 
        bool mRobo_balance_mode,
        int &forward,
        float &yawratecmd,
        float &anglecommand,
        bool &tail_mode_lflag
    );
    int GarageRunner(int line_value_lug, float mOdo, float angle,int line_value);                                 //ガレージ走行
    int StopRobo(
        int &forward,
        float &yawratecmd,
        float &anglecommand,
        bool &tail_mode_lflag
    );                                     //ロボット停止
    
};

#endif // !DIFFICULT_CTRL_H_
