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
        Tail_On_1st,
        POS_ADJ_1st,
        LUG_Mode_1st,
        LUG_1st,
        Pre_1st_Turn,
        Turn_1st,
  
        Approach_to_2nd_LUG,
        LUG_Mode_2nd,
        LUG_2nd,
        Pre_2nd_Turn,
        Turn_2nd,
  
        Approach_to_3rd_LUG,
        LUG_Mode_3rd,
        LUG_3rd,
  
        Tail_Stand_Up,
  
        LUG_Debug_00
      };

    enumLUG_Mode LUG_Mode;

    PID *gForward = new PID();

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
        bool &tail_stand_mode,
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
        bool &tail_stand_mode,
        bool &tail_lug_mode,
        bool mRobo_lug_mode,
        int16_t mSonar_dis
    );
    int GarageRunner(int line_value_lug, float mOdo, float angle,int line_value);                                 //ガレージ走行
    int StopRobo(
        int &forward,
        float &yawratecmd,
        float &anglecommand,
        bool &tail_stand_mode
    );                                     //ロボット停止
    
};

#endif // !DIFFICULT_CTRL_H_
