/*
 *      モーターバーツクラスのヘッダファイル
 */

#ifndef DIFFICULT_CTRL_H_
#define DIFFICULT_CTRL_H_

#if 0
#include "ev3api.h"
#include "Clock.h"
#include "parameter.h"
#include "util.h"
#include "Brain_Calc_Library.h"

using ev3api::Clock;
/*
 *  関数のプロトタイプ宣言
 */
 
class DifficultCtrl{
private:

    Clock*       gClock;
    CruiseCtrl  *gCruiseCtrl;

    BrainCalcLibrary *gStep = new BrainCalcLibrary();       //段差走行オブジェクト（計算ライブラリ）

    int   Forward;
    float Yawratecmd;//目標Yawrate
    float anglecommand;
    float target_tail_angle
	bool  tail_mode_lflag; //倒立走行フラグ
    
    enum enumStep_Mode{
      Step_Start,
      Approach_to_Step,
      Change_Right_Edge_Trace,
      Right_Edge_On,
      First_Dansa,
      First_Dansa_On,
      First_Dansa_Tail_On,
      First_Turn,
      First_Pre_Stand_Up,
      First_Dansa_Stand_Up,
      Approach_to_2nd_Step,
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

    enumStep_Mode   Step_Mode;
    enumLUG_Mode LUG_Mode;

public:


	int   forward;         //前進目標値
	float anglecommand;    //尻尾角度
	int   log_dat_00;
    DifficultCtrl();                       //コンストラクタ
    ~DifficultCtrl();                      //デストラクタ
    void DifficultCtrl::init();

	void init ();
    int StartDashRunner();                              //スタートダッシュ
    int StepRunner(int line_value, float odo, float angle, bool dansa, bool Robo_balance_mode);//段差走行
    //    void LookUpGateRunner();                             //ルックアップゲート走行
    int LookUpGateRunner(int line_value_lug, float odo, float angle,int line_value);
    int GarageRunner(int line_value_lug, float odo, float angle,int line_value);                                 //ガレージ走行
    int StopRobo();                                     //ロボット停止
    
};
#endif

#endif // !DIFFICULT_CTRL_H_
