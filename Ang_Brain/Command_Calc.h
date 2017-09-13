/******************************************************************************
 *  ang_brain.h (for LEGO Mindstorms EV3)
 *  Created on: 2017/07/25
 *  Implementation of the Class ang_brain
 *  Author: Keiichi Tomii
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "parameter.h"
#include "util.h"
#include "Brain_Calc_Library.h"

class CommandCalc {
public:
	explicit CommandCalc();//コンストラクタ

	void init ();
	void SetCurrentData(int   linevalue,
			    int linevalue_lug,
                            float xvalue,
                            float yvalue,
			    float odo,
			    float speed,
                            float yawrate,
                            float yawangle,
			    int   robo_tail_angle,
			    bool  robo_stop,
			    bool  robo_forward,
			    bool  robo_back,
			    bool  robo_turn_left,
			    bool  robo_turn_right,
                            bool  dansa,
			    bool  det_gay,
          int32_t mSonar,
			    bool  robo_balance_mode,
			    int   max_forward,
			    float max_yawrate,
			    float min_yawrate
			    );//ロボの現在情報を取得

	void StrategyCalcRun(int strategy_num, int virtualgate_num);//走行戦略を計算
	void Track_run();

	int   forward;         //前進目標値
	float yawratecmd;      //目標ヨーレート
	float anglecommand;    //尻尾角度
	bool  tail_mode_lflag; //倒立走行フラグ
	int   log_dat_00;

private:


    void StartDashRunner();                              //スタートダッシュ
    float LineTracer(int line_value, float traceforward); //ライントレース
    void LineTracerYawrate(int line_value);              //ライントレース（ヨーレート）
    void MapTracer(int virtualgate_num);                 //仮想ゲート走行
    void StepRunner(int line_value, float odo, float angle, bool dansa);//段差走行
    //    void LookUpGateRunner();                             //ルックアップゲート走行
    void LookUpGateRunner(int line_value_lug, float odo, float angle,int line_value);
    void GarageRunner(int line_value_lug, float odo, float angle,int line_value);                                 //ガレージ走行
    void StopRobo();                                     //ロボット停止

    BrainCalcLibrary *gStartDash = new BrainCalcLibrary();  //スタートダッシュオブジェクト（脳みそ計算ライブラリ）
    BrainCalcLibrary *gLineTracer1 = new BrainCalcLibrary();//ライントレースオブジェクト（脳みそ計算ライブラリ）
    BrainCalcLibrary *gMapTrace = new BrainCalcLibrary();   //ライントレース（ヨーレート）オブジェクト（脳みそ計算ライブラリ）
    BrainCalcLibrary *gGoal = new BrainCalcLibrary();       //仮想ゲート走行オブジェクト（脳みそ計算ライブラリ）
    BrainCalcLibrary *gStep = new BrainCalcLibrary();       //段差走行オブジェクト（脳みそ計算ライブラリ）
    BrainCalcLibrary *gLookUpGate = new BrainCalcLibrary(); //ルックアップゲート走行オブジェクト（脳みそ計算ライブラリ）
    BrainCalcLibrary *gGarage = new BrainCalcLibrary();     //ガレージ走行オブジェクト（脳みそ計算ライブラリ）
    BrainCalcLibrary *gStop = new BrainCalcLibrary();       //ロボット停止オブジェクト（脳みそ計算ライブラリ）

    PID *gForward = new PID();

    enum enumStrategy{
      StartDash=510,
      LineTrace1=520,
      MapTrace=530,
      Goal=610,
      Goal2Step=650,
      Step=710,
      LookUpGate=810,
      Garage=910,
      Stop=1010
    };

    enum enumVirtualGate{
      Gate12=531,
      Gate23=532,
      Gate34=533,
      Gate45=534,
      Gate56=535,
      Gate67=536,
      Gate78=537,
      Gate89=538,
      None=539
    };

    enum enumTrack_Mode{
      Start_to_1st_Straight,
      Start_to_1st_Corner,
      Fst_Corner,
      Snd_Corner,
      Final_Corner,
      Final_Straight,
      Dead_Zone,
      Return_to_Line,
      Go_Step,
      Approach_to_Garage,
      Return_to_Line_Garage,
      Garage_In,
      Stop_Robo,
      Go_LUG,
      Track_Debug_00,
      Track_Debug_01,
      Track_Debug_02
    };

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

    enumStrategy    Strategy;
    enumVirtualGate VirtualGate;
    enumTrack_Mode  Track_Mode;
    enumStep_Mode   Step_Mode;
    enumLUG_Mode LUG_Mode;


    int   Mmode;
    int   mLinevalue; //ライン検出値
    int   mLinevalue_LUG;
    float mXvalue;    //x座標
    float mYvalue;    //y座標
    float mOdo;       //Total distance [mm] from start point
    float mSpeed;     //速度
    float mYawrate;   //ヨーレート
    float mYawangle;  //ヨー角
    int   mTail_angle;
    //signals for robo movement
    bool  mRobo_stop       = 0;
    bool  mRobo_forward    = 0;
    bool  mRobo_back       = 0;
    bool  mRobo_turn_left  = 0;
    bool  mRobo_turn_right = 0;

    bool  mDansa;      //段差検出値
    bool  mDet_gray;      //段差検出値

    int32_t mSonar;
    bool  mGarage = false;

    bool  mRobo_balance_mode;
    int   mMax_Forward;
    float mMax_Yawrate;
    float mMin_Yawrate;

    //	bool tail_mode_lflag_calc; // 倒立走行フラグ 0817

    float y_t;
    float y_t_prev; //0818 tada. passed y_t
    float pg = 0.35;//暫定0.9 //0818 tada
    float df = 0.024;//暫定-0.1 //0818 tada

};
