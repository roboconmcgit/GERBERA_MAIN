#include <stdlib.h>
#include "ev3api.h"
#include "ang_brain.h"
#define liting_radius 10; // liting spot radius [mm]
//#define DEBUG

Ang_Brain::Ang_Brain() {

}

void Ang_Brain::init() {
  gCommandCalc->init();
}

//ふるまいはコミュニケーション図に記載
void Ang_Brain::run() {
  
  //位置推定->走行戦略判定
  gStrategyDet->Det_run(mXvalue, mYvalue);        //3: 判定を実行

  GetStrategy(gStrategyDet->StrategyNum,
	      gStrategyDet->VirtualGateNum);      //4: 走行判定結果を取得
  
  //Running Strategy Determination->Robo Command
  gCommandCalc->SetCurrentData(mLinevalue,
			       mLinevalue_LUG,
			       mXvalue,
			       mYvalue,
			       mOdo,
			       mSpeed,
			       mYawrate,
			       mYawangle,
			       mTail_angle,
			       mRobo_stop,
			       mRobo_forward,
			       mRobo_back,
			       mRobo_turn_left,
			       mRobo_turn_right,
			       mDansa,
			       mDet_gray,
			       mSonar,
			       mRobo_balance_mode,
			       gStrategyDet->Max_Forward,
			       gStrategyDet->Max_Yawrate,
			       gStrategyDet->Min_Yawrate
			       );      //6: ロボットの現在情報を取得

  //  gCommandCalc->StrategyCalcRun(StrategyNum,VirtualGateNum);//7: 走行戦略を計算
  gCommandCalc->Track_run();
  
  GetCalcResult(gCommandCalc->forward,
		gCommandCalc->yawratecmd,
		gCommandCalc->anglecommand,
		gCommandCalc->tail_mode_lflag);   //8: 走行戦略の計算結果を取得
}

void Ang_Brain::setEyeCommand(int linevalue,
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
			      bool  det_gray,
				  int16_t sonar
			      ) {
  
  mLinevalue       = linevalue;
  mLinevalue_LUG   = linevalue_lug;
  mXvalue          = xvalue;
  mYvalue          = yvalue;
  mOdo             = odo; 
  mSpeed           = speed;
  mYawrate         = yawrate;
  mYawangle        = yawangle;
  mTail_angle      = robo_tail_angle;
  mRobo_stop       = robo_stop;
  mRobo_forward    = robo_forward;
  mRobo_back       = robo_back;
  mRobo_turn_left  = robo_turn_left;
  mRobo_turn_right = robo_turn_right;
  mDansa           = dansa;
  mDet_gray        = det_gray;
  mSonar           = sonar;
}


void Ang_Brain::setRoboCommand(bool robo_balance_mode){
  mRobo_balance_mode = robo_balance_mode;
}

void Ang_Brain::SetSysMode(int mode) {
  SysMode=static_cast<Sys_Mode>(mode);
}

void Ang_Brain::GetStrategy(int strategy_num, int virtualgate_num){

	Strategy=static_cast<enumStrategy>(strategy_num);
	VirtualGate=static_cast<enumVirtualGate>(virtualgate_num);

	StrategyNum=static_cast<int>(Strategy);
	VirtualGateNum=static_cast<int>(VirtualGate);
}

void Ang_Brain::GetCalcResult(int forward_calc,
			      float yawratecmd_calc,
			      float anglecommand_calc,
			      bool tail_mode_lflag_calc){
  
  forward         = forward_calc;
  yawratecmd      = yawratecmd_calc;
  anglecommand    = anglecommand_calc;
  tail_mode_lflag = tail_mode_lflag_calc;
}

