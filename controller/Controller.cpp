/**
 ******************************************************************************
 ** ファイル名 : Controller.cpp
 **
 ** 概要 : コントローラクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/
#include <stdlib.h>
#include "Controller.h"

#define liting_radius 10; // liting spot radius [mm]

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//*****************************************************************************
// 関数名 : コンストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
Controller::Controller(
	ColorParts  *Color,
	MotorParts  *Motor,
	GyroParts   *Gyro,
	SonarParts  *Sonar,
	TouchParts  *Touch):
	gColorParts(Color),
    gMotorParts(Motor),
    gGyroParts(Gyro),
    gSonarParts(Sonar),
	gTouchParts(Touch),
	mSys_Mode(SYS_INIT)
	{
	gBalancer  = new Balancer();
	gCruiseCtrl  = new CruiseCtrl(gGyroParts,
						  gMotorParts,
						  gBalancer);
#if 0
    gDifficultCtrl = new DifficultCtrl(gCruiseCtrl);
#endif
}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
Controller::~Controller(){
	delete gColorParts;
	delete gMotorParts;
	delete gGyroParts;
	delete gSonarParts;
	delete gTouchParts;

	delete gBalancer;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Controller::ControllerInit(){
	gCruiseCtrl->init();  //
}

//*****************************************************************************
// 関数名 : ControllerOperation
// 引数 : 
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Controller::ControllerOperation(){

	//Running Strategy Determination->Robo Command
	Track_run();
	
	gCruiseCtrl->setCommand(forward,
		yawratecmd,
		anglecommand,
		gMotorParts->yawrate,
		tail_mode_lflag);//指令値をあなご手足に渡す

	if(mSys_Mode == START){
		gCruiseCtrl->CruiseCtrlOperation();
	}
}

//*****************************************************************************
// 関数名 : ControllerOperation
// 引数 : 
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Controller::Track_run() {
	float Max_Yawrate;
	float Min_Yawrate;
	static float   ref_odo;
	
	  int dammy_line_value;
	  int speedcal = 0;
	  float y_t;
	
	
	  switch(Track_Mode){
	
	  case Start_to_1st_Straight:
		break;
	
	  case Start_to_1st_Corner:
	
		forward =  100;
		Max_Yawrate = 1.5;
		Min_Yawrate = -1.5;
		yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, Max_Yawrate, Min_Yawrate);
		anglecommand = TAIL_ANGLE_RUN; //0817 tada
		tail_mode_lflag = false;
	
		if(mYawangle < -2){
		  Track_Mode = Snd_Corner;
		}
		break;
	
	  case Snd_Corner:
		forward =  100;
		Max_Yawrate = 5.0;
		Min_Yawrate = -5.0;
		yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, Max_Yawrate, Min_Yawrate);
		anglecommand = TAIL_ANGLE_RUN; //0817 tada
		tail_mode_lflag = false;
	
		if(mYawangle > 0.5){
		  Track_Mode = Final_Corner;
		}
		break;
	
	  case Final_Corner:
		forward =  100;
		Max_Yawrate = 5.0;
		Min_Yawrate = -5.0;
		yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, Max_Yawrate, Min_Yawrate);
		anglecommand = TAIL_ANGLE_RUN; //0817 tada
		tail_mode_lflag = false;
	
		if((mYawangle < 1.0) &&(mRobo_forward == 1)){
		  Track_Mode = Final_Straight;
		  ref_odo = mOdo +  FINAL_STRAIGHT_LENGTH;
		}
		break;
	
	  case Final_Straight:
		forward =  100;
		Max_Yawrate = 1.0;
		Min_Yawrate = -1.0;
		yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, Max_Yawrate, Min_Yawrate);
		anglecommand = TAIL_ANGLE_RUN; //0817 tada
		tail_mode_lflag = false;
	
		if(mOdo > ref_odo){
		  //Track_Mode = Dead_Zone;
		  //ref_odo = mOdo + DEAD_ZONE_LENGTH;
		  Track_Mode = Return_to_Line;
		}
	
		break;
	
	  case Dead_Zone:
		forward =  50;
		Max_Yawrate = 1.0;
		Min_Yawrate = -1.0;
		dammy_line_value = 50 - 300*mYawangle;
		if(dammy_line_value > 100){
		  dammy_line_value = 100;
		}else if(dammy_line_value < 0){
		  dammy_line_value = 0;
		}
	
		yawratecmd = gCruiseCtrl->LineTracerYawrate(dammy_line_value, Max_Yawrate, Min_Yawrate);
	
		if(mOdo > ref_odo){
		  Track_Mode = Return_to_Line;
		}
		break;
	
	  case Return_to_Line:
		forward =  50;
		Max_Yawrate = 1.5;
		Min_Yawrate = -1.5;
		yawratecmd = gCruiseCtrl->LineTracerYawrate((2*mLinevalue), Max_Yawrate, Min_Yawrate);
		anglecommand = TAIL_ANGLE_RUN; //0817 tada
		tail_mode_lflag = false;
	
		if((mYawangle <  MINUS_RAD_5_DEG)&&(yawratecmd > 0) ){
		  yawratecmd = 0.0;
		}
	
	
		if((mYawangle > 2.5) &&(mRobo_forward == 1)){
		  forward =  30;
		  Max_Yawrate = 5.0;
		  Min_Yawrate = -5.0;
		  yawratecmd = gCruiseCtrl->LineTracerYawrate((2*mLinevalue), Max_Yawrate, Min_Yawrate);
		  Track_Mode = Go_LUG;
		  ref_odo = mOdo + 500;
		}
	
		break;
#if 0
		
	  case Go_Step:
		StepRunner(mLinevalue, mOdo, mYawangle, mDansa);
		ref_odo = mOdo;
		break;
	  case Go_LUG:
		  forward =  20;
		  Max_Yawrate = 1.5;
		  Min_Yawrate = -1.5;
		  yawratecmd = gCruiseCtrl->LineTracerYawrate((2*mLinevalue), Max_Yawrate, Min_Yawrate);
	
		if(mOdo >= ref_odo){
		  LookUpGateRunner(mLinevalue_LUG+30, mOdo, mYawangle,mLinevalue+30);
		}
		break;
	
#endif	
		
	  case Approach_to_Garage:
	
		ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
		//    y_t = -0.5*((FIVE_PAI+RAD_1_DEG) - mYawangle);
		y_t = -0.5*((FIVE_PAI) - mYawangle);
	
		yawratecmd = y_t;
	
		gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
		forward = 0.1*(gStep->CalcPIDContrInput(ref_odo, mOdo));
	
		Max_Yawrate = 1.0;
		Min_Yawrate = -1.0;

		anglecommand = TAIL_ANGLE_RUN;
		Track_Mode = Garage_In;
	
		break;
	
	  case Return_to_Line_Garage:
		yawratecmd = 0.0;
		if(mGarage == false){
		  if(mSonar > SONAR_DIST){
			mGarage = true;
			ref_odo = mOdo - GARAGE_LIT_DIST;
		  }
			speedcal = -20;
		}else{
		  if(mOdo <= ref_odo){
			Track_Mode = Garage_In;
		  }
		  speedcal =  -10;
		}
		forward =  speedcal;
		Max_Yawrate = 1.0;
		Min_Yawrate = -1.0;
		break;
	
	  case Garage_In:
#if 0
		StopRobo();
#endif
	  break;
	  default:
		forward = 0;
		Max_Yawrate = 1.0;
		Min_Yawrate = -1.0;
		yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, Max_Yawrate, Min_Yawrate);
		anglecommand = TAIL_ANGLE_RUN; //0817 tada
		tail_mode_lflag = true;
		break;
	  }
	}
	