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
  gDifficultCtrl = new DifficultCtrl(gCruiseCtrl);
	gClock       = new Clock();
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

	delete gCruiseCtrl;
	delete gDifficultCtrl;

	delete gBalancer;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Controller::ControllerInit(){
	Track_Mode = Start_to_1st_Corner;
  	mYaw_angle_offset = 0.0;
	gCruiseCtrl->init();  //
	gDifficultCtrl->init();
}

//*****************************************************************************
// 関数名 : ControllerOperation
// 引数 : 
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Controller::ControllerOperation(){
	mLinevalue         = gColorParts->linevalue;
	mLinevalue_LUG     = gColorParts->linevalue_LUG;   //ライン検出値
	mXvalue            = gMotorParts->xvalue;
	mYvalue            = gMotorParts->yvalue;
	mOdo               = gMotorParts->odo;
	mSpeed             = gMotorParts->velocity;
	mYawrate           = gMotorParts->yawrate;
	//mYawangle          = abs_angle + mYaw_angle_offset; 
	mYawangle          = gMotorParts->abs_angle;
	mTail_angle        = gMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO);
	mRobo_stop         = gMotorParts->robo_stop;
	mRobo_forward      = gMotorParts->robo_forward;
	mRobo_back         = gMotorParts->robo_back;
	mRobo_turn_left    = gMotorParts->robo_turn_left;
	mRobo_turn_right   = gMotorParts->robo_turn_right;
	mDansa             = gGyroParts->dansa;
	mDet_gray          = gColorParts->det_gray;
#ifdef RIGHT_MODE
	mSonar             = gSonarParts->sonarDistance;
#else
  	mSonar			   = 0;
#endif
	mRobo_balance_mode = gCruiseCtrl->balance_mode;

	Track_run();
	gCruiseCtrl->setCommand(forward,
		yawratecmd,
		anglecommand,
		gMotorParts->yawrate,
		tail_mode_lflag);//指令値渡す

}

//*****************************************************************************
// 関数名 : ControllerOperation
// 引数 : 
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Controller::run(){
	gCruiseCtrl->CruiseCtrlOperation();
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
  	static bool    line_det;
#ifdef RIGHT_MODE
#else
	static float   ref_angle;
#endif
	static int32_t clock_start;
	int dammy_line_value;
	int speedcal = 0;
	float y_t;
	

	switch(Track_Mode){

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
#ifdef RIGHT_MODE
			if(mYawangle > 0.5){
#else
			if(mYawangle > 2){
#endif
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
#ifdef RIGHT_MODE
				Track_Mode = Return_to_Line;
#else
				Track_Mode = Dead_Zone;
				ref_odo = mOdo + DEAD_ZONE_LENGTH;
#endif
			}
			break;

		case Get_Ref_Odo:
			forward =  50;
			dammy_line_value = 50 - 300*mYawangle;
			if(dammy_line_value > 100){
				dammy_line_value = 100;
			}else if(dammy_line_value < 0){
				dammy_line_value = 0;
			}
			Max_Yawrate = 5.0;
			Min_Yawrate = -5.0;
			yawratecmd = gCruiseCtrl->LineTracerYawrate(dammy_line_value, Max_Yawrate, Min_Yawrate);
			ref_odo = mOdo + DEAD_ZONE_LENGTH;
			Track_Mode = Dead_Zone;
			
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
#ifdef RIGHT_MODE
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
#else
			forward =  20; // 0910 tada
			Max_Yawrate = 5.0;
			Min_Yawrate = -5.0;
			if(mYawangle < 0.16 && mLinevalue <20){
				dammy_line_value = 80 - 300*mYawangle;
				if(dammy_line_value > 100){
					dammy_line_value = 100;
				}else if(dammy_line_value < 0){
					dammy_line_value = 0;
				}
				yawratecmd = gCruiseCtrl->LineTracerYawrate(dammy_line_value, Max_Yawrate, Min_Yawrate);
			}
			else{
				yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, Max_Yawrate, Min_Yawrate);
			}
			anglecommand = TAIL_ANGLE_RUN; //0817 tada
			tail_mode_lflag = false;

			if((mYawangle > 1) &&(mRobo_forward == 1)){
				Track_Mode = Go_Step;
			}
#endif
			break;

		case Go_Step:
			if(gDifficultCtrl->StepRunner(mLinevalue, 
																		mOdo, 
																		mYawangle, 
																		mDansa,
																		mRobo_balance_mode,
																		forward,
																		anglecommand,
																		yawratecmd,
																		tail_mode_lflag,
																		ref_x,
																		mXvalue)){
			
				Track_Mode = Approach_to_Garage;
			}
			ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
			line_det = false;
			break;
		case Go_LUG:
			forward =  20;
			Max_Yawrate = 1.5;
			Min_Yawrate = -1.5;
			yawratecmd = gCruiseCtrl->LineTracerYawrate((2*mLinevalue), Max_Yawrate, Min_Yawrate);

			if(mOdo >= ref_odo){
				if(gDifficultCtrl->LookUpGateRunner(mLinevalue_LUG+30,
																						mOdo, 
																						mYawangle,
																						mLinevalue+30,
																						mRobo_balance_mode,
																						forward,
																						anglecommand,
																						yawratecmd,
																						tail_mode_lflag)){
					Track_Mode = Return_to_Line_Garage;
				}
			}
			break;
		
		case Approach_to_Garage:
			ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
			ref_x   = ref_x + GARAGE_X_POS;
			line_det = false;

			y_t = -0.5*((FIVE_PAI) - mYawangle);
			yawratecmd = y_t;

			gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
			forward = 0.1*(gStep->CalcPIDContrInput(ref_odo, mOdo));
			
			anglecommand = TAIL_ANGLE_RUN;
			Track_Mode = Go_to_Garage;
			break;

		case Go_to_Garage:
			if(mLinevalue > 50){
				line_det = true;
				}
	
	
			if(line_det == false){
				y_t = -0.5*((FIVE_PAI+RAD_5_DEG) - mYawangle);
				yawratecmd = y_t;
			}else{
				Max_Yawrate = 5;
				Min_Yawrate = -5;
				yawratecmd = gCruiseCtrl->LineTracerYawrate((CL_SNSR_GAIN_GRAY * mLinevalue), Max_Yawrate, Min_Yawrate);
			}
			//    forward = 0.3*(gStep->CalcPIDContrInput(ref_odo, mOdo));
			forward = 0.7*(gStep->CalcPIDContrInput(ref_odo, mOdo));
			anglecommand = TAIL_ANGLE_RUN;
	
			if((mOdo >= ref_odo)||(mXvalue > ref_x)){
				Track_Mode = Garage_Tail_On;
			}
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
#ifdef RIGHT_MODE
			if(gDifficultCtrl->StopRobo(
				forward,
				anglecommand,
				yawratecmd,
				tail_mode_lflag)){

			}
#else
			tail_mode_lflag = true;
			if(mYawangle >= ref_angle){
				forward = 0;
				yawratecmd = 0;
				clock_start = gClock->now();
				ref_odo = mOdo - GARAGE_LENGTH;
				Track_Mode = Garage_Stop;      
			}else{
				forward = 0;
				y_t = -1.0;
				yawratecmd = y_t;
			}
#endif
			break;

		case Garage_Stop:
			tail_mode_lflag = true;
			if(gClock->now() - clock_start > 500){
				forward    = -10;
				yawratecmd = 0;
				if(mOdo < ref_odo){
					forward    = 0;
					yawratecmd = 0;
				}
			}else{
				forward    = 0;
				yawratecmd = 0;
			}
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
	