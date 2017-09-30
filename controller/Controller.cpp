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
	gStepRun = new StepRun(gCruiseCtrl);
	gLookUpGate = new LookUpGate(gCruiseCtrl);
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
	delete gStepRun;
	delete gLookUpGate;

	delete gBalancer;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Controller::ControllerInit(){
	Track_Mode = Start_to_1st_Straight;
#if 0
Track_Mode = Go_LUG;
#endif
  	mYaw_angle_offset = 0.0;
    left_line_edge    = true;
	gCruiseCtrl->init();
	gStepRun->init();
	gLookUpGate->init();
	
	tail_stand_mode   = false;
	tail_lug_mode     = false;
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
	mSonar_dis         = gSonarParts->sonarDistance;
#else
	mSonar_dis		   = 0;
#endif
	mRobo_balance_mode = gCruiseCtrl->balance_mode;
	mRobo_lug_mode = gCruiseCtrl->lug_mode;

	Track_run();
	gCruiseCtrl->setCommand(forward,
		yawratecmd,
		anglecommand,
		gMotorParts->yawrate,
		tail_stand_mode,
		tail_lug_mode);//指令値渡す

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
	static float   ref_odo;
  	static bool    line_det;
	static int32_t clock_start;
	int dammy_line_value;
#ifdef RIGHT_MODE
#else
	static float   ref_angle;
#endif
	float y_t;
	

	switch(Track_Mode){
		case Start_to_1st_Straight:
			forward =  100;
			yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, 1.5, -1.5);
			anglecommand = TAIL_ANGLE_RUN; //0817 tada
			tail_stand_mode = false;
			
			if(mYawangle < CORNER_CHECK[0]){
				Track_Mode = Start_to_1st_Corner;
			}
			break;
		case Start_to_1st_Corner:
			forward =  85;
			yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, 1.8, -1.8);
			anglecommand = TAIL_ANGLE_RUN; //0817 tada
			tail_stand_mode = false;
		
			if(mYawangle < CORNER_CHECK[1]){
				Track_Mode = Snd_Corner;
			}
			break;

		case Snd_Corner:
			forward =  85;
			yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, 1.8, -1.8);
			anglecommand = TAIL_ANGLE_RUN; //0817 tada
			tail_stand_mode = false;
			if(mYawangle > CORNER_CHECK[2]){
				Track_Mode = Final_Corner;
			}
		break;

		case Final_Corner:
			forward =  85;
			yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, 1.8, -1.8);
			anglecommand = TAIL_ANGLE_RUN; //0817 tada
			tail_stand_mode = false;

			if((mYawangle < CORNER_CHECK[3]) &&(mRobo_forward == 1)){
				Track_Mode = Final_Straight;
				ref_odo = mOdo +  FINAL_STRAIGHT_LENGTH;
			}
		break;

		case Final_Straight:
			forward =  100;
			yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, 1.0, -1.0);
			anglecommand = TAIL_ANGLE_RUN; //0817 tada
			tail_stand_mode = false;
			if(mOdo > ref_odo){
				Track_Mode = Dead_Zone;
				ref_odo = mOdo + DEAD_ZONE_LENGTH;
			}
			break;

		case Get_Ref_Odo:
			forward =  50;
			dammy_line_value = 50 - 300*(mYawangle-0.1);
			if(dammy_line_value > 100){
				dammy_line_value = 100;
			}else if(dammy_line_value < 0){
				dammy_line_value = 0;
			}
			yawratecmd = gCruiseCtrl->LineTracerYawrate(dammy_line_value, 5.0, -5.0);
			ref_odo = mOdo + DEAD_ZONE_LENGTH;
			Track_Mode = Dead_Zone;
			
			break;
	
		case Dead_Zone:
			forward =  50;
			dammy_line_value = 50 - 300*(mYawangle-0.1);
			if(dammy_line_value > 100){
				dammy_line_value = 100;
			}else if(dammy_line_value < 0){
				dammy_line_value = 0;
			}

			yawratecmd = gCruiseCtrl->LineTracerYawrate(dammy_line_value, 2.0, -2.0);

			if(mOdo > ref_odo){
				Track_Mode = Return_to_Line;
			}
			break;

		case Return_to_Line:
			forward =  20; // 0910 tada
			if(mYawangle < 0.16 && mLinevalue <20){
				dammy_line_value = 50 - 300*(mYawangle-0.1);
				if(dammy_line_value > 100){
					dammy_line_value = 100;
				}else if(dammy_line_value < 0){
					dammy_line_value = 0;
				}
				yawratecmd = gCruiseCtrl->LineTracerYawrate(dammy_line_value, 5.0, -5.0);
			}
			else{
				yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, 5.0, -5.0);
			}
			anglecommand = TAIL_ANGLE_RUN;
			tail_stand_mode = false;
		
#ifdef RIGHT_MODE
			if((mYawangle > CORNER_CHECK[4]) &&(mRobo_forward == 1)){
			  forward =  30;
			  yawratecmd = gCruiseCtrl->LineTracerYawrate((2*mLinevalue), 5.0, -5.0);
			  Track_Mode = Go_LUG;
			  ref_odo    = mOdo + APPROACH_TO_LUG_LENGTH;
			}
#else
			if((mYawangle > CORNER_CHECK[4]) &&(mRobo_forward == 1)){
				Track_Mode = Go_Step;
			}
#endif
			break;

		case Go_Step:
			if(gStepRun->StepRunner(mLinevalue, 
									mOdo, 
									mYawangle, 
									mDansa,
									mRobo_balance_mode,
									forward,
									yawratecmd,
									anglecommand,
									tail_stand_mode,
									ref_x,
									mXvalue)){

				Track_Mode = Approach_to_Garage;
			}
			ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
			line_det = false;
			break;
		case Go_LUG:
			forward =  30;
			yawratecmd = gCruiseCtrl->LineTracerYawrate((2*mLinevalue), 1.5, -1.5);

			if(gLookUpGate->LookUpGateRunner(mLinevalue_LUG,
											mOdo, 
											mYawangle,
											mLinevalue,
											mRobo_balance_mode,
											forward,
											yawratecmd,
											anglecommand,
											tail_stand_mode,
											tail_lug_mode,
											mRobo_lug_mode,
											mSonar_dis)){
				Track_Mode = Approach_to_Garage;//Return_to_Line_Garage;
			}
			break;
		
		case Approach_to_Garage:
#ifdef RIGHT_MODE
			tail_stand_mode = true;
			ref_odo = mOdo + LUG_TO_GARAGE_LENGTH;
			y_t = -0.5*(PAI - mYawangle);
			yawratecmd = y_t;
		
			gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
			forward = 0.1*(gStep->CalcPIDContrInput(ref_odo, mOdo));
			Track_Mode = Garage_In;
#else
			ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
			ref_x          = ref_x - GARAGE_X_POS;
			line_det = false;
			left_line_edge = false;

			y_t = -0.5*((FIVE_PAI) - mYawangle);
			yawratecmd = y_t;

			gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
			forward = 0.1*(gStep->CalcPIDContrInput(ref_odo, mOdo));
			
			clock_start = gClock->now();
			anglecommand = TAIL_ANGLE_RUN;
			Track_Mode = Go_to_Garage;
#endif
			break;

		case Go_to_Garage:

			if(line_det == false){
				y_t = -0.5*((FIVE_PAI+RAD_5_DEG) - mYawangle);
				yawratecmd = y_t;
			}else{
				yawratecmd = gCruiseCtrl->LineTracerYawrate((CL_SNSR_GAIN_GRAY * mLinevalue), 5.0, -5.0);
			}
			//    forward = 0.3*(gStep->CalcPIDContrInput(ref_odo, mOdo));
			forward = 0.7*(gStep->CalcPIDContrInput(ref_odo, mOdo));
			anglecommand = TAIL_ANGLE_RUN;

			if((mOdo >= ref_odo)||(mXvalue < ref_x)){ //Honban Yo
			//    if((mOdo >= ref_odo)||(mYvalue < ref_x)){ //Debug yo
			Track_Mode = Garage_Tail_On;
			}
			break;
		case Return_to_Line_Garage:
#ifdef RIGHT_MODE
			forward =  50;
			yawratecmd = gCruiseCtrl->LineTracerYawrate((4 * mLinevalue), 5.0, -5.0);
			anglecommand = TAIL_ANGLE_RUN; //0817 tada
			tail_stand_mode = false;
#else
#endif
			break;
		case Garage_Tail_On:
			tail_stand_mode = true;
			forward    = 0;
			yawratecmd = 0;
			if(mRobo_balance_mode == false){
				forward    = 0;
				yawratecmd = 0;
				clock_start = gClock->now();
#ifdef RIGHT_MODE
#else
				ref_angle = mYawangle + PAI + RAD_15_DEG;
#endif
				Track_Mode = Garage_In;
			}
		
			break;
		case Garage_In:
#ifdef RIGHT_MODE
			tail_stand_mode = true;
			y_t = -0.5*(PAI - mYawangle);
			yawratecmd = y_t;
			forward = 0.3*(gStep->CalcPIDContrInput(ref_odo, mOdo));
#else
			tail_stand_mode = true;
			if(mYawangle >= ref_angle){
				forward = 0;
				yawratecmd = 0;
				clock_start = gClock->now();
				ref_odo = mOdo - GARAGE_LENGTH;
				//Track_Mode = Garage_Stop;      
			}else{
				forward = 0;
				y_t = -1.0;
				yawratecmd = y_t;
			}
#endif
			break;

		case Garage_Stop:
			tail_stand_mode = true;
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
			yawratecmd = gCruiseCtrl->LineTracerYawrate(mLinevalue, 1.0, -1.0);
			anglecommand = TAIL_ANGLE_RUN; //0817 tada
			tail_stand_mode = true;
			break;
	}
}
	