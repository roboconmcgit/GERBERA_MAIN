#include "Command_Calc.h"
#include "ev3api.h"
#include "Clock.h"
#include "math.h"


using ev3api::Clock;

#define liting_radius 10; // liting spot radius [mm]
//#define STEP_DEBUG
//#define GARAGE_DEBUG
//#define LUG_DEBUG
//#define GARAGE_DEBUG

Clock*       gClock;

CommandCalc::CommandCalc(){

}

void CommandCalc::init( ){
  gClock       = new Clock();
  Track_Mode = Start_to_1st_Corner;
  Step_Mode  = Step_Start;
  LUG_Mode = LUG_Start;

#ifdef LUG_DEBUG
  Track_Mode = Go_LUG;
#endif

#ifdef STEP_DEBUG
  Track_Mode = Return_to_Line;
  //  Track_Mode = Go_Step;
#endif

#ifdef GARAGE_DEBUG
  //Track_Mode = Return_to_Line;
  Track_Mode =  Track_Debug_00;
#endif


}

void CommandCalc::SetCurrentData(int   linevalue,
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
				 int32_t sonar,
				 bool  robo_balance_mode,
				 int   max_forward,
				 float max_yawrate,
				 float min_yawrate

				 ) {

    mLinevalue         = linevalue;
    mLinevalue_LUG     = linevalue_lug;   //ライン検出値
    mXvalue            = xvalue;
    mYvalue            = yvalue;
    mOdo               = odo;
    mSpeed             = speed;
    mYawrate           = yawrate;
    mYawangle          = yawangle;
    mTail_angle        = robo_tail_angle;
    mRobo_stop         = robo_stop;
    mRobo_forward      = robo_forward;
    mRobo_back         = robo_back;
    mRobo_turn_left    = robo_turn_left;
    mRobo_turn_right   = robo_turn_right;
    mDansa             = dansa;
    mDet_gray          = det_gray;
    mSonar             = sonar;
    mRobo_balance_mode = robo_balance_mode;

    mMax_Forward = max_forward;
    mMax_Yawrate = max_yawrate;
    mMin_Yawrate = min_yawrate;

}

void CommandCalc::Track_run() {

  static float   ref_odo;
  static int32_t clock_start;

  int dammy_line_value;
  int speedcal = 0;



  switch(Track_Mode){

  case Start_to_1st_Straight:
    /*
    gForward->init_pid(0.3,0.005,0.05,dT_4ms);
    forward = 50;
    mMax_Forward = 100;
    mMax_Yawrate = 5;
    mMin_Yawrate = -5;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;
    Track_Mode = Start_to_1st_Straight;
    */
    break;

  case Start_to_1st_Corner:

    //    forward = gForward->calc_pid(mMax_Forward, forward);
    forward =  100;//mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if(mYawangle < -2){
      Track_Mode = Snd_Corner;
    }
    break;

  case Snd_Corner:
  forward =  50;//mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if(mYawangle > 0.5){
      Track_Mode = Final_Corner;
    }
    break;

  case Final_Corner:
  forward =  50;//mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if((mYawangle < 1.0) &&(mRobo_forward == 1)){
      Track_Mode = Final_Straight;
      ref_odo = mOdo +  FINAL_STRAIGHT_LENGTH;
    }
    break;

  case Final_Straight:
    //forward =  100;//mMax_Forward;
    forward =  50;//mMax_Forward;
    mMax_Yawrate = 1.0;
    mMin_Yawrate = -1.0;
    LineTracerYawrate(mLinevalue);
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
    dammy_line_value = 50 - 300*mYawangle;
    if(dammy_line_value > 100){
      dammy_line_value = 100;
    }else if(dammy_line_value < 0){
      dammy_line_value = 0;
    }

    LineTracerYawrate(dammy_line_value);

    if(mOdo > ref_odo){
      Track_Mode = Return_to_Line;
    }
    break;

  case Return_to_Line:
    forward =  50;
    LineTracerYawrate((2*mLinevalue));
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if((mYawangle <  MINUS_RAD_5_DEG)&&(yawratecmd > 0) ){
      yawratecmd = 0.0;
    }


    if((mYawangle > 2.5) &&(mRobo_forward == 1)){
      forward =  30;
      LineTracerYawrate((2*mLinevalue));
      Track_Mode = Go_LUG;
      ref_odo = mOdo + 500;
    }

    break;

  case Go_Step:
    StepRunner(mLinevalue, mOdo, mYawangle, mDansa);
    ref_odo = mOdo;
    break;

  case Go_LUG:
      forward =  20;
      LineTracerYawrate((2*mLinevalue));

    if(mOdo >= ref_odo){
      LookUpGateRunner(mLinevalue_LUG+30, mOdo, mYawangle,mLinevalue+30);
    }

#ifdef LUG_DEBUG

    LookUpGateRunner(mLinevalue_LUG+30, mOdo, mYawangle,mLinevalue+30);

#endif

#ifdef GARAGE_DEBUG

    switch(LUG_Mode){

    case END:
        GarageRunner(mLinevalue_LUG+30, mOdo, mYawangle,mLinevalue+30);

      break;

    default:

    LookUpGateRunner(mLinevalue_LUG+30, mOdo, mYawangle,mLinevalue+30);

    break;

    }

#endif

    break;


  case Approach_to_Garage:

    ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
    //    y_t = -0.5*((FIVE_PAI+RAD_1_DEG) - mYawangle);
    y_t = -0.5*((FIVE_PAI) - mYawangle);

    yawratecmd = y_t;

    gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    forward = 0.1*(gStep->CalcPIDContrInput(ref_odo, mOdo));

    anglecommand = TAIL_ANGLE_RUN;
    Track_Mode = Garage_In;

    /*
    if(mLinevalue > 70){
      forward    = 0;
      yawratecmd = 0;
      Track_Mode = Return_to_Line_Garage;
    }
    else if (mOdo - ref_odo < 200){
      forward    = 10;
      y_t = -0.6*(5.0*PAI+RAD_30_DEG - mYawangle);

#ifdef GARAGE_DEBUG
      y_t = -0.6*(RAD_30_DEG - mYawangle);
#endif
      yawratecmd = y_t;
    }else{
      forward    = 10;
      y_t = -0.5*(5.0*PAI+MINUS_RAD_30_DEG - mYawangle);

#ifdef GARAGE_DEBUG
      y_t = -0.5*(MINUS_RAD_30_DEG - mYawangle);
#endif

      yawratecmd = y_t;
    }
    */

    break;

  case Return_to_Line_Garage:
    yawratecmd = 0.0;
    //anglecommand = TAIL_ANGLE_GARAGE;
    //tail_mode_lflag = true;

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
    break;

  case Garage_In:
    StopRobo();
  break;
  case Track_Debug_00:
    ref_odo = 0;
    //    Track_Mode = Approach_to_Garage;

    //0904
    anglecommand = TAIL_ANGLE_RUN;
    tail_mode_lflag = false;
    clock_start = gClock->now();
    Track_Mode = Track_Debug_01;
    break;

  case Track_Debug_01:

    if(gClock->now() - clock_start > 5000){
      tail_mode_lflag = true;
    }
    if(mRobo_balance_mode == false){
      clock_start = gClock->now();
      Track_Mode = Track_Debug_02;
    }
    break;

  case Track_Debug_02:
    anglecommand = TAIL_ANGLE_RUN;
    if(gClock->now() - clock_start > 3000){
      tail_mode_lflag = false;
    }
    if(mRobo_balance_mode == true){
      Track_Mode = Track_Debug_00;
    }
    break;

  default:
    forward = 0;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = true;
    break;
  }
}

void CommandCalc::StrategyCalcRun(int strategy_num, int virtualgate_num) {

  Strategy=static_cast<enumStrategy>(strategy_num);

	switch(Strategy){
	case StartDash:
		StartDashRunner();
	break;

	case LineTrace1:
	  forward = mMax_Forward;
	  LineTracerYawrate(mLinevalue);
	  anglecommand = TAIL_ANGLE_RUN; //0817 tada
	  tail_mode_lflag = false;

	break;

	case MapTrace:
		//VirtualGateDet();
		MapTracer(virtualgate_num);
	break;

	case Goal:

	break;

	case Goal2Step:

	break;

	case Step:
	  gForward->init_pid(0.1,0.005,0.05,dT_4ms);
	  StepRunner(mLinevalue, mOdo, mYawangle, mDansa);
	break;

	case LookUpGate:
	  //		LookUpGateRunner();
	break;

	case Garage:
//		GarageRunner();
	break;

	case Stop:
		StopRobo();
	break;

	default:

	break;
	}

}

void CommandCalc::StartDashRunner(){

}
//17.07.31 k-tomii add for position estimation
//ライントレースプログラム
float CommandCalc::LineTracer(int line_value,float traceforward) {

	const int LineTraceCommand=80;			//目標ライン値
	const float KP=0.5,KI=0.005,KD=0.05;			//PIDゲインの設定

	static int error_old=0,error_P_old=0;		//過去の偏差
	static float u=0;							//制御入力

	float u_delta=0;							//制御入力の差分
	int error=0,error_P=0,error_I=0,error_D=0;	//偏差
	float u_P_delta=0,u_I_delta=0,u_D_delta=0;	//制御入力の差分

	error=LineTraceCommand-(line_value);			//制御偏差を計算
	error_P=error-error_old;					//P制御用の偏差を計算
	error_I=error;								//I制御用の偏差を計算
	error_D=error_P-error_P_old;				//D制御用の偏差を計算

	u_P_delta=KP*error_P;						//P制御用の入力差分を計算
	u_I_delta=KI*error_I;						//I制御用の入力差分を計算
	u_D_delta=KD*error_D;						//D制御用の入力差分を計算

	u_delta=u_P_delta+u_I_delta+u_D_delta;		//PID制御入力の差分を計算
	u=u+u_delta;								//制御入力を計算

	//入力制限
	if(u>10){
		u=10;
	}else if(u<-10){
		u=-10;
	}

	return u;										//目標turn値を更新
	forward=traceforward;						//目標forward値を更新

	error_old=error;							//過去の偏差を保存
	error_P_old=error_P;						//過去の偏差を保存

}

//2017/08/06多田さんライントレーサー
void CommandCalc::LineTracerYawrate(int line_value) {

    y_t = -1.0*(((float)line_value-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
    if(y_t > 10.0) y_t = 10.0;
    if(y_t < -10.0) y_t = -10.0;
	y_t = y_t + 7.0*(y_t/8.0)*(y_t/8.0)*(y_t/8.0);
//    yawratecmd = y_t/4.0;
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));

    if(yawratecmd > mMax_Yawrate){
      yawratecmd =  mMax_Yawrate;

    }else if (yawratecmd < mMin_Yawrate){
      yawratecmd = mMin_Yawrate;
    }else{
      yawratecmd = yawratecmd;
    }

    y_t_prev = y_t;

}

void CommandCalc::MapTracer(int virtualgate_num) {

	VirtualGate=static_cast<enumVirtualGate>(virtualgate_num);

	switch(VirtualGate){
	case Gate12:

	break;

	case Gate23:

	break;

	case Gate34:

	break;

	case Gate45:

	break;

	case Gate56:

	break;

	case Gate67:

	break;

	case Gate78:

	break;

	case Gate89:

	break;

	default:

	break;
	}

}

void CommandCalc::StepRunner(int line_value, float odo, float angle, bool dansa){
  /*前提条件：ロボットがライン上にあること*/

  float y_t;
  static float angle_change_right_edge_trace;
  static float target_odo;
  static float target_angle;
  static float target_tail_angle;
  static int32_t clock_start;

  switch(Step_Mode){

  case Step_Start:
    forward =  50;
    LineTracerYawrate((2*line_value));
    target_odo = odo + 500;


    clock_start = gClock->now();
    anglecommand = TAIL_ANGLE_RUN; //0817 tada

    Step_Mode = Approach_to_Step;

#ifdef STEP_DEBUG
    //        Step_Mode =  First_Dansa;
    //        target_odo = odo + 250; //for debug
    //        clock_start = gClock->now();
#endif

    break;

  case Approach_to_Step:

    if(odo > target_odo){
      forward =  20;
    }else{
      forward =  50;
    }

    LineTracerYawrate((2*line_value));
    if((angle >  RAD_90_DEG)&&(yawratecmd < 0) ){
      yawratecmd = 0.0;
    }
    if(dansa){
      Step_Mode = First_Dansa;
      target_odo = odo + FST_DANSA_POS;
    }


    break;

  case First_Dansa:
    if(odo > target_odo){
      forward = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = First_Dansa_On;
      gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    }else{
      forward    = 20;
      yawratecmd = 0;
    }
    break;

  case First_Dansa_On:

    forward = gStep->CalcPIDContrInput(target_odo, odo);
    forward = forward * 0.1;
    yawratecmd = 0;
    anglecommand = target_tail_angle;


    if((gClock->now() - clock_start) > 5000){
      Step_Mode = First_Dansa_Tail_On;
      clock_start = gClock->now();
    }

  break;

  case First_Dansa_Tail_On:

    tail_mode_lflag = true;
    forward    = 0;
    yawratecmd = 0;
    if(mRobo_balance_mode == false){
      forward    = 0;
      yawratecmd = 0;
      Step_Mode = First_Turn;
      clock_start = gClock->now();
      target_angle = angle + RAD_360_DEG + RAD_15_DEG;
    }

#ifdef STEP_DEBUG

#endif

    break;

  case First_Turn:
    tail_mode_lflag = true;
    if(angle >= target_angle){
      //      Step_Mode = First_Dansa_Stand_Up;
      Step_Mode = First_Pre_Stand_Up;
      clock_start = gClock->now();
      forward = 0;
      yawratecmd = 0;
    }else{
      forward = 0;
      y_t = -2.0;
      yawratecmd = y_t;
    }						//目標yawrate値を更新
    break;

  case First_Pre_Stand_Up:

    forward = 10;
    y_t = -0.5*(2.5*PAI - angle);
    yawratecmd = y_t;
    tail_mode_lflag = true;

    if((gClock->now() - clock_start) > 1500){
      forward    = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
      clock_start = gClock->now();
      Step_Mode = First_Dansa_Stand_Up;
    }

    break;

  case First_Dansa_Stand_Up:

    if((gClock->now() - clock_start) > 3000){

      forward    = 0;
      yawratecmd = 0;

      anglecommand = TAIL_ANGLE_RUN;
      tail_mode_lflag = false;
      if(mRobo_balance_mode == true){
	forward    = 0;
	yawratecmd = 0;
	anglecommand = TAIL_ANGLE_RUN;
	Step_Mode = Approach_to_2nd_Step;
	clock_start = gClock->now();
      }
    }else{
      tail_mode_lflag = true;
      forward    = 0;
      yawratecmd = 0;
    }
#ifdef STEP_DEBUG

#endif
    break;

  case Approach_to_2nd_Step:


    if((gClock->now() - clock_start) < 5000){
      forward    = 0;
      yawratecmd = 0;
      anglecommand = TAIL_ANGLE_RUN;
    }else{
      forward = 10;

      y_t = -0.5*(2.5*PAI - angle);
#ifdef STEP_DEBUG
      //      y_t = -0.5*(2.0*PAI - angle);
#endif
      yawratecmd = y_t;


      if(dansa){
	forward    = 10;
	yawratecmd = 0;
	Step_Mode = Second_Dansa;
	target_odo = odo + SCD_DANSA_POS;
      }
    }

    break;

  case Second_Dansa:
   if(odo > target_odo){
      forward    = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = Second_Dansa_On;
    }else{
      forward    = 15;
      yawratecmd = 0;
    }
    break;


  case Second_Dansa_On:
    gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    forward = gStep->CalcPIDContrInput(target_odo, odo);
    forward = forward * 0.1;
    yawratecmd = 0;
    anglecommand = target_tail_angle;

    /*    if(target_tail_angle < TAIL_ANGLE_DANSA){
      target_tail_angle = target_tail_angle + 0.1;
      target_tail_angle = target_tail_angle + 0.2;

      }*/

    if((gClock->now() - clock_start) > 5000){
      Step_Mode = Second_Dansa_Tail_On;
      clock_start = gClock->now();
      forward    = 0;
      yawratecmd = 0;
    }

  break;

  case Second_Dansa_Tail_On:
    forward    = 0;
    yawratecmd = 0;
    tail_mode_lflag = true;
    if(mRobo_balance_mode == false){
      forward    = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
      Step_Mode = Second_Turn;
      clock_start = gClock->now();
      target_angle = angle + RAD_450_DEG + RAD_5_DEG;
    }
    break;

  case Second_Turn:
    tail_mode_lflag = true;
    if((gClock->now() - clock_start) > 1000){
      if(angle >= target_angle){
	//Step_Mode = Second_Dansa_Stand_Up;
	Step_Mode = Second_Pre_Stand_Up;
	target_odo = odo + 50;
	clock_start = gClock->now();
	forward    = 0;
	yawratecmd = 0;

      }else{
	forward = 0;
	y_t     = -2.0;
	yawratecmd = y_t;						//目標yawrate値を更新
      }						//目標yawrate値を更新
    }
    else{
      forward = 0;
      yawratecmd = 0;
    }
    break;

  case Second_Pre_Stand_Up:

    forward = 10;
    y_t = -0.5*(5*PAI - angle);
    yawratecmd = y_t;
    tail_mode_lflag = true;

    if((gClock->now() - clock_start) > 1500){
      forward    = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
      clock_start = gClock->now();
      Step_Mode = Second_Dansa_Stand_Up;
    }

    break;


  case Second_Dansa_Stand_Up:

    if((gClock->now() - clock_start) > 15000){
      forward    = 0;
      yawratecmd = 0;
      anglecommand = TAIL_ANGLE_RUN;
      Step_Mode = Approach_to_Exit;
      clock_start = gClock->now();
      tail_mode_lflag = false;
    }

    if((gClock->now() - clock_start) > 3000){
      forward    = 0;
      yawratecmd = 0;

      anglecommand = TAIL_ANGLE_RUN;
      tail_mode_lflag = false;

      if(mRobo_balance_mode == true){
	forward    = 0;
	yawratecmd = 0;
	anglecommand = TAIL_ANGLE_RUN;
	Step_Mode = Approach_to_Exit;
	clock_start = gClock->now();

#ifdef STEP_DEBUG
	//    Step_Mode =  First_Dansa;
	//    target_odo = odo + 250; //for debug
	//    clock_start = gClock->now();
#endif

      }
    }else{
      forward = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
    }


    break;

  case Approach_to_Exit:

    forward    = 20;

    y_t = -0.5*(5*PAI - angle);
#ifdef STEP_DEBUG
    //    y_t = -0.5*(4.5*PAI - angle);
#endif

    yawratecmd = y_t;


    if(dansa){
      forward    = 20;
      yawratecmd = 0;
      Track_Mode = Approach_to_Garage;
    }
    if(mOdo > target_odo){
      forward    = 20;
      yawratecmd = 0;
      Track_Mode = Approach_to_Garage;
    }


    break;

  case Change_Left_Edge_Trace:

    break;

  case End_of_Step:

    break;



  default:
    yawratecmd = 0;
    forward = 0;
    break;
  }

}


void CommandCalc::LookUpGateRunner(int line_value_lug, float odo, float angle,int line_value){

	float y_t;
	  static float odo_starting_point;
	  static float target_angle;
	  static int32_t clock_start;
	  static float ref_angle;


	  switch(LUG_Mode){

	  case LUG_Start:
		    clock_start = gClock->now();
		    LUG_Mode = Approach_to_LUG;

	    break;

	  case Approach_to_LUG:
		forward = 0;
		yawratecmd = 0;
		anglecommand = TAIL_ANGLE_RUN;

	    if((gClock->now() - clock_start) > 2000){
		    LUG_Mode = LUG_Tail_On;
		    anglecommand = TAIL_ANGLE_RUN+80;
		    clock_start = gClock->now();
	    }
	    //Step_Mode = First_Dansa;
	    break;

	  case LUG_Tail_On:
		    tail_mode_lflag = true;
		    forward    = 0;
		    yawratecmd = 0;
		    if(mRobo_balance_mode == false){
		      forward    = 0;
		      yawratecmd = 0;
		      LUG_Mode = LUG_Tailangle;
		      clock_start = gClock->now();
		    }
	    break;

	  case LUG_Tailangle:
		    forward = 0;
		    yawratecmd = 0;
			anglecommand = anglecommand -0.01;
		    if(anglecommand<TAIL_ANGLE_RUN+62.0){
		      LUG_Mode = LUG_Stop0;
		      odo_starting_point = odo;
		      clock_start = gClock->now();
		    }
	    break;


	  case LUG_Stop0:
		    forward = 0;
		    yawratecmd=0;

		    if((gClock->now() - clock_start) > 1000){
		      LUG_Mode = LUG_forward;
		      ref_angle=angle;
		    }

	    break;

	  case LUG_forward:
		    forward = 30;

			    yawratecmd = LineTracer(line_value_lug,forward);

		    if((odo-odo_starting_point)>800){
		      LUG_Mode = LUG_Stop1;
		      clock_start = gClock->now();
		    }

	    break;

	  case LUG_Stop1:
		    forward = 0;
		    yawratecmd=0;

		    if((gClock->now() - clock_start) > 1000){
		      LUG_Mode = LUG_Trun1;
		      clock_start = gClock->now();
		      target_angle = angle - 2.8;
		    }

	    break;

	  case LUG_Trun1:
		      if(angle < target_angle){
			clock_start = gClock->now();
			forward = 0;
			yawratecmd = 0;
		    LUG_Mode = LUG_Stop2;
		    odo_starting_point = odo;
		      }else{
			forward = 0;
			//      y_t = -0.5*(target_angle - angle);
			y_t = 10;
			yawratecmd = y_t;						//目標yawrate値を更新
		      }						//目標yawrate値を更新
	    break;

	  case LUG_Stop2:
		    forward = 0;
		    yawratecmd=0;

		    if((gClock->now() - clock_start) > 1000){
		      LUG_Mode = LUG_forward2;
		      clock_start = gClock->now();
		    }

	    break;

	  case LUG_forward2:
		    forward = 29;
		    /*
		    y_t = (((float)line_value_lug-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
		    if(y_t > 10.0) y_t = 10.0;
		    if(y_t < -10.0) y_t = -10.0;

		    yawratecmd = y_t;						//目標yawrate値を更新
		    */
		    if(ref_angle-angle>0.78){

			    yawratecmd = 2;

		    }else if(ref_angle-angle<-0.78){

		    	yawratecmd = -2;

		    }else{

			    yawratecmd = LineTracer(line_value_lug,forward);
		    }

		    if((odo-odo_starting_point)>800){
		      LUG_Mode = LUG_Stop3;
		      clock_start = gClock->now();
		    }

	    break;

	  case LUG_Stop3:
		    forward = 0;
		    yawratecmd=0;

		    if((gClock->now() - clock_start) > 1000){
		      LUG_Mode = LUG_Trun2;
		      clock_start = gClock->now();
		      target_angle = angle - 2.8;
		    }

	    break;

	  case LUG_Trun2:
		      if(angle < target_angle){
			clock_start = gClock->now();
			forward = 0;
			yawratecmd = 0;
		    LUG_Mode = LUG_Stop4;
		    odo_starting_point = odo;

		      }else{
			forward = 0;
			//      y_t = -0.5*(target_angle - angle);
			y_t = 10;
			yawratecmd = y_t;						//目標yawrate値を更新
		      }						//目標yawrate値を更新

	    break;

	  case LUG_Stop4:
		    forward = 0;
		    yawratecmd=0;

		    if((gClock->now() - clock_start) > 1000){
		      LUG_Mode = LUG_forward3;
		      clock_start = gClock->now();
		    }

	    break;

	  case LUG_forward3:
		    forward = 32;
		    /*
		    y_t = (((float)line_value_lug-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
		    if(y_t > 10.0) y_t = 10.0;
		    if(y_t < -10.0) y_t = -10.0;

		    yawratecmd = y_t;						//目標yawrate値を更新
		    */
		    if(ref_angle-angle>0.78){

			    yawratecmd = 2;

		    }else if(ref_angle-angle<-0.78){

		    	yawratecmd = -2;

		    }else{

			    yawratecmd = LineTracer(line_value_lug,forward);
		    }

		    if((odo-odo_starting_point)>800){
		      LUG_Mode = LUG_Stop5;
		      clock_start = gClock->now();
		    }

	    break;

	  case LUG_Stop5:
		    forward = 0;
		    yawratecmd=0;

		    if((gClock->now() - clock_start) > 1000){
		      clock_start = gClock->now();
		      LUG_Mode = LUG_Stand_Up;
		    }

	    break;

	  case LUG_Stand_Up:
		    forward = -9;
		    yawratecmd = 0;
			anglecommand = anglecommand +0.025;
		    if(anglecommand>TAIL_ANGLE_RUN+85){
				LUG_Mode = LUG_Stop6;
				clock_start = gClock->now();
			      target_angle = angle - 3.1;
		    }
	    break;

	  case LUG_Stop6:
		    forward = 0;
		    yawratecmd=0;

		    if((gClock->now() - clock_start) > 1000){
		      clock_start = gClock->now();
		      LUG_Mode = Garage_Turn;
		    }

	    break;

	  case Garage_Turn:
	      if(angle < target_angle){
		clock_start = gClock->now();
		forward = 0;
		yawratecmd = 0;
		LUG_Mode = Garage_Stop0;
	    odo_starting_point = odo;

	      }else{
		forward = 0;
		//      y_t = -0.5*(target_angle - angle);
		y_t = 10;
		yawratecmd = y_t;						//目標yawrate値を更新
	      }						//目標yawrate値を更新

	    break;

	  case Garage_Stop0:
		    forward = 0;
		    yawratecmd = 0;

		    if((gClock->now() - clock_start) > 1000){
          LUG_Mode = END;
		    }

	    break;

	  case END:
		    forward = 0;
		    yawratecmd = 0;
        Track_Mode = Return_to_Line_Garage;
	    break;

	  }

}



void CommandCalc::GarageRunner(int line_value_lug, float odo, float angle,int line_value){

}

void CommandCalc::StopRobo(){

    forward =  0;
    yawratecmd = 0.0;

    //anglecommand = TAIL_ANGLE_GARAGE;
    //tail_mode_lflag = true;
  }
