/**
 ******************************************************************************
 ** ファイル名 : StepRun.cpp
 **
 ** 概要 : StepRunクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

 #include "math.h"
 #include "StepRun.h"
 
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
 StepRun::StepRun(CruiseCtrl* Cruise)
 :gCruiseCtrl(Cruise)
 {
 
 }
 
 //*****************************************************************************
 // 関数名 : デストラクタ
 // 引数 : unused
 // 返り値 : なし
 // 概要 : 
 //*****************************************************************************
 StepRun::~StepRun(){
 
 }
 
 //*****************************************************************************
 // 関数名 : 
 // 引数 : unused
 // 返り値 : なし
 // 概要 : 
 //*****************************************************************************
 void StepRun::init(){
     gClock       = new Clock();
     Step_Mode  = Step_Start;
   }
   //*****************************************************************************
   // 関数名 : 
   // 引数 : unused
   // 返り値 : なし
   // 概要 : 
   //*****************************************************************************
 float StepRun::LineTracerYawrate(int line_value){
     return(gCruiseCtrl->LineTracerYawrate(line_value,-1.0,-1.0));
 }
 
 //*****************************************************************************
 // 関数名 : 
 // 引数 : unused
 // 返り値 : なし
 // 概要 : 
 //*****************************************************************************
 int StepRun::StepRunner(
     int line_value, 
     float odo, 
     float angle, 
     bool dansa, 
     bool mRobo_balance_mode,
     int &forward,
     float &yawratecmd,
     float &anglecommand,
     bool &tail_stand_mode,
     float &ref_x,
     float mXvalue
 ){
 /*前提条件：ロボットがライン上にあること*/
     int ret = 0;
     float y_t;
  static float ref_odo;
  static float ref_angle;
  static float ref_tail_angle;
     static int32_t clock_start;
     static int   dansa_cnt;
  static int stable_cnt;
     
     switch(Step_Mode){
   
     case Step_Start:
       dansa_cnt = 0;
       forward =  70;
       yawratecmd = LineTracerYawrate((CL_SNSR_GAIN_GRAY*line_value));
    ref_odo = odo + STEP_START_LENGTH;
    
   
       clock_start = gClock->now();
       anglecommand = TAIL_ANGLE_RUN;
       Step_Mode = Approach_to_Step;   
   
   #ifdef STEP_DEBUG
       //    Step_Mode   =  First_Dansa;   
       //    target_mOdo  = mOdo + 250; //for debug
       //    clock_start = gClock->now();
   #endif
   
       break;
   
     case Approach_to_Step:

     if(odo > ref_odo){
      //      forward    =  20;
      forward    =  40;

      y_t        = -0.5*( RAD_88p5_DEG - angle);
      yawratecmd = y_t;
    }else{
      forward =  70;
      //forward =  30;
      LineTracerYawrate((CL_SNSR_GAIN_GRAY * line_value));
    }

       //yawratecmd = LineTracerYawrate((CL_SNSR_GAIN_GRAY*line_value));
       //yawratecmd = LineTracerYawrate((line_value));
       if((angle >  (RAD_90_DEG))&&(yawratecmd < 0) ){
        yawratecmd = 0.0;
      }
   
       if(dansa){
         Step_Mode   = First_Dansa;
          ref_odo  = odo + FST_DANSA_POS;
         clock_start = gClock->now();
         dansa_cnt   = 0;
          stable_cnt = 0;
          gForward->init_pid(0.1,0.01,0.001,dT_4ms);
         ref_x       = mXvalue; //reference x pos for Garage
   
   #ifdef STEP_DEBUG
          ref_x       =mYvalue; //it is for debug which start from return to line mode
   #endif
       }
       break;
   
     case First_Dansa:
       if(dansa){
         dansa_cnt++;
       }
       
    if((odo > ref_odo - 25 )&&(odo < ref_odo + 25)){
      stable_cnt++;
       }
       
       if(dansa_cnt < 50){
      if(stable_cnt > STBL_CNT_1st_DANSA){ //3sec // it would be chanced to Stable flag
       forward = 0;
       yawratecmd = 0;
	ref_tail_angle =  TAIL_ANGLE_RUN;
       clock_start = gClock->now();
       Step_Mode = First_Dansa_Tail_On;
       dansa = 0;
	stable_cnt = 0;
         }else{
	forward    = gForward->calc_pid(ref_odo, odo);
       forward    = forward * 0.3;
       yawratecmd = 0;
       clock_start = gClock->now();
         }
       }else{
         forward = -10;
         yawratecmd = 0;
         if((gClock->now() - clock_start) > 3000){
       dansa_cnt = 0;
         }
       }
       break;
   
     case First_Dansa_On:
    forward = gForward->calc_pid(ref_odo, odo);
       forward = forward * 0.1;
       yawratecmd = 0;
    anglecommand = ref_tail_angle;
   
       if((gClock->now() - clock_start) > 3000){
         Step_Mode = First_Dansa_Tail_On;
         clock_start = gClock->now();
       }
     break;
   
     case First_Dansa_Tail_On:
    if(odo < ref_odo){
         forward    = 10;
         yawratecmd = 0;
       }else{
         tail_stand_mode = true;
         forward    = 0;
         yawratecmd = 0;
       }
   
       if(mRobo_balance_mode == false){
         forward    = 0;
         yawratecmd = 0;
         Step_Mode = First_Turn;
         clock_start = gClock->now();
      ref_angle = angle + RAD_360_DEG + RAD_15_DEG;
       }
   
       break;
       
       //not used 0914 kota
     case Fst_Turn_Pos_Adj:
    forward = gForward->calc_pid(ref_odo, odo);
       forward = forward * 0.1;
       yawratecmd = 0;
    anglecommand = ref_tail_angle;
       
       if((gClock->now() - clock_start) > 500){
         Step_Mode = First_Turn;
         forward = 0;
         yawratecmd = 0;
         clock_start = gClock->now();
       }
       break;
   
     case First_Turn:
       tail_stand_mode = true;
    if(angle >= ref_angle){
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
    if(odo < ref_odo){
         forward = 15;
         y_t = -0.5*(2.5*PAI - angle);
         yawratecmd = y_t;
         tail_stand_mode = true;
       }else{
         forward    = 0;
         yawratecmd = 0;
         tail_stand_mode = true;
         clock_start = gClock->now();
         Step_Mode = First_Dansa_Stand_Up;
       }
       
       break;
   
     case First_Dansa_Stand_Up:
   
       if((gClock->now() - clock_start) > 100){
         forward    = 0;
         yawratecmd = 0;
         anglecommand = TAIL_ANGLE_RUN;
         tail_stand_mode = false;
         if(mRobo_balance_mode == true){
       forward    = 0;
       yawratecmd = 0;
       anglecommand = TAIL_ANGLE_RUN;
       Step_Mode = Approach_to_2nd_Step;
       clock_start = gClock->now();
         }
       }else{
         tail_stand_mode = true;
         forward    = 0;
         yawratecmd = 0;
       }
       break;
   
     case Approach_to_2nd_Step:
    //    if((gClock->now() - clock_start) < 1000){
    if((gClock->now() - clock_start) < 100){
       
         forward      = 0;
         yawratecmd   = 0;
         anglecommand = TAIL_ANGLE_RUN;
       }else{
         forward    = 10;
         y_t        = -0.5*(2.5*PAI - angle);
         yawratecmd = y_t;
   
      if((dansa)&&(odo > ref_odo)){
       Step_Mode = Second_Dansa;
	ref_odo  = odo + SCD_DANSA_POS;
       clock_start = gClock->now();
       dansa_cnt   = 0;
	stable_cnt = 0;
	gForward->init_pid(0.1,0.01,0.001,dT_4ms);
   
         }
   
       }
       break;
       
       //not used 0914 ota
     case Pre_Second_Dansa:
       if((gClock->now() - clock_start) > 1000){
       forward    = 0;
       yawratecmd = 0;
       Step_Mode = Second_Dansa;
       }else{
         forward    = -10;
         yawratecmd = 0;
       }
       break;
       
     case Second_Dansa:
       if(dansa){
         dansa_cnt++;
       }
   
    if((odo > ref_odo - 25 )&&(odo < ref_odo + 25)){
      stable_cnt++;
       }
       
       if(dansa_cnt < 50){
      if(stable_cnt > STBL_CNT_2nd_DANSA){
       forward    = 0;
       yawratecmd = 0;
	ref_tail_angle =  TAIL_ANGLE_RUN;
       clock_start = gClock->now();
       Step_Mode   = Second_Dansa_On;
       dansa       = 0;
	stable_cnt  = 0;
	ref_odo  = ref_odo + SCD_DANSA_ON_POS;
	gForward->init_pid(0.1,0.01,0.001,dT_4ms);
         }else{
	forward     = gForward->calc_pid(ref_odo, odo);
	//	forward     = forward * 0.2;
	forward     = forward * 0.4;
	if(forward > STEP_CLIMB_MAX_SPEED){
	  forward     = STEP_CLIMB_MAX_SPEED;
	}
       yawratecmd = 0;
       clock_start = gClock->now();
         }
       }else{
         forward    = -10;
         yawratecmd = 0;
         if((gClock->now() - clock_start) > 3000){
       dansa_cnt = 0;
         }
       }
       break;
   
   
   
     case Second_Dansa_On:
    forward      = gForward->calc_pid(ref_odo, odo);
    forward      = forward * 0.2;
       yawratecmd   = 0;
    anglecommand = ref_tail_angle;
   
    if((odo > ref_odo - 25 )&&(odo < ref_odo + 25)){
      stable_cnt++;
       }
   
    if(stable_cnt > STBL_CNT_2nd_DANSA_ON){
         Step_Mode = Second_Dansa_Tail_On;
         clock_start = gClock->now();
         forward    = 0;
         yawratecmd = 0;
       }
     break;
   
     case Second_Dansa_Tail_On:
       forward    = 0;
       yawratecmd = 0;
       tail_stand_mode = true;
       if(mRobo_balance_mode == false){
         forward    = 0;
         yawratecmd = 0;
         tail_stand_mode = true;
         Step_Mode = Second_Turn;
         clock_start = gClock->now();
      ref_angle = angle + RAD_450_DEG + RAD_15_DEG;
       }
       break;
   
     case Second_Turn:
       tail_stand_mode = true;
       if((gClock->now() - clock_start) > 500){
      if(angle >= ref_angle){
       Step_Mode = Second_Pre_Stand_Up;
       clock_start = gClock->now();
       forward    = 0;
       yawratecmd = 0;
       
         }else{
       forward = 0;
       y_t     = -2.0;
	yawratecmd = y_t;
      }
       }
       else{
         forward = 0;
         yawratecmd = 0;
       }
       break;
   
     case Second_Pre_Stand_Up:
       
       forward = 10;
       y_t = -10.0*(5*PAI - angle);
       yawratecmd = y_t;
       tail_stand_mode = true;
   

       if((gClock->now() - clock_start) > 2500){
         forward    = 0;
         yawratecmd = 0;
         tail_stand_mode = true;
         clock_start = gClock->now();
         Step_Mode = Second_Dansa_Stand_Up;
      ref_odo         = odo + 150;
       }
       break;
   
   
     case Second_Dansa_Stand_Up:
   
    //    if((gClock->now() - clock_start) > 1000){
    if((gClock->now() - clock_start) > 500){
         forward    = 0;
         yawratecmd = 0;
         anglecommand = TAIL_ANGLE_RUN;
         tail_stand_mode = false;

         if(mRobo_balance_mode == true){
       forward    = 40;
       yawratecmd = 0;
       anglecommand = TAIL_ANGLE_RUN;
       Step_Mode = Approach_to_Exit;
       clock_start = gClock->now();
         }

       }else{
         tail_stand_mode = true;
         forward    = 0;
         yawratecmd = 0;
       }
   
   
   
       break;
   
     case Approach_to_Exit:
   
       forward    = 20;
       yawratecmd =  0;
   
    if(odo > ref_odo){
         forward    = 20;
         yawratecmd = 0;
         ret = 1;
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
     return(ret);
 }
 