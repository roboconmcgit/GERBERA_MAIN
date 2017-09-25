/**
 ******************************************************************************
 ** ファイル名 : DifficultCtrl.cpp
 **
 ** 概要 : DifficultCtrlクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "math.h"
#include "DifficultCtrl.h"

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
DifficultCtrl::DifficultCtrl(CruiseCtrl* Cruise)
:gCruiseCtrl(Cruise)
{

}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
DifficultCtrl::~DifficultCtrl(){

}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void DifficultCtrl::init(){
    gClock       = new Clock();
    Step_Mode  = Step_Start;
    LUG_Mode = LUG_Start;
  }
  //*****************************************************************************
  // 関数名 : 
  // 引数 : unused
  // 返り値 : なし
  // 概要 : 
  //*****************************************************************************
float DifficultCtrl::LineTracerYawrate(int line_value){
    return(gCruiseCtrl->LineTracerYawrate(line_value,-1.0,-1.0));
}
//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
float DifficultCtrl::LineTracer(int line_value, float traceforward){
    return (gCruiseCtrl->LineTracer(line_value, traceforward));
}
//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
int DifficultCtrl::StartDashRunner(){
    return 0;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
int DifficultCtrl::StepRunner(
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
){
/*前提条件：ロボットがライン上にあること*/
    int ret = 0;
    float y_t;
    static float target_mOdo;
    static float target_angle;
    static float target_tail_angle;
    static int32_t clock_start;
    static int   dansa_cnt;
    static int target_cnt;
    
    switch(Step_Mode){
  
    case Step_Start:
      dansa_cnt = 0;
      //    forward =  50;
      forward =  70;
      yawratecmd = LineTracerYawrate((CL_SNSR_GAIN_GRAY*line_value));
      target_mOdo = mOdo + STEP_START_LENGTH;
   
  
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
  
      if(mOdo > target_mOdo){
        forward =  20;
  
      }else{
        //      forward =  50;
        forward =  70;
      }
  
      yawratecmd = LineTracerYawrate((CL_SNSR_GAIN_GRAY*line_value));
      if((angle >  RAD_90_DEG)&&(yawratecmd < 0) ){
        yawratecmd = 0.0;
      }
  
      /*kota0915
      if(dansa){
        Step_Mode   = First_Dansa;
        target_mOdo  = mOdo + FST_DANSA_POS;
        clock_start = gClock->now();
        dansa_cnt   = 0;
        ref_x       = mXvalue; //reference x pos for Garage
  #ifdef STEP_DEBUG
        ref_x       =mYvalue;
  #endif
      }
      break;
      */
  
      if(dansa){
        Step_Mode   = First_Dansa;
        target_mOdo  = mOdo + FST_DANSA_POS;
        clock_start = gClock->now();
        dansa_cnt   = 0;
        target_cnt = 0;
        gStep->SetInitPIDGain(0.1,0.01,0.001,dT_4ms);
        ref_x       = mXvalue; //reference x pos for Garage
  
  #ifdef STEP_DEBUG
        ref_x       =mYvalue;
  #endif
      }
      break;
  
      /*
    case First_Dansa:
      if(dansa){
        dansa_cnt++;
      }
  
      if(dansa_cnt < 50){
        if(mOdo > target_mOdo){
      forward = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = First_Dansa_On;
      gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
      dansa = 0;
        }else{
      forward    =  STEP_CLIMB_SPPED;
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
      */
  
    case First_Dansa:
      if(dansa){
        dansa_cnt++;
      }
      
      if((mOdo > target_mOdo - 25 )&&(mOdo < target_mOdo + 25)){
        target_cnt++;
      }
      
      if(dansa_cnt < 50){
        /*
        if(mOdo > target_mOdo){
      forward = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      //Step_Mode = First_Dansa_On;
      //	gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
      dansa = 0;
        */
        if(target_cnt > 750){ //3sec
      forward = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = First_Dansa_Tail_On;
      dansa = 0;
      target_cnt = 0;
        }else{
      forward    = gStep->CalcPIDContrInput(target_mOdo, mOdo);
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
      forward = gStep->CalcPIDContrInput(target_mOdo, mOdo);
      forward = forward * 0.1;
      yawratecmd = 0;
      anglecommand = target_tail_angle;
  
  
      //    if((gClock->now() - clock_start) > 5000){
      if((gClock->now() - clock_start) > 3000){
        Step_Mode = First_Dansa_Tail_On;
        clock_start = gClock->now();
      }
  
    break;
  
    case First_Dansa_Tail_On:
      if(mOdo < target_mOdo){
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
        //      Step_Mode = Fst_Turn_Pos_Adj;
        Step_Mode = First_Turn;
        clock_start = gClock->now();
        target_angle = angle + RAD_360_DEG + RAD_15_DEG;
      }
  
      break;
      
      //not used 0914 kota
    case Fst_Turn_Pos_Adj:
      forward = gStep->CalcPIDContrInput(target_mOdo, mOdo);
      forward = forward * 0.1;
      yawratecmd = 0;
      anglecommand = target_tail_angle;
      
      //    if((gClock->now() - clock_start) > 3000){
      if((gClock->now() - clock_start) > 500){
        Step_Mode = First_Turn;
        forward = 0;
        yawratecmd = 0;
        clock_start = gClock->now();
      }
      break;
  
    case First_Turn:
      tail_stand_mode = true;
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
      
      if(mOdo < target_mOdo){
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
  
  
      //    if((gClock->now() - clock_start) > 3000){
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
      
  
      //    if((gClock->now() - clock_start) < 5000){
      if((gClock->now() - clock_start) < 1000){
        forward      = 0;
        yawratecmd   = 0;
        anglecommand = TAIL_ANGLE_RUN;
      }else{
        forward    = 10;
        y_t        = -0.5*(2.5*PAI - angle);
        yawratecmd = y_t;
  
        /*0914 kota
        if(dansa){
      forward    = 0;
      yawratecmd = 0;
      //	Step_Mode = Pre_Second_Dansa;
      Step_Mode = Second_Dansa;
      clock_start = gClock->now();
      target_mOdo = mOdo + SCD_DANSA_POS;
      }*/
  
        if(dansa){
      Step_Mode   = Second_Dansa;
      target_mOdo  = mOdo + SCD_DANSA_POS;
      clock_start = gClock->now();
      dansa_cnt   = 0;
      target_cnt = 0;
      gStep->SetInitPIDGain(0.1,0.01,0.001,dT_4ms);
  
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
      
      /*
    case Second_Dansa:
  
      if(dansa){
        dansa_cnt++;
      }
  
      if(dansa_cnt < 50){
        if(mOdo > target_mOdo - 30){
      forward    = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = Second_Dansa_On;
      gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
      dansa = 0;
        }else{
      forward    =  STEP_CLIMB_SPPED;
      yawratecmd = 0;
      clock_start = gClock->now();
        }
      }else{
        forward = -10;
        yawratecmd = 0;
        if((gClock->now() - clock_start) > 1000){
      dansa_cnt = 0;
        }
      }
    
      break;
      */
  
    case Second_Dansa:
      if(dansa){
        dansa_cnt++;
      }
      
      if((mOdo > target_mOdo - 25 )&&(mOdo < target_mOdo + 25)){
        target_cnt++;
      }
      
      if(dansa_cnt < 50){
  
        if(target_cnt > 750){ //3sec
      forward    = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode   = Second_Dansa_On;
      dansa       = 0;
      target_cnt  = 0;
      target_mOdo  = target_mOdo + 600;
      gStep->SetInitPIDGain(0.1,0.01,0.001,dT_4ms);
        }else{
      forward    = gStep->CalcPIDContrInput(target_mOdo, mOdo);
      forward    = forward * 0.2;
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
      forward      = gStep->CalcPIDContrInput(target_mOdo, mOdo);
      forward      = forward * 0.1;
      yawratecmd   = 0;
      anglecommand = target_tail_angle;
      /*
      if((gClock->now() - clock_start) > 5000){
        Step_Mode = Second_Dansa_Tail_On;
        clock_start = gClock->now();
        forward    = 0;
        yawratecmd = 0;
        }*/
  
      if((mOdo > target_mOdo - 25 )&&(mOdo < target_mOdo + 25)){
        target_cnt++;
      }
  
      if(target_cnt > 750){ //3sec
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
        target_angle = angle + RAD_450_DEG + RAD_15_DEG;
      }
      break;
  
    case Second_Turn:
      tail_stand_mode = true;
      //    if((gClock->now() - clock_start) > 1000){
      if((gClock->now() - clock_start) > 500){
        if(angle >= target_angle){
      Step_Mode = Second_Pre_Stand_Up;
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
      y_t = -10.0*(5*PAI - angle);
      yawratecmd = y_t;
      tail_stand_mode = true;
  
      //    if((gClock->now() - clock_start) > 1500){
      if((gClock->now() - clock_start) > 2500){
        //    if((gClock->now() - clock_start) > 2500){
  
        forward    = 0;
        yawratecmd = 0;
        tail_stand_mode = true;
        clock_start = gClock->now();
        Step_Mode = Second_Dansa_Stand_Up;
        target_mOdo = mOdo + 150;
      }
      
      break;
  
  
    case Second_Dansa_Stand_Up:
  
      //    if((gClock->now() - clock_start) > 3000){
      if((gClock->now() - clock_start) > 1000){
        
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
  
  
      /*    if(dansa){
        forward    = 20;
        yawratecmd = 0;
        Track_Mode = Approach_to_Garage;
        }*/
  
      if(mOdo > target_mOdo){
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


//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
int DifficultCtrl::LookUpGateRunner(
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
){
    int ret = 0;
    static float ref_forward;
    static float y_t;
    static float ref_odo;
  
    switch(LUG_Mode){
  
    case LUG_Start:
      forward = 50;
      LineTracerYawrate((2*line_value));
      LUG_Mode      = Approach_to_LUG;
      ref_forward   = 0.0;
      ref_odo       = odo +  APPROACH_TO_LUG_LENGTH;  
      gForward->init_pid(0.05,0.01,0.001,dT_4ms);
      break;
  
    case Approach_to_LUG:
  
      ref_forward = (ref_odo - odo)/10.0+0.5;
  
      if(ref_forward > 70){
        ref_forward = 70;
      }else if(ref_forward < 10){
        ref_forward = 10;
      }else{
        ref_forward = ref_forward;
      }
      forward = (int)ref_forward;
      LineTracerYawrate((2*line_value));
  
      if(mSonar_dis <= STOP_POS_FROM_LUG){
        forward     = 0;
        yawratecmd  = 0;
        LUG_Mode    = Tail_On_1st;
      }
      break;
  
    case Tail_On_1st:
      tail_stand_mode = true;
      tail_lug_mode  = false;
  
      forward    = 0;
      yawratecmd = 0;
      if(mRobo_balance_mode == false){
        forward    = 0;
        yawratecmd = 0;
        ref_odo    = odo + APPROACH_TO_1st_LUG;
        LUG_Mode   = POS_ADJ_1st;
      }
      break;
  
    case POS_ADJ_1st:
      
      if(odo < ref_odo){
        forward         = 15;
        y_t             = -2.0*(PAI - angle);
        yawratecmd      = y_t;
        tail_stand_mode = true;
        tail_lug_mode  = false;
      }else{
        forward         = 0;
        yawratecmd      = 0;
        tail_stand_mode = true;
        tail_lug_mode   = false;
        LUG_Mode        = LUG_Mode_1st;
      }
      break;
  
  
    case LUG_Mode_1st:
      forward      = 0;
      ref_forward  = 0.0;
      yawratecmd   = 0;
      tail_lug_mode  = true;
  
      if(mRobo_lug_mode == true){
        ref_odo       = odo + LUG_1st_STOP;
        LUG_Mode      = LUG_1st;
  
      }
      break;
  
    case LUG_1st:
  
      ref_forward = ref_forward+0.1; //modify later
      forward     = (int)(ref_forward + 0.5);
  
      if(forward >= 10){
        forward = 10;
      }
      y_t = -2.0*(PAI - angle);
      yawratecmd = y_t;
  
      /*
      if(mSonar_dis < min_sonar_dis){
        min_sonar_dis = mSonar_dis;
      }
      if((min_sonar_dis < 7)&&(mSonar_dis > 100)){
        ref_odo = odo + 100;
        min_sonar_dis = 10;
      }
      */
  
      if(odo > ref_odo){
        LUG_Mode    = Pre_1st_Turn;
      }
      break;
      
    case Pre_1st_Turn:
      forward       = 0;
      yawratecmd    = 0;
      tail_lug_mode = false;
  
      if(mRobo_lug_mode == false){
        LUG_Mode    = Turn_1st;
      }
      
      break;
      
    case Turn_1st:
        if(angle < 0){
            forward     = 0;
            yawratecmd  = 0;
      LUG_Mode    = Approach_to_2nd_LUG;
      ref_odo     = odo + APPROACH_TO_2nd_LUG;
        }else{
    forward = 0;
    y_t = y_t + 0.005;
    if(y_t >= 1){
      y_t = 1;
    }
    yawratecmd = y_t;
        }
        break;
  
    case Approach_to_2nd_LUG:
      if(odo < ref_odo){
        forward         = 15;
        y_t             = -2.0*(0 - angle);
        yawratecmd      = y_t;
        tail_stand_mode = true;
        tail_lug_mode   = false;
      }else{
        forward         = 0;
        yawratecmd      = 0;
        tail_stand_mode = true;
        tail_lug_mode   = false;
        LUG_Mode        = LUG_Mode_2nd;
      }
      break;
  
    case LUG_Mode_2nd:
      forward       = 0;
      yawratecmd    = 0;
      tail_lug_mode = true;
  
      if(mRobo_lug_mode == true){
        ref_odo     = odo + LUG_2nd_STOP;
        ref_forward  = 0.0;
        LUG_Mode    = LUG_2nd;
      }
      break;
  
    case LUG_2nd:
  
      ref_forward = ref_forward+0.1; //modify later
      forward     = (int)(ref_forward + 0.5);
  
      if(forward >= 10){
        forward = 10;
      }
  
      y_t = -2.0*(0 - angle);
      yawratecmd = y_t;
  
      if(odo > ref_odo){
        LUG_Mode    = Pre_2nd_Turn;
      }
      break;
  
    case Pre_2nd_Turn:
      forward       = 0;
      yawratecmd    = 0;
      tail_lug_mode = false;
  
      if(mRobo_lug_mode == false){
        LUG_Mode    = Turn_2nd;
      }
      
      break;
  
  
  
    case Turn_2nd:
      if(angle > PAI){
            forward     = 0;
            yawratecmd  = 0;
      LUG_Mode    = Approach_to_3rd_LUG;
      ref_odo     = odo + APPROACH_TO_3rd_LUG;
        }else{
    forward = 0;
    y_t = y_t - 0.005;
    if(y_t <= -1){
      y_t = -1;
    }
    yawratecmd = y_t;
        }
        break;
  
    case Approach_to_3rd_LUG:
      if(odo < ref_odo){
        forward         = 15;
        y_t             = -2.0*(PAI - angle);
        yawratecmd      = y_t;
        tail_stand_mode = true;
        tail_lug_mode   = false;
      }else{
        forward         = 0;
        yawratecmd      = 0;
        tail_stand_mode = true;
        tail_lug_mode   = false;
        LUG_Mode        = LUG_Mode_3rd;
      }
      break;
  
    case LUG_Mode_3rd:
      forward      = 0;
      yawratecmd   = 0;
      tail_lug_mode  = true;
  
      if(mRobo_lug_mode == true){
        ref_odo      = odo + LUG_3rd_STOP;
        ref_forward  = 0.0;
        LUG_Mode     = LUG_3rd;
      }
      break;
  
    case LUG_3rd:
  
      ref_forward = ref_forward+0.1; //modify later
      forward     = (int)(ref_forward + 0.5);
      if(forward >= 10){
        forward = 10;
      }
      y_t = -2.0*(PAI - angle);
      yawratecmd = y_t;
  
      if(odo > ref_odo){
        LUG_Mode    = Tail_Stand_Up;
      }
      break;
  
    case Tail_Stand_Up:
      forward       = 0;
      yawratecmd    = 0;
      tail_lug_mode = false;
  
      if(mRobo_lug_mode == false){
        ret = 1;
      }
      break;
  
    default:
      forward      = 0;
      yawratecmd   = 0;
      anglecommand = TAIL_ANGLE_RUN; //0817 tada
      tail_stand_mode = false;
      break;
  
    }
    return(ret);
}



//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
int DifficultCtrl::GarageRunner(int line_value_lug, float mOdo, float angle,int line_value){
    return 0;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
int DifficultCtrl::StopRobo(
    int &forward,
    float &yawratecmd,
    float &anglecommand,
    bool &tail_stand_mode
){
    forward =  0;
    yawratecmd = 0.0;
    return 1;
}
