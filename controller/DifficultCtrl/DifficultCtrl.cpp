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
    bool &tail_mode_lflag,
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
        tail_mode_lflag = true;
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
      
      if(mOdo < target_mOdo){
        forward = 15;
        y_t = -0.5*(2.5*PAI - angle);
        yawratecmd = y_t;
        tail_mode_lflag = true;
      }else{
        forward    = 0;
        yawratecmd = 0;
        tail_mode_lflag = true;
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
      tail_mode_lflag = true;
      if(mRobo_balance_mode == false){
        forward    = 0;
        yawratecmd = 0;
        tail_mode_lflag = true;
        Step_Mode = Second_Turn;
        clock_start = gClock->now();
        target_angle = angle + RAD_450_DEG + RAD_15_DEG;
      }
      break;
  
    case Second_Turn:
      tail_mode_lflag = true;
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
      tail_mode_lflag = true;
  
      //    if((gClock->now() - clock_start) > 1500){
      if((gClock->now() - clock_start) > 2500){
        //    if((gClock->now() - clock_start) > 2500){
  
        forward    = 0;
        yawratecmd = 0;
        tail_mode_lflag = true;
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
        tail_mode_lflag = false;
        if(mRobo_balance_mode == true){
      forward    = 40;
      yawratecmd = 0;
      anglecommand = TAIL_ANGLE_RUN;
      Step_Mode = Approach_to_Exit;
      clock_start = gClock->now();
        }
      }else{
        tail_mode_lflag = true;
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
    bool &tail_mode_lflag
){
    int ret = 0;

	float y_t;
    static float odo_starting_point;
    static float target_angle;
    static int32_t clock_start;


    switch(LUG_Mode){

    case LUG_Start:
          clock_start = gClock->now();
          LUG_Mode = Approach_to_LUG;

      break;

    case Approach_to_LUG:
      forward = 0;
      yawratecmd = 0;
      anglecommand = TAIL_ANGLE_RUN;

      if((gClock->now() - clock_start) > 1500){
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
          if(anglecommand<TAIL_ANGLE_RUN+63.5){
            LUG_Mode = LUG_Stop0;
            odo_starting_point = odo;
            clock_start = gClock->now();
          }
      break;


    case LUG_Stop0:
          forward = 0;
          yawratecmd=0;

          if((gClock->now() - clock_start) > 800){
            LUG_Mode = LUG_forward;
          }

      break;

    case LUG_forward:
          forward = 32;
          /*
          y_t = (((float)line_value_lug-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
          if(y_t > 10.0) y_t = 10.0;
          if(y_t < -10.0) y_t = -10.0;

          yawratecmd = y_t;						//目標yawrate値を更新
          */

          yawratecmd = LineTracer(line_value_lug,forward);

          //yawratecmd = 0;

          if((odo-odo_starting_point)>800){
            LUG_Mode = LUG_Stop1;
            clock_start = gClock->now();
          }

      break;

    case LUG_Stop1:
          forward = 0;
          yawratecmd=0;

          if((gClock->now() - clock_start) > 800){
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

          if((gClock->now() - clock_start) > 800){
            LUG_Mode = LUG_forward2;
            clock_start = gClock->now();
          }

      break;

    case LUG_forward2:
          forward = 31;
          /*
          y_t = (((float)line_value_lug-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
          if(y_t > 10.0) y_t = 10.0;
          if(y_t < -10.0) y_t = -10.0;

          yawratecmd = y_t;						//目標yawrate値を更新
          */

          yawratecmd = LineTracer(line_value_lug,forward);

          if((odo-odo_starting_point)>800){
            LUG_Mode = LUG_Stop3;
            clock_start = gClock->now();
          }

      break;

    case LUG_Stop3:
          forward = 0;
          yawratecmd=0;

          if((gClock->now() - clock_start) > 800){
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

          if((gClock->now() - clock_start) > 800){
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

          yawratecmd = LineTracer(line_value_lug,forward);

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
          forward = -7;
          yawratecmd = 0;
          anglecommand = anglecommand +0.025;
          if(anglecommand>TAIL_ANGLE_RUN+85){
              LUG_Mode = LUG_Stop6;
              clock_start = gClock->now();
                target_angle = angle - 3.14;
          }
      break;

    case LUG_Stop6:
          forward = 0;
          yawratecmd=0;

          if((gClock->now() - clock_start) > 800){
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

          if((gClock->now() - clock_start) > 800){
        LUG_Mode = END;
          }

      break;

    case END:
          forward = 0;
          yawratecmd = 0;
      ret = 1;
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
    bool &tail_mode_lflag
){
    forward =  0;
    yawratecmd = 0.0;
    return 1;
}
