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
DifficultCtrl::DifficultCtrl(CruiseCtrl* Cruise):gCruiseCtrl(Cruise){

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
    float odo, 
    float angle, 
    bool dansa, 
    bool Robo_balance_mode,
    int &forward,
    float &anglecommand,
    float &Yawratecmd,
    bool &tail_mode_lflag
){
/*前提条件：ロボットがライン上にあること*/
    int ret = 0;
    float y_t;
    static float target_odo;
    static float target_angle;
    static float target_tail_angle;
    static int32_t clock_start;

    switch(Step_Mode){

        case Step_Start:
            forward =  50;
            gCruiseCtrl->LineTracerYawrate((2*line_value),-1.0,-1.0);
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

            gCruiseCtrl->LineTracerYawrate((2*line_value),-1.0,-1.0);
            if((angle >  RAD_90_DEG)&&(Yawratecmd < 0) ){
                Yawratecmd = 0.0;
            }
            if(dansa){
                Step_Mode = First_Dansa;
                target_odo = odo + FST_DANSA_POS;
            }
            break;

        case First_Dansa:
            if(odo > target_odo){
                forward = 0;
                Yawratecmd = 0;
                target_tail_angle =  TAIL_ANGLE_RUN;
                clock_start = gClock->now();
                Step_Mode = First_Dansa_On;
                gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
            }else{
                forward    = 20;
                Yawratecmd = 0;
            }
            break;

        case First_Dansa_On:
            forward = gStep->CalcPIDContrInput(target_odo, odo);
            forward = forward * 0.1;
            Yawratecmd = 0;
            anglecommand = target_tail_angle;

            if((gClock->now() - clock_start) > 5000){
                Step_Mode = First_Dansa_Tail_On;
                clock_start = gClock->now();
            }

            break;

        case First_Dansa_Tail_On:

            tail_mode_lflag = true;
            forward    = 0;
            Yawratecmd = 0;
            if(Robo_balance_mode == false){
                forward    = 0;
                Yawratecmd = 0;
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
                Yawratecmd = 0;
            }else{
                forward = 0;
                y_t = -2.0;
                Yawratecmd = y_t;
            }						//目標yawrate値を更新
            break;

        case First_Pre_Stand_Up:

            forward = 10;
            y_t = -0.5*(2.5*PAI - angle);
            Yawratecmd = y_t;
            tail_mode_lflag = true;

            if((gClock->now() - clock_start) > 1500){
                forward    = 0;
                Yawratecmd = 0;
                tail_mode_lflag = true;
                clock_start = gClock->now();
                Step_Mode = First_Dansa_Stand_Up;
            }
            break;

        case First_Dansa_Stand_Up:

            if((gClock->now() - clock_start) > 3000){
                forward    = 0;
                Yawratecmd = 0;

                anglecommand = TAIL_ANGLE_RUN;
                tail_mode_lflag = false;
                if(Robo_balance_mode == true){
                    forward    = 0;
                    Yawratecmd = 0;
                    anglecommand = TAIL_ANGLE_RUN;
                    Step_Mode = Approach_to_2nd_Step;
                    clock_start = gClock->now();
                }
            }else{
                tail_mode_lflag = true;
                forward    = 0;
                Yawratecmd = 0;
            }
            #ifdef STEP_DEBUG

            #endif
            break;

        case Approach_to_2nd_Step:
            if((gClock->now() - clock_start) < 5000){
                forward    = 0;
                Yawratecmd = 0;
                anglecommand = TAIL_ANGLE_RUN;
            }else{
                forward = 10;

                y_t = -0.5*(2.5*PAI - angle);
                #ifdef STEP_DEBUG
                //      y_t = -0.5*(2.0*PAI - angle);
                #endif
                Yawratecmd = y_t;


                if(dansa){
                    forward    = 10;
                    Yawratecmd = 0;
                    Step_Mode = Second_Dansa;
                    target_odo = odo + SCD_DANSA_POS;
                }
            }

            break;

        case Second_Dansa:
            if(odo > target_odo){
                forward    = 0;
                Yawratecmd = 0;
                target_tail_angle =  TAIL_ANGLE_RUN;
                clock_start = gClock->now();
                Step_Mode = Second_Dansa_On;
            }else{
                forward    = 15;
                Yawratecmd = 0;
            }
            break;


        case Second_Dansa_On:
            gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
            forward = gStep->CalcPIDContrInput(target_odo, odo);
            forward = forward * 0.1;
            Yawratecmd = 0;
            anglecommand = target_tail_angle;

            if((gClock->now() - clock_start) > 5000){
                Step_Mode = Second_Dansa_Tail_On;
                clock_start = gClock->now();
                forward    = 0;
                Yawratecmd = 0;
            }

            break;

        case Second_Dansa_Tail_On:
            forward    = 0;
            Yawratecmd = 0;
            tail_mode_lflag = true;
            if(Robo_balance_mode == false){
                forward    = 0;
                Yawratecmd = 0;
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
                    Yawratecmd = 0;

                }else{
                    forward = 0;
                    y_t     = -2.0;
                    Yawratecmd = y_t;						//目標yawrate値を更新
                }						//目標yawrate値を更新
            }
            else{
                forward = 0;
                Yawratecmd = 0;
            }
            break;

        case Second_Pre_Stand_Up:
            forward = 10;
            y_t = -0.5*(5*PAI - angle);
            Yawratecmd = y_t;
            tail_mode_lflag = true;

            if((gClock->now() - clock_start) > 1500){
                forward    = 0;
                Yawratecmd = 0;
                tail_mode_lflag = true;
                clock_start = gClock->now();
                Step_Mode = Second_Dansa_Stand_Up;
            }

            break;

        case Second_Dansa_Stand_Up:

            if((gClock->now() - clock_start) > 15000){
                forward    = 0;
                Yawratecmd = 0;
                anglecommand = TAIL_ANGLE_RUN;
                Step_Mode = Approach_to_Exit;
                clock_start = gClock->now();
                tail_mode_lflag = false;
            }

            if((gClock->now() - clock_start) > 3000){
                forward    = 0;
                Yawratecmd = 0;

                anglecommand = TAIL_ANGLE_RUN;
                tail_mode_lflag = false;

                if(Robo_balance_mode == true){
                    forward    = 0;
                    Yawratecmd = 0;
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
                Yawratecmd = 0;
                tail_mode_lflag = true;
            }


            break;

        case Approach_to_Exit:

            forward    = 20;

            y_t = -0.5*(5*PAI - angle);
            #ifdef STEP_DEBUG
            //    y_t = -0.5*(4.5*PAI - angle);
            #endif

            Yawratecmd = y_t;


            if(dansa){
                forward    = 20;
                Yawratecmd = 0;
                ret = 1;
            }
            if(odo > target_odo){
                forward    = 20;
                Yawratecmd = 0;
                ret = 1;
            }


            break;

        case Change_Left_Edge_Trace:

        break;

        case End_of_Step:

        break;

        default:
            Yawratecmd = 0;
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
    bool Robo_balance_mode,
    int &forward,
    float &anglecommand,
    float &Yawratecmd,
    bool &tail_mode_lflag
){
    int ret = 0;
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
        Yawratecmd = 0;
        anglecommand = TAIL_ANGLE_RUN;

        if((gClock->now() - clock_start) > 2000){
            LUG_Mode = LUG_Tail_On;
            anglecommand = TAIL_ANGLE_RUN+80;
            clock_start = gClock->now();
        }
        break;

    case LUG_Tail_On:
        tail_mode_lflag = true;
        forward    = 0;
        Yawratecmd = 0;
        if(Robo_balance_mode == false){
            forward    = 0;
            Yawratecmd = 0;
            LUG_Mode = LUG_Tailangle;
            clock_start = gClock->now();
        }
        break;

    case LUG_Tailangle:
        forward = 0;
        Yawratecmd = 0;
        anglecommand = anglecommand -0.01;
        if(anglecommand<TAIL_ANGLE_RUN+62.0){
            LUG_Mode = LUG_Stop0;
            odo_starting_point = odo;
            clock_start = gClock->now();
        }
        break;

    case LUG_Stop0:
        forward = 0;
        Yawratecmd=0;

        if((gClock->now() - clock_start) > 1000){
            LUG_Mode = LUG_forward;
            ref_angle=angle;
        }

        break;

    case LUG_forward:
        forward = 30;
        Yawratecmd = gCruiseCtrl->LineTracer(line_value_lug,forward);

        if((odo-odo_starting_point)>800){
            LUG_Mode = LUG_Stop1;
            clock_start = gClock->now();
        }

        break;

    case LUG_Stop1:
        forward = 0;
        Yawratecmd=0;

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
            Yawratecmd = 0;
            LUG_Mode = LUG_Stop2;
            odo_starting_point = odo;
        }else{
            forward = 0;
            //      y_t = -0.5*(target_angle - angle);
            y_t = 10;
            Yawratecmd = y_t;						//目標yawrate値を更新
        }						//目標yawrate値を更新
        break;

    case LUG_Stop2:
        forward = 0;
        Yawratecmd=0;

        if((gClock->now() - clock_start) > 1000){
            LUG_Mode = LUG_forward2;
            clock_start = gClock->now();
        }

        break;

    case LUG_forward2:
        forward = 29;
        if(ref_angle-angle>0.78){

            Yawratecmd = 2;

        }else if(ref_angle-angle<-0.78){

            Yawratecmd = -2;

        }else{

            Yawratecmd = gCruiseCtrl->LineTracer(line_value_lug,forward);
        }

        if((odo-odo_starting_point)>800){
            LUG_Mode = LUG_Stop3;
            clock_start = gClock->now();
        }

        break;

    case LUG_Stop3:
        forward = 0;
        Yawratecmd=0;

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
            Yawratecmd = 0;
            LUG_Mode = LUG_Stop4;
            odo_starting_point = odo;

        }else{
            forward = 0;
            //      y_t = -0.5*(target_angle - angle);
            y_t = 10;
            Yawratecmd = y_t;						//目標yawrate値を更新
        }						//目標yawrate値を更新

        break;

    case LUG_Stop4:
        forward = 0;
        Yawratecmd=0;

        if((gClock->now() - clock_start) > 1000){
            LUG_Mode = LUG_forward3;
            clock_start = gClock->now();
        }

        break;

    case LUG_forward3:
        forward = 32;
        if(ref_angle-angle>0.78){

            Yawratecmd = 2;

        }else if(ref_angle-angle<-0.78){

            Yawratecmd = -2;

        }else{

            Yawratecmd = gCruiseCtrl->LineTracer(line_value_lug,forward);
        }

        if((odo-odo_starting_point)>800){
            LUG_Mode = LUG_Stop5;
            clock_start = gClock->now();
        }

        break;

    case LUG_Stop5:
        forward = 0;
        Yawratecmd=0;

        if((gClock->now() - clock_start) > 1000){
            clock_start = gClock->now();
            LUG_Mode = LUG_Stand_Up;
        }

        break;

    case LUG_Stand_Up:
        forward = -9;
        Yawratecmd = 0;
        anglecommand = anglecommand +0.025;
        if(anglecommand>TAIL_ANGLE_RUN+85){
            LUG_Mode = LUG_Stop6;
            clock_start = gClock->now();
            target_angle = angle - 3.1;
        }
        break;

    case LUG_Stop6:
        forward = 0;
        Yawratecmd=0;

        if((gClock->now() - clock_start) > 1000){
            clock_start = gClock->now();
            LUG_Mode = Garage_Turn;
        }

        break;

    case Garage_Turn:
        if(angle < target_angle){
            clock_start = gClock->now();
            forward = 0;
            Yawratecmd = 0;
            LUG_Mode = Garage_Stop0;
            odo_starting_point = odo;

        }else{
            forward = 0;
            //      y_t = -0.5*(target_angle - angle);
            y_t = 10;
            Yawratecmd = y_t;						//目標yawrate値を更新
        }						//目標yawrate値を更新

        break;

    case Garage_Stop0:
        forward = 0;
        Yawratecmd = 0;

        if((gClock->now() - clock_start) > 1000){
            LUG_Mode = END;
        }

        break;

    case END:
        forward = 0;
        Yawratecmd = 0;
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
int DifficultCtrl::GarageRunner(int line_value_lug, float odo, float angle,int line_value){
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
    float &anglecommand,
    float &Yawratecmd,
    bool &tail_mode_lflag
){
    forward =  0;
    Yawratecmd = 0.0;
    return 1;
}
