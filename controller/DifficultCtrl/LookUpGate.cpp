/**
 ******************************************************************************
 ** ファイル名 : LookUpGate.cpp
 **
 ** 概要 : LookUpGateクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

 //#include "math.h"
 #include "LookUpGate.h"
 
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
 LookUpGate::LookUpGate(CruiseCtrl* Cruise)
 :gCruiseCtrl(Cruise)
 {
 }
 
 //*****************************************************************************
 // 関数名 : デストラクタ
 // 引数 : unused
 // 返り値 : なし
 // 概要 : 
 //*****************************************************************************
 LookUpGate::~LookUpGate(){
 
 }
 
 //*****************************************************************************
 // 関数名 : 
 // 引数 : unused
 // 返り値 : なし
 // 概要 : 
 //*****************************************************************************
 void LookUpGate::init(){
     LUG_Mode = LUG_Start;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
 float LookUpGate::LineTracerYawrate(int line_value){
     return(gCruiseCtrl->LineTracerYawrate(line_value,-1.0,-1.0));
 }

 //*****************************************************************************
 // 関数名 : 
 // 引数 : unused
 // 返り値 : なし
 // 概要 : 
 //*****************************************************************************
 int LookUpGate::LookUpGateRunner(
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

     switch(LUG_Mode){
   
     case LUG_Start:
       forward = 50;
       LineTracerYawrate((2*line_value));
       LUG_Mode      = Approach_to_LUG;
       ref_forward   = 0.0;
       ref_odo       = odo +  APPROACH_TO_LUG_LENGTH;
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
         y_t             = -2.0*(PAI - angle + LUG_GARAGE_OFFSET);
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
       
#if 0
       if(mSonar_dis <= STOP_POS_APP_LUG){
        forward         = 0;
        yawratecmd      = 0;
        tail_stand_mode = true;
        tail_lug_mode   = false;
        LUG_Mode        = LUG_Mode_1st;
       }
#endif
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
       y_t = -2.0*(PAI - angle + LUG_GARAGE_OFFSET);
       yawratecmd = y_t;
   
       if(odo > ref_odo){
         //LUG_Mode    = Pre_1st_Turn;

#if 1
         if(mSonar_dis == 255){
          LUG_Mode    = Pre_1st_Turn;
         }
  #endif
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

#if 0
       if(mSonar_dis <= STOP_POS_APP_LUG){
        forward         = 0;
        yawratecmd      = 0;
        tail_stand_mode = true;
        tail_lug_mode   = false;
        LUG_Mode        = LUG_Mode_2nd;
       }
#endif
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
   
       y_t = -2.0*(0 - angle + LUG_GARAGE_OFFSET);
       yawratecmd = y_t;
   
       if(odo > ref_odo){
         //LUG_Mode    = Pre_2nd_Turn;

#if 1
         if(mSonar_dis == 255){
          LUG_Mode    = Pre_2nd_Turn;
         }
#endif
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
         y_t             = -2.0*(PAI - angle + LUG_GARAGE_OFFSET);
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

#if 0
       if(mSonar_dis <= STOP_POS_APP_LUG){
        forward         = 0;
        yawratecmd      = 0;
        tail_stand_mode = true;
        tail_lug_mode   = false;
        LUG_Mode        = LUG_Mode_3rd;
       }
#endif
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
       y_t = -2.0*(PAI - angle + LUG_GARAGE_OFFSET);
       yawratecmd = y_t;
   
       if(odo > ref_odo){
         //LUG_Mode    = Tail_Stand_Up;
         
#if 1
        if(mSonar_dis == 255){
        LUG_Mode    = Tail_Stand_Up;
        }
#endif
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
 
 
 
 