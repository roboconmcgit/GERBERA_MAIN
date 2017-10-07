/*
 *      モーターバーツクラスのヘッダファイル
 */

 #ifndef LOOKUPGATE_H_
 #define LOOKUPGATE_H_
 
 #include "ev3api.h"
 #include "parameter.h"
 #include "util.h"
 
 #include "CruiseCtrl.h"
 
 using ev3api::Clock;
 /*
  *  関数のプロトタイプ宣言
  */
  
 class LookUpGate{
 private:
 
     CruiseCtrl  *gCruiseCtrl;
     float LineTracerYawrate(int line_value);
 
 public:    

    enum enumLUG_Mode{
        LUG_Start,
        //1
        Approach_to_LUG,
        Tail_On_1st,
        POS_ADJ_1st,
        LUG_Mode_1st,
        LUG_1st,
        Pre_1st_Turn,
        Turn_1st,
        //8
        Approach_to_2nd_LUG,
        LUG_Mode_2nd,
        LUG_2nd,
        Pre_2nd_Turn,
        Turn_2nd,
  
        Approach_to_3rd_LUG,
        LUG_Mode_3rd,
        LUG_3rd,
  
        Tail_Stand_Up,
  
        LUG_Debug_00
      };

    enumLUG_Mode LUG_Mode;

    float ref_forward;
    float y_t;
    float ref_odo;

     LookUpGate(CruiseCtrl* Cruise);     //コンストラクタ
     ~LookUpGate();                      //デストラクタ
 
     void init ();
     
     int LookUpGateRunner(
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
     );
 };
 
 #endif // !LOOKUPGATE_H_
 