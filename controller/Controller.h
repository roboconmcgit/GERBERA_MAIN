/*
 *      コントローラクラスのヘッダファイル
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "ev3api.h"
#include "parameter.h"
#include "app.h"
#include "CruiseCtrl.h"
#include "DifficultCtrl.h"
#include "Brain_Calc_Library.h"
#include "Clock.h"

#include "ColorParts.h"
#include "MotorParts.h"
#include "GyroParts.h"
#include "SonarParts.h"
#include "TouchParts.h"

#include <deque>

using namespace std;
using ev3api::Clock;

/*
 *  関数のプロトタイプ宣言
 */
 
 enum Sys_Mode{
    SYS_INIT            = 110,
    BT_CONECT           = 120,
    DISPLAY_SELECT_APLI = 210,
    CALIB_COLOR_SENSOR  = 310,
    WAIT_FOR_START      = 410,
    START               = 510,
    RUN                 = 530,
    LINE_TRACE_MODE     = 520,
    GOAL                = 610,
    DANSA               = 710,
    GARAGE              = 810,
    LUP                 = 910,
    STOP                = 1010,
    RESET               = 0
};

class Controller{
private:
    ColorParts  *gColorParts;
    MotorParts  *gMotorParts;
    GyroParts   *gGyroParts;
    SonarParts  *gSonarParts;
    TouchParts  *gTouchParts;
    
    Clock*       gClock;

    BrainCalcLibrary *gStep = new BrainCalcLibrary();       //段差走行オブジェクト（脳みそ計算ライブラリ）
    
    float mYaw_angle_offset;

    float ref_x;
    int   mLinevalue; //ライン検出値
    int   mLinevalue_LUG;
    float mXvalue;    //x座標
    float mYvalue;    //y座標
    float mOdo;       //Total distance [mm] from start point
    float mSpeed;     //速度
    float mYawrate;   //ヨーレート
    float mYawangle;  //ヨー角
    int   mTail_angle;
    //signals for robo movement
    bool  mRobo_stop       = 0;
    bool  mRobo_forward    = 0;
    bool  mRobo_back       = 0;
    bool  mRobo_turn_left  = 0;
    bool  mRobo_turn_right = 0;
    bool  mDansa;      //段差検出値
    bool  mDet_gray;      //段差検出値
    bool  mGarage = false;
    int32_t mSonar;

//protected:
public:
    CruiseCtrl  *gCruiseCtrl;
    DifficultCtrl *gDifficultCtrl;

    Balancer    *gBalancer;

    Sys_Mode mSys_Mode;

    int SysModeNum;
    int   Mmode;
    bool  mRobo_balance_mode;

	bool  left_line_edge = true;
	int   forward;         //前進目標値
	float yawratecmd;      //目標ヨーレート
	float anglecommand;    //尻尾角度
    bool  tail_mode_lflag; //倒立走行フラグ
    
    enum enumTrack_Mode{
        Start_to_1st_Straight,
        Start_to_1st_Corner,
        Fst_Corner,
        Snd_Corner,
        Final_Corner,
        Final_Straight,
        Get_Ref_Odo,
        Dead_Zone,
        Return_to_Line,
        Go_Step,
        Approach_to_Garage,
        Go_to_Garage,
        Garage_Tail_On,
        Return_to_Line_Garage,
        Garage_In,
        Garage_Stop,
        Stop_Robo,
        Go_LUG,
        Track_Debug_00,
        Track_Debug_01,
        Track_Debug_02
    };
    enumTrack_Mode  Track_Mode;

    Controller(
        ColorParts  *Color,
        MotorParts  *Motor,
        GyroParts   *Gyro,
        SonarParts  *Sonar,
        TouchParts  *Touch);            //コンストラクタ
    ~Controller();                                 //デストラクタ
    void ControllerInit();
    void run();
    void ControllerOperation();                    //動作判断

	void init();
	//走行戦略を計算
	void Track_run();
};

#endif // !CONTROLLER_H_
