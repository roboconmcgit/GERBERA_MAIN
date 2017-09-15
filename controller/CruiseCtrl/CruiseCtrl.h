/*
 *      走行制御クラスのヘッダファイル
 */

#ifndef CRUISE_CTRL_H_
#define CRUISE_CTRL_H_

#include "ev3api.h"
#include "Clock.h"

#include "util.h"
#include "parameter.h"

#include "GyroParts.h"
#include "Motor.h"

#include "MotorParts.h"
#include "GyroParts.h"

#include "BalancerCpp.h"
#include <deque>
#include "MotorParts.h"

using namespace std;
using ev3api::Clock;

//17.07.28 k-ota copy from 3-apex
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */

/*
 *  関数のプロトタイプ宣言
 */
 
class CruiseCtrl{
private:
    GyroParts* mGyroParts;
    MotorParts* mMotorParts;
    Balancer* mBalancer;

    enum enumStand_Mode{
        Balance_Mode,
        Tail_Down,
        Tail_On,
        Tail_Stand,
        Stand_Vert,
        Stand_to_Balance,
        Tail_for_Run,
        Debug_00
      };
      enumStand_Mode  Stand_Mode;

   
    signed int mAngleCommand;
    //17.07.28 kota copy from 3-apex

    float YawrateController(float yawrate, float yawrate_cmd);
	float yaw_ctl_dt = 0.004;

    float turn_tmp;
    float r_yaw_rate = 0.0;
    float r_yaw_rate_ud = 0.0;
    float I_gain1 = 1.0+1.0/0.8;
    float I_gain2 = yaw_ctl_dt/0.06/0.8;
    float I_gain3 = 1.0-yaw_ctl_dt/0.06;

    float F_controller(float r_yaw_rate);
    float F_in = 0.0;
    float F_out = 0.0;
//    float F_gain = 1/0.062;
    float F_gain = 1/0.062; //0818 tada
    
    float E_controller(float r_yaw_rate);
    float E_in;
    float E_in_d;
    float E_in_dd;
    float E_in_ddd;
    float E_in_dddd;
    float E_in_ddddd = 0.0;
    float E_in_dddddd = 0.0;

    float E_out = 0.0;
    float E_gain1 = yaw_ctl_dt/0.1;
    float E_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
    float E_ud1 = 0.0;

    float C_controller(float E_out, float yawrate, float S_out);
    float C_in = 0.0;
    float C_out = 0.0;
    //float C_gain = yaw_ctl_dt*10.0;
    float C_gain = yaw_ctl_dt*50.0;
    float C_ud1 = 0.0;

    float S_controller(float C_out);
    float S_gain1 = 1.0;
    float S_in;
    float S_in_d;
    float S_in_dd;
    float S_in_ddd;
    float S_in_dddd;
    float S_in_ddddd = 0.0;
    float S_in_dddddd = 0.0;
    float S_out = 0.0;
    float Pn_gain1 = yaw_ctl_dt/0.1;
    float Pn_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
    float Pn_ud1 = 0.0;
    float Pd_gain1 = yaw_ctl_dt/0.1;
    float Pd_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
    float Pd_ud1 = 0.0;

    void TailMode(int mForward, float mTurn); //PWM Gen. without Balancer task 0814

    bool balance_off_en;
    bool pre_balancer_on;
    
#ifdef DEBUG_ROBO
    dequefloat> dsave_log1;
    deque<float> dsave_log2;
    deque<float> dsave_log3;
    deque<float> dsave_log4;
    deque<float> dsave_log5;
    deque<float> dsave_log6;
    deque<float> dsave_log7;
    deque<float> dsave_log8;
    deque<float> dsave_log9;
    deque<float> dsave_log10;
#endif


    int mtail_mode_pwm_l;
    int mtail_mode_pwm_r;

    // LineTracerYawrate
    float y_t=0;
    float y_t_prev=0; //0818 tada. passed y_t
//protected:
public:
    static const int LOW;
    static const int NORMAL;
    static const int HIGH;

    Clock*       robo_Clock;

    int  offset;
    bool balance_mode;
    int   mmForward;
    int   mmTurn;
    float mmYawratecmd;//目標Yawrate
    float mmYawrate;

    float mYawratecmd;//目標Yawrate
    float mYawrate;
    int   mForward;
    float mTurn;

    bool mTailModeFlag;//0816
    
    int   log_forward;
    int   log_turn;
    int   log_gyro;
    int   log_left_wheel_enc;
    int   log_right_wheel_enc;
    int   log_battery;
    int   log_left_pwm;
    int   log_right_pwm;

    // LineTracerYawrate
    float pg = 0.35;//暫定0.9 //0818 tada
    float df = 0.024;//暫定-0.1 //0818 tada

    CruiseCtrl(GyroParts* gyro,
           MotorParts* motor,
           Balancer* balancer);            //コンストラクタ
    ~CruiseCtrl();                                 //デストラクタ
    void init();
    void CruiseCtrlOperation();
    void setCommand(int forward, float yawratecmd, signed int anglecommand, float yawrate, bool tail_mode_lflag);//0816
    
    void tail_stand_from_balance();
    void exportRobo(char *csv_header);
    void saveData(int idata_num);

    float LineTracer(int line_value, float traceforward); //ライントレース
    float LineTracerYawrate(int line_value, float Max_Yawrate, float Min_Yawrate);              //ライントレース（ヨーレート）
    
};

#endif // !CRUISE_CTRL_H_
