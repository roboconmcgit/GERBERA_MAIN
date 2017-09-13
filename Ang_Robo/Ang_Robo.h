/******************************************************************************
 *  Ang_Robo.h (for LEGO Mindstorms EV3)
 *****************************************************************************/

#ifndef EV3_UNIT_BALANCINGWALKER_H_
#define EV3_UNIT_BALANCINGWALKER_H_

#include "util.h"
#include "parameter.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "BalancerCpp.h"
#include <deque>

using namespace std;

//17.07.28 k-ota copy from 3-apex
//#define P_GAIN             0.65F /* 完全停止用モータ制御比例係数 */
#define P_GAIN             1 /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */

class Ang_Robo {
public:
    static const int LOW;
    static const int NORMAL;
    static const int HIGH;

    Ang_Robo(const ev3api::GyroSensor& gyroSensor,
                    ev3api::Motor& leftWheel,
                    ev3api::Motor& rightWheel,
                    ev3api::Motor& tail_motor,
                    Balancer* balancer);

    void init();
    void run();
//    void setCommand(int forward, float yawratecmd, signed int anglecommand, float yawrate);
    void setCommand(int forward, float yawratecmd, signed int anglecommand, float yawrate, bool tail_mode_lflag);//0816


    void tail_control(signed int angle); //2017.07.28 kota copy from 3-apex
   //170816 ota add tail control
    void tail_reset();
    void tail_stand_up(); //tail for gyro reset and color sensor calibration
    
    void exportRobo(char *csv_header);
    void saveData(int idata_num);
    
    bool balance_mode;
    int   mmForward;
    int   mmTurn;
    float mmYawratecmd;//目標Yawrate
    float mmYawrate;

private:
    const ev3api::GyroSensor& mGyroSensor;
    ev3api::Motor& mLeftWheel;
    ev3api::Motor& mRightWheel;
    ev3api::Motor& mTail_Motor;
    Balancer* mBalancer;
    PID *gTail_pwm = new PID();

    int   mForward;
    float mTurn;
    float mYawratecmd;//目標Yawrate
    float mYawrate;

    bool mTailModeFlag;//0816
   
    signed int mAngleCommand;
    //17.07.28 kota copy from 3-apex
    float angle2_e;   /* angle2用変数型宣言 */
    float angle2_eo1; /* angle2用変数型宣言 */
    float angle2_eo2; /* angle2用変数型宣言 */
    float pwm_o;      /* angle2用変数型宣言 */
    float pwm2;
    float pwm;

    float YawrateController(float yawrate, float yawrate_cmd);
	float yaw_ctl_dt = 0.004;

    float turn_tmp;
    float r_yaw_rate = 0.0;
    float r_yaw_rate_ud;
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
    int mtail_mode_pwm_l;
	int mtail_mode_pwm_r;

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

};

#endif  // EV3_UNIT_BALANCINGWALKER_H_
