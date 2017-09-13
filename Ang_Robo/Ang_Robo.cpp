/******************************************************************************
 *  Ang_Robo.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Ang_Robo
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "Ang_Robo.h"

//補助輪モード
//#define DEBUG_STOP
//#define DEBUG_LINETRACE
//#define DEBUG_LINETRACE_BALANCER


// 定数宣言
const int Ang_Robo::LOW    = 30;    // 低速
const int Ang_Robo::NORMAL = 50;    // 通常
const int Ang_Robo::HIGH   = 70;    // 高速

/**
 * コンストラクタ
 * @param gyroSensor ジャイロセンサ
 * @param leftWheel  左モータ
 * @param rightWheel 右モータ
 * @param balancer   バランサ
 */
Ang_Robo::Ang_Robo(const ev3api::GyroSensor& gyroSensor,
                                 ev3api::Motor& leftWheel,
                                 ev3api::Motor& rightWheel,
                                 ev3api::Motor& tail_motor,
                                 Balancer* balancer)
    : mGyroSensor(gyroSensor),
      mLeftWheel(leftWheel),
      mRightWheel(rightWheel),
      mTail_Motor(tail_motor),
      mBalancer(balancer),
      mForward(LOW),
      mTurn(LOW),

      //17.07.28 k-ota add for tail_control
     angle2_e(0),   /* angle2用変数型宣言 */
     angle2_eo1(0), /* angle2用変数型宣言 */
     angle2_eo2(0), /* angle2用変数型宣言 */
     pwm_o(0),      /* angle2用変数型宣言 */
     pwm2(0),
     pwm(0)
 {
}

/**
 * バランス走行する
 */
void Ang_Robo::run() {
    int16_t angle = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    int rightWheelEnc = mRightWheel.getCount();       // 右モータ回転角度
    int leftWheelEnc  = mLeftWheel.getCount();        // 左モータ回転角度

    tail_control(mAngleCommand);

	//アクティブヨーレート();
    mTurn = YawrateController(mYawrate, mYawratecmd);
    mBalancer->setCommand(mForward, mTurn);

#ifdef DEBUG_LINETRACE_BALANCER
    mBalancer->setCommand(mForward, mYawratecmd);
#endif


    int battery = ev3_battery_voltage_mV();
    mBalancer->update(angle, rightWheelEnc, leftWheelEnc, battery);
    //    TailMode(mForward, mTurn);

    // 左右モータに回転を指示する

    /*	if(mTailModeFlag == false) //0816
	{
        mLeftWheel.setPWM(mBalancer->getPwmLeft());
        mRightWheel.setPWM(mBalancer->getPwmRight());
	}
	else
	{
		mLeftWheel.setPWM(mtail_mode_pwm_l);
		mRightWheel.setPWM(mtail_mode_pwm_r);
		}*/

    if(mTailModeFlag == true)
	{
	  TailMode(mForward, mYawratecmd);
	  mLeftWheel.setPWM(mtail_mode_pwm_l);
	  mRightWheel.setPWM(mtail_mode_pwm_r);
	  balance_mode = false;
	}
	else
	{
	  mLeftWheel.setPWM(mBalancer->getPwmLeft());
	  mRightWheel.setPWM(mBalancer->getPwmRight());
	  balance_mode = true;
	}

#ifdef DEBUG_LINETRACE
    mLeftWheel.setPWM(mForward-mYawratecmd);
    mRightWheel.setPWM(mForward+mYawratecmd);
#endif

#ifdef DEBUG_STOP
    mLeftWheel.setPWM(0);
    mRightWheel.setPWM(0);
#endif
}

/**
 * バランス走行に必要なものをリセットする
 */
void Ang_Robo::init() {
    int offset = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    // モータエンコーダをリセットする
    mLeftWheel.reset();
    mRightWheel.reset();

    // 倒立振子制御初期化
    mBalancer->init(offset);
    balance_mode = true;

    gTail_pwm->init_pid(0.1, 0.01, 0.01, dT_4ms);

    //    tail_reset();
    //    tail_upright();
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void Ang_Robo::setCommand(int forward, float yawratecmd, signed int anglecommand, float yawrate, bool tail_mode_lflag) {
    mForward = forward;
    mYawratecmd    = yawratecmd;
    mAngleCommand= anglecommand;
    mYawrate = yawrate;
	mTailModeFlag = tail_mode_lflag;//0816

    mmForward = mForward;
    mmTurn = mYawratecmd;
    mmYawratecmd = mAngleCommand;//目標Yawrate
    mmYawrate = mYawrate;
}

//2017.07.28 k-ota copy from 3-apex
//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void Ang_Robo::tail_control(signed int angle)
{

  //  pwm = (float)(angle - mTail_Motor.getCount()*P_GAIN); /* 比例制御 */
  pwm = gTail_pwm->calc_pid(angle, mTail_Motor.getCount());
  pwm = pwm*0.1;
  /* PWM出力飽和処理 */
  if (pwm > PWM_ABS_MAX)
    {
      pwm = PWM_ABS_MAX;
    }
  else if (pwm < -PWM_ABS_MAX)
    {
      pwm = -PWM_ABS_MAX;
    }

  if (pwm == 0)
    {
      //17.07.28 kota modify//        ev3_motor_stop(tail_motor, true);
      mTail_Motor.stop();
    }
  else
    {
      //17.07.28 kota modify//        ev3_motor_set_power(tail_motor, (signed char)pwm);
      mTail_Motor.setPWM((signed int)pwm);
    }
  angle2_eo1 = 0.0;
  angle2_eo2 = 0.0;
  pwm_o = pwm; /* angle2用変数型初期化 */

}

//170816 ota add tail control

void Ang_Robo::tail_reset(){
  int32_t angle    = 0;
  int32_t angle_1d = 0;

  mTail_Motor.setPWM(-10);
  angle = 0;
  angle_1d = 1;

  while(1){
    if(angle == angle_1d){
      mTail_Motor.stop();
      mTail_Motor.reset();
      break;
    }
    else{
      angle_1d = angle;
      tslp_tsk(1000);
      angle = mTail_Motor.getCount();
    }
  }
  mTail_Motor.stop();
  mTail_Motor.reset();
}

void Ang_Robo::tail_stand_up(){
    while(1){
      if(mTail_Motor.getCount() == TAIL_ANGLE_STAND_UP){
	mTail_Motor.stop();
	break;
      }
      else{
	mTail_Motor.setPWM(5);
      }
    }
    mTail_Motor.stop();
} //tail for gyro reset and color sensor calibration

//2017/08/06多田さんヨーレートコントローラー
float Ang_Robo::YawrateController(float yawrate, float yawrate_cmd)
{
	r_yaw_rate = yawrate_cmd*I_gain1 - r_yaw_rate_ud;
	r_yaw_rate_ud =(r_yaw_rate*I_gain2 + r_yaw_rate_ud*I_gain3);
	F_out = F_controller((float)r_yaw_rate);
	E_out = E_controller((float)r_yaw_rate);
	C_out = C_controller(E_out, (float)yawrate, S_out);
	S_out = S_controller(C_out);
	turn_tmp = F_out + C_out;
	if(turn_tmp > 100) turn_tmp = 100;
	if(turn_tmp < -100) turn_tmp = -100;


	return turn_tmp;//制御出力

}

float Ang_Robo::F_controller(float r_yaw_rate)
{
	F_in = r_yaw_rate;

	F_out = F_in * F_gain;

	return F_out;
}

float Ang_Robo::E_controller(float r_yaw_rate)
{
	E_in_dddddd = E_in_ddddd;
	E_in_ddddd = E_in_dddd;
	E_in_dddd = E_in_ddd;
	E_in_ddd = E_in_dd;
	E_in_dd = E_in_d;
	E_in_d = E_in;
	E_in = (r_yaw_rate);
	E_out = E_ud1;
	E_ud1 = (E_in_dddddd * E_gain1) + E_ud1 * E_gain2;

	return E_out;
}

float Ang_Robo::C_controller(float E_out, float yawrate, float S_out)
{
//	C_in = (E_out - yawrate - S_out);
	C_in = (E_out + yawrate - S_out);//0816
//	C_in = (E_out - yawrate);
	C_out = C_ud1;
	C_ud1 = C_in * C_gain + (C_out * 1.0);


	return C_out;
}

float Ang_Robo::S_controller(float C_out)
{
	S_in_dddddd = S_in_ddddd;
	S_in_ddddd = S_in_dddd;
	S_in_dddd = S_in_ddd;
	S_in_ddd = S_in_dd;
	S_in_dd = S_in_d;
	S_in_d = S_in;
	S_in = C_out*S_gain1;
	S_out = Pn_ud1 - Pd_ud1;
	Pn_ud1 = S_in*Pn_gain1 + Pn_ud1*Pn_gain2;
	Pd_ud1 = S_in_dddddd*Pd_gain1 + Pd_ud1*Pd_gain2;

	return S_out;
}

void Ang_Robo::TailMode(int mForward, float mTurn){
	mtail_mode_pwm_l = 0.5*mForward + 1.0*mTurn ;
	mtail_mode_pwm_r = 0.5*mForward - 1.0*mTurn;
}

void Ang_Robo::saveData(int idata_num){
#ifdef DEBUG_ROBO

	dsave_log1.push_front(0);
	dsave_log2.push_front(0);
	dsave_log3.push_front(0);
	dsave_log4.push_front(0);
	dsave_log5.push_front(0);
	dsave_log6.push_front(0);
	dsave_log7.push_front(0);
	dsave_log8.push_front(0);
	dsave_log9.push_front(0);
	dsave_log10.push_front(0);

    int current_size = (int)dsave_log1.size();

    if(current_size > idata_num){
    	dsave_log1.pop_front();
    	dsave_log2.pop_front();
    	dsave_log3.pop_front();
    	dsave_log4.pop_front();
    	dsave_log5.pop_front();
    	dsave_log6.pop_front();
    	dsave_log7.pop_front();
    	dsave_log8.pop_front();
    	dsave_log9.pop_front();
    	dsave_log10.pop_front();
    }
#endif

}

void Ang_Robo::exportRobo(char *csv_header){

#ifdef DEBUG_ROBO

    FILE* file_id;
    char csv_file_name[100];
    char csv_file_name_tmp[] = "Robo_Data.csv";
    sprintf(csv_file_name, "%s_%s", csv_header, csv_file_name_tmp);
    file_id = fopen( csv_file_name ,"w");

    fprintf(file_id, "log1, log2, log3, log4, log5, log6, log7, log8, log9, log10  \n");

    int cnt_max = (int)dsave_log1.size()-1;
    int cnt;
    int cnt2;
    //
    for(cnt2 = 0; cnt2 <= cnt_max ; cnt2++){
        cnt = cnt_max - cnt2;

        //fprintf(file_id, "%f, %f, %f, %f, %f, %f, %f  \n", dsave_cpu_time[cnt], dsave_data_target[cnt], dsave_data_measured[cnt], dsave_diff_for_kp[cnt], dsave_diff_for_ki[cnt],dsave_diff_for_kd[cnt], dsave_res_pid[cnt]);
        fprintf(file_id, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f  \n",
        		dsave_log1[cnt], dsave_log2[cnt], dsave_log3[cnt], dsave_log4[cnt],
				dsave_log5[cnt], dsave_log6[cnt], dsave_log7[cnt], dsave_log8[cnt], dsave_log9[cnt], dsave_log10[cnt]);
    }


    fclose(file_id);
#endif

}
