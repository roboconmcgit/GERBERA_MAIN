/**
 ******************************************************************************
 ** ファイル名 : MotorParts.cpp
 **
 ** 概要 : モーターバーツクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "MotorParts.h"
#include "parameter.h"
#include <string.h>
#include "math.h"

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
MotorParts::MotorParts():
	leftMotor(LEFT_MOTOR_CH),
	rightMotor(RIGHT_MOTOR_CH),
	tailMotor(TAIL_MOTOR_CH),
	angle2_e(0),   /* angle2用変数型宣言 */
	angle2_eo1(0), /* angle2用変数型宣言 */
	angle2_eo2(0), /* angle2用変数型宣言 */
	pwm_o(0),      /* angle2用変数型宣言 */
	pwm2(0),
	pwm(0)
{
	//memset((void *)&MotorParts_Calib, 0, sizeof(MOTORPARTS_CALIBRA));
	memset((void *)&MotorParts_State, 0, MOTORPARTS_NUM);
	
  enum_Mode = CALIB_ANGLE;
	#ifdef DEBUG_NANSYO
		enum_Mode = DET_MOVEMENT;
	#endif
		robo_stop       = 1;
		robo_forward    = 0;
		robo_back       = 0;
		robo_turn_left  = 0;
		robo_turn_right = 0;
		
		// Tail Control
    gTail_pwm->init_pid(0.1, 0.01, 0.01, dT_4ms);
		
}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
MotorParts::~MotorParts(){
	//delete leftMotor;
	//delete rightMotor;
	//delete tailMotor;
}

//*****************************************************************************
// 関数名 : MotorPartsCalibra
// 引数 : MOTORPARTS_CALIBRA s_calibra
// 返り値 : なし
// 概要 : 全てのモータのキャリブレーションを設定する
//*****************************************************************************
void MotorParts::MotorPartsCalibra(MOTORPARTS_CALIBRA s_calibra){
	//memcpy(&MotorParts_Calib, &s_calibra, sizeof(MOTORPARTS_CALIBRA));
}

//*****************************************************************************
// 関数名 : MotorPartsCalibra
// 引数 : int8_t motorBit
// 返り値 : なし
// 概要 : 指定したモータをリセットする
//*****************************************************************************
void MotorParts::MotorPartsReset(int8_t motorBit){
	if((motorBit&MOTORPARTS_LEFT_BIT)>0){
		leftMotor.reset();
	}
	if((motorBit&MOTORPARTS_RIGHT_BIT)>0){
		rightMotor.reset();
	}
	if((motorBit&MOTORPARTS_TAIL_BIT)>0){
		tailMotor.reset();
	}
}

//*****************************************************************************
// 関数名 : getMotorPartsPwm
// 引数 : int8_t motorNo
// 返り値 : なし
// 概要 : 指定したモータの現在値を取得する
//*****************************************************************************
int32_t MotorParts::getMotorPartsPwm(int8_t motorNo){
	int32_t ret = 0;
	switch(motorNo){
		case MOTORPARTS_LEFT_NO:
			ret = leftMotor.getCount();
			break;
		case MOTORPARTS_RIGHT_NO:
			ret = rightMotor.getCount();
			break;
		case MOTORPARTS_TAIL_NO:
			ret = tailMotor.getCount();
			break;
		default:
			break;
	}

	return(ret);
}

//*****************************************************************************
// 関数名 : setMotorPartsLeftRight
// 引数 : int rightMotorPwm, int leftMotorPwm
// 返り値 : なし
// 概要 : 左右モータを動かす
//*****************************************************************************
void MotorParts::setMotorPartsLeftRight(int rightMotorPwm, int leftMotorPwm){
	int pwm;
	pwm = rightMotorPwm;
    rightMotor.setPWM(pwm);
	pwm = leftMotorPwm;
    leftMotor.setPWM(pwm);
}

//*****************************************************************************
// 関数名 : stopMotorPartsLeftRight
// 引数 : unused
// 返り値 : なし
// 概要 : 左右モータを止める
//*****************************************************************************
void MotorParts::stopMotorPartsLeftRight(void){
    rightMotor.stop();
    leftMotor.stop();
}

//*****************************************************************************
// 関数名 : setMotorPartsTail
// 引数 : int8_t tailMove
// 返り値 : なし
// 概要 : テールモータを動かす
//*****************************************************************************
void MotorParts::setMotorPartsTail(int8_t tailMove){
	int32_t angle = 0;
	float pwm = 0;

	switch(tailMove){
		case TAIL_RESET:
			angle = 0;
		break;
		case TAIL_STAND_UP:
			angle = TAIL_ANGLE_STAND_UP;
			break;
		case TAIL_DRIVE:
			angle = TAIL_ANGLE_DRIVE;
			break;
		default:
			break;
	}

    pwm = (float)(angle - tailMotor.getCount()) * P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }
    tailMotor.setPWM(pwm);
}

//2017.07.28 k-ota copy from 3-apex
//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void MotorParts::tail_control(signed int angle)
{
  //  pwm = (float)(angle - tailMotor.getCount()*P_GAIN); /* 比例制御 */
  pwm = gTail_pwm->calc_pid(angle, tailMotor.getCount());
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
      tailMotor.stop();
    }
  else
    {
      //17.07.28 kota modify//        ev3_motor_set_power(tail_motor, (signed char)pwm);
      tailMotor.setPWM((signed int)pwm);
    }
  angle2_eo1 = 0.0;
  angle2_eo2 = 0.0;
  pwm_o = pwm; /* angle2用変数型初期化 */

}

//170816 ota add tail control
//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void MotorParts::tail_reset(){
  int32_t angle    = 0;
  int32_t angle_1d = 0;

  tailMotor.setPWM(-10);
  angle = 0;
  angle_1d = 1;

  while(1){
    if(angle == angle_1d){
      tailMotor.stop();
      tailMotor.reset();
      break;
    }
    else{
      angle_1d = angle;
      tslp_tsk(1000);
      angle = tailMotor.getCount();
    }
  }
  tailMotor.stop();
  tailMotor.reset();
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void MotorParts::tail_stand_up(){
    while(1){
      if(tailMotor.getCount() == TAIL_ANGLE_STAND_UP){
				tailMotor.stop();
				break;
      }
      else{
				tailMotor.setPWM(5);
      }
    }
    tailMotor.stop();
} //tail for gyro reset and color sensor calibration

//*****************************************************************************
// 関数名 : WheelOdometry
// 引数 : 
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void MotorParts::WheelOdometry(float dT) {
  static float odo_prev;
  static float velocity_input;
  static float velocity_prev;
  //LPF 10[rad/s]/////////////////////////////////////
  static float Alpfd = 0.9391; // LPF
  static float Blpfd = 1; // LPF
  static float Clpfd = 0.0609; // LPF
  static float Dlpfd = 0; // LPF
  //////////////////////////////////////////
  static float old_rel_angle;     //過去のYaw角[rad]
  int   WheelAngRdeg = rightMotor.getCount();  //右モータ回転角度[deg]
  int   WheelAngLdeg = leftMotor.getCount();   //右モータ回転角度[deg]
  int i;

  odo            = ((float)WheelAngLdeg + (float)WheelAngRdeg)/2.0 * RAD_1_DEG * WHEEL_R; //[mm]
  velocity_input = (odo - odo_prev)/dT;
  velocity       = Clpfd * velocity_prev + Dlpfd * velocity_input;
  velocity_prev  = Alpfd * velocity_prev + Blpfd * velocity_input;
  
  xvalue = xvalue+(-1)*(odo-odo_prev)*sin(relative_angle);
  yvalue = yvalue+(odo-odo_prev)*cos(relative_angle);    

  relative_angle =  ((float)WheelAngRdeg - (float)WheelAngLdeg) * RAD_1_DEG * WHEEL_R / RoboTread; //ロボのYaw角[rad]
  relative_angle = relative_angle + correction_angle;
  abs_angle      = relative_angle + RAD_90_DEG + correction_angle;
#ifdef DEBUG_NANSYO
  abs_angle      = relative_angle;
#endif
  yawrate  =(relative_angle-old_rel_angle)/dT;           //ロボのYawレート[rad/s]

  old_rel_angle=relative_angle;         //過去のYaw角[rad]
  odo_prev = odo;

  switch(enum_Mode){
  case CALIB_ANGLE:  
    robo_stop       = 0;
    robo_forward    = 1;
    robo_back       = 0;
    robo_turn_left  = 0;
    robo_turn_right = 0;
    //Correct absolute angle
    if(odo > 500 && odo <= 1500 && cap_cnt < cap_size){
      //cap_dat[cap_cnt] = abs_angle;
      angle_sum_dat = angle_sum_dat + abs_angle;
      cap_cnt++;
    }else if(odo > 1500 && odo <= 1505){
      angle_ave_dat = angle_sum_dat/cap_cnt;
      correction_angle = RAD_90_DEG - angle_ave_dat;
      xvalue = 0.0;
      yvalue = 1505;

      enum_Mode = DET_MOVEMENT;
      angle_sum_dat = 0;
      cap_cnt = 0;
    }else{
      cap_cnt = 0;
      angle_sum_dat = 0.0;
      angle_ave_dat = 0.0;
    }
    break;

  case DET_MOVEMENT:

    if(cap_cnt == 125){
      cap_cnt = 0;
    }
    angle_dat_500ms[cap_cnt]    = abs_angle;
    velocity_dat_500ms[cap_cnt] = velocity;
    cap_cnt++;

    angle_sum_dat    = 0;    
    velocity_sum_dat = 0;

    for (i=0; i<125; i++){
      angle_sum_dat    = angle_sum_dat    + angle_dat_500ms[i];
      velocity_sum_dat = velocity_sum_dat + velocity_dat_500ms[i];

    }

    angle_ave_dat    = angle_sum_dat/125;
    velocity_ave_dat = velocity_sum_dat/125;

    dif_angle_ave_dat    = angle_ave_dat - old_angle_ave_dat;
    dif_velocity_ave_dat = velocity_ave_dat - old_velocity_ave_dat;

    old_angle_ave_dat    = angle_ave_dat;
    old_velocity_ave_dat = velocity_ave_dat;

    if (dif_angle_ave_dat > -0.001 && dif_angle_ave_dat < 0.001){

      if(velocity_ave_dat > 0){
	robo_stop       = 0;
	robo_forward    = 1;
	robo_back       = 0;
	robo_turn_left  = 0;
	robo_turn_right = 0;
      }else if (velocity_ave_dat < 0){
	robo_stop       = 0;
	robo_forward    = 0;
	robo_back       = 1;
	robo_turn_left  = 0;
	robo_turn_right = 0;
      }else{
	robo_stop       = 1;
	robo_forward    = 0;
	robo_back       = 0;
	robo_turn_left  = 0;
	robo_turn_right = 0;
      }

    }else if(dif_angle_ave_dat >= 0.001){
      robo_stop       = 0;
      robo_forward    = 0;
      robo_back       = 0;
      robo_turn_left  = 1;
      robo_turn_right = 0;
    }else if(dif_angle_ave_dat <= -0.001){
      robo_stop       = 0;
      robo_forward    = 0;
      robo_back       = 0;
      robo_turn_left  = 0;
      robo_turn_right = 1;
    }else {
      robo_stop       = 0;
      robo_forward    = 0;
      robo_back       = 0;
      robo_turn_left  = 0;
      robo_turn_right = 0;
    }
    
break;

  default:
    break;
  }

#ifdef DEBUG_EYE_DEBUG
  saveData( );
#endif

}