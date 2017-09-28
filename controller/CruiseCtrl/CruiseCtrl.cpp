/**
 ******************************************************************************
 ** ファイル名 : CruiseCtrl.cpp
 **
 ** 概要 : 走行制御クラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "CruiseCtrl.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

// 定数宣言
const int CruiseCtrl::LOW    = 30;    // 低速
const int CruiseCtrl::NORMAL = 50;    // 通常
const int CruiseCtrl::HIGH   = 70;    // 高速

#define liting_radius 10; // liting spot radius [mm]

//*****************************************************************************
// 関数名 : コンストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
CruiseCtrl::CruiseCtrl(
    GyroParts* gyro,
    MotorParts* motor,
    Balancer* balancer)
: mGyroParts(gyro),
mMotorParts(motor),
mBalancer(balancer),
mForward(LOW),
mTurn(LOW)
{
}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
CruiseCtrl::~CruiseCtrl(){
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : バランス走行に必要なものをリセットする
//*****************************************************************************
void CruiseCtrl::init() {
	offset = mGyroParts->GetGyroPartsData();  // ジャイロセンサ値
    
  robo_Clock       = new Clock();        

  mMotorParts->MotorPartsReset(MOTORPARTS_LEFT_BIT&MOTORPARTS_RIGHT_BIT);
  
  mBalancer->init(offset);
  balance_mode = true; 
  lug_mode     = false;

  Tail_Mode = Ang_Balance;

  Stand_Mode = Balance_Mode;
}

//*****************************************************************************
// 関数名 : 
// 引数 :  @param forward 前進値
//         @param turn    旋回値
// 返り値 : なし
// 概要 : PWM値を設定する
//*****************************************************************************
void CruiseCtrl::setCommand(int forward, float yawratecmd, signed int tail_ang_req, float yawrate, bool tail_stand_mode, bool tail_lug_mode) {
		
	mForward         = forward;
	mYawratecmd      = yawratecmd;
	mTail_ang_req    = tail_ang_req;
	mYawrate         = yawrate;
	mTail_stand_mode = tail_stand_mode;
	mTail_lug_mode   = tail_lug_mode;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : バランス走行する
//*****************************************************************************
void CruiseCtrl::CruiseCtrlOperation() {
	int16_t angle = mGyroParts->GetGyroPartsData();  // ジャイロセンサ値
	int rightWheelEnc = mMotorParts->getMotorPartsPwm(MOTORPARTS_RIGHT_NO);       // 右モータ回転角度
	int leftWheelEnc  = mMotorParts->getMotorPartsPwm(MOTORPARTS_LEFT_NO);        // 左モータ回転角度
  
	//アクティブヨーレート();
	mTurn = YawrateController(mYawrate, mYawratecmd);

	if(mTail_stand_mode == true){
		tail_stand_from_balance();
	  }else if((mTail_stand_mode == false) && (Stand_Mode != Balance_Mode)){
		if(Stand_Mode == Tail_Stand){
			Stand_Mode = Stand_Vert;
		}
		tail_stand_from_balance();
	}else{
		mMotorParts->tail_control(mTail_ang_req);
		balance_off_en = false;
	}
  
	int battery = ev3_battery_voltage_mV();
	if((Stand_Mode == Stand_to_Balance)&&(log_left_pwm < -10)){
		mBalancer->setCommand(mForward, mTurn);
		mBalancer->update(10, rightWheelEnc, leftWheelEnc, battery);
	}else if((Stand_Mode == Stand_to_Balance)&&(log_left_pwm > 10)){
		mBalancer->setCommand(mForward, mTurn);
		mBalancer->update(-10, rightWheelEnc, leftWheelEnc, battery);
	}else{
		mBalancer->setCommand(mForward, mTurn);
		mBalancer->update(angle, rightWheelEnc, leftWheelEnc, battery);
	}

	log_forward         = mForward;
	log_turn            = mTurn;
	log_gyro            = angle;
	log_left_wheel_enc  = leftWheelEnc;
	log_right_wheel_enc = rightWheelEnc;
	log_battery         = battery;
	log_left_pwm        = mBalancer->getPwmLeft();
	log_right_pwm       = mBalancer->getPwmRight();

    if( ((balance_off_en == true) && (mMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) >  70)) || (Stand_Mode == Lug_to_Stand) ){
		TailMode(mForward, mTurn);
		mMotorParts->setMotorPartsLeftRight(mtail_mode_pwm_r,mtail_mode_pwm_l);
		balance_mode = false; 
		//    }else if((mTail_lug_mode == true) && (balance_off_en == true) && (balance_mode = false) ){
	  }else if(mTail_lug_mode == true){
		TailMode(mForward, mTurn);
		mMotorParts->setMotorPartsLeftRight(mtail_mode_pwm_r,mtail_mode_pwm_l);
		balance_mode = false; 
	}else{
		mMotorParts->setMotorPartsLeftRight(mBalancer->getPwmRight(),mBalancer->getPwmLeft());
		balance_mode = true;

	}
  }

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void CruiseCtrl::tail_stand_from_balance(){
	static float   target_tail_angle;
	static int32_t clock_start;
  
	switch(Stand_Mode){
	case Balance_Mode:
	  mForward = 0;
	  mTurn = 0;
	  target_tail_angle =  TAIL_ANGLE_RUN;
	  Stand_Mode = Tail_Down;
	  balance_off_en = false;
	  pre_balancer_on = false;
	  break;
  
	case Tail_Down:
	  mForward = 0;
	  mTurn = 0;
	  if(target_tail_angle <= TAIL_ANGLE_DANSA){
		target_tail_angle = target_tail_angle + 0.5;
	  }
	  if(mMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) >= TAIL_ANGLE_DANSA){
		Stand_Mode = Tail_On;
		clock_start = robo_Clock->now();
	  }
	  mMotorParts->tail_control(target_tail_angle);
	  balance_off_en = false;
	  break;
  
	case Tail_On:
	  mForward = 0;
	  mTurn = 0;
	  //    if((robo_Clock->now() - clock_start) < 5000){
	  if((robo_Clock->now() - clock_start) < 3500){
		mForward = 0;
		mTurn = 0;
		balance_off_en = false;
	  }
	  //    else if((robo_Clock->now() - clock_start) > 7000){
	  else if((robo_Clock->now() - clock_start) > 5000){
		mForward = 0;
		mTurn = 0;
		balance_off_en = true;
		Stand_Mode = Tail_Stand;
		clock_start = robo_Clock->now();
	  }else{
		mForward = -20;
		mTurn = 0;
		balance_off_en = false;
	  }
	  break;
  
	case Tail_Stand:
    //tail_control(TAIL_ANGLE_DANSA);
    lug_mode     = false;
	  if((robo_Clock->now() - clock_start) < 500){
		mForward = 0;
		mTurn = 0;
		balance_off_en = true;
	  }else{
		balance_off_en = true;
	  }
  
  
	  if(mTail_lug_mode == true){
		balance_off_en = true;
		Stand_Mode = Tail_Lug;
	  }
	  break;
  
	case Tail_Lug:
    balance_off_en = true;
  
    if(mMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) <= TAIL_ANGLE_LUG){
		clock_start = robo_Clock->now();
		mMotorParts->tail_control(TAIL_ANGLE_LUG);
		mMotorParts->BrakeMotorPartsTail(true);
		lug_mode     = true;
	  }else{
		target_tail_angle = target_tail_angle - 0.05;
		mMotorParts->tail_control(target_tail_angle);
	  }
  
	  if(mTail_lug_mode == false){
		Stand_Mode = Lug_to_Stand;
		mMotorParts->BrakeMotorPartsTail(false);
	  }
  
	  break;
  
	case Lug_to_Stand:
	  balance_off_en = true;
	  mTurn = 0;
  
	  if(mMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) < 75){
		mForward = -100;
		mMotorParts->setMotorPartsTailPwm(100);
	  }else{
		mForward = 0;
		mMotorParts->tail_control(TAIL_ANGLE_DANSA);
		target_tail_angle = TAIL_ANGLE_DANSA;
		if(mMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) >= TAIL_ANGLE_DANSA){
	  mMotorParts->BrakeMotorPartsTail(true);
	  mMotorParts->setMotorPartsTailPwm(0);
	  Stand_Mode = Tail_Stand;
	  clock_start = robo_Clock->now();
		}
	  }
	  break;
  
	case Stand_Vert:
	  mForward = 0;
	  mTurn    = 0;
	  balance_off_en = true;
	  mMotorParts->BrakeMotorPartsTail(false);
	  if(mMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) >= 95){
		mMotorParts->tail_control(96);
		clock_start = robo_Clock->now();
		Stand_Mode = Stand_to_Balance;
	  }
  
	  if(mMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) < 96){
		target_tail_angle = target_tail_angle + 0.02;
		mMotorParts->tail_control(target_tail_angle);
	  }else{
		mMotorParts->tail_control(96);
		clock_start = robo_Clock->now();
		Stand_Mode = Stand_to_Balance;
	  }
	  break;
  
	case Stand_to_Balance:
	  mForward = 0;
	  mTurn    = 0;
	  balance_off_en = true;
	  mMotorParts->tail_control(96);
  
	  if((robo_Clock->now() - clock_start) > 1000){
		//    if((log_left_pwm >= -20) && (log_left_pwm <= 20) && ((robo_Clock->now() - clock_start) > 1000)){
		if((log_left_pwm >= -10) && (log_left_pwm <= 10)){
	  mForward       = 0;
	  mTurn          = 0;
	  //	balance_off_en = false;
	  balance_off_en = true;
	  Stand_Mode     = Tail_for_Run;
	  mMotorParts->tail_control(96);
		}
	  }
	  break;
  
	case Tail_for_Run:
	  mForward = 0;
	  mTurn    = 0;
	  mMotorParts->tail_control(98);
	  balance_off_en = true;
  
	  if(mMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) >=  97){
		Stand_Mode     = Balance_Mode;
		balance_off_en = false;
	  }
	  break;
	default:
	  mForward = 0;
	  mTurn = 0;
	  balance_off_en = false;
	  break;
	}
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
//2017/08/06多田さんヨーレートコントローラー
float CruiseCtrl::YawrateController(float yawrate, float yawrate_cmd)
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

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
float CruiseCtrl::F_controller(float r_yaw_rate)
{
	F_in = r_yaw_rate;

	F_out = F_in * F_gain;

	return F_out;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
float CruiseCtrl::E_controller(float r_yaw_rate)
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

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
float CruiseCtrl::C_controller(float E_out, float yawrate, float S_out)
{
//	C_in = (E_out - yawrate - S_out);
	C_in = (E_out + yawrate - S_out);//0816
//	C_in = (E_out - yawrate);
	C_out = C_ud1;
	C_ud1 = C_in * C_gain + (C_out * 1.0);


	return C_out;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
float CruiseCtrl::S_controller(float C_out)
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

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void CruiseCtrl::TailMode(int mForward, float mTurn){
	mtail_mode_pwm_l = 0.5*mForward + 1.0*mTurn ;
	mtail_mode_pwm_r = 0.5*mForward - 1.0*mTurn;
}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
//17.07.31 k-tomii add for position estimation
//ライントレースプログラム
float CruiseCtrl::LineTracer(int line_value,float traceforward) {
	
	const int LineTraceCommand=80;			//目標ライン値
	const float KP=0.5,KI=0.005,KD=0.05;			//PIDゲインの設定

	static int error_old=0,error_P_old=0;		//過去の偏差
	static float u=0;							//制御入力

	float u_delta=0;							//制御入力の差分
	int error=0,error_P=0,error_I=0,error_D=0;	//偏差
	float u_P_delta=0,u_I_delta=0,u_D_delta=0;	//制御入力の差分

	error=LineTraceCommand-(line_value);			//制御偏差を計算
	error_P=error-error_old;					//P制御用の偏差を計算
	error_I=error;								//I制御用の偏差を計算
	error_D=error_P-error_P_old;				//D制御用の偏差を計算

	u_P_delta=KP*error_P;						//P制御用の入力差分を計算
	u_I_delta=KI*error_I;						//I制御用の入力差分を計算
	u_D_delta=KD*error_D;						//D制御用の入力差分を計算

	u_delta=u_P_delta+u_I_delta+u_D_delta;		//PID制御入力の差分を計算
	u=u+u_delta;								//制御入力を計算

	//入力制限
	if(u>10){
		u=10;
	}else if(u<-10){
		u=-10;
	}

	return u;										//目標turn値を更新
//	forward=traceforward;						//目標forward値を更新

//	error_old=error;							//過去の偏差を保存
//	error_P_old=error_P;						//過去の偏差を保存

}

//*****************************************************************************
// 関数名 : 
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
//2017/08/06多田さんライントレーサー
float CruiseCtrl::LineTracerYawrate(int line_value, float Max_Yawrate, float Min_Yawrate) {

	y_t = -1.0*(((float)line_value-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
	if(y_t > 10.0) y_t = 10.0;
	if(y_t < -10.0) y_t = -10.0;
	y_t = y_t + 7.0*(y_t/8.0)*(y_t/8.0)*(y_t/8.0);
//    yawratecmd = y_t/4.0;
	float Yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));

	if(Yawratecmd > Max_Yawrate){
		Yawratecmd =  Max_Yawrate;

	}else if (Yawratecmd < Min_Yawrate){
		Yawratecmd = Min_Yawrate;
	}else{
	}

	y_t_prev = y_t;
	return(Yawratecmd);
}
