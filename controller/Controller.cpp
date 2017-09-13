/**
 ******************************************************************************
 ** ファイル名 : Controller.cpp
 **
 ** 概要 : コントローラクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "Controller.h"

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
Controller::Controller(
	ColorParts  *Color,
	MotorParts  *Motor,
	GyroParts   *Gyro,
	SonarParts  *Sonar,
	TouchParts  *Touch):
	gColorParts(Color),
    gMotorParts(Motor),
    gGyroParts(Gyro),
    gSonarParts(Sonar),
    gTouchParts(Touch)
	{
	mC_Cruise = new Cruise();
}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
Controller::~Controller(){

}

//*****************************************************************************
// 関数名 : ControllerCalibra
// 引数 : Controller_CALIBRA s_calibra
// 返り値 : なし
// 概要 : 全てのモータのキャリブレーションを設定する
//*****************************************************************************
void Controller::ControllerOperation(void){

}

