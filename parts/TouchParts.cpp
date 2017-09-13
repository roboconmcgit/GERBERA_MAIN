/**
 ******************************************************************************
 ** ファイル名 : TouchParts.cpp
 **
 ** 概要 : タッチセンサパーツクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "TouchParts.h"
#include "parameter.h"
#include <string.h>

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
TouchParts::TouchParts():touchSensor(TOUCHPARTS_CH)
{
	memset(&TouchParts_Calib, 0, sizeof(TOUCHPARTS_CALIBRA));
	memset(&Fillter_Data[0], 0, sizeof(TOUCH_FILLTER_NUM));
	
	TouchParts_State = false;
	IsTouch = false;
}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
TouchParts::~TouchParts(){
	//delete touchSensor;
}

//*****************************************************************************
// 関数名 : TouchPartsCalibra
// 引数 : TouchParts_CALIBRA s_calibra
// 返り値 : なし
// 概要 : キャリブレーションを設定する
//*****************************************************************************
void TouchParts::TouchPartsCalibra(TOUCHPARTS_CALIBRA s_calibra){
	memcpy(&TouchParts_Calib, &s_calibra, sizeof(TOUCHPARTS_CALIBRA));
}

//*****************************************************************************
// 関数名 : TouchPartsReset
// 引数 : unused
// 返り値 : なし
// 概要 : タッチセンサパーツをリセットする
//*****************************************************************************
void TouchParts::TouchPartsReset(){
	memset(&Fillter_Data[0], 0, sizeof(TOUCH_FILLTER_NUM));
	IsTouch = false;
	TouchParts_State = false;
}

//*****************************************************************************
// 関数名 : TouchPartsTask
// 引数 : unused
// 返り値 : なし
// 概要 : フィルタリング用の定期処理
//*****************************************************************************
void TouchParts::TouchPartsTask(){
	//定期的にデータ取得
	int i;
	for(i = 0;i < TOUCH_FILLTER_NUM - 1; i++){
		Fillter_Data[i+1] = Fillter_Data[i];
	}
	Fillter_Data[0] = GetTouchPartsData();
	for(i = 0;i < TOUCH_FILLTER_NUM; i++){
		if(Fillter_Data[i] == false){
			IsTouch = false;
			break;
		}
	}
	if(i == TOUCH_FILLTER_NUM){
		IsTouch = true;
	}
}

//*****************************************************************************
// 関数名 : GetTouchPartsData
// 引数 : unused
// 返り値 : 現在のタッチセンサ値
// 概要 : タッチセンサセンサの現在値を取得する
//*****************************************************************************
bool TouchParts::GetTouchPartsData(){
	bool ret = touchSensor.isPressed();
	return(ret);
}

//*****************************************************************************
// 関数名 : GetTouchPartsFillter
// 引数 : unused
// 返り値 : フィルタリングしたタッチセンサ値
// 概要 : タッチセンサ値をチャタリングした値を取得する
//*****************************************************************************
bool TouchParts::GetTouchPartsFillter(){
	return(IsTouch);
}

