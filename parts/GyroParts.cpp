/**
 ******************************************************************************
 ** ファイル名 : GyroParts.cpp
 **
 ** 概要 : ジャイロパーツクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "GyroParts.h"
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
GyroParts::GyroParts():gyroSensor(GYRO_CH)
{
	memset(&GyroParts_Calib, 0, sizeof(GYROPARTS_CALIBRA));
	memset(gyro_250d, 0, 250);
	GyroParts_State = false;
}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
GyroParts::~GyroParts(){
	//delete gyroSensor;
}

//*****************************************************************************
// 関数名 : GyroPartsCalibra
// 引数 : GyroParts_CALIBRA s_calibra
// 返り値 : なし
// 概要 : 全てのモータのキャリブレーションを設定する
//*****************************************************************************
void GyroParts::GyroPartsCalibra(GYROPARTS_CALIBRA s_calibra){
	memcpy(&GyroParts_Calib, &s_calibra, sizeof(GYROPARTS_CALIBRA));
}

//*****************************************************************************
// 関数名 : GyroPartsCalibra
// 引数 : GyroParts_CALIBRA s_calibra
// 返り値 : なし
// 概要 : 全てのモータのキャリブレーションを設定する
//*****************************************************************************
void GyroParts::GyroPartsReset(){
	gyroSensor.reset();
	//gyroSensor.getAnglerVelocity();
}

//*****************************************************************************
// 関数名 : GetGyroPartsData
// 引数 : unused
// 返り値 : なし
// 概要 : ジャイロセンサの現在値を取得する
//*****************************************************************************
int32_t GyroParts::GetGyroPartsData(){
	int32_t ret = gyroSensor.getAnglerVelocity();
	return(ret);
}

//*****************************************************************************
// 関数名 : det_Dansa
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void GyroParts::det_Dansa() {
	int cnt;
	gyro_250d[0] = GetGyroPartsData();
  
	for(cnt =9; cnt > 0; cnt--){
	  gyro_250d[cnt] = gyro_250d[cnt-1];
	}
   
	for(cnt = 0; cnt < 250; cnt++){
	  if(gyro_250d[cnt] < -100 || gyro_250d[cnt] > 100){
		dansa = 1;
		cnt = 250;
	  }else{
		dansa = 0;
	  }
	}
  }