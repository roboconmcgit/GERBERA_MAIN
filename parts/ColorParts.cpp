/**
 ******************************************************************************
 ** ファイル名 : ColorParts.cpp
 **
 ** 概要 : カラーセンサパーツクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "ColorParts.h"
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
ColorParts::ColorParts():colorSensor(COLORPARTS_CH)
{
	memset(&ColorParts_Calib, 0, sizeof(COLORPARTS_CALIBRA));
	memset(&Fillter_Data[0], 0, sizeof(COLOR_FILLTER_NUM));
	
	ColorParts_Calib.white=40;
	ColorParts_Calib.black=20;
	ColorParts_Calib.white_slant=40;
	ColorParts_Calib.black_slant=20;

	ColorParts_State = false;
}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
ColorParts::~ColorParts(){
	//delete colorSensor;
}

//*****************************************************************************
// 関数名 : ColorPartsCalibra
// 引数 : ColorParts_CALIBRA s_calibra
// 返り値 : なし
// 概要 : キャリブレーションを設定する
//*****************************************************************************
void ColorParts::ColorPartsCalibra(COLORPARTS_CALIBRA s_calibra){
	memcpy(&ColorParts_Calib, &s_calibra, sizeof(COLORPARTS_CALIBRA));
}

//*****************************************************************************
// 関数名 : ColorPartsReset
// 引数 : unused
// 返り値 : なし
// 概要 : カラーセンサパーツをリセットする
//*****************************************************************************
void ColorParts::ColorPartsReset(){
	int set = GetColorPartsData();
	memset(&Fillter_Data[0], set, sizeof(COLOR_FILLTER_NUM));
}

//*****************************************************************************
// 関数名 : ColorPartsTask
// 引数 : unused
// 返り値 : なし
// 概要 : フィルタリング用の定期処理
//*****************************************************************************
void ColorParts::ColorPartsTask(){
	//定期的にデータ取得
	float k,k_lug;
	float adj_brightness,adj_brightness_lug;
	int i;

	//Fillter=============================================================================
	for(int i = 0;i < COLOR_FILLTER_NUM - 1; i++){
		Fillter_Data[i+1] = Fillter_Data[i];
	}
	Fillter_Data[0] = GetColorPartsData();

	// Line Value Calc=============================================================================
	k= 100.0/(ColorParts_Calib.white-ColorParts_Calib.black);
	k_lug= 100.0/(ColorParts_Calib.white_slant-ColorParts_Calib.black_slant);
	adj_brightness = k*(GetColorPartsData()-ColorParts_Calib.black);
	adj_brightness_lug = k_lug*(GetColorPartsData()-ColorParts_Calib.black_slant);
  
	if(adj_brightness < 0){
	  adj_brightness = 0;
	}
	else if(adj_brightness > 100){
	  adj_brightness = 100;
	}
	linevalue = 100-adj_brightness;
  
	if(adj_brightness_lug < 0){
		adj_brightness_lug = 0;
	}
	else if(adj_brightness_lug > 100){
		adj_brightness_lug = 100;
	}
	linevalue_LUG = 100-adj_brightness_lug;

	//Gray Check=============================================================================
	if(cap_cnt == 125){
		cap_cnt = 0;
    }
	line_dat_500ms[cap_cnt]     = linevalue;
	cap_cnt++;
	max_line_dat     = 0;
	min_line_dat     = 100;

	for (i=0; i<125; i++){
	  if(line_dat_500ms[i] > max_line_dat){
	max_line_dat = line_dat_500ms[i];
	  }
	  if(line_dat_500ms[i] < min_line_dat){
	min_line_dat = line_dat_500ms[i];
	  }

	}
	if((max_line_dat - min_line_dat) < 10){
		 det_gray  = 1;
	}else{
		 det_gray  = 0;
	}
}

//*****************************************************************************
// 関数名 : GetColorPartsData
// 引数 : unused
// 返り値 : 現在のカラーセンサ値
// 概要 : カラーセンサセンサの現在値を取得する
//*****************************************************************************
int8_t ColorParts::GetColorPartsData(){
	int8_t ret = colorSensor.getBrightness();
	return(ret);
}

//*****************************************************************************
// 関数名 : GetColorPartsFillter
// 引数 : unused
// 返り値 : フィルタリングしたカラーセンサ値
// 概要 : カラーセンサ値をフィルタリングした値を取得する
//*****************************************************************************
int8_t ColorParts::GetColorPartsFillter(){
	int16_t sum = 0;
	int8_t ret;	
	for(int i = 0;i < COLOR_FILLTER_NUM; i++){
		sum += Fillter_Data[i];
	}
	ret = (int8_t)(sum / COLOR_FILLTER_NUM);
	return(ret);
}

//*****************************************************************************
// 関数名 : GetLineDetection
// 引数 : unused
// 返り値 : ライン検知有無
// 概要 : ラインを検知を通知する
//*****************************************************************************
bool ColorParts::GetLineDetection(){
	bool ret = false;
    if(GetColorPartsFillter() >= LineEdge_Data){
		ret = true;
	}
	return(ret);
}
