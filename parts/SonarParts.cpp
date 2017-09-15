/**
 ******************************************************************************
 ** ファイル名 : SonarParts.cpp
 **
 ** 概要 : ソナーセンサパーツクラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "SonarParts.h"
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
SonarParts::SonarParts():sonarSensor(SONARPARTS_CH)
{
	//memset(&SonarParts_Calib, 0, sizeof(SONARPARTS_CALIBRA));
	memset(&Fillter_Data[0], 0, sizeof(SONAR_FILLTER_NUM));
	
	SonarParts_State = false;
}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
SonarParts::~SonarParts(){
	//delete sonarSensor;
}

//*****************************************************************************
// 関数名 : SonarPartsCalibra
// 引数 : SonarParts_CALIBRA s_calibra
// 返り値 : なし
// 概要 : キャリブレーションを設定する
//*****************************************************************************
void SonarParts::SonarPartsCalibra(SONARPARTS_CALIBRA s_calibra){
	//memcpy(&SonarParts_Calib, &s_calibra, sizeof(SONARPARTS_CALIBRA));
}

//*****************************************************************************
// 関数名 : SonarPartsReset
// 引数 : unused
// 返り値 : なし
// 概要 : ソナーセンサパーツをリセットする
//*****************************************************************************
void SonarParts::SonarPartsReset(){
	int set = GetSonarPartsData();
	SonarParts_State = false;
	memset(&Fillter_Data[0], set, sizeof(SONAR_FILLTER_NUM));
}

//*****************************************************************************
// 関数名 : SonarPartsTask
// 引数 : unused
// 返り値 : なし
// 概要 : フィルタリング用の定期処理
//*****************************************************************************
void SonarParts::SonarPartsTask(){
	//定期的にデータ取得
    if(sonar_counter%50 == 0){
		sonarDistance = sonarSensor.getDistance();
		
		//Obstacle Check
		int16_t dSonar = sonarDistance - oldSonar;
		if((dSonar > 0)&&(oldDsonar <= 0)){
		Obstacle_cnt++;
		if(Obstacle_cnt > 2){
			Obstacle = true;
			oldDsonar = dSonar;
			Obstacle_cnt=0;
		}
		}else{
		oldDsonar = dSonar;
		Obstacle_cnt=0;
		}
		oldSonar = sonarDistance;
	  }
	sonar_counter++;
	if(sonar_counter>10000) sonar_counter = 0;
	
	for(int i = 0;i < SONAR_FILLTER_NUM - 1; i++){
		Fillter_Data[i+1] = Fillter_Data[i];
	}
	Fillter_Data[0] = sonarDistance;
	
}

//*****************************************************************************
// 関数名 : GetSonarPartsData
// 引数 : unused
// 返り値 : 現在のソナーセンサ値
// 概要 : ソナーセンサセンサの現在値を取得する
//*****************************************************************************
int16_t SonarParts::GetSonarPartsData(){
	return(sonarDistance);
}

//*****************************************************************************
// 関数名 : GetSonarPartsFillter
// 引数 : unused
// 返り値 : フィルタリングしたソナーセンサ値
// 概要 : ソナーセンサ値をフィルタリングした値を取得する
//*****************************************************************************
int16_t SonarParts::GetSonarPartsFillter(){
	double sum = 0;
	int16_t ret;
	for(int i = 0;i < SONAR_FILLTER_NUM; i++){
		sum += Fillter_Data[i];
	}
	ret = (int16_t)(sum / SONAR_FILLTER_NUM);
	return(ret);
}

//*****************************************************************************
// 関数名 : GetObstacleDetection
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
bool SonarParts::GetObstacleDetection(void)
{
    bool ret = false;
    int16_t distance = GetSonarPartsFillter();

	if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
	{
		ret = true; /* 障害物を検知 */
	}
    return (ret);
}