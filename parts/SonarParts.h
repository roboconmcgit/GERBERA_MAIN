/*
 *      ソナーセンサバーツクラスのヘッダファイル
 */

#ifndef SONARPARTS_H_
#define SONARPARTS_H_
#include "ev3api.h"
#include "SonarSensor.h"

using namespace ev3api;

// キャリブレーションパラメータ
typedef struct SonarParts_Calibra{
    int32_t Offset;
}SONARPARTS_CALIBRA;

#define SONAR_FILLTER_NUM           (5)  //フィルタ回数

/*
 *  関数のプロトタイプ宣言
 */
 
class SonarParts{
private:
    SonarSensor    sonarSensor;              //SonarParts用のクラス

    SONARPARTS_CALIBRA SonarParts_Calib;    //キャリブレーションデータ
    bool SonarParts_State;                  //状態フラグ　0:正常、1:以上
    int   sonar_counter = 0;
    int32_t Fillter_Data[SONAR_FILLTER_NUM];                  //フィルタ用格納データ
//protected:

public:
    int16_t sonarDistance = 0; // 距離 [cm]
    bool  sonar_stop  = false;

    SonarParts();                       //コンストラクタ
    ~SonarParts();                      //デストラクタ
    void SonarPartsCalibra(SONARPARTS_CALIBRA s_calibra);
                                        //キャリブレーションを設定する
    void SonarPartsReset();
                                        //ソナーセンサパーツをリセットする
    void SonarPartsTask();
                                        //フィルタリング用の定期処理
    int16_t GetSonarPartsData();
                                        //ソナーセンサセンサの現在値を取得する
    int16_t GetSonarPartsFillter();
                                        //ソナーセンサ値をフィルタリングした値を取得する
    bool GetObstacleDetection();
                                        //障害物を検知を通知する
};

#endif // !SONARPARTS_H_
