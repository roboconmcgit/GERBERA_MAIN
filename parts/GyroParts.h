/*
 *      ジャイロバーツクラスのヘッダファイル
 */

#ifndef GYROPARTS_H_
#define GYROPARTS_H_

#include "ev3api.h"
#include "GyroSensor.h"

using namespace ev3api;

// キャリブレーションパラメータ
typedef struct GyroParts_Calibra{
    int32_t Offset;
}GYROPARTS_CALIBRA;

#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */

/*
 *  関数のプロトタイプ宣言
 */
 
class GyroParts{
private:
    GyroSensor     gyroSensor;              //GyroParts用のクラス

    GYROPARTS_CALIBRA GyroParts_Calib;    //キャリブレーションデータ
    bool GyroParts_State;        //状態フラグ　0:正常、1:以上

//protected:

	int gyro_250d[250];
public:
    bool  dansa     = 0;
    
    GyroParts();                       //コンストラクタ
    ~GyroParts();                      //デストラクタ
    void GyroPartsCalibra(GYROPARTS_CALIBRA s_calibra);
                                        //キャリブレーションを設定する
    void GyroPartsReset();
                                        //ジャイロパーツをリセットする
    int32_t GetGyroPartsData();
                                        //ジャイロセンサの現在値を取得する
    void  det_Dansa();
                                        
};

#endif // !GYROPARTS_H_
