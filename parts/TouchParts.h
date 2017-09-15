/*
 *      タッチセンサバーツクラスのヘッダファイル
 */

#ifndef TOUCHPARTS_H_
#define TOUCHPARTS_H_
#include "ev3api.h"
#include "TouchSensor.h"

using namespace ev3api;

// キャリブレーションパラメータ
typedef struct TouchParts_Calibra{
    int32_t Offset;
}TOUCHPARTS_CALIBRA;

#define TOUCH_FILLTER_NUM           (3)  //チャタリング回数

/*
 *  関数のプロトタイプ宣言
 */
 
class TouchParts{
private:
    TouchSensor    touchSensor;              //TouchParts用のクラス

    TOUCHPARTS_CALIBRA TouchParts_Calib;    //キャリブレーションデータ
    bool TouchParts_State;                  //状態フラグ　0:正常、1:以上
    bool Fillter_Data[TOUCH_FILLTER_NUM];                  //フィルタ用格納データ
    bool IsTouch;

//protected:

public:
    TouchParts();                       //コンストラクタ
    ~TouchParts();                      //デストラクタ
    void TouchPartsCalibra(TOUCHPARTS_CALIBRA s_calibra);
                                        //キャリブレーションを設定する
    void TouchPartsReset();
                                        //タッチセンサパーツをリセットする
    void TouchPartsTask();
                                        //フィルタリング用の定期処理
    bool GetTouchPartsData();
                                        //タッチセンサセンサの現在値を取得する
    bool GetTouchPartsIsTouch();
                                        //タッチセンサ値をチャタリングした値を取得する
};

#endif // !TOUCHPARTS_H_
