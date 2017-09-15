/*
 *      カラーセンサバーツクラスのヘッダファイル
 */

#ifndef COLORPARTS_H_
#define COLORPARTS_H_

#include "ev3api.h"
#include "parameter.h"
#include "ColorSensor.h"

using namespace ev3api;

// キャリブレーションパラメータ
typedef struct ColorParts_Calibra{
    //int32_t Offset;
    int8_t white;
    int8_t black;
    int8_t white_slant;
    int8_t black_slant;
}COLORPARTS_CALIBRA;

#define COLOR_FILLTER_NUM           (5)  //フィルタ回数

/*
 *  関数のプロトタイプ宣言
 */
 
class ColorParts{
private:
    ColorSensor     colorSensor;              //ColorParts用のクラス

    COLORPARTS_CALIBRA ColorParts_Calib;    //キャリブレーションデータ
    bool ColorParts_State;                  //状態フラグ　0:正常、1:以上
    int8_t LineEdge_Data;                   //ライン際の色情報
    int8_t Fillter_Data[COLOR_FILLTER_NUM];                  //フィルタ用格納データ

    int   line_dat_500ms[125]; //detecting for gray line
    int   max_line_dat;
    int   min_line_dat;
    
    int   cap_cnt = 0;
    
//protected:

public:
    int   linevalue = 0;//ライン値
    int   linevalue_LUG = 0;//ライン値
    bool  det_gray  = 0;

    ColorParts();                       //コンストラクタ
    ~ColorParts();                      //デストラクタ
    void ColorPartsCalibra(COLORPARTS_CALIBRA s_calibra);
                                        //キャリブレーションを設定する
    void ColorPartsReset();
                                        //カラーセンサパーツをリセットする
    void ColorPartsTask();
                                        //フィルタリング用の定期処理
    int8_t GetColorPartsData();
                                        //カラーセンサセンサの現在値を取得する
    int8_t GetColorPartsFillter();
                                        //カラーセンサ値をフィルタリングした値を取得する
    bool GetLineDetection();
                                        //ラインを検知を通知する
};

#endif // !COLORPARTS_H_
