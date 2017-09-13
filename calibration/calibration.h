
#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "ev3api.h"
#include "Ang_Robo.h" //it will be changed to Ang_Robo
#include "ColorParts.h"
#include "TouchParts.h"


// using宣言
//using namespace ev3api;

#define RIGHT_COURCE_MODE

class calibration{
private:
    ColorParts  *gColorParts;
    TouchParts  *gTouchParts;
    MotorParts  *gMotorParts;
//protected:
//    xxx          *xxx;               //calibration用のクラス

public:
    calibration(
        ColorParts *Color,
        TouchParts *Touch,
        MotorParts *motor
    ):  gColorParts(Color),
        gTouchParts(Touch),
        gMotorParts(motor)
    {};   //コンストラクタ
    ~calibration(){};                      //デストラクタ
    int set_calibration();
                                        //キャリブレーションを設定する
};
#endif // !CALIBRATION_H_
