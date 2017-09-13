
#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "ev3api.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "Ang_Robo.h" //it will be changed to Ang_Robo


// using宣言
//using namespace ev3api;
using ev3api::ColorSensor;
using ev3api::TouchSensor;

typedef struct calibrationData{
    unsigned char white;
    unsigned char black;
    unsigned char white_slant;
    unsigned char black_slant;
}ST_CALIB;

#define RIGHT_COURCE_MODE

class calibration{
private:
    ev3api::ColorSensor& gColorSensor;
    ev3api::TouchSensor& gTouchSensor;
    Ang_Robo    *gAng_Robo;
//protected:
//    xxx          *xxx;               //calibration用のクラス

public:
    calibration(
        ev3api::ColorSensor& Color,
        ev3api::TouchSensor& Touch,
        Ang_Robo *Robo
    ):  gColorSensor(Color),
        gTouchSensor(Touch),
        gAng_Robo(Robo)
    {};   //コンストラクタ
    ~calibration(){};                      //デストラクタ
    calibrationData set_calibration();
                                        //キャリブレーションを設定する
};
#endif // !CALIBRATION_H_
