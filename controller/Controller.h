/*
 *      コントローラクラスのヘッダファイル
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "ev3api.h"
#include "Cruise.h"

#include "ColorParts.h"
#include "MotorParts.h"
#include "GyroParts.h"
#include "SonarParts.h"
#include "TouchParts.h"

/*
 *  関数のプロトタイプ宣言
 */
 
class Controller{
private:
    ColorParts  *gColorParts;
    MotorParts  *gMotorParts;
    GyroParts   *gGyroParts;
    SonarParts  *gSonarParts;
    TouchParts  *gTouchParts;

//protected:
    Cruise              *mC_Cruise;                   //走行制御用のクラスポインタ

public:
    Controller(
        ColorParts  *Color,
        MotorParts  *Motor,
        GyroParts   *Gyro,
        SonarParts  *Sonar,
        TouchParts  *Touch);            //コンストラクタ
    ~Controller();                                 //デストラクタ
    void ControllerOperation();                    //動作判断
};

#endif // !CONTROLLER_H_
