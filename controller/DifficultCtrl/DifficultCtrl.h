/*
 *      モーターバーツクラスのヘッダファイル
 */

#ifndef DIFFICULT_CTRL_H_
#define DIFFICULT_CTRL_H_

#include "ev3api.h"
#include "parameter.h"

#include "CruiseCtrl.h"

/*
 *  関数のプロトタイプ宣言
 */
 
class DifficultCtrl{
private:

    CruiseCtrl  *gCruiseCtrl;

public:    
    DifficultCtrl(CruiseCtrl* Cruise);     //コンストラクタ
    ~DifficultCtrl();                      //デストラクタ

    void init ();
    int StartDashRunner();                              //スタートダッシュ
    int GarageRunner(int line_value_lug, float mOdo, float angle,int line_value);                                 //ガレージ走行
    int StopRobo(
        int &forward,
        float &yawratecmd,
        float &anglecommand,
        bool &tail_stand_mode
    );                                     //ロボット停止
    
};

#endif // !DIFFICULT_CTRL_H_
