/*
 *      モーターバーツクラスのヘッダファイル
 */

#ifndef CLOCKTIMER_H_
#define CLOCKTIMER_H_
#include "ev3api.h"
#include "Clock.h"

using namespace ev3api;
/*
 *  関数のプロトタイプ宣言
 */
 
class ClockTimer : public Clock{
private:

//protected:

public:
    ClockTimer();                       //コンストラクタ
    ~ClockTimer();                      //デストラクタ

};

#endif // !CLOCKTIMER_H_
