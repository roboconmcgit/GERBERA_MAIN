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
 
#define STOPWATCH_MAX  (10)

class ClockTimer : public Clock{
private:
    uint32_t stopwatch_time[STOPWATCH_MAX];

//protected:

public:
    ClockTimer();                       //コンストラクタ
    ~ClockTimer();                      //デストラクタ

    void StopWatch_Start(int8_t stopwatch_no);
    uint32_t StopWatch_Stot(int8_t stopwatch_no);

};

#endif // !CLOCKTIMER_H_
