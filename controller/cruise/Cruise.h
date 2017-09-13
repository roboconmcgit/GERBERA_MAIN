/*
 *      走行制御クラスのヘッダファイル
 */

#ifndef CRUISE_H_
#define CRUISE_H_

#include "ev3api.h"

/*
 *  関数のプロトタイプ宣言
 */
 
class Cruise{
private:

//protected:

public:
    Cruise();            //コンストラクタ
    ~Cruise();                                 //デストラクタ
    void CruiseOperation();                    //動作判断
};

#endif // !CRUISE_H_
