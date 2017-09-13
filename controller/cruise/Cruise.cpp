/**
 ******************************************************************************
 ** ファイル名 : Cruise.cpp
 **
 ** 概要 : 走行制御クラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "Cruise.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//*****************************************************************************
// 関数名 : コンストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
Cruise::Cruise(){

}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
Cruise::~Cruise(){
}

//*****************************************************************************
// 関数名 : CruiseCalibra
// 引数 : Cruise_CALIBRA s_calibra
// 返り値 : なし
// 概要 : 全てのモータのキャリブレーションを設定する
//*****************************************************************************
void Cruise::CruiseOperation(void){

}
