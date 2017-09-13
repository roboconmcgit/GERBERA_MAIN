/******************************************************************************
 *  ang_brain.h (for LEGO Mindstorms EV3)
 *  Created on: 2017/07/25
 *  Implementation of the Class ang_brain
 *  Author: Keiichi Tomii
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "parameter.h"

class StrategyDet {
public:
	explicit StrategyDet();                      //コンストラクタ
	void Det_run(float x_value, float y_value); //判定を実行

	int StrategyNum;   //走行戦略
	int VirtualGateNum;//仮想ゲート
	int Max_Forward;
	float Max_Yawrate;
	float Min_Yawrate;

private:

	void RunningStrategyDet(float x_value, float y_value);//走行戦略判定
	void VirtualGateDet(float x_value, float y_value);//仮想ゲート判定

	bool Robo_Area_Estimator(float x_left,
                                float x_right,
                                float y_under,
                                float y_top,
                                float x_value,
                                float y_value);//走行エリア推定

	enum enumStrategy{
		StartDash=510,
		LineTrace1=520,
		MapTrace=530,
		Goal=610,
		Goal2Step=650,
		Step=710,
		LookUpGate=810,
		Garage=910,
		Stop=1010
	};

	enum enumVirtualGate{
		Gate12=531,
		Gate23=532,
		Gate34=533,
		Gate45=534,
		Gate56=535,
		Gate67=536,
		Gate78=537,
		Gate89=538,
		None=539
	};
	enumStrategy Strategy;
	enumVirtualGate VirtualGate;

};
