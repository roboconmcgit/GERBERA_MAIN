/******************************************************************************
 *  ang_brain.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2017/07/29
 *  Implementation of the Class RandomWalker
 *  Author: Keiichi Tomii
 *****************************************************************************/

#include "Brain_Calc_Library.h"
#include "math.h"
//#define DEQUE_EN

BrainCalcLibrary::BrainCalcLibrary(){

}

void BrainCalcLibrary::SetInitPIDGain(float kp, float ti, float td, float dT){

	KP=kp;
	TI=ti;
	TD=td;
	DT=dT;

}
int BrainCalcLibrary::CalcPIDContrInput(float command, float contrvalue){

	static int error_old=0,error_P_old=0;		//過去の偏差

	float u_delta=0;							//制御入力の差分
	int error=0,error_P=0,error_I=0,error_D=0;	//偏差
	float u_P_delta=0,u_I_delta=0,u_D_delta=0;	//制御入力の差分
	int u=0;

	error=command-contrvalue;					//制御偏差を計算
	error_P=error-error_old;					//P制御用の偏差を計算
	error_I=error;								//I制御用の偏差を計算
	error_D=error_P-error_P_old;				//D制御用の偏差を計算

	u_P_delta=KP*error_P;						//P制御用の入力差分を計算
	u_I_delta=(DT/TI)*error_I;						//I制御用の入力差分を計算
	u_D_delta=(TD/DT)*error_D;						//D制御用の入力差分を計算

	u_delta=u_P_delta+u_I_delta+u_D_delta;		//PID制御入力の差分を計算
	u=u+u_delta;								//制御入力を計算

	error_old=error;							//過去の偏差を保存
	error_P_old=error_P;						//過去の偏差を保存

	//入力制限
	return Sat(100,-100,u);


}
int BrainCalcLibrary::Sat(int max, int min, int inputvalue){

	if(inputvalue>max){

		return max;

	}else if(inputvalue<min){

		return min;

	}else{

		return inputvalue;

}

}

void BrainCalcLibrary::SetValue(float setvaule){

	setcheckvalue=setvaule;
}
bool BrainCalcLibrary::CheckValue(float checkvalue){

	if(setcheckvalue==checkvalue){

		return true;
		setcheckvalue=0;
	}else{

		return false;

	}

}

void BrainCalcLibrary::SetValueArea(float setvaule_max, float setvaule_min){

    area_max=setvaule_max;
    area_min=setvaule_min;
}
bool BrainCalcLibrary::CheckValueArea(float checkvalue){

	if(area_max>=checkvalue&&checkvalue>area_min){

		return true;
	    area_max=0;
	    area_min=0;
	}else{

		return false;

	}

}

float BrainCalcLibrary::Delay(int delaystep, float delayvaule){
#ifdef DEQUE_EN
    delay.push_front(delayvaule);

    int current_size = (int)delay.size();
    //もしデータ数が指定値より大きくなった場合、一番古いデータを捨てる
    if(current_size > delaystep){

    	delay.pop_front();

    }
#endif
    return delay[delaystep];

}

void BrainCalcLibrary::SetTimer(float sampleperiod){

	Sampleperiod=sampleperiod;

}

bool BrainCalcLibrary::CheckTimer(float checktime){

	currenttime=call_cnt*Sampleperiod;

	call_cnt++;

	if(checktime>=currenttime){

		return true;
		call_cnt=1;
		currenttime=0;

	}else{

		return false;

	}

}
