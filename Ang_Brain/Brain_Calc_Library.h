/******************************************************************************
 *  ang_brain.h (for LEGO Mindstorms EV3)
 *  Created on: 2017/07/25
 *  Implementation of the Class ang_brain
 *  Author: Keiichi Tomii
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include <deque>
//#define DEBUG

using namespace std;

class BrainCalcLibrary {
public:
	explicit BrainCalcLibrary();
	void SetInitPIDGain(float kp, float ki, float kd, float dT);
	int CalcPIDContrInput(float command, float contrvalue);
	int Sat(int max, int min, int inputvalue);

	void SetValue(float setvaule);
	bool CheckValue(float checkvalue);

	void SetValueArea(float setvaule_max, float setvaule_min);
	bool CheckValueArea(float checkvalue);

	float Delay(int delaystep, float delayvaule);

	void SetTimer(float sampleperiod);
	bool CheckTimer(float checktime);

private:

    float KP=0, TI=0, TD=0, DT=1;//PIDparameter

    float setcheckvalue=0;
    float area_max=0, area_min=0;
    float Sampleperiod=0, currenttime=0, call_cnt=1;

    deque<float> delay;

};
