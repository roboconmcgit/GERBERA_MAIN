#include "ang_eye.h"
#include "math.h"

//#define DEBUG_ODOMETRY
//#define DEBUG_EYE_DEBUG
//#define DEBUG_NANSYO

// 定数宣言
/**
 * コンストラクタ
 * @param colorSensor カラーセンサ
 */
Ang_Eye::Ang_Eye(ColorParts* colorSensor,
		 ev3api::Motor& leftWheel,
		 ev3api::Motor& rightWheel,
		 ev3api::GyroSensor& gyro,
     ev3api::SonarSensor& sonar)
  :
    mColorSensor(colorSensor),
    mLeftWheel(leftWheel),
    mRightWheel(rightWheel),
    mGyro(gyro),
    mSonar(sonar)

{
}

void Ang_Eye::init(){

}


void Ang_Eye::set_White_Black_Threshold(int8_t white, int8_t black, int8_t white_slant, int8_t black_slant) {

}

void Ang_Eye::det_Line_Value() {

}



void Ang_Eye::WheelOdometry(float dT) {
}

//170815
void Ang_Eye::det_Dansa( ) {
}

//170906
void Ang_Eye::setSonarDistance(void) {
}

#ifdef DEBUG_EYE_DEBUG

void Ang_Eye::saveData( ){

}

void Ang_Eye::export_dat( ){
}

#endif
