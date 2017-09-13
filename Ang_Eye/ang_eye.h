#ifndef ANAGO_EYE_H_
#define ANAGO_EYE_H_

//#define EYE_DEBUG

#include "parameter.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "GyroSensor.h"
#include "SonarSensor.h"

#include "ColorParts.h"

// 定義
class Ang_Eye {
public:
    explicit Ang_Eye(ColorParts* colorSensor,
		     ev3api::Motor& leftWheel,
		     ev3api::Motor& rightWheel,
             ev3api::GyroSensor& gyro,
             ev3api::SonarSensor& sonar);

    void  init(); //17.0.28 k-ota add
    void  set_White_Black_Threshold(int8_t white, int8_t black, int8_t white_slant, int8_t black_slant);
    void  det_Line_Value();
    void  WheelOdometry(float dT);
    void  det_Dansa();
    void  setSonarDistance(void);


#ifdef EYE_DEBUG
    void  saveData( );
    void  export_dat( );

    int   log_size = 25000;
    int   log_cnt = 0;
    int   log_dat_00[25000];
    int   log_dat_01[25000];
    int   log_dat_02[25000];
    int   log_dat_03[25000];
    float log_fdat_00[25000];
    float log_fdat_01[25000];
    float log_fdat_02[25000];

#endif

private:

    static const int8_t INITIAL_WHITE_THRESHOLD;
    static const int8_t INITIAL_BLACK_THRESHOLD;

    ColorParts* mColorSensor;
    ev3api::Motor& mLeftWheel;
    ev3api::Motor& mRightWheel;
    ev3api::GyroSensor& mGyro;
    ev3api::SonarSensor& mSonar;
    //signals for detection robo's direction
};

#endif  // ANAGO_EYE_H_
