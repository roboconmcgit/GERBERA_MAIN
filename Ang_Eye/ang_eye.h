#ifndef ANAGO_EYE_H_
#define ANAGO_EYE_H_

//#define EYE_DEBUG

#include "parameter.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "GyroSensor.h"
#include "SonarSensor.h"

// 定義
class Ang_Eye {
public:
    explicit Ang_Eye(const ev3api::ColorSensor& colorSensor,
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

    int   linevalue = 0;//ライン値
    int   linevalue_LUG = 0;//ライン値
    
    //    float xvalue    = 0;//x座標推定値
    //    float yvalue    = 0;//y座標推定値

    float xvalue    = 0;//x座標推定値
    float yvalue    = 0;//y座標推定値

    float odo       = 0;//odometry
    float velocity  = 0;//Velocity

    int   encR      = 0;//右側タイヤ角度
    int   encL      = 0;
    float yawrate   = 0;
    float abs_angle = 0;
    bool  dansa     = 0;
    bool  det_gray  = 0;
    //signals for robo movement
    bool  robo_stop       = 0;
    bool  robo_forward    = 0;
    bool  robo_back       = 0;
    bool  robo_turn_left  = 0;
    bool  robo_turn_right = 0;

    float WheelAngVLt = 0;
    float WheelAngVRt = 0;
    float RoboVt      = 0;
    float RoboAngVt   = 0;
    bool  stop_sys    = 0;

    int16_t sonarDistance = 0; // 距離 [cm]
    bool  sonar_stop  = false;

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

    const ev3api::ColorSensor& mColorSensor;
    ev3api::Motor& mLeftWheel;
    ev3api::Motor& mRightWheel;
    ev3api::GyroSensor& mGyro;
    ev3api::SonarSensor& mSonar;

    enum Ang_Eye_Mode{
      CALIB_ANGLE,
      DET_MOVEMENT,
    };
    Ang_Eye_Mode enum_Mode;


    int8_t mWhite;
    int8_t mBlack;
    int8_t mWhite_slant;
    int8_t mBlack_slant;
    int8_t dColor_val[5]; //170814 ota signals for filter of color sensor value.
    
    float relative_angle;
    float correction_angle = 0.0;

    //signals for detection robo's direction
    int   cap_size = 1000;
    int   cap_cnt = 0;

    float angle_sum_dat = 0.0;
    float angle_ave_dat = 0.0;
    float dif_angle_ave_dat = 0.0;
    float old_angle_ave_dat = 0.0;
    float angle_dat_500ms[125]; //data array during 500ms @ 4ms task term

    float velocity_sum_dat = 0.0;
    float velocity_ave_dat = 0.0;
    float dif_velocity_ave_dat = 0.0;
    float old_velocity_ave_dat = 0.0;
    float velocity_dat_500ms[125]; //data array during 500ms @ 4ms task term

    int   line_dat_500ms[125]; //detecting for gray line
    int   max_line_dat;
    int   min_line_dat;

    int   sonar_counter = 0;
};

#endif  // ANAGO_EYE_H_
