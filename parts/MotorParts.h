/*
 *      モーターバーツクラスのヘッダファイル
 */

#ifndef MOTORPARTS_H_
#define MOTORPARTS_H_

#include "ev3api.h"
#include "util.h"
#include "Motor.h"

#include "parameter.h"

using namespace ev3api;

// モータ種別　番号
enum{
    MOTORPARTS_LEFT_NO = 0,
    MOTORPARTS_RIGHT_NO,
    MOTORPARTS_TAIL_NO,
    MOTORPARTS_NUM
};

// モータ種別　Bit位置
#define MOTORPARTS_LEFT_BIT  (0x01<<MOTORPARTS_LEFT_NO)
#define MOTORPARTS_RIGHT_BIT (0x01<<MOTORPARTS_RIGHT_NO)
#define MOTORPARTS_TAIL_BIT  (0x01<<MOTORPARTS_TAIL_NO)

// テール動作モード
enum{
    TAIL_RESET = 0,
    TAIL_STAND_UP,
    TAIL_DRIVE
};

// キャリブレーションパラメータ
typedef struct MotorParts_Calibra{
    int32_t Offset[MOTORPARTS_NUM];
}MOTORPARTS_CALIBRA;

/*
 *  関数のプロトタイプ宣言
 */
 
class MotorParts{
private:
    MOTORPARTS_CALIBRA MotorParts_Calib;          //キャリブレーションデータ
    bool MotorParts_State[MOTORPARTS_NUM];        //状態フラグ　0:正常、1:以上

//protected:
    Motor          leftMotor;               //左モータパーツ用のモータクラス
    Motor          rightMotor;              //右モータパーツ用のモータクラス
    Motor          tailMotor;               //テールモータパーツ用のモータクラス

    // WheelOdometry用
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
    float relative_angle;
    float correction_angle = 0.0;

    // Tail control
    PID *gTail_pwm = new PID();
    float angle2_e;   /* angle2用変数型宣言 */
    float angle2_eo1; /* angle2用変数型宣言 */
    float angle2_eo2; /* angle2用変数型宣言 */
    float pwm_o;      /* angle2用変数型宣言 */
    float pwm2;
    float pwm;

public:
    // WheelOdometry用
    float xvalue    = 735.96;//x座標推定値
    float yvalue    = 415.74;//y座標推定値

    float odo       = 0.0;//odometry
    float velocity  = 0.0;//Velocity

    int   encR      = 0;//右側タイヤ角度
    int   encL      = 0;
    float yawrate   = 0;
    float abs_angle = 0;
    //signals for robo movement
    bool  robo_stop       = 0;
    bool  robo_forward    = 0;
    bool  robo_back       = 0;
    bool  robo_turn_left  = 0;
    bool  robo_turn_right = 0;

    enum Ang_Eye_Mode{
        CALIB_ANGLE,
        DET_MOVEMENT,
      };
      Ang_Eye_Mode enum_Mode;
      
    MotorParts();                       //コンストラクタ
    ~MotorParts();                      //デストラクタ
    void MotorPartsCalibra(MOTORPARTS_CALIBRA s_calibra);
                                        //全てのモータのキャリブレーションを設定する
    void MotorPartsReset(int8_t motorBit);
                                        //指定したモータをリセットする
    int32_t getMotorPartsPwm(int8_t motorNo);
                                        //指定したモータの現在値を取得する
    void setMotorPartsLeftRight(int rightMotorPwm, int leftMotorPwm);
                                        //左右モータを動かす
    void stopMotorPartsLeftRight(void);
                                        //左右モータを止める
    void setMotorPartsTail(int8_t tailMove);
                                        //テールモータを動かす
    void  WheelOdometry(float dT);

    void tail_control(signed int angle); //2017.07.28 kota copy from 3-apex
    //170816 ota add tail control
     void tail_reset();
     void tail_stand_up(); //tail for gyro reset and color sensor calibration
     
};

#endif // !MOTORPARTS_H_
