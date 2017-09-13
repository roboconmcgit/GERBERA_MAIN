/*
 *      サンプルプログラム(1)のヘッダファイル
 */

#ifndef LOGGING_H_
#define LOGGING_H_
#include "ev3api.h"

//#ifdef __cplusplus
//extern "C" {
//#endif

#define LOG_SIZE 10       //10000の場合、約50秒で一杯になる→制御周期0.005秒か？


/*
 *  関数のプロトタイプ宣言
 */
 
//using namespace ev3api{
class Logging{
public:
    Logging();
    ~Logging();
    void LogWrite(float turn,float motor_ang_l,float motor_ang_r, float volt, float pwm_L, float pwm_R);
    void file_save(void);
    uint32_t LogCount(void){
        return(i_LOG);
    }
private:
    //static float LOG_forward[LOG_SIZE] ={0};
    float LOG_turn[LOG_SIZE];
    //static float LOG_gyro[LOG_SIZE] ={0};
    //static float LOG_GYRO_OFFSET[LOG_SIZE] ={0};
    float LOG_motor_ang_l[LOG_SIZE];
    float LOG_motor_ang_r[LOG_SIZE];
    float LOG_volt[LOG_SIZE];
    float LOG_pwm_L[LOG_SIZE];
    float LOG_pwm_R[LOG_SIZE];
    //static float LOG_angle[LOG_SIZE] ={0};
    //static float LOG_color_sensor[LOG_SIZE] ={0};
    uint32_t i_LOG;
};
//};

//#ifdef __cplusplus
//}
//#endif
#endif // !LOGGING_H_
