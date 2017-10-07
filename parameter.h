#ifndef PRAMETER_H_
#define PRAMETER_H_

/*
#ifdef __cplusplus
extern "C" {
#endif

#define dT_100ms             0.1 //タスク周期[s]

#ifdef __cplusplus
}
#endif
*/

//#define RIGHT_MODE

//#define LEFT_MODE

#define COLORPARTS_CH         (PORT_3)
#define GYRO_CH               (PORT_4)
#define LEFT_MOTOR_CH         (PORT_C)
#define RIGHT_MOTOR_CH        (PORT_B)
#define TAIL_MOTOR_CH         (PORT_A)
#define SONARPARTS_CH         (PORT_2)
#define TOUCHPARTS_CH         (PORT_1)

extern int INITIAL_WHITE_THRESHOLD;
extern int INITIAL_BLACK_THRESHOLD;

#define LIGHT_WHITE          40  /* 白色の光センサ値 */
#define LIGHT_BLACK           0  /* 黒色の光センサ値 */
//#define TAIL_ANGLE_STAND_UP  80  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
//#define P_GAIN             0.65F /* 完全停止用モータ制御比例係数 */
#define P_GAIN             1 /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */
#define SONAR_ALERT_DISTANCE 30  /* 超音波センサによる障害物検知距離[cm] */

//Parameter of Robo
extern int TAIL_ANGLE_STAND_UP;  /* 完全停止時の角度[度] */
extern int TAIL_ANGLE_RUN;     /* バランス走行時の角度[度] */
extern int TAIL_ANGLE_DANSA; /* 完全停止時の角度[度] */
extern int TAIL_ANGLE_LUG; /* 3点移動時の角度[度] */
extern int TAIL_ANGLE_GARAGE; /* 完全停止時の角度[度] */

extern float WheelDiameter; //背面から見てタイヤの直径[mm]
extern float WHEEL_R;       //radius of wheel[mm]
extern int   RoboTread;      //トレッド長さ[mm]

extern int   CL_SNSR_GAIN_GRAY;

//Parameter of time length unit
extern float dT_100ms;
extern float dT_4ms;

extern float PAI;
extern float FIVE_PAI;

extern float RAD_315_DEG;
extern float RAD_1_DEG; //deg@1rad 
extern float RAD_5_DEG; //
extern float RAD_15_DEG; //deg@1rad 
extern float RAD_30_DEG; //

extern float MINUS_RAD_5_DEG; //
extern float MINUS_RAD_15_DEG; //
extern float MINUS_RAD_30_DEG; //

extern float RAD_90_DEG;
extern float RAD_120_DEG;
extern float RAD_150_DEG;
extern float RAD_315_DEG;
extern float RAD_345_DEG;
extern float RAD_360_DEG;
extern float RAD_450_DEG;

//Parameter of Course
extern float CORNER_CHECK[];
extern float FINAL_STRAIGHT_LENGTH;
extern float DEAD_ZONE_LENGTH;

//Parameter of Step
extern float STEP_START_LENGTH;
extern float FST_DANSA_POS;
extern float SCD_DANSA_POS;
extern float SCD_DANSA_ON_POS;

extern int   STEP_CLIMB_MAX_SPEED;

extern int   STBL_CNT_1st_DANSA;
extern int   STBL_CNT_2nd_DANSA;
extern int   STBL_CNT_2nd_DANSA_ON;

//LUG
extern float APPROACH_TO_LUG_LENGTH;
extern int STOP_POS_FROM_LUG;
extern int STOP_POS_APP_LUG;

extern float APPROACH_TO_1st_LUG;
extern float APPROACH_TO_2nd_LUG;
extern float APPROACH_TO_3rd_LUG;

extern float LUG_1st_STOP;
extern float LUG_2nd_STOP;
extern float LUG_3rd_STOP;

extern float LUG_YAW_GAIN;
extern int   LUG_COL_VAL_OFFSET;
extern int   LUG_COL_VAL_GAIN;
extern float LUG_GRAY_TO_GARAGE;

//Parameter of Garage
extern float STEP_TO_GARAGE_LENGTH;
extern float LUG_TO_GARAGE_LENGTH;
extern float GARAGE_X_POS;
extern float GARAGE_LENGTH;

extern int   SONAR_DIST;
extern int   GARAGE_LIT_DIST;

extern float GARAGE_OFFSET_ANGLE;

extern int Approach_to_LUG_time;
extern int LUG_Tail_On_time;
extern int LUG_Tail_On_fowrd;
extern int LUG_Tailangle_fowrd;
extern float LineTracer_KP;
extern float LineTracer_KI;
extern float LineTracer_KD;

extern float TURN_PAI;

extern int SONAR_TIME;

#endif // !PRAMETER_H_
