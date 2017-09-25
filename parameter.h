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

#define RIGHT_MODE

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
extern float RAD_315_DEG;
extern float RAD_345_DEG;
extern float RAD_360_DEG;
extern float RAD_450_DEG;

//Parameter of Course
extern float CORNER_CHECK[];
extern float FINAL_STRAIGHT_LENGTH;
extern float DEAD_ZONE_LENGTH;
extern float STEP_START_LENGTH;
extern float FST_DANSA_POS;
extern float SCD_DANSA_POS;
extern int   STEP_CLIMB_SPPED;

extern float STEP_TO_GARAGE_LENGTH;
extern float GRAY_TO_GARAGE_LENGTH;
extern float GARAGE_X_POS;
extern float GARAGE_LENGTH;

//LUG
extern float APPROACH_TO_LUG_LENGTH;
extern float STOP_POS_FROM_LUG;

extern float APPROACH_TO_1st_LUG;
extern float APPROACH_TO_2nd_LUG;
extern float APPROACH_TO_3rd_LUG;

extern float LUG_1st_STOP;
extern float LUG_2nd_STOP;
extern float LUG_3rd_STOP;

//Parameter of Garage
extern float STEP_TO_GARAGE_LENGTH;
extern float LUG_TO_GARAGE_LENGTH;
extern float GARAGE_X_POS;
extern float GARAGE_LENGTH;

extern int   SONAR_DIST;
extern int   GARAGE_LIT_DIST;

//Parameter of Area
extern float LineTrace1Area[4];
extern float MapTraceArea[4];
extern float MapTraceArea1[4];
extern float MapTraceArea2[4];
extern float MapTraceArea3[4];
extern float MapTraceArea4[4];
extern float MapTraceArea5[4];
extern float MapTraceArea6[4];
extern float MapTraceArea7[4];
extern float MapTraceArea8[4];

extern float StartArea[4];
extern float First_Straight[4];
extern float First_Corner[4];
extern float Second_Straight[4];
extern float Second_Corner[4];

extern float GoalArea[4];
extern float Goal_to_Step[4];
extern float StepArea[4];
extern float LookUpGateArea[4];
extern float GarageArea[4];
extern float StopArea[4];

extern float Gate12Area[4];
extern float Gate23Area[4];
extern float Gate34Area[4];
extern float Gate45Area[4];
extern float Gate56Area[4];
extern float Gate67Area[4];
extern float Gate78Area[4];
extern float Gate89Area[4];


extern int Approach_to_LUG_time;
extern int LUG_Tail_On_time;
extern int LUG_Tail_On_fowrd;
extern int LUG_Tailangle_fowrd;
extern float LineTracer_KP;
extern float LineTracer_KI;
extern float LineTracer_KD;

extern float TURN_PAI;

#endif // !PRAMETER_H_
