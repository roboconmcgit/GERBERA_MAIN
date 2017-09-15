//Parameter of Robo
int TAIL_ANGLE_STAND_UP = 98; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ANGLE_DANSA    = 85; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_LUG      = 75; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_GARAGE   = 94; /* 完全停止時の角度[度]     */

int INITIAL_WHITE_THRESHOLD = 40;  // 黒色の光センサ値
int INITIAL_BLACK_THRESHOLD = 20;  // 黒色の光センサ値

float WheelDiameter = 79.95;  //背面から見て左タイヤの直径[mm] 0817 tada
float WHEEL_R       = 39.975; //Wheel radius
int   RoboTread      = 160; //トレッド長さ[mm]

int   CL_SNSR_GAIN_GRAY = 2;

//Parameter of time length unit
float dT_100ms = 0.1;
float dT_4ms   = 0.004;

float PAI         =  3.1472;
float FIVE_PAI    = 15.708;

float RAD_1_DEG   = 0.0175; //deg@1rad 
float RAD_5_DEG   = 0.0873; //
float RAD_15_DEG  = 0.2618; //
float RAD_30_DEG  = 0.5236; //

float MINUS_RAD_5_DEG  = -0.0873; //
float MINUS_RAD_15_DEG = -0.2618; //
float MINUS_RAD_30_DEG = -0.5236; //

float RAD_90_DEG  = 1.5708; //
float RAD_120_DEG = 2.0944; //
float RAD_315_DEG = 5.4978; //
float RAD_345_DEG = 6.0214; //
float RAD_360_DEG = 6.2832; //
float RAD_450_DEG = 7.8540;

float DEAD_ZONE_LENGTH      =  400.0; //0910 tada

//Parameter of Step
float STEP_START_LENGTH     = 400;
float FST_DANSA_POS         = 260;
float SCD_DANSA_POS         = 260;
int   STEP_CLIMB_SPPED      = 15;

float GARAGE_X_POS          = 1100;
float GARAGE_LENGTH         =  150;

int   SONAR_DIST            = 70;
int   GARAGE_LIT_DIST       = 50;
float GRAY_TO_GARAGE_LENGTH =  200;
float STEP_TO_GARAGE_LENGTH = 1100;

#ifdef RIGHT_MODE
//Parameter of Course
float FINAL_STRAIGHT_LENGTH = 1900;

//Parameter of Area
float LineTrace1Area[4]={0, 0, 0, 0};
float MapTraceArea[4]={0, 0, 0, 0};
float MapTraceArea1[4]={0.0, 936.52, 2000.0, 2558.54};
float MapTraceArea2[4]={0.0, 2100.0, 2558.54, 3800.0};
float MapTraceArea3[4]={936.52, 2100.0, 2497.44, 2558.54};
float MapTraceArea4[4]={936.52, 1614.4, 1290.24, 2497.44};
float MapTraceArea5[4]={936.52, 1614.4, 0.0, 1290.24};
float MapTraceArea6[4]={1614.4, 2255.65, 0.0, 2497.44};
float MapTraceArea7[4]={2255.65, 3202.03, 0.0, 2497.44};
float MapTraceArea8[4]={3202.03, 4200.0, 0.0, 2497.44};

float StartArea[4]       = {-200,  200, -200,  500};
float First_Straight[4]  = {-200,  200,  500, 2000};
float First_Corner[4]    = {-200, 2000, 2000, 3500};
float Second_Corner[4]   = {-200, 2000, -200, 2000};
float Second_Straight[4] = {2000, 3000, 1000, 2000};
float GoalArea[4]        = {3000, 4000, 1000, 2000};
float Goal_to_Step[4]    = {4000, 6000, 1000, 2000};
float StepArea[4]        = {4000, 6000, 2000, 5000};

float LookUpGateArea[4]={0, 0, 0, 0};
float GarageArea[4]={0, 0, 0, 0};
float StopArea[4]={0, 0, 0, 0};

float Gate12Area[4]={0, 0, 0, 0};
float Gate23Area[4]={0, 0, 0, 0};
float Gate34Area[4]={0, 0, 0, 0};
float Gate45Area[4]={0, 0, 0, 0};
float Gate56Area[4]={0, 0, 0, 0};
float Gate67Area[4]={0, 0, 0, 0};
float Gate78Area[4]={0, 0, 0, 0};
float Gate89Area[4]={0, 0, 0, 0};


#else
float FINAL_STRAIGHT_LENGTH = 1100.0;

//x_left, x_right, y_under, y_top
/*ライン（攻）
float LineTrace1Area[4]={0.0, 936.52, 0.0, 2000.0};
float MapTraceArea1[4]={0.0, 936.52, 2000.0, 2558.54};
float MapTraceArea2[4]={0.0, 2100.0, 2558.54, 3800.0};
float MapTraceArea3[4]={936.52, 2100.0, 2523.54, 2558.54};
float MapTraceArea4[4]={936.52, 1592.76, 1344.29, 2523.54};
float MapTraceArea5[4]={936.52, 1592.76, 0.0, 1344.29};
float MapTraceArea6[4]={1592.76, 2487.29, 0.0, 2523.54};
float MapTraceArea7[4]={2487.29, 3024.44, 0.0, 2523.54};
float MapTraceArea8[4]={3024.44, 4200.0, 0.0, 2523.54};
*/
/* ライン（安）１
float LineTrace1Area[4]={0.0, 936.52, 0.0, 2000.0};
float MapTraceArea1[4]={0.0, 936.52, 2000.0, 2558.54};
float MapTraceArea2[4]={0.0, 2100.0, 2558.54, 3800.0};
float MapTraceArea3[4]={936.52, 2100.0, 2497.44, 2558.54};
float MapTraceArea4[4]={936.52, 1601.26, 1290.24, 2497.44};
float MapTraceArea5[4]={936.52, 1601.26, 0.0, 1290.24};
float MapTraceArea6[4]={1601.26, 2487.29, 0.0, 2497.44};
float MapTraceArea7[4]={2487.29, 3302.03, 0.0, 2497.44};
float MapTraceArea8[4]={3302.03, 4200.0, 0.0, 2497.44};
*/
///*
float LineTrace1Area[4]={0.0, 936.52, 0.0, 2000.0};
float MapTraceArea1[4]={0.0, 936.52, 2000.0, 2558.54};
float MapTraceArea2[4]={0.0, 2100.0, 2558.54, 3800.0};
float MapTraceArea3[4]={936.52, 2100.0, 2497.44, 2558.54};
float MapTraceArea4[4]={936.52, 1614.4, 1290.24, 2497.44};
float MapTraceArea5[4]={936.52, 1614.4, 0.0, 1290.24};
float MapTraceArea6[4]={1614.4, 2255.65, 0.0, 2497.44};
float MapTraceArea7[4]={2255.65, 3202.03, 0.0, 2497.44};
float MapTraceArea8[4]={3202.03, 4200.0, 0.0, 2497.44};
//*/
float StartArea[4]       = {-200.0,  200.0, -200.0,  500.0};
float First_Straight[4]  = {-200.0,  200.0,  500.0, 2000.0};
float First_Corner[4]    = {-200.0, 2000.0, 2000.0, 3500.0};
float Second_Corner[4]   = {-200.0, 2000.0, -200.0, 2000.0};
float Second_Straight[4] = {2000.0, 3000.0, 1000.0, 2000.0};
float GoalArea[4]        = {3000.0, 4000.0, 1000.0, 2000.0};
float Goal_to_Step[4]    = {4000.0, 6000.0, 1000.0, 2000.0};
float StepArea[4]        = {4000.0, 6000.0, 2000.0, 5000.0};

float LookUpGateArea[4]={0.0, 0.0, 0.0, 0.0};
float GarageArea[4]={0.0, 0.0, 0.0, 0.0};
float StopArea[4]={0.0, 0.0, 0.0, 0.0};

/* ライン（攻）
float Gate12Area[4]={0.0, 936.52, 2000.0, 2558.54};
float Gate23Area[4]={0.0, 2100.0, 2558.54, 3800.0};
float Gate34Area[4]={936.52, 2100.0, 2523.54, 2558.54};
float Gate45Area[4]={936.52, 1592.76, 1344.29, 2523.54};
float Gate56Area[4]={936.52, 1592.76, 0.0, 1344.29};
float Gate67Area[4]={1592.76, 2487.29, 0.0, 2523.54};
float Gate78Area[4]={2487.29, 3024.44, 0.0, 2523.54};
float Gate89Area[4]={3024.44, 4200.0, 0.0, 2523.54};
*/
/* ライン（安）１
float Gate12Area[4]={0.0, 936.52, 2000.0, 2558.54};
float Gate23Area[4]={0.0, 2100.0, 2558.54, 3800.0};
float Gate34Area[4]={936.52, 2100.0, 2497.44, 2558.54};
float Gate45Area[4]={936.52, 1601.26, 1290.24, 2497.44};
float Gate56Area[4]={936.52, 1601.26, 0.0, 1290.24};
float Gate67Area[4]={1601.26, 2487.29, 0.0, 2497.44};
float Gate78Area[4]={2487.29, 3302.03, 0.0, 2497.44};
float Gate89Area[4]={3302.03, 4200.0, 0.0, 2497.44};
*/
///*
float Gate12Area[4]={0.0, 936.52, 2000.0, 2558.54};
float Gate23Area[4]={0.0, 2100.0, 2558.54, 3800.0};
float Gate34Area[4]={936.52, 2100.0, 2497.44, 2558.54};
float Gate45Area[4]={936.52, 1614.4, 1290.24, 2497.44};
float Gate56Area[4]={936.52, 1614.4, 0.0, 1290.24};
float Gate67Area[4]={1614.4, 2255.65, 0.0, 2497.44};
float Gate78Area[4]={2255.65, 3202.03, 0.0, 2497.44};
float Gate89Area[4]={3202.03, 4200.0, 0.0, 2497.44};
//*/
//Parameter of Garage
#endif


