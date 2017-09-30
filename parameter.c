#include "parameter.h"

//Parameter of Robo
int TAIL_ANGLE_STAND_UP = 98; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ANGLE_DANSA    = 85; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_LUG      = 65; /* 3点移動時の角度[度]      */
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
int   STEP_CLIMB_SPPED      = 15;

float GARAGE_X_POS          = 1000;
float GARAGE_LENGTH         =  150;

//Parameter of Step
float STEP_START_LENGTH     = 550;
float FST_DANSA_POS         = 260;
float SCD_DANSA_POS         = 200;
//float SCD_DANSA_ON_POS      =  60;
float SCD_DANSA_ON_POS      =  70;

int   STEP_CLIMB_MAX_SPEED    = 35;

//int   STBL_CNT_1st_DANSA      = 750;
int   STBL_CNT_1st_DANSA      = 200;
//int   STBL_CNT_2_DANSA      = 750;
int   STBL_CNT_2nd_DANSA      = 50;
int   STBL_CNT_2nd_DANSA_ON   = 400;

//LUG
float APPROACH_TO_LUG_LENGTH = 900;
//float STOP_POS_FROM_LUG      = 5;
float STOP_POS_FROM_LUG      = 10;

float APPROACH_TO_1st_LUG    = 150;
float APPROACH_TO_2nd_LUG    = 150;
float APPROACH_TO_3rd_LUG    = 150;


float LUG_1st_STOP           = 150;
//float LUG_1st_STOP           = 200;
float LUG_2nd_STOP           = 150;
//float LUG_3rd_STOP           = 150;
float LUG_3rd_STOP           = 200;

//Parameter of Garage
float STEP_TO_GARAGE_LENGTH = 1100;
float LUG_TO_GARAGE_LENGTH  =  650;

//Garage
int   SONAR_DIST            = 75;
int   GARAGE_LIT_DIST       = 48;
float GRAY_TO_GARAGE_LENGTH =  200;

#ifdef RIGHT_MODE
//Parameter of Course
float FINAL_STRAIGHT_LENGTH = 1900;
// Start_to_1st_Straight, Start_to_1st_Corner, Snd_Corner, Final_Corner, Return_to_Line 
float CORNER_CHECK[5]={1.0, -2.0, 1.0, 1.0, 2.5};

#else
float FINAL_STRAIGHT_LENGTH = 1100.0;
// Start_to_1st_Straight, Start_to_1st_Corner, Snd_Corner, Final_Corner, Return_to_Line 
float CORNER_CHECK[5]={1.0, -2.0, 2.0, 1.0, 1.0};

#endif


