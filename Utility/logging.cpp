/**
 ******************************************************************************
 ** ファイル名 : logging.cpp
 **
 ** 概要 : 
 **
 ** 注記 : 
 ******************************************************************************
 **/

#include "logging.h"
#include <stdlib.h>
#include <string.h>

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/* LCDフォントサイズ */
//#define CALIB_FONT (EV3_FONT_SMALL)
//#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
//#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* 関数プロトタイプ宣言 */



//*****************************************************************************
// 関数名 : コンストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
Logging::Logging(){
    //LOG_forward[] ={0};
	memset((void *)LOG_turn, 0, LOG_SIZE);
    //static float LOG_gyro[LOG_SIZE] ={0};
    //static float LOG_GYRO_OFFSET[LOG_SIZE] ={0};
	memset((void *)LOG_motor_ang_l, 0, LOG_SIZE);
	memset((void *)LOG_motor_ang_r, 0, LOG_SIZE);
	memset((void *)LOG_volt, 0, LOG_SIZE);
	memset((void *)LOG_pwm_L, 0, LOG_SIZE);
	memset((void *)LOG_pwm_R, 0, LOG_SIZE);
    //static float LOG_angle[LOG_SIZE] ={0};
    //static float LOG_color_sensor[LOG_SIZE] ={0};
    i_LOG = 0;  

}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
Logging::~Logging(){



}

//*****************************************************************************
// 関数名 : LogWrite
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Logging::LogWrite(float turn,float motor_ang_l,float motor_ang_r, float volt, float pwm_L, float pwm_R){
   	if(i_LOG >= LOG_SIZE) return;
    LOG_turn[i_LOG] =turn;
    LOG_motor_ang_l[i_LOG] =motor_ang_l;
    LOG_motor_ang_r[i_LOG] =motor_ang_r;
    LOG_volt[i_LOG] =volt;
    LOG_pwm_L[i_LOG] =pwm_L;
    LOG_pwm_R[i_LOG] =pwm_R;
    i_LOG++;
}


//*****************************************************************************
// 関数名 : file_save
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void Logging::file_save(void){
//    ev3_lcd_draw_string("Log Writing Start", 0, CALIB_FONT_HEIGHT*1);
	FILE *fout = fopen("/LOG_DATA.txt", "w");
	//fprintf(fout, "time,forward,turn,gyro,GYRO_OFFSET,motor_ang_l,motor_ang_r,volt,pwm,color_sensor,angle\n");
	for(int i = 0 ; i < LOG_SIZE ; i++){
//		fprintf(fout, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
	    fprintf(fout, "%f,%f,%f,%f,%f,%f,%f\n", 
			i*0.005, 
//			LOG_forward[i], 
			LOG_turn[i], 
//			LOG_gyro[i], 
//			LOG_GYRO_OFFSET[i], 
			LOG_motor_ang_l[i], 
			LOG_motor_ang_r[i], 
			LOG_volt[i], 
			LOG_pwm_L[i], 
			LOG_pwm_R[i]); 
//			LOG_color_sensor[i]);
//		    LOG_angle[i];
	}
	fclose(fout);
//    ev3_lcd_draw_string("Log Writing Finish", 0, CALIB_FONT_HEIGHT*1);
}
