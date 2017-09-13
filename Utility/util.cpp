
#include "util.h"
#include <math.h>

#define EMG_TH_V 350
#define EMG_TH_N 250
#define EMG_TH_C 100

// 初期処理用
void init_f(const char *str) {
  // フォントの設定と0行目の表示
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string(str, 0, 0);
}

/**
 * 行単位で引数の文字列を表示
 * @param str 表示する文字列
 * @param line 20ドットごとの行番号（1から5）
 */
void msg_f(const char *str, int32_t line) {
  const int8_t line_height = 20;
  ev3_lcd_fill_rect(0, line * line_height, EV3_LCD_WIDTH, line_height, EV3_LCD_WHITE);
  ev3_lcd_draw_string(str, 0, line * line_height);
}

int emergencyStop(float velocity){

	static int emg_num;
	static int clr_cnt;
	
	if(fabsf(velocity)>EMG_TH_V)
		emg_num++;
	else
		clr_cnt++;
	
	if(clr_cnt>EMG_TH_C){
		emg_num=0;
		clr_cnt=0;
	}
	
	if(emg_num>EMG_TH_N)
		return 1;
	else
		return 0;

}

PID::PID(){

}

void PID::init_pid(float kp, float ki, float kd, float dT){
  KP=0;
  TI=0;
  TD=0;
  DT=1;
  
  error_old=0;
  error_P_old=0;		//過去の偏差
}

int PID::calc_pid(float command, float contrvalue){
  float u_delta=0;							//制御入力の差分
  int error=0;
  int error_P=0;
  int error_I=0;
  int error_D=0;	//偏差
  float u_P_delta=0;
  float u_I_delta=0;
  float u_D_delta=0;	//制御入力の差分
  int u=0;

  error=command-contrvalue;	        //制御偏差を計算
  error_P=error-error_old;		//P制御用の偏差を計算
  error_I=error;			//I制御用の偏差を計算
  error_D=error_P-error_P_old;		//D制御用の偏差を計算

  u_P_delta=KP*error_P;			//P制御用の入力差分を計算
  u_I_delta=(DT/TI)*error_I;		//I制御用の入力差分を計算
  u_D_delta=(TD/DT)*error_D;		//D制御用の入力差分を計算
  
  u_delta=u_P_delta+u_I_delta+u_D_delta;//PID制御入力の差分を計算
  u=u+u_delta;				//制御入力を計算

  error_old=error;			//過去の偏差を保存
  error_P_old=error_P;			//過去の偏差を保存

  //入力制限
  return sat(100,-100,u);
}

int PID::sat(int max, int min, int inputvalue){

  if(inputvalue>max){
    return max;
  }else if(inputvalue<min){
    return min;
  }else{
    return inputvalue;
    
  }
}
