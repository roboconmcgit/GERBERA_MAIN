//Anago System
//Date:2017.7.25
//Author:Kaoru Ota


#include "util.h"
#include "ParamFileRead.h"
#include "ev3api.h"
#include "app.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"

//anagoサブシステム
#include "ang_eye.h"
#include "ang_brain.h"
#include "Ang_Robo.h" //it will be changed to Ang_Robo

#include "calibration.h"

// デストラクタ問題の回避
// https://github.com/ETrobocon/etroboEV3/wiki/problem_and_coping
void *__dso_handle=0;

// using宣言
//using namespace ev3api;
using ev3api::ColorSensor;
using ev3api::GyroSensor;
using ev3api::TouchSensor;
using ev3api::SonarSensor;
using ev3api::Motor;
using ev3api::Clock;

//#define EYE_DEBUG

// Device objects
// オブジェクトを静的に確保する
ColorSensor gColorSensor (PORT_3);
GyroSensor  gGyroSensor  (PORT_4);
Motor       gLeftWheel   (PORT_C);
Motor       gRightWheel  (PORT_B);
Motor       gTailMotor   (PORT_A); //2017.07.28 k-ota add
TouchSensor gTouchSensor (PORT_1);
SonarSensor gSonarSensor (PORT_2);

//Clock       gClock;


enum Sys_Mode{
    SYS_INIT            = 110,
    BT_CONECT           = 120,
    DISPLAY_SELECT_APLI = 210,
    CALIB_COLOR_SENSOR  = 310,
    WAIT_FOR_START      = 410,
    START               = 510,
    RUN                 = 530,
    LINE_TRACE_MODE     = 520,
    GOAL                = 610,
    DANSA               = 710,
    GARAGE              = 810,
    LUP                 = 910,
    STOP                = 1010,
    RESET               = 0
};

Sys_Mode mSys_Mode;

static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt     = NULL;   /* Bluetoothファイルハンドル */

static Ang_Eye   *gAng_Eye;
static Ang_Brain *gAng_Brain;
static Ang_Robo  *gAng_Robo;
static Balancer  *gBalancer;

#ifdef LOG_RECORD
static int   log_size = 15000;
static int   log_cnt  = 0;
static int   log_dat_00[15000];
static int   log_dat_01[15000];
static int   log_dat_02[15000];
static int   log_dat_03[15000];
static float log_fdat_00[15000];
static float log_fdat_01[15000];    
static float log_fdat_02[15000];
#endif

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
//System Initialization
static void sys_initialize() {
  int  battery;
  char battery_str[32];

  ParamFileRead fileRead;
  fileRead.fileRead();
  fileRead.setParameters();

  // [TODO] タッチセンサの初期化に2msのdelayがあるため、ここで待つ
  tslp_tsk(2);
  mSys_Mode=SYS_INIT;
  
  // オブジェクトの作成
  gBalancer  = new Balancer();
  gAng_Eye   = new Ang_Eye(gColorSensor,
			   gLeftWheel,
			   gRightWheel,
			   gGyroSensor,
			   gSonarSensor);

  gAng_Brain = new Ang_Brain();
  gAng_Robo  = new Ang_Robo(gGyroSensor,
			    gLeftWheel,
			    gRightWheel,
			    gTailMotor,
			    gBalancer);
  gAng_Robo->tail_reset();
  gAng_Robo->tail_stand_up();

  ev3_speaker_set_volume(5);
  ev3_speaker_play_tone(NOTE_C4,200);

  battery = ev3_battery_voltage_mV();
  sprintf(battery_str, "Battery:%d", battery);

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("Right Line Trace",0, 20);

  ev3_lcd_draw_string(battery_str,0, 40);
  ev3_lcd_draw_string("Set ANG on GND",0, 60);
  ev3_lcd_draw_string("PUSH TS 4 RESET",0, 80);


  //  ev3_lcd_draw_string("Set the robot on the Ground",0, CALIB_FONT_HEIGHT*1);
  //  ev3_lcd_draw_string("Touch the T-Sensor for reset sensors",0, CALIB_FONT_HEIGHT*2);

  while(1){
    if (gTouchSensor.isPressed()){
      break; /* タッチセンサが押された */
    }
    tslp_tsk(10); //What dose it mean? kota 170812
  }
  ev3_speaker_play_tone(NOTE_E4,200);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

  gAng_Eye->init();   //reset gyro
  gAng_Robo->init();  //
  gAng_Brain->init(); //initialize mode

  /* Open Bluetooth file */
  bt = ev3_serial_open_file(EV3_SERIAL_BT);
  assert(bt != NULL);

  /* Bluetooth通信タスクの起動 */
  act_tsk(BT_TASK);

  // 初期化完了通知
  //  ev3_led_set_color(LED_ORANGE);
  ev3_led_set_color(LED_OFF);

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_speaker_play_tone(NOTE_C4,200);
}


//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
//Systen Destroy
static void sys_destroy(){
  delete gAng_Eye;
  delete gAng_Brain;
  delete gAng_Robo;
  delete gBalancer;
}

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
#ifdef LOG_RECORD
static void log_dat( ){

  log_dat_00[log_cnt]  = gAng_Brain->tail_mode_lflag;
  log_dat_01[log_cnt]  = gAng_Robo->balance_mode;
  log_dat_02[log_cnt]  = gAng_Eye->sonarDistance;
  log_dat_03[log_cnt]  = gAng_Brain->forward;
  log_fdat_00[log_cnt] = gAng_Eye->abs_angle;
  log_fdat_01[log_cnt] = 0;
  log_fdat_02[log_cnt] = gAng_Eye->odo;

  log_cnt++;
  if (log_cnt == log_size){
    log_cnt  = 0;
  }
}

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
static void export_log_dat( ){
    FILE* file_id;
    int battery = ev3_battery_voltage_mV();
    file_id = fopen( "log_dat.csv" ,"w");
    fprintf(file_id, "battery:%d\n",battery);
    fprintf(file_id, "cnt,tail_mode,balance,sonarDistance,forward_ord,angle,right_pwm,odo\n");
    int cnt;

    for(cnt = 0; cnt < log_size ; cnt++){
      fprintf(file_id, "%d,%d,%d,%d,%d,%f,%f,%f\n",cnt, log_dat_00[cnt],log_dat_01[cnt], log_dat_02[cnt],log_dat_03[cnt],log_fdat_00[cnt],log_fdat_01[cnt], log_fdat_02[cnt]);
    }
    fclose(file_id);
}
#endif

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
//Anago Eye Task
void eye_cyc(intptr_t exinf) {
    act_tsk(EYE_TASK);//0817 tada
}

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
void eye_task(intptr_t exinf) {

#ifdef LOG_RECORD
  log_dat();
#endif

  if (emergencyStop(gAng_Eye->velocity)) {
    wup_tsk(MAIN_TASK);
  }

  if (ev3_button_is_pressed(BACK_BUTTON)) {
    wup_tsk(MAIN_TASK);  // バックボタン押下
  } else {
    gAng_Eye->det_Line_Value();
    gAng_Eye->WheelOdometry(dT_4ms);
    gAng_Eye->det_Dansa();
    gAng_Eye->setSonarDistance();
    gAng_Brain->setEyeCommand(gAng_Eye->linevalue,
                              gAng_Eye->linevalue_LUG,
                              gAng_Eye->xvalue,
                              gAng_Eye->yvalue,
                              gAng_Eye->odo,
                              gAng_Eye->velocity,
                              gAng_Eye->yawrate,
                              gAng_Eye->abs_angle,
			      gTailMotor.getCount(),
			      gAng_Eye->robo_stop,
			      gAng_Eye->robo_forward,
			      gAng_Eye->robo_back,
			      gAng_Eye->robo_turn_left,
			      gAng_Eye->robo_turn_right,
                              gAng_Eye->dansa,
			      gAng_Eye->det_gray,
            gAng_Eye->sonarDistance
			      );//指令値をあなごの脳みそに渡す

    gAng_Brain->setRoboCommand(gAng_Robo->balance_mode);//指令値をあなごの脳みそに渡す
  }
  ext_tsk();
}

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
//Anago Brain Task
void brain_cyc(intptr_t exinf) {
    act_tsk(BRAIN_TASK); //0817 tada
}

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
void brain_task(intptr_t exinf) {

    if (ev3_button_is_pressed(BACK_BUTTON)) {
        wup_tsk(MAIN_TASK);  // バックボタン押下

    } else {

      gAng_Brain->SetSysMode(static_cast<int>(mSys_Mode));
      gAng_Brain->run();//あなご脳みそ計算スタート
      gAng_Robo->setCommand(gAng_Brain->forward,
                            gAng_Brain->yawratecmd,
                            gAng_Brain->anglecommand,
                            gAng_Eye->yawrate,
                            gAng_Brain->tail_mode_lflag);//指令値をあなご手足に渡す
    }
    ext_tsk();
}

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
//Anago Robo(Teashi) Task
void robo_cyc(intptr_t exinf) {
    act_tsk(ROBO_TASK);

}

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
void robo_task(intptr_t exinf) {

#ifdef LOG_RECORD  
  if (gTouchSensor.isPressed()){
    wup_tsk(MAIN_TASK);
  }
#endif

  if (ev3_button_is_pressed(BACK_BUTTON)) {
    wup_tsk(MAIN_TASK);  // バックボタン押下
  } else {
    gAng_Robo->run();
    //kota 0811      gAng_Robo->saveData(500);
  }
  ext_tsk();
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1){
      uint8_t c = fgetc(bt); /* 受信 */
      switch(c){
      case '1':
	bt_cmd = 1;
	break;

      case '0':
	ev3_speaker_play_tone(NOTE_C4,200);
	ev3_led_set_color(LED_GREEN);
	break;

      default:
	ev3_led_set_color(LED_OFF);
	break;
      }
      fputc(c, bt); /* エコーバック */
    }
}


//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
//Main Task
void main_task(intptr_t unused) {
  sys_initialize();
  calibration C_calib(gColorSensor, gTouchSensor, gAng_Robo);
  //calibrate color sensor and set threshold of anago eye
  mSys_Mode = CALIB_COLOR_SENSOR;

  calibrationData calib = C_calib.set_calibration();
  gAng_Eye->set_White_Black_Threshold(calib.white,calib.black,calib.white_slant,calib.black_slant);

  //REDAY for START
  gAng_Robo->tail_reset();
  gAng_Robo->tail_stand_up();

  mSys_Mode = WAIT_FOR_START;

  ev3_sta_cyc(EYE_CYC);
  ev3_sta_cyc(BRAIN_CYC);
  
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("Set ANG on Start Line",0, 40);
  ev3_lcd_draw_string("PRESS TS or 1",0, 80);
  while(1){

    if(ev3_bluetooth_is_connected()){
      ev3_lcd_draw_string("BT connected",0, 60);
    }else{
      ev3_lcd_draw_string("BT unconnected",0, 60);
    }

    if (bt_cmd == 1){
      break; /* リモートスタート */
    }
    if (gTouchSensor.isPressed()){
      break; /* タッチセンサが押された */
    }
    tslp_tsk(10); //What dose it mean? kota 170812
  }
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_led_set_color(LED_OFF);
  mSys_Mode=START;

  //Start Dash sequence from here to robo_task.  
  while(gTailMotor.getCount() <= 100){
    tslp_tsk(50);
    gAng_Robo->tail_control(TAIL_ANGLE_STAND_UP); //0819 changed by tada. original is 120
    TAIL_ANGLE_STAND_UP++;
  }
  
  ev3_sta_cyc(ROBO_CYC);
  ter_tsk(BT_TASK);
  slp_tsk();  // バックボタンが押されるまで待つ

  ev3_stp_cyc(EYE_CYC);
  ev3_stp_cyc(BRAIN_CYC);
  ev3_stp_cyc(ROBO_CYC);

  gLeftWheel.~Motor();
  gRightWheel.~Motor();
  gTailMotor.~Motor();

  ev3_led_set_color(LED_ORANGE);
  ev3_lcd_set_font(EV3_FONT_SMALL);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_draw_string("Stop",0, CALIB_FONT_HEIGHT*1);

#ifdef LOG_RECORD
  ev3_lcd_draw_string("Saving Log Data",0, CALIB_FONT_HEIGHT*2);
  export_log_dat( );
  ev3_lcd_draw_string("Saving Log Data is done",0, CALIB_FONT_HEIGHT*3);
#endif

#ifdef EYE_DEBUG
  ev3_lcd_draw_string("Saving Log Data",0, CALIB_FONT_HEIGHT*2);
  gAng_Eye->export_dat( );
  ev3_lcd_draw_string("Saving Log Data is done",0, CALIB_FONT_HEIGHT*3);
#endif
  ev3_led_set_color(LED_OFF);

  sys_destroy();
  ext_tsk();
}// end::main_task

