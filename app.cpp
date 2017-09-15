//Anago System
//Date:2017.7.25
//Author:Kaoru Ota


#include "util.h"
#include "ParamFileRead.h"
#include "ev3api.h"
#include "app.h"
#include "Clock.h"

#include "ColorParts.h"
#include "MotorParts.h"
#include "GyroParts.h"
#include "SonarParts.h"
#include "TouchParts.h"

// コントローラクラス
#include "Controller.h"

// キャリブレーション
#include "calibration.h"

// デストラクタ問題の回避
// https://github.com/ETrobocon/etroboEV3/wiki/problem_and_coping
void *__dso_handle=0;

// using宣言
static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt     = NULL;   /* Bluetoothファイルハンドル */

static ColorParts  *gColorParts;
static MotorParts  *gMotorParts;
static GyroParts   *gGyroParts;
static SonarParts  *gSonarParts;
static TouchParts  *gTouchParts;

static Controller  *gController;

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
static float log_fdat_03[15000];
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
  int gyro = 0;
  char gyro_str[32];

  ParamFileRead fileRead;
  fileRead.fileRead();
  fileRead.setParameters();

  // [TODO] タッチセンサの初期化に2msのdelayがあるため、ここで待つ
  tslp_tsk(2);
  
  // オブジェクトの作成
  gColorParts = new ColorParts();
  gMotorParts = new MotorParts();
  gGyroParts  = new GyroParts();
#ifdef RIGHT_MODE
  gSonarParts = new SonarParts();
#endif
  gTouchParts = new TouchParts();

  gController = new Controller(gColorParts,gMotorParts,gGyroParts,gSonarParts,gTouchParts);

  gController->mSys_Mode=SYS_INIT;

  gMotorParts->tail_reset();
  gMotorParts->tail_stand_up();

  ev3_speaker_set_volume(5);
  ev3_speaker_play_tone(NOTE_C4,200);

  battery = ev3_battery_voltage_mV();
  sprintf(battery_str, "Battery:%d", battery);

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
#ifdef RIGHT_MODE
  ev3_lcd_draw_string("Right Line Trace",0, 20);
#else
  ev3_lcd_draw_string("Left Line Trace",0, 20);
#endif

  ev3_lcd_draw_string(battery_str,0, 40);
  ev3_lcd_draw_string("Set ANG on GND",0, 60);
  ev3_lcd_draw_string("PUSH TS 4 RESET",0, 80);

  while(1){
    if (gTouchParts->GetTouchPartsData()){
      gGyroParts->GyroPartsReset();
      gController->ControllerInit();
      break; /* タッチセンサが押された */
    }
    gyro = gGyroParts->GetGyroPartsData();  // ジャイロセンサ値
    sprintf(gyro_str, "Gyro:%d", gyro);
    ev3_lcd_draw_string(gyro_str,0, 100);

    tslp_tsk(10); //What dose it mean? kota 170812
  }
  ev3_speaker_play_tone(NOTE_E4,200);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

  tslp_tsk(500);
  
  while(1){
    if (gTouchParts->GetTouchPartsData()){
      break; /* タッチセンサが押された */
    }
    gyro = gGyroParts->GetGyroPartsData();  // ジャイロセンサ値
    sprintf(gyro_str, "Gyro:%d", gyro);
    ev3_lcd_draw_string(gyro_str,0, 100);
    tslp_tsk(20); //What dose it mean? kota 170812
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  }
  ev3_speaker_play_tone(NOTE_E4,200);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    

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
  delete gColorParts;
  delete gMotorParts;
  delete gGyroParts;
#ifdef RIGHT_MODE
  delete gSonarParts;
#endif
  delete gTouchParts;
  delete gController;
}

//*****************************************************************************
// 関数名 : 
// 引数 : 
// 返り値 : 
// 概要 : 
//*****************************************************************************
#ifdef LOG_RECORD
static void log_dat( ){

  log_dat_00[log_cnt]  = gColorParts->linevalue;
  log_dat_01[log_cnt]  = gGyroParts->GetGyroPartsData();
  log_dat_02[log_cnt]  = gGyroParts->gyro_250d[0];
  log_dat_03[log_cnt]  = gController->gCruiseCtrl->mForward;
  log_fdat_00[log_cnt] = gController->gCruiseCtrl->offset;;
  log_fdat_01[log_cnt] = gController->gCruiseCtrl->mYawratecmd;
  log_fdat_02[log_cnt] = gController->gCruiseCtrl->mYawrate;  
  log_fdat_03[log_cnt] = gController->gCruiseCtrl->mTurn;  

  log_cnt++;
  if (log_cnt == log_size){
    log_cnt  = 0;
  }
}

//*****************************************************************************
static void export_log_dat( ){
    FILE* file_id;
    int battery = ev3_battery_voltage_mV();
    file_id = fopen( "log_dat.csv" ,"w");
    fprintf(file_id, "battery:%d\n",battery);
    fprintf(file_id, "cnt,linevalue,gyro,gyro2,offset,mYawratecmd,mYawrate,mTurn\n");
    int cnt;

    for(cnt = 0; cnt < log_cnt ; cnt++){
      fprintf(file_id, "%d,%d,%d,%d,%f,%f,%f,%f\n",cnt, log_dat_00[cnt],log_dat_01[cnt], log_dat_02[cnt],log_fdat_00[cnt],log_fdat_01[cnt], log_fdat_02[cnt],log_fdat_03[cnt]);
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
  if (gTouchParts->GetTouchPartsIsTouch()){
    wup_tsk(MAIN_TASK);
  }
#endif
  if (emergencyStop(gMotorParts->velocity)) {
    //wup_tsk(MAIN_TASK);
  }

  if (ev3_button_is_pressed(BACK_BUTTON)) {
    wup_tsk(MAIN_TASK);  // バックボタン押下
  } else {
    gColorParts->ColorPartsTask();
    gMotorParts->WheelOdometry(dT_4ms);
    gGyroParts->GyroPartsTask();
#ifdef RIGHT_MODE
    gSonarParts->SonarPartsTask();
#endif
    gTouchParts->TouchPartsTask();
#if 0
    char r_char[256];
    char l_char[256];
    sprintf(r_char, "R_PWM:%6ld", gMotorParts->getMotorPartsPwm(MOTORPARTS_RIGHT_NO));
    ev3_lcd_draw_string(r_char,0, 20);
    sprintf(l_char, "L_PWM:%6ld", gMotorParts->getMotorPartsPwm(MOTORPARTS_LEFT_NO));
    ev3_lcd_draw_string(l_char,0, 40);
#else
    gController->ControllerOperation();
#endif
    
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
if (gTouchParts->GetTouchPartsIsTouch()){
  wup_tsk(MAIN_TASK);
}
#endif

if (ev3_button_is_pressed(BACK_BUTTON)) {
  wup_tsk(MAIN_TASK);  // バックボタン押下
} else {
  gController->run();
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
  //calibrate color sensor and set threshold of anago eye
  gController->mSys_Mode = CALIB_COLOR_SENSOR;
  
  calibration calib(gColorParts, gTouchParts, gMotorParts);
  int error = calib.set_calibration();
  if(error == -1){
    ev3_lcd_draw_string("Calibration Error",0, 40);
  }
  //REDAY for START
  gMotorParts->tail_reset();
  gMotorParts->tail_stand_up();

  gController->mSys_Mode = WAIT_FOR_START;

  ev3_sta_cyc(EYE_CYC);
  //ev3_sta_cyc(BRAIN_CYC);
  
  ev3_lcd_draw_string("Set GERBERA on Start Line",0, 40);
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
    if (gTouchParts->GetTouchPartsData()){
      break; /* タッチセンサが押された */
    }
    tslp_tsk(10); //What dose it mean? kota 170812
  }
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_led_set_color(LED_OFF);
  gController->mSys_Mode=START;

  //Start Dash sequence from here to robo_task.  
  while(gMotorParts->getMotorPartsPwm(MOTORPARTS_TAIL_NO) <= 100){
    tslp_tsk(50);
    gMotorParts->tail_control(TAIL_ANGLE_STAND_UP); //0819 changed by tada. original is 120
    TAIL_ANGLE_STAND_UP++;
  }
  
  ev3_sta_cyc(ROBO_CYC);
  ter_tsk(BT_TASK);
  slp_tsk();  // バックボタンが押されるまで待つ

  gController->mSys_Mode = STOP;
  ev3_stp_cyc(EYE_CYC);
  //ev3_stp_cyc(BRAIN_CYC);
  ev3_stp_cyc(ROBO_CYC);

  gMotorParts->stopMotorPartsLeftRight();

  ev3_led_set_color(LED_ORANGE);
  ev3_lcd_set_font(EV3_FONT_SMALL);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_draw_string("Stop",0, CALIB_FONT_HEIGHT*1);

#ifdef LOG_RECORD
  ev3_lcd_draw_string("Saving Log Data",0, CALIB_FONT_HEIGHT*2);
  export_log_dat( );
  ev3_lcd_draw_string("Saving Log Data is done",0, CALIB_FONT_HEIGHT*3);
#endif

  ev3_led_set_color(LED_OFF);

  sys_destroy();
  ext_tsk();
}// end::main_task

