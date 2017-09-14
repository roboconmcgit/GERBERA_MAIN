
#include "app.h"
#include "calibration.h"

#include "parameter.h"
//*****************************************************************************
// 関数名 : set_calibration
// 引数 : unused
// 返り値 : なし
// 概要 :
// Copy from 3-apex 2016
//*****************************************************************************
int calibration::set_calibration(){
  COLORPARTS_CALIBRA color_calib;
  int error = 0;
//0729 kota Color Sensor Calibration
  color_calib.Offset      = 0;
  color_calib.white       = 60;
  color_calib.black       = 2;
  color_calib.white_slant = 12;
  color_calib.black_slant = 2;

  unsigned char calib_flag1=0;
  unsigned char calib_flag2=0;
#ifdef RIGHT_MODE
  unsigned char calib_flag3=0;
  unsigned char calib_flag4=0;
#endif
  unsigned char calib_ref;
  char s[20]={'\0'};

  ev3_speaker_play_tone(NOTE_C4,200);
  tslp_tsk(150);
  ev3_speaker_play_tone(NOTE_E4,200);
  tslp_tsk(150);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_SMALL);
  ev3_lcd_draw_string("Calib BLACK", 0, CALIB_FONT_HEIGHT*1);

  while(1){
    if (calib_flag1) break;
    if (gTouchParts->GetTouchPartsData()){
      calib_flag2=1;
      break;
    }
    //    calib_ref=ev3_color_sensor_get_reflect(color_sensor);
    calib_ref=gColorParts->GetColorPartsData();
    sprintf(s,"BLACK : %2d",calib_ref);
    ev3_lcd_draw_string(s, 0, CALIB_FONT_HEIGHT*2);
    ev3_lcd_draw_string("To capture : E",0, CALIB_FONT_HEIGHT*3);
    ev3_lcd_draw_string("To escape : TOUCH",0, CALIB_FONT_HEIGHT*4);

    if (ev3_button_is_pressed(ENTER_BUTTON)){
      color_calib.black=calib_ref;
      ev3_lcd_draw_string("NG : L, OK : R", 0, CALIB_FONT_HEIGHT*3);
      while(1){
    if (ev3_button_is_pressed(RIGHT_BUTTON)){
      calib_flag1=1;
      break;
    }
    if (ev3_button_is_pressed(LEFT_BUTTON)) break;
    tslp_tsk(10);
      }
    }
    tslp_tsk(50);
  }//kokomade wa OK

  ev3_speaker_play_tone(NOTE_F5,200);
  tslp_tsk(100);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_draw_string("Calib WHITE", 0, CALIB_FONT_HEIGHT*1);
  ev3_lcd_draw_string("                       ", 0, CALIB_FONT_HEIGHT*4);

  while(1){
    if (calib_flag2) break;
    if (gTouchParts->GetTouchPartsData()){
      break;
    }
    calib_ref=gColorParts->GetColorPartsData();
    sprintf(s,"WHITE : %2d",calib_ref);
    ev3_lcd_draw_string(s, 0, CALIB_FONT_HEIGHT*2);
    ev3_lcd_draw_string("To capture : E",0, CALIB_FONT_HEIGHT*3);

    if (ev3_button_is_pressed(ENTER_BUTTON)){
      color_calib.white=calib_ref;
      ev3_lcd_draw_string("NG : L, OK : R", 0, CALIB_FONT_HEIGHT*3);
      while(1){
    if (ev3_button_is_pressed(RIGHT_BUTTON)){
      calib_flag2=1;
      break;
    }
    if (ev3_button_is_pressed(LEFT_BUTTON)) break;
    tslp_tsk(10);
      }
    }
    tslp_tsk(50);
  }
#ifdef RIGHT_MODE
  gMotorParts->tail_reset();
  ev3_speaker_play_tone(NOTE_G5,200);
  tslp_tsk(100);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_draw_string("Calib BLACK_SLANT", 0, CALIB_FONT_HEIGHT*1);
  ev3_lcd_draw_string("                       ", 0, CALIB_FONT_HEIGHT*4);

  while(1){
    if (calib_flag3) break;
    if (gTouchParts->GetTouchPartsData()){
      calib_flag4=1;
      break;
    }
    gMotorParts->tail_control(TAIL_ANGLE_LUG);


    calib_ref=gColorParts->GetColorPartsData();
    sprintf(s,"BLCK_SLANT : %2d",calib_ref);
    ev3_lcd_draw_string(s, 0, CALIB_FONT_HEIGHT*2);
    ev3_lcd_draw_string("To capture : E",0, CALIB_FONT_HEIGHT*3);

    if (ev3_button_is_pressed(ENTER_BUTTON)){
      color_calib.black_slant=calib_ref;
      ev3_lcd_draw_string("NG : L, OK : R", 0, CALIB_FONT_HEIGHT*3);
      while(1){
    gMotorParts->tail_control(TAIL_ANGLE_LUG);

    if (ev3_button_is_pressed(RIGHT_BUTTON)){
      calib_flag3=1;
      break;
    }
    if (ev3_button_is_pressed(LEFT_BUTTON)) break;
    tslp_tsk(50);
      }
    }
    tslp_tsk(50);
  }

  ev3_speaker_play_tone(NOTE_A5,200);
  tslp_tsk(100);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_draw_string("Calib BLACK_SLANT", 0, CALIB_FONT_HEIGHT*1);
  ev3_lcd_draw_string("                       ", 0, CALIB_FONT_HEIGHT*4);

  while(1){
    if (calib_flag4) break;
    if (gTouchParts->GetTouchPartsData()){
      break;
    }
    gMotorParts->tail_control(TAIL_ANGLE_LUG);

    calib_ref=gColorParts->GetColorPartsData();
    sprintf(s,"WHITE_SLANT : %2d",calib_ref);
    ev3_lcd_draw_string(s, 0, CALIB_FONT_HEIGHT*2);
    ev3_lcd_draw_string("To capture : E",0, CALIB_FONT_HEIGHT*3);

    if (ev3_button_is_pressed(ENTER_BUTTON)){
      color_calib.white_slant=calib_ref;
      ev3_lcd_draw_string("NG : L, OK : R", 0, CALIB_FONT_HEIGHT*3);
      while(1){
    gMotorParts->tail_control(TAIL_ANGLE_LUG);

    if (ev3_button_is_pressed(RIGHT_BUTTON)){
      calib_flag4=1;
      break;
    }
    if (ev3_button_is_pressed(LEFT_BUTTON)) break;
    tslp_tsk(50);
      }
    }
    tslp_tsk(50);
  }
#endif

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  sprintf(s,"BLACK : %2d",color_calib.black);
  ev3_lcd_draw_string(s, 0, CALIB_FONT_HEIGHT*1);
  sprintf(s,"WHITE : %2d",color_calib.white);
  ev3_lcd_draw_string(s, 0, CALIB_FONT_HEIGHT*2);
#ifdef RIGHT_MODE
  sprintf(s,"BLACK_SLANT : %2d",color_calib.black_slant);
  ev3_lcd_draw_string(s, 0, CALIB_FONT_HEIGHT*3);
  sprintf(s,"WHITE_SLANT : %2d",color_calib.white_slant);
  ev3_lcd_draw_string(s, 0, CALIB_FONT_HEIGHT*4);
#endif
  ev3_speaker_play_tone(NOTE_A4,200);
  tslp_tsk(150);
  ev3_speaker_play_tone(NOTE_C5,200);

  gColorParts->ColorPartsCalibra(color_calib);
  return (error);
}
