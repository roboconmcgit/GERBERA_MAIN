/*
 *      LCD表示のヘッダファイル
 */

#ifndef LCDDRAW_H_
#define LCDDRAW_H_

#include "ev3api.h"

#define lcd_line_max 10
/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/*
 *  関数のプロトタイプ宣言
 */
 
class lcdDraw{
private:
    int8_t lcd_line_count = 0;
    void lcdDrawExe( char *buf, int8_t lineNum);

public:
    lcdDraw();                       //コンストラクタ
    ~lcdDraw();                      //デストラクタ
    void LCD_INIT(){
        lcd_line_count = 0;
        ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);   
    };
    void LCD_DRAW( char *buf, int8_t lineNum = -1);
    void LCD_DRAW_DATA_INT32( char *buf, int32_t data, int8_t lineNum = -1);
    void LCD_DRAW_DATA_FLOAT( char *buf, float data, int8_t lineNum = -1);
};

#endif // !LCDDRAW_H_
