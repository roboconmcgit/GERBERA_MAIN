/**
 ******************************************************************************
 ** ファイル名 : lcdDraw.cpp
 **
 ** 概要 : LCD描画クラス
 **
 ** 注記 : 各種初期パラメータは_prmファイル参照
 ******************************************************************************
 **/

#include "lcdDraw.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//*****************************************************************************
// 関数名 : コンストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
lcdDraw::lcdDraw(){

}

//*****************************************************************************
// 関数名 : デストラクタ
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
lcdDraw::~lcdDraw(){

}

//*****************************************************************************
// 関数名 : lcdDrawExe
// 引数 : unused
// 返り値 : なし
// 概要 : 
//*****************************************************************************
void lcdDraw::lcdDrawExe(char *buf ,int8_t lineNum){
	ev3_lcd_draw_string(buf, 0, CALIB_FONT_HEIGHT*lineNum);
	if(lcd_line_count > lcd_line_max){
		lcd_line_count=0;
	}
}

//*****************************************************************************
// 関数名 : LCD_DRAW
// 引数 : char *buf
// 返り値 : なし
// 概要 : LCDに描画する
//*****************************************************************************
void lcdDraw::LCD_DRAW( char *buf ,int8_t lineNum) {
	/* LCD画面表示 */
	lcd_line_count++;
	char buf_tmp[100];
	int8_t line_count_tmp = lcd_line_count;
	if((lineNum != -1)&&(lineNum > 0)&&(lineNum <= 15)) line_count_tmp = lineNum;
	sprintf(buf_tmp, " %d | %s", line_count_tmp, buf);
	lcdDrawExe(buf_tmp, line_count_tmp);
}

//*****************************************************************************
// 関数名 : LCD_DRAW_DATA_INT32
// 引数 : char *buf
// 返り値 : なし
// 概要 : LCDに描画する
//*****************************************************************************
void lcdDraw::LCD_DRAW_DATA_INT32( char *buf, int32_t data ,int8_t lineNum) {
	/* LCD画面表示 */
	lcd_line_count++;
	char buf_tmp[100];
	int8_t line_count_tmp = lcd_line_count;
	if((lineNum != -1)&&(lineNum > 0)&&(lineNum <= 15)) line_count_tmp = lineNum;
	sprintf(buf_tmp, " %d | %s : %ld", line_count_tmp, buf, data);
	lcdDrawExe(buf_tmp, line_count_tmp);
}

//*****************************************************************************
// 関数名 : LCD_DRAW_DATA_FLOAT
// 引数 : char *buf
// 返り値 : なし
// 概要 : LCDに描画する
//*****************************************************************************
void lcdDraw::LCD_DRAW_DATA_FLOAT( char *buf, float data ,int8_t lineNum) {
	/* LCD画面表示 */
	lcd_line_count++;
	char buf_tmp[100];
	int8_t line_count_tmp = lcd_line_count;
	if((lineNum != -1)&&(lineNum > 0)&&(lineNum <= 15)) line_count_tmp = lineNum;
	sprintf(buf_tmp, " %d | %s : %g", line_count_tmp, buf, data);
	lcdDrawExe(buf_tmp, line_count_tmp);
}
