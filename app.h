#ifdef __cplusplus
extern "C" {
#endif

#include "ev3api.h"
#include "parameter.h"

#define ROBO_PRIORITY    TMIN_APP_TPRI + 3
#define MAIN_PRIORITY    TMIN_APP_TPRI + 4
#define EYE_PRIORITY     TMIN_APP_TPRI + 1
#define BRAIN_PRIORITY   TMIN_APP_TPRI + 2
#define BT_PRIORITY      TMIN_APP_TPRI + 5

#ifndef STACK_SIZE
#define STACK_SIZE      4096
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

/* LCDフォントサイズ */
#define CALIB_FONT        (EV3_FONT_MEDIUM)
#define CALIB_FONT_WIDTH  (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (20/*TODO: magic number*/)
#define LOG_RECORD

extern void main_task(intptr_t exinf);
extern void bt_task(intptr_t exinf); //17.07.28 k-ota add

extern void eye_task(intptr_t exinf);
extern void eye_cyc(intptr_t exinf);

extern void robo_task(intptr_t exinf);
extern void robo_cyc(intptr_t exinf);
	
extern void brain_task(intptr_t exinf);
extern void brain_cyc(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
