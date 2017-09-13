#ifndef MY_UNTL_H_
#define MY_UNTL_H_

#include "ev3api.h"

extern void init_f(const char *str);
extern void msg_f(const char *str, int32_t line);
extern int emergencyStop(float velocity);

class PID{
public:
  explicit PID();
  void init_pid(float kp, float ki, float kd, float dT);
  int  calc_pid(float command, float contrvalue);
  int  sat(int max, int min, int inputvalue);

private:
  float KP=0;
  float TI=0;
  float TD=0;
  float DT=1;

  int error_old=0;
  int error_P_old=0;		//過去の偏差
};

#endif  // MY_UNTL_H_
