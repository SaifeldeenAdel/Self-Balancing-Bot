#ifndef PID_H
#define PID_H

#include <Arduino.h>

struct PIDSystem {
  int target;
  double rollFeedback;
  double kp;
  double ki;
  double kd;
  double lastError;
  double errorSum;
  long lastTime;
  double pidOutput;
  
 
};

void initPID(struct PIDSystem sys,int target,double rollFeedback,double pidOutput ,double kp, double ki, double kd);
void computePID(struct PIDSystem sys, double rollFeedback);

#endif
