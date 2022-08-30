#include "pid.h"

void initPID(struct PIDSystem sys,int target,double rollFeedback,double pidOutput ,double kp, double ki, double kd) {
   sys.target = target;
   sys.rollFeedback = rollFeedback;
   sys.pidOutput = pidOutput;
   sys.kp = kp;
   sys.ki = ki;
   sys.kd = kd;
   sys.errorSum = 0;
   sys.lastError = 0;
   sys.lastTime = millis();
   
}


void computePID(struct PIDSystem sys, double rollFeedback){
  long currentTime = millis();
  sys.rollFeedback = rollFeedback;
  double error = sys.rollFeedback - sys.target;
  double errorSlope = (error - sys.lastError) / (currentTime - sys.lastTime);
  sys.errorSum += error;
  sys.lastError = error;
  sys.lastTime = currentTime;

  sys.pidOutput = (sys.kp * error) + (sys.ki * sys.errorSum) + (sys.kd * errorSlope);

}
