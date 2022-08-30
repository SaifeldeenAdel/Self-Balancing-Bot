#include <Wire.h>
#include "mpu_func.h"
#include "motor_func.h"
#include "pid.h"

// Motor variable declaration
int left_motor = 5;
int right_motor =6;
int motor_speed = 10;

// MPU variable declaration
double gForceX, gForceY, gForceZ; // Accelerometer data converted to g force units
double gyroX, gyroY, gyroZ; // Gyroscope data converted to degrees
double accAngleX, gyroAngleX;
double roll;
double elapsedTime, currentTime, previousTime;


// PID variable declaration
double kp=25;
double ki=0;
double kd=0.8;

int target= 0;
double pidOutput=0;
double rollFeedback=0;

struct PIDSystem pid;

void setup() {
  
  // Function for waking up MPU and setting it up
  setupMPU();
  initPID(pid, target, rollFeedback, pidOutput, kp, ki, kd);

  // For gyro calculations later - not PID
  currentTime = millis();
}


void loop() {
  
  // Running function that gets the MPU ready for reading Accelerometer data
  requestAccData();
  
   // Reading 2 bytes for each axis, dividing by LSB sensitivity according to datasheet
  gForceX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  gForceY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  gForceZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  
  // We only need to calculate the roll for our project as there is no pitching
  // Roll angle using accel data
  accAngleX = (atan(gForceX / sqrt(pow(gForceY, 2) + pow(gForceZ, 2))) * 180 / PI);


  // Running function that gets the MPU ready for reading Gyroscope data
  requestGyroData() ;
  
  // Doing the same thing as accelerometer, getting data as degrees per second
  gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Getting current and previous time values for gyro calculations
  previousTime = currentTime;  
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  
  // Roll angle using integrations of gyro data while converting from deg/s to degrees 
  gyroAngleX += gyroX * elapsedTime;
  
  // Complementary filter combining both gyroscope and accelerometer data to deal with noise and other drifts
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;

  
  // Runs function that computes a new output using roll feedback value and pid
  computePID(pid, roll);

  // The absolute value of the pid output is the motor speed
  motor_speed = abs(pid.pidOutput);


  // Call apropriate function according to roll value
  
  if(roll < 0){
    anticlockwise(motor_speed, left_motor, right_motor);
  } else if (roll > 0) {
    clockwise(motor_speed, left_motor, right_motor);
  } else if (roll == 0) {
    halt(left_motor, right_motor);
  }
  
}
