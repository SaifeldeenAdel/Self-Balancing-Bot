#include "mpu_func.h"


void setupMPU () {
  Wire.begin();
  Wire.beginTransmission(MPU_address); // Beginning transmission on 0x68, the address of the MPU according to the datasheet
  Wire.write(0x6B);
  Wire.write(0); // Waking up the MPU by writing a 0 byte to the power management register at 0x6B
  Wire.endTransmission();
  // Accelerometer and gyroscope full scale ranges are by default set to ±2g and ±250deg respectively so you dont need to change that
}

void requestAccData() {
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B); // Setting the address I want to receive data from. Accelerometer data starts from 0x3B
  Wire.endTransmission();
  Wire.requestFrom(MPU_address, 6); // Requesting 6 bytes as accelerometer data is 6 bytes long (2 bytes for each axis)
}

void requestGyroData() {
  Wire.beginTransmission(MPU_address);
  Wire.write(0x43); // Setting gyroscope starting address
  Wire.endTransmission();
  Wire.requestFrom(MPU_address, 6); 
}
