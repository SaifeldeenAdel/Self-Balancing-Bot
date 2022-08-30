#ifndef MPU_FUNCTIONS
#define MPU_FUNCTIONS

#include <Arduino.h>
#include <Wire.h>

// Defining the address of the MPU as a constant to be used in the functions
const int MPU_address = 0x68;
 
// MPU Function prototypes
void setupMPU();
void requestAccData();
void requestGyroData();

#endif
