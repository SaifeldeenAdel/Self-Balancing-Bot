#ifndef MOTOR_FUNCTIONS
#define MOTOR_FUNCTIONS

#include <Arduino.h>

// Each function takes in the speed and the pins that each motor is connected to
void clockwise(int motor_speed, int left_motor, int right_motor);
void anticlockwise(int motor_speed, int left_motor, int right_motor);

// Function to stop both motors, takes in motor pin numbers
void halt(int left_motor, int right_motor);

#endif
