#include "motor_func.h"

// Each function takes in the speed and the pins that each motor is connected to
void clockwise(int motor_speed, int left_motor, int right_motor)
{
  analogWrite(right_motor,motor_speed);
  analogWrite(left_motor,motor_speed);
}

void anticlockwise(int motor_speed, int left_motor, int right_motor)
{
  analogWrite(right_motor,motor_speed * -1);
  analogWrite(left_motor,motor_speed * -1);
}


// Function to stop both motors, takes in motor pin numbers
void halt(int left_motor, int right_motor)
{
  analogWrite(right_motor,0);
  analogWrite(left_motor,0);
}
