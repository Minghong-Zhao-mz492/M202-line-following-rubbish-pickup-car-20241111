#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <Adafruit_MotorShield.h>

// Constants
const float WHEEL_RADIUS = 4.0;  // Wheel radius in cm
const float COMPUTER_ACTUAL_SPEED_FACTOR = 0.1;  // Calibration factor

// Motor Shield and Motor objects
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *leftMotor;
extern Adafruit_DCMotor *rightMotor;

// Function declarations
void initializeMotors();
void runLeftMotor(char direction, int speed, float distance);
void runRightMotor(char direction, int speed, float distance);
float calculateRealSpeed(int speed);
float calculateRunTime(float distance, float real_speed);

#endif
