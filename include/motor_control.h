#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_
#include <Arduino.h>

// pin definition             
const int motorLeftPin1 = 22; 
const int motorLeftPin2 = 21; 
const int motorRightPin1 = 19; 
const int motorRightPin2 = 18; 

const int motorENA = 23; 
const int motorENB = 17; 
const int pwmFrequency = 5000;
const int pwmResolution = 8; // 8-bit PWM resolution (0-255)
const int pwmMaxValue = 200; // 100 min speed // 150 medium 

void moveLeft(); 
void moveRight();
void moveForward(); 
void stopMotors();
void init_motors();


#endif 
