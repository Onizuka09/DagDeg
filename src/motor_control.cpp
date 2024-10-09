#include "motor_control.h"
void init_motors(){ 
      // Initialize motor pins
    pinMode(motorLeftPin1, OUTPUT);
    pinMode(motorLeftPin2, OUTPUT);
    pinMode(motorRightPin1, OUTPUT);
    pinMode(motorRightPin2, OUTPUT);

    // Initialize PWM channels
    ledcSetup(0, pwmFrequency, pwmResolution);
    ledcAttachPin(motorENA, 0);
    ledcSetup(1, pwmFrequency, pwmResolution);
    ledcAttachPin(motorENB, 1);

}
void moveForward() {
    ledcWrite(0, pwmMaxValue);
    ledcWrite(1, pwmMaxValue);
    digitalWrite(motorLeftPin1,0);     
    digitalWrite(motorLeftPin2,1);     
    digitalWrite(motorRightPin1,1);     
    digitalWrite(motorRightPin2,0);     

}

void moveLeft(){ 
    ledcWrite(0, pwmMaxValue);
    ledcWrite(1, pwmMaxValue);
    digitalWrite(motorLeftPin1,0);     
    digitalWrite(motorLeftPin2,0);     
    digitalWrite(motorRightPin1,1);     
    digitalWrite(motorRightPin2,0);   
}
void moveRight(){ 
    ledcWrite(0, pwmMaxValue);
    ledcWrite(1, pwmMaxValue);
    digitalWrite(motorLeftPin1,1);     
    digitalWrite(motorLeftPin2,0);     
    digitalWrite(motorRightPin1,0);     
    digitalWrite(motorRightPin2,0);
}
void stopMotors() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
}
