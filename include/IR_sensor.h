#ifndef PID_CONF_H_
#define PID_CONF_H_
#include <Arduino.h>
#include<QTRSensors.h>
#include "pin_conf.h"
#include<PID_v1.h>
#define IR_PIN_COUNT 8
#define EMIT_PIN 16
const uint8_t IR_ARRAY[8] = {26,25,33,32,35,34,39,36 };
const uint8_t MR_IR= 27; 
const uint8_t ML_IR= 14; 
const uint8_t D_IR= 13; 

class IR_sensor{ 
public:
    uint16_t THRESHOLD = 3500; // 4000;  
    uint16_t _IR_Value[IR_PIN_COUNT]={0};
    uint16_t MR_IR_Val=0; 
    uint16_t ML_IR_Val=0; 
    uint16_t MR_IR_Val_old=0; 
    uint16_t ML_IR_Val_old=0; 
    uint16_t D_IR_Val=0; 


public:
    IR_sensor(); 
    void IR_sensor_init(const uint8_t* IR_pins, const uint8_t emit_pin); 
    ~IR_sensor();
    // void followLine(int sensorValues[], bool inverse , int bias ) ;
    void Read_sensor(); 
    void Print_sensor_values();
    void followLine( bool inverse = false, int bias_R = 0, int bias_L=0); 
    void init_pid();
    void print_PID_output();
    int compute_pid(int);
    void print_pid_values(); 
public:
    int offset = 90; 
    double  Kp = 0.004;
    double  Kd = 0.00036; // 0.00036 
    double  Ki = 0.00001;
private: 
    QTRSensors qtr; 
    uint8_t _IR_pins[IR_PIN_COUNT];
    uint8_t _emit_pin=0; 
  
    double setPoint = 0;
    double input, output;
    int leftMotorSpeed = 0 ;   
    int rightMotorSpeed =0 ; 
    int error=0;
    unsigned long previous_time=0;
    int previous_error=0; 
    int out_derivative=0;
    int out_proportional=0;
    // PID controller
public: 
    PID* pid;



};

#endif
