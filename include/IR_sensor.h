#ifndef PID_CONF_H_
#define PID_CONF_H_
#include <Arduino.h>
#include<QTRSensors.h>
#include "pin_conf.h"
#include<PID_v1.h>
#define IR_PIN_COUNT 8
#define EMIT_PIN 16
const uint8_t IR_ARRAY[8] = {26,25,33,32,35,34,39,36 };

class IR_sensor{ 
public:
    uint16_t THRESHOLD = 4000;  
    uint16_t _IR_Value[IR_PIN_COUNT];

public:
    IR_sensor(); 
    void IR_sensor_init(const uint8_t* IR_pins, const uint8_t emit_pin); 
    ~IR_sensor();
    // void followLine(int sensorValues[], bool inverse , int bias ) ;
    void Read_sensor(); 
    void Print_sensor_values();
    void followLine( bool inverse = false, int bias = 0); 
    void init_pid();
private: 
    QTRSensors qtr; 
    uint8_t _IR_pins[IR_PIN_COUNT];
    uint8_t _emit_pin=0; 
    double Kp = 2.0, Ki = 5.0, Kd = 1.0;
    double setPoint = 0;
    double input, output;
    // PID controller
    PID* pid; ;



};

#endif
