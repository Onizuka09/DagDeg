#include "IR_sensor.h"
#include "motor_control.h"


IR_sensor::IR_sensor(){ 

}
IR_sensor::~IR_sensor(){ 
    delete pid;
}
void IR_sensor::IR_sensor_init(const uint8_t* sensor_p, uint8_t emit_pin ){ 
    // _IR_pins= sensor_p; 

    
    qtr.setTypeAnalog(); 
    qtr.setSensorPins(sensor_p , emit_pin); 
    qtr.setDimmingLevel(0);
}


void IR_sensor::Read_sensor(){ 
    qtr.read(_IR_Value); 
    for (int  i = 0 ; i < IR_PIN_COUNT; i++ ){ 
       ( _IR_Value[i] >= THRESHOLD ? _IR_Value[i]=HIGH: _IR_Value[i]=LOW ) ;  
    }

}

void IR_sensor::Print_sensor_values(){ 
    // debutg senor values ; 
    for (uint8_t i = 0 ; i<IR_PIN_COUNT; i++){ 
            Serial.print(_IR_Value[i]); 
            Serial.print('\t'); 
    }
    Serial.println();
}
//  set sensor pins 

void IR_sensor::init_pid(){ 
    pid = new  PID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);
    pid->SetMode(AUTOMATIC);
    pid->SetOutputLimits(-pwmMaxValue, pwmMaxValue);
}
void IR_sensor::followLine(bool inverse, int bias) {
    // addd it to support the 8 IR sensors 
    int line = (inverse == true ? HIGH : LOW);
    int error = (_IR_Value[0] == line ? -1 + bias : 0) + 
                (_IR_Value[1] == line ? -0.75 : 0) + 
                (_IR_Value[2] == line ? -0.5 : 0) + 
                (_IR_Value[3] == line ? -0.25 : 0)+ 
                (_IR_Value[4] == line ? 0.25 : 0) + 
                (_IR_Value[5] == line ? 0.5 : 0) + 
                (_IR_Value[6] == line ? 0.75 : 0)+
                (_IR_Value[7] == line ? 1 + bias : 0);
    
    input = error;
    pid->Compute();

    int leftMotorSpeed = pwmMaxValue - output;
    int rightMotorSpeed = pwmMaxValue + output;

    ledcWrite(0, constrain(leftMotorSpeed, 0, pwmMaxValue));
    ledcWrite(1, 0);
    ledcWrite(2, constrain(rightMotorSpeed, 0, pwmMaxValue));
    ledcWrite(3, 0);
}

