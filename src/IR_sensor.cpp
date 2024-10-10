#include "IR_sensor.h"
#include "motor_control.h"


IR_sensor::IR_sensor(){ 
    // previous_time = millis();

}
IR_sensor::~IR_sensor(){ 
    delete pid;
}
void IR_sensor::IR_sensor_init(const uint8_t* sensor_p, uint8_t emit_pin ){ 
    // _IR_pins= sensor_p; 

    
    qtr.setTypeAnalog(); 
    qtr.setSensorPins((const uint8_t []){26,25,33,32,35,34,39,36 } , 8); 
    qtr.setEmitterPin(16);
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
    pid->SetOutputLimits(-95, 80 );
}
void IR_sensor::followLine(bool inverse, int bias) {
    // addd it to support the 8 IR sensors 
    int line = (inverse == true ? HIGH : LOW);
    // 1 black // 0  white 
    error =(_IR_Value[0] == line ? -4000 + bias : 0) + 
           (_IR_Value[1] == line ? -3000 : 0) + 
           (_IR_Value[2] == line ? -2000 : 0) + 
           (_IR_Value[3] == line ? -1000 : 0)+ 
           (_IR_Value[4] == line ? 1000 : 0) + 
           (_IR_Value[5] == line ? 2000 : 0) + 
           (_IR_Value[6] == line ? 3000 : 0)+
           (_IR_Value[7] == line ?  4000 + bias : 0);
    
    input = error;
    pid->Compute();
    // output=  compute_pid(error);
    leftMotorSpeed = 175 - output;
    rightMotorSpeed = 175+  output;
    int pwmL= constrain(leftMotorSpeed, 0, pwmMaxValue) ;  
    int pwmR= constrain(rightMotorSpeed, 0, pwmMaxValue) ; 
    moveForward(pwmL, pwmR);
}
int IR_sensor::compute_pid(int error){ 
    unsigned long now = millis();
    unsigned long dt= ( now - previous_time) / 1000 ; 
    out_proportional=   error; 
    out_derivative= (error - previous_error) / dt;  
    previous_error= error; 
    int out = (Kp * out_proportional)  + (Kd * out_derivative);
    return out;
}
void IR_sensor::print_pid_values(){ 
    Serial.print(" Kp ");
    Serial.print(Kp);
    Serial.print(" Ki ");
    Serial.print(Ki);
    Serial.print(" Kd ");
    Serial.print(Ki);
    Serial.println();


}
void IR_sensor::print_PID_output(){ 
    Serial.print( " Error: ");
    Serial.print(error);
    Serial.print(" PID output: "); 
    Serial.print(output); 
    Serial.print(" LEFT motor output: "); 
    Serial.print(constrain(leftMotorSpeed, 0, pwmMaxValue)); 
    Serial.print(" Right motor output: "); 
    Serial.print(constrain(rightMotorSpeed, 0, pwmMaxValue)); 
    Serial.println(); 

}

