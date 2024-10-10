// #include "bluetooth_conf.h"
// #include "../include/bluetooth_conf.h"
#include "motor_control.h"
#include "IR_sensor.h"
#include <Arduino.h>
#include "bluetooth_conf.h"
#include "test.h"

IR_sensor IR; 
extern BluetoothSerial SerialBT;

// int sensorPins[] = {4, 16, 17}; // l m r

// PID constants

enum State {
  TEST_FOLLOW_LINE,
  START_POINT,
  CURVES_PATH,
  SPLIT_PATH,
  DISCONTINUED_LINE_1,
  GO_Forwad_ignore,
  WAIT_POINT,
  CIRCLE_PATH,
  DISCONTINUED_LINE_2,
  INVERSE_PATH,
  ZIGZAG_PATH,
  END_POINT,
  TEST,
  TEST_Motor,
  TEST_BLUETOOTH,
};

State currentState = START_POINT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("PID_Robot"); // Bluetooth name
  IR.IR_sensor_init((const uint8_t *) IR_ARRAY,EMIT_PIN);
//   pinMode(LED_DEBUG,OUTPUT);

  // Initialize sensor pins

  init_motors();
  // Initialize PID
  IR.init_pid();
//   stopMotors();
}

void loop() {
    // handleBluetoothData(); // Check and handle Bluetooth commands
    // Read sensor values
    int sum =0 ; 
    uint16_t sensorValues[8];
    // IR.Read_sensor();
    // IR.Print_sensor_values();
    currentState=TEST_BLUETOOTH;
    switch (currentState) {
        case TEST_BLUETOOTH: 
            handleBluetoothData(); 
        break; 
        case TEST_FOLLOW_LINE:
            IR.print_PID_output();
            for (int i = 0 ; i < 8 ; i++) { 
                int sum = IR._IR_Value[i]; 
            }
            if (sum=8){ 
                 currentState=  END_POINT; 
            }
            IR.followLine(false,0);

            delay(250);
        break; 
        case TEST:
            IR.Print_sensor_values();
            // moveForward();
            delay(1000); 
            break;
        case START_POINT:
            if (IR._IR_Value[0] == HIGH && IR._IR_Value[7] == HIGH) {
                currentState = CURVES_PATH;
            } else {
                moveForward(0, 0 );
            }
            break;

        case CURVES_PATH:
            if (IR._IR_Value[0] == LOW && IR._IR_Value[7] == LOW) {
                currentState = SPLIT_PATH;
            } else {
                IR.followLine();
            }
            break;

        case SPLIT_PATH: // TODO: make sure of the sign of the bias
           if (IR._IR_Value[0] == HIGH && IR._IR_Value[1] == HIGH &&  IR._IR_Value[2] == HIGH && IR._IR_Value[3] == HIGH && IR._IR_Value[4] ==HIGH) { 
                currentState = DISCONTINUED_LINE_1;
            } else {
                IR.followLine( false, -0.25);
            }
            break;

        case DISCONTINUED_LINE_1:
            if (IR._IR_Value[0] == LOW && IR._IR_Value[8] == LOW) {
                currentState = WAIT_POINT;
            } else {
                IR.followLine();
            }
            break;

         case WAIT_POINT: // TODO: improve the wait point logic or fine tune  the wait value so that the robot stops in the middle */
            moveForward(0 , 0 );
            delay(500);
            stopMotors();
            delay(5000); // Wait for 5 seconds
            moveForward(0,  0 );
            delay(500);
            currentState = CIRCLE_PATH;
            break;

        case CIRCLE_PATH: // TODO: make sure of the sign of the bias
            if (IR._IR_Value[0] == HIGH && IR._IR_Value[1] == HIGH && IR._IR_Value[2] == HIGH && IR._IR_Value[3] == HIGH && IR._IR_Value[4] ==HIGH) {
                currentState = DISCONTINUED_LINE_2;
            } else {
                IR.followLine( false, -0.25);
            }
            break;

        case DISCONTINUED_LINE_2:
            if (IR._IR_Value[0] == LOW && IR._IR_Value[4] == LOW) {
                currentState = INVERSE_PATH;
            } else {
                IR.followLine();
            }
            break;

        case INVERSE_PATH:
            if (IR._IR_Value[0] == HIGH && IR._IR_Value[4] == HIGH) {
                currentState = ZIGZAG_PATH;
            } else {
                IR.followLine(true);
            }
            break;

        case ZIGZAG_PATH:
            if (sensorValues[0] == LOW && sensorValues[4] == LOW) {
                currentState = END_POINT;
            } else {
                IR.followLine();
            }
            break;

        case END_POINT:
            // moveForward(0,0);
            // delay(500);
            stopMotors();
            break;
        case TEST_Motor:
            moveForward(150,150);
            delay(3000);
            stopMotors();     

    }
}
