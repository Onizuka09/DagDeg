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
  TEST_STATE,
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
};

State currentState = START_POINT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("PID_Robot"); // Bluetooth name
  IR.IR_sensor_init((const uint8_t *) IR_ARRAY,EMIT_PIN);
  pinMode(LED_DEBUG,OUTPUT);

  // Initialize sensor pins

//   init_motors();
  // Initialize PID
  // init_pid();&
//   stopMotors();
}

void loop() {
    // handleBluetoothData(); // Check and handle Bluetooth commands
    // Read sensor values
    uint16_t sensorValues[8];
    IR.Read_sensor();
    IR.Print_sensor_values();
    currentState=TEST;
    switch (currentState) {
        case TEST:
            IR.Print_sensor_values();
            delay(1000); 
            break;
        case START_POINT:
            if (sensorValues[0] == HIGH && sensorValues[4] == HIGH) {
                currentState = CURVES_PATH;
            } else {
                moveForward();
            }
            break;

        case CURVES_PATH:
            if (sensorValues[0] == LOW && sensorValues[4] == LOW) {
                currentState = SPLIT_PATH;
            } else {
                IR.followLine();
            }
            break;

        case SPLIT_PATH: // TODO: make sure of the sign of the bias
           if (sensorValues[0] == HIGH && sensorValues[1] == HIGH &&  sensorValues[2] == HIGH && sensorValues[3] == HIGH && sensorValues[4] ==HIGH) { 
                currentState = DISCONTINUED_LINE_1;
            } else {
                IR.followLine( false, -0.25);
            }
            break;

        case DISCONTINUED_LINE_1:
            if (sensorValues[0] == LOW && sensorValues[4] == LOW) {
                currentState = WAIT_POINT;
            } else {
                IR.followLine();
            }
            break;

         case WAIT_POINT: // TODO: improve the wait point logic or fine tune  the wait value so that the robot stops in the middle */
            moveForward();
            delay(500);
            stopMotors();
            delay(5000); // Wait for 5 seconds
            moveForward();
            delay(500);
            currentState = CIRCLE_PATH;
            break;

        case CIRCLE_PATH: // TODO: make sure of the sign of the bias
            if (sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH && sensorValues[3] == HIGH && sensorValues[4] ==HIGH) {
                currentState = DISCONTINUED_LINE_2;
            } else {
                IR.followLine( false, -0.25);
            }
            break;

        case DISCONTINUED_LINE_2:
            if (sensorValues[0] == LOW && sensorValues[4] == LOW) {
                currentState = INVERSE_PATH;
            } else {
                IR.followLine();
            }
            break;

        case INVERSE_PATH:
            if (sensorValues[0] == HIGH && sensorValues[4] == HIGH) {
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
            moveForward();
            delay(500);
            stopMotors();
            break;

        case TEST_STATE:
            IR.followLine();
            delay(250);
            break; 
    }
}
