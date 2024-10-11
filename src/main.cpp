// #include "bluetooth_conf.h"
// #include "../include/bluetooth_conf.h"
#include "motor_control.h"
#include "IR_sensor.h"
#include <Arduino.h>
#include "bluetooth_conf.h"
#include "test.h"
#include "state.h"

IR_sensor IR; 
State currentState = START_POINT;
my_Bluetooth bl(IR,currentState); 

// int sensorPins[] = {4, 16, 17}; // l m r

// PID constants
unsigned long now =0  ; 
void setup() {
  Serial.begin(115200);
  bl.init_bluetooth("Robot_follower" );
//   bl.update_pid_val(&SetTunings);
//   bl.update_state(&set_state); 
   // Bluetooth name
  IR.IR_sensor_init((const uint8_t *) IR_ARRAY,EMIT_PIN);
//   pinMode(LED_DEBUG,OUTPUT);

  // Initialize sensor pins

  init_motors();
  // Initialize PID
  IR.init_pid();
//   stopMotors();
currentState=TEST_FOLLOW_LINE;
// currentState=TEST_BLUETOOTH;

}

void loop() {
    // handleBluetoothData(); // Check and handle Bluetooth commands
    // Read sensor values
    int sum =0 ; 
    // uint16_t sensorValues[8];
    bl.handleBluetoothData(); 
    IR.Read_sensor();
    // IR.Print_sensor_values();
    // IR.Read_sensor();
    
    switch (currentState) {
        case TEST_BLUETOOTH: 
            IR.print_pid_values();
        break; 
        // case: 
        case TEST_FOLLOW_LINE:
            if (IR.MR_IR_Val== LOW && IR._IR_Value[0] == LOW &&
                 IR._IR_Value[1] == LOW  && IR._IR_Value[2] == LOW && IR._IR_Value[3] == LOW  && IR._IR_Value[4] == LOW &&
                IR._IR_Value[5] ==LOW   && IR._IR_Value[6] == LOW &&
                IR._IR_Value[7] ==LOW && IR.ML_IR_Val==LOW  ){
                // Serial.println("set point new ...."); 
                currentState = TEST_FOLLOW_LINE; 
            }else { 
            IR.Print_sensor_values();
            IR.print_PID_output();
            IR.followLine(false,0);
            // delay(250);
            }
        break; 
        case TEST_STOP_MOTOR:
            // Serial.println("waaaaa ...."); 
            stopMotors();
            Serial.println("Motor stopped ");
            currentState=TEST_FOLLOW_LINE;

            break;
        case TEST:
            // IR.Print_sensor_values();
            Serial.println("Value changed ...");
            // moveForward();
            delay(1000); 
            break;
        // ---------------- Real work here -----------------------------    
        case START_POINT:
            if (IR.ML_IR_Val == LOW && IR.MR_IR_Val == LOW) {
                currentState = SENTIER_ANCIENT;
            } else {
                // moveForward(0, 0 );
                IR.followLine();
            }
            break;

        case SENTIER_ANCIENT:
            if (IR._IR_Value[0] == HIGH &&
                 IR._IR_Value[1] == HIGH  && IR._IR_Value[2] == HIGH &&
                IR._IR_Value[3] == HIGH  && IR._IR_Value[4] == HIGH &&
                IR._IR_Value[5] ==HIGH   && IR._IR_Value[6] == HIGH &&
                IR._IR_Value[7] ==HIGH   ) {
                currentState = HEXAGONE;
            } else {
                IR.followLine();
            }
            break;

        case HEXAGONE: // TODO: make sure of the sign of the bias
           if (IR.MR_IR_Val== LOW && IR._IR_Value[0] == LOW &&
                 IR._IR_Value[1] == LOW  && IR._IR_Value[2] == LOW &&
                IR._IR_Value[3] == LOW  && IR._IR_Value[4] == LOW &&
                IR._IR_Value[5] == LOW   && IR._IR_Value[6] == LOW &&
                IR._IR_Value[7] ==LOW && IR.ML_IR_Val==LOW) { 
                currentState = SENTIER_BIRSE;
            } else {
                IR.followLine( false, 1000);
            }
            break;

        case SENTIER_BIRSE:
            if (IR.MR_IR_Val == HIGH ) {
                currentState = SENTIER_BIRSE_2;
            } else {
                IR.followLine();
            }
            break;
        case SENTIER_BIRSE_2://90 angle 
            if (IR.MR_IR_Val == HIGH ) {
                currentState = REPOS_ANCIENT;
            } else {
                 now = millis( );
                if ( millis() - now  <= 1000 ){ 
                    moveRight(); 
                }else { 
                IR.followLine(false,5000,-5000);
                }
            }
            break;

         case REPOS_ANCIENT: // TODO: improve the wait point logic or fine tune  the wait value so that the robot stops in the middle */
         if (IR._IR_Value[0] == HIGH && IR._IR_Value[1] == HIGH && IR._IR_Value[2] == HIGH && IR._IR_Value[3] == HIGH && IR._IR_Value[4] ==HIGH){ 
            currentState = PASSAGE_CERCLE ; 
         }
        now = millis( );
        if ( millis() - now  <= 1000 ){ 
            moveRight(); 
        }else { 
            now = millis( );
            if ( millis() - now <= 500){ 
                IR.followLine();
            }else { 
                if (IR.D_IR_Val == HIGH){ 
                    delay(5000); 
                }
                moveForward(0 , 0 );
                delay(500);
                stopMotors();
                delay(5000); // Wait for 5 seconds
                moveForward(0,  0 );
                delay(500);
                currentState = PASSAGE_CERCLE;
            }
        }
        break;

        case PASSAGE_CERCLE: // TODO: make sure of the sign of the bias
            if (IR._IR_Value[0] == LOW && IR._IR_Value[1] == LOW && IR._IR_Value[2] == LOW && IR._IR_Value[3] == LOW && IR._IR_Value[4] ==LOW) {
                currentState = ESCALIER_DEFI;
            } else {
                IR.followLine( false, 1000,0);
            }
            break;

        case ESCALIER_DEFI:
            if (IR.ML_IR_Val == HIGH && IR.MR_IR_Val == HIGH) {
                currentState = CHEMIN_SINUSOIDALE ;
            } else {
                IR.followLine(false, 1000,-1000);
            }
            break;

        case CHEMIN_SINUSOIDALE:
            if (IR.ML_IR_Val == LOW && IR.MR_IR_Val == LOW) {
                currentState = TRIANGLE_LABYRINTHE;
            } else {
                IR.followLine(true);
            }
            break;

        case TRIANGLE_LABYRINTHE:
            if (IR.MR_IR_Val== HIGH && IR._IR_Value[0] == HIGH &&
                 IR._IR_Value[1] == HIGH  && IR._IR_Value[2] == HIGH && IR._IR_Value[3] == HIGH  && IR._IR_Value[4] == HIGH && IR._IR_Value[5] ==HIGH   && IR._IR_Value[6] == HIGH && IR._IR_Value[7] ==HIGH && IR.ML_IR_Val==HIGH) {
                currentState = END_POINT;
            } else {
                IR.followLine(false, 2000,-2000);
            }
            break;
        case END_POINT:
            if ( IR.D_IR_Val== HIGH ){ 
                 now = millis( );
                if ( millis() - now  <= 1000 ){ 
                    moveForward(100,100); 
                }else { 
                    stopMotors();
                }
            }
            moveForward(100,100);
            break;
        case TEST_Motor:
            moveForward(150,150);
            delay(3000);
            stopMotors();   
            break;
        default: 
            Serial.println("Unkonw sate "); 
            break;  

    }
    // delay(250); 
}
