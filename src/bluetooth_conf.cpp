#include "bluetooth_conf.h"

#include "motor_control.h"
BluetoothSerial SerialBT;

void handleBluetoothData() {
    if (SerialBT.available()) {
        String data = SerialBT.readStringUntil('\n');
        data.trim(); 
        if ( data == "on" ){ 
            moveForward(); 
        }else if (data == "off"){ 
          stopMotors(); 
        }else if (data == "F"){
          moveForward(); 
        }else if (data == "B"){ 
          //TODO: not implemented backward 
        }else if (data == "R"){ 
            moveRight(); 
        } else if (data == "L"){ 
          moveLeft();
        }
          
        // Process Bluetooth data to update parameters
        // TODO: handle stop, start, refresh, bias value and inverse value
        /* if (data.startsWith("Kp=")) { */
        /*     Kp = data.substring(3).toFloat(); */
        /*     pid.SetTunings(Kp, Ki, Kd); */
        /*     Serial.println("Kp updated to: " + String(Kp)); */
        /* } else if (data.startsWith("Ki=")) { */
        /*     Ki = data.substring(3).toFloat(); */
        /*     pid.SetTunings(Kp, Ki, Kd); */
        /*     Serial.println("Ki updated to: " + String(Ki)); */
        /* } else if (data.startsWith("Kd=")) { */
        /*     Kd = data.substring(3).toFloat(); */
        /*     pid.SetTunings(Kp, Ki, Kd); */
        /*     Serial.println("Kd updated to: " + String(Kd)); */
        /* } else if (data.startsWith("state=")) { */
        /*     int stateValue = data.substring(6).toInt(); */
        /*     if (stateValue >= TEST_STATE && stateValue <= END_POINT) { */
        /*         currentState = static_cast<State>(stateValue); */
        /*         Serial.println("State updated to: " + String(stateValue)); */
        /*     } */
        /* } */
    }
}

