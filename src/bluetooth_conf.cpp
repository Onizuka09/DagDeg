#include "bluetooth_conf.h"

#include "motor_control.h"
#include "state.h"

my_Bluetooth::my_Bluetooth(){ 
    SerialBT.begin("Robot_follower");
}

void my_Bluetooth::handleBluetoothData() {
    if (SerialBT.available()) {
        String data = SerialBT.readStringUntil('\n');


        Serial.println(data);
        data.trim();
        // if ( data == "on" ){ 
        //     moveForward(150,150); 
        // }else if (data == "off"){ 
        //   stopMotors(); 
        // }else if (data == "F"){
        //   moveForward(150,150); 
        // }else if (data == "B"){ 
        //   //TODO: not implemented backward 
        // }else if (data == "R"){ 
        //     moveRight(); 
        // } else if (data == "L"){ 
        //   moveLeft();
        // }

          
        // Process Bluetooth data to update parameters
        // TODO: handle stop, start, refresh, bias value and inverse value
         if (data.startsWith("kp=")) { 
             Kp = data.substring(3).toFloat(); 

            //  pid.SetTunings(Kp, Ki, Kd); 
             Serial.println("Kp updated to: " + String(Kp)); 
         } else if (data.startsWith("ki=")) { 
             Ki = data.substring(3).toFloat(); 
            //  pid.SetTunings(Kp, Ki, Kd); 
             Serial.println("Ki updated to: " + String(Ki)); 
         } else if (data.startsWith("Kd=")) { 
             Kd = data.substring(3).toFloat(); 
            //  pid.SetTunings(Kp, Ki, Kd); 
             Serial.println("kd updated to: " + String(Kd)); 
         } else if (data.startsWith("state=")) { 
             int stateValue = data.substring(6).toInt(); 
             if (stateValue >= TEST_STATE && stateValue <= END_POINT) { 
                 Stmp = static_cast<State>(stateValue); 
                 Serial.println("State updated to: " + String(stateValue)); 
             } 
         } 
    }
}

void my_Bluetooth::update_pid_val(){ 
    
}

