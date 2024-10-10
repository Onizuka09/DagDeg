#include "bluetooth_conf.h"

#include "motor_control.h"
#include "state.h"

my_Bluetooth::my_Bluetooth(IR_sensor& _sens, State& _st):sens(_sens), Stmp(_st){ 
}
void my_Bluetooth::init_bluetooth(String n){ 
    SerialBT.begin(n);

}
void my_Bluetooth::handleBluetoothData() {
    if (SerialBT.available()) {
        String data = SerialBT.readStringUntil('\n');


        Serial.println(data);
        data.trim();
        // Process Bluetooth data to update parameters
        // TODO: handle stop, start, refresh, bias value and inverse value
         if (data.startsWith("kp=")) { 
             sens.Kp = data.substring(3).toFloat(); 

            //  pid.SetTunings(Kp, Ki, Kd); 
             Serial.println("Kp updated to: " + String(Kp)); 
         } else if (data.startsWith("ki=")) { 
             sens.Ki = data.substring(3).toFloat(); 
            //  pid.SetTunings(Kp, Ki, Kd); 
             Serial.println("Ki updated to: " + String(Ki)); 
         } else if (data.startsWith("Kd=")) { 
             sens.Kd = data.substring(3).toFloat(); 
            //  pid.SetTunings(Kp, Ki, Kd);

             Serial.println("kd updated to: " + String(Kd)); 
         } else if (data.startsWith("state=")) { 
             int stateValue = data.substring(6).toInt(); 
             if (stateValue >= START_POINT && stateValue <= END_STATE) { 
                 Stmp = static_cast<State>(stateValue); 
                 Serial.println("State updated to: " + String(stateValue)); 
             } 
         } 
    }
}


