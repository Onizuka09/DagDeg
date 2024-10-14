#ifndef BLUETOOTH_CONF_H_
#define BLUETOOTH_CONF_H
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "state.h"
#include "IR_sensor.h"
// Bluetooth serial communication


class my_Bluetooth { 
public: 
    BluetoothSerial SerialBT;
    my_Bluetooth(IR_sensor& _sens, State& _st); 
    void init_bluetooth(String); 
    ~my_Bluetooth(){}
    void handleBluetoothData();
    void Send_data(String data);
    int Kp=0; 
    int Ki=0 ; 
    int Kd=0; 
     
    void update_pid_val(void (*func)(int, int, int));
    void update_state(void (*func)(State& st)); 
    private: 
    IR_sensor& sens; 
    State& Stmp ;
};




#endif 
