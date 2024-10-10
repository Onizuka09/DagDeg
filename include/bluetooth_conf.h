#ifndef BLUETOOTH_CONF_H_
#define BLUETOOTH_CONF_H
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "state.h"
// Bluetooth serial communication


class my_Bluetooth { 
public: 
    BluetoothSerial SerialBT;
    my_Bluetooth(); 
    ~my_Bluetooth(){}
    void handleBluetoothData();
    int Kp=0; 
    int Ki=0 ; 
    int Kd=0; 
    State Stmp ; 
    void update_pid_val(void (*func)(int, int, int));
    void update_state(void (*func)(State& st)); 
};




#endif 
