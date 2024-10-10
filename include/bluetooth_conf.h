#ifndef BLUETOOTH_CONF_H_
#define BLUETOOTH_CONF_H
#include <Arduino.h>
#include <BluetoothSerial.h>
// Bluetooth serial communication


class my_Bluetooth { 
    public: 
    BluetoothSerial SerialBT;
    my_Bluetooth(); 
    ~my_Bluetooth(); 
    void handleBluetoothData();
    int Kp=0; 
    int Ki=0 ; 
    int Kd=0; 
    State Stmp ; 
    int invoke(int x, int y, int (*func)(int, int))
{
    return func(x, y);
}
void update_pid_val(void (*func)(int,int,int));
}




#endif 
