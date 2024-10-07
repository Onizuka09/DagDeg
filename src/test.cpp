#include "../include/test.h" 
#include "esp32-hal.h"
#include <Arduino.h>
void test_IR(int arr_sens[]){ 
  int size=3 ; 
  int l = digitalRead(arr_sens[1]); // left  
  int r = digitalRead(arr_sens[2]); // right 
  Serial.print("Left:  "); 
  Serial.println(l); 
 Serial.print("right:  "); 
  Serial.println(r); 
  delay(2000);
}
