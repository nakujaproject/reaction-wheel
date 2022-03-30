#pragma once
#include<ESP32Servo.h>
Servo ESC;

void  write_pwm(float _speed){
  ESC.writeMicroseconds(_speed);  //returns a pulse of width (_speed)microseconds every 20 milliseconds
 
}
