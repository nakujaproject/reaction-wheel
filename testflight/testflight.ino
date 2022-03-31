#pragma once
//#include "initialiseMPU.h"
#include "N2pid.h"
#include "readValues.h"
//#include "writeESC.h"

void setup() {
//Serial.println(115200);
mpusetup();
 ESC.attach(14); 

PIDsetup();

}

void loop() {
 unsigned long start = millis();
  readMPU();
  
PIDloop(Input);
readPWM();
 write_pwm(pwm);
  
    unsigned long endtime = millis();
    float freq = 1000 / (endtime - start);
    Serial.print("rate: ");
    Serial.println(freq);
}
