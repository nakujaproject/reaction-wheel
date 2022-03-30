#pragma once
#include "PID_v1.h"
#include "readValues.h"

//long timeCur, timePrev, timeStart; 

float change=0;
double prevInput=0;
static double Output;
double  Setpoint = 0;
float Kp = 2.0;
float Ki = 0.0;
float Kd = 1.0;

PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);


void PIDsetup(){
  balancePID.SetMode(AUTOMATIC); //
    balancePID.SetOutputLimits(-176,344);//to range 
}
void PIDloop(double Input){
  Setpoint = 0;   
  //Input = (ypr[2])* 180/M_PI;
  change=prevInput-Input;
        prevInput=Input;
    while ( Input<= -180) Input += 360; 
            while (Input > 180)   Input  -= 360; 

            
          Serial.print("Input:\t");
          Serial.println(Input);
if(Input< 0){
        balancePID.SetControllerDirection(REVERSE);
        }
        else{
        balancePID.SetControllerDirection(DIRECT);
        }
            
        balancePID.Compute(); 
          



}
