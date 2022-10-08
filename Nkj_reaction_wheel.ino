///NKJ

#include<Wire.h>
#include <ESP32Servo.h>
#include <PID_v1.h>


// motor parameters

int neutral = 1488;
int fullForward = 1832;
int fullReverse = 1312;

int sensVal;           
float filterVal = 0.001;      
float smoothedVal;     
float smoothedVal2;  
int i, j;  

Servo firstESC;
const int MPU=0x68;  
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
const int numReadings = 10;

int readings[numReadings];      
int loc = 0;                
int total = 0;                  // the running total
int average = 0;                // the average

int goal = 300;
double Input;
double Output;
double Setpoint = 14000;
int Kp = 5;
int Ki = 0.5;
int Kd = 1;
PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);  

int value = 0;
int valiue2= 0;

void setup(){
  pinMode(14, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission(true);
  firstESC.attach(14);    
  balancePID.SetMode(AUTOMATIC);
  balancePID.SetOutputLimits(0,255);
  Serial.begin(9600);         
  
  Serial.println("Calibration procedure for Mamba ESC.");
  Serial.println("Turn on ESC.");
  firstESC.writeMicroseconds(1312);
  Serial.println("Starting Calibration.");
  delay(5000);
  firstESC.writeMicroseconds(1832);
  Serial.println("Writing Full Throttle.");
  delay(5000);
  firstESC.writeMicroseconds(1312);
  Serial.println("Writing Full Reverse.");
  delay(5000);
  firstESC.writeMicroseconds(1488);
  Serial.println("Writing Neutral.");
  delay(1000);
  Serial.println("Calibration Complete.");


  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;

  
}
void loop(){
  Input = abs(smooth(readGyro(),filterVal,smoothedVal));
  if(Input < 0){
    balancePID.SetControllerDirection(REVERSE);
  }
  else{
    balancePID.SetControllerDirection(DIRECT);
  }
  if(Input > 15000){
    balancePID.SetControllerDirection(REVERSE);
  }
  if(Input < 15000){
    balancePID.SetControllerDirection(DIRECT);
  }
  Serial.print("Average Value (X): ");Serial.println(Input);
  balancePID.Compute();
  if(abs(Input) > 15000){
    value = (Output*5.3);
      motorSpeed(value);
  }
  if(abs(Input) < 15000){
    value = (Output*5.3)+1249;
      motorSpeed(value);
  }
  Serial.println(value);
 


}

int motorSpeed(int newValue){
  firstESC.writeMicroseconds(newValue);


}

int averageValue(int GyX){
    total= total - readings[loc];
    readings[loc] = GyX;
    total= total + readings[loc];     
    loc+=1;                    
    if (loc >= numReadings){              
      loc = 0;                          
    }
    average = total / numReadings;  
    //Serial.print("Average Value (X): ");Serial.println(average); 
   return average;   
}

int readGyro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
  if(AcY < 0){
    return ((AcX^2)+(AcY^2))^(1/2);
  }
  if(AcY > 0){
    return -((AcX^2)+(AcY^2))^(1/2);
  }
}

int smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){     
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}
