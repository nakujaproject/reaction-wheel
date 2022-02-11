#include <Arduino.h>
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

MPU6050 mpu;

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

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accRoll = 0;
float kalRoll = 0;
float pitch = 0;

float Sensitivity = 0.05;

void setup() 
{
  Serial.begin(115200);

  // initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
 
  // Calibration of gyroscopes. When calibrating, the sensor must be stationary.
  mpu.calibrateGyro();// If you don't want to calibrate, comment on this line.

  // Create a label on each output. 
  Serial.println("| accRoll \t| kalRoll\t|");

  firstESC.attach(14);  //49

  balancePID.SetMode(AUTOMATIC);
  balancePID.SetOutputLimits(0,255);
  Serial.begin(9600)

   Serial.println("Calibration procedure for Mamba ESC.");
  Serial.println("Turn on ESC.");
  firstESC.writeMicroseconds(0);
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


  // for (int thisReading = 0; thisReading < numReadings; thisReading++)
  //   readings[thisReading] = 0;



}

void loop()
{

  //add globals and logic for printing only changing/nonstill outputs

  //add logic for tolerance allowed in rotation before printing in degrees or increments in %*1024
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculating Pitch &amp; Roll of accelerometer (degrees)
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI; // actual roll
  //calculating pitch
  pitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  //does readNormalizeGyro incorporate a low pass filter
  //if not include smooth, average value, loc variable and 
  //gyr.XAxis = abs(smooth(readGyro(),filterVal,smoothedVal));

  // Kalman filter fuses accelerometer and gyro xaxis values
  kalRoll = kalmanX.update(accRoll, gyr.XAxis); //required variable

  // View Roll data before Filter
  Serial.print(accRoll);
  Serial.print(",");

  // View Roll data after Filter
  Serial.println(kalRoll);

  Print

  //add logic for roll values +/- 180
  
  float printRoll (float accRoll, float kalRoll) {
  float prev accRoll
  }

  
  Input = kalRoll; //Get Input of PID from Kalman filter
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

}

//add logic for roll values +/- 180