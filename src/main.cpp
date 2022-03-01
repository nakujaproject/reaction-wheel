#include <Arduino.h>
//#include <Streaming.h>
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define M_PI		3.14159265358979323846

MPU6050 mpu;

bool Label = true;

// motor parameters
//These values were set when programming the BhHeli ESC

//TO BE CHANGED TO BE FLOATS HERE AND IN THE MAPPING
int neutral = 1488; 
int fullForward = 1832; 
int fullReverse = 1312;

//these variables are required when smoothing raw gyroscope values.
/*int sensVal;           
float filterVal = 0.001;       
float smoothedVal;     
float smoothedVal2;  
//iterators for traversing array when smoothing
int i, j; 
*/ 

Servo firstESC;
const int MPU=0x68;  //I2C address of mpu
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

//used to implement a low pass filter for raw gyroscope values
/*const int numReadings = 10;
int readings[numReadings];  
//Initialising the array
for (int thisReading = 0; thisReading < numReadings; thisReading++)
readings[thisReading] = 0;
int loc = 0;                
int total = 0;                  // the running total
int average = 0;                // the average
*/

//int goal = 300; 
double Input = 0;
double Output = 0;
double Setpoint = 0;
float Kp = 1.0;
float Ki = 0.0;
float Kd = 1.0;
PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);  

int pwm = 0;
//int valiue2= 0;

KalmanFilter kalmanX(0.001, 0.003, 0.03); //Initialised with angle, rate and bias
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accelRoll = 0; //To hold roll calculated form accelerometer
float kalmanRoll = 0; //To hold roll calculated by fusing gyroscope and accelerometer measurements
float pitch = 0; 

//double Sensitivity = 0.05;
//To be used to make gyro print only values that change by some margin..
//if there is need to avoid redundancy
//the alternative is logging periodically (after a set duration)

//CHANGED RETURN TYPE TO VOID!!!!
void write_pwm(float speed){
  firstESC.writeMicroseconds(speed);
 // return speed;
}

void calibrateESC () {
   Serial.println("Calibration procedure for Mamba ESC.");
  Serial.println("Turn on ESC.");
  firstESC.writeMicroseconds(0);
  Serial.println("Starting Calibration.");
  delay(1000);
  firstESC.writeMicroseconds(1832);
  Serial.println("Writing Full Throttle.");
  delay(1000);
  firstESC.writeMicroseconds(1312);
  Serial.println("Writing Full Reverse.");
  delay(1000);
  firstESC.writeMicroseconds(1488);
  Serial.println("Writing Neutral.");
  delay(1000);
  Serial.println("Calibration Complete.");
}

void printLabel() {
Serial.println("| accelRoll \t | gyr.XAxis \t | kalmanRoll\t| Output\t | pwm\t|");
}


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
  
  firstESC.attach(14);  //49

  balancePID.SetMode(AUTOMATIC); //
  balancePID.SetOutputLimits(0,255); //Investigating use of map instead of setlimits for this reverse relationship
                                        //and 1488, 1832 for direct
  Serial.begin(9600);

 calibrateESC();

}


void loop()
{

  while(Label){
  printLabel();
  delay(5000);
  Label = false;
  }

  //Vector has been defined as a struct in MPU6050.h
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculating Roll from accelerometer (degrees)
  accelRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI; // actual roll
  //calculating pitch
  pitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  

  // Kalman filter fuses accelerometer and gyro xaxis values
  kalmanRoll = kalmanX.update(accelRoll, gyr.XAxis); //required variable

  // View Roll data before Filter

/*
  Serial.print(accelRoll);
  Serial.print(",");
  delay(1000);
  Serial.print (gyr.XAxis);
  Serial.print(",");
  delay(1000);
  Serial.println(kalmanRoll);
  //Serial.print(",");
    delay(1000); 
    */

  Input = kalmanRoll; //Get Input of PID from Kalman filter
  if(Input < 0){
    balancePID.SetControllerDirection(REVERSE);
  }
  else{
    balancePID.SetControllerDirection(DIRECT);
  }
  // if(Input > 15000){ 
  //   balancePID.SetControllerDirection(REVERSE);
  // }
  // if(Input < 15000){
  //   balancePID.SetControllerDirection(DIRECT);
  
  balancePID.Compute(); 
  /*
  Serial.print("Output: ");
  Serial.println(Output);
  delay(1000); */

                    //this function calculates error and hence Output of pid, but returns A BOOLEAN
  if(abs(Input) > 15000){                //15000 represents max of raw gyroscope values
                                          //TODO: Change to suitable filtered/smoothed/manipulated equivalent

    pwm = map(Output, 0, 255, 1488, 1832); //map the output 
  }
  if(abs(Input) < 15000){
    pwm = map(Output, 0, 255, 1488, 1312);
  }

  //changed from motorspeed
  int constrainedpwm = constrain(pwm, 1312, 1832);
  write_pwm(constrainedpwm); //Ensure pwm values is within limits required by ESC
  /* 
  Serial.print("Constrained pwm:  ");
  Serial.println(constrainedpwm);
    delay(1000);
    */


 Serial.println("Done");


}



//add logic for roll values +/- 180
//add globals and logic for logging only changing/nonstill outputs, use Sensitivity variable
//Is low pass filtering and smoothing a gamechanger
//Set up own calibration and offsets for gyro, read using wire, smooth, filter (process the raw values). Offsets so it reads 0 when flat.
//Is unpredictable bitshifting in Wire.read()<<|8 causing problems with raw values
 //remove the printline from setup and use a boolean to print once in testing
 //use printlabel function
 //calibrateESC function
 //add logic for roll values +/- 180
  
  //one printCSV function
  /* float printcsv (float accelRoll, float kalmanRoll) {
  float prev accelRoll
  } 
  */

 //COMPUTE RETURNS BOOLEAN
 //Implement controller direction change when setpoint < 0;
 //setsampletime . ki*this, kd/this