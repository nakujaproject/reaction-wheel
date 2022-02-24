//this code seeks to harmonise some of the preexisting code
//refactoring ongoing
//changed some of the variables to more descriptive names
#include <Arduino.h>
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>


MPU6050 mpu;

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
double Input;
double Output;
double Setpoint = 0;
float Kp = 1.0;
float Ki = 0.0;
float Kd = 1.0;
PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);  

int pwm = 0;
//int valiue2= 0; //not used

KalmanFilter kalmanX(0.001, 0.003, 0.03); //Initialised with angle, rate and bias
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accelRoll = 0; //To hold roll calculated form accelerometer
float kalmanRoll = 0; //To hold roll calculated by fusing gyroscope and accelerometer measurements
float pitch = 0; 

//float Sensitivity = 0.05;
//To be used to make gyro print only values that change by some margin..
//if there is need to avoid redundancy
//the alternative is logging periodically (after a set duration)

//CHANGED RETURN TYPE TO VOID!!!!
void write_pwm(float speed){
  firstESC.writeMicroseconds(speed);
 // return speed;
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
  Serial.println("| accelRoll \t| kalmanRoll\t|");

  firstESC.attach(14);  //49

  balancePID.SetMode(AUTOMATIC); //
  balancePID.SetOutputLimits(1312,1488); //Investigating use of map instead of setlimits for this reverse relationship
                                        //and 1488, 1832 for direct
  Serial.begin(9600);

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

}

void loop()
{

 

  //Vector has been defined as a struct in MPU6050.h
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  //print gyro roll for comparison with accelRoll and kaRroll
  Serial.println("gyr.XAxis");


  // Calculating Roll from accelerometer (degrees)
  accelRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI; // actual roll
  //calculating pitch
  pitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  //does readNormalizeGyro incorporate a low pass filter
  //if not include smooth, average value, loc variable and 
  //gyr.XAxis = abs(smooth(readGyro(),filterVal,smoothedVal));

  // Kalman filter fuses accelerometer and gyro xaxis values
  kalmanRoll = kalmanX.update(accelRoll, gyr.XAxis); //required variable

  // View Roll data before Filter
  Serial.print(accelRoll);
  Serial.print(",");

  // View Roll data after Filter
  Serial.println(kalmanRoll);

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
  }
  Serial.print("Average Value (X): "); 
  Serial.println(Input);
  balancePID.Compute(); //this function calculates error and hence Output of pid
  if(abs(Input) > 15000){  //15000 represents max of raw gyroscope values
  //TODO: Change to suitable filtered/smoothed/manipulated equivalent

    pwm = map(Output, 0, 255, 1488, 1832); //map the output 
  }
  if(abs(Input) < 15000){
    pwm = map(Output, 0, 255, 1488, 1312);
  }

  //changed from motorspeed
  write_pwm(constraint(pwm, 1312, 1832)); //ensure pwm values is within limits required by esc
  Serial.println(pwm);
 


}



//add logic for roll values +/- 180
//add globals and logic for logging only changing/nonstill outputs
//use Sensitivity variable

//What is the need for low pass filtering and smoothing
//needed when reading raw values, and does using ReadNormalizeAcel really make them redundant
//is the bitshifting causing problems with raw values
 //remove the printline from setup and use a boolean to print once in testing
 
 //add logic for roll values +/- 180
  
  /* float printRoll (float accelRoll, float kalmanRoll) {
  float prev accelRoll
  } 
  */