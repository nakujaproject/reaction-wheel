#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <PID.h>

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accRoll = 0;
float kalRoll = 0;

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
}

void loop()
{
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculating Pitch &amp; Roll of accelerometer (degrees)
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/PI; // actual roll

  // Kalman filter
  kalRoll = kalmanX.update(accRoll, gyr.XAxis); //required variable

  // View Roll data before in Filter
  Serial.print("| ");
  Serial.print(accRoll);
  Serial.print("\t| ");

  // View Roll data after in Filter
  Serial.print(kalRoll);
  Serial.println("\t|");

}
