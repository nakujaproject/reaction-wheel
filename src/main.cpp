#include <Arduino.h>
 
/*
    Kalman Filter for MPU6050. Output for processing with Serial Plotter.
    Web: https://indobot.co.id
    (created) 2021 by Zamisyak Oby Indobot Academy
    The Fastest Way to Electronics Mastery
*/

#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
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
  Serial.println("accPitch,kalPitch,accRoll,kalRoll");
}

void loop()
{
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculating Pitch &amp; Roll of accelerometer (degrees)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

  // View pitch data before in Filter
  Serial.print(accPitch);
  Serial.print(",");

  // View Pitch data after Filter
  Serial.print(kalPitch);
  Serial.print(",");

  // View Roll data before in Filter
  Serial.print(accRoll);
  Serial.print(",");

  // View Roll data after in Filter
  Serial.print(kalRoll);
  Serial.println();

}