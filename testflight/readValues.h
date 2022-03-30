#pragma once
#include "initialiseMPU.h"
#include "writeESC.h"

#define sampleTime 10
double elapsedTime = 0;
double  timeCur = 0;
double timePrev = 0;
double anglePrev = 0;
double angleCur = 0;
double _speed = 0;
double rollVel=0;
float pwm=0;
double Input=0;



#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

double readMPU(){  
  if (!dmpReady) //return;
    // read a packet from FIFO
    if ((mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) && (elapsedTime > sampleTime));  { // Get the Latest packet 
              // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

         mpu.getMotion6(&accData[0], &accData[1], &accData[2], &gyrData[0], &gyrData[1], &gyrData[2]);
         
        ypr[2] = ypr[2] * 180/M_PI; //angle in degrees.
        timePrev = timeCur;
        timeCur = millis();
        angleCur=ypr[2];
        elapsedTime = timeCur - timePrev;
                     
            
            while (ypr[2]<= -180) ypr[2] += 360; 
            while (ypr[2] > 180)   ypr[2]  -= 360;
       Serial.print("ypr[2]:\t");
          Serial.println(ypr[2]);
     
        rollVel = (((angleCur-anglePrev)/(elapsedTime/1000.00))*(30/M_PI));//to convert the velocity to rpm multiply the value by (60/2*M_PI)
             Serial.println("rollVel  ");
            Serial.println(rollVel);
            
       //finding avionics values
            Serial.println("ax  ");
            Serial.println(accData[0]);
            Serial.println("ay  ");
            Serial.println(accData[1]);
            Serial.println("az  ");
            Serial.println(accData[2]);
            Serial.println("gx  ");
            Serial.println(gyrData[0]);
            Serial.println("gy  ");
            Serial.println(gyrData[1]);
            Serial.println("gz ");
            Serial.println(gyrData[2]);
                 
    }
      Input = (ypr[2])* 180/M_PI;  
      return Input;
   
  }
  void readPWM(){
    
            Serial.println("pwm");
            Serial.println(pwm);
        
  }
 
