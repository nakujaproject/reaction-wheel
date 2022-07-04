#include "PID_v1.h"
#include <ESP32Servo.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define SD_CS 5

#include "telemetry.h"


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t accData[3], gyrData[3];

Servo ESC;

int neutral = 1488; 
int fullForward = 1832; 
int fullReverse = 1148;


//long timeCur, timePrev, timeStart; 
double Input;
static double Output;
double Setpoint = 0;
float Kp = 2.0;
float Ki = 0.0;
float Kd = 1.0;


#define sampleTime 10
double elapsedTime = 0;
double timeCur = 0;
double timePrev = 0;
double anglePrev = 0;
double angleCur = 0;
double _speed = 0;

//variables that hold the SD logged values
double rollVel=0;
double pwm = 0;


String dataMessage;
File file;


PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);


//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" 
#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 22 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SDA 21
#define SCL 22
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in  
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void write_pwm(float motorspeed){
  ESC.writeMicroseconds(motorspeed);
 
}


double constrainpwm (double pwm, double Min, double Max){
  if   (pwm< Min) return Min;
  else if    (pwm> Max) return Max;

  else return pwm;

}

void calibrateESC () {
  ESC.writeMicroseconds(1000);
  delay(2000);
  ESC.writeMicroseconds(2000);
  delay(1000);}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(SDA, SCL);
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT_PULLUP); 

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    

    // load and configure the DMP
   // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    ESC.attach(14);  //49
    balancePID.SetMode(AUTOMATIC); //
    balancePID.SetOutputLimits(-176,344);//to range from 1312 to 1832( -176,344
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    calibrateESC();

 //Initialising the SD card in the setup
    Serial.begin(115200);
    
    //telemetry
    ConnectWifi();
    ListenUDP();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if ((mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) && (elapsedTime > sampleTime));  { // Get the Latest packet 
              // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        mpu.getMotion6(&accData[0], &accData[1], &accData[2], &gyrData[0], &gyrData[1], &gyrData[2]);
        
        Input = ypr[0] * 180/M_PI; 
        timePrev = timeCur;
        timeCur = millis();
        angleCur=Input;
        elapsedTime = timeCur - timePrev;
        rollVel = ((angleCur-anglePrev)/(elapsedTime/1000.00));
            
        struct SensorData sdt;
        sdt.accel[0] = accData[0];
        sdt.accel[1] = accData[1];
        sdt.accel[2] = accData[2];
        sdt.gyro[0] = gyrData[0];
        sdt.gyro[1] = gyrData[1];
        sdt.gyro[2] = gyrData[2];
        sdt.ypr[0] = ypr[0];
        sdt.ypr[1] = ypr[1];
        sdt.ypr[2] = ypr[2];

        PublishSensorDataUDP(sdt);
            
                  

//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);

            
            while ( Input<= -180) Input += 360; 
            while (Input > 180)   Input  -= 360;
       
          Serial.print("Input:\t");
          Serial.println(Input);
     

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        Setpoint = 0;

       // Serial.print("Input:\t");
       // Serial.println(Input);

        
        if(Input < 0){
        balancePID.SetControllerDirection(REVERSE);
        }
        else{
        balancePID.SetControllerDirection(DIRECT);
        }

        //Serial.print("Output1:\t");
        //Serial.println(Output);
        
        balancePID.Compute(); 

        //Serial.print("Output2:\t");
       // Serial.println(Output);
        pwm=1488+Output;
          constrainpwm(pwm,1450,1510);
        
           timePrev = timeCur;
           anglePrev = angleCur;
        //pwm Output
        //Serial.print("pwm: ");
        //Serial.println(pwm);   

        //pwm, output
        write_pwm(pwm);
        
        delay(1000);
            
        //Serial.println("Done");

        dataMessage = String(Input) + "," + String(rollVel) + "," + String(pwm) + "\r\n";
        
    }
    }