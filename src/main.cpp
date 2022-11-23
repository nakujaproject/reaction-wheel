#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <FastLED.h>
#include <SPI.h>
#include <SD.h>
#include <state_machine.h>
#include <PID.h>
#include <ESP32Servo.h>
#include <models.pb.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_decode.h>
#include <pb_encode.h>

Servo ESC;

state_machine sm;

PID pid(10,0,0);

const int ChipSelect = 5;
File myFIle;

#define DATA_PIN    14

#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    2
#define BRIGHTNESS  255

#define AVIONICS_STATE 17

CRGB leds[NUM_LEDS];

MPU6050 mpu(Wire);

SPIClass spiSD(HSPI);

uint8_t buffer[128];


const char* ssid = "sammy2";
const char* password = "12345678";

unsigned long previous_time;
unsigned long elapsed_time;


void initESC(){
  ESC.writeMicroseconds(1000);
  Serial.println("setup");
  delay(2000);
  ESC.writeMicroseconds(2000);
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  
  spiSD.begin(13, 15, 4, 5);

  ESC.attach(33); // (pin, min pulse width, max pulse width in microseconds) 
  delay(2000);
  
  //initESC();
  

  pinMode(AVIONICS_STATE, INPUT);
  
  EEPROM.begin(1);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int resets = EEPROM.read(0);
  Serial.print("resets: ");
  Serial.println(resets);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  if (resets != 1) {
    EEPROM.write(0,1);
    EEPROM.commit();
    delay(100);
    resets = EEPROM.read(0);
    Serial.print("resets: ");
    Serial.println(resets);
    delay(2000);
    ESP.restart();
  }

  EEPROM.write(0,0);
  EEPROM.commit();
  delay(2000);

  Wire.begin();

  mpu.setAddress(0x69);
  
  byte status = mpu.begin();

  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(TypicalLEDStrip)
    .setDither(BRIGHTNESS < 255);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  Serial.print("Initializing SD card...");

  if (!SD.begin(ChipSelect, spiSD)) {
    Serial.println("initialization failed!");
    //while (1);
  }
  Serial.println("initialization done.");

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  mpu.update();
  float roll = mpu.getGyroY();

  SensorEvent sensorReading = SensorEvent_init_zero;
  sensorReading.accRaw.x = mpu.getAccX();
  sensorReading.accRaw.y = mpu.getAccY();
  sensorReading.accRaw.z = mpu.getAccZ();
  sensorReading.gyroRaw.x = mpu.getGyroX();
  sensorReading.gyroRaw.y = mpu.getGyroY();
  sensorReading.gyroRaw.z = mpu.getGyroZ();
  sensorReading.angles.x = mpu.getAccAngleX();
  sensorReading.angles.y = mpu.getAccAngleY();


  if (roll < 0) {
    leds[0] = CRGB::Red;//ccw
    FastLED.show();
  }else{
    leds[0] = CRGB::Green;//cw
    FastLED.show();
  }
  delay(1000);

  switch(sm.getCurrentState()) {
    case RocketState::idle:{
      Serial.print("state: ");
  Serial.println(0);
      //ArduinoOTA.handle();
      leds[1] = CRGB::Indigo;
      FastLED.show();
      if (digitalRead(AVIONICS_STATE) == HIGH || !WiFi.isConnected()){
        previous_time = millis();
        sm.transition();
        break;
      }
      break;
    }
    case RocketState::active_flight:{
      elapsed_time = millis() - previous_time;
      leds[1] = CRGB::Orange;
      FastLED.show();
      Serial.print("state: ");
      Serial.println(1);
      float error = 0 - roll;
      double pidOut = pid.Calculate(error, millis());
      double ESCInpt = pidOut + 1500;
      sensorReading.esc = ESCInpt;
      ESC.writeMicroseconds(ESCInpt);

      pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    	if (!pb_encode(&stream, SensorEvent_fields, &sensorReading))
    	{
        	Serial.println("failed to encode temp proto");
        	Serial.println(PB_GET_ERROR(&stream));
        	return;
    	}

      myFIle = SD.open("/data.txt", FILE_APPEND);
      if (myFIle) {
        for(int i = 0; i<stream.bytes_written; i++){
          myFIle.printf("%02X",buffer[i]);
        }
        myFIle.println();
      }
      myFIle.close();
      Serial.print("elapsed time: ");
      Serial.println(elapsed_time);
      if (digitalRead(AVIONICS_STATE) == LOW && elapsed_time > 20000){
        sm.transition();
        break;
      }
      break;
    }
    case RocketState::post_flight:{
      Serial.print("state: ");
  Serial.println(2);
      leds[1] = CRGB::Cyan;
      FastLED.show();
      myFIle = SD.open("/data.txt", FILE_APPEND);
      pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    	if (!pb_encode(&stream, SensorEvent_fields, &sensorReading))
    	{
        	Serial.println("failed to encode temp proto");
        	Serial.println(PB_GET_ERROR(&stream));
        	return;
    	}
      if (myFIle) {
        for(int i = 0; i<stream.bytes_written; i++){
          myFIle.printf("%02X",buffer[i]);
        }
        myFIle.println();
      }
      myFIle.close();
      break;
    }
  }
  ArduinoOTA.handle();
}