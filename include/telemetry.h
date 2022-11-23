#ifndef TELEMETRY_H
#define TELEMETRY_H

//#include "WiFi.h"
#include <PubSubClient.h>

#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <Ticker.h>
Ticker ticker;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13 // ESP32 DOES NOT DEFINE LED_BUILTIN
#endif

int LED = LED_BUILTIN;
struct SensorData
{
    float ypr[3];
    int16_t gyro[3];
    int16_t accel[3];
};

const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (250)
char msg[MSG_BUFFER_SIZE];

int rocketState;

void CheckState(char* topic, byte* payload, unsigned int length) {
    /*char str[length+1];
    int i = 0;
    for (i = 0; i < length; i++) {
        str[i] = (char)payload[i];
    }
    str[i] = 0;*/
    StaticJsonDocument <256> doc;
    deserializeJson(doc, payload);

    rocketState = doc["state"];
}
void tick()
{
  //toggle state
  digitalWrite(LED, !digitalRead(LED));     // set pin to the opposite state
}
//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void ConnectWifi() {
    WiFi.mode(WIFI_STA);
    pinMode(LED, OUTPUT);
    ticker.attach(0.6, tick);
    WiFiManager wm;

    wm.setAPCallback(configModeCallback);
    if (!wm.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
    }
    Serial.println("FC connected...yeey :)");
    ticker.detach();
}

void ConfigMQTT () {
    client.setServer(mqtt_server, 1883);
    client.setCallback(CheckState);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe("NakujaState");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void PublishSensorData (SensorData sd) {
    StaticJsonDocument<256> doc;
    JsonArray yprVals = doc.createNestedArray("ypr");
    yprVals.add(sd.ypr[0]);
    yprVals.add(sd.ypr[1]);
    yprVals.add(sd.ypr[2]);
    JsonArray gyroVals = doc.createNestedArray("gyro");
    gyroVals.add(sd.gyro[0]);
    gyroVals.add(sd.gyro[1]);
    gyroVals.add(sd.gyro[2]);
    JsonArray accelVals = doc.createNestedArray("accel");
    accelVals.add(sd.accel[0]);
    accelVals.add(sd.accel[1]);
    accelVals.add(sd.accel[2]);
    char out[128];
    serializeJson(doc, out);
    client.publish("NakujaTelemetry", out);
}


#endif
