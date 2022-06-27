#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "WiFi.h"
#include "AsyncUDP.h"

struct SensorData
{
    float ypr[3];
    int16_t gyro[3];
    int16_t accel[3];
};

const char * ssid = "sammy2";
const char * password = "12345678";

AsyncUDP udp;

int port = 1234;

void ConnectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    delay(100);

    Serial.print("Connecting...");
    while (WiFi.waitForConnectResult() != WL_CONNECTED) { 
      Serial.print(".");    
      delay(500);
    }
}
void ListenUDP() {
    if(udp.listen(port)) {
        udp.onPacket([](AsyncUDPPacket packet) {
          Serial.print("Received data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
        });
    }
}
void PublishSensorDataUDP(SensorData d) {
    char buf(100);
    sprintf(&buf, "%s %s %s %s %s %s", String(d.ypr[0]), String(d.ypr[1]),String(d.ypr[2]),String(d.gyro[0]),String(d.gyro[1]),String(d.gyro[2]),String(d.accel[0]),String(d.accel[1]),String(d.accel[2]) );
    udp.broadcastTo(&buf, port);
}
#endif
