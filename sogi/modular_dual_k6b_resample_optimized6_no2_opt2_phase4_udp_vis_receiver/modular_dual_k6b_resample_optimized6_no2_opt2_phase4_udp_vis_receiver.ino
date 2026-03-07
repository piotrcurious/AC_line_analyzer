#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "SOGIvisualizer.h"

/* WiFi Credentials - CHANGE THESE */
const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";
const char* udpAddress = "239.1.2.3";
const int udpPort = 12345;

WiFiUDP udp;
SOGIVisualizer vis;

struct VisPacket {
    float    freq;
    uint16_t index;
    int16_t  v[128];
    int16_t  i[128];
} __attribute__((packed));

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.printf("Connecting to %s\n", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");

    udp.beginMulticast(IPAddress(239, 1, 2, 3), udpPort);
    vis.begin();
    Serial.println("UDP Multicast listener started");
}

float v_float[128];
float i_float[128];

void loop() {
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(VisPacket)) {
        VisPacket packet;
        udp.read((uint8_t*)&packet, sizeof(packet));

        // Convert int16 back to float for visualizer
        // Since they were centered at DC, we can pass 0 for DC offsets
        for (int i = 0; i < 128; i++) {
            v_float[i] = (float)packet.v[i];
            i_float[i] = (float)packet.i[i];
        }

        // update(const float* vBuffer, const float* iBuffer, int bufLen, int startIdx, int count,
        //        float freq, float magnitude, float v_dc, float i_dc)
        vis.update(v_float, i_float, 128, 0, 128, packet.freq, 0.0f, 0.0f, 0.0f);

        static uint32_t last_log = 0;
        if (millis() - last_log > 1000) {
            Serial.printf("Received packet %u, Freq: %.4f Hz\n", packet.index, packet.freq);
            last_log = millis();
        }
    }
}
