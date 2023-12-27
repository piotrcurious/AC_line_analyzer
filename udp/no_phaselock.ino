// ESP32 Arduino code to measure and record one full AC period from AC voltage source
// and send it over UDP broadcast packet
// The code is based on the following sources:
// [1](https://www.circuitschools.com/measure-ac-current-by-interfacing-acs712-sensor-with-esp32/) - How to use ADC of ESP32 - Measuring voltage example
// [2](https://forum.arduino.cc/t/voltage-sensor-analog-with-esp32/953852) - Voltage sensor (analog) with ESP32
// [3](https://www.instructables.com/ESP32-WIFI-Autoconnect-and-UDP-Broadcast/) - ESP32 WIFI Autoconnect and UDP Broadcast
// [4](https://microcontrollerslab.com/ac-voltage-measurement-arduino/) - AC Voltage Measurement using Arduino - Difference Amplifier Technique

#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi credentials
const char* ssid = "your-ssid";
const char* password = "your-password";

// UDP settings
const int udpPort = 1234; // UDP port to send packets
WiFiUDP udp; // UDP object
IPAddress broadcastIP; // broadcast IP address

// ADC settings
const int adcPin = 34; // analog input pin
const int adcResolution = 12; // ADC resolution in bits
const int adcMaxValue = pow(2, adcResolution) - 1; // ADC maximum value
const float adcRefVoltage = 3.3; // ADC reference voltage in volts
const float adcScaleFactor = adcRefVoltage / adcMaxValue; // ADC scale factor
const float voltageOffset = 20; // voltage offset in millivolts
const float voltageGain = 1.65; // voltage gain of the difference amplifier

// AC settings
const float acFrequency = 50; // AC frequency in hertz
const float acPeriod = 1000 / acFrequency; // AC period in milliseconds
const int acSamples = 100; // number of samples to take per AC period
const float acSampleTime = acPeriod / acSamples; // sampling time in milliseconds
const int acBufferSize = acSamples + 1; // buffer size to store the samples and the index
byte acBuffer[acBufferSize]; // buffer to store the samples and the index
byte acIndex = 0; // index to indicate the start of a new AC period

void setup() {
  Serial.begin(115200); // initialize serial monitor
  WiFi.begin(ssid, password); // connect to WiFi network
  while (WiFi.status() != WL_CONNECTED) { // wait until connected
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  broadcastIP = WiFi.localIP(); // get the local IP address
  broadcastIP[3] = 255; // change the last octet to 255 for broadcast
  udp.begin(udpPort); // start UDP
}

void loop() {
  measureAC(); // measure and record one full AC period
  sendUDP(); // send the buffer over UDP broadcast
}

void measureAC() {
  acIndex++; // increment the index
  if (acIndex > 255) { // reset the index if it exceeds 255
    acIndex = 0;
  }
  acBuffer[0] = acIndex; // store the index in the first byte of the buffer
  for (int i = 1; i < acBufferSize; i++) { // loop through the buffer
    int adcValue = analogRead(adcPin); // read the analog value
    float voltage = adcValue * adcScaleFactor; // convert to voltage
    voltage = voltage - voltageOffset / 1000; // subtract the offset
    voltage = voltage * voltageGain; // multiply by the gain
    voltage = voltage * 100; // convert to centivolts
    byte sample = constrain(voltage, 0, 255); // convert to byte and limit to 0-255
    acBuffer[i] = sample; // store the sample in the buffer
    delay(acSampleTime); // wait for the sampling time
  }
}

void sendUDP() {
  udp.beginPacket(broadcastIP, udpPort); // start UDP packet
  udp.write(acBuffer, acBufferSize); // write the buffer to the packet
  udp.endPacket(); // end and send the packet
  Serial.print("Sent UDP packet with index ");
  Serial.println(acIndex);
}
