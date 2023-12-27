#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi settings
const char* ssid = "yourNetwork";
const char* password = "yourPassword";

// UDP settings
WiFiUDP udp;
const int localPort = 2000; // port to listen on
const IPAddress broadcastIP (192, 168, 1, 255); // broadcast address of the network
const int remotePort = 2000; // port to send to

// ADC settings
const int analogPin = 34; // pin to read AC voltage from
const int samples = 256; // number of samples to take per period
const float vref = 3.3; // reference voltage of ADC
const int adcResolution = 4096; // resolution of ADC
const float voltageFactor = 0.5; // voltage divider factor

// Phase synchronization settings
const int zeroCrossPin = 35; // pin to detect zero crossing
const int zeroCrossDelay = 100; // delay in microseconds after zero crossing
volatile bool zeroCrossFlag = false; // flag to indicate zero crossing

// Buffer settings
const int bufferSize = 1450; // maximum size of UDP packet
byte buffer[bufferSize]; // buffer to store data
int bufferIndex = 0; // index of buffer
byte packetIndex = 0; // index of packet

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Initialize UDP
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  // Initialize ADC
  analogReadResolution(12); // set ADC resolution to 12 bits
  analogSetAttenuation(ADC_11db); // set ADC attenuation to 11 dB

  // Initialize zero crossing interrupt
  pinMode(zeroCrossPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zeroCrossISR, FALLING);
}

void loop() {
  // Wait for zero crossing
  while (!zeroCrossFlag) {
    delayMicroseconds(1);
  }
  zeroCrossFlag = false;

  // Delay after zero crossing
  delayMicroseconds(zeroCrossDelay);

  // Read and store samples
  for (int i = 0; i < samples; i++) {
    int adcValue = analogRead(analogPin); // read ADC value
    float voltage = adcValue * vref / adcResolution / voltageFactor; // convert to voltage
    int voltageInt = round(voltage * 100); // convert to integer with two decimal places
    byte voltageHigh = highByte(voltageInt); // get high byte of voltage
    byte voltageLow = lowByte(voltageInt); // get low byte of voltage
    buffer[bufferIndex++] = voltageHigh; // store high byte in buffer
    buffer[bufferIndex++] = voltageLow; // store low byte in buffer

    // Check if buffer is full
    if (bufferIndex == bufferSize) {
      // Send buffer as UDP packet
      udp.beginPacket(broadcastIP, remotePort);
      udp.write(packetIndex); // write packet index
      udp.write(buffer, bufferSize); // write buffer data
      udp.endPacket();
      Serial.print("Sent packet ");
      Serial.println(packetIndex);

      // Increment packet index
      packetIndex++;

      // Reset buffer index
      bufferIndex = 0;
    }
  }
}

// Interrupt service routine for zero crossing
void zeroCrossISR() {
  zeroCrossFlag = true;
}
