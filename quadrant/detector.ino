#include <Arduino.h>

const int analogPin = 34; // Change this to your ESP32's analog pin
const int samplesPerSecond = 1000; // Sample rate
const float referenceVoltage = 3.3; // Maximum voltage ESP32 can read
const int adcResolution = 4095; // 12-bit ADC resolution for ESP32

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Set ADC resolution to 12 bits
}

void loop() {
  int rawValue = analogRead(analogPin); // Read the analog pin
  float voltage = (rawValue / adcResolution) * referenceVoltage; // Convert to voltage
  float angle = asin(voltage / referenceVoltage) * (180.0 / PI); // Calculate the angle

  // Determine the quadrant
  int quadrant;
  if (angle >= 0 && angle <= 90) {
    quadrant = 1;
  } else if (angle > 90 && angle <= 180) {
    quadrant = 2;
  } else if (angle < 0 && angle >= -90) {
    quadrant = 3;
  } else { // angle < -90 && angle >= -180
    quadrant = 4;
  }

  // Output the quadrant
  Serial.print("Quadrant: ");
  Serial.println(quadrant);

  delay(1000 / samplesPerSecond); // Wait for the next sample
}
