/*
Explanation:
Circular Buffer: The circular buffer (analogBuffer) stores recent samples of the analog input, allowing for more robust signal analysis.

Zero-Crossing Detection: The detectZeroCrossing() function calculates the average value of the samples in the buffer and checks for zero crossings by comparing consecutive samples to this average value.

Derivative Calculation: The detectQuadrant() function calculates the first and second derivatives of the signal. The first derivative helps identify rising and falling edges, while the second derivative helps identify peaks and troughs.

Quadrant Detection: Based on the calculated derivatives, the function determines the current quadrant:

Quadrant 1: Identified by a rising edge in the first derivative.
Quadrant 2: Identified by a peak (positive to negative transition in the second derivative).
Quadrant 3: Identified by a falling edge in the first derivative.
Quadrant 4: Identified by a trough (negative to positive transition in the second derivative).
Internal Oscillator: The internal oscillator is simulated using a timer and an LED, toggled at a frequency of 50 Hz as an example.

This updated code provides a more robust method for detecting zero crossings and quadrants, utilizing derivatives to enhance the accuracy of feature detection.

*/



#include <Arduino.h>

// Constants for the analog input pin and the frequency of the internal oscillator
const int analogInputPin = 34;
const int ledPin = 2; // LED pin to visualize the internal oscillator

// Constants for the circular buffer
const int bufferSize = 32;
int analogBuffer[bufferSize];
int bufferIndex = 0;

// Variables to hold the current and previous analog values
int currentAnalogValue = 0;
int previousAnalogValue = 0;

// Variables to detect zero-crossing and quadrant
bool zeroCrossingDetected = false;
int quadrant = 1; // 1: 0-90°, 2: 90-180°, 3: 180-270°, 4: 270-360°

unsigned long lastMicros = 0;
const int periodMicroseconds = 1000000 / 50; // Example: 50 Hz internal oscillator

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Initialize the circular buffer
  for (int i = 0; i < bufferSize; i++) {
    analogBuffer[i] = 0;
  }
}

void loop() {
  currentAnalogValue = analogRead(analogInputPin);
  analogBuffer[bufferIndex] = currentAnalogValue;
  bufferIndex = (bufferIndex + 1) % bufferSize;

  zeroCrossingDetected = detectZeroCrossing();
  detectQuadrant();

  if (zeroCrossingDetected) {
    Serial.println("Zero Crossing Detected");
  }
  Serial.print("Quadrant: ");
  Serial.println(quadrant);

  unsigned long currentMicros = micros();
  if (currentMicros - lastMicros >= periodMicroseconds) {
    lastMicros = currentMicros;
    digitalWrite(ledPin, !digitalRead(ledPin));
  }

  previousAnalogValue = currentAnalogValue;
  delay(10);
}

bool detectZeroCrossing() {
  long sum = 0;
  for (int i = 0; i < bufferSize; i++) {
    sum += analogBuffer[i];
  }
  int averageValue = sum / bufferSize;

  bool zeroCrossing = false;
  for (int i = 1; i < bufferSize; i++) {
    if ((analogBuffer[i - 1] < averageValue && analogBuffer[i] >= averageValue) ||
        (analogBuffer[i - 1] > averageValue && analogBuffer[i] <= averageValue)) {
      zeroCrossing = true;
      break;
    }
  }
  return zeroCrossing;
}

void detectQuadrant() {
  long sum = 0;
  for (int i = 0; i < bufferSize; i++) {
    sum += analogBuffer[i];
  }
  int averageValue = sum / bufferSize;

  // Calculate first and second derivatives
  int firstDerivative[bufferSize];
  int secondDerivative[bufferSize];
  for (int i = 1; i < bufferSize; i++) {
    firstDerivative[i] = analogBuffer[i] - analogBuffer[i - 1];
    if (i > 1) {
      secondDerivative[i] = firstDerivative[i] - firstDerivative[i - 1];
    } else {
      secondDerivative[i] = 0;
    }
  }

  // Detect transitions using derivatives
  for (int i = 2; i < bufferSize; i++) {
    if (firstDerivative[i] > 0 && firstDerivative[i - 1] <= 0) {
      quadrant = 1; // Rising edge, 0-90°
    } else if (firstDerivative[i] <= 0 && firstDerivative[i - 1] > 0) {
      quadrant = 3; // Falling edge, 180-270°
    } else if (secondDerivative[i] > 0 && secondDerivative[i - 1] <= 0) {
      quadrant = 2; // Peak, 90-180°
    } else if (secondDerivative[i] <= 0 && secondDerivative[i - 1] > 0) {
      quadrant = 4; // Trough, 270-360°
    }
  }
}
