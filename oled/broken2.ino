// Include the Adafruit_SSD1306 library
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// Define the OLED display pins
#define OLED_MOSI  9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13

// Define the OLED display size
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Create an OLED object
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// Define the analog input pins
#define AC_INPUT_PIN 34 // for sampling 50Hz sine wave AC input
#define TRIGGER_PIN 35 // for trigger and phase detection

// Define the sampling frequency and period
#define SAMPLING_FREQ 1000 // in Hz
#define SAMPLING_PERIOD 1000 // in microseconds

// Define the buffer size and index
#define BUFFER_SIZE 40 // for two 50Hz periods minus one sample
#define BUFFER_INDEX 39 // the last index of the buffer

// Define the RC filter parameters
#define R 10000 // in ohms
#define C 0.000001 // in farads
#define RC (R * C) // in seconds
#define ALPHA (SAMPLING_PERIOD / 1000000.0) / (RC + (SAMPLING_PERIOD / 1000000.0)) // the smoothing factor

// Define the gradient descent parameters
#define LEARNING_RATE 0.01 // the step size
#define TOLERANCE 0.001 // the error threshold
#define MAX_ITER 100 // the maximum number of iterations

// Define the voltage threshold for zero crossing detection
#define V_THRESHOLD 1.65 // in volts

// Define the global variables
float triggerBuffer[BUFFER_SIZE]; // the buffer for the trigger input
float acBuffer[BUFFER_SIZE]; // the buffer for the AC input
float triggerValue; // the current value of the trigger input
float acValue; // the current value of the AC input
float triggerFiltered; // the filtered value of the trigger input
float acFiltered; // the filtered value of the AC input
float phase; // the phase offset in microseconds
float error; // the phase error in radians
float gradient; // the gradient of the error function
int iter; // the iteration counter
bool phaseLocked; // the flag for phase lock status

void setup() {
  // Initialize the serial communication
  Serial.begin(115200);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("ESP32 Oscilloscope");
  display.display();

  // Initialize the analog input pins
  pinMode(AC_INPUT_PIN, INPUT);
  pinMode(TRIGGER_PIN, INPUT);

  // Initialize the global variables
  triggerValue = 0;
  acValue = 0;
  triggerFiltered = 0;
  acFiltered = 0;
  phase = 0;
  error = 0;
  gradient = 0;
  iter = 0;
  phaseLocked = false;
}

void loop() {
  // Sample the trigger input into the buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Read the analog value and convert it to voltage
    triggerValue = analogRead(TRIGGER_PIN) * 3.3 / 4095.0;

    // Apply the RC filter to smooth the signal
    triggerFiltered = triggerFiltered + ALPHA * (triggerValue - triggerFiltered);

    // Store the filtered value in the buffer
    triggerBuffer[i] = triggerFiltered;

    // Wait for the sampling period
    delayMicroseconds(SAMPLING_PERIOD);
  }

  // Analyze the buffer to determine the phase offset
  phase = getPhaseOffset();

  // Adjust the phase of the sampling by introducing fine delay
  delayMicroseconds(phase);

  // Sample the AC input into the buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Read the analog value and convert it to voltage
    acValue = analogRead(AC_INPUT_PIN) * 3.3 / 4095.0;

    // Apply the RC filter to smooth the signal
    acFiltered = acFiltered + ALPHA * (acValue - acFiltered);

    // Store the filtered value in the buffer
    acBuffer[i] = acFiltered;

    // Wait for the sampling period
    delayMicroseconds(SAMPLING_PERIOD);
  }

  // Display the AC input waveform on the OLED display
  displayWaveform();

  // Check if the phase lock is achieved
  if (abs(error) < TOLERANCE) {
    phaseLocked = true;
  } else {
    phaseLocked = false;
  }

  // Print the phase offset, error, and lock status on the serial monitor
  Serial.print("Phase offset: ");
  Serial.print(phase);
  Serial.println(" us");
  Serial.print("Phase error: ");
  Serial.print(error);
  Serial.println(" rad");
  Serial.print("Phase lock: ");
  Serial.println(phaseLocked ? "Yes" : "No");
}

// A function that returns the phase offset in microseconds based on the buffer analysis
float getPhaseOffset() {
  // Initialize the local variables
  float sum = 0; // the sum of the zero crossing times
  float avg = 0; // the average of the zero crossing times
  float offset = 0; // the phase offset in microseconds
  int count = 0; // the number of zero crossings
  bool rising = false; // the flag for rising edge detection
  bool falling = false; // the flag for falling edge detection

  // Loop through the buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Check if the signal is rising
    if (triggerBuffer[i] > V_THRESHOLD && !rising) {
      rising = true;
      falling = false;
      // Interpolate the zero crossing time
      float t = i - 1 + (V_THRESHOLD - triggerBuffer[i-1]) / (triggerBuffer[i] - triggerBuffer[i-1]);
      // Add the time to the sum
      sum += t;
      // Increment the count
      count++;
    }
    // Check if the signal is falling
    if (triggerBuffer[i] < V_THRESHOLD && !falling) {
      rising = false;
      falling = true;
      // Interpolate the zero crossing time
      float t = i - 1 + (V_THRESHOLD - triggerBuffer[i-1]) / (triggerBuffer[i] - triggerBuffer[i-1]);
      // Add the time to the sum
      sum += t;
      // Increment the count
      count++;
    }
  }

  // Calculate the average of the zero crossing times
  avg = sum / count;

  // Calculate the phase offset in microseconds
  offset = avg * SAMPLING_PERIOD;

  // Update the phase error in radians
  error = 2 * PI * offset / 20000.0;

  // Update the gradient of the error function
  gradient = 2 * PI / 20000.0;

  // Update the iteration counter
  iter++;

  // Apply the gradient descent algorithm to correct the phase offset
  if (iter < MAX_ITER) {
    offset = offset - LEARNING_RATE * gradient * error;
  }

  // Return the phase offset
  return offset;
}

// A function that displays the AC input waveform on the OLED display
void displayWaveform() {
  // Clear the display
  display.clearDisplay();

  // Draw the waveform
  for (int i = 0; i < BUFFER_INDEX; i++) {
    // Map the voltage values to pixel coordinates
    int x1 = map(i, 0, BUFFER_INDEX, 0, OLED_WIDTH - 1);
    int y1 = map(acBuffer[i], 0, 3.3, OLED_HEIGHT - 1, 0);
    int x2 = map(i + 1, 0, BUFFER_INDEX, 0, OLED_WIDTH - 1);
    int y2 = map(acBuffer[i + 1], 0, 3.3, OLED_HEIGHT - 1, 0);

    // Draw a line between two consecutive points
    display.drawLine(x1, y1, x2, y2, WHITE);
  }

  // Display the waveform
  display.display();
}
