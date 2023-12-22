// Define the pins for analog inputs and OLED display
#define ANALOG_INPUT_1 34 // AC input
#define ANALOG_INPUT_2 35 // Trigger input
#define OLED_DC 4
#define OLED_CS 5
#define OLED_RESET 16

// Include the libraries for OLED display and SPI communication
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>

// Create an OLED object
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

// Define some constants for the oscilloscope
const int BUFFER_SIZE = 736; // Number of samples per screen
const int SAMPLING_RATE = 1000; // Sampling rate in microseconds
const int TRIGGER_THRESHOLD = 2048; // Threshold for trigger detection
const int PHASE_ERROR_TOLERANCE = 10; // Tolerance for phase error correction in microseconds
const int SCREEN_WIDTH = 128; // Width of the OLED screen
const int SCREEN_HEIGHT = 64; // Height of the OLED screen
const int VOLTAGE_SCALE = 8; // Voltage scale factor for display

// Declare some global variables for the oscilloscope
int buffer1[BUFFER_SIZE]; // Buffer for AC input
int buffer2[BUFFER_SIZE]; // Buffer for trigger input
int phaseError = 0; // Phase error between AC input and trigger input
int phaseCorrection = 0; // Phase correction for sampling
int bufferIndex = 0; // Index for buffer filling
bool bufferReady = false; // Flag for buffer readiness
bool triggerDetected = false; // Flag for trigger detection

void setup() {
  // Initialize the serial communication
  Serial.begin(115200);

  // Initialize the analog inputs
  pinMode(ANALOG_INPUT_1, INPUT);
  pinMode(ANALOG_INPUT_2, INPUT);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 Oscilloscope");
  display.display();
  delay(2000);
}

void loop() {
  // Fill the buffers with samples
  fillBuffers();

  // Check if the buffers are ready
  if (bufferReady) {
    // Calculate the phase error
    calculatePhaseError();

    // Adjust the phase correction
    adjustPhaseCorrection();

    // Display the waveform
    displayWaveform();

    // Reset the buffer index and flag
    bufferIndex = 0;
    bufferReady = false;
  }
}

// Function to fill the buffers with samples
void fillBuffers() {
  // Get the current time
  unsigned long currentTime = micros();

  // Check if it is time to take a sample
  if (currentTime % SAMPLING_RATE == phaseCorrection) {
    // Read the analog inputs
    int value1 = analogRead(ANALOG_INPUT_1);
    int value2 = analogRead(ANALOG_INPUT_2);

    // Store the values in the buffers
    buffer1[bufferIndex] = value1;
    buffer2[bufferIndex] = value2;

    // Increment the buffer index
    bufferIndex++;

    // Check if the buffer is full
    if (bufferIndex == BUFFER_SIZE) {
      // Set the buffer ready flag
      bufferReady = true;
    }

    // Check if the trigger is detected
    if (!triggerDetected) {
      // Check if the trigger input crosses the threshold
      if (value2 > TRIGGER_THRESHOLD) {
        // Set the trigger detected flag
        triggerDetected = true;
      }
    }
    else {
      // Check if the trigger input falls below the threshold
      if (value2 < TRIGGER_THRESHOLD) {
        // Reset the trigger detected flag
        triggerDetected = false;
      }
    }
  }
}

// Function to calculate the phase error between the AC input and the trigger input
void calculatePhaseError() {
  // Declare some local variables
  int triggerIndex = 0; // Index for trigger buffer analysis
  int triggerCount = 0; // Count for trigger buffer analysis
  int triggerPeriod = 0; // Period for trigger buffer analysis
  int triggerOffset = 0; // Offset for trigger buffer analysis
  bool triggerRising = false; // Flag for trigger buffer analysis

  // Loop through the trigger buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Check if the trigger input crosses the threshold
    if (buffer2[i] > TRIGGER_THRESHOLD) {
      // Check if the trigger is rising
      if (!triggerRising) {
        // Set the trigger rising flag
        triggerRising = true;

        // Increment the trigger count
        triggerCount++;

        // Check if the trigger count is 1
        if (triggerCount == 1) {
          // Store the trigger index
          triggerIndex = i;
        }
        // Check if the trigger count is 2
        else if (triggerCount == 2) {
          // Calculate the trigger period
          triggerPeriod = i - triggerIndex;

          // Break the loop
          break;
        }
      }
    }
    else {
      // Reset the trigger rising flag
      triggerRising = false;
    }
  }

  // Check if the trigger period is valid
  if (triggerPeriod > 0) {
    // Calculate the trigger offset
    triggerOffset = triggerIndex + triggerPeriod - BUFFER_SIZE;

    // Calculate the phase error
    phaseError = triggerOffset * SAMPLING_RATE;
  }
}

// Function to adjust the phase correction for sampling
void adjustPhaseCorrection() {
  // Check if the phase error is within the tolerance
  if (abs(phaseError) < PHASE_ERROR_TOLERANCE) {
    // Reset the phase correction
    phaseCorrection = 0;
  }
  else {
    // Update the phase correction
    phaseCorrection = phaseCorrection - phaseError;
  }
}

// Function to display the waveform on the OLED screen
void displayWaveform() {
  // Clear the display
  display.clearDisplay();

  // Draw the waveform
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    // Map the buffer value to the screen height
    int y = map(buffer1[i], 0, 4095, SCREEN_HEIGHT, 0) / VOLTAGE_SCALE;

    // Draw a pixel on the display
    display.drawPixel(i, y, WHITE);
  }

  // Display the buffer
  display.display();
}
