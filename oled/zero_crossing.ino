// This code is generated by Bing and may not be accurate or complete
// It is based on some web search results and not tested on a real device
// Use it at your own risk and always verify the functionality

// Include the libraries for ESP32 and OLED display
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define the pins for the OLED display
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16
#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Create an OLED display object
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RST);

// Define the pin for the input signal
#define INPUT_PIN 34

// Define the sampling rate in Hz
#define SAMPLING_RATE 1000

// Define the sampling period in microseconds
#define SAMPLING_PERIOD (1000000 / SAMPLING_RATE)

// Define the size of the sampling array
#define SAMPLING_SIZE 100

// Define the threshold for detecting zero crossing
#define ZERO_THRESHOLD 0.1

// Declare the sampling array
float samples[SAMPLING_SIZE];

// Declare the index of the current sample
int sample_index = 0;

// Declare the flag for indicating a new sample
bool new_sample = false;

// Declare the flag for indicating a zero crossing
bool zero_crossing = false;

// Declare the variable for storing the previous sample
float prev_sample = 0;

// Declare the variable for storing the phase
float phase = 0;

// Declare the variable for storing the phase increment
float phase_inc = 0;

// Declare the variable for storing the oscillator output
float osc_output = 0;

// Declare the array for storing the samples at zero phase
float zero_samples[SAMPLING_SIZE];

// Declare the index of the current zero sample
int zero_index = 0;

// Declare the flag for indicating a new zero sample
bool new_zero_sample = false;

// Declare the flag for indicating a full zero array
bool full_zero_array = false;

// Declare the variable for storing the minimum value of the zero array
float zero_min = 0;

// Declare the variable for storing the maximum value of the zero array
float zero_max = 0;

// Declare the variable for storing the scaling factor for the OLED display
float zero_scale = 0;

// Declare the variable for storing the offset for the OLED display
float zero_offset = 0;

// The setup function runs once when the board is powered on or reset
void setup() {
  // Initialize the serial port
  Serial.begin(115200);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Grid-locked phase oscillator");
  display.display();

  // Initialize the input pin as an analog input
  pinMode(INPUT_PIN, ANALOG);

  // Initialize the timer interrupt for sampling
  timerAttachInterrupt(timerBegin(0, 80, true), sampleISR, true);
  timerAlarmWrite(timerBegin(0, 80, true), SAMPLING_PERIOD, true);
  timerAlarmEnable(timerBegin(0, 80, true));
}

// The loop function runs over and over again
void loop() {
  // Check if there is a new sample
  if (new_sample) {
    // Reset the new sample flag
    new_sample = false;

    // Check if there is a zero crossing
    if (zero_crossing) {
      // Reset the zero crossing flag
      zero_crossing = false;

      // Reset the phase to zero
      phase = 0;

      // Reset the zero index to zero
      zero_index = 0;

      // Reset the full zero array flag
      full_zero_array = false;
    }

    // Update the phase by adding the phase increment
    phase += phase_inc;

    // Wrap the phase to the range [0, 2*PI]
    phase = fmod(phase, 2 * PI);

    // Generate the oscillator output as a sine wave
    osc_output = sin(phase);

    // Store the oscillator output in the zero array if the phase is zero
    if (phase == 0) {
      zero_samples[zero_index] = osc_output;
      zero_index++;
      new_zero_sample = true;
    }

    // Check if the zero array is full
    if (zero_index == SAMPLING_SIZE) {
      // Set the full zero array flag
      full_zero_array = true;

      // Find the minimum and maximum values of the zero array
      zero_min = zero_samples[0];
      zero_max = zero_samples[0];
      for (int i = 1; i < SAMPLING_SIZE; i++) {
        if (zero_samples[i] < zero_min) {
          zero_min = zero_samples[i];
        }
        if (zero_samples[i] > zero_max) {
          zero_max = zero_samples[i];
        }
      }

      // Calculate the scaling factor and offset for the OLED display
      zero_scale = (OLED_HEIGHT - 2) / (zero_max - zero_min);
      zero_offset = 1 - zero_min * zero_scale;
    }
  }

  // Check if there is a new zero sample
  if (new_zero_sample) {
    // Reset the new zero sample flag
    new_zero_sample = false;

    // Print the zero sample to the serial port in csv format
    Serial.print(zero_samples[zero_index - 1], 4);
    Serial.print(",");
  }

  // Check if the zero array is full
  if (full_zero_array) {
    // Reset the full zero array flag
    full_zero_array = false;

    // Print a new line to the serial port
    Serial.println();

    // Clear the OLED display
    display.clearDisplay();

    // Draw the zero array on the OLED display as a line graph
    for (int i = 0; i < SAMPLING_SIZE - 1; i++) {
      display.drawLine(i, zero_samples[i] * zero_scale + zero_offset, i + 1, zero_samples[i + 1] * zero_scale + zero_offset, WHITE);
    }

    // Update the OLED display
    display.display();
  }
}

// The interrupt service routine for sampling
void IRAM_ATTR sampleISR() {
  // Read the input signal as a voltage
  float input = analogRead(INPUT_PIN) * 3.3 / 4095;

  // Store the input in the sampling array
  samples[sample_index] = input;

  // Increment the sample index
  sample_index++;

  // Wrap the sample index to the range [0, SAMPLING_SIZE - 1]
  sample_index = sample_index % SAMPLING_SIZE;

  // Set the new sample flag
  new_sample = true;

  // Check if the input crosses zero from positive to negative
  if (prev_sample > ZERO_THRESHOLD && input < -ZERO_THRESHOLD) {
    // Set the zero crossing flag
    zero_crossing = true;

    // Calculate the phase increment based on the zero crossing interval
    phase_inc = 2 * PI / (SAMPLING_RATE * sample_index);
  }

  // Update the previous sample
  prev_sample = input;
}