// This code is created by Bing using an online search tool
// This code is not tested and may contain errors
// For more information, visit [ESP32 Arduino Online Documentation](https://github.com/espressif/arduino-esp32)

// Include the libraries for SPI and OLED display
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define the pins for SPI and OLED display
#define OLED_MOSI  23
#define OLED_CLK   18
#define OLED_DC    21
#define OLED_CS    22
#define OLED_RESET 19
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Create an object for the OLED display
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// Define the pin for the input signal
#define INPUT_PIN 34

// Define the sampling rate in Hz
#define SAMPLING_RATE 1000

// Define the size of the sampling array
#define SAMPLING_SIZE 100

// Define the threshold for phase detection
#define PHASE_THRESHOLD 0.5

// Declare the global variables
float sampling_array[SAMPLING_SIZE]; // The array to store the input samples
float zero_array[SAMPLING_SIZE]; // The array to store the samples when the phase is zero
int sampling_index = 0; // The index for the sampling array
int zero_index = 0; // The index for the zero array
float phase = 0; // The current phase of the input signal
float prev_phase = 0; // The previous phase of the input signal
float phase_diff = 0; // The difference between the current and previous phase
bool phase_locked = false; // The flag to indicate if the phase is locked

// The setup function runs once when the board is powered on or reset
void setup() {
  // Initialize the serial port
  Serial.begin(115200);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Phase Oscillator");
  display.display();

  // Initialize the input pin as an analog input
  pinMode(INPUT_PIN, ANALOG);

  // Set the sampling timer
  timerAttachInterrupt(timerBegin(0, 80, true), samplingISR, true);
  timerAlarmWrite(timerBegin(0, 80, true), 1000000 / SAMPLING_RATE, true);
  timerAlarmEnable(timerBegin(0, 80, true));
}

// The loop function runs over and over again
void loop() {
  // Check if the phase is locked
  if (phase_locked) {
    // Print the content of the zero array in csv format to the serial port
    for (int i = 0; i < zero_index; i++) {
      Serial.print(zero_array[i]);
      Serial.print(",");
    }
    Serial.println();

    // Graph the content of the zero array on the OLED display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Phase Oscillator");
    display.drawLine(0, 10, OLED_WIDTH - 1, 10, WHITE);
    display.drawLine(0, OLED_HEIGHT - 1, OLED_WIDTH - 1, OLED_HEIGHT - 1, WHITE);
    for (int i = 0; i < zero_index - 1; i++) {
      int x1 = map(i, 0, zero_index - 1, 0, OLED_WIDTH - 1);
      int y1 = map(zero_array[i], 0, 3.3, OLED_HEIGHT - 2, 11);
      int x2 = map(i + 1, 0, zero_index - 1, 0, OLED_WIDTH - 1);
      int y2 = map(zero_array[i + 1], 0, 3.3, OLED_HEIGHT - 2, 11);
      display.drawLine(x1, y1, x2, y2, WHITE);
    }
    display.display();

    // Reset the zero index and the phase locked flag
    zero_index = 0;
    phase_locked = false;
  }
}

// The interrupt service routine for the sampling timer
void IRAM_ATTR samplingISR() {
  // Read the input signal and convert it to voltage
  float input = analogRead(INPUT_PIN) * 3.3 / 4095.0;

  // Store the input sample in the sampling array
  sampling_array[sampling_index] = input;

  // Increment the sampling index and wrap it around
  sampling_index = (sampling_index + 1) % SAMPLING_SIZE;

  // Calculate the phase of the input signal using a robust algorithm
  // For example, you can use the zero-crossing method or the Hilbert transform method
  // Here we use a simple method based on the average value of the input signal
  float avg = 0;
  for (int i = 0; i < SAMPLING_SIZE; i++) {
    avg += sampling_array[i];
  }
  avg /= SAMPLING_SIZE;
  phase = (input - avg) / (3.3 - avg) * PI;

  // Calculate the phase difference between the current and previous phase
  phase_diff = phase - prev_phase;

  // Check if the phase difference is within the threshold
  if (abs(phase_diff) < PHASE_THRESHOLD) {
    // Store the input sample in the zero array
    zero_array[zero_index] = input;

    // Increment the zero index and wrap it around
    zero_index = (zero_index + 1) % SAMPLING_SIZE;

    // Set the phase locked flag to true
    phase_locked = true;
  }

  // Update the previous phase
  prev_phase = phase;
}
