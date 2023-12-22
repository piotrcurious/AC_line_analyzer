// Define the analog input pins
#define AC_PIN 34 // the pin for the AC input
#define TRIGGER_PIN 35 // the pin for the trigger input

// Define the OLED display parameters
#define OLED_CS 5 // the chip select pin for the OLED
#define OLED_DC 4 // the data/command pin for the OLED
#define OLED_RESET 16 // the reset pin for the OLED
#define OLED_WIDTH 128 // the width of the OLED in pixels
#define OLED_HEIGHT 64 // the height of the OLED in pixels

// Include the libraries for the OLED display and the SPI communication
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>

// Create an object for the OLED display
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// Define the sampling frequency and the buffer size
#define SAMPLING_FREQ 1000 // the sampling frequency in Hz
#define BUFFER_SIZE 40 // the buffer size in samples

// Define the phase shift detection parameters
#define PERIOD 20 // the period of the 50 Hz signal in samples
#define THRESHOLD 2048 // the threshold for the trigger input

// Declare the global variables for the buffers and the indices
int ac_buffer[BUFFER_SIZE]; // the buffer for the AC input
int trigger_buffer[BUFFER_SIZE]; // the buffer for the trigger input
int ac_index = 0; // the index for the AC buffer
int trigger_index = 0; // the index for the trigger buffer
int phase_error = 0; // the phase error in samples
int period_start = 0; // the estimated start of the period in microseconds

// The setup function runs once when the board is powered on or reset
void setup() {
  // Initialize the serial communication for debugging
  Serial.begin(115200);

  // Initialize the analog input pins
  pinMode(AC_PIN, INPUT);
  pinMode(TRIGGER_PIN, INPUT);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 Oscilloscope");
  display.display();
}

// The loop function runs repeatedly after the setup function is completed
void loop() {
  // Get the current time in microseconds
  unsigned long current_time = micros();

  // Check if it is time to sample the AC input
  if (current_time - period_start >= ac_index * 1000000 / SAMPLING_FREQ) {
    // Read the analog value from the AC pin and store it in the AC buffer
    ac_buffer[ac_index] = analogRead(AC_PIN);

    // Increment the AC index and wrap it around if it reaches the buffer size
    ac_index = (ac_index + 1) % BUFFER_SIZE;
  }

  // Check if it is time to sample the trigger input
  if (current_time - period_start >= trigger_index * 1000000 / SAMPLING_FREQ) {
    // Read the analog value from the trigger pin and store it in the trigger buffer
    trigger_buffer[trigger_index] = analogRead(TRIGGER_PIN);

    // Increment the trigger index and wrap it around if it reaches the buffer size
    trigger_index = (trigger_index + 1) % BUFFER_SIZE;
  }

  // Check if the trigger buffer is full
  if (trigger_index == 0) {
    // Perform the phase shift detection algorithm
    phase_error = detect_phase_error();

    // Adjust the next estimate of the period start by using the phase error
    period_start = current_time + phase_error * 1000000 / SAMPLING_FREQ;

    // Display the AC buffer on the OLED display
    display_buffer();
  }
}

// The function to detect the phase error from the trigger buffer
int detect_phase_error() {
  // Declare a variable to store the phase error
  int error = 0;

  // Declare a variable to store the previous value of the trigger input
  int prev_value = trigger_buffer[BUFFER_SIZE - 1];

  // Loop through the trigger buffer from the beginning
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Get the current value of the trigger input
    int curr_value = trigger_buffer[i];

    // Check if the trigger input crossed the threshold from below
    if (prev_value < THRESHOLD && curr_value >= THRESHOLD) {
      // Calculate the phase error as the difference between the expected and the actual crossing point
      error = i - PERIOD;

      // Break the loop
      break;
    }

    // Update the previous value
    prev_value = curr_value;
  }

  // Return the phase error
  return error;
}

// The function to display the AC buffer on the OLED display
void display_buffer() {
  // Clear the display
  display.clearDisplay();

  // Draw the horizontal and vertical axes
  display.drawLine(0, OLED_HEIGHT / 2, OLED_WIDTH, OLED_HEIGHT / 2, WHITE);
  display.drawLine(OLED_WIDTH / 2, 0, OLED_WIDTH / 2, OLED_HEIGHT, WHITE);

  // Declare a variable to store the previous value of the AC input
  int prev_value = ac_buffer[0];

  // Loop through the AC buffer from the beginning
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Get the current value of the AC input
    int curr_value = ac_buffer[i];

    // Map the analog value to the display height
    int y1 = map(prev_value, 0, 4095, OLED_HEIGHT, 0);
    int y2 = map(curr_value, 0, 4095, OLED_HEIGHT, 0);

    // Draw a line between the previous and the current value
    display.drawLine(i - 1, y1, i, y2, WHITE);

    // Update the previous value
    prev_value = curr_value;
  }

  // Display the buffer on the screen
  display.display();
}
