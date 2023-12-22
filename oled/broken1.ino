// Define the pins for analog inputs and OLED display
#define ANALOG_IN_1 34 // first analog input pin for AC input
#define ANALOG_IN_2 35 // second analog input pin for trigger
#define OLED_DC 4 // OLED data/command pin
#define OLED_CS 5 // OLED chip select pin
#define OLED_RESET 16 // OLED reset pin

// Include the libraries for OLED display and SPI communication
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>

// Create an OLED display object
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

// Define some constants for the oscilloscope parameters
#define SAMPLE_RATE 1000 // sampling rate in microseconds
#define BUFFER_SIZE 40 // buffer size for two 50Hz periods minus one sample
#define V_REF 3.3 // reference voltage for analog inputs
#define ADC_RESOLUTION 4096 // resolution for analog inputs
#define TRIGGER_THRESHOLD 2048 // threshold for trigger detection
#define PHASE_STEP 10 // phase adjustment step in microseconds
#define PHASE_TOLERANCE 5 // phase tolerance in microseconds
#define MAX_ITERATIONS 100 // maximum number of iterations for phase lock
#define SCREEN_WIDTH 128 // OLED screen width in pixels
#define SCREEN_HEIGHT 64 // OLED screen height in pixels

// Declare some global variables for the oscilloscope logic
int buffer[BUFFER_SIZE]; // buffer to store the samples
int phase = 0; // phase offset in microseconds
int iteration = 0; // iteration counter for phase lock
bool phase_locked = false; // flag for phase lock status

// Declare some helper functions
void sample(); // function to sample the analog inputs
void analyze(); // function to analyze the buffer and adjust the phase
void display(); // function to display the waveform on the OLED
int find_zero_crossing(int start, int end); // function to find the zero crossing index in the buffer

void setup() {
  // Initialize the serial communication for debugging
  Serial.begin(115200);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 Oscilloscope");
  display.display();

  // Initialize the analog inputs
  analogReadResolution(12); // set the analog resolution to 12 bits
  analogSetAttenuation(ADC_11db); // set the analog attenuation to 11 dB
}

void loop() {
  // Check if the phase is locked
  if (phase_locked) {
    // Sample the first analog input and display the waveform
    sample();
    display();
  } else {
    // Sample the second analog input and adjust the phase
    sample();
    analyze();
  }
}

void sample() {
  // Loop through the buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Read the analog input depending on the phase lock status
    if (phase_locked) {
      // Read the first analog input
      buffer[i] = analogRead(ANALOG_IN_1);
    } else {
      // Read the second analog input
      buffer[i] = analogRead(ANALOG_IN_2);
    }
    // Wait for the sampling rate
    delayMicroseconds(SAMPLE_RATE);
  }
}

void analyze() {
  // Increment the iteration counter
  iteration++;

  // Find the first zero crossing in the buffer
  int z1 = find_zero_crossing(0, BUFFER_SIZE - 1);

  // Check if the zero crossing was found
  if (z1 != -1) {
    // Find the second zero crossing in the buffer
    int z2 = find_zero_crossing(z1 + 1, BUFFER_SIZE - 1);

    // Check if the second zero crossing was found
    if (z2 != -1) {
      // Calculate the phase error
      int error = (z1 + z2) / 2;

      // Check if the phase error is within the tolerance
      if (abs(error) <= PHASE_TOLERANCE) {
        // Set the phase lock flag to true
        phase_locked = true;

        // Print a message to the serial monitor
        Serial.println("Phase locked!");
      } else {
        // Adjust the phase using gradient descent
        phase = phase - error * PHASE_STEP;

        // Check if the phase is within the valid range
        if (phase < 0) {
          phase = 0;
        } else if (phase > SAMPLE_RATE) {
          phase = SAMPLE_RATE;
        }

        // Wait for the phase adjustment
        delayMicroseconds(phase);

        // Print a message to the serial monitor
        Serial.print("Phase error: ");
        Serial.println(error);
      }
    } else {
      // Print a message to the serial monitor
      Serial.println("Second zero crossing not found");
    }
  } else {
    // Print a message to the serial monitor
    Serial.println("First zero crossing not found");
  }

  // Check if the maximum number of iterations is reached
  if (iteration >= MAX_ITERATIONS) {
    // Set the phase lock flag to true
    phase_locked = true;

    // Print a message to the serial monitor
    Serial.println("Maximum iterations reached");
  }
}

void display() {
  // Clear the OLED display
  display.clearDisplay();

  // Draw the waveform on the OLED display
  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    // Map the buffer values to the screen coordinates
    int x1 = map(i, 0, BUFFER_SIZE - 1, 0, SCREEN_WIDTH - 1);
    int y1 = map(buffer[i], 0, ADC_RESOLUTION - 1, SCREEN_HEIGHT - 1, 0);
    int x2 = map(i + 1, 0, BUFFER_SIZE - 1, 0, SCREEN_WIDTH - 1);
    int y2 = map(buffer[i + 1], 0, ADC_RESOLUTION - 1, SCREEN_HEIGHT - 1, 0);

    // Draw a line between the two points
    display.drawLine(x1, y1, x2, y2, WHITE);
  }

  // Display the OLED buffer
  display.display();
}

int find_zero_crossing(int start, int end) {
  // Loop through the buffer from start to end
  for (int i = start; i < end; i++) {
    // Check if the buffer value crosses the trigger threshold
    if ((buffer[i] < TRIGGER_THRESHOLD && buffer[i + 1] > TRIGGER_THRESHOLD) ||
        (buffer[i] > TRIGGER_THRESHOLD && buffer[i + 1] < TRIGGER_THRESHOLD)) {
      // Return the index of the zero crossing
      return i;
    }
  }
  // Return -1 if no zero crossing was found
  return -1;
}
