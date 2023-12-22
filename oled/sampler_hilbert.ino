// Include libraries for SPI and OLED display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define pins for SPI and OLED display
#define OLED_MOSI  9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Create an object for the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// Define pins for input signal and LED
#define INPUT_PIN  34 // Analog input pin for the signal
#define LED_PIN    2  // Digital output pin for the LED

// Define constants for the phase oscillator
#define SAMPLE_RATE  1000 // Sampling rate in Hz
#define SAMPLE_TIME  (1000000 / SAMPLE_RATE) // Sampling time in microseconds
#define PHASE_STEP   0.01 // Phase increment per sample
#define PHASE_MAX    6.28 // Maximum phase value (2 * PI)
#define PHASE_OFFSET 1.57 // Phase offset for the Hilbert transform (PI / 2)

// Define variables for the phase oscillator
float phase = 0; // Current phase value
float phase_ref = 0; // Reference phase value
float phase_err = 0; // Phase error value
float phase_out = 0; // Output phase value
float phase_hil = 0; // Hilbert phase value

// Define constants for the sampling array
#define ARRAY_SIZE  128 // Size of the sampling array
#define ARRAY_MAX   4095 // Maximum value of the sampling array (2^12 - 1)

// Define variables for the sampling array
int array[ARRAY_SIZE]; // Sampling array
int array_index = 0; // Current index of the sampling array
int array_count = 0; // Number of samples in the sampling array
int array_sum = 0; // Sum of the samples in the sampling array
int array_avg = 0; // Average of the samples in the sampling array
int array_min = ARRAY_MAX; // Minimum of the samples in the sampling array
int array_max = 0; // Maximum of the samples in the sampling array

// Define a flag for the phase zero crossing
bool phase_zero = false;

// Define a function to read the input signal and return its phase
float read_phase() {
  // Read the analog input value
  int input = analogRead(INPUT_PIN);

  // Convert the input value to a voltage
  float voltage = input * (3.3 / 4095.0);

  // Convert the voltage to a phase
  float phase = voltage * (PHASE_MAX / 3.3);

  // Return the phase value
  return phase;
}

// Define a function to update the phase oscillator
void update_phase() {
  // Read the input phase value
  phase_ref = read_phase();

  // Calculate the phase error
  phase_err = phase_ref - phase;

  // Adjust the phase error to the range [-PHASE_MAX / 2, PHASE_MAX / 2]
  if (phase_err > PHASE_MAX / 2) {
    phase_err -= PHASE_MAX;
  }
  else if (phase_err < -PHASE_MAX / 2) {
    phase_err += PHASE_MAX;
  }

  // Update the current phase value
  phase += PHASE_STEP + phase_err;

  // Wrap the current phase value to the range [0, PHASE_MAX]
  if (phase > PHASE_MAX) {
    phase -= PHASE_MAX;
  }
  else if (phase < 0) {
    phase += PHASE_MAX;
  }

  // Calculate the output phase value
  phase_out = phase * (3.3 / PHASE_MAX);

  // Calculate the Hilbert phase value
  phase_hil = phase + PHASE_OFFSET;

  // Wrap the Hilbert phase value to the range [0, PHASE_MAX]
  if (phase_hil > PHASE_MAX) {
    phase_hil -= PHASE_MAX;
  }
  else if (phase_hil < 0) {
    phase_hil += PHASE_MAX;
  }
}

// Define a function to update the sampling array
void update_array() {
  // Check if the phase is close to zero
  if (abs(phase) < PHASE_STEP / 2) {
    // Set the phase zero flag to true
    phase_zero = true;
  }

  // Check if the phase zero flag is true
  if (phase_zero) {
    // Read the analog input value
    int input = analogRead(INPUT_PIN);

    // Store the input value in the sampling array
    array[array_index] = input;

    // Update the array index
    array_index++;

    // Wrap the array index to the range [0, ARRAY_SIZE - 1]
    if (array_index >= ARRAY_SIZE) {
      array_index = 0;
    }

    // Update the array count
    array_count++;

    // Limit the array count to the array size
    if (array_count > ARRAY_SIZE) {
      array_count = ARRAY_SIZE;
    }

    // Reset the phase zero flag to false
    phase_zero = false;
  }
}

// Define a function to print the sampling array in csv format
void print_array() {
  // Check if the array is full
  if (array_count == ARRAY_SIZE) {
    // Reset the array sum, min and max
    array_sum = 0;
    array_min = ARRAY_MAX;
    array_max = 0;

    // Loop through the array elements
    for (int i = 0; i < ARRAY_SIZE; i++) {
      // Get the current element
      int element = array[i];

      // Add the element to the array sum
      array_sum += element;

      // Update the array min and max
      if (element < array_min) {
        array_min = element;
      }
      if (element > array_max) {
        array_max = element;
      }

      // Print the element in csv format
      Serial.print(element);

      // Print a comma separator
      if (i < ARRAY_SIZE - 1) {
        Serial.print(",");
      }
    }

    // Print a new line
    Serial.println();

    // Calculate the array average
    array_avg = array_sum / ARRAY_SIZE;
  }
}

// Define a function to graph the sampling array on the OLED display
void graph_array() {
  // Check if the array is full
  if (array_count == ARRAY_SIZE) {
    // Clear the display buffer
    display.clearDisplay();

    // Set the text color to white
    display.setTextColor(WHITE);

    // Set the text size to 1
    display.setTextSize(1);

    // Display the array average, min and max
    display.setCursor(0, 0);
    display.print("Avg: ");
    display.print(array_avg);
    display.setCursor(64, 0);
    display.print("Min: ");
    display.print(array_min);
    display.setCursor(0, 8);
    display.print("Max: ");
    display.print(array_max);

    // Draw a horizontal line
    display.drawLine(0, 16, 127, 16, WHITE);

    // Loop through the array elements
    for (int i = 0; i < ARRAY_SIZE; i++) {
      // Get the current element
      int element = array[i];

      // Map the element to the display height
      int y = map(element, 0, 4095, 63, 17);

      // Draw a vertical line
      display.drawLine(i, 63, i, y, WHITE);
    }

    // Display the buffer on the screen
    display.display();
  }
}

// Define a function to blink the LED
void blink_led() {
  // Check if the phase is close to zero
  if (abs(phase) < PHASE_STEP / 2) {
    // Turn on the LED
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    // Turn off the LED
    digitalWrite(LED_PIN, LOW);
  }
}

// Define the setup function
void setup() {
  // Initialize the serial port
  Serial.begin(115200);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC);

  // Set the LED pin as output
  pinMode(LED_PIN, OUTPUT);
}

// Define the loop function
void loop() {
  // Get the current time in microseconds
  unsigned long time = micros();

  // Update the phase oscillator
  update_phase();

  // Update the sampling array
  update_array();

  // Print the sampling array in csv format
  print_array();

  // Graph the sampling array on the OLED display
  graph_array();

  // Blink the LED
  blink_led();

  // Wait until the next sampling time
  while (micros() - time < SAMPLE_TIME);
}
