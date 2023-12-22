// This code is based on the previous code and the description from the user
// It uses the micros() function to read the input at a defined rate
// It assumes the input signal is between 1.1 KHz and 1.3 KHz
// It uses a software phase-locked loop (SPLL) to track a PSK modulated signal
// It uses a sampling array to store the input samples and determine the phase error
// It uses a second array to store the samples when the phase error variable is zero
// It prints the content of the second array in csv format to the serial port
// It graphs the content of the second array on the SPI 128x64 oled display
// It clears the trigger and repeats the whole process
// It uses an ESP32 board with an ADC pin connected to the input signal
// It uses a DAC pin to output a square wave with the same frequency and phase as the input signal
// It uses an SPI 128x64 oled display module connected to the SPI pins
// It moves the phase detection to a separate function
// It uses a different, more robust algorithm for phase detection
// It uses the Goertzel algorithm [^1^][1] to estimate the phase of the input signal
// It compares the estimated phase with the expected phase of the square wave
// It adjusts the oscillator variable accordingly to minimize the phase error

#define ADC_PIN 34 // The pin to read the input signal
#define DAC_PIN 25 // The pin to output the square wave
#define SAMPLE_RATE 8000 // The sampling rate in Hz
#define SAMPLE_SIZE 32 // The size of the sampling array
#define NATURAL_FREQ 250 // The natural frequency of the SPLL in Hz
#define BUFFER_SIZE 128 // The size of the second array
#define SAMPLE_INTERVAL 125 // The interval between samples in microseconds
#define SIGNAL_FREQ 1200 // The frequency of the input signal in Hz
#define SIGNAL_PHASE 0 // The phase of the input signal in radians

// The pins for the SPI oled display module
#define OLED_MOSI 23 // The Master Out Slave In pin
#define OLED_CLK 18 // The clock pin
#define OLED_DC 21 // The Data/Command pin
#define OLED_CS 5 // The Chip Select pin
#define OLED_RESET 19 // The Reset pin

int b = 0; // The oscillator variable
int samples[SAMPLE_SIZE]; // The sampling array
int buffer[BUFFER_SIZE]; // The second array
int index = 0; // The index of the current sample
int buffer_index = 0; // The index of the current buffer element
bool triggered = false; // The flag to indicate the trigger condition
unsigned long last_sample_time = 0; // The time of the last sample

// Include the libraries for the oled display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Create an Adafruit_SSD1306 object for the oled display
Adafruit_SSD1306 display(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS, 8000000);

void setup() {
  // Initialize the serial port with 9600 baud rate
  Serial.begin(9600);
  // Set the ADC pin as input
  pinMode(ADC_PIN, INPUT);
  // Set the DAC pin as output
  pinMode(DAC_PIN, OUTPUT);
  // Set the ADC resolution to 8 bits
  analogReadResolution(8);
  // Set the DAC resolution to 8 bits
  analogWriteResolution(8);
  // Initialize the sampling array with zeros
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    samples[i] = 0;
  }
  // Initialize the second array with zeros
  for (int i = 0; i < BUFFER_SIZE; i++) {
    buffer[i] = 0;
  }
  // Initialize the oled display with internal voltage
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  // Clear the oled display
  display.clearDisplay();
}

void loop() {
  // Get the current time in microseconds
  unsigned long current_time = micros();
  // Check if the sample interval has elapsed
  if (current_time - last_sample_time >= SAMPLE_INTERVAL) {
    // Update the last sample time
    last_sample_time = current_time;
    // Read the input signal and store it in the sampling array
    samples[index] = analogRead(ADC_PIN);
    // Increment the index and wrap around if necessary
    index = (index + 1) % SAMPLE_SIZE;
    // Call the phase detection function to update the oscillator variable
    phase_detection();
    // Write the square wave to the DAC pin
    analogWrite(DAC_PIN, b);
    // Check if the phase error variable is zero and the trigger is not set
    if (a == 0 && !triggered) {
      // Set the trigger flag to true
      triggered = true;
      // Reset the buffer index to zero
      buffer_index = 0;
    }
    // Check if the trigger flag is true
    if (triggered) {
      // Store the input signal in the second array
      buffer[buffer_index] = samples[index];
      // Increment the buffer index and wrap around if necessary
      buffer_index = (buffer_index + 1) % BUFFER_SIZE;
      // Check if the buffer index is zero, meaning the second array is full
      if (buffer_index == 0) {
        // Print the content of the second array in csv format to the serial port
        for (int i = 0; i < BUFFER_SIZE; i++) {
          Serial.print(buffer[i]);
          // Add a comma after each element except the last one
          if (i < BUFFER_SIZE - 1) {
            Serial.print(",");
          }
        }
        // Add a newline after the csv line
        Serial.println();
        // Graph the content of the second array on the oled display
        // Clear the oled display
        display.clearDisplay();
        // Set the text color to white
        display.setTextColor(SSD1306_WHITE);
        // Set the text size to 1
        display.setTextSize(1);
        // Set the cursor position to the top left corner
        display.setCursor(0, 0);
        // Print a title for the graph
        display.println("Input signal graph");
        // Draw a horizontal line below the title
        display.drawFastHLine(0, 8, 128, SSD1306_WHITE);
        // Draw a vertical line on the left side of the graph
        display.drawFastVLine(0, 8, 56, SSD1306_WHITE);
        // Draw a vertical line on the right side of the graph
        display.drawFastVLine(127, 8, 56, SSD1306_WHITE);
        // Loop through the second array elements
        for (int i = 0; i < BUFFER_SIZE; i++) {
          // Map the element value to a pixel position on the oled display
          // The element value ranges from 0 to 255
          // The pixel position ranges from 8 to 63
          int pixel = map(buffer[i], 0, 255, 63, 8);
          // Draw a pixel on the oled display at the mapped position
          display.drawPixel(i, pixel, SSD1306_WHITE);
        }
        // Show the oled display buffer on the screen
        display.display();
        // Clear the trigger flag to false
        triggered = false;
      }
    }
  }
}

// The phase detection function
void phase_detection() {
  // Declare some variables for the Goertzel algorithm
  float k; // The frequency index
  float w; // The normalized frequency
  float cosine; // The cosine of the normalized frequency
  float sine; // The sine of the normalized frequency
  float coeff; // The coefficient for the algorithm
  float q0; // The current state variable
  float q1; // The previous state variable
  float q2; // The previous previous state variable
  float real; // The real part of the result
  float imag; // The imaginary part of the result
  float magnitude; // The magnitude of the result
  float phase; // The phase of the result
  float phase_error; // The phase error between the input signal and the square wave

  // Calculate the frequency index
  k = round(SIGNAL_FREQ * SAMPLE_SIZE / SAMPLE_RATE);
  // Calculate the normalized frequency
  w = 2 * PI * k / SAMPLE_SIZE;
  // Calculate the cosine and sine of the normalized frequency
  cosine = cos(w);
  sine = sin(w);
  // Calculate the coefficient for the algorithm
  coeff = 2 * cosine;
  // Initialize the state variables
  q0 = 0;
  q1 = 0;
  q2 = 0;
  // Loop through the sampling array
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    // Update the state variables
    q0 = coeff * q1 - q2 + samples[i];
    q2 = q1;
    q1 = q0;
  }
  // Calculate the real and imaginary parts of the result
  real = q1 - q2 * cosine;
  imag = q2 * sine;
  // Calculate the magnitude of the result
  magnitude = sqrt(real * real + imag * imag);
  // Calculate the phase of the result
  phase = atan2(imag, real);
  // Compare the estimated phase with the expected phase of the square wave
  // The expected phase is the same as the phase of the input signal
  phase_error = phase - SIGNAL_PHASE;
  // Adjust the oscillator variable accordingly to minimize the phase error
  // The adjustment factor is proportional to the magnitude of the result
  b += 16 + int(phase_error * magnitude / 1024);
  // Update the phase error variable by multiplying the input signal with the square wave
  a += ((b & 256) ? 1 : -1) * samples[index] - a / 512;
}
