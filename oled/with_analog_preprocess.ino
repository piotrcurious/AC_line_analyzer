// Define the analog input pin
#define ANALOG_PIN 36

// Define the SPI OLED display pins
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16

// Include the libraries for OLED display and graphics
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Create an OLED display object
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);

// Define the buffer size and the sampling frequency
#define BUFFER_SIZE 1000
#define SAMPLING_FREQ 1000

// Define the target frequency and the tolerance
#define TARGET_FREQ 50
#define TOLERANCE 0.01

// Define the learning rate for gradient descent
#define LEARNING_RATE 0.01

// Declare the buffer array and the index variable
int buffer[BUFFER_SIZE];
int index = 0;

// Declare the variables for phase lock and offset
float phase_lock = 0;
float phase_offset = 0;

// Declare the variables for key features detection
bool zero_crossing = false;
bool rising_edge = false;
bool falling_edge = false;
bool middle_zero_crossing = false;
bool falling_edge_before = false;
bool rising_edge_before = false;

// Declare the variables for integration and overlap
float positive_sum = 0;
float negative_sum = 0;
float overlap = 0;

// Declare the variables for display
int x = 0;
int y = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 Oscilloscope");
  display.display();

  // Start the sampling timer
  timer1_attachInterrupt(samplingISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(80000000 / 16 / SAMPLING_FREQ);
}

void loop() {
  // Check if the buffer is full
  if (index == BUFFER_SIZE) {
    // Stop the sampling timer
    timer1_disable();

    // Analyze the buffer data
    analyzeBuffer();

    // Adjust the phase lock using gradient descent
    adjustPhaseLock();

    // Display the waveform on the OLED screen
    displayWaveform();

    // Reset the buffer index
    index = 0;

    // Restart the sampling timer
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  }
}

// The sampling interrupt service routine
void IRAM_ATTR samplingISR() {
  // Read the analog value from the input pin
  int value = analogRead(ANALOG_PIN);

  // Store the value in the buffer
  buffer[index] = value;

  // Increment the buffer index
  index++;
}

// The function to analyze the buffer data
void analyzeBuffer() {
  // Reset the key features detection variables
  zero_crossing = false;
  rising_edge = false;
  falling_edge = false;
  middle_zero_crossing = false;
  falling_edge_before = false;
  rising_edge_before = false;

  // Reset the integration and overlap variables
  positive_sum = 0;
  negative_sum = 0;
  overlap = 0;

  // Loop through the buffer data
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Get the current and the previous value
    int current = buffer[i];
    int previous = buffer[i - 1];

    // Check for zero crossing
    if (current == 2048 && previous != 2048) {
      zero_crossing = true;
    }

    // Check for rising edge
    if (current > 2048 && previous < 2048) {
      rising_edge = true;
    }

    // Check for falling edge
    if (current < 2048 && previous > 2048) {
      falling_edge = true;
    }

    // Check for middle zero crossing
    if (current == 0 && previous != 0) {
      middle_zero_crossing = true;
    }

    // Check for falling edge before zero crossing
    if (falling_edge && !zero_crossing) {
      falling_edge_before = true;
    }

    // Check for rising edge before zero crossing
    if (rising_edge && !zero_crossing) {
      rising_edge_before = true;
    }

    // Calculate the positive and negative sum
    if (current > 0) {
      positive_sum += current;
    }
    if (current < 0) {
      negative_sum += current;
    }
  }

  // Calculate the overlap
  overlap = min(abs(positive_sum), abs(negative_sum));
}

// The function to adjust the phase lock using gradient descent
void adjustPhaseLock() {
  // Calculate the error between the target frequency and the actual frequency
  float error = TARGET_FREQ - SAMPLING_FREQ / phase_lock;

  // Check if the error is within the tolerance
  if (abs(error) < TOLERANCE) {
    // No need to adjust the phase lock
    return;
  }

  // Calculate the gradient of the error with respect to the phase lock
  float gradient = -SAMPLING_FREQ / (phase_lock * phase_lock);

  // Update the phase lock using the gradient and the learning rate
  phase_lock = phase_lock - LEARNING_RATE * gradient * error;

  // Calculate the phase offset using the key features detection
  if (zero_crossing) {
    phase_offset = 0;
  }
  else if (rising_edge) {
    phase_offset = 0.25;
  }
  else if (falling_edge) {
    phase_offset = 0.75;
  }
  else if (middle_zero_crossing) {
    phase_offset = 0.5;
  }
  else if (falling_edge_before) {
    phase_offset = 0.875;
  }
  else if (rising_edge_before) {
    phase_offset = 0.125;
  }

  // Adjust the total sampling time using the phase lock and the phase offset
  timer1_write(80000000 / 16 / (SAMPLING_FREQ * phase_lock + phase_offset));
}

// The function to display the waveform on the OLED screen
void displayWaveform() {
  // Clear the display
  display.clearDisplay();

  // Loop through the buffer data
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Get the current value
    int value = buffer[i];

    // Map the value to the display coordinates
    x = map(i, 0, BUFFER_SIZE, 0, 127);
    y = map(value, 0, 4095, 63, 0);

    // Draw a pixel on the display
    display.drawPixel(x, y, WHITE);
  }

  // Update the display
  display.display();
}
