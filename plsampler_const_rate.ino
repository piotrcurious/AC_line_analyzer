// This code is based on the previous code and the description from the user
// It uses the micros() function to read the input at a defined rate
// It assumes the input signal is between 1.1 KHz and 1.3 KHz
// It uses a software phase-locked loop (SPLL) to track a PSK modulated signal
// It uses a sampling array to store the input samples and determine the phase error
// It uses a second array to store the samples when the phase error variable is zero
// It prints the content of the second array in csv format to the serial port
// It clears the trigger and repeats the whole process
// It uses an ESP32 board with an ADC pin connected to the input signal
// It uses a DAC pin to output a square wave with the same frequency and phase as the input signal

#define ADC_PIN 34 // The pin to read the input signal
#define DAC_PIN 25 // The pin to output the square wave
#define SAMPLE_RATE 8000 // The sampling rate in Hz
#define SAMPLE_SIZE 32 // The size of the sampling array
#define NATURAL_FREQ 250 // The natural frequency of the SPLL in Hz
#define BUFFER_SIZE 64 // The size of the second array
#define SAMPLE_INTERVAL 125 // The interval between samples in microseconds

int a = 0; // The phase error variable
int b = 0; // The oscillator variable
int samples[SAMPLE_SIZE]; // The sampling array
int buffer[BUFFER_SIZE]; // The second array
int index = 0; // The index of the current sample
int buffer_index = 0; // The index of the current buffer element
bool triggered = false; // The flag to indicate the trigger condition
unsigned long last_sample_time = 0; // The time of the last sample

void setup() {
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
  // Initialize the serial port with 9600 baud rate
  Serial.begin(9600);
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
    // Update the phase error variable by multiplying the input signal with the square wave
    a += ((b += 16 + a / 1024) & 256 ? 1 : -1) * samples[index] - a / 512;
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
        // Clear the trigger flag to false
        triggered = false;
      }
    }
  }
}
