// This code is based on the previous code and the description from the user
// It uses a second array to store the samples when the phase error variable is zero
// It prints the content of the second array in csv format to the serial port
// It clears the trigger and repeats the whole process

#define ADC_PIN 34 // The pin to read the input signal
#define DAC_PIN 25 // The pin to output the square wave
#define SAMPLE_RATE 8000 // The sampling rate in Hz
#define SAMPLE_SIZE 32 // The size of the sampling array
#define NATURAL_FREQ 250 // The natural frequency of the SPLL in Hz
#define BUFFER_SIZE 64 // The size of the second array

int a = 0; // The phase error variable
int b = 0; // The oscillator variable
int samples[SAMPLE_SIZE]; // The sampling array
int buffer[BUFFER_SIZE]; // The second array
int index = 0; // The index of the current sample
int buffer_index = 0; // The index of the current buffer element
bool triggered = false; // The flag to indicate the trigger condition

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
