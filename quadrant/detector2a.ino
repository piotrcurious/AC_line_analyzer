// Include the LEDC library to generate PWM signals
#include <esp32-hal-ledc.h>

// Define the analog pin to read the AC signal
#define AC_PIN 34

// Define the PWM channel, frequency and resolution
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RES 8

// Define the rational function parameters
// f(x) = (a*x + b) / (c*x + d)
#define A 1
#define B 2
#define C 3
#define D 4

// Define the ADC resolution and reference voltage
#define ADC_RES 4096
#define ADC_REF 3.3

// Define the AC signal frequency range
#define AC_MIN_FREQ 50
#define AC_MAX_FREQ 60

// Define the AC signal amplitude range
#define AC_MIN_AMP 0
#define AC_MAX_AMP 5

// Define the AC signal offset
#define AC_OFFSET 2.5

// Define the sampling period in microseconds
#define SAMPLING_PERIOD 100

// Define the number of samples to take per cycle
#define SAMPLES_PER_CYCLE 20

// Define the number of samples to search for zero crossing
#define ZERO_CROSSING_WINDOW 5

// Define a buffer to store the samples
int sample_buffer[ZERO_CROSSING_WINDOW];

// Define a variable to store the index of the buffer
int buffer_index = 0;

// Define a variable to store the current quadrant
int quadrant = 0;

// Define a variable to store the current phase in degrees
float phase = 0;

// Define a variable to store the PWM duty cycle
int duty_cycle = 0;

// Define a variable to store the elapsed time
unsigned long elapsed_time = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the analog pin as input
  pinMode(AC_PIN, INPUT);

  // Initialize the PWM channel
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);

  // Attach the PWM channel to pin 2
  ledcAttachPin(2, PWM_CHANNEL);
}

void loop() {
  // Read the current sample from the analog pin
  int curr_sample = analogRead(AC_PIN);

  // Store the sample in the buffer
  sample_buffer[buffer_index] = curr_sample;

  // Update the buffer index
  buffer_index = (buffer_index + 1) % ZERO_CROSSING_WINDOW;

  // Check if the buffer is full
  if (buffer_index == 0) {

    // Find the minimum and maximum samples in the buffer
    int min_sample = ADC_RES;
    int max_sample = 0;
    for (int i = 0; i < ZERO_CROSSING_WINDOW; i++) {
      if (sample_buffer[i] < min_sample) {
        min_sample = sample_buffer[i];
      }
      if (sample_buffer[i] > max_sample) {
        max_sample = sample_buffer[i];
      }
    }

    // Check if the buffer contains a zero crossing
    if ((min_sample < AC_OFFSET * ADC_RES / ADC_REF && max_sample > AC_OFFSET * ADC_RES / ADC_REF)) {

      // Find the two samples that bracket the zero crossing
      int prev_sample = 0;
      int next_sample = 0;
      for (int i = 0; i < ZERO_CROSSING_WINDOW; i++) {
        int j = (buffer_index + i) % ZERO_CROSSING_WINDOW;
        if (sample_buffer[j] < AC_OFFSET * ADC_RES / ADC_REF && sample_buffer[(j + 1) % ZERO_CROSSING_WINDOW] > AC_OFFSET * ADC_RES / ADC_REF) {
          prev_sample = sample_buffer[j];
          next_sample = sample_buffer[(j + 1) % ZERO_CROSSING_WINDOW];
          break;
        }
        if (sample_buffer[j] > AC_OFFSET * ADC_RES / ADC_REF && sample_buffer[(j + 1) % ZERO_CROSSING_WINDOW] < AC_OFFSET * ADC_RES / ADC_REF) {
          prev_sample = sample_buffer[j];
          next_sample = sample_buffer[(j + 1) % ZERO_CROSSING_WINDOW];
          break;
        }
      }

      // Convert the samples to voltage
      float prev_voltage = prev_sample * ADC_REF / ADC_RES;
      float next_voltage = next_sample * ADC_REF / ADC_RES;

      // Calculate the AC signal frequency
      float frequency = 1000000.0 / elapsed_time;

      // Check if the frequency is within the range
      if (frequency >= AC_MIN_FREQ && frequency <= AC_MAX_FREQ) {

        // Calculate the AC signal amplitude
        float amplitude = abs(next_voltage - AC_OFFSET);

        // Check if the amplitude is within the range
        if (amplitude >= AC_MIN_AMP && amplitude <= AC_MAX_AMP) {

          // Calculate the current quadrant
          if (next_voltage > AC_OFFSET && prev_voltage > next_voltage) {
            quadrant = 1;
          } else if (next_voltage > AC_OFFSET && prev_voltage < next_voltage) {
            quadrant = 2;
          } else if (next_voltage < AC_OFFSET && prev_voltage < next_voltage) {
            quadrant = 3;
          } else if (next_voltage < AC_OFFSET && prev_voltage > next_voltage) {
            quadrant = 4;
          }

          // Calculate the current phase in degrees using linear interpolation
          phase = quadrant * 90 - map(AC_OFFSET * ADC_RES / ADC_REF, prev_sample, next_sample, 0, 90);

          // Calculate the PWM duty cycle using the rational function
          duty_cycle = constrain((A * phase + B) / (C * phase + D), 0, 1) * 255;

          // Write the PWM duty cycle to the channel
          ledcWrite(PWM_CHANNEL, duty_cycle);

          // Print the frequency, amplitude, quadrant, phase and duty cycle to the serial monitor
          Serial.print("Frequency: ");
          Serial.print(frequency);
          Serial.print(" Hz, Amplitude: ");
          Serial.print(amplitude);
          Serial.print(" V, Quadrant: ");
          Serial.print(quadrant);
          Serial.print(", Phase: ");
          Serial.print(phase);
          Serial.print(" degrees, Duty Cycle: ");
          Serial.println(duty_cycle);
        }
      }

      // Reset the elapsed time
      elapsed_time = 0;
    }
  }

  // Update the elapsed time
  elapsed_time += SAMPLING_PERIOD;

  // Wait for the sampling period
  delayMicroseconds(SAMPLING_PERIOD);
}
