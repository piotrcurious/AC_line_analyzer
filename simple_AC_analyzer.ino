// ESP32 Arduino code for AC line analyzer
// This code uses the built-in ADC to sample the AC line voltage
// It also uses a zero-crossing detector to synchronize the sampling
// It calculates the line parameters such as harmonic distortion, RMS, peak voltage, integrated voltage and difference between RMS and integrated voltage
// Disclaimer: This code is for educational purposes only and not intended to be used in a real device. Use it at your own risk.

// Define the pins
#define ZCD_PIN 2 // Zero-crossing detector pin
#define AC_PIN 34 // AC line voltage pin
#define SAMPLES 256 // Number of samples per cycle

// Define the constants
#define VREF 3.3 // Reference voltage for ADC
#define ADC_BITS 12 // Resolution of ADC
#define ADC_MAX 4095 // Maximum value of ADC
#define LINE_FREQ 50 // Line frequency in Hz
#define SAMPLE_FREQ 3200 // Sampling frequency in Hz
#define SAMPLE_TIME 312 // Sampling time in microseconds

// Declare the variables
volatile int sampleIndex = 0; // Current sample index
volatile bool sampling = false; // Sampling flag
int sampleBuffer[SAMPLES]; // Sample buffer
float vRMS = 0; // RMS voltage
float vPeak = 0; // Peak voltage
float vInt = 0; // Integrated voltage
float vDiff = 0; // Difference between RMS and integrated voltage
float THD = 0; // Total harmonic distortion

// Interrupt service routine for zero-crossing detector
void IRAM_ATTR zcdISR() {
  // Start sampling
  sampling = true;
  // Reset sample index
  sampleIndex = 0;
}

// Interrupt service routine for ADC conversion
void IRAM_ATTR adcISR() {
  // Read the ADC value
  int adcValue = analogRead(AC_PIN);
  // Store the value in the buffer
  sampleBuffer[sampleIndex] = adcValue;
  // Increment the sample index
  sampleIndex++;
  // Check if the buffer is full
  if (sampleIndex == SAMPLES) {
    // Stop sampling
    sampling = false;
  }
}

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  // Initialize the ADC
  analogReadResolution(ADC_BITS);
  analogSetAttenuation(ADC_11db); // Set the attenuation for 0-3.3V range
  // Initialize the zero-crossing detector pin
  pinMode(ZCD_PIN, INPUT_PULLUP);
  // Attach the interrupt for zero-crossing detector
  attachInterrupt(digitalPinToInterrupt(ZCD_PIN), zcdISR, RISING);
  // Start the timer for ADC conversion
  timer = timerBegin(0, 80, true); // Use timer 0, 80 prescaler, count up
  timerAttachInterrupt(timer, &adcISR, true); // Attach the interrupt
  timerAlarmWrite(timer, SAMPLE_TIME, true); // Set the alarm value and repeat
  timerAlarmEnable(timer); // Enable the alarm
}

// Loop function
void loop() {
  // Check if sampling is done
  if (!sampling) {
    // Detach the interrupt for zero-crossing detector
    detachInterrupt(digitalPinToInterrupt(ZCD_PIN));
    // Disable the timer alarm
    timerAlarmDisable(timer);
    // Calculate the line parameters
    calculateParameters();
    // Print the results
    printResults();
    // Attach the interrupt for zero-crossing detector
    attachInterrupt(digitalPinToInterrupt(ZCD_PIN), zcdISR, RISING);
    // Enable the timer alarm
    timerAlarmEnable(timer);
  }
}

// Function to calculate the line parameters
void calculateParameters() {
  // Declare the variables
  float sum = 0; // Sum of squares
  float max = 0; // Maximum value
  float min = ADC_MAX; // Minimum value
  float integral = 0; // Integral of voltage
  float mean = 0; // Mean of voltage
  float variance = 0; // Variance of voltage
  float stdDev = 0; // Standard deviation of voltage
  float harmonics[SAMPLES / 2]; // Harmonic spectrum
  float fundamental = 0; // Fundamental frequency
  float distortion = 0; // Distortion power
  // Loop through the samples
  for (int i = 0; i < SAMPLES; i++) {
    // Convert the ADC value to voltage
    float voltage = map(sampleBuffer[i], 0, ADC_MAX, 0, VREF);
    // Update the maximum and minimum values
    if (voltage > max) {
      max = voltage;
    }
    if (voltage < min) {
      min = voltage;
    }
    // Calculate the sum of squares
    sum += voltage * voltage;
    // Calculate the integral of voltage
    integral += voltage;
    // Calculate the mean of voltage
    mean += voltage / SAMPLES;
  }
  // Calculate the RMS voltage
  vRMS = sqrt(sum / SAMPLES);
  // Calculate the peak voltage
  vPeak = (max - min) / 2;
  // Calculate the integrated voltage
  vInt = integral / SAMPLES;
  // Calculate the difference between RMS and integrated voltage
  vDiff = vRMS - vInt;
  // Calculate the variance and standard deviation of voltage
  for (int i = 0; i < SAMPLES; i++) {
    // Convert the ADC value to voltage
    float voltage = map(sampleBuffer[i], 0, ADC_MAX, 0, VREF);
    // Calculate the variance of voltage
    variance += pow(voltage - mean, 2) / SAMPLES;
  }
  // Calculate the standard deviation of voltage
  stdDev = sqrt(variance);
  // Calculate the harmonic spectrum using FFT
  FFT.Windowing(sampleBuffer, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(sampleBuffer, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(sampleBuffer, SAMPLES);
  // Normalize the spectrum
  for (int i = 0; i < SAMPLES / 2; i++) {
    // Divide by the number of samples
    harmonics[i] = sampleBuffer[i] / SAMPLES;
    // Multiply by 2 to account for the symmetry
    harmonics[i] = harmonics[i] * 2;
    // Compensate for the windowing loss
    harmonics[i] = harmonics[i] / 0.54;
  }
  // Get the fundamental frequency
  fundamental = harmonics[1];
  // Calculate the distortion power
  for (int i = 2; i < SAMPLES / 2; i++) {
    // Sum the squares of the harmonics
    distortion += harmonics[i] * harmonics[i];
  }
  // Calculate the total harmonic distortion
  THD = sqrt(distortion) / fundamental;
}

// Function to print the results
void printResults() {
  // Print the line parameters
  Serial.print("RMS voltage: ");
  Serial.print(vRMS, 3);
  Serial.println(" V");
  Serial.print("Peak voltage: ");
  Serial.print(vPeak, 3);
  Serial.println(" V");
  Serial.print("Integrated voltage: ");
  Serial.print(vInt, 3);
  Serial.println(" V");
  Serial.print("Difference between RMS and integrated voltage: ");
  Serial.print(vDiff, 3);
  Serial.println(" V");
  Serial.print("Total harmonic distortion: ");
  Serial.print(THD, 3);
  Serial.println(" %");
}
