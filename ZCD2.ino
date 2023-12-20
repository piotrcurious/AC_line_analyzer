// ESP32 Arduino code for AC line zero crossing detector
// The device uses PLL method to synchronize internal frequency estimator with observed zero crossing
// and Kalman filter using noise measured around zero crossing to determine measurement noise
// The device uses analog input as AC input
// The device keeps simulated zero crossing output by synchronizing internal timer with measured zero crossing

// Define the pins
#define AC_IN_PIN 34 // Analog input pin for AC signal
#define ZC_OUT_PIN 25 // Digital output pin for simulated zero crossing
#define LED_PIN 2 // Digital output pin for LED indicator

// Define the constants
#define AC_MAX 4095 // Maximum value of analog input
#define AC_MIN 0 // Minimum value of analog input
#define AC_THRESHOLD 2048 // Threshold value of analog input for zero crossing detection
#define AC_FREQ 50 // Nominal frequency of AC signal in Hz
#define AC_PERIOD 20000 // Nominal period of AC signal in microseconds
#define PLL_GAIN 0.01 // Gain of PLL loop filter
#define KALMAN_Q 0.001 // Process noise covariance of Kalman filter
#define KALMAN_R 0.1 // Measurement noise covariance of Kalman filter

// Declare the global variables
volatile uint32_t zc_time; // Time of last zero crossing in microseconds
volatile uint32_t zc_period; // Period of last zero crossing in microseconds
volatile float zc_freq; // Frequency of last zero crossing in Hz
volatile float zc_phase; // Phase of last zero crossing in radians
volatile float zc_error; // Error of last zero crossing in radians
volatile float zc_output; // Output of PLL loop filter
volatile float kalman_x; // State estimate of Kalman filter
volatile float kalman_p; // Error estimate of Kalman filter
volatile float kalman_k; // Kalman gain
hw_timer_t * timer = NULL; // Hardware timer object
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Mutex for critical section

// Interrupt service routine for analog input
void IRAM_ATTR ac_isr() {
  static uint32_t last_time = 0; // Time of previous analog input interrupt
  static uint16_t last_value = 0; // Value of previous analog input
  static bool rising = false; // Flag for rising edge detection
  uint32_t current_time = micros(); // Time of current analog input interrupt
  uint16_t current_value = analogRead(AC_IN_PIN); // Value of current analog input
  // Check if rising edge is detected
  if (current_value > AC_THRESHOLD && last_value <= AC_THRESHOLD) {
    rising = true;
  }
  // Check if falling edge is detected
  if (current_value < AC_THRESHOLD && last_value >= AC_THRESHOLD) {
    // Check if rising edge was detected before
    if (rising) {
      // Enter critical section
      portENTER_CRITICAL_ISR(&timerMux);
      // Calculate the time and period of zero crossing
      zc_time = current_time - (current_value - AC_THRESHOLD) * (current_time - last_time) / (current_value - last_value);
      zc_period = zc_time - last_time;
      // Calculate the frequency and phase of zero crossing
      zc_freq = 1000000.0 / zc_period;
      zc_phase = 2 * PI * zc_time / 1000000.0;
      // Calculate the error of zero crossing
      zc_error = zc_phase - zc_output;
      // Normalize the error to [-PI, PI]
      while (zc_error > PI) {
        zc_error -= 2 * PI;
      }
      while (zc_error < -PI) {
        zc_error += 2 * PI;
      }
      // Update the output of PLL loop filter
      zc_output += PLL_GAIN * zc_error;
      // Normalize the output to [0, 2*PI]
      while (zc_output > 2 * PI) {
        zc_output -= 2 * PI;
      }
      while (zc_output < 0) {
        zc_output += 2 * PI;
      }
      // Update the state and error estimate of Kalman filter
      kalman_x = zc_output;
      kalman_p = kalman_p + KALMAN_Q;
      // Calculate the Kalman gain
      kalman_k = kalman_p / (kalman_p + KALMAN_R);
      // Update the state and error estimate with measurement
      kalman_x = kalman_x + kalman_k * (zc_phase - kalman_x);
      kalman_p = (1 - kalman_k) * kalman_p;
      // Exit critical section
      portEXIT_CRITICAL_ISR(&timerMux);
      // Reset the rising edge flag
      rising = false;
      // Toggle the LED indicator
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
  // Update the previous time and value
  last_time = current_time;
  last_value = current_value;
}

// Interrupt service routine for hardware timer
void IRAM_ATTR timer_isr() {
  // Enter critical section
  portENTER_CRITICAL_ISR(&timerMux);
  // Get the current time
  uint32_t current_time = micros();
  // Calculate the simulated zero crossing phase
  float sim_phase = 2 * PI * current_time / 1000000.0;
  // Normalize the phase to [0, 2*PI]
  while (sim_phase > 2 * PI) {
    sim_phase -= 2 * PI;
  }
  while (sim_phase < 0) {
    sim_phase += 2 * PI;
  }
  // Compare the simulated phase with the estimated phase
  if (sim_phase > kalman_x) {
    // Set the output pin to high
    digitalWrite(ZC_OUT_PIN, HIGH);
  }
  else {
    // Set the output pin to low
    digitalWrite(ZC_OUT_PIN, LOW);
  }
  // Exit critical section
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Setup function
void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);
  // Initialize the analog input pin
  pinMode(AC_IN_PIN, INPUT);
  // Initialize the digital output pins
  pinMode(ZC_OUT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  // Initialize the analog input interrupt
  analogReadResolution(12); // Set the resolution to 12 bits
  analogSetWidth(12); // Set the width to 12 bits
  analogSetCycles(8); // Set the number of cycles per sample
  analogSetSamples(1); // Set the number of samples in the range
  analogSetClockDiv(1); // Set the clock divider
  analogSetAttenuation(ADC_0db); // Set the attenuation
  analogSetPinAttenuation(AC_IN_PIN, ADC_0db); // Set the pin attenuation
  analogSetAlign(ADC_ALIGN_RIGHT); // Set the alignment
  adcAttachPin(AC_IN_PIN); // Attach the pin to ADC
  adcStart(AC_IN_PIN); // Start ADC
  adcAttachInterrupt(AC_IN_PIN, ac_isr, ADC_CHANGE); // Attach the interrupt
  // Initialize the hardware timer interrupt
  timer = timerBegin(0, 80, true); // Use timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, &timer_isr, true); // Attach the interrupt
  timerAlarmWrite(timer, 10, true); // Set the alarm value and repeat
  timerAlarmEnable(timer); // Enable the alarm
  // Initialize the global variables
  zc_time = 0;
  zc_period = AC_PERIOD;
  zc_freq = AC_FREQ;
  zc_phase = 0;
  zc_error = 0;
  zc_output = 0;
  kalman_x = 0;
  kalman_p = 1;
  kalman_k = 0;
}

// Loop function
void loop() {
  // Print the zero crossing frequency and phase
  Serial.print("Frequency: ");
  Serial.print(zc_freq);
  Serial.print(" Hz, Phase: ");
  Serial.print(zc_phase);
  Serial.print(" rad, Error: ");
  Serial.print(zc_error);
  Serial.print(" rad, Output: ");
  Serial.print(zc_output);
  Serial.print(" rad, Kalman: ");
  Serial.print(kalman_x);
  Serial.println(" rad");
  // Wait for 1 second
  delay(1000);
}
